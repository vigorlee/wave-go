#!/usr/bin/python3
"""Publish RViz markers and an annotated NWM-Cosmos3Edge camera view."""

from __future__ import annotations

from copy import deepcopy
import json
import math
import os
from io import BytesIO
from pathlib import Path
from typing import Any

from geometry_msgs.msg import PoseStamped
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


ROOT_DIR = Path(__file__).resolve().parents[1]
FONT_PATHS = (
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
    Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
)


def load_font(size: int) -> ImageFont.ImageFont:
    for path in FONT_PATHS:
        if path.is_file():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def wrap_text(
    draw: ImageDraw.ImageDraw,
    text: str,
    font: ImageFont.ImageFont,
    max_width: int,
    max_lines: int,
) -> list[str]:
    lines: list[str] = []
    for paragraph in str(text).replace("\t", " ").splitlines() or [""]:
        current = ""
        for character in paragraph:
            candidate = current + character
            if current and draw.textlength(candidate, font=font) > max_width:
                lines.append(current.rstrip())
                current = character.lstrip()
            else:
                current = candidate
        if current or not lines:
            lines.append(current.rstrip())
    if len(lines) > max_lines:
        lines = lines[:max_lines]
        lines[-1] = lines[-1].rstrip() + "..."
    return lines


class CosmosVlnVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("cosmos_vln_visualizer")
        self.camera_topic = os.environ.get(
            "COSMOS_VLN_CAMERA_TOPIC", "/image_raw/compressed"
        )
        self.output_path = Path(
            os.environ.get(
                "COSMOS_VLN_VISUALIZATION_IMAGE",
                str(ROOT_DIR / ".run/cosmos_vln/latest_visualization.jpg"),
            )
        )
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_publisher = self.create_publisher(
            Image, "/cosmos_vln/annotated_image", 1
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/cosmos_vln/markers", 10
        )
        self.create_subscription(
            CompressedImage, self.camera_topic, self.on_image, camera_qos
        )
        self.create_subscription(
            PoseStamped, "/cosmos_vln/target_pose", self.on_target, 10
        )
        self.create_subscription(
            String, "/cosmos_vln/future_prediction", self.on_prediction, 10
        )
        self.create_subscription(
            String, "/cosmos_vln/mission_status", self.on_mission_status, 10
        )

        self.latest_image: bytes | None = None
        self.latest_frame_id = "camera"
        self.target_pose: PoseStamped | None = None
        self.prediction: dict[str, Any] = {}
        self.get_logger().info(
            "NWM-Cosmos3Edge visualization ready: "
            "image=/cosmos_vln/annotated_image markers=/cosmos_vln/markers"
        )

    def on_image(self, message: CompressedImage) -> None:
        self.latest_image = bytes(message.data)
        self.latest_frame_id = message.header.frame_id or "camera"
        self.publish_annotated_image()

    def on_target(self, message: PoseStamped) -> None:
        self.target_pose = message
        self.publish_markers()
        self.publish_annotated_image()

    def on_prediction(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Ignoring invalid prediction JSON: {exc}")
            return
        if not isinstance(payload, dict):
            self.get_logger().warning("Ignoring prediction JSON that is not an object")
            return
        self.prediction = payload
        self.target_pose = None
        self.clear_markers()
        self.publish_annotated_image()

    def on_mission_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        state = str(payload.get("state", ""))
        self.prediction["mission_state"] = state
        if state in {"blocked", "failed", "canceled"}:
            self.target_pose = None
            self.clear_markers()
        if state != "goal_requested":
            self.publish_annotated_image()
            return
        target = payload.get("target")
        if not isinstance(target, dict):
            return
        self.prediction.update(
            {
                "instruction": payload.get("mission", ""),
                "action": "navigate_to_route",
                "route_id": payload.get("route_id"),
                "stage_id": payload.get("stage_id"),
                "stage_index": payload.get("stage_index"),
                "stage_count": payload.get("stage_count"),
                "map_target": target,
                "confidence": payload.get("confidence", 0.0),
                "future_prediction": payload.get("future_prediction", {}),
                "mission_state": state,
            }
        )
        self.publish_annotated_image()

    def clear_markers(self) -> None:
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_publisher.publish(MarkerArray(markers=[marker]))

    def publish_annotated_image(self) -> None:
        if self.latest_image is None:
            return
        try:
            with PILImage.open(BytesIO(self.latest_image)) as source:
                source.load()
                camera = source.convert("RGB")
        except Exception as exc:
            self.get_logger().warning(f"Cannot decode camera image: {exc}")
            return

        if camera.width > 1280:
            resampling = getattr(PILImage, "Resampling", PILImage)
            height = max(1, round(camera.height * 1280 / camera.width))
            camera = camera.resize((1280, height), resampling.LANCZOS)

        marker = self.prediction.get("marker")
        if isinstance(marker, dict):
            corners = marker.get("corners")
            if isinstance(corners, list) and len(corners) == 4:
                try:
                    points = [(float(point[0]), float(point[1])) for point in corners]
                except (TypeError, ValueError, IndexError):
                    points = []
                if len(points) == 4:
                    overlay = ImageDraw.Draw(camera)
                    overlay.line(points + [points[0]], fill=(72, 235, 143), width=4)
                    overlay.text(
                        (points[0][0], max(0.0, points[0][1] - 24.0)),
                        f"QR/ArUco {marker.get('id', '?')}",
                        font=load_font(18),
                        fill=(72, 235, 143),
                    )

        title_font = load_font(max(20, camera.width // 42))
        body_font = load_font(max(15, camera.width // 64))
        sample = PILImage.new("RGB", (camera.width, 32), "black")
        measure = ImageDraw.Draw(sample)
        line_height = max(23, int(getattr(body_font, "size", 16) * 1.45))

        confidence = min(1.0, max(0.0, safe_float(self.prediction.get("confidence"))))
        action = str(self.prediction.get("action", ""))
        map_target = self.prediction.get("map_target", {})
        if not isinstance(map_target, dict):
            map_target = {}
        future = self.prediction.get("future_prediction", {})
        if not isinstance(future, dict):
            future = {}
        hazards = future.get("hazards", [])
        if isinstance(hazards, list):
            hazards_text = ", ".join(str(item) for item in hazards)
        else:
            hazards_text = str(hazards)

        mapless = action == "mapless_visual_search"
        resolved_target = (
            "Live QR/ArUco target; no map coordinate"
            if mapless
            else "x={:.2f} m, y={:.2f} m, yaw={:+.2f} rad".format(
                safe_float(map_target.get("x")),
                safe_float(map_target.get("y")),
                safe_float(map_target.get("yaw_rad")),
            )
        )
        fields = [
            ("Instruction", str(self.prediction.get("instruction", "Waiting for command"))),
            ("NWM command", action or "waiting"),
            ("Route ID", str(self.prediction.get("route_id") or "none")),
            (
                "Route stage",
                "{} ({}/{})".format(
                    self.prediction.get("stage_id") or "pending",
                    int(safe_float(self.prediction.get("stage_index"))) + 1,
                    max(1, int(safe_float(self.prediction.get("stage_count"), 1))),
                ),
            ),
            (
                "Resolved target",
                resolved_target,
            ),
            ("Expected", str(future.get("expected_observation", "Waiting"))),
            ("Hazards", hazards_text or "None reported"),
            ("Progress", str(future.get("progress", "Waiting"))),
            ("Mission state", str(self.prediction.get("mission_state", "waiting"))),
        ]
        label_width = max(
            measure.textlength(f"{label}:", font=body_font)
            for label, _value in fields
        )
        value_x = max(166, int(math.ceil(24 + label_width + 18)))
        value_width = max(120, camera.width - value_x - 24)
        wrapped_fields: list[tuple[str, list[str]]] = []
        total_lines = 0
        for label, value in fields:
            lines = wrap_text(measure, value, body_font, value_width, 3)
            wrapped_fields.append((label, lines))
            total_lines += max(1, len(lines))

        panel_height = 78 + total_lines * line_height + 24
        canvas = PILImage.new(
            "RGB", (camera.width, camera.height + panel_height), (22, 26, 30)
        )
        canvas.paste(camera, (0, 0))
        draw = ImageDraw.Draw(canvas)
        panel_y = camera.height
        draw.rectangle((0, panel_y, camera.width, panel_y + 5), fill=(77, 207, 155))
        draw.text(
            (24, panel_y + 18),
            "NWM-Cosmos3Edge",
            font=title_font,
            fill=(232, 238, 242),
        )
        confidence_color = (
            (242, 92, 92)
            if action == "hold_position"
            else (76, 210, 139)
            if confidence >= 0.65
            else (245, 184, 65)
            if confidence >= 0.35
            else (242, 92, 92)
        )
        status = {
            "navigate_to_route": "ROUTE",
            "semantic_search": "SEARCH",
            "mapless_visual_search": "MAPLESS SEARCH",
            "hold_position": "HOLD",
            "mission_complete": "COMPLETE",
        }.get(action, "WAITING")
        confidence_text = f"{status}  confidence {confidence:.2f}"
        confidence_width = draw.textlength(confidence_text, font=body_font)
        draw.text(
            (camera.width - confidence_width - 24, panel_y + 25),
            confidence_text,
            font=body_font,
            fill=confidence_color,
        )

        y = panel_y + 66
        for label, lines in wrapped_fields:
            draw.text((24, y), f"{label}:", font=body_font, fill=(115, 205, 255))
            for index, line in enumerate(lines):
                draw.text(
                    (value_x, y + index * line_height),
                    line,
                    font=body_font,
                    fill=(215, 222, 228),
                )
            y += max(1, len(lines)) * line_height

        output = Image()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.latest_frame_id
        output.height = canvas.height
        output.width = canvas.width
        output.encoding = "rgb8"
        output.is_bigendian = 0
        output.step = canvas.width * 3
        output.data = canvas.tobytes()
        self.image_publisher.publish(output)

        temporary_path = self.output_path.with_name(
            f"{self.output_path.stem}.tmp{self.output_path.suffix}"
        )
        canvas.save(temporary_path, format="JPEG", quality=90)
        os.replace(temporary_path, self.output_path)

    def publish_markers(self) -> None:
        if self.target_pose is None:
            return
        confidence = min(1.0, max(0.0, safe_float(self.prediction.get("confidence"))))
        if confidence >= 0.65:
            red, green, blue = 0.25, 0.9, 0.5
        elif confidence >= 0.35:
            red, green, blue = 1.0, 0.68, 0.15
        else:
            red, green, blue = 0.95, 0.25, 0.25

        header = self.target_pose.header
        arrow = Marker()
        arrow.header = header
        arrow.ns = "cosmos_vln"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = deepcopy(self.target_pose.pose)
        arrow.pose.position.z = 0.12
        arrow.scale.x = 0.9
        arrow.scale.y = 0.18
        arrow.scale.z = 0.18
        arrow.color.r = red
        arrow.color.g = green
        arrow.color.b = blue
        arrow.color.a = 1.0

        target = Marker()
        target.header = header
        target.ns = "cosmos_vln"
        target.id = 1
        target.type = Marker.CYLINDER
        target.action = Marker.ADD
        target.pose.position = deepcopy(self.target_pose.pose.position)
        target.pose.position.z = 0.04
        target.pose.orientation.w = 1.0
        target.scale.x = 0.34
        target.scale.y = 0.34
        target.scale.z = 0.08
        target.color.r = red
        target.color.g = green
        target.color.b = blue
        target.color.a = 0.75

        text = Marker()
        text.header = header
        text.ns = "cosmos_vln"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = self.target_pose.pose.position.x
        text.pose.position.y = self.target_pose.pose.position.y + 0.75
        text.pose.position.z = 0.85
        text.pose.orientation.w = 1.0
        text.scale.z = 0.24
        text.color.r = 0.95
        text.color.g = 0.97
        text.color.b = 1.0
        text.color.a = 1.0
        route_id = str(self.prediction.get("route_id") or "pending")
        mission_state = str(self.prediction.get("mission_state") or "command-ready")
        text.text = (
            f"NWM-Cosmos3Edge route\n{route_id}\nconf={confidence:.2f}\n{mission_state}"
        )

        self.marker_publisher.publish(MarkerArray(markers=[target, arrow, text]))


def main() -> None:
    rclpy.init()
    node = CosmosVlnVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
