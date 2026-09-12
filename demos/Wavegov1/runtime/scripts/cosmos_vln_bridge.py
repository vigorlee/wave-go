#!/usr/bin/python3
"""Use Cosmos3-Edge to issue allowlisted high-level navigation commands."""

from __future__ import annotations

import json
from io import BytesIO
import math
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from cosmos_vln_protocol import (
    ACTION_COMPLETE,
    ACTION_HOLD,
    ACTION_NAVIGATE,
    extract_route_command_json,
    load_route_catalog,
    prompt_route_catalog,
    validate_route_command,
)


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_COSMOS_ROOT = Path("/home/unitree/matrix_g1_lcm_demo")


def env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        parsed = float(value)
    except ValueError:
        return default
    return parsed if math.isfinite(parsed) else default


def prepare_reasoner_image(
    image_bytes: bytes, max_edge_px: int
) -> tuple[bytes, tuple[int, int], tuple[int, int]]:
    with Image.open(BytesIO(image_bytes)) as image:
        image.load()
        original_size = image.size
        image = image.convert("RGB")
        resampling = getattr(Image, "Resampling", Image)
        image.thumbnail((max_edge_px, max_edge_px), resampling.LANCZOS)
        inference_size = image.size
        output = BytesIO()
        image.save(output, format="JPEG", quality=90, optimize=True)
    return output.getvalue(), original_size, inference_size


class CosmosVlnBridge(Node):
    def __init__(self) -> None:
        super().__init__("cosmos_vln_bridge")
        self.camera_topic = os.environ.get(
            "COSMOS_VLN_CAMERA_TOPIC", "/image_raw/compressed"
        )
        self.instruction_topic = os.environ.get(
            "COSMOS_VLN_INSTRUCTION_TOPIC", "/cosmos_vln/instruction"
        )
        self.command_topic = os.environ.get(
            "COSMOS_VLN_COMMAND_TOPIC", "/cosmos_vln/route_command"
        )
        self.framework_dir = Path(
            os.environ.get(
                "COSMOS_VLN_FRAMEWORK",
                str(DEFAULT_COSMOS_ROOT / "packages/cosmos-framework"),
            )
        )
        self.checkpoint_dir = Path(
            os.environ.get(
                "COSMOS_VLN_CHECKPOINT", str(DEFAULT_COSMOS_ROOT / "Cosmos3-Edge")
            )
        )
        self.model_config = Path(
            os.environ.get(
                "COSMOS_VLN_MODEL_CONFIG",
                str(
                    self.framework_dir
                    / "cosmos_framework/inference/configs/model/Cosmos3-Edge.yaml"
                ),
            )
        )
        self.routes_file = Path(
            os.environ.get(
                "COSMOS_VLN_ROUTES_FILE",
                str(ROOT_DIR / "config/cosmos_vln_routes.json"),
            )
        )
        self.routes = load_route_catalog(self.routes_file)
        self.route_catalog_prompt = prompt_route_catalog(self.routes.values())
        # The mission supervisor may impose a route contract for a validated
        # reproduction. Expose that contract to the reasoner as context so a
        # shorter prefix (for example ``upper_landing``) is not selected for a
        # task that explicitly asks for the farthest endpoint.
        self.required_route_id = os.environ.get(
            "COSMOS_VLN_MISSION_REQUIRED_ROUTE_ID", ""
        ).strip()
        self.jobs_dir = Path(
            os.environ.get("COSMOS_VLN_JOBS_DIR", str(ROOT_DIR / ".run/cosmos_vln/jobs"))
        )
        self.max_new_tokens = int(
            min(1536, max(256, env_float("COSMOS_VLN_MAX_NEW_TOKENS", 768)))
        )
        self.max_image_age_sec = max(
            0.1, env_float("COSMOS_VLN_MAX_IMAGE_AGE_SEC", 3.0)
        )
        self.max_image_edge_px = int(
            min(1024.0, max(256.0, env_float("COSMOS_VLN_MAX_IMAGE_EDGE_PX", 640.0)))
        )
        self.inference_timeout_sec = max(
            30.0, env_float("COSMOS_VLN_INFERENCE_TIMEOUT_SEC", 900.0)
        )

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.command_publisher = self.create_publisher(String, self.command_topic, 10)
        self.prediction_publisher = self.create_publisher(
            String, "/cosmos_vln/future_prediction", 10
        )
        self.status_publisher = self.create_publisher(String, "/cosmos_vln/status", 10)
        self.create_subscription(
            CompressedImage, self.camera_topic, self.on_image, camera_qos
        )
        self.create_subscription(String, self.instruction_topic, self.on_instruction, 10)

        self.image_lock = threading.Lock()
        self.latest_image: tuple[bytes, float] | None = None
        self.worker_lock = threading.Lock()
        self.worker_active = False
        self.jobs_dir.mkdir(parents=True, exist_ok=True)
        self.publish_status(
            "ready",
            camera_topic=self.camera_topic,
            command_topic=self.command_topic,
            approved_routes=sorted(self.routes),
            checkpoint=str(self.checkpoint_dir),
        )
        self.get_logger().info(
            "Cosmos route-command bridge ready: "
            f"camera={self.camera_topic} command={self.command_topic} "
            f"routes={','.join(sorted(self.routes))}"
        )

    def publish_status(self, state: str, **details: Any) -> None:
        payload = {"state": state, "time": time.time(), **details}
        self.status_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

    def on_image(self, message: CompressedImage) -> None:
        with self.image_lock:
            self.latest_image = (bytes(message.data), time.monotonic())

    def on_instruction(self, message: String) -> None:
        instruction = message.data.strip()
        if not instruction:
            self.publish_status("rejected", reason="empty_instruction")
            return
        with self.worker_lock:
            if self.worker_active:
                self.publish_status("busy", instruction=instruction)
                return
            with self.image_lock:
                image = self.latest_image
            if image is None:
                self.publish_status("rejected", reason="no_camera_frame")
                return
            image_bytes, captured_at = image
            image_age = time.monotonic() - captured_at
            if image_age > self.max_image_age_sec:
                self.publish_status(
                    "rejected", reason="stale_camera_frame", image_age_sec=image_age
                )
                return
            self.worker_active = True
        threading.Thread(
            target=self.run_inference,
            args=(instruction, image_bytes),
            daemon=True,
        ).start()

    def planner_prompt(self, instruction: str) -> str:
        route_contract_hint = ""
        if self.required_route_id in self.routes:
            route_contract_hint = f"""

For this validated run, the mission contract requires the complete catalog route_id
`{self.required_route_id}`. The task asks for the longest/farthest route, so do not
select a shorter prefix such as `upper_landing` or `far_end_via_stairs`.
"""
        return f"""You are the high-level navigation command layer for a Unitree Go2-W robot.
User task: {instruction}

The robot already has Nav2 path planning, obstacle avoidance, and a trained DreamWaQ locomotion policy. You do not control motion. Never output velocity, steering, gait, local displacement, local subgoals, waypoints, trajectories, or map coordinates. Your only role is to select one approved route command and predict route-level future risk from the current first-person image.

Approved route catalog:
{self.route_catalog_prompt}
{route_contract_hint}

Choose action={ACTION_NAVIGATE} only when the task matches one approved route and the visible scene is consistent with safely starting it. Use exactly that catalog route_id. The selected route must satisfy the entire task, not merely its first phase. In particular, upper_landing is invalid when the task asks to descend, cross both staircases, continue beyond the landing, or reach the far end. Choose action={ACTION_HOLD} when the task is ambiguous, no approved route matches, the route appears blocked, or visual evidence is insufficient. Choose action={ACTION_COMPLETE} only when the image clearly proves the task is already complete. route_id must be null unless action={ACTION_NAVIGATE}.

Predict the expected observation after the selected route completes, list visible or credible route-level hazards, and state how the command advances the task. In the final answer output exactly one JSON object with no prose outside it and exactly these fields:
{{
  "action": "{ACTION_NAVIGATE}",
  "route_id": "approved_route_id",
  "future_prediction": {{
    "expected_observation": "what should be visible after the route completes",
    "hazards": ["route-level hazard"],
    "progress": "how the route advances the task"
  }},
  "confidence": 0.5,
  "reason": "brief evidence-based reason"
}}

Do not add any fields. Keep progress inside future_prediction; never place progress at the top level. Confidence must reflect visible evidence and be in [0, 1]."""

    def run_inference(self, instruction: str, image_bytes: bytes) -> None:
        job_id = time.strftime("%Y%m%dT%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
        job_dir = self.jobs_dir / job_id
        try:
            job_dir.mkdir(parents=True, exist_ok=False)
            image_bytes, original_size, inference_size = prepare_reasoner_image(
                image_bytes, self.max_image_edge_px
            )
            image_path = job_dir / "camera.jpg"
            input_path = job_dir / "request.json"
            output_dir = job_dir / "output"
            log_path = job_dir / "inference.log"
            image_path.write_bytes(image_bytes)
            input_path.write_text(
                json.dumps(
                    {
                        "model_mode": "reasoner",
                        "prompt": self.planner_prompt(instruction),
                        "vision_path": str(image_path),
                        "max_new_tokens": self.max_new_tokens,
                        "do_sample": False,
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
            self.publish_status(
                "inference_started",
                job_id=job_id,
                instruction=instruction,
                original_image_size=original_size,
                inference_image_size=inference_size,
            )
            command = [
                str(self.framework_dir / ".venv/bin/python"),
                "-m",
                "cosmos_framework.scripts.inference",
                "--parallelism-preset=latency",
                "-i",
                str(input_path),
                "-o",
                str(output_dir),
                "--checkpoint-path",
                str(self.checkpoint_dir),
                "--config-file",
                str(self.model_config),
                "--seed",
                "0",
                "--no-guardrails",
            ]
            model_env = os.environ.copy()
            model_env["LD_LIBRARY_PATH"] = ""
            model_env["PYTHONPATH"] = ""
            model_env["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
            model_env["PYTORCH_ALLOC_CONF"] = "expandable_segments:True"
            model_env["COSMOS_TRAINING"] = "0"
            started_at = time.monotonic()
            with log_path.open("w") as log_file:
                result = subprocess.run(
                    command,
                    cwd=self.framework_dir,
                    env=model_env,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    timeout=self.inference_timeout_sec,
                    check=False,
                )
            elapsed = time.monotonic() - started_at
            if result.returncode != 0:
                raise RuntimeError(
                    f"Cosmos inference failed with code {result.returncode}; see {log_path}"
                )
            reasoner_path = output_dir / input_path.stem / "reasoner_text.txt"
            if not reasoner_path.is_file():
                raise RuntimeError(f"missing reasoner output: {reasoner_path}")
            raw_text = reasoner_path.read_text()
            response = validate_route_command(
                extract_route_command_json(raw_text), self.routes.keys()
            )
            self.publish_command(
                instruction=instruction,
                response=response,
                raw_text=raw_text,
                job_id=job_id,
                elapsed=elapsed,
            )
        except subprocess.TimeoutExpired:
            self.publish_status("failed", job_id=job_id, reason="inference_timeout")
            self.get_logger().error(f"Cosmos inference timed out for job {job_id}")
        except Exception as exc:
            self.publish_status("failed", job_id=job_id, reason=str(exc))
            self.get_logger().error(f"Cosmos route-command job {job_id} failed: {exc}")
        finally:
            with self.worker_lock:
                self.worker_active = False

    def publish_command(
        self,
        *,
        instruction: str,
        response: dict[str, Any],
        raw_text: str,
        job_id: str,
        elapsed: float,
    ) -> None:
        payload = {
            "job_id": job_id,
            "instruction": instruction,
            **response,
            "inference_sec": elapsed,
            "raw_reasoner_output": raw_text[-8000:],
        }
        message = String(data=json.dumps(payload, ensure_ascii=False))
        self.prediction_publisher.publish(message)
        self.command_publisher.publish(message)
        self.publish_status(
            "command_ready",
            job_id=job_id,
            action=response["action"],
            route_id=response["route_id"],
            confidence=response["confidence"],
            navigation_requested=False,
        )


def main() -> None:
    rclpy.init()
    node = CosmosVlnBridge()
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
