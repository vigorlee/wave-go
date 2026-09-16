#!/usr/bin/python3
"""Record MATRiX's real UE RGB sensor stream as an MP4.

The recorder subscribes directly to ``/image_raw/compressed``.  It does not
capture a desktop window and it does not synthesize/overlay frames, so the
recorded scene is the same UE instance that publishes the LiDAR used by Nav2.
"""

from __future__ import annotations

import argparse
import csv
import queue
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class Recorder(Node):
    def __init__(self, topic: str, output: Path, fps: float) -> None:
        super().__init__("nezha2_ue_camera_recorder")
        self.output = output
        self.fps = fps
        self.writer: cv2.VideoWriter | None = None
        self.frames = 0
        self.received = 0
        self.dropped = 0
        self.timestamps: list[tuple[int, float]] = []
        self.next_frame_time = 0.0
        self._queue: queue.Queue = queue.Queue(maxsize=max(32, int(fps * 10)))
        self._writer_error: Exception | None = None
        self._closed = False
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="ue-camera-writer", daemon=True
        )
        self._writer_thread.start()
        self.subscription = self.create_subscription(
            CompressedImage, topic, self.callback, qos_profile_sensor_data
        )

    def callback(self, message: CompressedImage) -> None:
        now = time.monotonic()
        self.received += 1
        if now < self.next_frame_time:
            return
        self.next_frame_time = now + (1.0 / self.fps)
        try:
            # Keep the ROS callback short. JPEG decoding and disk I/O happen
            # on the writer thread so a 10-15 Hz sensor stream is not sampled
            # down by the subscription callback itself.
            self._queue.put_nowait((now, bytes(message.data)))
        except queue.Full:
            self.dropped += 1

    def _writer_loop(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                captured_at, encoded = item
                frame = cv2.imdecode(
                    np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if frame is None:
                    continue
                if self.writer is None:
                    self.output.parent.mkdir(parents=True, exist_ok=True)
                    height, width = frame.shape[:2]
                    self.writer = cv2.VideoWriter(
                        str(self.output),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        self.fps,
                        (width, height),
                    )
                    if not self.writer.isOpened():
                        raise RuntimeError(f"cannot create MP4: {self.output}")
                self.writer.write(frame)
                self.frames += 1
                self.timestamps.append((self.frames, captured_at))
            except Exception as exc:  # propagate through close() after drain
                self._writer_error = exc
                return
            finally:
                self._queue.task_done()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        # Drain all accepted frames before asking the writer thread to exit.
        self._queue.join()
        self._queue.put(None)
        self._writer_thread.join()
        if self.writer is not None:
            self.writer.release()
        with self.output.with_suffix(".timestamps.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("frame", "monotonic_seconds"))
            writer.writerows(self.timestamps)
        if self._writer_error is not None:
            raise self._writer_error


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/image_raw/compressed")
    parser.add_argument("--duration", type=float, default=75.0)
    parser.add_argument("--fps", type=float, default=5.0)
    args = parser.parse_args()
    if args.duration <= 0 or args.fps <= 0:
        raise SystemExit("--duration and --fps must be positive")

    rclpy.init()
    recorder = Recorder(args.topic, args.output, args.fps)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                rclpy.spin_once(recorder, timeout_sec=0.25)
            except (KeyboardInterrupt, ExternalShutdownException, RCLError):
                # The navigation stack may tear down the shared ROS context
                # before the recorder receives its own stop signal.  The
                # frames already written are still a valid, finalized video.
                break
        if recorder.frames == 0:
            raise TimeoutError(f"no frames received from {args.topic}")
        print(
            f"recorded UE RGB frames={recorder.frames} received={recorder.received} "
            f"dropped={recorder.dropped} output={args.output}",
            flush=True,
        )
    finally:
        recorder.close()
        recorder.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # A shared Zenoh/ROS context can already be shut down by the
            # stack supervisor; do not turn a closed, playable MP4 into a
            # recorder failure solely because of that teardown race.
            pass


if __name__ == "__main__":
    main()
