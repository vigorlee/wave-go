#!/usr/bin/python3
"""Display the live annotated NWM-Cosmos3Edge camera stream."""

from __future__ import annotations

import sys

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CosmosImageWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("NWM-Cosmos3Edge Camera Prediction")
        self.setWindowFlag(Qt.WindowStaysOnTopHint, True)
        self.resize(960, 780)
        self.move(1280, 60)
        self.setMinimumSize(640, 520)

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet("background-color: #161a1e;")
        self.setCentralWidget(self.image_label)

        self.node = Node("cosmos_vln_image_view")
        self.node.create_subscription(
            Image, "/cosmos_vln/annotated_image", self.on_image, 10
        )
        self.pixmap: QPixmap | None = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(10)

    def spin_ros(self) -> None:
        if not rclpy.ok():
            self.timer.stop()
            self.close()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except RuntimeError:
            self.timer.stop()
            self.close()

    def on_image(self, message: Image) -> None:
        if message.encoding not in {"rgb8", "bgr8"}:
            self.node.get_logger().warning(
                f"Unsupported annotated image encoding: {message.encoding}"
            )
            return
        image_format = (
            QImage.Format_RGB888
            if message.encoding == "rgb8"
            else QImage.Format_BGR888
        )
        image = QImage(
            bytes(message.data),
            message.width,
            message.height,
            message.step,
            image_format,
        ).copy()
        self.pixmap = QPixmap.fromImage(image)
        self.update_pixmap()

    def update_pixmap(self) -> None:
        if self.pixmap is None:
            return
        self.image_label.setPixmap(
            self.pixmap.scaled(
                self.image_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def resizeEvent(self, event: object) -> None:
        super().resizeEvent(event)
        self.update_pixmap()

    def closeEvent(self, event: object) -> None:
        self.timer.stop()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        super().closeEvent(event)


def main() -> None:
    rclpy.init()
    application = QApplication(sys.argv)
    window = CosmosImageWindow()
    window.show()
    window.raise_()
    window.activateWindow()
    exit_code = application.exec_()
    if rclpy.ok():
        window.node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
