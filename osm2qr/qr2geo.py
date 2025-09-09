#!/usr/bin/env python

import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from nav2_msgs.action import FollowGPSWaypoints


class QR2Geo(Node):
    def __init__(self):
        super().__init__("qr2geo")
        self.declare_parameter("camera_topic", "camera")
        self.declare_parameter("talk_topic", "")
        self.camera_topic = (
            self.get_parameter("camera_topic").get_parameter_value().string_value
        )
        self.talk_topic = (
            self.get_parameter("talk_topic").get_parameter_value().string_value
        )

        self.sub = self.create_subscription(
            CompressedImage, self.camera_topic, self.read_qr_, 10
        )
        self.last_geo = None

        if self.talk_topic != "":
            self.pub = self.create_publisher(String, self.talk_topic, 10)

        self.action_client = ActionClient(
            self, FollowGPSWaypoints, "follow_gps_waypoints"
        )

    def read_qr_(self, msg):
        """Read QR code from camera topic and publish geocode"""
        if (self.last_geo is not None) and (
            (self.get_clock().now() - self.last_geo).nanoseconds * 1e-9
        ) < 5:
            return

        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        codes = pyzbar.decode(image)

        if codes:
            for code in codes:
                if code.type == "QRCODE":
                    data = code.data.decode("utf-8")
                    if data[:3] == "geo":
                        data = data[4:].split(",").strip()
                        self.get_logger().info(data)
                        self.last_geo = self.get_clock().now()
                        self.get_logger().info("Geo QR code detected")
                        waypoint_msg = FollowGPSWaypoints.Goal()
                        waypoint_msg.gps_poses = ...

                        msg = String()
                        msg.data = "Geo QR code detected"
                        self.pub.publish(msg)


def main():
    rclpy.init()
    qr2geo = QR2Geo()
    rclpy.spin(qr2geo)
    qr2geo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
