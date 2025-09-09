#!/usr/bin/env python

import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geographic_msgs.msg import GeoPose
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
                        data = list(map(float, data[4:].split(",").strip()))
                        self.last_geo = self.get_clock().now()
                        self.get_logger().info(f"Geo QR code detected: {data}")

                        geo_pose = GeoPose()
                        geo_pose.position.latitude = data[0]
                        geo_pose.position.longitude = data[1]
                        geo_pose.position.altitude = data[2] if data[2] else 0
                        waypoint_msg = FollowGPSWaypoints.Goal()
                        waypoint_msg.gps_poses = [geo_pose]

                        send_goal_future = self._action_client.send_goal_async(
                            waypoint_msg, feedback_callback=self.feedback_callback
                        )
                        send_goal_future.add_done_callback(self.goal_response_callback)

                        msg = String()
                        msg.data = "Geo QR code detected"
                        self.pub.publish(msg)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            return

        self.get_logger().info("Goal accepted by server")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # self.send_data_url("update")
        pass

    def result_callback(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(f"Missed waypoints: {result.missed_waypoints}")
        else:
            self.get_logger().info("Successfully navigated all waypoints")
        rclpy.shutdown()


def main():
    rclpy.init()
    qr2geo = QR2Geo()
    rclpy.spin(qr2geo)
    qr2geo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
