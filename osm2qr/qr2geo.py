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
        self.declare_parameter("read_interval", 10.0)
        self.declare_parameter("plan_wait", 5.0)

        self.camera_topic = (
            self.get_parameter("camera_topic").get_parameter_value().string_value
        )
        self.talk_topic = (
            self.get_parameter("talk_topic").get_parameter_value().string_value
        )
        self.read_interval = (
            self.get_parameter("read_interval").get_parameter_value().double_value
        )
        self.plan_wait = (
            self.get_parameter("plan_wait").get_parameter_value().double_value
        )

        self.sub = self.create_subscription(
            CompressedImage, self.camera_topic, self.read_qr_, 10
        )
        self.last_geo = None

        if self.talk_topic != "":
            self.pub = self.create_publisher(String, self.talk_topic, 10)
            self.talk_detect = String()
            self.talk_finish = String()
            if "cs" in self.talk_topic:
                self.talk_detect.data = "QR kód detekován"
                self.talk_finish.data = "Cíl dosažen"
            else:
                self.talk_detect.data = "Geo QR code detected"
                self.talk_finish.data = "Goal reached"

        self.action_client = ActionClient(
            self, FollowGPSWaypoints, "follow_gps_waypoints"
        )

    def read_qr_(self, msg):
        """Read QR code from camera topic and publish geocode"""
        if (self.last_geo is not None) and (
            (self.get_clock().now() - self.last_geo).nanoseconds * 1e-9
        ) < self.read_interval:
            return

        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        codes = pyzbar.decode(image)

        if codes:
            for code in codes:
                if code.type == "QRCODE":
                    data = code.data.decode("utf-8")
                    if data[:3] == "geo":
                        if self.talk_topic != "":
                            self.pub.publish(self.talk_detect)

                        data = list(map(float, data[4:].strip().split(",")))
                        self.last_geo = self.get_clock().now()
                        self.get_logger().info(f"Geo QR code detected: {data}")

                        geo_pose = GeoPose()
                        geo_pose.position.latitude = data[0]
                        geo_pose.position.longitude = data[1]
                        self.waypoint_msg_ = FollowGPSWaypoints.Goal()
                        self.waypoint_msg_.gps_poses = [geo_pose]

                        self.get_clock().sleep_for(
                            rclpy.duration.Duration(seconds=self.plan_wait)
                        )
                        self._send_goal()

    def _send_goal(self):
        self.get_logger().info("Sending goal to action server")
        send_goal_future = self.action_client.send_goal_async(
            self.waypoint_msg_, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            return

        self.get_logger().info("Goal accepted by server")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        pass

    def _result_callback(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(f"Missed waypoints: {result.missed_waypoints}")
            self._send_goal()
        else:
            self.get_logger().info("Successfully navigated all waypoints")
            if self.talk_topic != "":
                self.pub.publish(self.talk_finish)


def main():
    rclpy.init()
    qr2geo = QR2Geo()
    rclpy.spin(qr2geo)
    qr2geo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
