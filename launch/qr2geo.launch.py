#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="osm2qr",
                executable="qr2geo",
                name="qr2geo",
                respawn=True,
                respawn_delay=2,
                parameters=[
                    {
                        "camera_topic": "/luxonis/oak/left/image_raw/compressed",
                        "talk_topic": "/speak_cs/info",
                        "read_interval": 10.0,
                        "plan_wait": 5.0,
                    }
                ],
            )
        ]
    )
