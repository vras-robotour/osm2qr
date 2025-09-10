#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    osm2qr_share_dir = get_package_share_directory("osm2qr")
    return LaunchDescription(
        [
            Node(
                package="osm2qr",
                executable="gen_qr",
                name="gen_qr",
                respawn=False,
                parameters=[
                    {
                        "coords_file": osm2qr_share_dir + "/data/kn.gpx",
                    }
                ],
            )
        ]
    )
