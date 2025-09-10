#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from osm2qr.gui import GUI


class args:
    def __init__(self):
        self.coords_file = ""
        self.save_path = None


class GenQR(Node):
    def __init__(self):
        super().__init__("gen_qr")
        self.declare_parameter("coords_file", "")
        self.coords_file = (
            self.get_parameter("coords_file").get_parameter_value().string_value
        )
        self.args = args()
        self.args.coords_file = self.coords_file

        self.window = GUI(self.args)
        self.window.create_window()
        self.window.run()


def main():
    rclpy.init()
    qr2geo = GenQR()
    rclpy.spin(qr2geo)
    qr2geo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
