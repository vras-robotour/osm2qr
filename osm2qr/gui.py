#!/usr/bin/env python

import os
import argparse

import utm
import pyqrcode
import numpy as np
import tkinter as tk
from PIL import ImageTk
import matplotlib.pyplot as plt
from gpxpy import parse as gpxparse

from osm2qr.background_map import get_background_image


OSM_RECTANGLE_MARGIN = 50  # meters, margin around the map
RESERVE = 10  # meters, reserve around the waypoints


class GUI:
    def __init__(self, args):
        # WINDOW
        self.window_width = 1200  # px
        self.window_height = self.window_width * 3 // 4
        self.window_title = "RoboTour GUI"
        self.show_qr_key = "v"
        self.save_qr_key = "s"
        self.window_destroy_key = "q"
        self.wait_restore = 3000  # ms

        # COORDS DATA
        self.coords_file = args.coords_file
        self.parse_coords_data()

        # DOT
        self.dot_color = "red"
        self.dot_size = 12  # px
        self.is_dot = False

        # BOTTOM FRAME
        self.bottom_frame_text = "Latitude: {0}, Longitude: {1}"
        self.bottom_frame_bg = "white"
        self.font_type = "Helvetica"
        self.font_size = self.window_width // 50
        self.text_font = (self.font_type, self.font_size)
        self.text_font_italic = (self.font_type, self.font_size, "italic")
        self.instructions = "Click on the map to get latitude and longitude"

        # LATLON
        self.lat = None
        self.lon = None
        self.counter = 0

        # QR CODE
        self.save_path = args.save_path
        self.qr_scale = 8
        self.saved = "QR code no. {0} was saved."

    def parse_coords_data(self):
        def get_margin(self):
            """
            Get the margin for the map.

            Returns:
            --------
            x_margin : float
                Margin in the x direction.
            y_margin : float
                Margin in the y direction.
            """
            y_margin = (self.max_lat - self.min_lat) * 0.1
            x_margin = (self.max_lon - self.min_lon) * 0.1

            if y_margin < x_margin:
                y_margin = x_margin
            else:
                x_margin = y_margin

            return x_margin, y_margin

        def waypoints_to_utm(waypoints):
            """
            Convert waypoints obtained from .gpx file from lat/lon (WGS84) to UTM.

            Parameters:
            -----------
            waypoints : numpy.ndarray
                Array of waypoints in lat/lon format.

            Returns:
            --------
            utm_coords : numpy.ndarray
                Array of waypoints in UTM format.
            zone_number : int
                UTM zone number.
            zone_letter : str
                UTM zone letter.
            """
            utm_arr = utm.from_latlon(waypoints[:, 0], waypoints[:, 1])
            utm_coords = np.concatenate(
                (utm_arr[0].reshape(-1, 1), utm_arr[1].reshape(-1, 1)), axis=1
            )
            zone_number = utm_arr[2]
            zone_letter = utm_arr[3]
            return utm_coords, zone_number, zone_letter

        gpx_f = open(self.coords_file, "r")
        gpx_object = gpxparse(gpx_f)
        self.waypoints = np.array(
            [[point.latitude, point.longitude] for point in gpx_object.waypoints]
        )
        self.waypoints, self.zone_number, self.zone_letter = waypoints_to_utm(
            self.waypoints
        )

        self.max_x = np.max(self.waypoints[:, 0]) + RESERVE
        self.min_x = np.min(self.waypoints[:, 0]) - RESERVE
        self.max_y = np.max(self.waypoints[:, 1]) + RESERVE
        self.min_y = np.min(self.waypoints[:, 1]) - RESERVE

        self.max_lat, self.max_lon = utm.to_latlon(
            self.max_x + OSM_RECTANGLE_MARGIN,
            self.max_y + OSM_RECTANGLE_MARGIN,
            self.zone_number,
            self.zone_letter,
        )
        self.min_lat, self.min_lon = utm.to_latlon(
            self.min_x - OSM_RECTANGLE_MARGIN,
            self.min_y - OSM_RECTANGLE_MARGIN,
            self.zone_number,
            self.zone_letter,
        )

        self.margin_lat, self.margin_lon = get_margin(self)

    def create_window(self):
        """Create gui window"""
        self.root = tk.Tk()
        self.root.title(self.window_title)

        self.canvas = tk.Canvas(
            self.root,
            width=self.window_width,
            height=self.window_height,
            highlightthickness=0,
        )
        self.canvas.pack()

        self.background_image = self.get_background_map()
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.background_image)

        self.bottom_frame = tk.Frame(self.root, bg=self.bottom_frame_bg)
        self.bottom_frame.pack(side=tk.BOTTOM, fill=tk.X)

        self.coords_label = tk.Label(
            self.bottom_frame, text="", bg=self.bottom_frame_bg, font=self.text_font
        )
        self.coords_label.pack(pady=5)

        self.coords_label.config(text=self.instructions, font=self.text_font_italic)

        self.root.bind("<Button-1>", self.cursor_click)
        self.root.bind("<Key>", self.key_press)

    def key_press(self, event):
        """Handle various key press events"""
        if event.char.lower() == self.window_destroy_key.lower():
            self.root.destroy()
            print("RoboTour GUI closed.")
            exit(0)
        elif event.char.lower() == self.save_qr_key.lower():
            if self.save_path is None:
                pass
            elif self.counter > 0:
                if os.path.exists(self.save_path) is False:
                    os.makedirs(self.save_path)

                qr = self.create_qr((self.lat, self.lon))
                qr.png(self.save_path + f"qr{self.counter}.png", scale=self.qr_scale)
                self.coords_label.config(
                    text=self.saved.format(self.counter), font=self.text_font
                )
                print(self.saved.format(self.counter))
            else:
                print("No coordinates selected yet!")
        elif event.char.lower() == self.show_qr_key.lower():
            qr = self.create_qr((self.lat, self.lon))
            lines = qr.text().strip().split("\n")
            img = np.array(
                [[int(char) for char in line] for line in lines], dtype=np.uint8
            )
            plt.figure(figsize=(5, 5))
            plt.imshow(1 - img, cmap="gray")
            plt.axis("off")
            plt.show()
        elif event.char.lower() == "r":
            self.restore_window()

    def cursor_click(self, event):
        """Get cursor click coordinates and draw red dot"""
        self.counter += 1

        # Remove old dot if drawn and draw new one
        if self.is_dot:
            self.restore_window()
        x, y = self.transform_window_coords(event.x, event.y)
        self.lat, self.lon = self.coords2latlon(x, y)
        print(
            f"({self.counter}) Selected place has latitude {self.lat} and longitude {self.lon}."
        )
        self.coords_label.config(
            text=self.bottom_frame_text.format(self.lat, self.lon), font=self.text_font
        )
        self.draw_dot(x, y)

    def transform_window_coords(self, x, y):
        """Transform (0,0) from top-left to bottom-left corner"""
        return x, self.window_height - y

    def draw_dot(self, x, y):
        """Create red dot at position (x,y)"""
        x, y = self.transform_window_coords(x, y)
        self.canvas.create_oval(
            x - self.dot_size / 2,
            y - self.dot_size / 2,
            x + self.dot_size / 2,
            y + self.dot_size / 2,
            fill=self.dot_color,
            outline=self.dot_color,
            tags="dot",
        )
        self.is_dot = True

    def restore_window(self):
        """Remove red dot and bottom text after N seconds"""
        self.canvas.delete("dot")
        self.coords_label.config(text="")
        self.coords_label.config(text=self.instructions, font=self.text_font_italic)

    def run(self):
        """Run main loop"""
        self.root.mainloop()

    def get_background_map(self):
        """Get background map"""
        img = get_background_image(
            self.min_lon,
            self.max_lon,
            self.min_lat,
            self.max_lat,
            self.margin_lat,
            self.margin_lon,
        )
        assert img.size[0] / img.size[1] - 4 / 3 < 0.1, "Image is not 4:3!"
        img = img.resize((self.window_width, self.window_height))
        background_image = ImageTk.PhotoImage(img)
        return background_image

    def coords2latlon(self, x, y):
        """Convert image coordinates to latitude and longitude"""
        min_utm = utm.from_latlon(
            self.min_lat - self.margin_lat,
            self.min_lon - self.margin_lon,
        )
        max_utm = utm.from_latlon(
            self.max_lat + self.margin_lat,
            self.max_lon + self.margin_lon,
        )

        min_utm_easting, min_utm_northing = min_utm[0], min_utm[1]
        max_utm_easting, max_utm_northing = max_utm[0], max_utm[1]
        assert (
            min_utm[2] == max_utm[2] and min_utm[3] == max_utm[3]
        ), "Different UTM zones!"
        zone_number, zone_letter = min_utm[2], min_utm[3]
        step_easting = (max_utm_easting - min_utm_easting) / self.window_width
        step_northing = (max_utm_northing - min_utm_northing) / self.window_height
        easting = min_utm_easting + x * step_easting
        northing = min_utm_northing + y * step_northing

        lat, lon = utm.to_latlon(easting, northing, zone_number, zone_letter)
        return lat, lon

    def create_qr(self, geo):
        """Create and save QR code with geo coordinates"""
        geo_str = f"geo:{geo[0]},{geo[1]}"
        qr = pyqrcode.create(geo_str)
        return qr


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--coords_file",
        type=str,
        required=True,
        help="Path to the GPX file with coordinates data.",
    )
    parser.add_argument(
        "--save_path",
        type=str,
        default=None,
        help="Path to save generated QR codes.",
    )

    return parser.parse_args()


def main():
    args = parse_args()
    window = GUI(args)
    window.create_window()
    window.run()


if __name__ == "__main__":
    main()
