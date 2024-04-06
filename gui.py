#!/usr/bin/env python3

import sys
sys.path.append('./../map_data/scripts/')
sys.path.append('./../map_data/data/')
import tkinter as tk
import tkinter.font as tkFont
import pyqrcode
import pickle
from PIL import Image, ImageTk
import utm

class GUI:
    def __init__(self):
        # WINDOW
        self.window_width = 1200 # px  
        self.window_height = self.window_width * 3 // 4
        self.window_title = "RoboTour GUI"
        self.window_destroy_key = 'w'
        self.save_qr_key = 's'
        self.wait_restore = 3000 # ms
        # BACKGROUND IMAGE
        self.bgd_path = sys.path[-1]
        self.bgd_name = "bgd_map.png"
        self.coords_name = "buchlovice_1.mapdata"
        self.coords_data = self.load_map_data(self.coords_name)
        # DOT
        self.dot_color = "red"
        self.dot_size = 12 # px
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
        self.qr_scale = 8
        self.saved = "QR code no. {0} was saved."

    def create_window(self):
        """ Create main window """
        self.root = tk.Tk()
        self.root.title(self.window_title)

        self.canvas = tk.Canvas(self.root, width=self.window_width, height=self.window_height, highlightthickness=0)
        self.canvas.pack()

        self.background_image = self.get_background_map()
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.background_image)

        self.bottom_frame = tk.Frame(self.root, bg=self.bottom_frame_bg)
        self.bottom_frame.pack(side=tk.BOTTOM, fill=tk.X)

        self.coords_label = tk.Label(self.bottom_frame, text="", bg=self.bottom_frame_bg, font=self.text_font)
        self.coords_label.pack(pady=5)

        self.coords_label.config(text=self.instructions, font=self.text_font_italic)

        self.root.bind("<Button-1>", self.cursor_click)
        self.root.bind("<Key>", self.key_press)

    def key_press(self, event):
        """ Handle various key press events """
        # Close window with 'w' key press 
        if event.char.lower() == self.window_destroy_key.lower():
            self.root.destroy()
            print("RoboTour GUI closed.")
        # Generate and save QR code with 's' key press
        if event.char.lower() == self.save_qr_key.lower():
            if self.counter > 0:
                self.create_qr((self.lat, self.lon))
                self.coords_label.config(text=self.saved.format(self.counter), font=self.text_font)
                print(self.saved.format(self.counter))
                self.root.after(self.wait_restore, self.restore_window)
            else: 
                print("No coordinates selected yet!")
        # Restore window with 'r' key press
        if event.char.lower() == 'r':
            self.restore_window()

    def cursor_click(self, event):
        """ Get cursor click coordinates and draw red dot """
        self.counter += 1
        # Remove old dot if drawn and draw new one
        if self.is_dot:
            self.restore_window()
        x, y = self.transform_window_coords(event.x, event.y)
        self.lat, self.lon = self.coords2latlon(x, y)
        print(f"({self.counter}) Selected place has latitude {self.lat} and longitude {self.lon}.")
        self.coords_label.config(text=self.bottom_frame_text.format(self.lat, self.lon), font=self.text_font)
        self.draw_dot(x, y)
        
    def transform_window_coords(self, x, y):
        """ Transform (0,0) from top-left to bottom-left corner """
        return x, self.window_height - y

    def draw_dot(self, x, y):
        """ Create red dot at position (x,y) """
        x, y = self.transform_window_coords(x, y)
        self.canvas.create_oval(x-self.dot_size/2, y-self.dot_size/2, x+self.dot_size/2, y+self.dot_size/2,
                                fill=self.dot_color, outline=self.dot_color, tags="dot")
        self.is_dot = True

    def restore_window(self):
        """ Remove red dot and bottom text after N seconds """
        self.canvas.delete("dot")
        self.coords_label.config(text="")
        self.coords_label.config(text=self.instructions, font=self.text_font_italic)

    def run(self):
        """ Run main loop """
        self.root.mainloop()
            
    def get_background_map(self):
        """ Get background map """
        img = Image.open(self.bgd_path + self.bgd_name)
        assert (img.size[0]/img.size[1] - 4/3 < 0.1), "Image is not 4:3!"
        img = img.resize((self.window_width, self.window_height))
        background_image = ImageTk.PhotoImage(img)
        return background_image

    def load_map_data(self, file_name):
        """ Load map data from file """
        with open(sys.path[-1] + file_name, "rb") as fh:
            map_data = pickle.load(fh)
            return map_data.coords_data

    def coords2latlon(self, x, y):
        """ Convert image coordinates to latitude and longitude """
        min_utm = utm.from_latlon(self.coords_data.min_lat - self.coords_data.y_margin, \
                                  self.coords_data.min_long - self.coords_data.x_margin)
        max_utm = utm.from_latlon(self.coords_data.max_lat + self.coords_data.y_margin, \
                                  self.coords_data.max_long + self.coords_data.x_margin)

        min_utm_easting, min_utm_northing = min_utm[0], min_utm[1]
        max_utm_easting, max_utm_northing = max_utm[0], max_utm[1]
        assert (min_utm[2] == max_utm[2] and min_utm[3] == max_utm[3]), "Different UTM zones!"
        zone_number, zone_letter = min_utm[2], min_utm[3]
        step_easting  = (max_utm_easting  - min_utm_easting)  / self.window_width
        step_northing = (max_utm_northing - min_utm_northing) / self.window_height
        easting  = min_utm_easting  + x * step_easting
        northing = min_utm_northing + y * step_northing

        lat, lon = utm.to_latlon(easting, northing, zone_number, zone_letter)
        return lat, lon
    
    def create_qr(self, geo):
        """ Create and save QR code with geo coordinates """
        geo_str = f"geo:{geo[0]},{geo[1]}"
        qr = pyqrcode.create(geo_str)
        return qr.png(f'qr{self.counter}.png', scale=self.qr_scale)


if __name__ == "__main__":
    window = GUI()
    window.create_window()
    window.run()
