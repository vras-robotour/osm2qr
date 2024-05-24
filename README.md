# osm2qr
ROS tools to work with OSM data and QR codes

- [Overview](#overview)
- [How to use](#how-to-use)
    - [Graphical User Interface](#graphical-user-interface)
    - [Getting geo coordinates from a QR code](#getting-geo-coordinates-from-a-qr-code)
    - [Generating QR codes from a region of the map](#generating-qr-codes-from-a-region-of-the-map)
- [License](#license)

## Overview
This is a ROS package that provides tools to work with OSM data and QR codes. The package is composed of three nodes that can be used independently.  

The package was developed and tested on Ubuntu 20.04 with ROS Noetic and should wokr with other versions of ROS as well. 

## How to use
The scripts can be run either with the `rosrun` command or as an executable. We recommend using the launch file provided for each of the scripts. All launch files will set the parameters for the nodes and launch the particular node. With the launch files you can change the topic names and parameters. The launch files are located in the `./launch/` directory.

### Graphical User Interface
The script `gui` is a GUI which contains an OpenStreetMap map. The interface allows a user to select a location in the map, generate a QR code with the geo coordinates of that location, and save the code as an image file. The GUI is implemented using the `tkinter` library, the script uses the `pyqrcode` library to generate the QR code. The loaded map is generated using the `map_data` [package](https://github.com/vras-robotour/map_data).

To launch the script with the launch file, run the following command:
```bash
roslaunch osm2qr gui.launch
```

The script takes several parameters from the ROS parameter server:
- `~bgd_path`: path to the map file
- `~bgd_name`: name of the map file
- `~coords_name`: name of the coordinates used to generate the OSM map from the `map_data` package
- `~save_path`: path to save the QR code image file

After starting the GUI, the user can click on the map to select a location. The selected location is marked with a red circle and the longitude and latitude of the location are displayed in the window. The user can generate a QR code with the geo coordinates and save the code as an image file. If not, the selected point in the map is removed after 3 seconds to allow the user to select a new location.

#### Keyboard shortcuts
- `s`: generates a QR code with the geo coordinates of the selected location by clicking on the map, the QR code is saved as an image file
- `w`: destroys the GUI window

### Getting geo coordinates from a QR code
The script `qr2geo` is used to obtain the geo coordinates, namely longitude and latitude, of the location saved as a QR code. The script subscribes to the camera topic of a robot and looks for any QR code appearing in the image. If a QR code is detected with the correct format (i.e., containing the string `geo:`), longitude and latitude of the location are published. The script uses the `pyzbar` library to decode the QR codes. The location is published on the topic `/geocode`. The provided launch file has one argument:
- `robot_name`: the name of the robot, we consider four different robots: `ctu-robot`, `flip`, `marv-robot`, and `spot`, the script will a correct camera topic name for the selected robot

To launch the script with the launch file, run the following command:
```bash
roslaunch osm2qr qr2geo.launch robot:=<robot_name>
```

### Generating QR codes from a region of the map
The script `rviz2qr` is used to generate QR codes with geo coordinates of a selected location in the RViz robot map vizualization. When a goal pose in RViz is chosen by the user, the script subscribes to the topic `/move_base_simple/goal` and transforms the goal pose from the `local frame` to the `utm frame`. With the transformed coordinates, latitude and longitude are calculated and a QR code with the geo coordinates is generated and published using CvBridge. The script uses the `pyqrcode` library to generate the QR code. The QR code is published on the topic `/rviz_qr_codes`.

To launch the script with the launch file, run the following command:
```bash
roslaunch osm2qr rviz2qr.launch
```

The script takes several parameters from the ROS parameter server:
- `~utm_frame`: the name of the UTM frame to which the goal pose is transformed, default is `utm`
- `~target_altitude`: the altitude of the goal pose, default is `0`
- `~zone_number`: the UTM zone number, default is `33`
- `~zone_letter`: the UTM zone letter, default is `U`

## License
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/vras-robotour/osm2qr/blob/master/LICENSE)

