#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import pyzbar.pyzbar as pyzbar
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

# pyzbar install: https://pypi.org/project/pyzbar/

class QR2Geo:

    def __init__(self):
        rospy.loginfo('Initializing QR2Geo node')
        rospy.init_node('qr2geo')
        self.initialized = False

        # subscriber
        self.sub = rospy.Subscriber('/camera_front/image_color/compressed', CompressedImage, self.read_qr)

        # publisher
        self.pub = rospy.Publisher('/qr2geo/geo_codes', String, queue_size=10)

        self.initialized = True
        rospy.loginfo('QR2Geo node initialized')

    def read_qr(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        codes = pyzbar.decode(image_np)
        geo_codes = []

        if not codes:
            rospy.loginfo('No QR code detected')
        else:
            for code in codes:
                if code.type == 'QRCODE':
                    rospy.loginfo('QR code detected')
                    data = code.data.decode("utf-8")
                    if data[:3] == 'geo':
                        rospy.loginfo('Geo QR code detected')
                        lat, lon = data[4:].split(',')
                        geo_codes.append((lat, lon))
                    else:
                        rospy.loginfo('No Geo QR code detected')
        return np.array(geo_codes)

if name == '__main__':
    barcode = './codes/prague_castle.png'
    qr2geo = QR2Geo()
    rospy.spin()