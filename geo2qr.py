#!/usr/bin/env python3

import png
import rospy
import pyqrcode
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class Geo2QR:

	def __init__(self):
		rospy.loginfo('Initializing Geo2QR node')
		rospy.init_node('geo2qr')
		self.initialized = False

		# subscriber
		self.sub = rospy.Subscriber('/geo2qr/geo', String, queue_size=10)

		# publisher
		self.pub = rospy.Publisher('/geo2qr/qr_codes', CompressedImage, self.create_qr)

		self.initialized = True
		rospy.loginfo('Geo2QR node initialized')

	def create_qr(self, geo):
		geo_str = f"geo:{geo[0]},{geo[1]}"
		return pyqrcode.create(geo_str)


if __name__ == '__main__':
	lat = 50.089748
	lon = 14.398400
	geo = (lat, lon)

	barcode = 'geo_qr.png'
	qr = Geo2QR()
	rospy.spin()