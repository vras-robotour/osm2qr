import png
import cv2
import numpy as np
import pyqrcode
import pyzbar.pyzbar as pyzbar

def geo2qr(geo):
    geo_str = f"geo:{geo[0]},{geo[1]}"
    return pyqrcode.create(geo_str)


def qr2geo(barcode):
	img = cv2.imread(barcode)
	codes = pyzbar.decode(img)
	geo_codes = []

	if not codes:
		print('No QR code detected')
	else:
		for code in codes:
			assert code.type == 'QRCODE'
			data = code.data.decode("utf-8")
			if data[:3] == 'geo':
				print('Geo QR code detected')
				lat, lon = data[4:].split(',')
				geo_codes.append((lat, lon))
			else:
				print('No Geo QR code detected')
	print(geo_codes)


if __name__ == '__main__':
	
    lat = 50.089748
    lon = 14.398400
    geo = (lat, lon)
    filename = 'geo_qr.png'
	
    qr = geo2qr(geo)
    qr.png(filename, scale=10)
	
    barcode = './' + filename
    qr2geo(barcode)
