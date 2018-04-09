from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import serial
import time
import cv2
import argparse
import imutils

# frame variables
xFrame = 240
yFrame = 320

#threshold for the number of points circle should have
circleThres = 20
minArea =20000
close = True
initialized = False

print('here')

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (xFrame, yFrame)
camera.framerate = 32
camera.rotation = 270
rawCapture = PiRGBArray(camera, size=(xFrame, yFrame))

print('here1')

ser = serial.Serial(
  
   port='/dev/ttyACM0',
   baudrate = 115200,
   parity=serial.PARITY_NONE,
   stopbits=serial.STOPBITS_ONE,
   bytesize=serial.EIGHTBITS,
   timeout=1
)

# allow the camera to warmup
time.sleep(5)

# initialize the rescue zone code on arduino side
ser.write("S")
initialized = True

class ShapeDetector:
		def __init__(self):
			pass
		   

		def detect(self, c):
			# initialize the shape name and approximate the contour
			shape = -1
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.055 * peri, True) #0.04

			# if the shape is a triangle, it will have 3 vertices
			if len(approx) == 3:
				shape = 3
				return shape

			# if the shape has 4 vertices, it is either a square or a rectangle
			elif len(approx) == 4:
				# compute the bounding box of the contour and use the
				# bounding box to compute the aspect ratio
				(x, y, w, h) = cv2.boundingRect(approx)
				ar = w / float(h)
				shape = 4 
				return shape
			elif len(approx) > circleThres: # circle
				shape = 2
				return shape
			else:
			# return the name of the shape
				return shape


	# sends if the camera has detected an evacuation zone
def sendToArduino(found):
		if found:
			ser.write('1')  # evac zone found
		else:
			ser.write('1')  # not found 

def servoDown():
	pass

def servoUp():
	pass

def servoRelease():
	pass

#wait for the arduino to tell pi to look for zone
while initialized is True:
	while True:
		receiving = '0'
		receiving = ser.read() 
		print(receiving)
		#evac zone codes
		if (receiving is 'F'):
			close = False
			print("reachedF1")
			break
			
		if (receiving is 'C'):
			close = True
			break

		#servo codes
		if (receiving is S):
			nextChar = ser.read()
			if (nextChar is D):
				servoDown()
			if (nextChar is U):
				servoUp()
			if (nextChar is R):
				servoRelease()
				
	# capture frames from the camera
	print("reach")
	camera.capture(rawCapture, format="bgr")
	image = rawCapture.array
	 
	#image conversions
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)[1]

	# find contours in the thresholded image and initialize the shape detector
	
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
	sd = ShapeDetector()

	# get max contour
	c1 = max(cnts, key = cv2.contourArea)

	# compute the center of the max contour, then detect the name of the
	# shape using only the contour
	M = cv2.moments(c1)
	if (M["m00"] > 0 and cv2.contourArea(c1) > minArea): 
			cX = int((M["m10"] / M["m00"]))
			cY = int((M["m01"] / M["m00"]))
			shape = sd.detect(c1)
	
			#draw the contours on the image
			if(shape is 4):
					c1 = c1.astype("int")
					cv2.drawContours(image, [c1], -1, (0, 255, 0), 2)
					#for debugging, print the shape -1 nothing, 3 triangle
					print(shape)
					print (cv2.contourArea(c1))
					sendToArduino(True)
			else:
				print('something but not zone')
				sendToArduino(False)
	else:
			print('nothing')
			sendToArduino(False)

	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		initialized = False
		break
