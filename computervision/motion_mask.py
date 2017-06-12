# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import time

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=10,
	help="max buffer size")
ap.add_argument("-a", "--min-area", type=int, default=500, help="minimum area size")

args = vars(ap.parse_args())
# define the lower and upper boundaries of the "blue"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (48, 62, 88)
greenUpper = (151, 238, 255)
pts = deque(maxlen=args["buffer"])
tintervals = deque(maxlen=args["buffer"])
tPrev = 0;
pRad = 0
mapix = 0
mspeed = 0
lastT = time.time()

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
	camera = cv2.VideoCapture(0)

# otherwise, grab a reference to the video file
else:
	camera = cv2.VideoCapture(args["video"])
	# keep looping

#initialize background subtraction
fgbg = cv2.createBackgroundSubtractorMOG2()

while True:
	# grab the current frame
	(grabbed, frame) = camera.read()

	# start counting time
	tPrev = time.time()

	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if args.get("video") and not grabbed:
		break

	# resize the frame and apply background subtraction
	frame = imutils.resize(frame, width=720)
	mask = fgbg.apply(frame)
	res = cv2.bitwise_and(frame, frame, mask = mask)

    # blur the frame and convert it to the HSV
	blurred = cv2.GaussianBlur(res, (11, 11), 0)
	hsv = cv2.cvtColor(res, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "blue", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask

	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)


	# show the frame to our screen
	cv2.imshow("Frame", res)
	nowT = time.time() - lastT
	nowT = 0.033 - nowT
	nowT *= 1000
	nowT = int(nowT)
	cv2.waitKey(nowT)
	lastT = time.time()
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
