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

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		pRad = radius
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

	# update time intervals queue
	tintervals.appendleft(time.time() - tPrev)

	# update the points queue
	pts.appendleft(center)

	# predict position of the ball
	if (pRad > 0 and len(pts) > 5):
		if pts[0] != None and pts[1] != None:
			apix = 98.1/(0.032/pRad)
			mapix = apix

			y0 = pts[0][1]
			x0 = pts[0][0]

			dy = pts[0][1] - pts[1][1]
			dx = pts[0][0] - pts[1][0]

			dt = tintervals[0]

			listX = []
			listY = []

			for i in range(1, 11):
				t = 0.01 * i
				y = y0 + dy/dt * t + (apix * (t ** 2)) / 2
				x = x0 + dx/dt * t
				listX.append(int(x))
				listY.append(int(y))
				mspeed = dy/dt


			for i in range(0, 9):
				cv2.line(frame, (listX[i], listY[i]), (listX[i+1], listY[i+1]), (255, 0, 0), 4)



	# loop over the set of tracked points
	for i in xrange(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)


	# show the frame to our screen
	cv2.imshow("Frame", frame)
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
