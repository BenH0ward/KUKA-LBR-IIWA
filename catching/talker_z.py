# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import time
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float64

def runVision():
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
	blueLower = (48, 62, 88)
	blueUpper = (151, 238, 255)
	armPixLower = 140
	armPixUpper = 300
	armMLower = 0.930
	armMUpper = 0.140
	armPixZero = 340 #position of Z in pixels when Cartesian Z = 0
	przelicznik = (armMLower-armMUpper)/(armPixUpper-armPixLower)
	ballInArmRange = False

	pts = deque(maxlen=args["buffer"])
	tintervals = deque(maxlen=args["buffer"])
	tPrev = 0;
	pRad = 0
	mapix = 0
	yinters = 0
	passPosition = 0

	pub = rospy.Publisher('positionZ', Float64, queue_size=10)
	joint = PoseStamped()
	rospy.init_node('talker', anonymous=True)
	count = 0

	#initalize Kalman parameters
	tkalman = 0.01
	kalman = cv2.KalmanFilter(6,6)
	kalman.measurementMatrix = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],
										[0,0,0,0,0,1]],np.float32)
	kalman.transitionMatrix = np.array([[1,0,tkalman,0,(0.5*(tkalman**2.0)),0],[0,1,0,tkalman,0,(0.5*(tkalman**2.0))],
										[0,0,1,0,tkalman,0],[0,0,0,1,0,tkalman],[0,0,0,0,1,0],[0,0,0,0,0,1]],np.float32)
	kalman.processNoiseCov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],
										[0,0,0,0,0,1]],np.float32) * 0.01
	kalman.measurementNoiseCov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],
										[0,0,0,0,0,1]],np.float32) * 0.00001


	# if a video path was not supplied, grab the reference
	# to the webcam
	if not args.get("video", False):
		camera = cv2.VideoCapture(1)

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

		mask = cv2.inRange(hsv, blueLower, blueUpper)
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
				y0 = pts[0][1]
				x0 = pts[0][0]
				dy = pts[0][1] - pts[1][1]
				dx = pts[0][0] - pts[1][0]
				dt = tintervals[0]
				vx = dx/dt
				vy = dy/dt
				ax = 0
				ay = 98.1/przelicznik

				kalmanin = np.array((6,1), np.float32) # measurement
				kalmanout = np.zeros((6,1), np.float32) # tracked / prediction
				kalmanin = np.array([[np.float32(x0)],[np.float32(y0)],[np.float32(vx)],[np.float32(vy)],[np.float32(ax)],[np.float32(ay)]])
				kalman.correct(kalmanin)
				kalmanout = kalman.predict()

				x0 = kalmanout[0]
				y0 = kalmanout[1]
				vx = kalmanout[2]
				vy = kalmanout[3]
				ax = kalmanout[4]
				ay = kalmanout[5]

				listX = []
				listY = []

				for i in range(1, 200):
					t = 0.005 * i
					y = y0 + (dy)/dt * t + (ay * (t ** 2)) / 2
					x = x0 + (dx)/dt * t
					y = int(y)
					x = int(x)
					listX.append(x)
					listY.append(y)

					if ((x < 4 and x > -4) and (y >= 140 and y <= 300)):
						yinters = y
						ballInArmRange = True

				setZ = (armPixZero - yinters) * przelicznik


				if (ballInArmRange):
					positionz = setZ
					rospy.loginfo(positionz)
					pub.publish(positionz)

				for i in range(0, 100):
					if listY[i] >= 0 and listY[i] <= 500:
						cv2.line(frame, (listX[i], listY[i]), (listX[i+1], listY[i+1]), (255, 0, 0), 4)

				if (ballInArmRange):
					cv2.line(frame, (0, yinters), (600, yinters), (0, 255, 0), 4)
					ballInArmRange = False



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

		cv2.putText(frame, "passPosition: {}".format(passPosition),
			(120, frame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX,
			0.5, (0, 0, 255), 1)

		cv2.putText(frame, "yinters: {}".format(yinters),
			(120, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX,
			0.5, (0, 0, 255), 1)

		cv2.putText(frame, "przelicznik: {}".format(przelicznik),
			(120, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX,
			0.5, (0, 0, 255), 1)

		cv2.putText(frame, "x, y: {}".format(pts[0]),
			(10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
			0.35, (0, 0, 255), 1)

		# show the frame to our screen
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF



		# if the 'q' key is pressed, stop the loop
		if key == ord("q"):
			break

	# cleanup the camera and close any open windows
	camera.release()
	cv2.destroyAllWindows()



if __name__ == '__main__':
	try:
		runVision()
	except rospy.ROSInterruptException:
		pass
