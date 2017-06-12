import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

count = 0
getXpix = 360
getYpix = 270
getZ = 0.535
getY = 0.66

def callback(data):
    global getYpix
    global count
    rospy.loginfo('callback1: %f', data.data)
    getYpix = data.data

def callback2(data):
    global getXpix
    rospy.loginfo('callback2: %f', data.data)
    getXpix = data.data

def callback3(data):
    global getZ
    rospy.loginfo('callback3: %f', data.data)
    getZ = data.data

def callback4(data):
    global getY
    rospy.loginfo('callback4: %f', data.data)
    getY = data.data
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

def publisher():
    global count
    global getXpix
    global getYpix
    global getY
    global getZ

    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    joint = PoseStamped()

    joint.header.seq = count
    joint.header.stamp = rospy.Time.now()
    joint.header.frame_id = ""
    joint.pose.position.x = -0.126
    joint.pose.orientation.x = 0.37997234165
    joint.pose.orientation.y = -0.596354351601
    joint.pose.orientation.z = -0.379993107133
    joint.pose.orientation.w = 0.596311748028

    if ((getXpix < 360 - 15) and (getY - 0.010 >= 0.515)):
        joint.pose.position.y = getY - 0.010

        if ((getYpix > 270 + 15) and (getZ - 0.010 >= 0.140)):
            joint.pose.position.z = getZ - 0.010

        elif ((getYpix < 270 - 15) and (getZ + 0.010 <= 0.624)):
            joint.pose.position.z = getZ + 0.010
        else:
            joint.pose.position.z = getZ

    elif ((getXpix > 360 + 15) and (getY + 0.010 <= 0.774)):
        joint.pose.position.y = getY + 0.010

        if ((getYpix > 270 + 15) and (getZ - 0.010 >= 0.140)):
            joint.pose.position.z = getZ - 0.010

        elif ((getYpix < 270 - 15) and (getZ + 0.010 <= 0.624)):
            joint.pose.position.z = getZ + 0.010

        else:
            joint.pose.position.z = getZ

    else:
        joint.pose.position.y = getY

        if ((getYpix > 270 + 15) and (getZ - 0.010 >= 0.140)):
            joint.pose.position.z = getZ - 0.010

        elif ((getYpix < 270 - 15) and (getZ + 0.010 <= 0.624)):
            joint.pose.position.z = getZ + 0.010

        else:
            joint.pose.position.z = getZ


    rospy.loginfo(joint)
    pub.publish(joint)
    count += 1

def listener():
    rospy.init_node('subscribertracking', anonymous=True)
    rospy.Subscriber("centery", Float64, callback)
    rospy.Subscriber("centerx", Float64, callback2)
    rospy.Subscriber("feedz", Float64, callback3)
    rospy.Subscriber("feedy", Float64, callback4)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
