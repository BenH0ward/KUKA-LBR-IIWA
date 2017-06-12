import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

count = 0
getY = 0.66
getZ = 0.535

def callback(data):
    global count
    global getZ
    rospy.loginfo('callback1: %f', data.data)
    getZ = data.data

def callback2(data):
    global getY
    rospy.loginfo('callback2: %f', data.data)
    getY = data.data
    try:
        publisher1()
    except rospy.ROSInterruptException:
        pass

def publisher1():
    global count
    global getY
    global getZ
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    joint = PoseStamped()

    setZ = getZ
    setY = getY

    joint.header.seq = count
    joint.header.stamp = rospy.Time.now()
    joint.header.frame_id = ""

    joint.pose.position.x = -0.126
    joint.pose.position.z = setZ #0.535
    joint.pose.position.y = setY #0.66
    joint.pose.orientation.x = 0.37997234165
    joint.pose.orientation.y = -0.596354351601
    joint.pose.orientation.z = -0.379993107133
    joint.pose.orientation.w = 0.596311748028

    rospy.loginfo(joint)
    pub.publish(joint)
    count += 1

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("positionZ", Float64, callback)
    rospy.Subscriber("positionY", Float64, callback2)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
