import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

getZ = 0.535

def callback(data):
    global getZ
    getZ = data.pose.position.z
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

def listener():
    rospy.init_node('feedbackZ', anonymous=True)
    rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback)
    rospy.spin()


def publisher():
    global getZ
    pub = rospy.Publisher('feedz', Float64, queue_size=10)
    rospy.loginfo(getZ)
    pub.publish(getZ)


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
