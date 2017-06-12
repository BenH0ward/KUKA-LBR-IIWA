import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

getY = 0.66

def callback(data):
    global getY
    rospy.loginfo(data.pose.position.y)
    getY = data.pose.position.y
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

def listener():
    rospy.init_node('feedbackY', anonymous=True)
    rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, callback)
    rospy.spin()


def publisher():
    global getY
    pub = rospy.Publisher('feedy', Float64, queue_size=10)
    rospy.loginfo(getY)
    pub.publish(getY)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
