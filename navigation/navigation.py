import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

angle=0 # steering between -3.0 and 3.0 (negative left)
speed=0.3 # speed between 1.0 and -1.0 (negative reverse)

def callback(pos: Float64):
    global angle
    rospy.loginfo("I heard %d", pos)
    if pos.data > 0 and pos.data <= 1.0:
        angle=(pos.data - 0.5) * 6.0 # map 0 - 1.0 => -3.0 - 3.0

def loop():
    global cmd_pub, angle, speed
    if rospy.is_shutdown():
        return
    twist = Twist()
    twist.linear.x = speed
    twist.angular.z = angle
    cmd_pub.publish(twist)

def init():
    global cmd_pub

    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('ball_schubser_control', anonymous=True)
    rospy.Subscriber("ball_pos", Float64, callback)
    rospy.loginfo("Starting navigation node ...")
    print("done")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        loop()
        rate.sleep()

if __name__ == '__main__':
    init()
