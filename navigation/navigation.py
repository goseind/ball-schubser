import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
# from std_msgs.msg import Float64

angle=0 # steering between -3.0 and 3.0 (negative left)
speed=0 # speed between 1.0 and -1.0 (negative reverse)
last_ball_pos=0

def callback(pos: Quaternion):
    global angle, speed, last_ball_pos
    ball_pos = pos.x
    if ball_pos == -1.0:
        speed=0
        if last_ball_pos >= 0.5:
            angle=0.75
        else:
            angle=-0.75
        # angle=0.25
    if ball_pos > 0 and ball_pos <= 1.0:
        # rospy.loginfo("I heard %f", ball_pos)
        last_ball_pos=ball_pos
        speed=1.0
        angle=(ball_pos - 0.5) * -3.0 # map 0 - 1.0 => -3.0 - 3.0

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
    rospy.Subscriber("ball_pos", Quaternion, callback)
    rospy.loginfo("Starting navigation node ...")
    print("done")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        loop()
        rate.sleep()

if __name__ == '__main__':
    init()
