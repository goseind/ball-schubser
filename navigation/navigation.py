import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
# from std_msgs.msg import Float64

angle=0 # steering between -3.0 and 3.0 (negative left)
current_speed=0 # speed between 1.0 and -1.0 (negative reverse)
speed=1
angle_factor=1.2
search_speed=0.6
last_target_pos=0
caught = False

def callback(pos: Quaternion):
    global angle, current_speed, last_target_pos, caught

    caught = caught or (pos.y >= 0.70)
    target_pos = pos.z if caught else pos.x
    # ball_pos = pos.x
    print(str(pos.x) + ", " + str(pos.y) + ", " + str(pos.z) + ", " + str(pos.w) + ": " + str(target_pos))
    print("bottle" if caught else "ball")

    # ball_pos = -0.1
    if target_pos == -1.0:
        current_speed=0
        if last_target_pos >= 0.5:
            angle=search_speed
        else:
            angle=-search_speed
        # angle=0.25
    if target_pos > 0 and target_pos <= 1.0:
        # rospy.loginfo("I heard %f", ball_pos)
        last_target_pos=target_pos
        current_speed=speed
        angle=(target_pos - 0.5) * -3.0 * angle_factor # map 0 - 1.0 => -3.0 - 3.0

def loop():
    global cmd_pub, angle, current_speed
    if rospy.is_shutdown():
        return
    twist = Twist()
    twist.linear.x = current_speed
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
