import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

angle=0 # steering between -3.0 and 3.0 (negative left)
current_speed=0 # speed between 1.0 and -1.0 (negative reverse)
speed=1
angle_factor=1.5
search_speed=0.6
last_target_x=0

ball_class = 32
bottle_class = 39
cup_class = 41
wine_glass = 40

target=ball_class

def callback(pos: Vector3):
    global angle, current_speed, last_target_x, caught, target

    _target = pos.z
    x = pos.x
    y = pos.y

    if _target == cup_class:
        mission_pub.publish("restart by removing the cup ...")
        target = ball_class
        angle=0
        current_speed=0
        return

    if target == -1:
        angle=0
        current_speed=0
        mission_pub.publish("the end")
        return

    if _target == wine_glass and x > 0.25 and x < 0.75:
        angle=(x - 0.75) * -3.0
        current_speed=0.25
        mission_pub.publish("bypass wine glass")
        return

    # noting found in the image => turn to search
    if _target == -1:
        current_speed=0
        if last_target_x >= 0.5:
            angle=search_speed
        else:
            angle=-search_speed
        # angle=0.25
        if target == ball_class:
            mission_pub.publish("searching for ball ...")
        elif target == bottle_class:
            mission_pub.publish("searching for bottle ...")
        return

    if target != _target:
        return

    # object is close => we caught it
    if y >= 0.8:
        if target == ball_class:
            mission_pub.publish("caught the ball => now hunting the bottle")
            target = bottle_class
        elif target == bottle_class:
            mission_pub.publish("caught the bottle => stopping (use a cup to restart)")
            target = -1 # end
        return

    # found => drive to target
    if x > 0 and x <= 1.0:
        last_target_x=x
        current_speed=speed
        angle=(x - 0.5) * -3.0 * angle_factor # map 0 - 1.0 => -3.0 - 3.0
        if target == ball_class:
            mission_pub.publish("driving towards ball")
        elif target == bottle_class:
            mission_pub.publish("driving towards bottle")

def loop():
    global cmd_pub, angle, current_speed
    if rospy.is_shutdown():
        return
    twist = Twist()
    twist.linear.x = current_speed
    twist.angular.z = angle
    cmd_pub.publish(twist)

def init():
    global cmd_pub, mission_pub

    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    mission_pub = rospy.Publisher('mission', String, queue_size=10)
    rospy.init_node('ball_schubser_control', anonymous=True)
    rospy.Subscriber("ball_pos", Vector3, callback)
    rospy.loginfo("Starting navigation node ...")
    mission_pub.publish("launching navigation node ...")
    print("done")

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        loop()
        rate.sleep()

if __name__ == '__main__':
    init()
