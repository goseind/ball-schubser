#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, Point

angle=0 # steering between 3.0 and -3.0 (negative left)
speed=0.3 # speed between 1.0 and -1.0 (negative reverse)

def callback(data):
    global angle
    rospy.loginfo("I heard %s", data.data)
    angle=-1.5 # TODO: get value from data

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

    cmd_pub = rospy.Publisher('cmd_vel_rc100', Twist, queue_size=10)
    rospy.init_node('ball_schubser_control', anonymous=True)

    rospy.loginfo("Welcome")

    rospy.Subscriber("img_pos", Point, callback)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        loop()
        rate.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
