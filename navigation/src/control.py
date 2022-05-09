#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

def talker():
        # create publisher called cmd_vel
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # pobably different node name
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
                # type for cmd_vel is Twist (has linear for velocity and angular for steering)
        twist = Twist()
        twist.linear.x = 0.4
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
