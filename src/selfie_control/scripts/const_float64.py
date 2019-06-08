#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('selfie_const_float64')

    topic = rospy.get_param('~topic')
    value = rospy.get_param('~value')
    publish_rate = rospy.get_param('~publish_rate', 50)

    pub = rospy.Publisher(topic, Float64, queue_size=1)

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        pub.publish(value)
        rate.sleep()
