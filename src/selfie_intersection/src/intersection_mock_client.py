#! /usr/bin/env python

from __future__ import print_function
import rospy
import actionlib

import selfie_msgs.msg 

def intersection_client():
    client = actionlib.SimpleActionClient('intersection', selfie_msgs.msg.intersectionAction)
    client.wait_for_server()
    goal = selfie_msgs.msg.intersectionGoal
    print("Sending goal")
    client.send_goal(goal)
    print("Goal sent.Waiting for result")
    client.wait_for_result()
    print("Result achieved")
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('intersection_mock_client_py')
        result = intersection_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)