#!/usr/bin/env python3
import rospy
import actionlib
from selfie_msgs.msg import startingAction, startingGoal


if __name__ == '__main__':
    rospy.init_node('mock_starting_procedure_client')
    client = actionlib.SimpleActionClient('starting_procedure',startingAction)
    client.wait_for_server()

    goal = startingGoal()
    print 'send goal'
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(10)) 