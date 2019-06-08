#!/usr/bin/env python
import rospy
import actionlib
from selfie_msgs.msg import parkAction, parkGoal,parkResult
from selfie_msgs.msg import searchAction,searchGoal,searchResult
from ackermann_msgs.msg import AckermannDriveStamped

rospy.init_node('park_manager')
park_client = actionlib.SimpleActionClient('park_server',parkAction)
search_client = actionlib.SimpleActionClient('detect_parking_spot', searchAction)
speed_pub = rospy.Publisher('/drive',AckermannDriveStamped,queue_size=10)
print 'init'
print 'wait1'
search_client.wait_for_server()
goal = searchGoal(0.7)
search_client.send_goal(goal)
search_client.wait_for_result()
spot = search_client.get_result()
print 'half'
print 'wait2'
goal2 = parkGoal()
goal2.parking_spot = result.parking_spot
goal2.park = True
park_client.wait_for_server()
park_client.send_goal(goal2)
park_client.wait_for_result()
print 'end'





