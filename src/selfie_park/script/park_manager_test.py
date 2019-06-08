#!/usr/bin/env python
import rospy
import actionlib
from selfie_msgs.msg import parkAction, parkGoal,parkResult
from selfie_msgs.msg import searchAction,searchGoal,searchResult
from ackermann_msgs.msg import AckermannDriveStamped

rospy.logwarn('0')
rospy.init_node('park_manager')
park_client = actionlib.SimpleActionClient('park',parkAction)
search_client = actionlib.SimpleActionClient('search', searchAction)
speed_pub = rospy.Publisher('/drive',AckermannDriveStamped,queue_size=10)

drive_msg = AckermannDriveStamped()
drive_msg.drive.speed = 0.4
drive_msg.drive.steering_angle = 0
rospy.logwarn('1')
search_client.wait_for_server()
rospy.logwarn('2')
goal = searchGoal(0.7)
speed_pub.publish(drive_msg)
search_client.send_goal(goal)
rospy.logwarn('3')
search_client.wait_for_result()
rospy.logwarn('4')
result = search_client.get_result()
print 'end search'
goal2 = parkGoal()
goal2.parking_spot = result.parking_spot
goal2.park = True
park_client.wait_for_server()
rospy.logwarn('5')
park_client.send_goal(goal2)
rospy.logwarn('6')
park_client.wait_for_result()
rospy.logwarn('7')
print 'end'





