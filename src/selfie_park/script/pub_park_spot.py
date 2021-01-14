#!/usr/bin/env python3
import rospy
import actionlib

from selfie_msgs.msg import parkAction, parkGoal
from geometry_msgs.msg import Point32


rospy.init_node('publish_place')
goal = parkGoal()
client = actionlib.SimpleActionClient('park', parkAction)
print 'hello'
bl = Point32()
bl.x = 0.3
bl.y = -0.25
bl.z = 0
tl = Point32()
tl.x = 0.9
tl.y = -0.3
tl.z = 0
br = Point32()
br.x = 0.3
br.y = -0.35
br.z = 0
tr = Point32()
tr.x = 0.9
tr.y = -0.35
tr.z = 0
goal.parking_spot.points.append(bl)
goal.parking_spot.points.append(br)
goal.parking_spot.points.append(tr)
goal.parking_spot.points.append(tl)
client.wait_for_server()
client.send_goal(goal)





