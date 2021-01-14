#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import actionlib
from std_srvs.srv import Empty


rospy.init_node("mock_scanner")
print 'wait'
rospy.wait_for_service("startGateScan")
print 'end wait'
client = rospy.ServiceProxy("startGateScan", Empty)
pub = rospy.Publisher("/scan",LaserScan,queue_size=10)
print 'calling'
#client()
rospy.sleep(rospy.Duration(1))

msg = LaserScan()
msg.ranges = [0.3]
msg.angle_min = 0.
msg.angle_increment = 0.1
msg.range_max = 1
msg.range_min = 0
pub.publish(msg)


rospy.sleep(rospy.Duration(0.9))
msg.ranges = [0.6]
pub.publish(msg)

rospy.sleep(rospy.Duration(0.1))
msg.angle_min = -0.78
msg.ranges = [0.3]
pub.publish(msg)

rospy.sleep(rospy.Duration(0.1))
msg.angle_min = 0.78
msg.ranges = [0.3]
pub.publish(msg)

rospy.sleep(rospy.Duration(0.05))
msg.ranges = [0.05]
msg.angle_min = 0.0
pub.publish(msg)

rospy.sleep(rospy.Duration(0.05))
msg.ranges = [0.4, 0.4]
msg.angle_min = 0.78
msg.angle_increment = -0.78
pub.publish(msg)




