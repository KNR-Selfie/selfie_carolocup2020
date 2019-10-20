#! /usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
import time

from std_msgs.msg import Float32
from selfie_msgs.msg import PolygonArray
import selfie_msgs.msg 

def intersection_client():
    client = actionlib.SimpleActionClient('intersection', selfie_msgs.msg.intersectionAction)
    client.wait_for_server()
    goal = selfie_msgs.msg.intersectionGoal()
    print("Sending goal")
    client.send_goal(goal)
    distance_pub=rospy.Publisher('/intersection_distance', Float32, queue_size=10)
    distance=Float32(data=5)
    time.sleep(0.5)
    print("Sending mock (far) distance to intersection.")
    distance_pub.publish(distance)
    polygons = PolygonArray()
    pub = rospy.Publisher('/obstacles', PolygonArray, queue_size=10)
    time.sleep(0.5)
    print("."),
    pub.publish(polygons)
    time.sleep(0.8)
    print("."),
    pub.publish(polygons)
    distance.data=0.05
    distance_pub.publish(distance)
    time.sleep(0.8)
    print("."),
    pub.publish(polygons)
    time.sleep(1)
    print("."),
    pub.publish(polygons)
    time.sleep(1)
    print("."),
    pub.publish(polygons)
    time.sleep(1)
    print("."),
    pub.publish(polygons)
    print('mock obstacles sent')
    client.wait_for_result()
    print("Result achieved")
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('intersection_mock_client_py')
        result = intersection_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)