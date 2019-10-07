#! /usr/bin/env python

from __future__ import print_function
import rospy


from selfie_msgs.msg import PolygonArray
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32


def callback(data):
    print('setpoint changed')
    rospy.loginfo(rospy.get_caller_id() + "I heard %l", data.data)


def mock_road_obstacle():
    pub = rospy.Publisher('/obstacles', PolygonArray, queue_size=10)
    sub = rospy.Subscriber('/setpoint', Float32, callback)
    rospy.init_node('mock_road_obstacle', anonymous=True)
    point = Point32(x=0.4, y=-0.15, z=0)
    polygon = Polygon()
    polygon.points.append(point)
    point.y = 0.15
    polygon.points.append(point)
    point.x = 0.45
    polygon.points.append(point)
    point.y = -0.15
    polygon.points.append(point)
    polygons = PolygonArray()
    polygons.polygons.append(polygon)
    pub.publish(polygons)
    print('polygon sent')
    rospy.spin()


if __name__ == '__main__':
    try:
        print("sending obstacle...")
        mock_road_obstacle()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
# TODO check points order in Polygon
