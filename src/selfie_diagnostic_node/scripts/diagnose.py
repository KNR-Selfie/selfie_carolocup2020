#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan #/scan
from sensor_msgs.msg import Image #/image_rect
from std_msgs.msg import Bool #/left_turn_indicator, /right_turn_indicator
from enum import Enum

class State(Enum):
    OK = 1
    WARNING = 2
    ERROR = 3
    FATAL = 4

    def key(self):
        return self.name

class Diagnostic :
    def laserScanCallback(self, scan):
        if(self.lidar["last_stamp"] == None):
            self.lidar["last_stamp"] = scan.header.stamp
        else:
            self.lidar["frequency"] = 1/(scan.header.stamp.to_sec() - self.lidar["last_stamp"].to_sec()) # Hz
            self.lidar["last_stamp"] = scan.header.stamp
        pass

    def __init__(self):
        self.left_blink_ = rospy.Publisher("/left_turn_indicator", Bool, queue_size=10)
        self.right_blink_ = rospy.Publisher("/right_turn_indicator", Bool, queue_size=10)
        self.lidar_sub_ = rospy.Subscriber("/scan", LaserScan, self.laserScanCallback)
        self.lidar = {
            "last_stamp" : None,
            "frequency" : 0.0,
        }      
        pass

    def diagnoseLidar(self):
        #if stamp wasnt filled once
        if(self.lidar["last_stamp"] == None):
            print(State.ERROR.key())
            print("1")
            return
        # checking if lidar stopped publishing while programme work
        # if rospy.get_time() - self.lidar["last_stamp"].to_sec() > 3.0: #TODO testing on car, not from bag
        #     print(State.ERROR.key())
        #     self.lidar["frequency"] = 0.0
        #     print("2")
        # checking if frequency of publishing is still 0 or is publishing with wrong frequency
        if(self.lidar["frequency"] == 0.0) or self.lidar["frequency"] > 10.5 or self.lidar["frequency"] < 9.5:
            print(State.ERROR.key())
            print("3 ")
        else:
            print(State.OK.key())
            # count how many times system was ok
            # condition on counter
            # lights blink
            self.left_blink_.publish(Bool(True))
            self.right_blink_.publish(Bool(True))
            rospy.sleep(2.)
            self.left_blink_.publish(Bool(False))
            self.right_blink_.publish(Bool(False))
            # Node shutdown
            rospy.signal_shutdown("Diagnose OK")
        pass


if __name__ == "__main__":
    diagnoser = Diagnostic()
    rospy.init_node("selfie_diagnostics", anonymous=True)
    print("Starting diagnostics")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        diagnoser.diagnoseLidar()
        rate.sleep()

