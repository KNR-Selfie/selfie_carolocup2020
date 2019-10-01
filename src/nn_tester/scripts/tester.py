#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from keras.preprocessing import image as IMG
from keras.models import load_model
#import cv_bridge

class NNSubscriber():
    def __init__(self):
        rospy.init_node('nn_subscriber')
        self.bridge_ = CvBridge()
        self.classifier_ = load_model("/home/vm/Downloads/klasyfikator_caly.h5")
        self.classifier_._make_predict_function()
        print(self.classifier_.summary())
        self.subscriber_ = rospy.Subscriber("image_publisher_1569926187587390772/image_raw", Image, self.callback)
        
    def callback(self, data):
        img = np.frombuffer(data.data, dtype=np.uint8)
        img = np.reshape(img, newshape=(442, 664, -1))
        img = cv2.resize(img,(64,64))
        cv2.imshow("XD",img)
        img = IMG.img_to_array(img)
        img = np.expand_dims(img, axis=0)
        result = self.classifier_.predict(img)
        print(result)
        cv2.waitKey(3)
        pass
    pass


if __name__ == "__main__":
    rospy.loginfo("Start ros")
    sub = NNSubscriber()
    print("XDDXDX")
    rospy.spin()
