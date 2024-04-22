#!/usr/bin/env python3.8

import rospy
from std_msgs.msg import String

import json

class Object_frontal:
    def __init__(self):
        self._yolo_data_sub = rospy.Subscriber('/yolo_data', String, self.callack)

    def callback(self, yolo_data):
        