#!/usr/bin/env python3.8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator  # ultralytics.yolo.utils.plotting is deprecated

import cv2


class Yolov8_detect:
    def __init__(self):
        self._image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self._bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

    def detect(self, img):
        results = self.model.predict(img)

        for r in results:
            annotator = Annotator(img)
            
            boxes = r.boxes
            for box in boxes:
                
                b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                c = box.cls
                annotator.box_label(b, self.model.names[int(c)])
            
        img = annotator.result()  
        cv2.imshow('YOLO V8 Detection', img)     
        cv2.waitKey(1)

    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
            self.detect(cv_image)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('yolov8_detect')
    yolov8_detect = Yolov8_detect()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass