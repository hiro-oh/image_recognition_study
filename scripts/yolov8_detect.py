#!/usr/bin/env python3.8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator  # ultralytics.yolo.utils.plotting is deprecated

import cv2

class Yolov8_detect(object):
    def __init__(self):
        self._image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        self._bridge = CvBridge()

    def detect(self, cap):
        # モデル読み込み（https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt）
        model = YOLO("yolov8n.pt")

        # cap = cv2.VideoCapture(0)
        cap.set(3, 640)
        cap.set(4, 480)

        while True:
            _, img = cap.read()
            
            # BGR to RGB conversion is performed under the hood
            # see: https://github.com/ultralytics/ultralytics/issues/2575
            results = model.predict(img)

            for r in results:
                
                annotator = Annotator(img)
                
                boxes = r.boxes
                for box in boxes:
                    
                    b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                    c = box.cls
                    annotator.box_label(b, model.names[int(c)])
                
            img = annotator.result()  
            cv2.imshow('YOLO V8 Detection', img)     
            if cv2.waitKey(1) & 0xFF == ord(' '):
                break

        cap.release()
        cv2.destroyAllWindows()

    def callback(self, data):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')
            detect(cv_image)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('yolov8_detect')
    detect = Yolov8_detect()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass