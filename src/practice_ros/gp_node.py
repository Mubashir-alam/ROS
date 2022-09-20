#! /usr/bin/env python3.8

from fyp_robot.msg import pexel
# import imp
import rospy 
from cv_bridge import CvBridge, CvBridgeError
# from fyp_robot.msg import _s
from fyp_robot.msg import pexel
#from fyp_robot.msg import *
import cv2

cap = cv2.VideoCapture(0)
res, frame = cap.read()
cv2.imshow("video", frame)
hight,width,channel=frame.shape


def talker():
    pub=rospy.Publisher('/webcam',pexel,queue_size=10)
    rospy.init_node('pexel_pub',anonymous=True)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame=cap.read()
        if not ret:
            break
        #msg =bridge.cv2_to_imgmsg(frame,"bgr8")
        #pub.publish(msg)
        #pub.publish(hight,width)
        #pub.publish(hight,width,channel)
        #pub.publish(width)

        if 0xFF == ord('q'):
            break
        if rospy.is_shutdown():
            cap.release()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

# print("hight ",hight)
# print("width ",width)
# print("channel ",channel)

