
import imp
from termios import FFDLY
from tkinter import Image
from traceback import print_tb
import rospy 
from sensor_msgs.msg import Image
from std_msgs.msg import String
# from fyp_robot import pexel
from cv_bridge import CvBridge, CvBridgeError
import cv2


cap = cv2.VideoCapture(0)
res, frame = cap.read()
cv2.imshow("video", frame)
hight,width,channel=frame.shape
hight=str(hight)
width=str(width)

def talker():
    pub = rospy.Publisher('pexel', String, queue_size=10)
    rospy.init_node('pexel_pub', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    i = 0
    while not rospy.is_shutdown():
        hello_str = "hight=" + hight + " and width=" + width + ""
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        i=i+1

if __name__ == '__main__':
    try:
        talker()
        print('hello')
    except rospy.ROSInterruptException:
        pass




################################################33
# bridge=CvBridge()
# ######################################
# def talker():
#     pub=rospy.Publisher('/webcam',String,queue_size=1)
#     rospy.init_node('image',anonymous=True)
#     rate=rospy.Rate(10)
#     while not rospy.is_shutdown():
#         ret, frame=cap.read()
#         if not ret:
#             break
#         msg =bridge.cv2_to_imgmsg(frame,"bgr8")
#         #pub.publish(msg)
#         #pub.publish(hight,width)
#         pub.publish(hight)
#         #pub.publish(width)

#         if 0xFF == ord('q'):
#             break
#         if rospy.is_shutdown():
#             cap.release()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass
##################################################3






# print(width)
# print(hight)
# print(type(width))
# print(type(hight))

######################################
#  def pexels():
#     print("the hight of video is ",hight)
#     print("the width of video is ",width)
#     print("the channel of video is ",channel)

# pexels()