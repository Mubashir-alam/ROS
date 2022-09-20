import rospy
from sensor_msgs import Image 

def camera():
    pub =rospy.Publisher('picture',Image,queue_size=10)

    rospy.init_node(camera,anonymous=True)

    rate=rospy.Rate(25)  #25 frames per secound 

    i=0   

    while not rospy.is_shutdown():
        pub_img = "I publish an image %s"%i 
        rospy.loginfo(pub_img)
        pub.publish(pub_img)
        rate.sleep()
        i=i+1

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass 