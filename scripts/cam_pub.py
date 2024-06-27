#!/usr/bin/python3

import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
  
def publish_message():
 
  # Node is publishing to the /Imagens topic using 
  # the message type Image
  pub = rospy.Publisher('/Imagens', Image, queue_size=10)
     
  # Tells rospy the name of the node.
  rospy.init_node('Receber_Imagem', anonymous=False)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz
     
  # Create a VideoCapture object
  cap = cv2.VideoCapture(0)
     
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
   # Desired resolution
  desired_width = 320
  desired_height = 240
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      ret, frame = cap.read()
         
      if ret == True:
        resized_frame = cv2.resize(frame, (desired_width, desired_height))
             
        # Publish the image.
        pub.publish(br.cv2_to_imgmsg(frame))
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
