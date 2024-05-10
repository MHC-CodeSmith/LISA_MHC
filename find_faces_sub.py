#!/usr/bin/python3
# Description:
# - Subscribes to real-time streaming video of detected faces.
# - Displays only detected faces.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
def callback(data):
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
 
    # Output debugging information to the terminal
    rospy.loginfo("Recebendo Rosto frame")
   
    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)
   
    # Display image
    cv2.imshow("Detected Faces", current_frame)
    # Wait for 1 millisecond to see if the user presses a key to close
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested shutdown.")
      
def receive_message():
    # Initialize the ROS node
    rospy.init_node('faces_sub_py', anonymous=True)
   
    # Subscribe to the /Face topic
    rospy.Subscriber('/Face', Image, callback)
 
    # Keep the node running
    rospy.spin()
 
    # Close down the video stream when done
    cv2.destroyAllWindows()
  
if __name__ == '__main__':
    receive_message()
