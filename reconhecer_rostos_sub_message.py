#!/usr/bin/python3
# Description:
# - Subscribes to real-time streaming video of detected faces.
# - Displays only detected faces and prints face coordinates.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rospy
from cv_bridge import CvBridge
import cv2
from webcam.msg import FaceWithCoords  # Import the custom message

def callback(data):
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("Recebendo Rostos")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data.face_image, "bgr8")
   
    # Display image
    cv2.imshow("Detected Faces", current_frame)
    # Print coordinates
    print("Center coordinates: ({}, {})".format(data.center_x, data.center_y))
    
    # Wait for 1 millisecond to see if the user presses a key to close
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested shutdown.")
      
def receive_message():
    # Initialize the ROS node
    rospy.init_node('faces_sub_py', anonymous=True)
   
    # Subscribe to the /Rostos topic
    rospy.Subscriber('/Rostos', FaceWithCoords, callback)
 
    # Keep the node running
    rospy.spin()
 
    # Close down the video stream when done
    cv2.destroyAllWindows()
  
if __name__ == '__main__':
    receive_message()

