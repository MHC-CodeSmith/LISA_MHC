#!/usr/bin/python3
# Basics ROS program to subscribe to real-time streaming video frames,
# detect faces, and publish them to the /Face topic.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def find_face(frame):
    # Load the cascade for face detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
     
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
     
    # Detect faces
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
     
    # If at least one face is detected, return the first face's region of interest
    for (x, y, w, h) in faces:
        return frame[y:y+h, x:x+w]

    # Return the original frame if no faces are detected
    return frame

def callback(data):
    br = CvBridge()
    rospy.loginfo('Encontrando Rostos')
    current_frame = br.imgmsg_to_cv2(data)
    face_frame = find_face(current_frame)
    
    if face_frame is not None:
        # Convert the face frame back to a ROS Image message
        face_msg = br.cv2_to_imgmsg(face_frame)
        pub.publish(face_msg)

def publish_faces():
    global pub
    pub = rospy.Publisher('/Rostos', Image, queue_size=10)
    rospy.init_node('encontrar_rostos_pub_py', anonymous=True)
    rospy.Subscriber('/Imagens', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_faces()
    except rospy.ROSInterruptException:
        pass
