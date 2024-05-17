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
import mediapipe as mp
from webcam.msg import FaceWithCoords

# Initialize MediaPipe Face Detection
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

def find_face(frame):
    # Convert frame to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Process the image and detect faces
    results = face_detection.process(rgb_frame)
    
    if results.detections:
        for detection in results.detections:
            # Get the bounding box of the first detected face
            bboxC = detection.location_data.relative_bounding_box
            ih, iw, _ = frame.shape
            bbox = (int(bboxC.xmin * iw), int(bboxC.ymin * ih),
                    int(bboxC.width * iw), int(bboxC.height * ih))
            # Return the face region and the bbox
            return frame[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]], bbox
    
    # Return the original frame and None if no faces are detected
    return frame, None

def callback(data):
    br = CvBridge()
    rospy.loginfo('Encontrando Rostos')
    current_frame = br.imgmsg_to_cv2(data)
    face_frame, bbox = find_face(current_frame)
    
    if face_frame is not None and bbox is not None:
        # Calcula o centro do rosto
        center_x = (bbox[0] + bbox[2] / 2) / current_frame.shape[1]
        center_y = (bbox[1] + bbox[3] / 2) / current_frame.shape[0]
        
        # Prepara a mensagem
        face_msg = FaceWithCoords()
        face_msg.face_image = br.cv2_to_imgmsg(face_frame, encoding="bgr8")
        face_msg.center_x = center_x
        face_msg.center_y = center_y
        
        pub.publish(face_msg)

def publish_faces():
    global pub
    pub = rospy.Publisher('/Rostos', FaceWithCoords, queue_size=10)
    rospy.init_node('encontrar_rostos_pub_py', anonymous=True)
    rospy.Subscriber('/Imagens', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_faces()
    except rospy.ROSInterruptException:
        pass
