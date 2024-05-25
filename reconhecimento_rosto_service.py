#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from estrutura.srv import RecognizeFace, RecognizeFaceResponse
import cv2
import mediapipe as mp

mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)
bridge = CvBridge()

class FaceRecognizer:
    def __init__(self):
        self.image = None
        self.processing = False
        self.latest_face = None
        rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.service = rospy.Service('recognize_face', RecognizeFace, self.handle_recognize_face)
        rospy.loginfo("Serviço de reconhecimento de rosto iniciado.")
        cv2.namedWindow("Face Detection", cv2.WINDOW_NORMAL)  # Create the window once

    def image_callback(self, msg):
        self.image = msg
        self.process_image()

    def process_image(self):
        if self.image is not None and not self.processing:
            self.processing = True
            try:
                frame = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                results = face_detection.process(frame_rgb)
                if results.detections:
                    for detection in results.detections:
                        bboxC = detection.location_data.relative_bounding_box
                        ih, iw, _ = frame.shape
                        bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), int(bboxC.width * iw), int(bboxC.height * ih)
                        face = frame[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
                        self.latest_face = face
                        rospy.loginfo("Rosto detectado.")
                        
                        # Display the face in a window
                        cv2.imshow("Face Detection", face)
                        cv2.waitKey(1)
                        return
                self.latest_face = None
            except Exception as e:
                rospy.logerr(f"Erro ao processar imagem: {e}")
            finally:
                self.processing = False

    def handle_recognize_face(self, req):
        if self.latest_face is None:
            rospy.logerr("Nenhum rosto disponível ainda.")
            return RecognizeFaceResponse()
        face_msg = bridge.cv2_to_imgmsg(self.latest_face, encoding="bgr8")
        return RecognizeFaceResponse(face=face_msg)

if __name__ == "__main__":
    rospy.init_node('recognize_face_server')
    recognizer = FaceRecognizer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        cv2.destroyAllWindows()  # Destroy the window upon shutdown
