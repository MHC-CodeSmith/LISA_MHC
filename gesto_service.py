#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

model_path = '/home/mhc/SEMEAR/lisa/lisa_ws/src/estrutura/gesture_recognizer.task'
base_options = python.BaseOptions(model_asset_path=model_path)
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)

bridge = CvBridge()
gesture_mapping = {
    'Thumbs_Up': 1,
    'Thumbs_Down': 2,
    'Open_Palm': 3,
    'Closed_Fist': 4,
    'Victory': 5,
    'Pointing_Up': 6,
}

class GestureRecognizer:
    def __init__(self):
        self.image = None
        self.latest_gesture = 0
        rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.service = rospy.Service('recognize_gesture', Trigger, self.handle_recognize_gesture)
        rospy.loginfo("Serviço de reconhecimento de gestos iniciado.")

    def image_callback(self, msg):
        self.image = msg

    def handle_recognize_gesture(self, req):
        self.process_image()
        response = TriggerResponse()
        response.success = True
        response.message = str(self.latest_gesture)
        return response

    def process_image(self):
        if self.image is not None:
            try:
                frame = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
                recognition_result = recognizer.recognize(mp_image)
                gesture = 0
                if recognition_result.gestures:
                    top_gesture = recognition_result.gestures[0][0]
                    gesture = gesture_mapping.get(top_gesture.category_name, 0)
                self.latest_gesture = gesture
                rospy.loginfo(f"Gesto detectado: {gesture}")
            except Exception as e:
                rospy.logerr(f"Erro ao processar imagem: {e}")

if __name__ == "__main__":
    rospy.init_node('recognize_gesture_server')
    GestureRecognizer()
    rospy.spin()
