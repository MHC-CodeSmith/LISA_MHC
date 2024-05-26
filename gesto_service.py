#!/usr/bin/env python3
import os
os.environ['GLOG_minloglevel'] = '2'  # Suprimir mensagens de log de inicialização do MediaPipe

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
        self.gesture_counts = {}
        rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.service = rospy.Service('recognize_gesture', Trigger, self.handle_recognize_gesture)
        rospy.loginfo("Serviço de reconhecimento de gestos iniciado.")

    def image_callback(self, msg):
        self.image = msg

    def handle_recognize_gesture(self, req):
        response = self.process_image()
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
                    gesture_name = top_gesture.category_name
                    gesture = gesture_mapping.get(gesture_name, 0)
                    self.update_gesture_count(gesture_name)
                else:
                    rospy.loginfo("Nenhum gesto detectado.")
                
                rospy.loginfo(f"Gesto detectado: {gesture}")
                return TriggerResponse(success=True, message=f"Gesto {gesture_name} reconhecido {self.gesture_counts[gesture_name]} vezes" if gesture else "Nenhum gesto reconhecido")
            except Exception as e:
                rospy.logerr(f"Erro ao processar imagem: {e}")
                return TriggerResponse(success=False, message=f"Erro ao processar imagem: {e}")
        else:
            return TriggerResponse(success=False, message="Nenhuma imagem recebida")

    def update_gesture_count(self, gesture_name):
        if gesture_name not in self.gesture_counts:
            self.gesture_counts[gesture_name] = 0
        self.gesture_counts[gesture_name] += 1

        if self.gesture_counts[gesture_name] >= 5:
            self.gesture_counts = {}  # Reset counter
            rospy.set_param('/stop_counting', False)  # Enable finger counting again
            rospy.loginfo(f"Gesto {gesture_name} reconhecido 5 vezes seguidas")

if __name__ == "__main__":
    rospy.init_node('recognize_gesture_server')
    GestureRecognizer()
    rospy.spin()
