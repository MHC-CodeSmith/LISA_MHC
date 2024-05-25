#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse
import cv2
import mediapipe as mp

bridge = CvBridge()
mp_hands = mp.solutions.hands

class FingerCounter:
    def __init__(self):
        self.image = None
        self.latest_finger_count = 0
        rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.service = rospy.Service('count_fingers', Trigger, self.handle_count_fingers)
        rospy.loginfo("Serviço de contagem de dedos iniciado.")

    def image_callback(self, msg):
        self.image = msg

    def handle_count_fingers(self, req):
        self.process_image()
        response = TriggerResponse()
        response.success = True
        response.message = str(self.latest_finger_count)
        return response

    def process_image(self):
        if self.image is not None:
            try:
                frame = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                with mp_hands.Hands(static_image_mode=True, model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
                    results = hands.process(frame_rgb)
                    finger_count = 0
                    if results.multi_hand_landmarks:
                        for hand_landmarks in results.multi_hand_landmarks:
                            hand_landmarks_list = [[lm.x, lm.y] for lm in hand_landmarks.landmark]
                            if hand_landmarks_list[8][1] < hand_landmarks_list[6][1]:  # Index finger
                                finger_count += 1
                            if hand_landmarks_list[12][1] < hand_landmarks_list[10][1]:  # Middle finger
                                finger_count += 1
                            if hand_landmarks_list[16][1] < hand_landmarks_list[14][1]:  # Ring finger
                                finger_count += 1
                            if hand_landmarks_list[20][1] < hand_landmarks_list[18][1]:  # Pinky
                                finger_count += 1
                self.latest_finger_count = finger_count
                rospy.loginfo(f"Contador de dedos: {finger_count}")
            except Exception as e:
                rospy.logerr(f"Erro ao processar imagem: {e}")

if __name__ == "__main__":
    rospy.init_node('count_fingers_server')
    FingerCounter()
    rospy.spin()
