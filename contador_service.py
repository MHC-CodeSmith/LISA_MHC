#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from estrutura.srv import CountFingers, CountFingersResponse

bridge = CvBridge()
mp_hands = mp.solutions.hands

class FingerCounter:
    def __init__(self, interval=0.7):
        self.image = None
        self.processing = False
        self.latest_finger_count = 0
        rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.service = rospy.Service('count_fingers', CountFingers, self.handle_count_fingers)
        rospy.loginfo("Serviço de contagem de dedos iniciado.")
        self.timer = rospy.Timer(rospy.Duration(interval), self.process_image)

    def image_callback(self, msg):
        self.image = msg

    def process_image(self, event):
        if self.image is not None and not self.processing:
            self.processing = True
            try:
                # Converte a imagem ROS para OpenCV
                frame = bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Processa a imagem para contar os dedos
                with mp_hands.Hands(static_image_mode=True, model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
                    results = hands.process(frame_rgb)
                    finger_count = 0
                    if results.multi_hand_landmarks:
                        for hand_landmarks in results.multi_hand_landmarks:
                            hand_landmarks_list = [[lm.x, lm.y] for lm in hand_landmarks.landmark]

                            # Check fingers: TIP y position must be lower than PIP y position
                            if hand_landmarks_list[8][1] < hand_landmarks_list[6][1]:       # Index finger
                                finger_count += 1
                            if hand_landmarks_list[12][1] < hand_landmarks_list[10][1]:     # Middle finger
                                finger_count += 1
                            if hand_landmarks_list[16][1] < hand_landmarks_list[14][1]:     # Ring finger
                                finger_count += 1
                            if hand_landmarks_list[20][1] < hand_landmarks_list[18][1]:     # Pinky
                                finger_count += 1
                self.latest_finger_count = finger_count
                rospy.loginfo(f"Contador de dedos: {finger_count}")
            except Exception as e:
                rospy.logerr(f"Erro ao processar imagem: {e}")
            finally:
                self.processing = False

    def handle_count_fingers(self, req):
        if self.latest_finger_count is None:
            rospy.logerr("Nenhuma contagem de dedos disponível ainda.")
            return CountFingersResponse(count=0)
        return CountFingersResponse(count=self.latest_finger_count)

if __name__ == "__main__":
    rospy.init_node('count_fingers_server')
    FingerCounter(interval=2.0)  # Ajuste o intervalo conforme necessário
    rospy.spin()
