#!/usr/bin/python3
import rospy
import os
from std_msgs.msg import Int32
import cv2
import mediapipe as mp
from threading import Thread, Lock
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

os.environ['DISPLAY'] = ':0.0'

# Inicializar o Mediapipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

def find_hands_and_fingers(img, draw=True):
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    hand_info = []

    if results.multi_hand_landmarks:
        for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
            # Dedos abertos são identificados pela posição y das landmarks
            finger_tips = [4, 8, 12, 16, 20]
            fingers = []

            # Verifica se o polegar está levantado
            if hand_landmarks.landmark[finger_tips[0]].x < hand_landmarks.landmark[finger_tips[0] - 2].x:
                fingers.append(True)
            else:
                fingers.append(False)

            # Verifica os outros dedos (índice, médio, anelar e mínimo)
            for tip in finger_tips[1:]:
                if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
                    fingers.append(True)
                else:
                    fingers.append(False)

            hand_type = handedness.classification[0].label
            hand_info.append((hand_type, fingers))
            if draw:
                mp_drawing.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    return img, hand_info

def publisher(finger_count):
    pub = rospy.Publisher('finger_count', Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        with finger_count_lock:
            count = finger_count[0]
        rospy.loginfo("Publishing: %d", count)
        pub.publish(count)
        rate.sleep()

def image_callback(msg):
    br = CvBridge()
    try:
        # Usando 'passthrough' para evitar problemas de codificação
        cv_image = br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr("Failed to convert image: %s", e)
        return

    cv_image = cv2.flip(cv_image, 1)
    _, hands_info = find_hands_and_fingers(cv_image)
    with finger_count_lock:
        # Somar a quantidade de dedos verdadeiros (abertos)
        finger_count[0] = sum(finger.count(True) for _, finger in hands_info)
    
    rospy.loginfo(f"Detected fingers: {finger_count[0]}")

if _name_ == '_main_': 
    rospy.init_node('finger_publisher', anonymous=True)
    finger_count = [0]
    finger_count_lock = Lock()
    thread_publisher = Thread(target=publisher, args=(finger_count,))
    thread_publisher.start()
    image_subscriber = rospy.Subscriber('/Imagens', Image, image_callback)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
