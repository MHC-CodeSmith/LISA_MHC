#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

# Inicializa o MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5)

def find_hands_and_fingers(img, draw=True):
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    hand_info = []

    if results.multi_hand_landmarks:
        for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
            fingers = []
            for id, lm in enumerate(hand_landmarks.landmark):
                if id % 4 == 0 and id > 0:
                    fingers.append(lm.y < hand_landmarks.landmark[id - 2].y)
            hand_type = "Right" if handedness.classification[0].label == "Right" else "Left"
            hand_info.append((hand_type, fingers))

            if draw:
                mp.solutions.drawing_utils.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    return img, hand_info

def image_callback(msg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    _, hand_info = find_hands_and_fingers(frame)

    for hand in hand_info:
        hand_type, fingers = hand
        rospy.loginfo(f"{hand_type} hand with fingers: {len([f for f in fingers if f])} raised")
        finger_count_pub.publish(len([f for f in fingers if f]))

def gesture_recognition_node():
    rospy.init_node('gesture_recognition_node', anonymous=True)
    rospy.Subscriber('/Imagens', Image, image_callback)
    global finger_count_pub
    finger_count_pub = rospy.Publisher('finger_count', Int32, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        gesture_recognition_node()
    except rospy.ROSInterruptException:
        pass
