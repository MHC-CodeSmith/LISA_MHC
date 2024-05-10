#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from std_msgs.msg import Int32

def detect_hand_gesture(frame):
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5)
    mp_drawing = mp.solutions.drawing_utils

    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(image)

    gesture = None
    if results.multi_hand_landmarks:
        # Your gesture recognition logic here
        # For example, if a specific hand landmark is detected, set gesture to 1
        gesture = 1

    return gesture

def image_callback(msg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    gesture = detect_hand_gesture(frame)
    if gesture is not None:
        rospy.loginfo("Gesto reconhecido: {}".format(gesture))
        gesture_pub.publish(gesture)

def gesture_recognition():
    rospy.init_node('reconhecimento_gesto_nodo', anonymous=True)
    rospy.Subscriber('/imagens', Image, image_callback)
    global gesture_pub
    gesture_pub = rospy.Publisher('/gestos', Int32, queue_size=10)
    rospy.spin()

if _name_ == '_main_':
    try:
        gesture_recognition()
    except rospy.ROSInterruptException:
        pass
