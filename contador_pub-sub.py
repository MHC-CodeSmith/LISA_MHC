#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32
import cv2
import mediapipe as mp
from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

def find_hands_and_fingers(img, draw=True):
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    hand_info = []

    if results.multi_hand_landmarks:
        for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
            fingers = [lm.y < hand_landmarks.landmark[id - 2].y for id, lm in enumerate(hand_landmarks.landmark) if id % 4 == 0 and id > 0]
            hand_type = "Right" if handedness.classification[0].label == "Right" else "Left"
            hand_info.append((hand_type, fingers))
            if draw:
                mp.solutions.drawing_utils.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    return img, hand_info

def publisher(finger_count):
    pub = rospy.Publisher('finger_count', Int32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing: %d", finger_count[0])
        pub.publish(finger_count[0])
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
    img, hands_info = find_hands_and_fingers(cv_image)
    finger_count[0] = sum(finger.count(True) for _, finger in hands_info)
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == 27:
        rospy.signal_shutdown("ESC pressed")

if _name_ == '_main_':
    rospy.init_node('finger_publisher', anonymous=True)
    finger_count = [0]
    Thread(target=publisher, args=(finger_count,)).start()
    image_subscriber = rospy.Subscriber('/Imagens', Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()
