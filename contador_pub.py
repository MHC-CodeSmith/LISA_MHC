#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32
import cv2
import mediapipe as mp


mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

camera = cv2.VideoCapture(0)


def find_hands_and_fingers(img, draw=True):
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    hand_info = []

    if results.multi_hand_landmarks:
        for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
            # Detecta os dedos levantados
            fingers = []
            for id, lm in enumerate(hand_landmarks.landmark):
                if id % 4 == 0 and id > 0:  # Pontas dos dedos têm IDs 4, 8, 12, 16, 20
                    fingers.append(lm.y < hand_landmarks.landmark[id - 2].y)  # Verifica se o dedo está levantado
            # Adiciona informações da mão (tipo e dedos levantados)
            hand_type = "Right" if handedness.classification[0].label == "Right" else "Left"
            hand_info.append((hand_type, fingers))

            if draw:
                mp.solutions.drawing_utils.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    return img, hand_info

def publisher():
    pub = rospy.Publisher('finger_count', Int32, queue_size=10)
    rospy.init_node('finger_publisher', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz
    count = 0
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing: %d", count)
        
        
        
        
        
        
        pub.publish(count)

        rate.sleep()

if __name__ == '_main_':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass




while True:
    success, img = camera.read()
    img = cv2.flip(img, 1)
    
    img, hands_info = find_hands_and_fingers(img)
    
    for hand in hands_info:
        hand_type, fingers = hand
        print(f"{hand_type} hand with fingers: {fingers.count(True)} raised")
    
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == 27:
        break

camera.release()
cv2.destroyAllWindows
