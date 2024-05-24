#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from std_msgs.msg import Int32

# Baixar o modelo de reconhecimento de gestos (execute uma vez antes de rodar o script)
# !wget -q https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task

# Configurações do modelo
base_options = python.BaseOptions(model_asset_path='gesture_recognizer.task')
options = vision.GestureRecognizerOptions(base_options=base_options)
recognizer = vision.GestureRecognizer.create_from_options(options)

# Inicializa o CvBridge uma vez
bridge = CvBridge()

# Mapeamento de gestos para inteiros
gesture_mapping = {
    'Thumbs_Up': 1,
    'Thumbs_Down': 2,
    'Open_Palm': 3,
    'Closed_Fist': 4,
    'Victory': 5,
    'Pointing_Up': 6,
}

def detect_hand_gesture(frame):
    # Converte a imagem de BGR para RGB
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)

    # Reconhece gestos na imagem
    recognition_result = recognizer.recognize(mp_image)
    gesture = None

    if recognition_result.gestures:
        # Obtém o gesto principal reconhecido
        top_gesture = recognition_result.gestures[0][0]
        gesture = top_gesture.category_name
        rospy.loginfo("Gesto detectado: {}".format(gesture))
    else:
        rospy.loginfo("Nenhum gesto detectado.")

    return gesture

def image_callback(msg):
    global gesture_pub
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    gesture = detect_hand_gesture(frame)
    if gesture in gesture_mapping:
        rospy.loginfo("Gesto reconhecido: {}".format(gesture))
        gesture_pub.publish(Int32(data=gesture_mapping[gesture]))
    else:
        rospy.loginfo("Gesto não reconhecido ou não mapeado: {}".format(gesture))

def gesture_recognition():
    global gesture_pub
    rospy.init_node('reconhecimento_gesto_nodo', anonymous=True)
    gesture_pub = rospy.Publisher('/Gestos', Int32, queue_size=10)
    rospy.Subscriber('/Imagens', Image, image_callback)
    rospy.loginfo("Nó de reconhecimento de gestos iniciado.")
    rospy.spin()

if _name_ == '_main_':
    try:
        gesture_recognition()
    except rospy.ROSInterruptException:
        pass
