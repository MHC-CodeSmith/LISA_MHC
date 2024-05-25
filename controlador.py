#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from estrutura.srv import CountFingers, CountFingersRequest, RecognizeGesture, RecognizeGestureRequest, RecognizeFace, RecognizeFaceRequest

bridge = CvBridge()

class Controller:
    def __init__(self):
        self.image = None
        rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.finger_count_pub = rospy.Publisher('/resultado_dedos', Int32, queue_size=10)
        self.gesture_pub = rospy.Publisher('/resultado_gesto', Int32, queue_size=10)
        self.face_pub = rospy.Publisher('/resultado_rosto', Image, queue_size=10)
        rospy.loginfo("Nó controlador iniciado.")
        rospy.Timer(rospy.Duration(0.7), self.process_image)

    def image_callback(self, msg):
        self.image = msg

    def process_image(self, event):
        if self.image is not None:
            try:
                # Contagem de Dedos
                rospy.wait_for_service('count_fingers')
                count_fingers = rospy.ServiceProxy('count_fingers', CountFingers)
                count_request = CountFingersRequest(image=self.image)
                count_response = count_fingers(count_request)
                rospy.loginfo(f"Número de dedos: {count_response.count}")
                self.finger_count_pub.publish(count_response.count)
                
                # Verifica o número de dedos e chama o serviço apropriado
                if count_response.count == 3:
                    self.process_gesture()
                elif count_response.count == 4:
                    self.process_face()
            except rospy.ServiceException as e:
                rospy.logerr(f"Erro ao chamar serviço: {e}")

    def process_gesture(self):
        if self.image is not None:
            try:
                # Reconhecimento de Gestos
                rospy.wait_for_service('recognize_gesture')
                recognize_gesture = rospy.ServiceProxy('recognize_gesture', RecognizeGesture)
                gesture_request = RecognizeGestureRequest(image=self.image)
                gesture_response = recognize_gesture(gesture_request)
                rospy.loginfo(f"Gesto reconhecido: {gesture_response.gesture}")
                self.gesture_pub.publish(gesture_response.gesture)
            except rospy.ServiceException as e:
                rospy.logerr(f"Erro ao chamar serviço: {e}")

    def process_face(self):
        if self.image is not None:
            try:
                # Reconhecimento de Rosto
                rospy.wait_for_service('recognize_face')
                recognize_face = rospy.ServiceProxy('recognize_face', RecognizeFace)
                face_request = RecognizeFaceRequest(image=self.image)
                face_response = recognize_face(face_request)
                if face_response.face:
                    rospy.loginfo("Rosto reconhecido e publicado.")
                    self.face_pub.publish(face_response.face)
                else:
                    rospy.loginfo("Nenhum rosto reconhecido.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Erro ao chamar serviço: {e}")

if __name__ == "__main__":
    rospy.init_node('controlador', anonymous=True)
    controller = Controller()
    rospy.spin()
