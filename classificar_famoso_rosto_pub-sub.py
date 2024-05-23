#!/usr/bin/env python3

import rospy
import cv2
import face_recognition as fr
import os
import glob
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from webcam.msg import FaceWithCoords
import time

class FaceRecognitionNode:
    def _init_(self):
        self.bridge = CvBridge()
        self.imagem_original = None
        self.pasta_imagens = '/home/murilo/lisa/lisa_desktop/Imagens'
        self.famoso = None
        self.encoded_images = self.preload_images()
        self.last_detection_time = 0
        self.detection_interval = 5  # segundos
        self.best_match = None
        self.best_distance = float('inf')
        
        self.image_sub = rospy.Subscriber('/Rostos', FaceWithCoords, self.image_callback)
        self.famoso_pub = rospy.Publisher('/imagem_famosa', String, queue_size=10)

    def preload_images(self):
        imagens = glob.glob(os.path.join(self.pasta_imagens, '.jpg')) + glob.glob(os.path.join(self.pasta_imagens, '.png'))
        encoded_images = []
        for caminho in imagens:
            imagem2 = fr.load_image_file(caminho)
            imagem2 = cv2.cvtColor(imagem2, cv2.COLOR_BGR2RGB)
            face_encodings = fr.face_encodings(imagem2)
            
            if face_encodings:  # Verifica se há pelo menos um rosto na imagem
                encoded_images.append((caminho, face_encodings[0]))
        return encoded_images

    def image_callback(self, msg):
        try:
            current_time = time.time()
            if current_time - self.last_detection_time < self.detection_interval:
                return
            
            frame = self.bridge.imgmsg_to_cv2(msg.face_image, 'bgr8')
            if frame is None or frame.size == 0:
                rospy.logwarn("Imagem recebida está vazia")
                return

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            face_encodings = fr.face_encodings(frame)
            if face_encodings:
                self.imagem_original = face_encodings[0]
                self.compare_images()
                self.last_detection_time = current_time
            else:
                rospy.loginfo("Nenhum rosto detectado na imagem recebida.")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        except Exception as e:
            rospy.logerr("Erro ao processar a imagem: {0}".format(e))

    def compare_images(self):
        if self.imagem_original is None:
            return

        for caminho, encoding in self.encoded_images:
            distancia = fr.face_distance([self.imagem_original], encoding)[0]

            if distancia < self.best_distance:
                self.best_match = caminho
                self.best_distance = distancia

        if self.best_match:
            self.famoso_pub.publish(String(self.best_match))
            rospy.loginfo(f"Imagem mais próxima: {self.best_match}, Distância: {self.best_distance}")

if _name_ == '_main_':
    rospy.init_node('face_recognition_node', anonymous=True)
    node = FaceRecognitionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down face recognition node")
