#!/usr/bin/python3

# para saber qual comando vc deve completar depois de #! ali em cima
# (ou seja, para setar seu processador)
# tem que colocar no terminal which python3
# copiar e adicionar logo em seguida

import rospy # biblioteca de python para ros
from sensor_msgs.msg import Image # o tipo da mensagem a ser enviada é do tipo imagem
from cv_bridge import CvBridge # pacote para converter ros para imagem open cv
import cv2 # biblioteca do open cv
 
def callback(data):
 
  # usado para converter ros para imagem open cv -- "ponte" entre os dois tipos
  br = CvBridge()
 
  # output para vermos se está tendo algum erro no envio
  rospy.loginfo("receiving video frame")
   
  # Converte imagem ros para imagem opencv
  current_frame = br.imgmsg_to_cv2(data)
   
  # mostrar a imagem com o nome "camera"
  cv2.imshow("camera", current_frame)
   
  cv2.waitKey(1)
      
def receive_message():
 
  # o primeiro argumento é o nome do nó (quem recebe é o nó subscriber)
  # o segundo argumento indica que deve ser um único nó a ser utilizado e com um nome apenas. 
  # Random numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # o nó está atuando como subscriber para o tópico das imagens (frames do vídeo nesse caso)
  rospy.Subscriber('video_frames', Image, callback)
 
  # spin() não deixa que o python pare de rodar antes que o nó pare de rodar
  rospy.spin()
 
  # fecha a guia da câmera quando termina de executar o nó
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
