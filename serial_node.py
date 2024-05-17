#!/usr/bin/python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import 
import cv2 # OpenCV library
import serial
import time



def callback(data):
  #só até 4 contador
  #1: parar 2: horario 3: antihorario
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("Recebendo video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
   
  # Display image
  cv2.imshow("camera", current_frame)
   
  cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Node name is set to Receber_Imagem
  rospy.init_node('Node_Serial', anonymous=False)
   
  # Node is subscribing to the /Imagens topic
  rospy.Subscriber('finger_count', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  # Substitua '/dev/ttyUSB0' pelo dispositivo correto se for diferente
  ser = serial.Serial('/dev/ttyUSB0', 115200)  # Abre a porta serial com 115200 baud
  
  try:
      #while True:
          #user_command = input("Controle do motor (HORARIO, ANTIHORARIO, PARAR): \n")
          #ser.write(user_command.encode('utf-8') + b'\n')  # Adiciona nova linha e codifica para bytes
      ser.write(msg)    
      time.sleep(1)  # Espera um segundo antes de enviar a próxima mensagem
  except KeyboardInterrupt:
      ser.close()  # Fecha a porta serial ao sair do programa
  
