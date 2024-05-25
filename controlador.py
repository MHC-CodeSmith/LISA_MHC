#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger
from std_msgs.msg import String

class Controlador:
    def __init__(self):
        self.result_pub = rospy.Publisher('resultados', String, queue_size=10)
        rospy.wait_for_service('count_fingers')
        rospy.wait_for_service('recognize_gesture')
        rospy.wait_for_service('recognize_face')
        self.count_fingers_srv = rospy.ServiceProxy('count_fingers', Trigger)
        self.recognize_gesture_srv = rospy.ServiceProxy('recognize_gesture', Trigger)
        self.recognize_face_srv = rospy.ServiceProxy('recognize_face', Trigger)
        rospy.Timer(rospy.Duration(1), self.control_loop)

    def control_loop(self, event):
        try:
            finger_count_resp = self.count_fingers_srv()
            if finger_count_resp.success:
                finger_count = int(finger_count_resp.message)
                if finger_count == 3:
                    gesture_resp = self.recognize_gesture_srv()
                    if gesture_resp.success:
                        self.result_pub.publish(f"Gesto reconhecido: {gesture_resp.message}")
                elif finger_count == 4:
                    face_resp = self.recognize_face_srv()
                    if face_resp.success:
                        rospy.loginfo(face_resp.message)
                else:
                    self.result_pub.publish(f"Numero de dedos: {finger_count}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Erro ao chamar serviço: {e}")

if __name__ == "__main__":
    rospy.init_node('controlador')
    Controlador()
    rospy.spin()
