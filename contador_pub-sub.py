#!/usr/bin/python3
import cv2
import mediapipe as mp
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from threading import Timer

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

class HandFingerCounter:
    def _init_(self, interval=2.0):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/Imagens', Image, self.image_callback)
        self.finger_count_pub = rospy.Publisher('/Contador', Int32, queue_size=10)
        self.hands = mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.latest_image = None
        self.processing = False
        self.interval = interval
        self.timer = Timer(self.interval, self.process_image)
        self.timer.start()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_image = image
        except Exception as e:
            rospy.logerr("Error converting image: %s", str(e))

    def process_image(self):
        if self.latest_image is not None and not self.processing:
            self.processing = True
            try:
                # Make a copy of the latest image to process
                image = self.latest_image.copy()

                # To improve performance, optionally mark the image as not writeable to pass by reference.
                image.flags.writeable = False
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = self.hands.process(image_rgb)

                # Draw the hand annotations on the image.
                image.flags.writeable = True
                image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                # Initially set finger count to 0 for each cap
                fingerCount = 0

                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # Set variable to keep landmarks positions (x and y)
                        handLandmarks = []

                        # Fill list with x and y positions of each landmark
                        for landmarks in hand_landmarks.landmark:
                            handLandmarks.append([landmarks.x, landmarks.y])

                        # Check fingers: TIP y position must be lower than PIP y position
                        if handLandmarks[8][1] < handLandmarks[6][1]:       # Index finger
                            fingerCount += 1
                        if handLandmarks[12][1] < handLandmarks[10][1]:     # Middle finger
                            fingerCount += 1
                        if handLandmarks[16][1] < handLandmarks[14][1]:     # Ring finger
                            fingerCount += 1
                        if handLandmarks[20][1] < handLandmarks[18][1]:     # Pinky
                            fingerCount += 1

                        # Draw hand landmarks
                        mp_drawing.draw_landmarks(
                            image_bgr,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style())

                # Publish the finger count
                rospy.loginfo(f"Contador: {fingerCount}")
                self.finger_count_pub.publish(fingerCount)

            except Exception as e:
                rospy.logerr("Error processing image: %s", str(e))
            finally:
                self.processing = False

        self.timer = Timer(self.interval, self.process_image)
        self.timer.start()

def main():
    rospy.init_node('hand_finger_counter', anonymous=True)
    HandFingerCounter(interval=0.7)  # Process every 2 seconds, adjust as needed
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if _name_ == '_main_':
    main()
