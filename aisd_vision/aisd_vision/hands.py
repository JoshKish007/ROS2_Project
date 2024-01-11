import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from aisd_msgs.msg import Hand

mp_hands = mp.solutions.hands

class Hands(Node):

    def __init__(self):
        # constructor of the parent class Hands with the name hands_node
        super().__init__('hands_node')
        # CVBridge, convert between ROS Image messages and OpenCV images.
        self.br = CvBridge()
        # Created a publisher for Hand messages on the topic 'cmd_hand'
        self.hand_publisher = self.create_publisher(Hand, 'cmd_hand', 10)
        # Creatde a subscriber for Image messages on the topic 'video_frames'
        # we also call back the fuction here named 'lister_callback'
        self.image_subscriber = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10
        )
    # lister_callback method is defined here and callback function automatically 
    # called when an image is received, responsible for processing image mesaage.
    def listener_callback(self, msg):
        try:
            # prints output message if receives an image
            self.get_logger().info('Received an image.')
            image = self.br.imgmsg_to_cv2(msg)
            # here given the x-axis range values for index and pinky finger to identify the hand 
            PINKY_FINGER_TIP = 20
            INDEX_FINGER_TIP = 8
            #Analyse the image for hands
            with mp_hands.Hands(
                    model_complexity=0,
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5) as myhands:
                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                # converts image to RGB format 
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                # detects hand and process further
                results = myhands.process(image)
                if results.multi_hand_landmarks:
                    #publisg the hand position in term of index finger and pinky
                    hand_msg = Hand()
                    hand_msg.xpinky = results.multi_hand_landmarks[0].landmark[PINKY_FINGER_TIP].x
                    hand_msg.xindex = results.multi_hand_landmarks[0].landmark[INDEX_FINGER_TIP].x
                    # uses if condition to check subscriber to the hand_publisher, and
                    # prints hand message
                    if self.hand_publisher.get_subscription_count()> 0:
                        self.hand_publisher.publish(hand_msg)
                        self.get_logger().info('Hand message published.')
                    else:
                        self.get_logger().info('waiting for subcriber')
                else:
                    # prints message if hand is not detected
                    self.get_logger().info('No hands detected.')
        except Exception as e:
            # prints error message if an exception occurs(useful as we using try clause in this method, 
            # also prevent the program from crashing.
            self.get_logger().error(f'Error in listener_callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    # creates an instance of Hand class, represents node for hand detection
    hands_node = Hands()
    try:
        rclpy.spin(hands_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    except KeyboardInterrupt:
        pass
    finally:
        hands_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
