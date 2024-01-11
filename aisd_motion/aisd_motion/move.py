import rclpy
from rclpy.node import Node
from aisd_msgs.msg import Hand
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

#from std_msgs.msg import String


class MoveSubscriber(Node):

    def __init__(self):
        super().__init__('move_subscriber')
        # Created a subscription to the 'cmd_hand' topic
        self.subscription = self.create_subscription(
            Hand,
            'cmd_hand',
            self.listener_callback,
            10)
        # image message conversion  (ROS img to openCV img)
        self.br = CvBridge()
        # Created a publisher for Twist messages on the 'cmd_vel' topic 
        self.move_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription  # prevent unused variable warning
    # def lister_callback method invokes when a msg 'cmd_hand' is received
    def listener_callback(self, msg):
        # describes the given angular and linear vel to zero
        angle = 0.0
        linear = 0.0
        # described angular vel on x-axis range for index finger to trace 
        # the movements of turtle left and right if angles are .45 & .55 respectively
        # also print logger msg right and left accordingly
        if msg.xindex > 0.55:
            self.get_logger().info('right')
            angle = -0.1
        elif msg.xindex < 0.45:
            self.get_logger().info('left')
            angle = 0.1
        else:
            angle = 0.0
        # again condition is introduced to trace the turtle action based on linear vel for index
        # and pinky finger, also prints the logger msg of the action.
        if msg.xindex > msg.xpinky:
            self.get_logger().info('come')
            linear = 0.5
        else:
            self.get_logger().info('stay')
            linear = 0.0
        # created a twist message for the cal linear and angular vel
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angle
        # condition statement is introduced to check  for subscriber to cmd_vel topic,
        # also prints twist msg.
        if self.move_publisher.get_subscription_count() > 0:
            self.move_publisher.publish(twist)
        else:
            self.get_logger().info('waiting for subcriber')


def main(args=None):
    rclpy.init(args=args)

    move_subscriber = MoveSubscriber()

    rclpy.spin(move_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
