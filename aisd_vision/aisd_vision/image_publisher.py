#!/usr/bin/env python3
#!/home/aisd/ros2_ws/install/aisd_vision/lib/python3.10/site-packages/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
# created a class ImagePublisher 
class ImagePublisher(Node):

    def __init__(self):
        # call constructor of parent class ImagePublisher(Node) with name 'image_publisher'
        super().__init__('image_publisher')
        # created a publisher for Img msgs on the topic 'video_frames'
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        timer_period = 0.2  # seconds (seconds changed to 0.2 for my convinience
        # initializes self timer for the range of given time_period
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Open video capture device in my case is used device name,
        #  instead of device index  "dev/video0"
        self.cap = cv2.VideoCapture("/dev/video0")
        self.br = CvBridge()

    # def timer_callback method acts as a callback fuction for the image that can be  published
    def timer_callback(self):
        # reads frame from video capture device "/dev/video0"
        ret, frame = self.cap.read()
        # condition is introduced for successful capture, if the statement is "True", 
        # then it converts the captured img to ROS img message and publish on video_frames topic.
        if ret == True:
            img_msg = self.br.cv2_to_imgmsg(frame)
            self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    # creates instance for the class
    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
