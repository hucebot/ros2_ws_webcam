#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2



class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        # Declare ROS2 parameters with default values
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('cam_frame_id', '/cam')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('framerate', 30)

        # Get parameter values
        self.device = self.get_parameter('video_device').get_parameter_value().string_value
        self.frame_id = self.get_parameter('cam_frame_id').get_parameter_value().string_value
        self.width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('framerate').get_parameter_value().integer_value

        # Publisher (compressed images)
        topic_name = '~/image_raw/compressed' 
        self.pub = self.create_publisher(CompressedImage, topic_name, 10)

        # Setup webcam
        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.bridge = CvBridge()

        # sanity check
        if self.cap.isOpened():
            self.get_logger().info(f'[OK] Device correctly opens: {self.device}')
        else:
            self.get_logger().info(f'[ERROR] Device not open: {self.device}')
        
        # Timer according to framerate
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    def timer_callback(self):

        ret, frame = self.cap.read()

        if ret:
            msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='png')
            # msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

