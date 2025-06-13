#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedCircleAvoider(Node):
    def __init__(self):
        super().__init__('red_circle_avoider')
        self.bridge = CvBridge()

        self.thrust_L_pub = self.create_publisher(Float64, '/box_usv/thruster_L', 10)
        self.thrust_R_pub = self.create_publisher(Float64, '/box_usv/thruster_R', 10)

        self.create_subscription(
            Image,
            '/world/my_world/model/box_usv/link/camera_link/sensor/camera_sensor/image',
            self.image_callback,
            10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.control_loop(cv_image)

    def control_loop(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        msg_L = Float64()
        msg_R = Float64()
        width = image.shape[1]

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            M = cv2.moments(largest)
            cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else width // 2
            if area > 200:
                if cx < width * 0.4:
                    msg_L.data = 3.0
                    msg_R.data = 6.0
                elif cx > width * 0.6:
                    msg_L.data = 6.0
                    msg_R.data = 3.0
                else:
                    msg_L.data = -3.0
                    msg_R.data = 3.0
            else:
                msg_L.data = 5.0
                msg_R.data = 5.0
        else:
            msg_L.data = 5.0
            msg_R.data = 5.0

        self.thrust_L_pub.publish(msg_L)
        self.thrust_R_pub.publish(msg_R)


def main(args=None):
    rclpy.init(args=args)
    node = RedCircleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
