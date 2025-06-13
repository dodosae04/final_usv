import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class RedCircleAvoider(Node):
    def __init__(self):
        super().__init__('red_circle_avoider')
        self.bridge = CvBridge()

        # 퍼블리셔 설정
        self.thrust_L_pub = self.create_publisher(Float64, '/box_usv/thruster_L', 10)
        self.thrust_R_pub = self.create_publisher(Float64, '/box_usv/thruster_R', 10)
        self.left=0

        # 카메라 이미지 구독
        self.image_sub = self.create_subscription(
            Image,
            '/world/my_world/model/box_usv/link/camera_link/sensor/camera_sensor/image',
            self.image_callback,
            10
        )




        # 최근 이미지 저장
        self.latest_image = None

    def image_callback(self, msg):
        # ROS 이미지 -> OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv_image.copy()


        # OpenCV 창으로 이미지 띄우기
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)
        self.control_loop()

    def control_loop(self):
        msg_L = Float64()
        msg_R = Float64()

        msg_L.data = 5.0
        msg_R.data = 5.0

        self.thrust_L_pub.publish(msg_L)
        self.thrust_R_pub.publish(msg_R)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RedCircleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
