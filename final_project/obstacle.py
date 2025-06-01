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

        # 카메라 이미지 구독
        self.image_sub = self.create_subscription(
            Image,
            '/world/my_world/model/box_usv/link/camera_link/sensor/camera_sensor/image',
            self.image_callback,
            10
        )

        # 상태 변수
        self.avoid_mode = False
        self.avoid_start_time = None

        # 타이머 (주기적으로 추진값 publish)
        self.timer = self.create_timer(0.1, self.control_loop)

        # 최근 이미지 저장
        self.latest_image = None

    def image_callback(self, msg):
        # ROS 이미지 -> OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv_image.copy()

        # HSV 변환
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 빨간색 범위 설정 (두 구간 필요)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # 윤곽선 검출
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                detected = True

        if detected and not self.avoid_mode:
            self.get_logger().info("Red object detected! Entering avoidance mode.")
            self.avoid_mode = True
            self.avoid_start_time = time.time()

        # OpenCV 창으로 이미지 띄우기
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)

    def control_loop(self):
        msg_L = Float64()
        msg_R = Float64()

        if self.avoid_mode:
            # 회피 시간 동안 선회
            if time.time() - self.avoid_start_time < 1.5:
                msg_L.data = -5.0  # 좌회전
                msg_R.data = 5.0
            elif time.time() - self.avoid_start_time < 2.5:
                msg_L.data =  10.0  # 좌회전
                msg_R.data = 0.0
            else:
                self.avoid_mode = False
                msg_L.data = 5.0
                msg_R.data = 5.0
                self.get_logger().info("Avoidance complete. Resuming forward motion.")
        else:
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
