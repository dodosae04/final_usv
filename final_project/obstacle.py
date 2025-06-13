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




        # 최근 이미지 저장
        self.latest_image = None

    def image_callback(self, msg):
        # ROS 이미지 -> OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv_image.copy()


        # 장애물 회피 로직 실행
        self.control_loop(cv_image)

    def control_loop(self, image):
        msg_L = Float64()
        msg_R = Float64()

        # 이미지에서 빨간색 영역 검출
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        width = image.shape[1]

        # 장애물이 발견되면 위치에 따라 속도를 조절
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            M = cv2.moments(largest)
            cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else width // 2

            if area > 200:
                if cx < width * 0.4:
                    # 장애물이 왼쪽에 있을 때 우회전
                    msg_L.data = 3.0
                    msg_R.data = 6.0
                elif cx > width * 0.6:
                    # 장애물이 오른쪽에 있을 때 좌회전
                    msg_L.data = 6.0
                    msg_R.data = 3.0
                else:
                    # 정면에 있을 때 제자리 회전
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
