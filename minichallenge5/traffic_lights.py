import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data

class TrafficLights(Node):
    def __init__(self):
        super().__init__('traffic_lights')
        
        self.img = np.ndarray((720, 1280, 3))
        # Valores de los colores
        self.color_ranges = {
            "stop": (np.array([0, 100, 100]), np.array([10, 255, 255])),
            "slow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
            "move": (np.array([40, 40, 40]), np.array([80, 255, 255]))
        }

        self.valid_img = False
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, qos_profile_sensor_data)
        self.sub_vel = self.create_subscription(Twist, 'cmd_vel_aux', self.vel_cb, 10)
        self.pub = self.create_publisher(Image, '/img_processing/color', 10)
        self.pub_status = self.create_publisher(String, '/status', 10)
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('CV Node started')

        self.status = String()
        self.vel = Twist()

    def count_color_in_circles(self, hsv_img, circles):
        mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv2.circle(mask, (x, y), r, 255, 2)
        
        color_counts = {color: cv2.countNonZero(cv2.bitwise_and(mask, cv2.inRange(hsv_img, lower, upper)))
                        for color, (lower, upper) in self.color_ranges.items()}

        return max(color_counts, key=color_counts.get)



    def process_color(self, mask):
        detected_output = cv2.bitwise_and(self.img, self.img, mask=mask)
        gray = cv2.cvtColor(detected_output, cv2.COLOR_BGR2GRAY)
        blur = cv2.medianBlur(gray, 5)

        # Detectar círculos usando la transformación de Hough
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30, param1=50, param2=30, minRadius=5, maxRadius=100)
        
        return circles

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.valid_img = True
        except:
            self.get_logger().info('Failed to get an image')

    def vel_cb(self, msg):
        try:
            self.vel = msg
        except:
            self.get_logger().info('Failed to get velocity')

    def timer_callback(self):
        try:
            if self.valid_img:
                hsvFrame = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)

                # Detectar círculos en la imagen
                circles = self.process_color(np.ones_like(hsvFrame[:,:,0], dtype=np.uint8) * 255)
                rgb = self.count_color_in_circles(hsvFrame, circles)

                if rgb == 'move':
                    self.pub_vel.publish(self.vel)

                elif rgb == 'slow':
                    self.vel.linear.x = self.vel.linear.x / 2.0
                    self.vel.angular.z = self.vel.angular.z / 2.0 
                    self.pub_vel.publish(self.vel)
                    self.vel.linear.x = self.vel.linear.x * 2.0
                    self.vel.angular.z = self.vel.angular.z * 2.0

                elif rgb == 'stop':
                    # Detener el robot
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.0
                    self.pub_vel.publish(self.vel)

                # Crear una máscara de los círculos detectados para visualización
                circle_mask = np.zeros_like(hsvFrame[:,:,0])
                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    for (x, y, r) in circles:
                        cv2.circle(circle_mask, (x, y), r, 255, 2)

                self.status.data = rgb
                self.pub_status.publish(self.status)
                self.pub.publish(self.bridge.cv2_to_imgmsg(circle_mask))
                self.valid_img = False
        except:
            self.get_logger().info('Failed to process image')

def main(args=None):
    rclpy.init(args=args)
    cv_e = TrafficLights()
    rclpy.spin(cv_e)
    cv_e.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
