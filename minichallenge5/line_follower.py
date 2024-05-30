import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollowingRobot(Node):
    def __init__(self):
        super().__init__('line_following_robot')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, 'video_source/raw', self.image_callback, 10)
        self.image_publisher = self.create_publisher(Image, 'processed_image', 10)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.robot_velocity = Twist()
        self.image_received = False
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Line Following Robot Node has started')

        # PID controller parameters
        self.Kp = 0.005
        self.Ki = 0.0
        self.Kd = 0.003

        # PID controller variables
        self.previous_error = 0
        self.integral = 0

        # Integral windup limits
        self.integral_max = 100.0
        self.integral_min = -100.0

        # Speed parameters
        self.max_linear_speed = 0.1  # Reduce max linear speed
        self.min_linear_speed = 0.03  # Reduce min linear speed
        self.max_angular_speed = 0.5  # Reduce max angular speed
        self.min_angular_speed = 0.35  # Reduce min angular speed
        self.turn_slowdown_threshold = 35  # Error threshold to start slowing down

        # Line position history
        self.line_position_history = []

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
        except Exception as e:
            self.get_logger().error('Error converting image: {}'.format(str(e)))

    def timer_callback(self):
        if self.image_received:
            image = self.cv_image.copy()
            height, width, _ = image.shape
            # Definir una región de interés (ROI) central más estrecha
            roi = image[120:240, :]  # Restringir la región de interés horizontalmente
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            _, binary = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY_INV)  # Invertir para que el negro sea blanco

            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Filtrar contornos basados en posición horizontal y área
                valid_contours = [c for c in contours if cv2.contourArea(c) > 100]  # Ajustar umbral de área según sea necesario
                if valid_contours:
                    # Encontrar el contorno más cercano al centro de la imagen
                    roi_center = roi.shape[1] // 2
                    min_distance = float('inf')
                    best_contour = None

                    for contour in valid_contours:
                        M = cv2.moments(contour)
                        if M['m00'] != 0:
                            cx = int(M['m10'] / M['m00'])
                            distance = abs(cx - roi_center)
                            if distance < min_distance:
                                min_distance = distance
                                best_contour = contour

                    if best_contour is not None:
                        M = cv2.moments(best_contour)
                        cx = int(M['m10'] / M['m00'])

                        cv2.circle(roi, (cx, int(roi.shape[0] / 2)), 10, (0, 255, 0), -1)

                        error = cx - roi_center

                        # PID controller calculations
                        self.integral += error

                        # Apply integral windup limits
                        self.integral = max(min(self.integral, self.integral_max), self.integral_min)

                        derivative = error - self.previous_error

                        # Calculate the control variable
                        control = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

                        self.previous_error = error

                        # Adjust speeds based on the error magnitude
                        if abs(error) > self.turn_slowdown_threshold:
                            linear_speed = self.min_linear_speed
                            angular_speed = max(self.min_angular_speed, min(self.max_angular_speed, abs(control) * 0.1))
                        else:
                            linear_speed = max(self.min_linear_speed, self.max_linear_speed - abs(control) * 0.01)
                            angular_speed = abs(control)

                        # Update robot velocities
                        self.robot_velocity.angular.z = angular_speed * (-1 if error > 0 else 1)
                        self.robot_velocity.linear.x = linear_speed

                        # Update line position history
                        self.line_position_history.append(cx)
                        if len(self.line_position_history) > 5:
                            self.line_position_history.pop(0)
                    else:
                        self.robot_velocity.angular.z = 0.0
                        self.robot_velocity.linear.x = 0.0
                else:
                    self.robot_velocity.angular.z = 0.0
                    self.robot_velocity.linear.x = 0.0
            else:
                self.get_logger().warn('Line not found')
                self.robot_velocity.angular.z = 0.0
                self.robot_velocity.linear.x = 0.0

            self.velocity_publisher.publish(self.robot_velocity)
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(roi, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()