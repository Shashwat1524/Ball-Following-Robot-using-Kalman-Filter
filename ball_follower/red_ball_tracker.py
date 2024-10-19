#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class TrackRedBall(Node):
    def __init__(self):
        super().__init__('track_red_ball')
        self.rate = self.create_rate(10)  
        self.bridge = CvBridge()

      
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.twist = Twist()

        
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.process_img, 1)

        
        self.prev_angular_z = 0.0

        
        self.kalman_x = KalmanFilter(1, 1)  
        self.kalman_dist = KalmanFilter(1, 1)  

    def process_img(self, data):
        try:
            
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")

            h, w, d = img.shape

           
            redLower = np.array([0, 100, 100])
            redUpper = np.array([10, 255, 255])

          
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, redLower, redUpper)

            gray_img = cv2.GaussianBlur(mask, (5, 5), 0)
            edges = cv2.Canny(gray_img, 35, 125)

            contours, hierarchy = cv2.findContours(edges.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # Variables to track the best circle found
            best_circle = None
            max_circularity = 0

            # Loop through contours to find the best circle
            for contour in contours:
                # Calculate area and perimeter
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)

                # Check if area is valid and calculate circularity
                if perimeter > 0:
                    circularity = (4 * np.pi * area) / (perimeter ** 2)

                    # Update best circle if circularity is greater than the previous
                    if circularity > 0.7 and area > 500:  
                        if circularity > max_circularity:
                            max_circularity = circularity
                            best_circle = contour

            # Process the best circle found
            if best_circle is not None:
                M = cv2.moments(best_circle)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0

                
                cv2.circle(img, (cx, cy), 20, (0, 255, 0), -1)
                cv2.putText(img, "centroid", (cx - 25, cy - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

                # Use bounding box size for distance estimation
                x, y, ball_width, h = cv2.boundingRect(best_circle)  # Bounding box around the detected ball
                distance_estimation = 1000 / (ball_width + 1)  

                # Kalman Filter update step for x-coordinate and distance
                kalman_x_est = self.kalman_x.update(cx)
                kalman_dist_est = self.kalman_dist.update(distance_estimation)
                print(f"Estimated Distance to Ball (Kalman): {kalman_dist_est:.2f}")

                # Control logic
                center_offset = (w // 2 - kalman_x_est)  

                # Moving towards the ball
                if kalman_dist_est > 6:  # Ball is far away
                    self.twist.linear.x = 0.2
                elif kalman_dist_est < 5:  # Ball is too close
                    self.twist.linear.x = -0.2
                else:  # Within acceptable distance range
                    self.twist.linear.x = 0.0

                # Adjust angular velocity to center the ball with a deadband
                deadband = 100  # Deadband range for centering
                if abs(center_offset) >= deadband:  # Only adjust if outside of deadband
                    new_angular_z = 0.001 * center_offset  # Proportional control
                else:
                    new_angular_z = 0.0  # Stop turning if within deadband

                # Apply low-pass filter to smooth out angular velocity
                self.twist.angular.z = 0.9 * self.prev_angular_z + 0.1 * new_angular_z
                self.prev_angular_z = self.twist.angular.z  

            else:
                # No valid circle found, rotate to search for the ball
                self.twist.angular.z = 0.5 
                self.twist.linear.x = 0.0

         
            self.vel_pub.publish(self.twist)

           
            cv2.imshow('Red Ball Tracking', img)
            cv2.waitKey(5)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')


class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        # Kalman filter parameters
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.error_estimate = 1.0

    def update(self, measurement):
      
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_variance)

        
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)

        self.error_estimate = (1 - kalman_gain) * self.error_estimate + abs(self.estimate) * self.process_variance

        return self.estimate


def main(args=None):
    rclpy.init(args=args)
    track_red_ball_node = TrackRedBall()

    try:
        rclpy.spin(track_red_ball_node)
    except KeyboardInterrupt:
        pass
    finally:
        track_red_ball_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
