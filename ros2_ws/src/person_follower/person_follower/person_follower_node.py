#!/usr/bin/env python3
"""
ROS2 node for visual servoing person following.
Subscribes to detections and publishes velocity commands to follow a person.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import Trigger
import time


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__("person_follower_node")

        self.declare_parameter("kp_angular", 2.0)
        self.declare_parameter("kp_linear", 1.5)
        self.declare_parameter("target_bbox_area", 0.25)
        self.declare_parameter("max_linear_vel", 0.75)
        self.declare_parameter("max_angular_vel", 1.0)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("angular_deadzone", 0.05)
        self.declare_parameter("linear_deadzone", 0.05)
        self.declare_parameter("detection_timeout", 2.0)
        self.declare_parameter("target_class_ids", ["face"])
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 640)

        self.kp_angular = self.get_parameter("kp_angular").value
        self.kp_linear = self.get_parameter("kp_linear").value
        self.target_bbox_area = self.get_parameter("target_bbox_area").value
        self.max_linear_vel = self.get_parameter("max_linear_vel").value
        self.max_angular_vel = self.get_parameter("max_angular_vel").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.angular_deadzone = self.get_parameter("angular_deadzone").value
        self.linear_deadzone = self.get_parameter("linear_deadzone").value
        self.detection_timeout = self.get_parameter("detection_timeout").value
        self.target_class_ids = self.get_parameter("target_class_ids").value

        self.is_active = False
        self.last_detection = None
        self.last_detection_time = None
        self.current_twist = Twist()

        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value

        self.cmd_vel_pub = self.create_publisher(Twist, "/person_following_cmd_vel", 10)

        self.detection_sub = self.create_subscription(Detection2DArray, "/detections", self.detection_callback, 10)

        self.activate_srv = self.create_service(Trigger, "activate_person_following", self.activate_callback)

        self.deactivate_srv = self.create_service(Trigger, "deactivate_person_following", self.deactivate_callback)

        self.control_timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)

        self.get_logger().info("Person follower node initialized")

    def activate_callback(self, request, response):
        """Handle activation service request."""
        self.is_active = True
        response.success = True
        response.message = "Person following activated"
        self.get_logger().info("Person following activated")
        return response

    def deactivate_callback(self, request, response):
        """Handle deactivation service request."""
        self.is_active = False
        self.stop_robot()
        response.success = True
        response.message = "Person following deactivated"
        self.get_logger().info("Person following deactivated")
        return response

    def detection_callback(self, msg):
        """Process detection messages."""
        if not msg.detections:
            return

        person_detections = []
        for detection in msg.detections:
            for result in detection.results:
                if result.hypothesis.class_id in self.target_class_ids:
                    person_detections.append(detection)
                    break

        if person_detections:
            self.last_detection = person_detections[0]
            self.last_detection_time = time.time()

    def control_loop(self):
        """Направлять корпус на последнюю свежую детекцию."""
        if not self.is_active:
            return

        if self.last_detection_time is None:
            self.get_logger().warn("No detections received yet", throttle_duration_sec=5.0)
            return

        if time.time() - self.last_detection_time > self.detection_timeout:
            self.get_logger().warn("Detection timeout - stopping robot")
            self.stop_robot()
            return

        if self.last_detection is None:
            return

        bbox = self.last_detection.bbox
        bbox_center_x = bbox.center.position.x
        bbox_center_y = bbox.center.position.y
        bbox_width = bbox.size_x
        bbox_height = bbox.size_y

        image_center_x = self.image_width / 2.0
        image_center_y = self.image_height / 2.0

        # Нормированная ошибка положения цели по горизонтали.
        angular_error = (bbox_center_x - image_center_x) / (self.image_width / 2.0)

        if abs(angular_error) < self.angular_deadzone:
            angular_error = 0.0

        # Знак соответствует направлению оси камеры и yaw робота.
        wz = -self.kp_angular * angular_error

        wz = max(-self.max_angular_vel, min(self.max_angular_vel, wz))

        # Площадь bbox служит грубой оценкой расстояния до цели.
        bbox_area = (bbox_width * bbox_height) / (self.image_width * self.image_height)
        area_error = self.target_bbox_area - bbox_area

        if abs(area_error) < self.linear_deadzone:
            area_error = 0.0

        vx = self.kp_linear * area_error

        vx = max(-self.max_linear_vel, min(self.max_linear_vel, vx))

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = 0.0  # No lateral movement
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = wz

        self.cmd_vel_pub.publish(twist)
        self.current_twist = twist

        self.get_logger().info(
            f"Control: vx={vx:.2f}, wz={wz:.2f}, bbox_area={bbox_area:.3f}, " f"angular_error={angular_error:.3f}",
            # throttle_duration_sec=1.0
        )

    def stop_robot(self):
        """Stop the robot by publishing zero velocities."""
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        self.current_twist = stop_twist


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down person follower node")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
