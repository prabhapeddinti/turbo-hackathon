# turbo-hackathon
#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class PursuitEvasion(Node):

    def __init__(self):
        super().__init__('turtlebot_pursuit')

        # =========================================================
        # HUNTER
        # =========================================================

        self.hunter_cmd = self.create_publisher(
            Twist,
            '/hunter/cmd_vel',
            10
        )

        self.hunter_odom = self.create_subscription(
            Odometry,
            '/hunter/odom',
            self.hunter_odom_callback,
            10
        )

        self.hunter_scan = self.create_subscription(
            LaserScan,
            '/hunter/scan',
            self.scan_callback,
            10
        )

        # Runner position
        self.target_sub = self.create_subscription(
            Point,
            '/runner/target_position',
            self.target_callback,
            10
        )

        # Hunter position
        self.hx = 3.0
        self.hy = 0.0

        # Runner position
        self.rx = 0.0
        self.ry = 0.0

        self.target_found = False

        # LiDAR
        self.front = 10.0
        self.left = 10.0
        self.right = 10.0

        # Competition limits
        self.max_speed = 0.31
        self.max_turn = 1.90

        # Capture condition
        self.capture_radius = 0.5
        self.capture_required_time = 1.0

        self.capture_start = None
        self.captured = False

        # =========================================================
        # RUNNER
        # =========================================================

        self.runner_cmd = self.create_publisher(
            Twist,
            '/runner/cmd_vel',
            10
        )

        self.runner_odom = self.create_subscription(
            Odometry,
            '/runner/odom',
            self.runner_odom_callback,
            10
        )

        self.runner_x = 0.0
        self.runner_y = 0.0

        self.runner_time = 0.0

        # =========================================================
        # CONTROL LOOP
        # =========================================================

        self.timer = self.create_timer(
            0.05,
            self.control
        )

        self.get_logger().info(
            'Hunter + Runner system started'
        )

    # =============================================================
    # HUNTER ODOMETRY
    # =============================================================

    def hunter_odom_callback(self, msg):

        self.hx = msg.pose.pose.position.x
        self.hy = msg.pose.pose.position.y

    # =============================================================
    # RUNNER ODOMETRY
    # =============================================================

    def runner_odom_callback(self, msg):

        self.runner_x = msg.pose.pose.position.x
        self.runner_y = msg.pose.pose.position.y

    # =============================================================
    # TARGET
    # =============================================================

    def target_callback(self, msg):

        self.rx = msg.x
        self.ry = msg.y

        self.target_found = True

    # =============================================================
    # LIDAR
    # =============================================================

    def scan_callback(self, msg):

        ranges = list(msg.ranges)

        if len(ranges) == 0:
            return

        n = len(ranges)

        front_data = ranges[
            int(n * 0.40):
            int(n * 0.60)
        ]

        left_data = ranges[
            int(n * 0.60):
            int(n * 0.80)
        ]

        right_data = ranges[
            int(n * 0.20):
            int(n * 0.40)
        ]

        self.front = self.minimum(front_data)
        self.left = self.minimum(left_data)
        self.right = self.minimum(right_data)

    # =============================================================
    # FIND MINIMUM VALID LIDAR VALUE
    # =============================================================

    def minimum(self, values):

        valid = []

        for value in values:

            if math.isfinite(value) and value > 0.05:
                valid.append(value)

        if len(valid) == 0:
            return 10.0

        return min(valid)

    # =============================================================
    # MAIN CONTROL
    # =============================================================

    def control(self):

        # Run Runner
        self.control_runner()

        # Run Hunter
        self.control_hunter()

    # =============================================================
    # HUNTER CONTROL
    # =============================================================

    def control_hunter(self):

        if self.captured:

            self.stop_hunter()
            return

        if not self.target_found:

            self.search()
            return

        dx = self.rx - self.hx
        dy = self.ry - self.hy

        distance = math.sqrt(
            dx * dx + dy * dy
        )

        # ---------------------------------------------------------
        # CAPTURE CONDITION
        # ---------------------------------------------------------

        if distance <= self.capture_radius:

            self.check_capture()

            return

        self.capture_start = None

        # ---------------------------------------------------------
        # OBSTACLE AVOIDANCE
        # ---------------------------------------------------------

        if self.front < 0.65:

            self.avoid_obstacle()

        else:

            self.follow_target(dx, dy, distance)

    # =============================================================
    # TARGET PURSUIT
    # =============================================================

    def follow_target(
        self,
        dx,
        dy,
        distance
    ):

        angle = math.atan2(
            dy,
            dx
        )

        msg = Twist()

        # Proportional angular controller
        msg.angular.z = 1.5 * angle

        # Limit angular velocity
        if msg.angular.z > self.max_turn:
            msg.angular.z = self.max_turn

        if msg.angular.z < -self.max_turn:
            msg.angular.z = -self.max_turn

        # Speed control
        msg.linear.x = min(
            self.max_speed,
            0.10 + 0.08 * distance
        )

        # Turn more sharply when target is sideways
        if abs(angle) > 1.0:

            msg.linear.x = 0.07

        self.hunter_cmd.publish(msg)

    # =============================================================
    # OBSTACLE AVOIDANCE
    # =============================================================

    def avoid_obstacle(self):

        msg = Twist()

        msg.linear.x = 0.05

        if self.left > self.right:

            msg.angular.z = 1.0

        else:

            msg.angular.z = -1.0

        self.hunter_cmd.publish(msg)

    # =============================================================
    # SEARCH FOR TARGET
    # =============================================================

    def search(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.8

        self.hunter_cmd.publish(msg)

    # =============================================================
    # CAPTURE CHECK
    # =============================================================

    def check_capture(self):

        now = self.get_clock().now()

        if self.capture_start is None:

            self.capture_start = now

            self.get_logger().info(
                'Hunter entered 0.5 m capture zone'
            )

            self.stop_hunter()

            return

        elapsed = (
            now - self.capture_start
        ).nanoseconds / 1e9

        if elapsed >= self.capture_required_time:

            self.captured = True

            self.stop_hunter()

            self.get_logger().info(
                '================================'
            )

            self.get_logger().info(
                '       TARGET CAPTURED!'
            )

            self.get_logger().info(
                'Hunter remained within 0.5 m '
                'for 1 second.'
            )

            self.get_logger().info(
                '================================'
            )

    # =============================================================
    # STOP HUNTER
    # =============================================================

    def stop_hunter(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.hunter_cmd.publish(msg)

    # =============================================================
    # RUNNER
    # =============================================================

    def control_runner(self):

        self.runner_time += 0.05

        msg = Twist()

        # Runner moves continuously
        msg.linear.x = 0.25

        # Smoothly changes direction
        msg.angular.z = (
            0.8 *
            math.sin(
                self.runner_time * 0.7
            )
        )

        # Competition limits
        if msg.linear.x > 0.31:
            msg.linear.x = 0.31

        if msg.angular.z > 1.90:
            msg.angular.z = 1.90

        if msg.angular.z < -1.90:
            msg.angular.z = -1.90

        self.runner_cmd.publish(msg)


# ================================================================
# MAIN
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = PursuitEvasion()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.stop_hunter()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()
