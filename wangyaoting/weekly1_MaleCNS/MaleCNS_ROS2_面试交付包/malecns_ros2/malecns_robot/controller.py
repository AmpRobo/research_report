import csv
import math
import os
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

from .connectome import Connectome


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class MaleCNSController(Node):
    def __init__(self):
        super().__init__("malecns_controller")
        self.declare_parameter("goal_x", 2.5)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("log_path", "/tmp/malecns_run.csv")
        base = Path(__file__).parent / "data"
        self.net = Connectome(base / "circuit.npz", base / "readout.npz")
        self.goal = [float(self.get_parameter("goal_x").value), float(self.get_parameter("goal_y").value)]
        self.pose = None
        self.scan = None
        self.done = False
        self.path = NavPath()
        self.path.header.frame_id = "odom"
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.activity_pub = self.create_publisher(Float32MultiArray, "/neural_activity", 10)
        self.path_pub = self.create_publisher(NavPath, "/robot_path", 10)
        self.goal_pub = self.create_publisher(Marker, "/goal_marker", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.timer = self.create_timer(0.1, self.control)
        log_path = Path(str(self.get_parameter("log_path").value))
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = log_path.open("w", newline="")
        self.log = csv.writer(self.log_file)
        self.log.writerow(["time", "x", "y", "yaw_rad", "distance_to_goal", "left_m", "front_m", "right_m",
                           "neural_speed", "neural_turn", "cmd_speed", "cmd_turn", "done"])
        self.get_logger().info("MaleCNS v1.0: %d neurons, goal=(%.2f, %.2f)" %
                               (len(self.net.ids), self.goal[0], self.goal[1]))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self.pose = (float(p.x), float(p.y), yaw)
        if len(self.path.poses) == 0 or math.hypot(p.x-self.path.poses[-1].pose.position.x,
                                                   p.y-self.path.poses[-1].pose.position.y) > 0.04:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            self.path.poses.append(ps)
            self.path.header.stamp = msg.header.stamp

    def on_scan(self, msg):
        self.scan = msg

    def on_goal(self, msg):
        self.goal = [msg.pose.position.x, msg.pose.position.y]
        self.done = False
        self.get_logger().info("New navigation goal: (%.2f, %.2f)" % tuple(self.goal))

    def sectors(self):
        msg = self.scan
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        ranges = np.where(np.isfinite(ranges), ranges, msg.range_max)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)
        def nearest(lo, hi):
            part = ranges[(angles >= lo) & (angles <= hi)]
            return float(np.min(part)) if len(part) else float(msg.range_max)
        return nearest(0.3, 1.2), nearest(-0.3, 0.3), nearest(-1.2, -0.3)

    def control(self):
        if self.pose is None or self.scan is None:
            return
        x, y, yaw = self.pose
        left, front, right = self.sectors()
        distance = math.hypot(self.goal[0]-x, self.goal[1]-y)
        angle = wrap_angle(math.atan2(self.goal[1]-y, self.goal[0]-x)-yaw)
        obstacle = np.clip((1.7 - np.array([left, front, right]))/1.5, 0, 1)
        goal = [np.clip(angle/1.57, 0, 1), np.clip(1-abs(angle)/1.57, 0, 1),
                np.clip(-angle/1.57, 0, 1)]
        u = np.r_[obstacle, goal]
        neural_speed, neural_turn = self.net.predict(u)
        speed = float(np.clip(neural_speed, 0, 0.26))
        turn = float(np.clip(neural_turn, -1.5, 1.5))
        if front < 0.50:
            speed = 0.0
            turn = 1.0 if left >= right else -1.0
        if distance < 0.35:
            if not self.done:
                self.get_logger().info("GOAL_REACHED distance=%.3f m" % distance)
            self.done = True
        if self.done:
            speed, turn = 0.0, 0.0
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = turn
        self.cmd_pub.publish(cmd)
        self.activity_pub.publish(Float32MultiArray(data=self.net.state.tolist()))
        self.path_pub.publish(self.path)
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = self.goal[0], self.goal[1], 0.16
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.32
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 0.2, 1.0
        self.goal_pub.publish(marker)
        t = self.get_clock().now().nanoseconds / 1e9
        self.log.writerow([f"{t:.2f}", f"{x:.3f}", f"{y:.3f}", f"{yaw:.4f}", f"{distance:.3f}",
                           f"{left:.3f}", f"{front:.3f}", f"{right:.3f}",
                           f"{neural_speed:.3f}", f"{neural_turn:.3f}",
                           f"{speed:.3f}", f"{turn:.3f}", int(self.done)])
        self.log_file.flush()


def main():
    rclpy.init()
    node = MaleCNSController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
