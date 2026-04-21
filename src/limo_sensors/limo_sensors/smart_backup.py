#!/usr/bin/env python3
"""
smart_backup.py
===============
Fused LiDAR + ultrasonic recovery for Ackermann LIMO robot.

Sensor fusion:
  - LiDAR /scan split into front/left/right sectors
  - Ultrasonics relayed via Range->float conversion
  - Each direction uses MIN(lidar_sector, ultrasonic) for decisions

Recovery flow:
  IDLE → detect obstacle (fused front < TRIGGER)
       → BACKING (straight back)
       → STEERING (slow arc toward clearest fused side)
       → CREEPING (slow forward until fused front clear)
       → re-send Nav2 goal → IDLE
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
import math

IDLE     = 'IDLE'
BACKING  = 'BACKING'
STEERING = 'STEERING'
CREEPING = 'CREEPING'


class SmartBackupNode(Node):
    def __init__(self):
        super().__init__('smart_backup')

        # ── tunable thresholds ───────────────────────────────────────
        self.TRIGGER_DIST = 0.50   # fused front < this  → start recovery
        self.CLEAR_DIST   = 1.0    # fused front > this  → recovery done
        self.BACKUP_SPEED = -0.08  # m/s  backward
        self.CREEP_SPEED  =  0.05  # m/s  forward arc
        self.STEER_SPEED  =  0.5   # rad/s during steering
        self.BACKUP_TICKS = 30     # 30 × 0.1 s = 3 s backup
        self.STEER_TICKS  = 25     # 25 × 0.1 s = 2.5 s steering

        # ── LiDAR sector angles ──────────────────────────────────────
        self.FRONT_DEG = 35.0   # ±35° forward cone
        self.SIDE_DEG  = 60.0   # 35°–95° side cones

        # ── raw sensor values (initialise to max range) ──────────────
        # LiDAR sectors
        self.lidar_front = 4.0
        self.lidar_left  = 4.0
        self.lidar_right = 4.0
        # Ultrasonics
        self.us_front = 4.0
        self.us_left  = 4.0
        self.us_right = 4.0

        # ── state machine ────────────────────────────────────────────
        self.state      = IDLE
        self.ticks      = 0
        self.steer_dir  = 1.0
        self.saved_goal = None
        self.goal_handle = None
        self.cooldown   = 0

        # ── LiDAR subscriber (standard QoS works fine) ───────────────
        self.create_subscription(
            LaserScan, '/scan',
            self.scan_cb, 10)

        # ── Ultrasonic subscribers (must use qos_profile_sensor_data) ─
        self.create_subscription(
            LaserScan, '/ultrasonic_front_scan',
            self.us_front_cb, 10)
        self.create_subscription(
            LaserScan, '/ultrasonic_left_scan',
            self.us_left_cb, 10)
        self.create_subscription(
            LaserScan, '/ultrasonic_right_scan',
            self.us_right_cb, 10)

        # ── goal listener ────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/goal_pose',
            self.goal_cb, 10)

        # ── cmd_vel publisher ────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Nav2 action client ───────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(0.1, self.update)
        self.get_logger().info(
            'SmartBackup ready — fused LiDAR + ultrasonic recovery')

    # ── LiDAR callback ───────────────────────────────────────────────
    def scan_cb(self, msg: LaserScan):
        self.lidar_front = self._sector_min(msg, -self.FRONT_DEG, self.FRONT_DEG)
        self.lidar_left  = self._sector_min(msg,  self.FRONT_DEG,
                                             self.FRONT_DEG + self.SIDE_DEG)
        self.lidar_right = self._sector_min(msg,
                                            -(self.FRONT_DEG + self.SIDE_DEG),
                                            -self.FRONT_DEG)

    def _sector_min(self, msg: LaserScan, lo_deg: float, hi_deg: float) -> float:
        lo = math.radians(lo_deg)
        hi = math.radians(hi_deg)
        min_r = 4.0
        for i, r in enumerate(msg.ranges):
            a = msg.angle_min + i * msg.angle_increment
            if lo <= a <= hi and math.isfinite(r):
                if msg.range_min <= r <= msg.range_max:
                    min_r = min(min_r, r)
        return min_r

    # ── Ultrasonic callbacks ──────────────────────────────────────────
    def us_front_cb(self, msg: LaserScan):
        self.us_front = min(r for r in msg.ranges if math.isfinite(r)) if msg.ranges else 4.0

    def us_left_cb(self, msg: LaserScan):
        self.us_left = min(r for r in msg.ranges if math.isfinite(r)) if msg.ranges else 4.0

    def us_right_cb(self, msg: LaserScan):
        self.us_right = min(r for r in msg.ranges if math.isfinite(r)) if msg.ranges else 4.0

    # ── Fusion: min of LiDAR sector and ultrasonic ───────────────────
    @property
    def front(self):
        return min(self.lidar_front, self.us_front)

    @property
    def left(self):
        return min(self.lidar_left, self.us_left)

    @property
    def right(self):
        return min(self.lidar_right, self.us_right)

    # ── Nav2 goal handling ───────────────────────────────────────────
    def goal_cb(self, msg: PoseStamped):
        self.saved_goal = msg

    def _cancel_nav2(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.get_logger().info('Nav2 goal cancelled')

    def _resend_goal(self):
        if self.saved_goal is None:
            self.get_logger().warn('No saved goal to re-send')
            return
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available')
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.saved_goal
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_sent_cb)
        self.get_logger().info('Nav2 goal re-sent')

    def _goal_sent_cb(self, future):
        self.goal_handle = future.result()

    def _stop(self):
        self.cmd_pub.publish(Twist())

    # ── Main state machine ───────────────────────────────────────────
    def update(self):
        if self.cooldown > 0:
            self.cooldown -= 1
            return

        # ── IDLE ─────────────────────────────────────────────────────
        if self.state == IDLE:
            if self.front < self.TRIGGER_DIST:
                self.get_logger().warn(
                    f'Obstacle detected! '
                    f'Fused F:{self.front:.2f} L:{self.left:.2f} R:{self.right:.2f} | '
                    f'LiDAR F:{self.lidar_front:.2f} L:{self.lidar_left:.2f} R:{self.lidar_right:.2f} | '
                    f'US F:{self.us_front:.2f} L:{self.us_left:.2f} R:{self.us_right:.2f}')
                self._cancel_nav2()
                self.steer_dir = 1.0 if self.left > self.right else -1.0
                self.ticks = 0
                self.state = BACKING
            return

        # ── BACKING ──────────────────────────────────────────────────
        if self.state == BACKING:
            self.ticks += 1
            twist = Twist()
            twist.linear.x  = self.BACKUP_SPEED
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            if self.ticks >= self.BACKUP_TICKS:
                self._stop()
                self.steer_dir = 1.0 if self.left > self.right else -1.0
                self.get_logger().info(
                    f'Backup done. '
                    f'Steering {"LEFT" if self.steer_dir > 0 else "RIGHT"} '
                    f'(fused L:{self.left:.2f} R:{self.right:.2f})')
                self.ticks = 0
                self.state = STEERING
            return

        # ── STEERING ─────────────────────────────────────────────────
        if self.state == STEERING:
            self.ticks += 1
            twist = Twist()
            twist.linear.x  = self.CREEP_SPEED
            twist.angular.z = self.steer_dir * self.STEER_SPEED
            self.cmd_pub.publish(twist)
            if self.ticks >= self.STEER_TICKS:
                self._stop()
                self.ticks = 0
                self.state = CREEPING
            return

        # ── CREEPING ─────────────────────────────────────────────────
        if self.state == CREEPING:
            if self.front >= self.CLEAR_DIST:
                self._stop()
                self.get_logger().info(
                    f'Path clear — fused front:{self.front:.2f}m '
                    f'(LiDAR:{self.lidar_front:.2f} US:{self.us_front:.2f}) '
                    f'— resuming Nav2')
                self._resend_goal()
                self.cooldown = 30
                self.state = IDLE
                return
            if self.front < 0.3:
                self.get_logger().warn(
                    'Still blocked while creeping — backing up again')
                self._stop()
                self.ticks = 0
                self.state = BACKING
                return
            twist = Twist()
            twist.linear.x  = self.CREEP_SPEED
            twist.angular.z = self.steer_dir * self.STEER_SPEED * 0.5
            self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SmartBackupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
