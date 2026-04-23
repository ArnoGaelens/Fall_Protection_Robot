#!/usr/bin/env python3
"""
Simulated person node — publishes a moving /person_pose (PoseStamped)
and moves a Gazebo cylinder model to match.

Behaviour: walks clicked waypoints in order, looping.
Natural variation: random pauses, speed fluctuations, slight path drift.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage
import math
import random
import subprocess
import threading
import time


class PersonSim(Node):
    def __init__(self):
        super().__init__('person_sim')

        self.declare_parameter('speed', 0.8)   # nominal m/s

        self.nominal_speed = self.get_parameter('speed').value

        self.pub        = self.create_publisher(PoseStamped, '/person_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/person_marker', 10)
        self.create_subscription(PointStamped, '/clicked_point', self._clicked_cb, 10)
        self.create_subscription(Empty,     '/resync_person',  self._resync_cb,    10)
        self.create_subscription(TFMessage, '/gz_world_poses', self._gz_poses_cb,  10)
        self.timer = self.create_timer(0.1, self.tick)

        # position and heading
        self._px  = 0.0
        self._py  = 0.0
        self._yaw = 0.0   # person heading toward next waypoint

        # waypoints
        self._waypoints = []
        self._wp_idx    = 0
        self._path_mode = False

        # natural behaviour state
        self._current_speed = 0.0          # actual speed (smoothed)
        self._target_speed  = self.nominal_speed
        self._pause_timer   = 0.0
        self._speed_timer   = 0.0
        self._accel         = 0.4          # m/s² — how fast speed changes

        # Gazebo ground truth position (read back from /gz_world_poses)
        self._gz_true_x  = None
        self._gz_true_y  = None

        self._gz_target   = None          # (x, y) latest position to send
        self._gz_force    = False         # force resend even if position unchanged
        self._gz_lock     = threading.Lock()
        self._gz_wakeup   = threading.Event()
        self._gz_thread   = threading.Thread(target=self._gz_worker, daemon=True)
        self._gz_thread.start()

        self.get_logger().info(
            'Person sim ready — click waypoints in RViz (Publish Point tool) to draw a path.'
        )

    # ------------------------------------------------------------------ #
    def _resync_cb(self, _msg: Empty):
        with self._gz_lock:
            self._gz_force = True
        self._gz_wakeup.set()
        self.get_logger().info('Manual Gazebo resync triggered.')

    def _gz_poses_cb(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id == 'person':
                self._gz_true_x = t.transform.translation.x
                self._gz_true_y = t.transform.translation.y
                return

    # ------------------------------------------------------------------ #
    def _clicked_cb(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y

        if self._waypoints:
            lx, ly = self._waypoints[-1]
            if math.hypot(x - lx, y - ly) < 0.1:
                self.get_logger().info(
                    f'Path closed ({len(self._waypoints)} waypoints) — looping.'
                )
                self._path_mode = True
                self._wp_idx = 0
                return

        self._waypoints.append((x, y))
        self.get_logger().info(
            f'Waypoint {len(self._waypoints)}: ({x:.2f}, {y:.2f})'
        )

        if len(self._waypoints) >= 2 and not self._path_mode:
            self._path_mode = True
            self._wp_idx = 0
            self._px, self._py = self._waypoints[0]
            self.get_logger().info('Walking started.')

    # ------------------------------------------------------------------ #
    def tick(self):
        if self._path_mode:
            self._update_behaviour()
            if self._pause_timer <= 0.0:
                self._step_waypoints()

        self._publish(self._px, self._py)

    # ------------------------------------------------------------------ #
    def _update_behaviour(self):
        dt = 0.1

        # count down pause
        if self._pause_timer > 0.0:
            self._pause_timer -= dt
            self._target_speed = 0.0
            if self._pause_timer <= 0.0:
                # resume walking after pause
                self._target_speed = self.nominal_speed * random.uniform(0.6, 1.1)
        else:
            # random pause trigger
            if random.random() < 0.008:
                duration = random.uniform(1.5, 4.0)
                self._pause_timer = duration
                self._target_speed = 0.0
                self.get_logger().info(
                    f'Person pausing for {duration:.1f}s', throttle_duration_sec=2.0
                )
            else:
                # occasional speed change
                self._speed_timer -= dt
                if self._speed_timer <= 0.0:
                    self._speed_timer = random.uniform(3.0, 8.0)
                    self._target_speed = self.nominal_speed * random.uniform(0.5, 1.2)

        # smooth acceleration toward target speed
        diff = self._target_speed - self._current_speed
        max_step = self._accel * dt
        self._current_speed += max(-max_step, min(max_step, diff))

    def _step_waypoints(self):
        if not self._waypoints:
            return

        tx, ty = self._waypoints[self._wp_idx]
        dx, dy = tx - self._px, ty - self._py
        dist = math.hypot(dx, dy)

        step = max(0.0, self._current_speed) * 0.1

        # update heading toward current target
        if dist > 0.01:
            self._yaw = math.atan2(dy, dx)

        if dist <= step:
            self._px, self._py = tx, ty
            self._wp_idx = (self._wp_idx + 1) % len(self._waypoints)
        else:
            drift = math.sin(self._px * 3.7 + self._py * 2.1) * 0.02 * min(1.0, dist)
            perp_x = -dy / dist
            perp_y =  dx / dist
            self._px += (dx / dist) * step + perp_x * drift
            self._py += (dy / dist) * step + perp_y * drift

    # ------------------------------------------------------------------ #
    def _gz_worker(self):
        """Background thread: sends gz service calls as fast as they complete."""
        last_sent = None
        last_periodic = time.monotonic()
        while True:
            now = time.monotonic()
            with self._gz_lock:
                target = self._gz_target
                force  = self._gz_force
                if force:
                    self._gz_force = False

            periodic = (now - last_periodic) >= 2.0

            if target is not None and (target != last_sent or force or periodic):
                px, py = target
                req = f'name: "person" position: {{x: {px:.3f} y: {py:.3f} z: 0.975}}'
                result = subprocess.run(
                    ['gz', 'service', '-s', '/world/empty/set_pose',
                     '--reqtype', 'gz.msgs.Pose',
                     '--reptype', 'gz.msgs.Boolean',
                     '--req', req,
                     '--timeout', '500'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                if result.returncode != 0:
                    self.get_logger().warn(
                        f'gz set_pose failed: {result.stderr.decode().strip()[:120]}',
                        throttle_duration_sec=5.0)
                last_sent = target
                last_periodic = time.monotonic()
            else:
                self._gz_wakeup.wait(timeout=0.05)
                self._gz_wakeup.clear()

    # ------------------------------------------------------------------ #
    def _publish(self, px, py):
        # use Gazebo ground truth if available (eliminates service-call lag)
        pub_x = self._gz_true_x if self._gz_true_x is not None else px
        pub_y = self._gz_true_y if self._gz_true_y is not None else py

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = pub_x
        msg.pose.position.y = pub_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(self._yaw / 2.0)
        msg.pose.orientation.w = math.cos(self._yaw / 2.0)
        self.pub.publish(msg)

        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = 'odom'
        marker.ns = 'person'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = pub_x
        marker.pose.position.y = pub_y
        marker.pose.position.z = 0.975
        marker.pose.orientation.z = math.sin(self._yaw / 2.0)
        marker.pose.orientation.w = math.cos(self._yaw / 2.0)
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 2.34
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

        with self._gz_lock:
            self._gz_target = (px, py)


def main(args=None):
    rclpy.init(args=args)
    node = PersonSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
