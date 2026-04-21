#!/usr/bin/env python3
"""
Simulated person node — publishes a moving /person_pose (PoseStamped)
and moves a Gazebo cylinder model to match.

Behaviour: walks clicked waypoints in order, looping.
Natural variation: random pauses, speed fluctuations, slight path drift.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
import math
import random
import subprocess


class PersonSim(Node):
    def __init__(self):
        super().__init__('person_sim')

        self.declare_parameter('speed', 0.8)   # nominal m/s

        self.nominal_speed = self.get_parameter('speed').value

        self.pub = self.create_publisher(PoseStamped, '/person_pose', 10)
        self.create_subscription(PointStamped, '/clicked_point', self._clicked_cb, 10)
        self.timer = self.create_timer(0.1, self.tick)

        # position
        self._px = 0.0
        self._py = 0.0

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

        self._gz_tick = 0
        self._gz_proc  = None   # last gz service subprocess

        self.get_logger().info(
            'Person sim ready — click waypoints in RViz (Publish Point tool) to draw a path.'
        )

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

        if dist <= step:
            self._px, self._py = tx, ty
            self._wp_idx = (self._wp_idx + 1) % len(self._waypoints)
        else:
            # slight lateral drift for realism (±2 cm max, fades near waypoint)
            drift = math.sin(self._px * 3.7 + self._py * 2.1) * 0.02 * min(1.0, dist)
            perp_x = -dy / dist
            perp_y =  dx / dist
            self._px += (dx / dist) * step + perp_x * drift
            self._py += (dy / dist) * step + perp_y * drift

    # ------------------------------------------------------------------ #
    def _publish(self, px, py):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

        self._gz_tick += 1
        if self._gz_tick >= 3:
            self._gz_tick = 0
            # only spawn if previous call has finished
            if self._gz_proc is None or self._gz_proc.poll() is not None:
                req = f'name: "person" position: {{x: {px:.3f} y: {py:.3f} z: 0.9}}'
                self._gz_proc = subprocess.Popen(
                    ['gz', 'service', '-s', '/world/empty/set_pose',
                     '--reqtype', 'gz.msgs.Pose',
                     '--reptype', 'gz.msgs.Boolean',
                     '--req', req,
                     '--timeout', '200'],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )


def main(args=None):
    rclpy.init(args=args)
    node = PersonSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
