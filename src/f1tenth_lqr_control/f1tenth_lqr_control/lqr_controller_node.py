#!/usr/bin/env python3
import csv
import math
import os
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class LQRControllerNode(Node):

  def __init__(self):
    super().__init__('lqr_controller_node')

    path_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )

    self.declare_parameter('csv_path', '')
    self.declare_parameter('debug', True)
    self.declare_parameter('target_speed_mps', 1.2)
    self.declare_parameter('throttle_limit', 0.2)

    csv_file_path = self.get_parameter('csv_path').get_parameter_value().string_value
    self.debug = self.get_parameter('debug').get_parameter_value().bool_value
    self.target_v = self.get_parameter('target_speed_mps').get_parameter_value().double_value
    self.throttle_k = self.get_parameter('throttle_limit').get_parameter_value().double_value

    self.finish_line_x = None
    self.finish_line_y = None
    self.finish_radius = 1.0
    self.min_lap_time_sec = 8.0

    self.lap_count = 0
    self.in_finish_zone = False
    self.start_time = None
    self.lap_start_time = None

    self.current_x = None
    self.current_y = None
    self.current_yaw = 0.0
    self.prev_x = None
    self.prev_y = None
    self.global_path = []

    self.Q = np.diag([2.0, 1.0])
    self.R = np.diag([0.5])

    self.lqr_iterations = 50
    self.eps_iter = 1e-3
    self.wheelbase = 0.33

    self.steering_pub = self.create_publisher(
        Float32, '/autodrive/f1tenth_1/steering_command', 10
    )
    self.throttle_pub = self.create_publisher(
        Float32, '/autodrive/f1tenth_1/throttle_command', 10
    )

    self.ips_sub = self.create_subscription(
        Point, '/autodrive/f1tenth_1/ips', self.ips_callback, 10
    )
    self.imu_sub = self.create_subscription(
        Imu, '/autodrive/f1tenth_1/imu', self.imu_callback, 10
    )

    if csv_file_path and os.path.exists(csv_file_path):
      self.get_logger().info(f'Cargando trayectoria desde CSV: {csv_file_path}')
      self.load_path_from_csv(csv_file_path)
    else:
      self.get_logger().info('Esperando trayectoria desde /smoothed_path...')
      self.path_sub = self.create_subscription(
          Path, '/smoothed_path', self.path_callback, path_qos
      )

    self.dt = 0.05
    self.control_timer = self.create_timer(self.dt, self.control_loop)

    if self.debug:
      self.get_logger().info(
          ' [DEBUG] Modo Debug ACTIVADO. Se mostrarán métricas detalladas en consola.'
      )

    self.get_logger().info('LQRControllerNode inicializado correctamente.')

  def process_and_close_path(self, raw_points):
    if len(raw_points) < 2:
      return

    start_pt = np.array(raw_points[0])
    end_pt = np.array(raw_points[-1])
    gap = np.linalg.norm(start_pt - end_pt)

    closed_points = list(raw_points)
    if gap > 0.1:
      num_pts = int(gap / 0.1)
      closing_x = np.linspace(end_pt[0], start_pt[0], num_pts)
      closing_y = np.linspace(end_pt[1], start_pt[1], num_pts)
      closing_segment = list(zip(closing_x, closing_y))[1:-1]
      closed_points.extend(closing_segment)

    path_with_yaw = []
    n = len(closed_points)
    for i in range(n):
      p1 = closed_points[i]
      p2 = closed_points[(i + 1) % n]
      yaw_ref = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
      path_with_yaw.append((p1[0], p1[1], yaw_ref))

    self.global_path = path_with_yaw
    self.get_logger().info(
        f' Trayectoria lista: {len(self.global_path)} waypoints en circuito cerrado.'
    )

  def load_path_from_csv(self, file_path):
    raw_points = []
    try:
      with open(file_path, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
          if row:
            try:
              raw_points.append((float(row[0]), float(row[1])))
            except ValueError:
              continue
      self.process_and_close_path(raw_points)
    except Exception as e:
      self.get_logger().error(f'Error leyendo CSV: {e}')

  def path_callback(self, msg: Path):
    if self.global_path:
      return
    raw_points = [
        (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
    ]
    self.process_and_close_path(raw_points)

  def ips_callback(self, msg: Point):
    self.prev_x = self.current_x
    self.prev_y = self.current_y

    self.current_x = msg.x
    self.current_y = msg.y

    if self.finish_line_x is None and self.finish_line_y is None:
      self.finish_line_x = self.current_x
      self.finish_line_y = self.current_y
      self.get_logger().info(
          f'🚩 Meta en ({self.finish_line_x:.2f}, {self.finish_line_y:.2f})'
      )

  def imu_callback(self, msg: Imu):
    q = msg.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

  def solve_lqr(self, A, B):
    P = self.Q.copy()
    for _ in range(self.lqr_iterations):
      BT_P = B.T @ P
      S = self.R + BT_P @ B + 1e-9 * np.eye(1)
      Ktmp = np.linalg.inv(S) @ (BT_P @ A)
      P_new = self.Q + A.T @ P @ A - A.T @ P @ B @ Ktmp
      if np.max(np.abs(P_new - P)) < self.eps_iter:
        P = P_new
        break
      P = P_new

    S = self.R + B.T @ P @ B + 1e-9 * np.eye(1)
    K = np.linalg.inv(S) @ (B.T @ P @ A)
    return K

  def compute_lqr_steering(self):
    distances = [
        math.hypot(self.current_x - pt[0], self.current_y - pt[1])
        for pt in self.global_path
    ]
    min_idx = int(np.argmin(distances))
    ref_x, ref_y, ref_yaw = self.global_path[min_idx]

    dx = self.current_x - ref_x
    dy = self.current_y - ref_y

    e_y = -math.sin(ref_yaw) * dx + math.cos(ref_yaw) * dy
    e_yaw = self.current_yaw - ref_yaw
    e_yaw = math.atan2(math.sin(e_yaw), math.cos(e_yaw))

    x_error = np.array([[e_y], [e_yaw]])

    v = max(self.target_v, 0.1)
    A = np.array([[1.0, v * self.dt], [0.0, 1.0]])
    B = np.array([[0.0], [(v * self.dt) / self.wheelbase]])

    K = self.solve_lqr(A, B)
    steering_raw = -float(K @ x_error)
    steering = max(-0.5, min(0.5, steering_raw))

    if self.debug:
      self.get_logger().info(
          f'🔍 [DEBUG] WP_idx: {min_idx}/{len(self.global_path)} | Pos:'
          f' ({self.current_x:.2f}, {self.current_y:.2f}) | Yaw:'
          f' {np.rad2deg(self.current_yaw):.1f}°\n    Error Lateral (e_y):'
          f' {e_y:.3f}m | Error Yaw (e_yaw): {np.rad2deg(e_yaw):.1f}°\n    '
          f'Ganancia K: [{K[0, 0]:.3f}, {K[0, 1]:.3f}] | Steering Raw:'
          f' {steering_raw:.3f} -> Clamped: {steering:.3f} rad',
          throttle_duration_sec=1.0,
      )

    return steering

  def check_lap_progress(self):
    if self.current_x is None or self.finish_line_x is None:
      return

    now = self.get_clock().now()
    if self.start_time is None:
      self.start_time = now
      self.lap_start_time = now
      return

    dist_to_finish = math.hypot(
        self.current_x - self.finish_line_x, self.current_y - self.finish_line_y
    )
    elapsed_lap_time = (now - self.lap_start_time).nanoseconds / 1e9

    if dist_to_finish <= self.finish_radius:
      if not self.in_finish_zone and elapsed_lap_time > self.min_lap_time_sec:
        self.in_finish_zone = True
        self.lap_count += 1

        lap_duration = elapsed_lap_time
        total_duration = (now - self.start_time).nanoseconds / 1e9

        self.get_logger().info('=' * 45)
        self.get_logger().info(
            f'🏁 VUELTA COMPLETADA -> Lap #{self.lap_count}'
        )
        self.get_logger().info(f'⏱️ Tiempo de Vuelta: {lap_duration:.3f} s')
        self.get_logger().info(f'⏱️ Tiempo Total: {total_duration:.3f} s')
        self.get_logger().info('=' * 45)

        self.lap_start_time = now
    else:
      self.in_finish_zone = False

  def check_for_collisions_or_stalls(self):
    if self.prev_x is None or self.current_x is None:
      return

    delta_dist = math.hypot(
        self.current_x - self.prev_x, self.current_y - self.prev_y
    )
    est_speed = delta_dist / self.dt

    if est_speed < 0.05 and self.throttle_k > 0:
      self.get_logger().warn(
          '⚠️ [ALERTA] ¡Posible choque o vehículo atascado! V_estimada:'
          f' {est_speed:.3f} m/s | Throttle: {self.throttle_k}',
          throttle_duration_sec=2.0,
      )

  def control_loop(self):
    self.check_lap_progress()

    if not self.global_path or self.current_x is None:
      if self.debug:
        self.get_logger().warn(
            '[DEBUG] Esperando por posición IPS o trayectoria global...',
            throttle_duration_sec=2.0,
        )
      return

    if self.debug:
      self.check_for_collisions_or_stalls()

    steering_val = self.compute_lqr_steering()
    throttle_val = self.throttle_k

    msg_steering = Float32()
    msg_steering.data = float(steering_val)

    msg_throttle = Float32()
    msg_throttle.data = float(throttle_val)

    self.steering_pub.publish(msg_steering)
    self.throttle_pub.publish(msg_throttle)

def main():
    rclpy.init()
    node = LQRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()