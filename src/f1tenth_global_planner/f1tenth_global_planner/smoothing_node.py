import os
import csv
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

from .smoothing import fit_smoothing_spline, evaluate_spline_dense


class SmoothingNode(Node):
    def __init__(self):
        super().__init__('path_smoother')

        self.declare_parameter('output_dir', os.path.expanduser('~/autodrive_ws/src/f1tenth_global_planner/output/'))
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value

        self.map_msg = None

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(Path, '/raw_path', self.on_path, 1)
        self.pub = self.create_publisher(Path, '/smoothed_path', 1)

        self.get_logger().info(f'Smoothing node listo. Guardando salidas en: {self.output_dir}')

    def on_map(self, msg):
        self.map_msg = msg

    def on_path(self, msg: Path):
        waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(waypoints) < 4:
            self.get_logger().warn('Path muy corto para suavizar (necesita >=4 puntos).')
            return

        tck, _u = fit_smoothing_spline(waypoints, tolerance_m=0.15, degree=3)
        dense_points = evaluate_spline_dense(tck, num_samples=1000)

        out = Path()
        out.header.frame_id = 'map'
        out.header.stamp = self.get_clock().now().to_msg()
        for x, y in dense_points:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)

        self.pub.publish(out)
        self.get_logger().info(f'Path suavizado publicado: {len(dense_points)} puntos.')

        self.save_outputs(waypoints, dense_points)

    # ---------- Guardado en disco ----------

    def save_outputs(self, raw_waypoints, smoothed_waypoints):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir = os.path.join(self.output_dir, timestamp)
        os.makedirs(run_dir, exist_ok=True)

        self._save_csv(raw_waypoints, os.path.join(run_dir, 'raw_path.csv'))
        self._save_csv(smoothed_waypoints, os.path.join(run_dir, 'smoothed_path.csv'))
        self.get_logger().info(f'CSV guardados en: {run_dir}')

        if self.map_msg is not None:
            plot_path = os.path.join(run_dir, 'path_plot.png')
            self.save_plot(raw_waypoints, smoothed_waypoints, plot_path)
            self.get_logger().info(f'Imagen guardada en: {plot_path}')
        else:
            self.get_logger().warn('No se recibió /map todavía; se omite la imagen del mapa.')

    @staticmethod
    def _save_csv(waypoints, filename):
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for x, y in waypoints:
                writer.writerow([x, y])

    def save_plot(self, raw_waypoints, smoothed_waypoints, out_path, dpi=150):
        w = self.map_msg.info.width
        h = self.map_msg.info.height
        res = self.map_msg.info.resolution
        origin = self.map_msg.info.origin

        grid = np.array(self.map_msg.data, dtype=np.int16).reshape(h, w)
        map_bin = np.where(grid > 50, 1, 0).astype(np.uint8)  # 1 = ocupado

        def to_pixel(x_world, y_world):
            col = (x_world - origin.position.x) / res
            row = (y_world - origin.position.y) / res
            return col, row

        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(map_bin, cmap='gray_r', origin='lower', interpolation='nearest')

        if raw_waypoints:
            xs, ys = zip(*(to_pixel(x, y) for x, y in raw_waypoints))
            ax.plot(xs, ys, '--', color='#888888', linewidth=1.2, label='Ruta cruda (LPA*)', zorder=2)

        if smoothed_waypoints:
            xs, ys = zip(*(to_pixel(x, y) for x, y in smoothed_waypoints))
            ax.plot(xs, ys, '-', color='#1f77b4', linewidth=2, label='Ruta suavizada', zorder=3)

        sx, sy = to_pixel(*raw_waypoints[0])
        gx, gy = to_pixel(*raw_waypoints[-1])
        ax.plot(sx, sy, 'o', color='#2ca02c', markersize=9, label='Start', zorder=4)
        ax.plot(gx, gy, '*', color='#d62728', markersize=14, label='Goal', zorder=4)

        ax.set_title('LPA* — Ruta cruda vs. suavizada')
        ax.legend(loc='upper right', framealpha=0.9)
        ax.set_xticks([])
        ax.set_yticks([])

        fig.tight_layout()
        fig.savefig(out_path, dpi=dpi)
        plt.close(fig)


def main():
    rclpy.init()
    node = SmoothingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()