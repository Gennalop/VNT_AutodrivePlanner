import os
import csv
from datetime import datetime

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import mark_inset

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

from .smoothing import fit_smoothing_spline, evaluate_spline_dense
from .grid_from_occupancy import grid_from_occupancy_msg


class SmoothingNode(Node):
    def __init__(self):
        super().__init__('path_smoother')

        # ---- Parámetros (vienen de config/planning_params.yaml) ----
        self.declare_parameter('clearance_m', 0.25)
        self.declare_parameter('occ_threshold', 50)
        self.declare_parameter('output_dir', '~/autodrive_ws/src/f1tenth_global_planner/output')
        self.declare_parameter('smoothing_tolerance_m', 0.15)
        self.declare_parameter('dense_samples', 1000)
        self.declare_parameter('waypoint_spacings', [0.5, 1.0])
        self.declare_parameter('zoom_radius_m', 1.0)
        self.declare_parameter('zoom_box_frac', 0.55)

        self.clearance_m = self.get_parameter('clearance_m').value
        self.occ_threshold = self.get_parameter('occ_threshold').value
        self.output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        self.tolerance_m = self.get_parameter('smoothing_tolerance_m').value
        self.dense_samples = self.get_parameter('dense_samples').value
        self.waypoint_spacings = list(self.get_parameter('waypoint_spacings').value)
        self.zoom_radius_m = self.get_parameter('zoom_radius_m').value
        self.zoom_box_frac = self.get_parameter('zoom_box_frac').value

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

        self.get_logger().info(
            f'Smoothing node listo. clearance={self.clearance_m}m | '
            f'tolerance={self.tolerance_m}m | output_dir={self.output_dir}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_map(self, msg):
        self.map_msg = msg

    def on_path(self, msg: Path):
        raw_waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(raw_waypoints) < 4:
            self.get_logger().warn('Path muy corto para suavizar (necesita >=4 puntos).')
            return

        tck, _u = fit_smoothing_spline(raw_waypoints, tolerance_m=self.tolerance_m, degree=3)
        dense_smoothed = evaluate_spline_dense(tck, num_samples=self.dense_samples)

        self.publish_smoothed(dense_smoothed)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir = os.path.join(self.output_dir, timestamp)
        os.makedirs(run_dir, exist_ok=True)

        self.save_map_debug(run_dir)

        zoom_center = self._find_max_curvature_point(raw_waypoints)

        self.save_variant('original', raw_waypoints, dense_smoothed, run_dir, zoom_center)
        for spacing in self.waypoint_spacings:
            raw_resampled = self.resample_path_by_distance(raw_waypoints, spacing)
            smooth_resampled = self.resample_path_by_distance(dense_smoothed, spacing)
            self.save_variant(f'{spacing:.1f}m', raw_resampled, smooth_resampled, run_dir, zoom_center)

        self.get_logger().info(f'Corrida completa guardada en: {run_dir}')

    def publish_smoothed(self, dense_points):
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
        self.get_logger().info(f'Path suavizado publicado en /smoothed_path: {len(dense_points)} puntos.')

    # ------------------------------------------------------------------
    # Remuestreo por distancia
    # ------------------------------------------------------------------

    @staticmethod
    def resample_path_by_distance(world_path, spacing):
        if len(world_path) < 2:
            return list(world_path)

        resampled = [world_path[0]]
        carry = 0.0

        for i in range(1, len(world_path)):
            p0 = np.array(world_path[i - 1], dtype=float)
            p1 = np.array(world_path[i], dtype=float)
            seg_vec = p1 - p0
            seg_len = np.linalg.norm(seg_vec)
            if seg_len == 0:
                continue
            seg_dir = seg_vec / seg_len

            traveled = 0.0
            while carry + (seg_len - traveled) >= spacing:
                step = spacing - carry
                traveled += step
                new_point = p0 + seg_dir * traveled
                resampled.append(tuple(new_point))
                carry = 0.0
            carry += seg_len - traveled

        last = np.array(world_path[-1], dtype=float)
        if not np.allclose(resampled[-1], last):
            resampled.append(tuple(last))

        return resampled

    # ------------------------------------------------------------------
    # Guardado por variante (original / 0.5m / 1.0m)
    # ------------------------------------------------------------------

    def save_variant(self, label, raw_points, smooth_points, run_dir, zoom_center=None):
        var_dir = os.path.join(run_dir, f'path_{label}')
        os.makedirs(var_dir, exist_ok=True)

        csv_path = os.path.join(var_dir, f'waypoints_path_{label}.csv')
        self._save_csv(smooth_points, csv_path)

        raw_img = os.path.join(var_dir, f'path_{label}.png')
        smooth_img = os.path.join(var_dir, f'path_{label}_smooth.png')
        overlap_img = os.path.join(var_dir, f'path_{label}_overlap.png')
        zoom_img = os.path.join(var_dir, f'path_{label}_zoom.png')
        curvature_img = os.path.join(var_dir, f'path_{label}_curvature.png')

        if self.map_msg is not None:
            self._plot(raw_points, None, raw_img, f'Ruta original ({label})')
            self._plot(None, smooth_points, smooth_img, f'Ruta suavizada ({label})')
            self._plot(raw_points, smooth_points, overlap_img, f'Original vs. suavizada ({label})')
            self._plot_zoom(raw_points, smooth_points, zoom_center, zoom_img, f'Zoom — original vs. suavizada ({label})')
        else:
            self.get_logger().warn(f'[{label}] No se recibió /map todavía; se omiten las imágenes con mapa de fondo.')

        self._plot_curvature(raw_points, smooth_points, curvature_img, f'Curvatura — original vs. suavizada ({label})')

        self.get_logger().info(
            f'[{label}] {len(raw_points)} pts crudos, {len(smooth_points)} pts suavizados -> {var_dir}'
        )

    @staticmethod
    def _save_csv(waypoints, filename):
        with open(filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for x, y in waypoints:
                writer.writerow([x, y])

    # ------------------------------------------------------------------
    # Plot base (con mapa de fondo)
    # ------------------------------------------------------------------

    def _draw_map_background(self, ax):
        w = self.map_msg.info.width
        h = self.map_msg.info.height
        res = self.map_msg.info.resolution
        origin = self.map_msg.info.origin

        grid = np.array(self.map_msg.data, dtype=np.int16).reshape(h, w)
        map_bin = np.where(grid > 50, 1, 0).astype(np.uint8)
        ax.imshow(map_bin, cmap='gray_r', origin='lower', interpolation='nearest')

        def to_pixel(x_world, y_world):
            col = (x_world - origin.position.x) / res
            row = (y_world - origin.position.y) / res
            return col, row

        return to_pixel

    def _plot(self, raw_points, smooth_points, out_path, title, dpi=150):
        fig, ax = plt.subplots(figsize=(8, 8))
        to_pixel = self._draw_map_background(ax)

        if raw_points:
            xs, ys = zip(*(to_pixel(x, y) for x, y in raw_points))
            style = '--' if smooth_points else '-'
            ax.plot(xs, ys, style, color='#888888', linewidth=1.4, label='Ruta original (LPA*)', zorder=2)

        if smooth_points:
            xs, ys = zip(*(to_pixel(x, y) for x, y in smooth_points))
            ax.plot(xs, ys, '-', color='#1f77b4', linewidth=2, label='Ruta suavizada', zorder=3)

        ref_points = raw_points if raw_points else smooth_points
        sx, sy = to_pixel(*ref_points[0])
        gx, gy = to_pixel(*ref_points[-1])
        ax.plot(sx, sy, 'o', color='#2ca02c', markersize=9, label='Start', zorder=4)
        ax.plot(gx, gy, '*', color='#d62728', markersize=14, label='Goal', zorder=4)

        ax.set_title(title)
        ax.legend(loc='upper right', framealpha=0.9)
        ax.set_xticks([])
        ax.set_yticks([])
        fig.tight_layout()
        fig.savefig(out_path, dpi=dpi)
        plt.close(fig)

    # ------------------------------------------------------------------
    # Zoom comparativo (inset con recuadro de ampliación)
    # ------------------------------------------------------------------

    def _plot_zoom(self, raw_points, smooth_points, zoom_center, out_path, title, dpi=150):
        if zoom_center is None:
            self.get_logger().warn('No se pudo determinar un centro de zoom (path muy corto); se omite el zoom.')
            return
    
        fig, ax = plt.subplots(figsize=(9, 9))
        to_pixel = self._draw_map_background(ax)
    
        raw_xs, raw_ys = zip(*(to_pixel(x, y) for x, y in raw_points))
        smooth_xs, smooth_ys = zip(*(to_pixel(x, y) for x, y in smooth_points))
    
        ax.plot(raw_xs, raw_ys, '--o', color='#888888', linewidth=1.2, markersize=3,
                label='Ruta original (waypoints)', zorder=2)
        ax.plot(smooth_xs, smooth_ys, '-o', color='#1f77b4', linewidth=1.6, markersize=2,
                label='Ruta suavizada', zorder=3)
        ax.legend(loc='upper right', framealpha=0.9)
        ax.set_title(title)
        ax.set_xticks([])
        ax.set_yticks([])
    
        cx_px, cy_px = to_pixel(*zoom_center)
        r_px = self.zoom_radius_m / self.map_msg.info.resolution
    
        box = self.zoom_box_frac
        origin_frac = (1.0 - box) / 2  # centra el recuadro; cambia esto si prefieres otra esquina
        axins = ax.inset_axes([origin_frac, origin_frac, box, box])
        axins.imshow(
            np.where(
                np.array(self.map_msg.data, dtype=np.int16).reshape(
                    self.map_msg.info.height, self.map_msg.info.width
                ) > 50, 1, 0
            ),
            cmap='gray_r', origin='lower', interpolation='nearest'
        )
        axins.plot(raw_xs, raw_ys, '--o', color='#888888', linewidth=1.4, markersize=6, zorder=2)
        axins.plot(smooth_xs, smooth_ys, '-o', color='#1f77b4', linewidth=1.8, markersize=4, zorder=3)
        axins.set_xlim(cx_px - r_px, cx_px + r_px)
        axins.set_ylim(cy_px - r_px, cy_px + r_px)
        axins.set_xticks([])
        axins.set_yticks([])
    
        mark_inset(ax, axins, loc1=2, loc2=4, fc='none', ec='0.4')
    
        fig.tight_layout()
        fig.savefig(out_path, dpi=dpi)
        plt.close(fig)

    @staticmethod
    def _find_max_curvature_point(points):
        """Devuelve el punto (x,y) del path con mayor curvatura (Menger, 3 puntos)."""
        if len(points) < 3:
            return None
        pts = np.array(points, dtype=float)
        best_kappa = -1.0
        best_point = None
        for i in range(1, len(pts) - 1):
            p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1]
            a = np.linalg.norm(p2 - p1)
            b = np.linalg.norm(p3 - p2)
            c = np.linalg.norm(p3 - p1)
            if a == 0 or b == 0 or c == 0:
                continue
            area = 0.5 * abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1]))
            kappa = 4 * area / (a * b * c)
            if kappa > best_kappa:
                best_kappa = kappa
                best_point = tuple(p2)
        return best_point

    # ------------------------------------------------------------------
    # Curvatura cruda vs. suavizada (línea vs. distancia recorrida)
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_curvature_profile(points):
        """Devuelve (arc_length, curvature) para cada punto interior del path."""
        if len(points) < 3:
            return np.array([]), np.array([])
        pts = np.array(points, dtype=float)

        arc_lengths = [0.0]
        for i in range(1, len(pts)):
            arc_lengths.append(arc_lengths[-1] + np.linalg.norm(pts[i] - pts[i - 1]))
        arc_lengths = np.array(arc_lengths)

        s_vals, kappa_vals = [], []
        for i in range(1, len(pts) - 1):
            p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1]
            a = np.linalg.norm(p2 - p1)
            b = np.linalg.norm(p3 - p2)
            c = np.linalg.norm(p3 - p1)
            if a == 0 or b == 0 or c == 0:
                continue
            area = 0.5 * abs((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p3[0] - p1[0]) * (p2[1] - p1[1]))
            kappa = 4 * area / (a * b * c)
            s_vals.append(arc_lengths[i])
            kappa_vals.append(kappa)

        return np.array(s_vals), np.array(kappa_vals)

    def _plot_curvature(self, raw_points, smooth_points, out_path, title, dpi=150):
        s_raw, k_raw = self._compute_curvature_profile(raw_points)
        s_smooth, k_smooth = self._compute_curvature_profile(smooth_points)

        if len(k_raw) == 0 and len(k_smooth) == 0:
            self.get_logger().warn('Path demasiado corto para calcular curvatura; se omite el gráfico.')
            return

        fig, ax = plt.subplots(figsize=(9, 4.5))
        if len(k_raw) > 0:
            ax.plot(s_raw, k_raw, '--', color='#888888', linewidth=1.2, label='Ruta original (LPA*)')
        if len(k_smooth) > 0:
            ax.plot(s_smooth, k_smooth, '-', color='#1f77b4', linewidth=1.6, label='Ruta suavizada')

        ax.set_xlabel('Distancia recorrida (m)')
        ax.set_ylabel('Curvatura (1/m)')
        ax.set_title(title)
        ax.legend(loc='upper right', framealpha=0.9)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(out_path, dpi=dpi)
        plt.close(fig)

    # ------------------------------------------------------------------
    # Debug de mapa
    # ------------------------------------------------------------------

    def save_map_debug(self, run_dir):
        if self.map_msg is None:
            self.get_logger().warn('No se recibió /map todavía; se omite el debug de mapa.')
            return
        maps_dir = os.path.join(run_dir, 'maps')
        grid_from_occupancy_msg(
            self.map_msg,
            occ_threshold=self.occ_threshold,
            clearance_m=self.clearance_m,
            debug_dir=maps_dir,
        )


def main():
    rclpy.init()
    node = SmoothingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()