# ~/autodrive_ws/src/f1tenth_global_planner/f1tenth_global_planner/smoothing_node.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from .smoothing import fit_smoothing_spline, evaluate_spline_dense


class SmoothingNode(Node):
    def __init__(self):
        super().__init__('path_smoother')
        self.create_subscription(Path, '/raw_path', self.on_path, 1)
        self.pub = self.create_publisher(Path, '/smoothed_path', 1)
        self.get_logger().info('Smoothing node listo. Esperando /raw_path...')

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


def main():
    rclpy.init()
    node = SmoothingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()