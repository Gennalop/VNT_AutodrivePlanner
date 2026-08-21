import csv
import numpy as np
from scipy.interpolate import splprep, splev


def load_waypoints_csv(path):
    waypoints = []
    with open(path, newline='') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            x, y = float(row[0]), float(row[1])
            waypoints.append((x, y))
    return waypoints


def fit_smoothing_spline(waypoints, tolerance_m=0.15, degree=3):
    if len(waypoints) <= degree:
        raise ValueError(
            f"Se necesitan al menos {degree + 1} waypoints para un spline "
            f"de grado {degree}; se recibieron {len(waypoints)}."
        )

    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]

    m = len(waypoints)
    s = m * (tolerance_m ** 2)

    tck, u = splprep([xs, ys], s=s, k=degree)
    return tck, u


def evaluate_spline_dense(tck, num_samples=2000):
    u_fine = np.linspace(0, 1, num_samples)
    x_fine, y_fine = splev(u_fine, tck)
    return list(zip(x_fine.tolist(), y_fine.tolist()))


def check_collisions(points_world, map_bin, resolution, origin, world_to_map_fn):
    h, w = map_bin.shape
    colliding_points = []

    for x_world, y_world in points_world:
        x_map, y_cart = world_to_map_fn(x_world, y_world, resolution, origin, map_bin.shape)
        row = h - 1 - y_cart
        col = x_map
        if map_bin[row, col] == 1:
            colliding_points.append((x_world, y_world))

    collision_ratio = len(colliding_points) / len(points_world) if points_world else 0.0
    return colliding_points, collision_ratio