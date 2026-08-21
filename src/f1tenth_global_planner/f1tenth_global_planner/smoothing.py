import csv
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import distance_transform_edt


def load_waypoints_csv(path):
    waypoints = []
    with open(path, newline='') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            x, y = float(row[0]), float(row[1])
            waypoints.append((x, y))
    return waypoints


def _clearance_weights(waypoints, map_bin, resolution, origin, world_to_map_fn,
                        min_weight=1.0, max_weight=20.0, eps_m=0.05):
    """
    Calcula un peso por waypoint inversamente proporcional a su distancia al
    obstáculo más cercano (en el mapa real, sin inflar). Waypoints en zonas
    angostas reciben más peso -> el spline los respeta más de cerca ahí, y
    tiene más libertad para suavizar en los tramos anchos.
    """
    free = (map_bin == 0).astype(np.uint8)
    dist_field = distance_transform_edt(free)  # en celdas
    h, w = map_bin.shape

    clearances_m = []
    for x, y in waypoints:
        x_map, y_cart = world_to_map_fn(x, y, resolution, origin, map_bin.shape)
        row = h - 1 - y_cart
        col = x_map
        row = int(np.clip(row, 0, h - 1))
        col = int(np.clip(col, 0, w - 1))
        clearances_m.append(dist_field[row, col] * resolution)

    clearances_m = np.array(clearances_m)
    weights = 1.0 / (clearances_m + eps_m)
    weights = np.clip(weights, min_weight, max_weight)
    return weights


def fit_smoothing_spline(waypoints, tolerance_m=0.15, degree=3, weights=None):
    if len(waypoints) <= degree:
        raise ValueError(
            f"Se necesitan al menos {degree + 1} waypoints para un spline "
            f"de grado {degree}; se recibieron {len(waypoints)}."
        )

    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]

    m = len(waypoints)
    s = m * (tolerance_m ** 2)

    tck, u = splprep([xs, ys], w=weights, s=s, k=degree)
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


def fit_safe_spline(waypoints, map_bin, resolution, origin, world_to_map_fn,
                     tolerance_m=0.15, degree=3, min_tolerance_m=0.02,
                     tolerance_decay=0.6, num_samples=3000, use_clearance_weights=True):
    """
    Ajusta un spline de suavizado y, si la curva resultante colisiona contra
    el mapa real, reduce la tolerancia (s más chico -> curva más pegada a los
    waypoints originales) y reintenta, hasta encontrar una curva sin
    colisiones o agotar el rango de tolerancia permitido.

    Devuelve (tck, dense_points, tolerance_usada, collision_ratio).
    Si no se logra eliminar la colisión del todo, devuelve el mejor intento
    (el de menor tolerancia probado) junto con una advertencia impresa.
    """
    weights = None
    if use_clearance_weights:
        weights = _clearance_weights(waypoints, map_bin, resolution, origin, world_to_map_fn)

    tol = tolerance_m
    best = None  # (tck, dense, tol, ratio)

    while tol >= min_tolerance_m:
        tck, _u = fit_smoothing_spline(waypoints, tolerance_m=tol, degree=degree, weights=weights)
        dense = evaluate_spline_dense(tck, num_samples=num_samples)
        colliding, ratio = check_collisions(dense, map_bin, resolution, origin, world_to_map_fn)

        if best is None or ratio < best[3]:
            best = (tck, dense, tol, ratio)

        if ratio == 0.0:
            return tck, dense, tol, ratio

        tol *= tolerance_decay

    print(
        f"  ADVERTENCIA: no se eliminaron todas las colisiones incluso con "
        f"tolerance_m={best[2]:.4f} (mínimo permitido {min_tolerance_m}). "
        f"Colisión residual: {best[3] * 100:.1f}%. Se devuelve el mejor intento."
    )
    return best