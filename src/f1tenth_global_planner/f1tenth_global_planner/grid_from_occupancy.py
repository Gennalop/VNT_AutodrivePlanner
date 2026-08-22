import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import cv2
from scipy.ndimage import distance_transform_edt, maximum_filter

from python_motion_planning.utils import Grid


def grid_from_occupancy_msg(msg, occ_threshold=50, clearance_m=0.25, debug_dir=None):
    w, h = msg.info.width, msg.info.height
    res = msg.info.resolution

    data = np.array(msg.data, dtype=np.int16).reshape(h, w)
    occ_raw = np.where((data >= occ_threshold) | (data == -1), 1, 0).astype(np.uint8)

    occ_processed = occ_raw.copy()
    clearance_cells = 0
    if clearance_m > 0:
        clearance_cells = max(1, round(clearance_m / res))
        kernel_size = 2 * clearance_cells + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        occ_processed = cv2.dilate(occ_raw, kernel, iterations=1)

    if debug_dir is not None:
        os.makedirs(debug_dir, exist_ok=True)
        _save_debug_images(occ_raw, occ_processed, res, clearance_m, clearance_cells, debug_dir)
        _diagnose_corridor_width(occ_processed, res, debug_dir)

    env = Grid(w, h)
    obstacles = {
        (int(col), int(row))
        for row in range(h) for col in range(w)
        if occ_processed[row, col] == 1
    }
    env.update(obstacles)

    return env, res, msg.info.origin


def _to_img(occ, scale=3):
    img = np.where(occ == 1, 0, 255).astype(np.uint8)
    return cv2.resize(img, (img.shape[1] * scale, img.shape[0] * scale), interpolation=cv2.INTER_NEAREST)


def _save_debug_images(occ_raw, occ_processed, res, clearance_m, clearance_cells, debug_dir):
    raw_path = os.path.join(debug_dir, 'map_raw.png')
    processed_path = os.path.join(debug_dir, f'map_inflated_clearance_{clearance_m:.2f}m.png')

    cv2.imwrite(raw_path, _to_img(occ_raw))
    cv2.imwrite(processed_path, _to_img(occ_processed))

    print(f'[grid_from_occupancy][debug] resolución: {res:.4f} m/celda | '
          f'clearance solicitado: {clearance_m:.2f} m | celdas de dilatación: {clearance_cells} | '
          f'clearance real: {clearance_cells * res:.3f} m')


def _diagnose_corridor_width(occ_processed, res, debug_dir):
    free = (occ_processed == 0).astype(np.uint8)
    dist = distance_transform_edt(free)
    local_max = (dist == maximum_filter(dist, size=3)) & (free == 1) & (dist > 0)

    ridge_dists = dist[local_max]
    if len(ridge_dists) == 0:
        print('[grid_from_occupancy][debug] No se encontraron corredores libres para diagnosticar.')
        return None

    min_width_m = 2 * ridge_dists.min() * res
    bottleneck_idx = np.argwhere(local_max)[np.argmin(dist[local_max])]
    row, col = bottleneck_idx

    vis = (dist / dist.max() * 255).astype(np.uint8) if dist.max() > 0 else dist.astype(np.uint8)
    vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    vis[occ_processed == 1] = [0, 0, 0]
    scale = 3
    vis = cv2.resize(vis, (vis.shape[1] * scale, vis.shape[0] * scale), interpolation=cv2.INTER_NEAREST)
    cv2.circle(vis, (col * scale, row * scale), 6, (255, 255, 255), 2)

    out_path = os.path.join(debug_dir, 'corridor_width_debug.png')
    cv2.imwrite(out_path, vis)

    print(f'[grid_from_occupancy][debug] ancho mínimo de corredor tras inflado: {min_width_m:.3f} m '
          f'(celda crítica: fila={row}, col={col})')

    return min_width_m, (row, col)