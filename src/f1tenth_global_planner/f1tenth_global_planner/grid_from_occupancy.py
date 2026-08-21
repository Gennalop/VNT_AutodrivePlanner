# ~/autodrive_ws/src/f1tenth_global_planner/f1tenth_global_planner/grid_from_occupancy.py
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))  # hace visible "python_motion_planning" como paquete top-level

from python_motion_planning.utils import Grid  # ahora import ABSOLUTO, no relativo


def grid_from_occupancy_msg(msg, occ_threshold=50):
    w, h = msg.info.width, msg.info.height
    data = msg.data

    env = Grid(w, h)
    obstacles = set()
    for row in range(h):
        for col in range(w):
            v = data[row * w + col]
            if v >= occ_threshold or v == -1:
                obstacles.add((col, row))
    env.update(obstacles)

    return env, msg.info.resolution, msg.info.origin