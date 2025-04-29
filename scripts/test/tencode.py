#!/usr/bin/env python3
import numpy as np
import cv2

import sys
import os
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..')) # utils
sys.path.append(project_root)
import utils.preprocessing as pp

def tencode_frame(events, width, height):
    if events.size == 0:
        return np.zeros((height, width, 3), np.uint8)

    t_min = events[0, 0]
    t_max = events[-1, 0]
    delta_t = t_max - t_min
    if delta_t <= 0:
        return np.zeros((height, width, 3), np.uint8)

    ts_map  = np.full((height, width), -np.inf, dtype=np.float64)
    pol_map = np.zeros((height, width), dtype=np.int8)

    for t, x, y, p in events:
        xi, yi = int(x), int(y)
        if 0 <= xi < width and 0 <= yi < height and t > ts_map[yi, xi]:
            ts_map[yi, xi]  = t
            pol_map[yi, xi] = int(p)

    norm = np.clip((t_max - ts_map) / delta_t, 0.0, 1.0)
    G = (norm * 255).astype(np.uint8)

    img = np.zeros((height, width, 3), np.uint8)
    on_mask  = (pol_map > 0)
    off_mask = (pol_map < 0)

    img[on_mask, 0] = 0
    img[on_mask, 1] = G[on_mask]
    img[on_mask, 2] = 255

    img[off_mask, 0] = 255
    img[off_mask, 1] = G[off_mask]
    img[off_mask, 2] = 0

    return img

if __name__ == "__main__":
    H5_PATH = "/root/datasets/self_dvs/event_cam2/250327/left_0.h5"
    WIDTH   = 346
    HEIGHT  = 260

    events = pp.load_h5_events(H5_PATH)
    filtered_events = pp.filter_events_by_time(events, t_start=5.0, t_end=5.0033)

    frame = tencode_frame(filtered_events, WIDTH, HEIGHT)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.namedWindow("Tencode View", cv2.WINDOW_NORMAL)
    cv2.imshow("Tencode View", gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
