#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_3d_events(events):
    """plot 3D events in a scatter plot.
    This function takes a 2D numpy array of events and plots them in 3D space.

    Args:
        events (numpy.ndarray): events array of shape (N, 4), where N is the number of events.
        Each event is represented by (t, x, y, p), where t is the time, x and y are the coordinates,
    """
    t = events[:, 0]
    x = events[:, 1]
    y = events[:, 2]
    p = events[:, 3]
    c = ['red' if polarity > 0 else 'blue' for polarity in p]
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    sc = ax.scatter(t, x, y, c=c, marker='.', s=1)

    ax.set_xlabel('Time')
    ax.set_ylabel('X')
    ax.set_zlabel('Y')
    ax.invert_zaxis()
    plt.title('3D Event Cloud (colored by polarity)')

    plt.show()

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


def time_surface(events, width, height):
    if events.size == 0:
        return np.zeros((height, width), np.float64)

    t_ref = events[-1, 0]
    tau = 50e-3
    last_ts = np.full((2, height, width), -np.inf, dtype=np.float32)
    for t, x, y, p in events:
        ix = int(round(x))
        iy = int(round(y))
        ch = 0 if p > 0 else 1
        last_ts[ch, iy, ix] = max(last_ts[ch, iy, ix], t)
    ts = np.exp(-(t_ref - last_ts) / tau)
    ts[last_ts < 0] = 0

    return ts[0], ts[1] # on, off
