#!/usr/bin/env python3
import argparse
import time
from collections import deque

import h5py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

class H5Event3DVisualizer:
    def __init__(self, ts, xs, ys, ps, window_size):
        self.window_size = window_size
        self.update_interval = 0.0001
        self.events = deque()
        self.ts, self.xs, self.ys, self.ps = ts, xs, ys, ps

        self.x_min, self.x_max = float(xs.min()), float(xs.max())
        self.y_min, self.y_max = float(ys.min()), float(ys.max())

        plt.ion()
        self.fig = plt.figure(figsize=(12, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('X (px)')
        self.ax.set_zlabel('Y (px)')

        plt.show(block=False)
        print('Press any key to start')

    def run(self):
        plt.waitforbuttonpress()

        start_time = time.time()
        next_time = start_time + self.update_interval
        total = len(self.ts)
        idx = 0

        while idx < total:
            plt.pause(0.0001)
            now = time.time()
            elapsed = now - start_time
            while idx < total and self.ts[idx] <= elapsed:
                self.events.append((self.ts[idx], self.xs[idx], self.ys[idx], self.ps[idx]))
                idx += 1
            if self.events:
                latest = self.events[-1][0]
                while self.events and (latest - self.events[0][0] > self.window_size):
                    self.events.popleft()
            if now >= next_time:
                self._update_plot()
                next_time += self.update_interval
        self._update_plot()
        plt.ioff()
        plt.show()

    def _update_plot(self):
        self.ax.clear()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('X (px)')
        self.ax.set_zlabel('Y (px)')
        if not self.events:
            return
        ts, xs, ys, ps = zip(*self.events)
        colors = ['red' if p else 'blue' for p in ps]
        self.ax.scatter(ts, xs, ys, c=colors, s=1)
        self.ax.set_xlim(max(0, ts[-1] - self.window_size), ts[-1])
        self.ax.set_ylim(self.x_min, self.x_max)
        self.ax.set_zlim(self.y_max, self.y_min)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def load_h5(path):
    with h5py.File(path, 'r') as f:
        g = f['events']
        ts = g['ts'][:].astype(float)
        xs = g['xs'][:]
        ys = g['ys'][:]
        ps = g['ps'][:].astype(int)
    ts -= ts[0]

    return ts, xs, ys, ps

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--path', required=True)
    parser.add_argument('--window', type=float, default=0.033)
    args = parser.parse_args()

    ts, xs, ys, ps = load_h5(args.path)
    viz = H5Event3DVisualizer(ts, xs, ys, ps, window_size=args.window)
    viz.run()

if __name__ == '__main__':
    main()
