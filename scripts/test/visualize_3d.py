#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from dvs_msgs.msg import EventArray
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from collections import deque

class Event3DVisualizer(Node):
    def __init__(self, window_size=0.1, update_interval=0.05):
        super().__init__('event_3d_visualizer')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5000
        )
        self.subscription = self.create_subscription(
            EventArray,
            '/dvs/events',
            self.event_callback,
            qos
        )
        self.window_size = window_size
        self.events = deque()
        self.start_time = None

        plt.ion()
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('X (px)')
        self.ax.set_zlabel('Y (px)')

        self.timer = self.create_timer(update_interval, self.update_plot)

    def event_callback(self, msg: EventArray):
        for e in msg.events:
            t = e.ts.sec + e.ts.nanosec * 1e-9
            if self.start_time is None:
                self.start_time = t
            t_rel = t - self.start_time
            p_val = getattr(e, 'p', None)
            if p_val is None:
                p_val = int(e.polarity)
            else:
                p_val = int(p_val)
            self.events.append((t_rel, e.x, e.y, p_val))

        if self.events:
            current_t = self.events[-1][0]
            while self.events and (current_t - self.events[0][0] > self.window_size):
                self.events.popleft()

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('X (px)')
        self.ax.set_zlabel('Y (px)')
        self.ax.invert_zaxis()
        if self.events:
            ts, xs, ys, ps = zip(*self.events)
            colors = ['red' if p > 0 else 'blue' for p in ps]
            self.ax.scatter(ts, xs, ys, c=colors, s=1)
            self.ax.set_xlim(max(0, ts[-1] - self.window_size), ts[-1])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    node = Event3DVisualizer(window_size=0.03, update_interval=0.1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

