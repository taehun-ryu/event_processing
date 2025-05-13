import numpy as np
import cv2

import argparse
import sys
import os
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..')) # utils
sys.path.append(project_root)
import utils.preprocessing as pp
import representation as rep

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load HDF5 events and visualize a Tencode frame.")
    parser.add_argument( "--h5_path", "-p", type=str, required=True, help="Path to the .h5 events file")
    args = parser.parse_args()

    H5_PATH=args.h5_path
    WIDTH = 346
    HEIGHT = 260

    events = pp.load_h5_events(H5_PATH)
    #filtered_events = pp.filter_events_by_time(events, t_start=5.0, t_end=5.0033)

    start_idx = 10000
    end_idx= start_idx + 200000
    filtered_events =  events[start_idx:end_idx]

    rep.plot_3d_events(filtered_events)

    tencode = rep.tencode_frame(filtered_events, WIDTH, HEIGHT)
    time_surface = rep.time_surface(filtered_events, WIDTH, HEIGHT)

    cv2.namedWindow("View", cv2.WINDOW_NORMAL)
    cv2.imshow("View", time_surface)
    cv2.waitKey(0)
    cv2.destroyAllWindows()