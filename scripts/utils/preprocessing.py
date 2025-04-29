import h5py
import numpy as np

def load_h5_events(h5_path):
    f = h5py.File(h5_path, 'r')
    ps = f['events/ps'][:]
    ts = f['events/ts'][:].astype(np.float64)
    xs = f['events/xs'][:]
    ys = f['events/ys'][:]
    f.close()

    ts -= ts.min()
    pols = np.where(ps, 1, -1).astype(np.int8)

    events = np.column_stack((ts, xs, ys, pols))  # [t, x, y, p]
    return events[events[:, 0].argsort()]

def filter_events_by_time(events, t_start=None, t_end=None):
    ts = events[:, 0]
    mask = np.ones_like(ts, dtype=bool)
    if t_start is not None:
        mask &= (ts >= t_start)
    if t_end is not None:
        mask &= (ts <= t_end)
    return events[mask]