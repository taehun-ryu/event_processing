#!/usr/bin/env python3
import os
import argparse
import h5py
import numpy as np
import dv_processing as dv

def save_events_h5(timestamps, xs, ys, ps, out_dir, out_fname):
    os.makedirs(out_dir, exist_ok=True)
    if not out_fname.endswith(".h5"):
        out_fname += ".h5"
    fullpath = os.path.join(out_dir, out_fname)

    with h5py.File(fullpath, "w") as hf:
        grp = hf.create_group("events")
        grp.create_dataset("ts",
                           data=np.array(timestamps, dtype=np.int64),
                           compression="gzip", chunks=True)
        grp.create_dataset("xs",
                           data=np.array(xs, dtype=np.int32),
                           compression="gzip", chunks=True)
        grp.create_dataset("ys",
                           data=np.array(ys, dtype=np.int32),
                           compression="gzip", chunks=True)
        grp.create_dataset("ps",
                           data=np.array(ps, dtype=bool),
                           compression="gzip", chunks=True)

    print(f"[+] Saved HDF5 events to: {fullpath}")

def main():
    parser = argparse.ArgumentParser(description="Convert AEDAT4 events to HDF5 (with optional zero-based timestamps)")
    parser.add_argument("aedat4_file", help="Input .aedat4 filename")
    parser.add_argument("--output_dir", "-o", default=".", help="Output directory for the .h5 file (default: current directory)")
    parser.add_argument("--output_name", "-n",default=None,help="Output filename (without extension).")
    parser.add_argument( "--zero_ts", "-z", action="store_true", help="Subtract first timestamp so that ts[0] == 0")
    args = parser.parse_args()

    # prepare reader
    reader = dv.io.MonoCameraRecording(args.input)
    print(f"[+] Opened {args.input} (camera: {reader.getCameraName()})")

    # accumulate
    timestamps, xs, ys, ps = [], [], [], []
    while reader.isRunning():
        batch = reader.getNextEventBatch()
        if batch is None:
            continue
        for e in batch:
            timestamps.append(e.timestamp())
            xs.append(e.x())
            ys.append(e.y())
            ps.append(1 if e.polarity() else 0)

    if len(timestamps) == 0:
        print("[-] No events read; exiting.")
        return

    # zero-base timestamps?
    if args.zero_ts:
        t0 = timestamps[0]
        timestamps = [t - t0 for t in timestamps]
        print(f"[+] Zero-based timestamps applied (t0 = {t0})")

    # determine output name
    basename = os.path.splitext(os.path.basename(args.input))[0]
    output_name = args.output_name or basename

    # save to h5
    save_events_h5(timestamps, xs, ys, ps, out_dir=args.output_dir, out_fname=output_name)

if __name__ == "__main__":
    main()
