#!/usr/bin/env python3
import sys
import numpy as np

path = sys.argv[1]
with open(path, "rb") as stream:
    while True:
        line = stream.readline()
        if line.startswith(b"DATA "):
            if line.strip() != b"DATA binary":
                raise RuntimeError("Expected uncompressed binary PCD")
            break
    cloud = np.fromfile(stream, dtype=np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")
    ]))

print("points", len(cloud))
for axis in "xyz":
    values = cloud[axis]
    print(axis, "min/max", float(values.min()), float(values.max()),
          "percentiles", np.percentile(values, [0.1, 1, 50, 99, 99.9]).tolist())

