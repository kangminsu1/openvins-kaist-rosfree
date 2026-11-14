#!/usr/bin/env python3
import pathlib
import statistics
import sys

import cv2
import numpy as np

root = pathlib.Path(sys.argv[1])
stamps = [int(x) for x in (root / "sensor_data/stereo_stamp.csv").read_text().splitlines() if x]
indices = [len(stamps) * n // 6 for n in range(1, 6)]

def read_calib(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    values = tuple(fs.getNode(k).mat() for k in (
        "camera_matrix", "distortion_coefficients", "rectification_matrix", "projection_matrix"
    ))
    fs.release()
    return values

k1, d1, r1, p1 = read_calib(root / "calibration/left.yaml")
k2, d2, r2, p2 = read_calib(root / "calibration/right.yaml")
size = (1280, 560)
map1 = cv2.initUndistortRectifyMap(k1, d1, r1, p1[:, :3], size, cv2.CV_32FC1)
map2 = cv2.initUndistortRectifyMap(k2, d2, r2, p2[:, :3], size, cv2.CV_32FC1)
baseline = abs(float(p2[0, 3] / p2[0, 0]))
print(f"fx={p1[0,0]:.6f} baseline={baseline:.6f} m")

orb = cv2.ORB_create(3000)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
all_dy, all_dx = [], []

for index in indices:
    stamp = stamps[index]
    left = cv2.imread(str(root / "image/stereo_left" / f"{stamp}.png"), cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(str(root / "image/stereo_right" / f"{stamp}.png"), cv2.IMREAD_GRAYSCALE)
    left = cv2.remap(left, *map1, cv2.INTER_LINEAR)
    right = cv2.remap(right, *map2, cv2.INTER_LINEAR)
    kp1, des1 = orb.detectAndCompute(left, None)
    kp2, des2 = orb.detectAndCompute(right, None)
    matches = sorted(matcher.match(des1, des2), key=lambda m: m.distance)
    matches = matches[: min(600, len(matches))]
    dx = [kp1[m.queryIdx].pt[0] - kp2[m.trainIdx].pt[0] for m in matches]
    dy = [kp1[m.queryIdx].pt[1] - kp2[m.trainIdx].pt[1] for m in matches]
    good = [(x, y) for x, y in zip(dx, dy) if abs(y) < 3 and 0 < x < 256]
    all_dx.extend(x for x, _ in good)
    all_dy.extend(y for _, y in good)
    print(f"frame={index} matches={len(matches)} geometric_good={len(good)}")

print(f"good_matches={len(all_dy)}")
print(f"median_abs_vertical_error={statistics.median(abs(v) for v in all_dy):.4f} px")
print(f"vertical_error_under_1px={100*sum(abs(v)<1 for v in all_dy)/len(all_dy):.2f}%")
print(f"median_positive_disparity={statistics.median(all_dx):.4f} px")

