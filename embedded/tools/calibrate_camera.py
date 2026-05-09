#!/usr/bin/env python3
"""
Camera calibration using an OpenCV checkerboard pattern.

Captures images from a live camera (or reads from a folder), detects
checkerboard corners, and runs cv2.calibrateCamera() to produce the
intrinsic parameters needed by somars-embedded's target_localizer:

    camera_fx, camera_fy, camera_cx, camera_cy

Usage
-----
  # Live capture (press SPACE to capture, Q when done):
  python3 calibrate_camera.py --camera 0 --cols 9 --rows 6 --square-size 0.025

  # From a folder of images:
  python3 calibrate_camera.py --images ./calibration_images --cols 9 --rows 6 --square-size 0.025

Output
------
  Prints the parameters ready to paste into config/params.yaml and
  saves the full calibration (camera matrix + distortion) to
  calibration_result.yaml.

Checkerboard
------------
  Print a checkerboard pattern (e.g. 10x7 squares → inner corners = 9x6).
  Measure the side length of one square in meters (e.g. 25 mm = 0.025).
  Take 15-30 images from varying angles and distances.
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np


def find_corners(image, board_size):
    """Detect checkerboard corners in a grayscale image."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if found:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return found, corners, gray


def capture_from_camera(camera_index, board_size, min_captures=15):
    """Interactively capture calibration frames from a live camera."""
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Error: cannot open camera {camera_index}")
        sys.exit(1)

    frames = []
    print(f"Capturing from camera {camera_index}")
    print(f"  Board inner corners: {board_size[0]}x{board_size[1]}")
    print(f"  SPACE = capture frame, Q = finish (need >= {min_captures})")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        display = frame.copy()
        found, corners, _ = find_corners(frame, board_size)
        if found:
            cv2.drawChessboardCorners(display, board_size, corners, found)
            cv2.putText(display, "DETECTED - press SPACE", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "No board found", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"Captured: {len(frames)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' ') and found:
            frames.append(frame.copy())
            print(f"  Captured frame {len(frames)}")
        elif key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    return frames


def load_images_from_folder(folder, board_size):
    """Load images from a directory."""
    exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    paths = []
    for ext in exts:
        paths.extend(glob.glob(os.path.join(folder, ext)))
    paths.sort()

    if not paths:
        print(f"Error: no images found in {folder}")
        sys.exit(1)

    frames = []
    for p in paths:
        img = cv2.imread(p)
        if img is not None:
            found, _, _ = find_corners(img, board_size)
            if found:
                frames.append(img)
                print(f"  {os.path.basename(p)}: board detected")
            else:
                print(f"  {os.path.basename(p)}: no board found, skipping")
    return frames


def calibrate(frames, board_size, square_size):
    """Run OpenCV camera calibration on collected frames."""
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size

    obj_points = []
    img_points = []
    img_size = None

    for frame in frames:
        found, corners, gray = find_corners(frame, board_size)
        if not found:
            continue
        obj_points.append(objp)
        img_points.append(corners)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])

    if len(obj_points) < 5:
        print(f"Error: only {len(obj_points)} valid frames, need at least 5")
        sys.exit(1)

    print(f"\nCalibrating with {len(obj_points)} frames...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None)

    return ret, camera_matrix, dist_coeffs, img_size


def main():
    parser = argparse.ArgumentParser(description="Camera calibration for SOMARS")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--camera", type=int, help="Camera index for live capture")
    group.add_argument("--images", type=str, help="Folder of calibration images")
    parser.add_argument("--cols", type=int, required=True,
                        help="Inner corners per row (e.g. 9 for a 10-column board)")
    parser.add_argument("--rows", type=int, required=True,
                        help="Inner corners per column (e.g. 6 for a 7-row board)")
    parser.add_argument("--square-size", type=float, default=0.025,
                        help="Checkerboard square side length in meters (default: 0.025)")
    parser.add_argument("--output", type=str, default="calibration_result.yaml",
                        help="Output YAML file for full calibration result")
    args = parser.parse_args()

    board_size = (args.cols, args.rows)

    if args.camera is not None:
        frames = capture_from_camera(args.camera, board_size)
    else:
        frames = load_images_from_folder(args.images, board_size)

    if len(frames) < 5:
        print(f"Error: only {len(frames)} frames captured, need at least 5")
        sys.exit(1)

    rms, mtx, dist, img_size = calibrate(frames, board_size, args.square_size)

    fx = mtx[0, 0]
    fy = mtx[1, 1]
    cx = mtx[0, 2]
    cy = mtx[1, 2]

    print(f"\n{'='*60}")
    print(f"  Calibration complete  (RMS reprojection error: {rms:.4f})")
    print(f"{'='*60}")
    print(f"  Image size: {img_size[0]}x{img_size[1]}")
    print(f"  fx = {fx:.2f}")
    print(f"  fy = {fy:.2f}")
    print(f"  cx = {cx:.2f}")
    print(f"  cy = {cy:.2f}")
    print(f"  Distortion: {dist.ravel()}")
    print()
    print("  Paste into config/params.yaml:")
    print()
    print("  target_localizer:")
    print("    ros__parameters:")
    print(f"      camera_fx: {fx:.1f}")
    print(f"      camera_fy: {fy:.1f}")
    print(f"      camera_cx: {cx:.1f}")
    print(f"      camera_cy: {cy:.1f}")
    print(f"\n{'='*60}")

    # Save full result
    fs = cv2.FileStorage(args.output, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", img_size[0])
    fs.write("image_height", img_size[1])
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.write("rms_reprojection_error", rms)
    fs.release()
    print(f"\n  Full calibration saved to: {args.output}")


if __name__ == "__main__":
    main()
