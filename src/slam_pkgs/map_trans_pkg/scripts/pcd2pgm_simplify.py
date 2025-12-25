#!/usr/bin/env python3
# pcd2pgm_simple.py
# Simple PCD → 2D Occupancy Grid (PGM + YAML)
# Only keeps points in a given height range and marks cells as occupied if they contain points.

import os
import numpy as np
import open3d as o3d
import cv2

def pcd_to_pgm(
    pcd_path,
    pgm_path,
    yaml_path,
    resolution=0.05,
    z_min=0.1,
    z_max=0.5,
    margin=1.0  # extra border around points (meters)
):
    # 1. Load point cloud
    print(f"Loading {pcd_path}...")
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    if len(points) == 0:
        raise ValueError("Point cloud is empty!")

    # 2. Clip by height
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    points = points[mask]
    print(f"Kept {len(points)} points after height clipping ({z_min} ~ {z_max} m)")

    if len(points) == 0:
        raise ValueError("No points remain after height filtering!")

    # 3. Get bounds
    x_min, x_max = points[:, 0].min(), points[:, 0].max()
    y_min, y_max = points[:, 1].min(), points[:, 1].max()

    # Add margin
    x_min -= margin
    x_max += margin
    y_min -= margin
    y_max += margin

    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))

    print(f"Map size: {width}x{height} @ {resolution}m/pixel")
    print(f"Origin: ({x_min:.3f}, {y_min:.3f})")

    # 4. Create empty grid
    grid = np.full((height, width), 255, dtype=np.uint8)  # unknown

    # 5. Compute grid indices
    ix = np.floor((points[:, 0] - x_min) / resolution).astype(int)
    iy = np.floor((points[:, 1] - y_min) / resolution).astype(int)

    # Clip indices to grid bounds
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix = ix[valid]
    iy = iy[valid]

    # 6. Mark occupied cells
    grid[iy, ix] = 0  # occupied (black in PGM)

    # Optional: mark free space around occupied (simple inflation)
    # kernel = np.ones((5,5), np.uint8)
    # grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, kernel)  # small inflation

    # 7. Flip vertically for correct PGM orientation (ROS origin bottom-left)
    grid = cv2.flip(grid, 0)

    # 8. Save PGM (note: OpenCV expects rows x cols, which matches our grid)
    cv2.imwrite(pgm_path, grid)
    print(f"Saved PGM: {pgm_path}")

    # 9. Save YAML
    yaml_content = f"""image: {os.path.basename(pgm_path)}
resolution: {resolution}
origin: [{x_min:.6f}, {y_min:.6f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"Saved YAML: {yaml_path}")


if __name__ == "__main__":
    # ----------------------- CONFIG -----------------------
    # Adjust these paths and parameters as needed
    root_dir = os.path.join(os.path.dirname(__file__), "../../../r2n_bringup_pkg")
    pcd_file = os.path.join(root_dir, "pcds", "test.pcd")
    pgm_file = os.path.join(root_dir, "maps", "test.pgm")
    yaml_file = os.path.join(root_dir, "maps", "test.yaml")

    os.makedirs(os.path.dirname(pgm_file), exist_ok=True)

    # Height range to keep (ground level, avoid ceiling/floor noise)
    Z_MIN = 0.1  # meters above sensor/ground
    Z_MAX = 2.0   # meters

    # Grid resolution (common: 0.05m = 5cm)
    RESOLUTION = 0.05

    # -----------------------------------------------------
    pcd_to_pgm(
        pcd_path=pcd_file,
        pgm_path=pgm_file,
        yaml_path=yaml_file,
        resolution=RESOLUTION,
        z_min=Z_MIN,
        z_max=Z_MAX,
        margin=1.0
    )