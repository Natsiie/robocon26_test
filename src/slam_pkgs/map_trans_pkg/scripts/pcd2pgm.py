#!/usr/bin/env python3
# map_trans_pkg/scripts/pcd2pgm_optimized.py

import os
import numpy as np
import open3d as o3d
from tqdm import tqdm
import time
import cv2


def get_yaml_str(target_resolution, origin_x, origin_y):
    return f"""
resolution: {target_resolution}
origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
"""


def get_pcd_range(points):
    if len(points) == 0:
        raise ValueError("Point cloud is empty")

    x_min, y_min, z_min = np.min(points, axis=0)
    x_max, y_max, z_max = np.max(points, axis=0)
    return x_min, x_max, y_min, y_max, z_min, z_max


def build_elevation_grid_fast(points, x_min, y_min, elv_size, target_resolution, ratio):
    """
    完全去除 Python 双循环的大版本优化。
    使用 np.lexsort + 分段处理替代 object 数组。
    """

    H, W = elv_size
    cell_res = target_resolution / ratio  # elv 精度

    # 栅格索引
    ix = ((points[:, 0] - x_min) / cell_res).astype(np.int32)
    iy = ((points[:, 1] - y_min) / cell_res).astype(np.int32)
    valid_mask = (ix >= 0) & (ix < H) & (iy >= 0) & (iy < W)

    ix, iy, zs = ix[valid_mask], iy[valid_mask], points[:, 2][valid_mask]

    # 组合一个排序 key：先按 ix，再按 iy
    sort_idx = np.lexsort((iy, ix))
    ix_s = ix[sort_idx]
    iy_s = iy[sort_idx]
    zs_s = zs[sort_idx]

    # 查找 cell 分界
    cell_ids = ix_s * W + iy_s
    cell_change = np.where(np.diff(cell_ids) != 0)[0] + 1
    segments = np.split(np.arange(len(zs_s)), cell_change)

    # 输出 elv_grid
    elv_grid = np.full((H, W), -np.inf, dtype=np.float32)

    # 取每个 cell 内 top-K 均值（你原先 K=5）
    K = 5

    for seg in segments:
        cx = ix_s[seg[0]]
        cy = iy_s[seg[0]]
        zvals = zs_s[seg]

        # 选取最高 K 个
        if len(zvals) > K:
            top_k = np.partition(zvals, -K)[-K:]
            elv_grid[cx, cy] = np.mean(top_k)
        else:
            elv_grid[cx, cy] = np.mean(zvals)

    return elv_grid


def classify_grid_fast(elv_grid, passable_thresh, kernel=5):
    valid_mask = (elv_grid != -np.inf).astype(np.uint8)
    kernel_mat = np.ones((kernel, kernel), np.uint8)

    # 有效点统计
    count_map = cv2.filter2D(valid_mask.astype(np.float32), -1, kernel_mat)
    unknown_mask = count_map < kernel * kernel * 0.2

    # 局部 min/max 用 OpenCV 加速
    max_map = cv2.dilate(elv_grid, kernel_mat)
    min_map = cv2.erode(elv_grid, kernel_mat)
    height_diff = max_map - min_map

    # 分类结果完整图
    pgm_full = np.zeros_like(elv_grid, dtype=np.uint8)
    pgm_full[height_diff <= passable_thresh] = 255  # passable
    pgm_full[unknown_mask] = 127  # unknown

    # 下采样到 PGM 尺寸
    pgm = pgm_full[::2, ::2]
    return pgm


if __name__ == "__main__":
    root = os.path.join(os.path.dirname(__file__), "../../../r2n_bringup_pkg")
    pcd_file = f"{root}/pcds/test.pcd"
    pgm_file = f"{root}/maps/test.pgm"
    yaml_file = f"{root}/maps/test.yaml"

    target_resolution = 0.10  
    ELV_TO_PGM_RATIO = 2.0 
    elv_scan_kernel_size = 5
    passable_thresh = 0.1

    os.makedirs(os.path.dirname(pgm_file), exist_ok=True)

    print("Loading PCD file...")
    start = time.time()
    pcd = o3d.io.read_point_cloud(pcd_file)

    points = np.asarray(pcd.points)
    print(f"Loaded {len(points)} points in {time.time() - start:.2f}s")

    x_min, x_max, y_min, y_max, z_min, z_max = get_pcd_range(points)
    print(
        f"PCD range: x[{x_min:.2f}, {x_max:.2f}], y[{y_min:.2f}, {y_max:.2f}], z[{z_min:.2f}, {z_max:.2f}]"
    )

    # PGM 尺寸
    pgm_width = int((x_max - x_min) / target_resolution)
    pgm_height = int((y_max - y_min) / target_resolution)
    pgm_size = (pgm_width, pgm_height)

    # ELV 尺寸按比例
    elv_size = (int(pgm_width * ELV_TO_PGM_RATIO), int(pgm_height * ELV_TO_PGM_RATIO))
    print(f"PGM grid size = {pgm_size}, ELV grid size = {elv_size}")

    # 构建高程图
    t0 = time.time()
    elv_grid = build_elevation_grid_fast(
        points, x_min, y_min, elv_size, target_resolution, ELV_TO_PGM_RATIO
    )
    print(f"Elevation grid built in {time.time() - t0:.2f}s")

    # 分类
    t0 = time.time()
    pgm_grid = classify_grid_fast(elv_grid, passable_thresh, elv_scan_kernel_size)
    print(f"Classification completed in {time.time() - t0:.2f}s")

    # 保存 PGM
    cv2.imwrite(pgm_file, pgm_grid.T)

    # 保存 YAML
    yaml_str = get_yaml_str(target_resolution, x_min, y_min)
    with open(yaml_file, "w") as f:
        f.write(yaml_str)

    print(f"Map saved to {pgm_file} and {yaml_file}")
