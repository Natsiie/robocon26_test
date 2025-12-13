#!/usr/bin/env python3
# virtual_lidar_driver_pkg/scripts/map_generator.py
"""
优化版：向量化上表面点生成、减少数组重分配、保留并轻微优化侧面生成。
主要优化点：
 - 用 numpy 向量化计算每个像素需要的采样数并一次性分配内存
 - 避免在循环中频繁 vstack/hstack，使用 np.repeat + 单次生成随机数
 - 只对边缘使用循环（边缘通常很稀疏）
 - 保留体素滤波（Open3D）以控制后处理点数
 - 可选 numba 加速（注释）
"""
import os
import cv2
import numpy as np
import open3d
from tqdm import tqdm
from scipy import ndimage
import time
from math import ceil

# 可选：如果安装了 numba，可以用来加速边缘循环（不是必须）
# try:
#     from numba import njit
# except Exception:
#     njit = lambda f: f


def auto_calculate_parameters(resolution, base_voxel_size=0.05):
    """
    根据分辨率自动计算其他参数，确保参数协调
    """
    voxel_size = max(0.01, min(0.1, resolution * 1.5))
    noise_std = voxel_size * 0.7
    surface_thickness = max(voxel_size * 2, 0.03)
    accessible_height = 0.1
    return {
        "voxel_size": voxel_size,
        "noise_std": noise_std,
        "surface_thickness": surface_thickness,
        "accessible_height": accessible_height,
    }


def png_to_pcd_enhanced(
    png_path,
    pcd_path,
    resolution=0.05,
    center_pixel=(100, 100),
    floor_thickness=0.3,
    generate_pgm_yaml=False,
    map_name="sim_map",
    auto_calculate=True,
    max_points_cap=None,  # 可选：限制最大点数以避免 OOM（None表示不限制）
    **manual_params,
):
    """
    优化后的点云生成函数。
    返回值：生成的 points (N,3) numpy array
    """
    start_time = time.time()
    print(f"正在读取图像: {png_path}")
    img = cv2.imread(png_path)
    if img is None:
        raise FileNotFoundError(f"无法读取图像: {png_path}")

    h, w = img.shape[:2]
    cx, cy = center_pixel

    # BGR -> 顶部高度（米）
    color_to_top_z = {
        (0, 0, 0): None,  # 黑 → 无 / 墙
        (255, 255, 255): 0.05,  # 白 -> floor (very low)
        (0, 0, 255): 0.25,  # 红
        (0, 255, 0): 0.45,  # 绿
        (255, 0, 0): 0.8,  # 蓝
    }

    # 参数
    if auto_calculate:
        params = auto_calculate_parameters(resolution)
        params.update(manual_params)
    else:
        params = {
            "voxel_size": manual_params.get("voxel_size", 0.05),
            "noise_std": manual_params.get("noise_std", 0.015),
            "surface_thickness": manual_params.get("surface_thickness", 0.05),
            "accessible_height": manual_params.get("accessible_height", 0.3),
        }

    voxel_size = params["voxel_size"]
    noise_std = params["noise_std"]
    surface_thickness = params["surface_thickness"]
    accessible_height = params["accessible_height"]

    print("参数:")
    print(
        f"  voxel_size: {voxel_size:.4f} m, noise_std: {noise_std:.4f} m, surface_thickness: {surface_thickness:.4f} m"
    )

    # 构建高度/占据/分类图（向量化）
    print("构建高度/占据图（向量化）...")
    height_map = np.full((h, w), -1.0, dtype=np.float32)
    floor_map = np.zeros((h, w), dtype=bool)
    occupancy_map = np.full((h, w), 254, dtype=np.uint8)  # 默认空闲254
    bgr = img.astype(np.int32)

    # 为每个颜色创建 mask（向量化）
    for (b, g, r), top_z in color_to_top_z.items():
        mask = (bgr[:, :, 0] == b) & (bgr[:, :, 1] == g) & (bgr[:, :, 2] == r)
        if not np.any(mask):
            continue
        if top_z is None:
            height_map[mask] = -1.0
            occupancy_map[mask] = 0
        else:
            # treat white as floor (special-case)
            if abs(top_z - 0.05) < 1e-6:
                floor_map[mask] = True
                height_map[mask] = 0.0
                occupancy_map[mask] = 254
            else:
                height_map[mask] = float(top_z)
                occupancy_map[mask] = np.where(top_z > accessible_height, 0, 254)

    valid_mask = height_map >= 0

    # 边缘检测（保留你原来的边缘检测思路）
    print("检测边缘...")
    edge_kernel = np.array([[1, 1, 1], [1, -8, 1], [1, 1, 1]])
    height_valid = np.where(valid_mask, height_map, 0.0)
    edges = ndimage.convolve(height_valid, edge_kernel, mode="constant", cval=0.0)
    edge_mask = (np.abs(edges) > 0.1) & valid_mask

    # 世界坐标网格（向量化）
    print("构建世界坐标网格（向量化）...")
    x_coords = np.arange(w)
    y_coords = np.arange(h)
    xx, yy = np.meshgrid(x_coords, y_coords)  # shape (h,w)
    wx_grid = (xx - cx) * resolution
    wy_grid = (cy - yy) * resolution

    # -------------------------
    # 1) 向量化生成“上表面”点云
    # -------------------------
    print("向量化生成上表面点云...")
    ys, xs = np.where(valid_mask)
    if ys.size == 0:
        print("没有有效高度像素，退出。")
        return np.zeros((0, 3), dtype=np.float32)

    heights_flat = height_map[ys, xs]
    wx_flat = wx_grid[ys, xs]
    wy_flat = wy_grid[ys, xs]
    is_floor_flat = floor_map[ys, xs]

    # 计算 z_start, z_end, delta
    z_start = np.where(
        is_floor_flat,
        -floor_thickness,
        np.maximum(0.0, heights_flat - surface_thickness),
    )
    z_end = np.where(is_floor_flat, 0.0, heights_flat)
    delta = z_end - z_start

    # 需要采样的点数（按 voxel_size），若 delta<=0 -> 0（不生成）
    # 但为了避免图层全被忽略，我们对于 delta<=0 设为0（在你原逻辑里会略过）
    raw_n = np.where(delta > 0, np.ceil(delta / voxel_size).astype(int), 0)
    # 保证每个像素至少 1 个点（如果 delta>0），否则 0
    raw_n = np.where(delta > 0, np.maximum(1, raw_n), 0)

    total_surface_points = int(raw_n.sum())
    print(f"将生成上表面点数（估计）: {total_surface_points:,}")

    if total_surface_points == 0:
        surface_points = np.zeros((0, 3), dtype=np.float32)
        surface_intensities = np.zeros((0,), dtype=np.float32)
    else:
        # 可选：限制最大总点数以避免 OOM（用户可传 max_points_cap）
        if max_points_cap is not None and total_surface_points > max_points_cap:
            scale = max_points_cap / total_surface_points
            # 对每个像素的采样数随机下采样使总数接近 cap
            rng = np.random.default_rng()
            mask_keep = rng.random(raw_n.shape) < scale
            raw_n = raw_n * mask_keep
            total_surface_points = int(raw_n.sum())
            print(f"应用点数上限后，上表面点数: {total_surface_points:,}")

        # 按像素重复 wx,wy,z_start,z_end,dist_to_edge等
        repeats = raw_n
        # 过滤掉 repeats==0 的像素
        keep_idx = repeats > 0
        ys_k = ys[keep_idx]
        xs_k = xs[keep_idx]
        wx_k = wx_flat[keep_idx]
        wy_k = wy_flat[keep_idx]
        z_start_k = z_start[keep_idx]
        z_end_k = z_end[keep_idx]
        repeats_k = repeats[keep_idx]

        # 计算距离图用于 intensity（到图边的距离归一化）
        dist_to_edge_pixel = np.minimum(
            np.minimum(xs_k, w - 1 - xs_k), np.minimum(ys_k, h - 1 - ys_k)
        ).astype(np.float32) / max(w, h)
        edge_factor_pixel = 1.0 - np.exp(-dist_to_edge_pixel * 5.0)
        base_intensity_pixel = 0.3 + 0.7 * edge_factor_pixel

        # repeat arrays
        wx_rep = np.repeat(wx_k, repeats_k)
        wy_rep = np.repeat(wy_k, repeats_k)
        z_start_rep = np.repeat(z_start_k, repeats_k)
        z_end_rep = np.repeat(z_end_k, repeats_k)
        intensity_base_rep = np.repeat(base_intensity_pixel, repeats_k)

        # 生成 z 值和噪声（一次性）
        rng = np.random.default_rng()
        u = rng.random(total_surface_points)  # uniform [0,1)
        z_vals = z_start_rep + u * (z_end_rep - z_start_rep)

        noise = rng.normal(0.0, noise_std, size=(total_surface_points, 3))
        pxs = wx_rep + noise[:, 0]
        pys = wy_rep + noise[:, 1]
        pzs = z_vals + noise[:, 2]

        surface_points = np.stack([pxs, pys, pzs], axis=1)
        surface_intensities = intensity_base_rep  # 已经是 (N,)

    # -------------------------
    # 2) 侧面点云（边缘处） - 保持循环（通常稀疏），但做小优化
    # -------------------------
    print("生成侧面点云（边缘处）...")
    edge_ys, edge_xs = np.where(edge_mask)
    side_points_list = []
    side_intens_list = []

    # 一个局部函数：为一个像素生成侧面点（x方向与y方向）
    def gen_side_for_pixel(x, y):
        cur_h = height_map[y, x]
        if cur_h < 0:
            return None, None
        y_min, y_max = max(0, y - 1), min(h, y + 2)
        x_min, x_max = max(0, x - 1), min(w, x + 2)
        neigh = height_map[y_min:y_max, x_min:x_max]
        valids = neigh[neigh >= 0]
        if valids.size == 0:
            return None, None
        min_h = float(np.min(valids))
        max_h = float(np.max(valids))
        if max_h <= min_h:
            return None, None

        wx = (x - cx) * resolution
        wy = (cy - y) * resolution

        side_pts = []
        side_ints = []

        # 每一侧的点数
        num_side_pts = max(1, int(ceil((max_h - min_h) / voxel_size)))

        # X 方向两侧
        side_xs_coords = [wx - resolution / 2.0, wx + resolution / 2.0]
        for sx in side_xs_coords:
            z_side = np.random.random(num_side_pts) * (max_h - min_h) + min_h
            noise_side = np.random.normal(0.0, noise_std * 1.2, size=(num_side_pts, 3))
            px = sx + noise_side[:, 0]
            py = wy + noise_side[:, 1]
            pz = z_side + noise_side[:, 2]
            # intensity: 更中间高度强度更高
            height_factor = (pz - min_h) / (max_h - min_h + 1e-9)
            intensity = 0.5 + 0.5 * (1.0 - np.abs(height_factor - 0.5) * 2.0)
            side_pts.append(np.stack([px, py, pz], axis=1))
            side_ints.append(intensity)

        # Y 方向两侧
        side_y_coords = [wy - resolution / 2.0, wy + resolution / 2.0]
        for sy in side_y_coords:
            z_side = np.random.random(num_side_pts) * (max_h - min_h) + min_h
            noise_side = np.random.normal(0.0, noise_std * 1.2, size=(num_side_pts, 3))
            px = wx + noise_side[:, 0]
            py = sy + noise_side[:, 1]
            pz = z_side + noise_side[:, 2]
            height_factor = (pz - min_h) / (max_h - min_h + 1e-9)
            intensity = 0.5 + 0.5 * (1.0 - np.abs(height_factor - 0.5) * 2.0)
            side_pts.append(np.stack([px, py, pz], axis=1))
            side_ints.append(intensity)

        if not side_pts:
            return None, None
        combined_pts = np.concatenate(side_pts, axis=0)
        combined_ints = np.concatenate(side_ints, axis=0)
        return combined_pts, combined_ints

    # 循环生成边缘点（通常远少于表面点）
    for iy, ix in zip(edge_ys, edge_xs):
        pts, ints = gen_side_for_pixel(ix, iy)
        if pts is not None:
            side_points_list.append(pts)
            side_intens_list.append(ints)

    if side_points_list:
        side_points = np.concatenate(side_points_list, axis=0)
        side_intensities = np.concatenate(side_intens_list, axis=0)
    else:
        side_points = np.zeros((0, 3), dtype=np.float32)
        side_intensities = np.zeros((0,), dtype=np.float32)

    # -------------------------
    # 合并 surface + side
    # -------------------------
    if surface_points.size == 0 and side_points.size == 0:
        all_points = np.zeros((0, 3), dtype=np.float32)
        all_intens = np.zeros((0,), dtype=np.float32)
    elif surface_points.size == 0:
        all_points = side_points
        all_intens = side_intensities
    elif side_points.size == 0:
        all_points = surface_points
        all_intens = surface_intensities
    else:
        all_points = np.concatenate([surface_points, side_points], axis=0)
        all_intens = np.concatenate([surface_intensities, side_intensities], axis=0)

    point_count = all_points.shape[0]
    print(
        f"共生成点数: {point_count:,}（含噪声） 生成耗时: {time.time()-start_time:.2f}s"
    )

    # 保存点云 (Open3D)
    os.makedirs(os.path.dirname(pcd_path), exist_ok=True)
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(all_points.astype(np.float64))

    # 将强度映射到颜色（灰度）
    # 将 intens 标准化到 [0,1]（当前 intensity 已是 0~1 区间）
    ints_clamped = np.clip(all_intens, 0.0, 1.0)
    colors = np.tile(ints_clamped.reshape(-1, 1), (1, 3))
    pcd.colors = open3d.utility.Vector3dVector(colors.astype(np.float64))

    # 可选体素滤波
    if point_count > 100_000:
        voxel_size_filter = max(0.01, min(0.05, resolution * 0.8))
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size_filter)
        print(f"体素滤波后点数: {len(pcd.points):,}")

    open3d.io.write_point_cloud(pcd_path, pcd)
    print(f"已保存点云: {pcd_path}")

    # 生成PGM+YAML（用于 Nav2）
    if generate_pgm_yaml:
        pgm_dir = os.path.dirname(pcd_path).replace("pcds", "maps")
        os.makedirs(pgm_dir, exist_ok=True)
        pgm_file = os.path.join(pgm_dir, f"{map_name}.pgm")
        yaml_file = os.path.join(pgm_dir, f"{map_name}.yaml")

        pgm_data = np.full((h, w), 205, dtype=np.uint8)
        pgm_data[occupancy_map == 254] = 254
        pgm_data[occupancy_map == 0] = 0
        cv2.imwrite(pgm_file, pgm_data)

        origin_x = -cx * resolution
        origin_y = -(h - cy) * resolution
        yaml_content = f"""image: {os.path.basename(pgm_file)}
resolution: {resolution}
origin: [{origin_x:.6f}, {origin_y:.6f}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
"""
        with open(yaml_file, "w") as f:
            f.write(yaml_content.strip() + "\n")

        print(f"生成地图: PGM->{pgm_file}, YAML->{yaml_file}")

    return all_points


# ==================== 主程序 ====================
if __name__ == "__main__":
    root = os.path.join(os.path.dirname(__file__), "../../../r2n_bringup_pkg")
    file_name = "sim_map"
    png_file = os.path.join(root, f"maps/{file_name}.png")
    pcd_file = os.path.join(root, f"pcds/{file_name}.pcd")

    if not os.path.exists(png_file):
        print(f"错误：找不到 {png_file}")
        exit(1)

    pts = png_to_pcd_enhanced(
        png_path=png_file,
        pcd_path=pcd_file,
        resolution=0.01,
        center_pixel=(100, 100),
        generate_pgm_yaml=True,
        map_name=file_name,
        auto_calculate=True,
        voxel_size=0.05,
        # 可选限制，避免 OOM： max_points_cap=2_000_000
    )
    
    print("生成点云的范围为：")
    print(pts.min(axis=0), pts.max(axis=0))

    print("\n所有任务完成")
