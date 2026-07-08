#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import os

# ========== 标定板参数 ==========
CHESSBOARD_SIZE = (13, 8)      # 内角点 (列-1, 行-1)
SQUARE_SIZE = 2.0              # 格子边长 cm
IMAGE_DIR = "./calib_images3"
IMAGE_EXT = "jpg"

# ========== 准备数据 ==========
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

images = sorted(glob.glob(os.path.join(IMAGE_DIR, f"*.{IMAGE_EXT}")))
if not images:
    print(f"未找到图片: {IMAGE_DIR}")
    exit(1)

print(f"共 {len(images)} 张图片，开始检测角点...")

successful = 0
failed_list = []

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"跳过无法读取: {fname}")
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 尝试检测棋盘格
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        successful += 1
        print(f"[{successful:2d}] {fname} 检测成功")
    else:
        failed_list.append(fname)
        print(f"[失败] {fname} 未检测到棋盘格")

# 打印失败文件列表
if failed_list:
    print(f"\n有 {len(failed_list)} 张未检测到角点：")
    for f in failed_list:
        print(f"  {f}")
    print("建议：删除或重新拍摄这几张，确保棋盘格完整、光照均匀。")

if successful < 10:
    print(f"成功图片太少 ({successful})，至少需要 15 张。")
    exit(1)

print(f"\n成功检测 {successful} 张，开始标定...")

# ========== 相机标定 ==========
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n========== 标定结果 ==========")
print(f"重投影误差: {ret:.4f}")
print("内参矩阵:\n", mtx)
print("畸变系数:\n", dist.ravel())

# ========== 保存结果 ==========
np.savez("camera_params.npz", mtx=mtx, dist=dist)
print("\n内参已保存到 camera_params.npz")

# 保存为 YAML（方便 ROS 使用）
try:
    import yaml
    calib_data = {
        "image_width": gray.shape[1],
        "image_height": gray.shape[0],
        "camera_name": "IMX219_1920x1080",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": mtx.flatten().tolist()
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": dist.flatten().tolist()
        },
        "reprojection_error": float(ret)
    }
    with open("camera_params.yaml", "w") as f:
        yaml.dump(calib_data, f, default_flow_style=False)
    print("内参已保存到 camera_params.yaml")
except ImportError:
    print("未安装 pyyaml，跳过 YAML 保存，你可以之后用 numpy 转换。")

# 生成一张去畸变示例图（保存到文件，不显示）
if images:
    test_img = cv2.imread(images[0])
    h, w = test_img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(test_img, mtx, dist, None, newcameramtx)
    x, y, w_roi, h_roi = roi
    dst = dst[y:y+h_roi, x:x+w_roi]
    cv2.imwrite("undistorted_example.jpg", dst)
    print("去畸变示例已保存到 undistorted_example.jpg")