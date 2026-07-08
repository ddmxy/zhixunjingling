#!/usr/bin/env python3
from hobot_vio import libsrcampy
import numpy as np
import cv2

cam = libsrcampy.Camera()
ret = cam.open_cam(0, 0, 30, 1920, 1080)   # 第2个参数是 video_index
if ret != 0:
    print("摄像头打开失败，先重启设备或关闭其他摄像头程序")
    exit(1)

# 抓取一帧
img_bytes = cam.get_img(0)
if len(img_bytes) == 0:
    print("未收到图像数据")
    exit(1)

print(f"数据长度: {len(img_bytes)} 字节")

# 将原始字节 reshape 为 (高度, 宽度, 2) 的交错格式
yuv = np.frombuffer(img_bytes, dtype=np.uint8).reshape((1080, 1920, 2))

# 尝试四种转换
conversions = {
    "YUYV": cv2.COLOR_YUV2BGR_YUY2,
    "UYVY": cv2.COLOR_YUV2BGR_UYVY,
    "YVYU": cv2.COLOR_YUV2BGR_YVYU,
    "NV12_upsample": None   # 手动上采样作为对照
}

for name, code in conversions.items():
    if code is not None:
        bgr = cv2.cvtColor(yuv, code)
    else:
        # 手动 NV12 上采样（不常用，但可作为参考）
        y_size = 1920*1080
        y = np.frombuffer(img_bytes[:y_size], dtype=np.uint8).reshape((1080,1920))
        uv = np.frombuffer(img_bytes[y_size:], dtype=np.uint8).reshape((540,960,2))
        u = cv2.resize(uv[:,:,0], (1920,1080), interpolation=cv2.INTER_LINEAR)
        v = cv2.resize(uv[:,:,1], (1920,1080), interpolation=cv2.INTER_LINEAR)
        yuv444 = cv2.merge([y, u, v])
        bgr = cv2.cvtColor(yuv444, cv2.COLOR_YUV2BGR)

    filename = f"/tmp/test_{name}.jpg"
    cv2.imwrite(filename, bgr)
    print(f"已保存: {filename}")

cam.close_cam()
print("所有测试图片已保存到 /tmp/")