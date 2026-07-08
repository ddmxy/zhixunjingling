# SVM 人脸识别小作业

## 方法说明

1. **人脸定位**：OpenCV Haar 级联检测器
2. **特征提取**：HOG（方向梯度直方图）+ 灰度直方图
3. **分类器**：`sklearn.svm.SVC`（RBF 核）
4. **输入**：笔记本摄像头实时画面

## 环境

```bash
pip install -r requirements.txt
```

## 使用步骤

### 1. 采集训练数据

```bash
python face_svm.py collect
```

| 按键 | 作用 |
|------|------|
| `1` | 保存为 person_a（可设为自己） |
| `2` | 保存为 person_b（同学/照片） |
| `3` | 保存为 person_c（可选） |
| `s` | 保存为 unknown（非目标人脸） |
| `q` | 退出 |

建议每人采集 **20~30 张**，稍微转动头部、变一下表情。

### 2. 训练 SVM

```bash
python face_svm.py train
```

终端会打印准确率、`precision/recall` 报告，并保存模型到 `model/face_svm.joblib`。

### 3. 实时识别

```bash
python face_svm.py detect
```

绿框 = 识别为已知人脸；红框 = unknown。

## 作业报告可写内容

- SVM 原理：寻找最大间隔超平面，适合中小规模特征分类
- HOG 特征：描述局部边缘方向，常用于行人/人脸检测
- 实验结果：粘贴 `train` 输出的 classification_report
- 不足：光照变化、侧脸、样本少时准确率下降
