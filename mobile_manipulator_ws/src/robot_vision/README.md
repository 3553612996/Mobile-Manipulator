# robot_vision

相机图像输入、YOLO/RKNN 推理与目标三维定位。

## 规划中的节点

`vision_detector`：

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| 订阅 | `/camera/color/image_raw` (`sensor_msgs/Image`) | 30 Hz，`BEST_EFFORT` |
| 发布 | `/detections` (`vision_msgs/Detection2DArray`) | 10 Hz，`BEST_EFFORT` |
| 发布 | 目标三维坐标 / TF | 深度图 + 相机内参 + 手眼标定 |

## 依赖

- OpenCV、`cv_bridge`、`image_transport`
- RKNN runtime（RK3566 NPU，板端专有库，不入库）
- YOLO 模型权重不入库，生成方式见 `models/README.md`

## 约束

- 推理耗时不能阻塞其他回调，使用多线程执行器与 `ReentrantCallbackGroup`；
- 相机内参与手眼标定文件必须带设备序列号和标定日期；
- 检测不到目标时发布空数组，不发布过期结果。

## 当前状态

M1 只有包骨架。推理节点在 M7（YOLO 训练与 RKNN 部署）实现。
