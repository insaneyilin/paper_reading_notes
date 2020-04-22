# A Baseline for 3D Multi-Object Tracking

中文论文解读参考：

https://zhuanlan.zhihu.com/p/80689721

主要是扩展了官方KITTI 2D MOT评估方法，增加了 3D MOT 指标。

类似于 2D 上的 SORT 和 Deep SORT，算法本身非常简单，所以是 "baseline"。

主要贡献：

- 提供简单而准确的3D MOT baseline 在线和实时应用系统。
- 将3D MOT评估方法扩增进KITTI 2D MOT评估中; 并提出两个MOT鲁棒性提出新评价指标（average MOTA and MOTP），这两个指标目的是解决当前单一 trajectory confidence threshold的问题，此评价方法平均了多个threshold，更好的反应了MOTA与MOTP的特性。 

整个方法pipeline，主要分为五阶段:

1. 3D目标检测模块提供LiDAR点云的边界框
2. 3D卡尔曼滤波器将对象状态预测到当前帧
3. 数据关联模块匹配预测轨迹与当前帧检测框
4. 3D卡尔曼滤波器基于测量结果更新对象状态
5. birth&death 记忆模块控制新出现和消失的轨迹

3D 检测直接用了现成的模型（PointRCNN，Monocular 3D Object Detection）。

Kalman Filter 使用 3D constant velocity 模型。

数据关联用匈牙利匹配。

birth&death 记忆，轨迹生命周期管理。

论文里提出了 CLEAR Metrics 的局限，没有考虑 object 的 confidence。为了提高 mota，很多方法会先卡一个 confidence 阈值来过滤 FP。这样做没办法看出来 confidence 对指标的影响。作者分析了不同 confidence 情况下 MOTA 等指标的变化：

> We define a set of confidence thresholds based on the recall value between 0 to 1 with an interval of 0.025. This results in 40 confidence thresholds in total excluding the confidence threshold which corresponds to the recall of zero. For each confidence threshold, we evaluate the tracking results using only trajectories with confidence higher than the threshold.

每个轨迹的 confidence：we define the confidence score of an object trajectory as the average confidence of the object across all frames.

AMOTA 和 AMOTP 指标，类似于 Average Precision.

> the AMOTA and AMOTP metrics are computed by integrating the MOTA and MOTP over all recall values.

Scaled Accuracy Metric: sAMOTA. 归一化，作者发现 MOTA 有一个小于 100% 的 upper bound。

---

总体感觉没什么亮点，优点是简单、有代码，改改可以拿来做 baseline 和评测工具。
