# Learning to Localize Using a LiDAR Intensity Map

UBER ATG 的论文，用 learning 的方法处理 lidar 反射值地图

Learning to Localize Using a LiDAR Intensity Map

A promising alternative to these methods is to leverage LiDAR intensity maps [7, 8], which encode information about the appearance and semantics of the scene. However, the intensity of commercial LiDARs is inconsistent across different beams and manufacturers, and prone to changes due to environmental factors such as temperature.

Therefore, intensity based localization methods rely heavily on **having very accurate intensity calibration of each LiDAR beam**.

This requires careful **fine-tuning of each vehicle** to achieve good performance, sometimes even on a daily basis.

we design a deep network that embeds both LiDAR intensity maps and online LiDAR sweeps in a common space where calibration is not required.

Localization is then simply done by searching exhaustively over 3-DoF poses (2D position on the map manifold plus rotation), where the score of each pose can be computed by the cross-correlation between the embeddings.

这种方法可以很容易利用 GPU。

related works

SLAM: 通过闭环来消除漂移问题，但是高速场景，往往无环可闭。或者利用 GPS 融合，但是 GPS 达不到厘米级精度。

Localization Using Light-weight Maps：用 google map 或者 baidu map 这种轻量级地图。精度有限。

Localization Using High-definition Maps：高精度定位地图，2d 反射值/高度值，或者 3D 地图

Matching Networks：Convolutional matching networks that compute similarity between local patches.

Learning to Localize：直接学 pose，室内场景研究比较多。

## 3 Robust Localization Using LiDAR Data

formulate localization as deep recursive Bayesian estimation problem

不是用单帧 lidar 数据来定位，而是 aggregate the k most recent LiDAR sweeps using the IMU and wheel odometry. Since k is small, drift is not an issue.

## 总结

比较偏实验性质的论文，用 learning 方法来做定位要实际使用，要考虑的东西还很多。
