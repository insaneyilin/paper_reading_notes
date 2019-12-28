# A Precise and Robust Segmentation-Based Lidar Localization System for Automated Urban Driving

Momenta 和同济大学一起发的文章

low-level semantic segmentation-based multiple types feature extraction algorithms

ground, road-curb, edge, and surface

pre-build feature point cloud map

L-M (Levenberg-Marquardt) optimization

the lidar localization results were fused with MEMS IMU data through a state-error Kalman filter

思路是用 Low level 方法提取道路的特征，如地面、路沿、道路边界等，然后和点云地图匹配。

这篇文章的 related works 部分介绍了一些定位比较新的进展：

- 视觉 SLAM: ORB-SLAM2, LDSO(DSO with loop closure)
- Lidar SLAM: LOAM, LeGO-LOAM, IMLS-SLAM

实际应用中，使用先验高精度地图是必要的：

It is necessary to establish a high-precision map in advance and realize accurate localization through
matching online frames with the priori map

Fast LIDAR localization using multiresolution Gaussian mixture maps.

Gaussian mixture maps: a collection of Gaussian mixture over the z-height distribution

这里评价了下百度 2017 msf 定位的论文：

This method relies on the intensity information of point cloud, but it is not easy to calibrate the Lidar intensity, and each Lidar has its own differences.


整体流程：

(1) Feature Extraction，去除动态物体，low level 分割提取道路特征；下面的处理都是基于这些特征
(2) Lidar Odometry, frame-frame matching was performed to obtain the ego-motion between two adjacent frames. The result was used as the initial value of frame-map matching in Lidar localization module.
(3) Local Map Load: the local prior feature map is loaded dynamically based on the current vehicle localization information.
(4) Lidar Localization: the current frame was matched with the loaded local map to obtain an accurate global pose and pushback the result into the filter.
(5) Error-state Kalman Filter. 融合原始 IMU 和 Lidar 定位结果。filter 初始化完成后，将 filter 结果作为 lidar localization frame-map matching 的初值，停止 lidar odometry。

特征提取，把原始点云转换成 organized point cloud，具体做法是转成一张 2d range image，每一行对应每一根线，每一列对应一个方位角

点云地面分割，基于法向量和梯度进行 region growing 的方法计算开销较大。

基于上面的 2d range image, 观察到 The differences in the z direction was far less than the x, y directions between two adjacent points in the same column and the former one was close to 0. 根据这个观察到的几何性质，定义了两个指标来判断点是否是地面点。对于 32 线，只对小于 17 的行做判断（因为只有这些线才能扫到地面）（我发现同济这一系列文章特别喜欢按照线的顺序处理点云）

curb 检测，The differences in the z direction was far less than the x, y directions between two adjacent points in the same column (the two points in the purple box of Figure 4) and the former one was close to 0. 基于 curb 平行于 lidar 的 x 轴或 y 轴，主车转弯时失效，解决方案是记录 yaw 角度对 lidar 点云进行变换，保持变换后的点云里 curb 平行于 lidar 的 x 轴或 y 轴。

surface 检测，定义一个指标来描述点的 smoothness。直观想法就是借鉴梯度思想。

edge 检测，the stable edge feature included light poles, tree trunks, architecture ridges, etc., all of which are vertical. 这里没有继续使用 2d range image，因为列方向的连续性不好。使用去除 ground 和 surface 后的点云，使用 x-y grid-based clustering，聚类后做一个 line fitting with RANSAC. 如果 Line 平行于 z 轴，则认为是一个 edge。

提取完特征，接下来要考虑的就是 frame-frame matching 和 frame-map matching。

category matching: we only found correspondences in the same category, because the structure of the feature points was stable.

Consider priori information: 简单来说就是匹配时充分利用特征的先验信息，比如对于 edge，可以直接认为其方向向量就是 (0, 0, 1)，对于 ground 可以直接认为其法向量就是 (0, 0, 1)，主要是为了降低计算复杂度。

LM 优化，LOAM 直接对 6DOf 参数进行了整体的优化。本文指出不同特征对不同参数的作用不一样，比如 ground points 只对 t_z, pitch 和 roll 有较强的约束，edge 对 t_x, t_y 有较好的约束，不应该全都放在一起优化。multi-step LM 优化，coarse to fine.


error state KF: The error-state was the difference between the estimated state and truth state, including position, orientation, velocity, gyroscopes biases, and accelerometers biases.

With Lidar pose measurements that estimated by Lidar frame-map matching, we update the error-state Kalman Filter’s state and then used the error-state to correct the SINS state as an update model in the Kalman filter.

实验部分，benchmark 建立和 msf 那篇提到的差不多，也是离线处理。精度 3~5cm，只需要 lidar 和 IMU。
