# LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain

LeGO-LOAM, a lightweight and ground-optimized lidar odometry and mapping method

- first apply point cloud segmentation to filter out noise, and feature extraction to obtain distinctive planar and edge features.
- A two-step Levenberg-Marquardt optimization method then uses the planar and edge features to solve different components of the six degree-of-freedom transformation across consecutive scans.

两步Levenberg-Marquardt优化方法：地面提取的平面特征用于在第一步中获得 [t_z,θ_roll,θ_pitch] ,在第二步中，通过匹配从分段点云提取的边缘特征来获得其余部分的变换[t_x,t_y,θ_yaw]。

LOAM: 高频低精度运动估计 + 低频高精度匹配、建图

LeGO LOAM 优点：

- 轻量级
- 分割地面，去除不可靠特征点
- 地面优化，两步优化位姿
- 集成了回环检测以校正运动估计漂移的能力

本文使用 VLP-16 lidar。VLP-16测量范围达100米，精度为±3厘米。它具有30°（15°）的垂直FOV和360°的水平FOV。16通道传感器提供2°的垂直角分辨率。水平角分辨根据旋转速率不同，从0.1°到0.4°变化。在整篇论文中，我们选择10Hz的扫描速率，其提供0.2°的水平角分辨率。

点云分割 -> 特征提取 -> Lidar Odometry -> Lidar Mapping -> Transform Integration （Lidar Odometry 和 Lidar Mapping 类似 LOAM，也是并行的）

点云分割：先将点云投影到 range image 上， 在分割之前进行地面图像的逐列评估，其可以被视为，用于地面点提取。在此过程之后，可能代表地面的点被标记为地面点而不用于分割。对点云进行分组聚类，仅保留可表示大对象（例如树干）和地面点的点。

an image-based segmentation method [23] is applied to the range image to group points into many clusters. 注意，这里地面点视为一种特殊的聚类。

每个点有 3 个属性：

1. its label as a ground point or segmented point
2. its column and row index in the range image
3. its range value.

特征提取过程类似于Zhang Ji的论文[20]，但不从原始点云提取，而是从地面点和 segmented points 提取特征。

对 range image 划分 sub images，根据 roughness of point 提取边特征和平面特征

Lidar Odometry. estimates the sensor motion between two consecutive scans. The transformation between two scans is found by performing point-to-edge and point-to-plane scan-matching. 特征匹配的方法基本上同 LOAM 一致，但做出了一些改进：

- Label Matching，只对相同标签的点进行匹配（平面-平面，边-边，地面-地面）。
- 两步 LM 优化。

lidar mapping. matching features to a surrounding point cloud map to further refine the pose transformation, runs at a lower frequency. 参考 LOAM。LeGO LOAM 的主要区别在于点云存储方式的不同。保存每个单独的特征集，而不是保存单个点云图。

通过特征集合得到点云地图有两种方式，第一种类似 LOAM：选择距离当前传感器位置100m以内的特征集合。选择的特征集合然后变换和融合到单个点云地图（t时刻）。第二种是结合 pose graph 优化，图的节点：每个特征集合的传感器位姿，特征集合被看做为这个节点上的传感器测量数据；雷达建图模型的位姿估计drift很低，假设在短时间内没有drift。通过选择一组最近的特征集合来构成点云图；新节点和已有节点之间加上空间约束（通过L-M优化得到的坐标变换）；可以增加 loop closure 进一步消除 drift，如果用ICP发现当前特征集和先前特征集之间有匹配，则添加新约束。

实验部分，LOAM 失败的 case 主要是由于 (1) 环境缺少特征 (2) noisy objects, e.g. grass and trees。LeGO LOAM 由于做了地面优化和点云分割的改进，效果更好。

KITTI 上的评测，对数据集的 HDL-64E 点云做了降采样（we downsample the scan from the HDL-64E to the same range image that is used in Section III for the VLP-16）

参考：

https://blog.csdn.net/wykxwyc/article/details/89605721

