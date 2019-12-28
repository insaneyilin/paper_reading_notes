# A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement

hdl_graph_slam 的论文

日本 AIST 和 Toyohashi University of Technology（丰桥技术科学大学）的工作

要解决的问题：measure and analyze people behavior to design systems which interact with people

People Detection and Tracking，利用行人穿戴 3d lidar 进行建图、定位、感知

The observer carries the backpack with a 3D LIDAR and follows the persons to be measured.

传感器：velodyne HDL-32e

## Offline Environmental Mapping

### 使用 Graph SLAM 方法

1. first estimate the sensor trajectory by iteratively applying NDT (Normal Distributions Transform) scan matching between consecutive frames. 得到 pose nodes 和 odometry edges(即相邻 pose 之间的 relative pose)
2. loop detection. 候选 loop edge 判断，根据 distance thresh 和 accumulated distance thresh 来判断，dist thresh 是防止 loop edge 边长太大，accumulated dist thresh 是防止 loop edge 太短（可以画个三角形帮助理解）
3. loop detection，利用 NDT 判断出真正的 loop edge，根据 ndt score 判断。

这里几个参数要注意，dist_thresh, accum_dist_thresh, ndt_fitness_score_thresh.
(需要看一下 hdl_graph_slam 里默认参数是啥，特别是 ndt_fitness_score_thresh，感觉 ndt score 不一定是个靠谱的评价标准)

这里 pose graph 中的约束有：

- Odometry Constraint
- Loop Closure Constraint
- GPS constraint
- Floor Plane constraint

对于大规模建图，时间长了高度误差会导致地图“弯曲”，室内场景用地面约束，室外场景用 GPS 约束。

### Ground Plane Constraint

height filtering + RANSAC

假设地面水平，detect the ground plane every 10 seconds and connect the corresponding sensor pose node with the fixed ground plane node.

参考 <Cpa-slam: Consistent plane model alignment for direct rgb-d slam> 建立地面约束误差项

注意，这里假设第 0 帧地面是完全水平的。

### GPS Constraint

室外场景，假设地面水平是不合理的；借助 GPS 进行约束。

- first transform GPS data into the UTM (Universal Transverse Mercator) coordinate, where a GPS data has easting, northing, and altitude values in a Cartesian coordinate.
- Then, each GPS data is associated with the pose node, which has the closest timestamp to the GPS data, as an unary edge of the prior position information.
（如果我们的 GPS 位置精度不够，可以只考虑使用 GPS 的高度值）

### SLAM framework evaluation

比较了 BLAM(berkeley localization and mapping) 和 LeGO-LOAM

室内场景下，BLAM 和 lego loam 建图都在中途失败了。

## Online People Behavior Measurement

这一部分暂时没看。

## 总结

这篇文章的作者在工程上做的工作非常棒，代码清晰易懂。

