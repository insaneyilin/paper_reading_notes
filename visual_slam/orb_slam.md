# ORB-SLAM: A Versatile and Accurate Monocular SLAM System

作者开篇就提出 BA 是 三维重建和相机定位的 gold standard method。

BA 计算开销大，一般用于离线处理。但近年来也有人基于 BA 做 real time 的应用，但这些工作都局限于 Visual Odometry。

本文提出了一个 Monocular SLAM system，实时，有闭环检测，全自动初始化。

## 1. Introduction

BA 是 the optimal method to estimate camera localization and a sparse geometrical reconstruction of a scene from a set of images.

但 BA 计算复杂度高，过去一般不在 real-time SLAM 中考虑使用。

a real time SLAM algorithm has to provide BA with:

- Corresponding observations of scene features (map points) among a subset of selected frames (keyframes).
- As complexity is cubic in the number of keyframes, their selection should avoid unnecessary redundancy.
- A strong network configuration of keyframes and points to produce accurate results, that is, a well spread set of keyframes observing points with significant parallax and with plenty of loop closure matches.
- An initial estimation of the keyframe poses and point locations for the non-linear optimization.

- 关键帧之间要有相对应的场景特征点（帧间特征点匹配）
- 关键帧的选择应该避免冗余
- 要有分布良好的关键帧，这些关键帧能提供良好的特征匹配，使得有显著的 parallax （视差）以及闭环的匹配
- BA 的非线性优化需要提供良好的初始估计（初值，初始化）
- local map 的维护，实现 scalability
- 能够快速进行 global optimization (pose graph), 进行实时的闭环优化

第一个实时的使用 BA 的 SLAM 算法是 PTAM。

本文主要的思路来自于 PTAM。主要参考文章：

- <Bags of binary words for fast place recognition in image sequences, IEEE Transactions on Robotics, vol. 28, no. 5, pp. 1188–1197, 2012.>
- <Scale drift-aware large scale monocular SLAM., Robotics: Science and Systems (RSS), Zaragoza, Spain, June 2010.>
- <Double window optimisation for constant time visual SLAM, IEEE International Conference on Computer Vision (ICCV), Barcelona, Spain, November 2011, pp. 2352–2359.>

本文 SLAM 系统的主要特点：

- 使用 ORB 特征，利用 bag of words 模型进行 recognition
- 大环境中的实时操作，thanks to the use of a covisibility graph to focus tracking and mapping operations in a local covisible area, independent of global map size.
- Real time loop closing based on the optimization of a pose graph that we call the Essential Graph, which is built from a spanning tree that is maintained by the system, the loop closure links and some strong edges from the covisibility graph.
- Real time camera relocalization with high invariance to viewpoint.
- A new automatic and robust initialization procedure based on model selection that permits to create an initial map from planar and non-planar scenes.
- A survival of the fittest approach to map point and keyframe selection that is little conservative in the spawning but very restrictive in the culling. This improves tracking robustness as many keyframes are created under hard conditions in exploration (i.e. strong rotations), while the culling allows to maintain a compact reconstruction of the scene as redundant keyframes are not retained, which enhances lifelong operation.
- 共见图 covisibility graph, 关注局部共见区域的 tracking 和 mapping，不考虑全局 map 的规模
- Essential Graph，实现实时闭环优化，一种从共见图中生成的 pose graph （利用 spanning tree）
- 自动初始化
- map 点和关键帧选择，避免冗余

闭环检测方法：

- [11] R. Mur-Artal and J. D. Tardos, “Fast relocalisation and loop closing in keyframe-based SLAM,” in Proc. IEEE Int. Conf. Robot. Autom., Hong Kong, Jun. 2014, pp. 846–853.

## 2. Related works
