# ORB-SLAM: A Versatile and Accurate Monocular SLAM System

作者开篇就提出 BA 是 三维重建和相机定位的 gold standard method。

BA 计算开销大，一般用于离线处理。但近年来也有人基于 BA 做 real time 的应用，但这些工作都局限于 Visual Odometry。

本文提出了一个 Monocular SLAM system，实时，有闭环检测，全自动初始化。

中文翻译参考：

https://blog.csdn.net/weixin_42905141/article/details/102857958

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

### A. Place Recognition

we proposed a bag of words place recognizer built on DBoW2  with ORB , which are binary features invariant to rotation and scale (in a limited range),

- <Bags of binary words for fast place recognition in image sequences, IEEE Transactions on Robotics, vol. 28, no. 5, pp. 1188–1197, 2012.>
- <ORB: an efficient alternative to SIFT or SURF>

### B. Map Initialization

单目 SLAM 系统需要进行 map 的初始化，因为不可能从一幅图像中恢复出 depth 。

We present in Section IV a new automatic approach based on model selection between a homography for planar scenes and a fundamental matrix for non-planar scenes. A statistical approach to model selection was proposed by Torr et al. [26]

一种初始化的方法：track a known structure，<MonoSLAM: Real-time single camera SLAM>

Two View 初始化方法，一般假设局部平面场景，从 homography 中来恢复相机 pose；或者通过计算 essential matrix （5点法），但可能有多个解。

如果视差过小，那么基于三位重建的初始化方法很容易误差过大；反之，如果非平面的场景在有足够视差的情况下被观察到，那么可以利用 8 点法求 Fundamental Matrix 来恢复相机姿态。

本文提出了一种自动初始化方法：

- 平面场景下用 homography
- 非平面场景下用 fundamental matrix。

delay the initialization until the method produces a unique solution with significant parallax.

### C. Monocular SLAM

单目视觉 SLAM 最早普遍采用滤波的方法，相机姿态和 map 中的点都作为状态变量。滤波方法的缺点：连续帧提供的新信息有限，容易受 **累积误差** 影响。

与滤波方法相对的是基于关键帧的方法，可以利用 BA 进行求解（关键帧是 **稀疏** 的，而不像滤波方法基于连续帧），且 mapping 过程不与帧率耦合在一起。

PTAM 是非常有代表性的基于关键帧的单目 SLAM 。PTAM 提出了 tracking 和 mapping 分离为两个线程。使用 FAST 角点，对 tracking 有帮助，但是对 place recognition 帮助有限。PTAM 没有大的闭环检测，re-localization 只是基于关键帧分辨率来校正。

<Scale drift-awarelarge scale monocular SLAM> 前端用光流 + FAST 特征 + motion only BA，后端用 sliding-window BA，闭环使用 pose graph 优化（7 DoF，可以纠正 scale）。

ORB SLAM 借用了 7 DoF pose graph 优化的思想。

LSD SLAM，基于像素，可以生成半稠密地图。闭环仍然要给予图像特征，精度比 PTAM 要低很多。

SVO，介于基于特征和基于像素方法之间，没有闭环，且目前的实现主要是考虑 downward looking cameras。

关键帧选择问题，基于关键帧来做 VSLAM 基本是目前的共识，如何选择关键帧？本文提出了一种 survival of fittest 策略。

## 3. System Overview

特征点选择：

- 实时性，排除 SIFT,SURF 和 AKAZE
- 旋转不变性，排除 BRIEF, LDB

ORB：快，有旋转不变性。

三个并行的 threads：

- tracking
- local mapping
- loop closing

One of the main properties of the system is that the same ORB features that are triangulated by the mapping and tracked by the tracking are used for place recognition to perform loop detection and relocalization

This allows to match them from wide baselines, boosting the accuracy of BA.

If the tracking is lost (e.g. due to occlusions, abrupt movement), the place recognition module is used to perform a global relocalization.

- tracking 中，首先当前帧与上一帧进行特征匹配，利用 motion-only BA 对 POSE 进行优化。如果 tracking 失败，运行 place recognition 模块，进行 global relocalization（全局重定位）。一旦有了相机姿态和特征匹配的初始估计，可以从系统维护的关键帧之间的共见图来获得一个局部的 map。Then, matches with the local map points are searched by reprojection, and camera pose is optimized again with all matches.
- local mapping 的过程处理新的关键帧，同时实施 local BA 来优化相机姿态，新关键帧中未匹配的 ORB 特征利用共见图来查找匹配和三角化新的点。tracking 过程中，还应用了一种 point culling 的策略来只保留高质量的点。
- loop closing 对每一个新的关键帧都查找是否有 loop。如果检测到 loop，计算一个相似变换来表示 drift。最后，进行 pose graph 优化，新颖的地方在于这里的 graph 是 Essential Graph，即从共见图中提取出的稀疏子图。

整个系统中的所有优化利用 g2o 的 LM 算法进行优化。

Map 中的点、关键帧以及它们的选择。

每个地图点保存如下信息：

1. 三维点的世界坐标
2. viewing direction，which is the mean unit vector of all its viewing directions (the rays that join the point with the optical center of the keyframes that observe it);
3. ORB 描述子，保留同其他所有特征点的 descriptor 相比 hamming 距离最小的那个 descriptor。
4. 该点能够被观察到的最大、最小距离（according to the scale invariance limits of the ORB features.）

每个关键帧保存如下信息：

1. 相机姿态，刚性变换 from the world to the camera coordinate system
2. 相机内参，焦距、光心
3. 该帧图像中所有的 ORB 特征点，可以与 map 点进行关联（非必须），如果有畸变系数，应该提供去畸变（undistort）后的坐标。

Map 点和关键帧的加入是比较宽松的，但之后会进行严格的 culling 策略（去除冗余信息，保留高质量的点）

关于 Covisibility Graph 和 Essential Graph

共见图：undirected weighted graph，参考 <Double window optimisation for constant time visual SLAM>. 每个节点是一个帧，如果两个帧之间有相同的观察点（至少 15 个），增加一条边，边的权重为相同观察点的数量

为了纠正一个 loop，利用 pose graph optimization，参考 <Scale drift-aware large scale monocular SLAM>. 共见图可能会比较 dense，为了提高优化效率，本文建立 Essential Graph，其保留共见图的所有节点，但是减少边数。增量构建 spanning tree ，这样可以提供一个最小边数的连通子图。Essential Graph 包含这个 spanning tree，其边具有较高的共见度（相同观察点至少 100 个）。

BoW Place Recognition. 利用 DBoW2 来进行闭环的检测，然后 relocalization。利用 BoW 还可以加速特征匹配。<Bags of binary words for fast place recognition in image sequences>

## 4. 自动地图初始化

map 初始化的目标：计算两帧之间的相对 pose，三角化初始的 map 点。

初始化应该不依赖于场景（平面或一般场景），而且好的初始化不应该需要人工的操作。（PTAM 需要人工操作初始化，即平移操作来获取有足够 parallax 的两帧图像）


本文利用两种模型：

- homography，针对平面场景
- fundamental matrix，针对一般场景

利用启发式方法进行模型选择。


具体步骤：

1. 找初始匹配（initial correspondence），提取 ORB 特征（only at the finest scale），当前帧 F_c ，参考帧 F_r。匹配关系 x_c <-> x_r。如果没有足够的匹配，重置参考帧
2. 并行(parallel)计算两种模型，即一个线程计算 homography 模型，一个线程计算 fundamental matrix 模型。homography 用 normalized DLT 计算， fundamental matrix 用 8 点法计算，同时也使用 RANSAC 。

对每种模型计算一个 score S_M，M \in {H, F}

这里参考 <MVG> 蓝皮书。

哪个模型的 score 高就用哪个模型进行初始化，如果两种模型都失败（没有足够的 inliers），回到第一步。

3. 模型选择：平面场景，近似平面或者低 parallax 情况下，选择 homography，此时也可以计算 fundamental matrix，但是 not well constrained （有多解）；非平面场景，选择 fundamental，但是此时 homography 也可以计算，因为匹配点中可能有一些子集恰好处于同一平面。使用经验公式（启发式）进行选择。

4. SfM。恢复 motion 和 structure，对于 homography，有八组解，<Motion and structure from motion in a piecewise planar environment>。本文的做法，对八组解（即 R T）都分别进行三角化，看那组解三角化得到的点“最好”（check if there is one solution with most points seen with parallax, in front of both cameras and with low reprojection error）。如果没有足够好的解，回到第一步。对于 fundamental matrix，我们利用内参矩阵 K 转化为 essential matrix，然后用 SVD 分解，有四组解，用同样的策略选出足够“好”的解（同 homography，先三角化，再检查）

5. BA。对于初始化的结果，实施一次 full BA。

## 5. Tracking

这一部分介绍 tracking thread（对于每一帧都进行 track），涉及的优化为 motion only BA。

### A. ORB 特征提取

We extract FAST corners at eight-scale levelswith a scale factor of 1.2. For image resolutions from 512×384 to 752×480 pixels we found suitable to extract 1000 corners, for higher resolutions, as the 1241×376 in the KITTI dataset , we extract 2000 corners.

对于每一个 scale level，划分出一个 grid，尽可能使得每个 cell 至少包含 5 个 corners。然后在每个 cell 中通过调整 detector threshold 使得其有足够多的 corners。对于 retained FAST corners，计算 orientation 和 ORB descriptor。区别于 PTAM，The ORB descriptor is used in all feature matching, in contrast with the search by patch correlation in PTAM.


### B. 从前一帧获取初始 Pose 估计

如果从上一帧 tracking 成功，利用一个 constant velocity motion model 来预测相机姿态（常速率运动模型），以及进行上一帧 map points 的 guided search。如果没有足够的匹配（违反了常速率运动模型），we use a wider search of the map points around their position in the last frame 。最后利用找到的 correspondences 进行 pose 优化

### C. 从 Global Relocalization 获取初始 Pose 估计

如果 tracking is lost，将当前帧转为 BoW 然后查询 recognition database，从而进行 global relocalization。

We compute correspondences with ORB associated with map points in each keyframe, as explained in Section III-E. We then perform alternatively RANSAC iterations for each keyframe and try to find a camera pose using the PnP algorithm [41]. If we find a camera pose with enough inliers, we optimize the pose and perform a guided search of more matches with the map points of the candidate keyframe. Finally, the camera pose is again optimized, and if supported with enough inliers, tracking procedure continues.

### D. Track Local Map

一旦有了相机姿态的估计以及初始的特征匹配，可以将 map 投影回到 frame 来 search more map point correspondence。为了限制复杂度，本文只投影 local map。这个 local map 包含一个 K_1 集合，即那些与当前帧共享 map points 的关键帧，一个 K_2 集合，从 K_1 集合中利用共见图查找得到的 neighbors。local map 还有一个 K_ref \in K_1，即共享最多 map points 的帧。在 K_1 和 K_2 中的每一个 map point 通过如下步骤来在当前帧中查找：

1. 计算 map point 到当前 frame 的投影，如果超出图像边界，舍弃
2. 计算当前 view ray 方向 v 和 map point 的平均 view 方向 n 之间的夹角，如果 v \cdot n < cos(60 度)，舍弃
3. 计算当前帧相机中心到 map point 的距离，如果 d 不在 [d_min, d_max] 内，舍弃
4. 计算 d/d_min（对于 d_min, d_max 的意义不懂），Compute the scale in the frame by the ratio d/dmin
5. Compare the representative descriptor D of the map point  with the still unmatched ORB features in the frame, at the predicted scale, and near x, and associate the map point with the best match

最后用所有在当前 frame 中找到的 map points 进行 pose 优化（motion only BA）

### E. 新关键帧选择

最后一步是决定当前帧是否能够作为新的 keyframe。在 local mapping 中，有 cull 冗余帧的机制；在 tracking 中，希望能尽可能快得加入关键帧。下面是加入 keyframe 需要满足的条件：

1. 距离上一次 global relocalization 至少跳过 20 帧
2. local mapping 处于空闲状态，或者距离上一个关键帧至少跳过 20 帧
3. 当前帧至少 track 了 50 个点
4. 当前帧 track 的点数少于 K_ref 中的点数的 90 %。  （足够的视点变化，一定程度避免冗余）

## 6. Local Mapping

local mapping 对于每一个 keyframe 都会实施。


### A. Keyframe Insertion

首先，更新共见图，增加一个新的 node，同时更新边信息（利用 shared map points from other keyframes）。然后更新 spanning tree，update the spanning tree  linking K_i with the keyframe with most points in common。然后计算当前帧的 BoW。

### B. Recent Map Point Culling

Map points, in order to be retained in the map, must pass a  restrictive test during the first three keyframes after creation.

一个 map point 必须满足如下两个条件：

1. The tracking must find the point in more than the 25% of the frames in which it is predicted to be visible. （不理解什么是 predicted to be visible）
2. If more than one keyframe have passed from map point creation, it must be observed fromat least three keyframes

Once a map point have passed this test, it can only be removed  if at any time it is observed from less than three keyframes. This can happen when keyframes are culled and when local BA discards outlier observations. This policy makes our map contain very few outliers.

### C. New Map Point Creation

新的 map point，通过对 ORB 特征点（从共见图中相连接的关键帧 K_c）进行三角化可以创建，对于每一个未匹配的 ORB 特征，在另一个 keyframe 中的未匹配的 ORB 特征中进行查找。舍弃不满足极线约束的匹配。ORB 特征三角化后，对 depth（应该为正数）、parallax 、重投影误差、scale consistency 进行检查。初始时，一个 map point 只被两帧观察到，但之后它可能在其他更多帧中被匹配到，因此应该将其投影到 connected keyframes 中进行 search。

### D. Local BA

The local BA optimizes the currently processed keyframe  K_i , all the keyframes connected to it in the covisibility graph  K_c , and all the map points seen by those keyframes. All other keyframes that see those points but are not connected to the currently processed keyframe are included in the optimization but remain fixed. Observations that are marked as outliers are discarded at the middle and at the end of the optimization.

### E. Local Keyframe Culling

local mapping 需要检测冗余的 keyframes，并且删除它们。

We  discard all the keyframes in K_c whose 90% of the map points have been seen in at least other three keyframes in the same or finer scale. 

## 7. Loop Closing

The loop closing thread takes K_i , the last keyframe processed  by the local mapping, and tries to detect and close loops.

### A. Loop Candidates Detection

在covisibility graph中根据 bow 特征来找候选的关键帧

### B. Compute the Similarity Transformation

单目SLAM系统有7个自由度，3个平移，3个旋转，1个尺度因子。我们需要计算从当前关键帧Ki到回环关键帧Kl的相似变换，以获得回环的累积误差。

### C. Loop fusion

融合重复的地图云点，在covisibility graph中插入与回环相关的的新边。

### D. Essential Graph optimization

通过Essential Graph优化位姿图，将回环闭合的误差分散到图像中去。

优化过后，每一个地图云点都根据关键帧的校正进行变换。

## 8. 实验

在 Newcollege 、TUM 和 Kitti 上进行了评测。

## 9. 结论

PTAM后端是BA优化，这是众所周知的离线SFM（从运动到结构）问题[2]的经典解法。PTAM算法和Mouragnon[3]早期作品的主要贡献是将BA算法引入到机器人SLAM框架下，并具有良好的实时性。

本文的主要贡献是将PTAM算法的适用性进一步扩展，使其可以应用于原来不可应用的场景下。为了实现这一目标，我们整合了前面几年的优秀作品，引入新的想法和算法，从头设计了一种新的单目SALM系统所用到的技术包括Gálvez-López和Tardós提出的论文[5]中的闭环检测，Strasdat等人在论文[6],[7]中提出的的闭环检测程序和covisibility graph，Kuemmerle等人在论文[37]中提出的g2o优化框架以及Rubble等人提出的ORB特征[9]。

我们还展示了ORB特征具有很好的识别能力，可识别剧烈视角变换情况下的场景信息。此外，它们能够被非常快速的提取和匹配（不需要多线程或GPU加速），这就使得跟踪和地图构建更加实时精确。

直接法，直接利用图像像素的亮度信息进行摄像头的定位与优化，并重构稠密或半稠密的环境地图。直接方法不需要特征提取，可以避免人工匹配。他们对图像模糊，弱纹理环境和像论文[45]这样的高频纹理环境的鲁棒性更好。

直接方法有他们自己的局限。首先，这些方法假设真实场景中的物体的像是由该物体本身的表面反射模型产生的，因此，算法采用的光度一致性寻找匹配点的思路就限制了匹配点之间的基线距离，通常都比特征匹配点的基线要窄。这对重构的精度影响很大，因为重构需要较宽的基线来减少深度的不确定性。如果直接建模不准确，则可能会受到快门，自动增益和自动曝光的影响（如TUM RGB-D 的对比测试）。最后，由于直接方法计算要求较高，因此为了满足计算速度，DTAM算法采用地图增量式扩张的方法，而LSD-SLAM则丢掉传感器测量信息，将地图优化降低为对位姿图的优化。

基于特征的方法可以在更宽的基线（baseline）上匹配特征，主要得益于特征匹配算法较好地视图不变特性。BA优化和相机位姿优化，地图云点通过传感器测量进行融合。在运动结构估计中，论文[46]已经指出了基于特征的方法相比直接方法的优势。在我们的实验第8部分B节中也直接提供了证据，表明基于特征的定位精度更高。未来单目SLAM应该会整合两种最好的方法。

系统的精度可以通过结合无限远点跟踪来进一步增强。这些在视图中看不到的平行线交点，并没有包含在本文算法构建的地图中，但对相机的旋转非常有用[21]。

另外一种方法是将稀疏地图更新到一个更加稠密的地图。由于我们关键帧的选择机制，关键帧组成了一个紧凑的地图，地图具有非常高精度的位姿信息和丰富的covisibility信息。所以，ORB-SLAM稀疏地图是一个非常优秀的初始估计框架，比稠密地图更好。

