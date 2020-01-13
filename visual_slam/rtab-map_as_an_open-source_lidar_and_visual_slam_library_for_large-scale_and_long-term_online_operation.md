# RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation

Real-Time Appearance-Based Mapping

loop closure detection with a memory management approach.

## 2 Popular SLAM Approaches Available on ROS

GMapping, TinySLAM. use particle filer to estimate the robot trajectory. GMapping 是基于粒子滤波的方法，有回环检测过程，是ROS默认的SLAM方法，广泛应用于生成2D栅格地图，地图建好后，可以配合蒙特卡洛定位方法使用。

Hector SLAM. 快速、计算开销低，无回环。

ETHZASL-ICP-Mapper. 基于 ICP ，无回环。

Karto SLAM, Google Cartographer are lidar graph-based SLAM approaches. While mapping, they create sub-maps that are linked by constraints in the graph. When a loop closure is detected, the position of the sub-maps are re-optimized to correct errors introduced by noise of the sensor and scan matching accuracy.

BLAM. UCB 的图优化 SLAM, 通过scan matching局部地进行闭环检测。

SegMatch. 3d lidar，图优化，闭环检测。闭环检测通过匹配3D分割信息（车辆，建筑，树木等）。

关于视觉 SLAM，本文只考虑带绝对尺度信息的方法（排除 PTAM、ORBSLAM、SVO 等方法）

maplab 和 VINS-Mono。二者都是 visual-inertial 方法，基于图优化。maplab 的流程：(1) VIO 记录开环数据；(2) 离线完成地图建立（闭环检测，图优化，稠密建图等）。vins-mono 采用在线建图，为了保证大规模环境的处理时间，vins-mono限制了graph的尺寸，首先移除没有闭环的节点，然后根据graph的稠密度移除其他节点。

ORB-SLAM2 和双目 PTAM。视觉 SLAM，基于图优化，闭环基于DBoW2，map优化使用BA。闭环优化和 camera tracking 并行，在单独的线程中对闭环后的graph进行优化，避免影响相机跟踪性能。闭环检测和图优化进程的耗时随着map的增长会产生明显延迟，并且map是稀疏特征，没有占用栅格和稠密地图，因此很难用在实际平台上。

DVO-SLAM, RGBiD-SLAM。直接法，不使用特征（feature）而是使用所有像素的光度和深度误差。

ElasticFusion。在线重建surfel-based地图，需要GPU。

上面的 visual slam 方法很少考虑到相机遮挡或者图像中缺少特征的情况，下面的方法考虑了这些情况：

- 多相机 PTAM(MCPTAM)
- RGBDSLAMv2

## 3 RTAB-Map Description

RTAB-Map 是基于图优化的 SLAM 方法。

传感器数据经过时间同步之后，进入STM（short term memory）模块，然后进行前端处理，后端进行图优化和全局地图维护。

Nodes are created at a fixed rate “Rtabmap/DetectionRate” set in milliseconds according to how much data created from nodes should overlap each other. 一个例子：当机器人运动快并且相机视场小的时候，这个值应该被提高，来保证连续节点的数据有重叠，但是设置太大会占用内存和耗费时间。

link 包含了两个节点之间的 rigid transform，在这里有三种link：邻居，闭环，相似。“邻居”连接是在STM里面通过odom转换建立的，“闭环”和“相似”也是通过闭环检测和相似性检测实现的。这些连接都用作图优化的约束条件。当一个新闭环或者相似产生时，图优化将计算误差传播到全图来减小里程计漂移。伴随着优化过程，Octomap，点云，2D栅格地图都在被全局更新。

RTAB-Map’ memory management approach runs on top of graph management modules. 为了防止地图变大带来的问题，rtab的内存被分成工作内存（WM）和长期内存（LTM），当一个节点被分到LTM中后，WM的流程对其不再起作用。当rtab的更新时间超过固定阈值“Rtabmap/TimeThr”，在WM的节点会被转换成LTM中来限制WM的size。同样，还有一个阈值“Rtabmap/MemoryThr”，可以设置WM中最大保持的节点数。

为了决定哪个节点被分到 LTM 中，一个权重机制决定了“哪个位置”更重要，使用一种启发式的方法：一个位置被观测的越长，它越重要，就应该放在WM中。 因此，当一个新的节点到来时，初始化它的权重为0，并且与上一个节点进行视觉相似性比较（deriving a percentage of corresponding visual words），如果它们很相似（对应的视觉单词超过相似阈值“Mem/RehearsalSimilarity“），新节点的权重等于旧节点权重加上1，同时将旧的节点权重置为零（权重传递），如果机器人没有移动，直接将旧的节点删除避免扩大graph的size。当时间或者内存满了，优先将权重最低的旧节点放到LTM里面。

当WM的一个位置发生闭环检测时，这个position的相邻节点（neighbor nodes）会被从LTM调回WM，当机器人在之前到过的地方移动时，它可以增量地记住过去的位置以扩展当前map并使用过去的位置进行定位。

### 3.1 Odometry Node

里程计节点可以使用任何一种简单的里程计方式（车轮、IMU、单相机、单雷达）。独立于使用的传感器，里程计为rtab提供至少一种位姿估计信息，以odometry message形式配合tf使用（例如/odom to /base_link），当本体传感器不准确时，视觉和雷达传感器是必需的。视觉里程计方面，rtab应用了两种标准的里程计方法，Frame-to-Map（F2M）和Frame-to-Frame（F2F）。

F2M就是当前帧和一张特征图进行关联，当然特征图有之前很多帧图像的特征（有用的、显著的）。F2F就是当前帧和上一帧进行关联。雷达传感器的scan-to-scan和scan-to-map同理于视觉（雷达的point cloud可以认为和视觉的3D visual features在前端等价）。

#### 3.1.1 Visual Odometry

- feature detection. 特征点，支持 OpenCV 中的特征，双目相机根据光流法来计算左右图像的特征视差，rgb-d相机的深度图作为GFTT的mask，避免了提取深度无效的特征。

- feature matching. 使用 BRIEF 描述子， NN search with nearest neighbor distance ratio (NNDR) test。F2M 匹配用描述子，F2F 匹配用光流（速度快）

- motion prediction. 限制feature matching过程中的搜索窗口来提供更好的匹配，尤其是在动态环境和重复的纹理特征。搜索窗口半径由”Vis/CorGuessWinSize“给出，使用常速运动模型。

- Local Bundle Adjustment. 转换结果通过对特征使用局部BA方法进行修正。在 F2M 下是将转换结果基于所有关键帧的特征上做优化，F2F 只基于上一帧的特征点。

- pose update. 更新里程计输出 pose，/odom to /base_link。

- key frame and feature map update. 如果运动估计过程中inliers的数量低于固定阈值“odom/keyframethr”，则关键帧或者特征地图被更新（这个阈值来衡量当前帧的信息是否丰富）。对于F2F，关键帧被当前帧代替，对于F2M，特征地图被更新（添加了新frame中没有匹配的特征），并且更新之前匹配过的特征点（经过BA优化后）的位置。当特征地图的size大于阈值之后，采取一些措施来维护特征地图，比如丢弃没有和当前帧匹配上的oldest特征。

如果当前相机的运动和预测值相差很多时，可能是没有找到一个有效的transformation，这时我们不用运动估计，再进行一次特征匹配：

    + 对于F2M，当前帧的特征会和特征地图的所有特征进行对比，然后会计算得到另一个transformation。
    + 对于F2F，为了提高无效结果出现时的系统鲁棒性，特征匹配使用NNDR而不是光流，然后提取BRIEF描述子。

如果transformation还是没用被计算出，里程计被认为已经丢失（之前是里程计可能丢失），并且下一帧图像会在没有运动预测的情况下进行对比。此时里程计输出的位姿会被设置为null且方差很大，这是里程计失效的信息会被模块订阅到。

#### 3.1.2 Lidar Odometry

类似 VO，匹配分为 S2S(scan to scan) 和 S2M(scan to map)

如果激光扫描仪的旋转频率相对于机器人速度较高，激光扫描的运动畸变非常小，因此可以忽略校正，而不会显著降低配准精度。

- Point Cloud Filtering. 降采样、计算法向量等，利用 tf 将点云转换到 base_link

- ICP Registration. S2S 或 S2M 配准，点云地图（point cloud map）由过去的关键帧组成。场景中平面较多时优先使用 point to plane ICP。

- Motion Prediction. 如果场景缺少几何特征信息，比如长隧道，ICP 会失败，需要提供额外的运动信息（比如轮速计或 IMU）。如果没有额外的运动信息，使用 motion prediction is done according to a constant velocity model based on the previous transformation，但这种方法容易导致漂移. 通过“环境复杂度”来判断是否需要额外运动，环境复杂度借助 点云法向量的 PCA 的特征值来判断。

- Pose Update. /odom icp -> /odom -> /base link, 更新位姿

- Key Frame and Point Cloud Map Update. 判断关键帧是根据一个相关性阈值决定，在S2S中，当前帧直接变成关键帧，但是在S2M中，当前帧要经过和原map融合的过程，添加新信息，删除旧信息（维护一个固定大小的地图）。

LO 不像 VO在 odometry lost 的情况下那么容易恢复，但一般 lidar odometry 很少 lost。

### 3.2 Synchronization

时间同步。ROS 中的时间同步策略有精确和近似两种，对于图像数据（RGB Image, Depth Image 等）先进行精确同步，再和 lidar scan 进行非精确同步。

### 3.3 STM

当新节点在STM里生成时，会得到一个局部占用栅格（local occupancy grid），基于地图的图位姿（pose of the map‘s graph）能把局部的栅格图拼成全局的，并且经过优化后的graph对建立全局栅格图很有帮助。根据传感器的不同格式可以生成不同的栅格地图，简单说就是2D数据只能生成2D栅格，3D数据可以生成2D或者3D。

### 3.4 Loop Closure and Proximity Detection

视觉闭环，基于词袋（bag-of-words）方法，STM提取新节点的视觉特征，并将它们量化为一个递增的视觉单词词汇表。特征可以用opencv中的SURF，SIFT，ORB等表示。当视觉里程计F2F或F2M被使用时，我们可以使用里程计中提取出来的特征来进行闭环检测。这节省了提取两次特征的时间。闭环检测需要的特征不需要想里程计中那么多，因此只需要之前特征集合的一个子集（典型特征组成的子集）来完成闭环检测。新建立的node和WM（work memory保留着整个过程中重要的nodes）中的nodes进行比较来检测闭环，STM里面的nodes都是最近才加入map的，因此这些节点不会用来去进行闭环检测。STM可以作为一个buffer在节点被移动至WM中。

Lidar 闭环，2017年提出的proximity detection是基于雷达scan来定位接近当前位置的nodes。 举例：在以不同的方向穿越长廊时，相机没法找到闭环，这时proximity detection就有帮助了。闭环检测的复杂度取决于WM的size，proximity detection的复杂度取决于靠近机器人的nodes数量。这些nodes在graph中必须很靠近（机器人运动增量很小），也就是说，这些nodes和最新node的连接（links）数必须小于一个阈值。

### 3.5 Graph Optimization

rtab 集成了三种图优化方法（1）TORO（2）G2O（3）GTSAM，（2）和（3）比（1）快，但是鲁棒性略差。但是经验表明，在6DoF的maps中G2O和GTSAM效果更好。

视觉闭环检测不是完全准确的，在相似的地方会触发无效的闭环检测，这反倒增加了map中的误差。为了检测到这些无效的处理，rtab使用了一个新参数，如果经过优化后，link的转换改变大于平移方差的一个因子“RGBD/optimizemaxerror”（假设两个节点理论上的link距离分布为N（0.5,0.2），而结果得到了2，明显就是优化错误。），所有基于新节点的闭环检测结果都被reject，保持闭环检测之前的graph。

### 3.6 Global Map Assembling

全局地图组装。在局部栅格地图信息上先“拼成”全局地图，然后根据后端优化结果对map进行修正。然后map发布在ros标准格式（sensor_msgs/pointcloud2）上。方便其他算法的开发和调试。

## 4 Evaluating Trajectory Performance of RTAB-Map Using Different Sensor Configurations

轨迹评估。在 4 个数据集上评测： KITTI, TUM RGBD, EuRoC, PR2 MIT Stata Center


## 5 Evaluating Computation Performance between Visual and Lidar SLAM Configurations with RTAB-Map

不同传感器、方法配置下对于 computation time, memory usage and map quality 的影响。

## 6 Discussion

In RTAB-Map, motion estimation during localization or loop closure detection is done primarily visually, then optionally refined by geometry if a lidar is available. This means that if the visual motion estimation fails, lidar motion estimation cannot be done.

RTAB-Map 总体还是以视觉为主的，没有 visual-lidar 融合。

一些结论：

- 室内导航，双目精度高，但是更推荐RGBD相机，因为可以检测到纹理特征少的表面来规避障碍。如果室内有玻璃等反射物品，视觉不再安全。
- 多相机模式能增大FOV，但是有标定和计算载荷问题，对于2D环境使用有点冗余
- 多相机比 3D Lidar 在成本上更有优势

---

reference:

https://blog.csdn.net/qq_38649880/article/details/90666446

