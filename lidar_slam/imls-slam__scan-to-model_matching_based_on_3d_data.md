# IMLS-SLAM: scan-to-model matching based on 3D data

IMLS SLAM

参考：

https://zhuanlan.zhihu.com/p/94685161

以IMLS曲面为基础进行 scan-to-model 匹配

一个 SLAM 系统的整体步骤：

1. 处理点云。包括去畸变，移除离群点，高级点可以移除动态物体
2. 对点云采样。一般都不会用全部点的，具体采样方法主要取决于自己的匹配方法
3. 建立地图。可以是特征地图，可以是点云直接拼接的地图，也可以是曲面模型
4. 匹配。把点云和地图进行匹配，更新位姿
5. 更新地图。根据新的点云，对地图进行更新。更新频率和策略一般也取决于匹配方法

IMLS SLAM 的思路：

- first have **a specific sampling strategy** based on the LiDAR scans.
- define our model as the previous localized LiDAR sweeps and use the **Implicit Moving Least Squares (IMLS) surface representation**

## III. SCAN EGOMOTION AND DYNAMIC OBJECT REMOVAL

先提取地面，然后对剩下的点云聚类，小于一定尺寸的聚类（14m×14m×4m）就全部去掉。比较粗糙的方法。

IV. SCAN SAMPLING STRATEGY

在一些 ICP 改进版中，提取点特征的时候往往都是先计算协方差矩阵，然后对矩阵计算特征值，根据特征值对特征分类，同时提取主轴方向，根据主轴方向和特征值进行点的筛选。

作者认为：主轴方向应该为车的方向，也就是雷达的方向，其次，提取点是为了匹配得到位姿，不同的点对位姿精度的贡献不一样，如果能直接提取出对位姿贡献大的点，那么不就既可以很大程度上减少点的数量，又可以取得很好的精度吗。

问题：怎样去判断哪些点对位姿精度贡献大。作者给出了九个指标。前六个是点对姿态精度贡献的定量评价，后三个是对位置精度贡献的定量评价。（联想一下 LOAM，LOAM 是使用曲率来提取 edge points 和 planar points）

V. SCAN-TO-MODEL MATCHING WITH IMPLICIT MOVING LEAST SQUARES (IMLS) SURFACE REPRESENTATION

KinectFusion, scan-to-model matching using an implicit surface from as a model

implicit surface is defined by a Truncated Signed Distance Function (TSDF) and is coded in a voxel map

KinectFusion使用的TSDF曲面表示方法不适合大场景，所以使用了IMSL曲面表示方法。作者对这个方法并没有做什么改进，只是使用，相关论文：

- IMSL：《Provably good moving least squares》
- MSL：《Mesh-Independent Surface Interpolation》（IMSL的前身）

scan和曲面地图之间做数据关联然后优化

地图更新：将新点云中提取的点加到已有地图中。为了匹配时和scan进行数据关联，要对地图进行KD树存储，目前的做法是每次更新完就重新建立KD树，显然这种做法比较耗计算量，这也是整个步骤中耗时最长的一步。所以设计增量式KD树是有必要的，即每次根据删除的老点和增加的新点更新KD树，而不是重新建。

个人感觉这篇文章比较一般，亮点应该就是使用了 IMLS 表示。
