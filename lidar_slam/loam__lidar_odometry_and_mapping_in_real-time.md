# LOAM: Lidar Odometry and Mapping in Real-time

loam 论文

构建实时里程计的一个难点在于点云不是相同时间获得的，因为点云中的点随着激光雷达运动会产生运动畸变，也就是点云中的点会相对实际环境中的物品表面上的点存在位置上的误差。这种运动畸变会造成点云在匹配时发生错误，从而不能正确获得两帧点云的相对位置关系也就无法获得正确的里程计信息。

没有 loop closure.

本文核心思想是将定位和建图的分离，通过两个算法：一个执行高频率的里程计但是低精度的运动估计（定位），另一个算法以低频率执行匹配点云和建图（建图和校正里程计）。both algorithms extract feature points located on sharp edges and planar surfaces, and match the feature points to edge line segments and planar surface patches, respectively.

scan-to-scan、scan-to-map、map-to-map。

scan-to-scan: 计算量小，缺点是误差累积

map-to-map: 优点是精度高，误差累计小；缺点就是计算量大

scan-to-map: 居中。

本文思路：虽然scan-to-scan匹配精度差，但是我们可以只是使用它做一个获取粗的里程计，用获取的结果用于去除匀速运动造成的运动畸变，由于scan-to-scan的计算量较小因此我们可以高频执行。其次，有了里程计校正后的点云数据，接下来我们就可以做一个map-to-map的匹配了。但是map-to-map存在计算量大的问题，因此 我们可以让其执行的频率降低。这样的高低频率结合就保证了计算量的同时又兼具了精度。

特征点提取：考虑计算开销，提取边缘点和平面点作为特征点来使用，提取方法计算简单，只需要计算一个点前后五个点就可以得到该点的曲率。

如果激光雷达的转速相比激光雷达本体运动的速度高很多的话就可以忽略由于运动造成的运动畸变。

本文 lidar 局部坐标系中，x轴指向左边，y轴指向上，z轴指向前。

全局坐标系在初始时刻和局部坐标系相同。

要解决的问题：给定点云计算每次扫描的运动并使用点云构建地图。

Lidar Odometry 节点：10Hz，scan-to-scan匹配进行运动估计。

Lidar Mapping 节点：1Hz, scan-to-map 匹配，建图，获得更准确的位姿。

特征点提取：

通过一次 scan 的点计算曲率c来提取特征点，特征点是c的最大值点–边缘点；特征点是c的最小值点–平面点。为了使特征点均匀的分布在环境中，将一次扫描划分为4个独立的子区域。每个子区域最多提供2个边缘点和4个平面点，同时特征点的选择需要满足阈值的要求。
同时需要考虑特征点选择中的一些越是：比如如果一个点被其它特征点包围，那么就不被选择；以及一些点满足c的要求不过是不稳定的特征点，比如断点等。

特征点关联：

LOAM里面的匹配（correspondence）并不是简单的feature point到feature point的，而是edge point 到 edge line以及planar point 到 planar patch，在后续运动估计中用到的dis也是点到线的距离以及点到面的距离。

运动估计：

假设激光雷达的运动是匀速的，因此如果知道了一帧数据终止点相对于起始点的转换矩阵就可以对这一帧数据中的任意点按照其获得时相对于起始点的时间进行插值。

LM 优化求 R t。

Lidar mapping:

mapping算法就是将已经消除畸变的点云先转换到全局坐标系，然后与局部地图做match。mapping算法的match跟odometry算法的match过程略有不同（主要体现在correspondence确定方式的不同），在odometry算法中，correspondence的确定是为了最快的计算速度（基于最近邻的思路找对应线以及对应面），而mapping算法是通过对特征点周围的点云簇进行PCA主成分分析（求点云簇的协方差矩阵的特征值和特征向量），来找到对应边和对应面。
