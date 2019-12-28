# The Three-Dimensional Normal-Distributions Transform - an Efficient Representation for Registration, Surface Analysis, and Loop Detection

3D NDT 算法

ch2 介绍了几种旋转的表示。euler 角存在 gimbal lock 问题；rotation matrix 在数值计算时要注意，数值误差可能会导致其失去正交性；四元数表示旋转时要求其是单位四元数，这个缺点是如果利用四元数进行数值优化，那么优化需要加上四元数单位长度的约束。rotation vector 表示很适合用于求解优化，但注意 rotation vector 之间没法直接运算(it is not possible to combine rotation vectors using ordinary vector algebra)，所以可以利用 rotation vector 表示进行优化，再转成四元数进行变换的复合。

配准前 downsampling 是常见的处理，降采样可以考虑法向、曲率等约束，本文不考虑。

SLAM，定位、建图，graph-SLAM，点云配准提供的 pose 观测

Most of the sensor types discussed in this section are based on the same principle: sending out an energy beam of some sort and measuring the reflected energy when it comes back.

Radar, radio detecting and ranging，一开始只用来指 radio wave （无线电波）探测，后来扩展为 all electromagnetic （电磁波）。

Lidar 缺点：The main drawback of laser sensors when compared to radars is the sensitivity to attenuation and scattering when scanning dusty or foggy environments, though infrared laser is better at penetrating smoke and dust than visible light.

Lidar 测距也有多种方式：triangulation, TOF, Phase shift 等。

Sonar，使用声波，缺点是精度低。

Stereo vision，低成本视觉方案。精度难保证。

结构光（Projected-light triangulation），project a light pattern onto the scene and analyse the shape of the pattern as seen by a video camera

Time-of-flight camera， TOF相机是通过入、反射光探测来获取的目标距离获取。作者很看好 TOF 相机。

ICP: iteratively refines the relative pose of two overlapping scans by minimising the sum of squared distances between corresponding points in the two scans. Corresponding point pairs are identified by the point-to-point distance.

A basic limitation of ICP is that the nearest neighbour point does not in general correspond to the same point on the scanned surface, especially if the two scans are far apart. 但是大部分应用使用 ICP 的效果还不错。

ICP 最耗时的地方就是找最近邻点。

一些 ICP 处理技巧：

- 加权
- Point-pair rejection （outlier removal）
- KD tree 加速查找最近邻点

point-to-plane ICP，损失函数是最小化源顶点到目标顶点所在的面的距离平方。

IDC, iterative dual correspondences (IDC) algorithm, ICP 的一种扩展。利用双向最近点规则进行约束，在初始 pose 不好时更 robust。

pIC, probabilistic iterative correspondence method. incorporate information about the uncertainties from both scanning noise and the uncertainty of the initial pose estimate. Mahalanobis distance. Use predefined covariance matrix describing the combined uncertainty of the scanner and the pose estimate

Point-based probabilistic registration. 基于概率的配准

NDT, 本文重点，The key element in this algorithm is its representation of the reference scan. Instead of matching the current scan to the points of the reference scan directly, the likelihood of finding a surface point at a certain position is modelled by **a linear combination of normal distributions**.

基于 Gaussian fields 的配准，use a Gaussian mixture model to measure both the spatial distance between points from two scans and the similarity of the local surface around points.

基于 Quadratic patches 的配准, describe the reference scan surface implicitly, using quadratic approximants to the squared distance function from the surface.

Likelihood-field matching, chiefly concerned with registration of 2D sonar scans.

CRF matching. Conditional random fields (CRFs) are a general probabilistic framework, for building probabilistic models of relational information. 计算开销大。

Branch-and-bound registration. 对平移部分分支定界求解，适用于 2D 情况。

Registration using local geometric features. local feature descriptor should be invariant to rigid motion, so that corresponding surface parts can be found regardless of the initial poses of the scans. 好的特征描述子可以帮助解决找对应点的问题。 spin-images 是一种局部特征表达。

ch6 详细介绍了 NDT 方法。The transform maps a point cloud to a smooth surface representation, described as a set of local probability density functions (PDFs), each of which describes the shape of a section of the surface.

NDT 第一步是把空间（reference scan）划分为一个个离散的小格子，每个格子计算出对应的 PDF (probability density functions), 每个格子中的点可以看成是基于 PDF 生成的。

The goal is to find the pose of the current scan that maximises the likelihood that the points of the current scan lie on the reference scan surface.

NDT score 函数的推导还是用 maximum likelihood estimation，使用一些技巧用高斯函数来近似 negative log-likelihood. (这里比原始 2d ndt 论文解释的清楚).

计算 NDT score 需要协方差矩阵的逆矩阵，如果格子里的点是共面、共线的，那么无法求逆（singular covariance matrix）（少于 3 个点则肯定是共面的）；NDT 要求每个格子里至少 5 个点。通过特征值来判断协方差矩阵是否 singular，然后特征值分解/奇异值分解来近似表示 covariance matrix.

使用牛顿法求解优化 ndt score。

这里作者也指出了之前 NDT 的文章都直接用 the sum of Gaussians 来表示 score，但没有给出为什么，本文算是给出了一种在 MLE 角度下的解释。

2D NDT 的推导较简单。3D NDT 复杂度提高，主要是旋转更复杂。

作者之前的工作，求解优化使用的是 axis/angle 表示，问题在于每次迭代后要对 axis vector 进行 normalize，而且有 stray to infeasible region 的风险。本文作者优化直接用了 euler angle，好处是无约束，而且考虑到使用场景，基本不太可能出现 gimbal lock 问题。

关于角度，相关计算也可以用近似值：

- sin(\theta) -> \theta
- cos(\theta) -> 1
- sqr(\theta) -> 0

这样可以大大简化计算量，不过只适用于角度变化很小的情况（10度以内）

3D NDT 最重要的参数是 cell size. Any feature that is much smaller than the size of a cell will be blurred by the function that describes the local surface shape around it, because normal distributions are unimodal.

cell size 太大会导致配准精度变差。Another issue is that with smaller cells, parts of the scan with low point density may not be used at all. 参考点云里每个格子至少 5 个点才计算 PDF，所以也不能用太小的 cell size。格子外的点也没法计算 score，对配准没有贡献。

The optimal size and distribution of cells therefore depend on the shape and density of the input data. 要考虑 input point cloud 的点的密度，设置分辨率的时候要注意。

关于格子的划分有很多种策略，最简单的就是 Fixed discretisation，好处是 point-to-cell lookup 简单、快，缺点是格子大小选取要尤其注意。

可以用八叉树来表达层级的 PDFs。

Iterative discretisation，设置不同 cell size 的网格，做多次 NDT；如果第一次就足够好了，那么就不再继续做了。

Adaptive clustering，A more adaptive discretisation method is to use a clustering algorithm that divides the points of the scan into a number of clusters, based on their positions, and to use one NDT cell for each cluster.

Linked cells，input scan 里有些点不在 reference scan 的网格里，对配准就没贡献了，使用一种 closest-occupied-cell strategy，“linked cells”: to have each cell in the NDT grid store a pointer to the nearest occupied cell and use these pointers when querying the grid for the cell that corresponds to a certain point. 或者用 kd tree 找最近邻格子。

Trilinear interpolation，格子交接处有不连续的问题，In the original 2D NDT implementation [7], the discretisation effects were minimised by using four overlapping 2D cell grids. 或者使用相邻格子 score 的加权平均。

Choosing a good cell size is important when using NDT. If the cells are too large, the structure within each cell will not be described well by a single Gaussian. If the cells are too small, the Gaussians may be dominated by scanner. noise. For sparsely sampled point clouds, there may be too few points within each cell to compute a reliable density function if the cells are small.

The best cell size **depends most strongly on the scale of the input data**. 格子大小要考虑输入点云的尺度。

输入点云降采样的程度也要仔细考虑。

ch7 介绍了 colour NDT，增加了三维颜色信息。（也可以考虑反射值）

ch8 介绍 loop detection.

Given two 3D scans, the question to be asked is: “Have I seen this before?” A good loop-detection algorithm aims at maximising the recall rate — that is, the percentage of true positives (scans acquired at the same place that are recognised as such) — while minimising false positives (scans that are erroneously considered to be acquired at the same place).

本文使用 Appearance-based loop detection.

Appearance-based loop detection can be thought of as place recognition.

基于 NDT 实现 loop detection，相关论文：<Appearance-based loop detection from 3D laser data using the normal distributions transform>

ch9 Surface-shape analysis for boulder detection

