# The Normal Distributions Transform: A New Approach to Laser Scan Matching

2D ndt 算法

Normal Distributions Transform

matching of one scan to another scan

The NDT transforms the discrete set of 2D points reconstructed from a single scan into a piecewise continuous and differentiable probability density defined on the 2D plane.

This probability density consists of a set of normal distributions, that can be easily calculated.

The goal of matching two range scans is to find the relative pose between the two positions.

The NDT models the distribution of all reconstructed 2D-Points of one laser scan by a collection of local normal distributions.

把参考点云（reference scan）划分成一个个小格子，计算输入点云每个点在对应格子中的概率密度，所有概率密度加起来作为 score，最大化这个 score。

用牛顿法进行优化。

SLAM 过程，位置跟踪使用一个简单的 keyframe 策略，建图采取保存关键帧和对应的 pose。After a new keyframe is added, the map is refined by optimizing an error function defined over the parameters of all keyframes.

参考：

https://www.cnblogs.com/21207-iHome/p/8039741.html
