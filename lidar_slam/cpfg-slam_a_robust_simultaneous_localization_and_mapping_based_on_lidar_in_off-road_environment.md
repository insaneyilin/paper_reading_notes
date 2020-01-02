# CPFG-SLAM:a robust Simultaneous Localization and Mapping based on LIDAR in off-road environment

CPFG-SLAM：野外大场景鲁棒SLAM

野外的特点是场景略微空旷，周围特征不规则。使用 LOAM 会飘，使用 ICP 计算效率低，NDT 没有考虑特征。本文思路，结合特征和NDT。

提取线特征和面特征：

- 对点集计算协方差矩阵，然后提取其特征值。当有两个主要特征值时就是面，当有一个主要特征值时就是线。

利用多帧点云提取特征：

- 单帧点云是稀疏的，如果某一帧点云没有提取出特征，等下一帧到来时，按照位姿拼在一起，再进行特征提取

