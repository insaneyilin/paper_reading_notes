# Map-Based Localization Method for Map-Based Vehicles Localization Method for Autonomous Using 3D-LIDAR Autonomous Vehicles Using 3D-LIDAR

主要工作：

1. curb detection
2. a beam model is utilized to extract the contour of the multi-frame curbs
3. ICP, KF with high precision map

The Velodyne HDL-32E LIDAR

curb: a stone or concrete edging to a street or path.

CURB 应该就是我们中文常说的路沿、马路牙子。

## CURB-MAP BASED LOCALIZATION

- 先检测单帧 curb；
- 考虑时序，Based on the vehicle dynamics, the detected curbs are densified by projecting former curbs into the current vehicle coordinate system.
- the beam model is applied to extract the contour of the densified curbs
- Finally, the extracted contour is matched to the high-precision map by ICP algorithm

curb 的特点：

- 一般比实际道路高 10~15cm
- 和道路相接处高度有明显的变化

单帧 curb 检测：<A real-time curb detection and tracking method for UGVs by using a 3D-LIDAR sensor>

多帧 curb，利用定位 pose 信息对历史单帧 curb 进行补偿

提取轮廓：Beam model proposed by Thrun (2006) (个人感觉和概率机器人里面 occupancy grid 使用的是同一个模型)

Map matching: 使用 ICP 求 R t。地图里的 curbs 要密一些，用的应该是 point to point ICP

Localization：用了两个 kalman filters，第一个做 GPS+INS 的融合，然后把融合结果和 lidar map matching 结果送到第二个 kalman filter 再进行融合。看起来状态量为 [x y \theta] （速度 v 和角度增量作为控制量）

看实验结果，纵向误差一般，Because there is no longitudinal distinction of the curb-features in straight roads。
