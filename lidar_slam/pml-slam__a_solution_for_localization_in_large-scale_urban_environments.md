# PML-SLAM: a solution for localization in large-scale urban environments

城市大场景SLAM

主要启发是地图的管理和“防劫持”

参考：

https://zhuanlan.zhihu.com/p/95354875

- 论文1：Vehicle Perception: Localization, Mapping with Detection, Classification and Tracking of Moving Objects
- 论文2：A Real-Time Robust SLAM for Large-Scale Outdoor Environments
- 论文3：A Real-Time Robust Global Localization for Autonomous Mobile Robots in Large Environments
- 论文4：PML-SLAM: a solution for localization in large-scale urban environments

论文1是一篇博士论文，其中有一个章节是构建基于最大似然方法和栅格地图的SLAM的系统，论文2在它的基础上做了一些改进，比如多尺度地图等，论文3和论文2是同一个作者，所以这两篇文章极其相似，只是论文3多了一个全局定位和被劫持后的重定位功能，论文4在论文3的基础上添加了地图内存管理功能，使得SLAM系统不会因为场景的不断扩大而不断积累内存。

## 位姿初始化

给定一张建好的地图，在机器人启动的时候怎样判断自己在地图中哪个位置。最简单的方法：搜索。但直接搜索效果不好，效率低。

- 使用多分辨率地图，索时先用低分辨率找出粗略位置，然后逐级提高，最终找出精确位置
- 使用多帧点云拼接。先移动一段距离，建立一个小的局部地图，然后拿这个局部地图去和全局地图匹配。

## 位姿更新

依赖具体方法。这里用的是 probabilistic maximum likelihood

## 防劫持

载体被意外移动到另外一个地方，在移动过程中位姿更新有可能被破坏，导致不准或者跑飞，所以在劫持状态结束后，要能够自己判断出来并恢复正常导航模式。这其实比初始化多了一些难度，因为它在全局搜索的基础上多了劫持状态判断。

维护一个持续的位姿质量评测，具体的办法就是把当前帧点云按照当前位姿做一个转换，然后把转换后的点投影到地图中去，如果点投影到了被占据的栅格，那这个点倾向于认为位姿是正确的，如果投影倒了空的栅格，那这个点倾向于认为位姿是错的，最终对所有点做一个统计，得到一个指标，则可以此作为定量判断的依据去判断位姿是否正确。

## 地图内存管理

大场景，如果所有地图数据都放到内存里不太可行（看用什么地图）。

增加一个 Map Manager，使用一个滑窗，滑窗之外的地图数据放在硬盘里，随着位姿的更新，以当前位姿为中心计算新的滑窗，从硬盘加载数据，然后不断循环这个过程
