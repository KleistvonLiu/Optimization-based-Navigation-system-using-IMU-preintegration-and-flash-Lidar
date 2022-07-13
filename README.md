# Optimization-based-Navigation-system-using-IMU-preintegration-and-flash-Lidar
刘博，这里是github的链接，你试试能不能下载下来，因为我也不是很会用。首先，我必须说明下，代码确实有些乱，因为我还没有整理。比较重要的代码有：
请先把你之前发给我的R q euler angle之间转换的库添加到路径
testEnvironments.m 你可以从第268行开始运行到 502，这里面主要是设置了参数，加载数据，然后就是调用函数。
estimatorv3.m 是最主要的类，它实现了IMU的积分，对点云的处理，还有就是调用优化函数，滑窗。
IntegrationBasev3.m 实现了IMU的预积分，它被estimator类调用
build_ceres_optimization.m 是生成cpp files 的mex file
ceres_optimization.cpp 是最主要的cpp file，其他的files都是被他调用，这里面有很大一部分代码是被if 0 comment掉了，你可以从ctr + f 一下if 1 从那里开始看
IMUfactor实现了IMU的残差
PCfactor实现了PC的残差
