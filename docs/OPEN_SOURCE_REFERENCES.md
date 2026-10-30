# 公开方案参考与实现边界

本项目以原作业代码为基础重新实现相关算法，不引入 ROS 或开源仓库作为运行依赖。

1. 同济大学 SuperPower，`sp_vision_25`，MIT，检查版本
   `bd9f5e798fa3c6dd3b483ae6627796afb41c608d`。
   [solver.cpp](https://github.com/TongjiSuperPower/sp_vision_25/blob/bd9f5e798fa3c6dd3b483ae6627796afb41c608d/tasks/auto_aim/solver.cpp)
   对普通装甲板使用安装 pitch 先验优化 yaw，重投影使用相同的安装几何；
   [target.cpp](https://github.com/TongjiSuperPower/sp_vision_25/blob/bd9f5e798fa3c6dd3b483ae6627796afb41c608d/tasks/auto_aim/target.cpp)
   使用 `[x,vx,y,vy,z,vz,a,w,r,l,h]` 的 11 维状态表示两组半径和高度。
   本项目参考这两点，采用固定板槽、11 维整车模型，并将安装姿态改为带标准差的软先验。
   原因是本作业的 camera/IMU 精细安装外参未提供，实拍中可见数度的残余滚转，硬约束会导致量测被大量拒绝。
2. Chen Jun，`rm_auto_aim`，MIT，检查版本
   `f244d0e296327d73171311ec50bf5f88856077b1`。
   [tracker.cpp](https://github.com/chenjunnn/rm_auto_aim/blob/f244d0e296327d73171311ec50bf5f88856077b1/armor_tracker/src/tracker.cpp)
   区分跟踪、暂时丢失、丢失，切板时处理另一组半径和高度。
   本项目保留四个固定板槽，切板通过关联选择槽号，失配时预测，连续失配后重捕。
   因此不需要在普通切板时重写整车角度或交换协方差。

本项目另外实现：从像素 Jacobian 传播完整位置/yaw 协方差；同帧共享 IMU 误差的联合更新；
更新前创新的 NIS；在有已知真值的 Monte Carlo 实验中计算 NEES。实际视频没有真值，未计算实际视频 NEES。
公开实现中的噪声常数、外参、板尺寸以及诊断阈值没有直接移植到本视频。

上述项目的 MIT 版权与许可文本保存在 `docs/licenses/`。
