# 运动学与到位判定（第一批）
## 本次结论
PX4 的配置运动学链与 so101.sdf 基本一致。102 组角度（零位、折叠位、100 组随机位）最大末端位置差 0.000003533 m，旋转矩阵差范数 0.000025006。
这项测试是配置链对照，不是 Gazebo 动态真值标定，也不证明 gripper_frame_link 正好位于两指抓取中心。

## 坐标约定
arm_cartesian_setpoint.position 单位为 m，坐标系为 SO101 base_link，目标点为 gripper_frame_link 原点。
当前 IK 只求位置，不保证抓取朝向；位置到位不等于可抓取。
无人机倒挂安装的变换应由上游将世界坐标转换到机械臂 base_link 时处理。

## 修改文件
- src/modules/arm_control/arm_control.cpp：启动 ee_target 改为折叠关节目标的 FK；有效 IK 新目标重置到位计时；拒绝非有限夹爪输入；发布实测误差和到位结果。
- src/modules/arm_control/arm_control.hpp：增加到位检查器和接受目标计数。
- src/modules/arm_control/ArrivalCheck.hpp：连续满足条件才报告到位的独立判定器。
- msg/ArmControlStatus.msg：增加 target_sequence、position_error、max_joint_error、max_joint_speed、feedback_age、position_reached。
- msg/ArmCartesianSetpoint.msg：修正单位注释并明确坐标系。
- src/modules/arm_control/check_sdf_fk.py：可重复的配置运动学对照。
- src/modules/arm_control/test_arrival.cpp：到位判定边界测试。

## 到位条件（初始验收阈值）
所有条件连续满足 0.5 秒仿真时间：
- 末端位置误差 <= 0.005 m。
- 前五关节最大目标误差 <= 0.03 rad。
- 前五关节最大实测速度 <= 0.05 rad/s。
- 模块已初始化、目标有效、反馈有效，反馈接收年龄 < 0.2 s。
条件失效立即清除到位状态；新接受的 IK 目标重置计时。
夹爪不参与这项位置判定：抓取时夹爪可被物体挡住，抓取成功需要单独接触判定。
target_sequence 是模块内接受 IK 目标的计数，0 表示启动折叠目标，不是上游任务 ID。
feedback_age 是模块接收年龄，不是传感器端到端延迟。
现有 sanitizeCartesianTarget 可能调整目标；ee_target 是最终接受的目标，应与原始请求核对。

## 已执行验证
- python3 src/modules/arm_control/check_sdf_fk.py：通过。
- g++ -std=c++14 -Wall -Wextra -Werror src/modules/arm_control/test_arrival.cpp -o /tmp/so101_arrival_test && /tmp/so101_arrival_test：通过。
- make px4_sitl_default -j2：通过。
- git diff --check：通过。
- 无界面 yhang550 实测：
  - ee_target = [0.10619, 0.01247, -0.00502] m
  - ee_measured = [0.13542, 0.01425, 0.01640] m
  - position_error = 0.03628 m
  - max_joint_error = 0.14293 rad
  - max_joint_speed = 0.00422 rad/s
  - position_reached = false
验证了轨迹结束但存在静态偏差时不会误报成功。真实运动达到阈值的正例尚待静态偏差修复后验证。

## 复测
在 PX4 控制台执行：
    listener arm_control_status 1
不要以 ik_error=0 或 joint_command==joint_target 代替 position_reached。
uORB 消息布局有新增字段，依赖旧生成头文件的消费者需重新生成/编译；本次没有更改 MAVLink XML。

## 下一步
肘关节约 8.2 度偏差仍未修复，不能进入抓取成功验收。
需要测量接触和作用力矩，区分重力下的 PD 静差、力矩饱和及接触阻挡，再决定补偿方式。
还需抓取中心几何标定及 IK 目标运动实验。本次不改变 PID、惯性或碰撞参数，不会通过放宽到位阈值掩盖误差。
