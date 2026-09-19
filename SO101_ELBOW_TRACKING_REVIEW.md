# 肘部静差诊断与到位正例（2026-09-19）

本记录更新 SO101_KINEMATICS_REVIEW.md 中“36 mm 偏差尚未解决”的结论。
范围：当前 yhang550 落地、空载折叠姿态；不是任意 IK 目标或携物验证。

## 基线证据
- 肘部目标误差约 0.14293 rad。
- 新诊断主题测得 PID 力矩与 Joint::GetForce 均约 0.357 Nm。
- P=2.5，2.5 * 0.14293 = 0.357325 Nm，符合纯 PD 抵消重力所需的稳态误差。
- 总输出限幅 0.6 Nm，关节 effort=0.8 Nm：观测值未饱和。
- 两秒接触采样只观察到起落架与 asphalt_plane 的接触，没有机械臂接触记录。
- 末端误差 0.03628 m，position_reached=false。

## 修改
1. /home/pcz/super_ws/src/so101_gazebo/src/joint_position_controller.cpp
   - 增加 SDF iMax 读取，默认 0，要求有限且 0 <= iMax <= cmdMax。
   - PID 积分上下界由原先固定 0 改为 +/-iMax。
   - 新增每关节 status/effort（gazebo.msgs.Vector3d）诊断：
     x = PID 输出力矩 Nm，y = Joint::GetForce(0) Nm，z = 目标角度减实际角度 rad。
     y 不是电流，也不是完整的接触反力矩。
2. Gazebo 子模块 models/so101/so101.sdf
   - 仅 elbow_flex_controller：i 从 0 改为 1.0，新增 iMax=0.45。
   - P=2.5、D=0.12、cmdMax=0.6 不变。
   - 其他关节 PID、碰撞和惯性不变；继续使用此前验证的 1 ms 步长。
   - 积分限幅限制累积量，不等于完整的反算抗饱和或抓取安全控制。

## 结果
ROS 插件编译通过；主仓库及子模块 git diff --check 通过。
仿真约 12 秒：
- position_error=0.00029 m
- max_joint_error=0.00374 rad
- position_reached=true
- 肘部力矩约 0.3305 Nm，角度误差约 0.00032 rad

仿真约 40 秒：
- position_error=0.00023 m
- max_joint_error=0.00374 rad
- max_joint_speed=0.01218 rad/s
- position_reached=true
- 实际肘角约 1.45000 rad
- 接触采样仍仅有起落架与地面

到位阈值未改变。这是由关节反馈 FK 得到的真实跟踪正例，
尚非外部传感器或 Gazebo 世界末端位姿的独立测量。

## 复测
编译：
    cd /home/pcz/super_ws
    source /opt/ros/noetic/setup.bash
    catkin_make --pkg so101_gazebo -j2

重启仿真后：
    gz topic -e /gazebo/default/so101/elbow_flex/status/effort -d 2

PX4 控制台：
    listener arm_control_status 1
    listener arm_joint_status 1

## 限制
需要继续验证新的笛卡尔目标、目标切换、命令超时后的积分状态、接触阻挡和携物负载。
当前积分仅为无接触肘部静差修正；不要直接推广为夹爪夹持控制。
源文件分属 PX4 仓库、Gazebo 子模块和 super_ws，后者不会随 PX4 提交自动保存。
