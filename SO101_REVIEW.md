# SO101 控制、运动学与抓取实验（统一审阅文档）

本文件合并原 INTERFACE、KINEMATICS、ELBOW_TRACKING、TARGET_SWITCH、
GRASP 五份专项记录。后续修改和实验只维护本文；原记录可从 Git 历史恢复。
以下历史章节描述各阶段当时的结果，“未提交”和“下一步”不代表当前状态。
最新增量见文末「第九步：夹爪失联保持与界面复现」；本轮修改了 Ubuntu20
的下位 ROS 插件，但 ROS 仓库仍不提交。历史章节的“未修改 ROS”仅描述当时状态。

## 目录与职责（2026-09-28）

```text
src/modules/arm_control/
  arm_control.cpp / arm_control.hpp    PX4 运行模块、IK、目标轨迹和反馈
  ArrivalCheck.hpp                    严格到位判定
  CMakeLists.txt / Kconfig             固件构建配置
  tests/
    kinematics/                       FK 对照与到位判定边界测试
    grasp/                            几何审查、隔离夹具、整臂抓取实验
```

主机实验源码不加入飞控模块 CMake。迁移修正了 Python 仓库根目录定位
及 test_arrival.cpp 头文件引用；脚本名和隔离实验参数保持不变。
运行结果集中在 build/grasp_validation 或 build/full_arm_grasp，不放进源码目录。
ROS 库不提交；正常机体、模型和控制参数不因目录迁移而修改。

## 接口规范与现有约束

- 命令：arm_control → arm_joint_command → MAVLink → Gazebo Transport →
  六个控制插件；实际状态沿反方向进入 arm_joint_status。
- Gazebo 桥接器等待六个关节各自的新反馈再发送，不无限重发缓存。
  PX4 拒绝非有限位置/速度；joint_count 为 6。
- current=NaN 表示电流未测量，不是物理求解失败；力矩诊断不等于电流反馈。
- controlSource 在 ros/gazebo 中二选一。当前集成场景是 gazebo；
  没有启动 ROS API 时 ROS joint_states 不发布，不能据此判断 MAVLink 断路。
- PX4 反馈超过 1 秒仿真时间失效时停止推进和发布命令；
  Gazebo 命令超过 1 秒仿真时间超时后，以当时实测角度保持一次。
  这不是机械制动，也不保证零跟踪误差。暂停物理会暂停这些超时。
- 反馈协议仍缺源时间戳、序列号和有效位；六个新样本不等于严格同步采样，
  feedback_age 是 PX4 接收年龄，不是端到端延迟。
- 物理步长 1 ms；正常 empty.world 为 10 迭代，隔离夹持测试默认 40 迭代。

## 运动学、末端目标与严格到位

arm_control target x y z gripper 经 uORB 投递，不能直接修改工作队列内的 IK 状态。
前三项单位米，坐标系是 SO101 base_link，目标是 gripper_frame_link 原点；
gripper 单位弧度。世界目标必须先转换到机械臂基座坐标。
当前是位置 IK，不保证夹指方向；工具坐标原点也不是两指物理夹持中心。
输入必须有限、模块必须运行。命令投递成功不等于 IK 接受或实际到位。

每条接受目标最多推进 20 mm，这是单条消息限制，不会自动继续走向原始请求。
必须核对 ee_target 与请求值；target_sequence 是接受 IK 目标计数，不是任务 ID。

PX4 原严格到位：实测末端误差 ≤5 mm、前五关节误差 ≤0.03 rad、
速度 ≤0.05 rad/s、反馈有效且年龄 <0.2 s，连续满足 0.5 秒仿真时间。
新目标重置计时；夹爪不参与此判定，必须另测物体接触和相对运动。
ik_error 或 joint_command==joint_target 均不能代替实测到位。

配置链检查：102 组姿态最大位置差 3.533 μm，旋转矩阵差 0.000025006。
这是 SDF/PX4 配置一致性检查，不是外部测量标定或碰撞安全证明。
迁移后 FK 检查与到位边界测试重新通过。

复测入口：

```bash
python3 src/modules/arm_control/tests/kinematics/check_sdf_fk.py
g++ -std=c++14 -Wall -Wextra -Werror src/modules/arm_control/tests/kinematics/test_arrival.cpp -o /tmp/so101_arrival_test
/tmp/so101_arrival_test
```

## 历史：肘部 36 mm 静差修复与目标切换（2026-09-19）

初始末端误差 36.28 mm，肘部角差 0.14293 rad。测得力矩约 0.357 Nm，
等于 P=2.5 与角差之积，且未达到 0.6 Nm 输出上限；短接触样本仅有起落架接地。
证据支持该姿态下 PD 重力静差，而不是力矩饱和。

ROS 控制插件增加有限范围 iMax 读取及每关节 status/effort 诊断：
x 为 PID 力矩 Nm，y 为 Joint::GetForce Nm，z 为目标角差 rad。
仅 SDF 肘部增加 I=1.0、iMax=0.45 Nm，保留 P=2.5、D=0.12、cmdMax=0.6。
这是积分限幅，不是完整抗饱和或抓取安全控制。ROS 工作区不随 PX4 提交保存。

修复后 12/40 秒折叠位误差 0.29/0.23 mm，肘角约 1.45000 rad，
严格 position_reached=true；这是基于反馈 FK 的真实跟踪正例，
不是世界末端外部测量。SITL、Gazebo、ROS 插件编译均通过。
停止/重启 arm_control 后命令与反馈恢复，腕滚速度约 -0.00072 rad/s；
不代表任意姿态、负载或故障注入均通过。

空载目标切换（gripper=0.3）：

| 目标，base_link 米 | 末端误差 | 严格到位 |
| --- | ---: | --- |
| 折叠位 | 0.23 mm | true |
| A=(0.11,0.0125,0.02)，分两条消息推进 | 0.62 mm | true |
| B=(0.12,0.015,0.03) | 1.80 mm | true |
| 返回 A | 0.50 mm | true |

短接触样本仅有起落架接地，不证明整条路径无碰撞。
无自主恢复、碰撞规划、全面重力补偿或完整飞行携物验证。

## 历史：夹指、方块和接触诊断（2026-09-20）

## 第一步：夹指碰撞几何校准

## 范围

只修改 Ubuntu-20.04 的 SO101 两个夹指碰撞盒；不改转轴、关节限位、
PID、摩擦、质量或惯量，不提交 ROS 库。不宣称已实现抓取。

## 依据与修改

按 SDF visual 变换将现有二进制 STL 顶点转换到各自 link 坐标系。
新增 `src/modules/arm_control/tests/grasp/check_gripper_geometry.py` 可复现测量；
脚本使用当前 visual 的理想化 90/180 度旋转，模型变换更改后需同步更新。

活动指网格 Z 范围 -82 至 +9.995 mm，原碰撞盒却位于 +10 至 +28 mm，
基本处于可见网格之外。固定指远端网格位于负 X，原碰撞盒横跨 X=0。

| 碰撞盒 | 新中心（米，各自 link 坐标） | 新尺寸（米） |
| --- | --- | --- |
| 固定指 | -0.014, -0.000218, -0.088 | 0.012, 0.014, 0.026 |
| 活动指 | -0.006, -0.0189, -0.070 | 0.012, 0.014, 0.024 |

这是远端夹持面的简化代理，不是完整夹指网格或精确凸分解。
只保留远端代理意味着不能用本模型宣称整个夹指都具备避碰覆盖。
新盒子依据末端顶点范围选取，不代表已经通过逐三角形贴合检查。

## 审阅要点与后续门槛

零角度时，两个代理内侧沿 X 的间隙约 16.2 mm；正角度朝增大开口方向。
活动指绕 Y 旋转，间隙随 Z 改变，16.2 mm 不是恒定平行夹爪开口。
本步骤提出候选方块为 20 mm / 10 g；调整结果见第三步。

必须继续检查：Gazebo 实际碰撞体显示、慢速空载开合、双侧接触、
受载关节力矩与物体滑移。网格范围检查和 SDF 解析不能替代这些测试。
夹指原有质心/惯量未校准，后续动力学测试仍需审查。

本步骤保留为未提交改动供审阅。

## 第二步：空载开合（2026-09-20）

测试：Ubuntu-20.04，无界面 yhang550 SITL，无人机未解锁，未生成测试方块。
使用修正后的夹指碰撞盒，末端目标保持 base_link 坐标
`0.11 0.0125 0.02` 米。经 PX4 `arm_control target` 改变夹爪角度，
不直接绕过命令链路控制 Gazebo。其他五关节不改变目标。

| 测试 | 观测 |
| --- | --- |
| 初始 0.3 rad | 反馈 0.28891 rad |
| 第一轮打开至 0.6 rad | 约 5 秒后 0.55298；随后 0.57038 rad |
| 第一轮闭合至 0 rad | 约 5 秒后 0.00945 rad，速度 -0.00154 rad/s |
| 第二轮打开，5 秒采样 | 240 组位置/速度均有限；末值 0.553505；速度绝对峰值 0.527486 rad/s |
| 第二轮闭合，5 秒采样 | 239 组位置/速度均有限；末值 0.009452；速度绝对峰值 0.529818 rad/s |
| 闭合后控制力矩短采样 | 约 -0.00028 N m，明显低于 0.01 N m 输出上限 |
| 最后末端状态 | 误差 0.56 mm，position_reached=true |

结论：本姿态、0 至 0.6 rad 范围内完成两轮空载开合，未观察到位置或速度
NaN / 坍缩。存在静态角度偏差；速度峰值略超 0.5 rad/s，不能宣称严格限速通过。
本步骤不调大 PID、输出力矩或摩擦。

闭合后 2 秒接触采样仅见两个起落架与 asphalt；不是全程接触覆盖证明。
`self_collide=false` 未更改，不能据此证明夹指间或机械臂自身无几何重叠。
`current=nan` 表示未测量电流，与物理求解状态的 NaN 不同。
`position_reached` 只判定前五关节末端，不包含夹爪抓取判定。
无界面测试未完成视觉碰撞叠加审查。测试后已停止本次仿真。

## 第三步：缩小测试方块（2026-09-20）

修改 Gazebo 子模块 `models/test_cube/model.sdf`：

- visual 与 collision 尺寸同步从 50 mm 改为 20 mm。
- 质量从 50 g 改为 10 g，作为低载荷初始测试，不代表最终载荷。
- 均匀立方体惯量按 I=m*a*a/6 重算，三对角项为 6.6666666667e-7 kg m²。
- 保留 dynamic、摩擦 0.8、接触 min_depth=0.0005 和 max_vel=0.05。

未改原模型名，因此引用 test_cube 的其他场景也会使用新尺寸。
后续地面生成时中心至少高于支撑面 0.01 m，且应留少量落定间隙，
不能沿用旧方块中心高度来判断是否悬空或穿透。
20 mm 是候选夹持尺寸，尚未验证两侧接触或承载；不能据此宣称抓取成功。

## 下一步与未完成项

在固定机体/可控支撑条件下生成方块，检查真实双侧接触，再评估力矩、
物体滑移及抬升保持。不要因角度或末端到位直接报告抓取成功。
本轮所有改动保留未提交，ROS 库不提交。

## 第四步：双侧接触及首次静态保持（2026-09-20）

### 测试夹具与新增工具

新增 `src/modules/arm_control/tests/grasp/grasp_fixture.py`，从当前 SO101 SDF 复制
gripper_link、活动指、gripper 关节和控制插件，生成临时测试 world。
夹爪基座通过 fixed joint 连到 world，关节和方块仍为动态对象。
此为接触单元测试，不是完整无人机或 PX4 数据链路验证。
不使用 static=true 固定整个夹爪，也没有将方块焊接到夹爪。
原始 SO101 和 test_cube 的物理参数没有在本步骤改动。

夹爪基座世界高度 1 m，方块初始中心 (0.0025,-0.000218,0.912) m。
初始支撑台顶面 0.902 m，与 20 mm 方块底面相接。
物理步长 1 ms，ODE quick 10 次迭代；未完全复制 empty.world 的所有接触设置，
因此结果还需要在整机场景复验。

新增 `grasp_probe.cpp`，只作为主机 Gazebo Transport 工具，不进入飞控固件。
10 Hz 发布命令，5 秒保持打开，然后按目标 0.1 rad/s 从 0.6 闭合至 -0.15 rad。
记录实际角度、速度、控制器输出力矩、方块位姿及接触帧数。
同一 Contacts 消息中两侧夹指都与 test_cube 接触才计为 bilateral。
目前未汇总接触法向力/切向力，控制器输出力矩不能当作接触力。
早期静止物体未推送 pose 消息时 pose_seen=0，该阶段零位姿不可作为测量。

### 观测结果

- 保留台面时，实际夹爪被方块挡在约 0.058 rad，目标 -0.15 rad；
  力矩约 -0.00625 N m，未达到 0.01 N m 输出上限。
- 多个连续窗口的双侧接触比例接近 100%，确认真实双侧接触。
- 测试约第 36 至 37 秒之间，手动删除临时 cube_support；
  删除仅影响此次仿真，重新生成并启动夹具即可恢复。
- 持续闭合命令仍在发送时，从采样第 36 秒到第 44 秒：
  方块 Z 从 0.911775 降至 0.906128 m（下滑 5.647 mm）；
  Y 从 -0.000209778 移至 0.00509992 m（侧移约 5.31 mm）。
- 中途独立位姿查询得到 roll 约 0.449 rad，说明还伴随明显转动。
- 无支撑末期仍有约 97% 的双侧接触帧，说明接触存在不等于抓取稳定。
- 本次采样关节位置和速度均为有限值，没有观察到求解坍缩。

结论：双侧接触通过；首次无支撑静态保持失败，不进入携物阶段。
下滑发生在采样器退出前，不能归因于退出后的一秒命令超时。
测试结束后停止临时 gzserver，未修改正常仿真启动配置。

### 复现实验

早期手工生成 world、运行 probe、删除支撑的方式仅作历史记录。
现请用仿真时间驱动的 run_grasp_case.py，避免删模型造成停滞或墙钟时序误差：
`python3 src/modules/arm_control/tests/grasp/run_grasp_case.py unique_case --seconds 60`。
脚本自动编译、执行、确认撤台并保存 manifest/CSV/summary。
complete 只表示实验流程完整，不等于抓取成功。
本次原始临时日志：`/tmp/so101_grasp_probe.csv`（重跑会覆盖）。

### 下一步修改方向

先增加接触力和物体相对夹爪姿态的测量，再区分：
夹持法向力不足、指面角度/接触位置产生滚转、接触求解误差。
只改一个变量做对照，不能直接同时增大摩擦、PID 和力矩上限。
当前活动指碰撞代理、质心与惯量仍待进一步校准。
稳定保持通过之前，不报告抓取成功，不开展无人机携物。

## 第五步：接触力诊断与单变量调整（2026-09-20）

### 测量与测试工具修改

`grasp_probe.cpp` 新增：每侧法向力、切向力模长、世界 Z 向力、
世界 X 向接触力矩、法向力加权接触中心 Y/Z、方块 roll/pitch/yaw。
力先对各接触点求和，再在约一秒消息窗口内取平均；接触中心按法向力加权。
无接触点时中心为 NaN，表示无测量，不是物理失效。
选择作用在方块一侧的 wrench，使用最近一次方块姿态转到世界坐标。
此处姿态与接触消息非严格同步，因此力分解是诊断近似，不作为高频控制输入。
contact normal/position 与 link-local wrench 不可直接混用。
坐标系依据：[Gazebo Classic ODEPhysics::UpdatePhysics 源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEPhysics.cc)。

撤台改为第20秒自动请求，并增加：最近窗口有反馈、有限关节状态、
双侧接触帧比例大于90%才撤台。请求异步执行、2秒超时，不阻塞10Hz命令。
若撤台响应失败则试验退出，不计保持成功。
修复前一次重复测试停在第19秒，未完成撤台，已排除，不混入结果。
完整重试使用修复后的程序；早期四组均完成到第44秒。

`grasp_fixture.py` 增加仅针对临时夹具的 `--p`、`--iterations`、
`--cube-z-offset` 参数与范围检查。默认迭代数调整为40，
可用 `--iterations 10` 重现基线。没有修改整机 empty.world，
也没有修改正式模型的 PID、摩擦、夹指几何或 ROS 库。

### 对照结果

均为20 mm / 10 g方块；步长1 ms；目标闭合-0.15 rad；
约第20秒撤台，第44秒取末值。高度变化以第19秒撤台前值作参考。
初始四组随机种子123。所列力为末个消息窗口均值，不是整个试验峰值。

| 相对基线只改一项 | 总下滑 mm | 第25至44秒再下滑 mm | 末值 roll | 两侧法向力 N |
| --- | ---: | ---: | ---: | --- |
| 基线：P=0.03，10迭代 | 13.845 | 9.182 | 36.16° | 0.0781 / 0.0764 |
| P提高至0.045 | 11.846 | 6.579 | 20.60° | 0.1172 / 0.1171 |
| 迭代数提高至40，P恢复0.03 | 0.563 | 0.011 | -4.19° | 0.0912 / 0.0912 |
| 方块/支撑上移6mm，其余恢复基线 | 18.360 | 12.439 | 28.59° | 0.0777 / 0.0760 |
| 40迭代重复：种子456，修复后的异步撤台 | 0.568 | 0.016 | -4.16° | 0.0913 / 0.0913 |

两次完整40迭代测试末窗口均100%双侧接触。重复试验正常退出；
所有本轮临时仿真均在采样后关闭。未执行整机参数移植或携物。

原始日志位于 `/tmp/so101_force_baseline.csv`、`/tmp/so101_force_p045.csv`、
`/tmp/so101_force_iter40.csv`、`/tmp/so101_force_z6.csv`，临时目录可能被清理。
`/tmp/so101_force_iter40_repeat.csv` 为未完成试验，禁止当作成功样本。
修复后重复试验另存 `/tmp/so101_force_iter40_retry.csv`。

### 分析与保留的调整

1. 增大P确实提高了法向力，但没有消除持续下滑，不能归因于单纯夹持力不足。
2. 改变夹持位置未解决问题，并改变了滚转/下滑程度；几何和接触位置仍有影响，
   但本次并未证明哪个几何形状最优，不能直接据此重做夹指。
3. 只提高求解迭代数就显著减小持续漂移，是本轮证据最强的可修复因素。
   保留40次迭代作为隔离夹具默认值，撤销提高P及上移方块的实验设置。
4. 40次迭代末窗口两侧竖直力和约0.0971 N，接近方块重力0.0981 N；
   固定指与活动指承担竖直载荷不均，仍需后续验证抗滚转裕量。
5. 不使用简单的切向力模长/法向力比直接判断ODE摩擦饱和，
   摩擦方向与求解模型也会影响该判断。

该结果只支持本夹具、本姿态、本载荷下短时保持改善；
约4°的初始滚转仍存在。没有完成全姿态、长时间、受扰或携物验证。
不把短时静态保持改善当作完整抓取demo通过。

40 iterations 在约 75 s 的撤台后稳定保持阶段表现良好，但约 101 s 后发生突发滚转失稳；最终三维漂移 6.62 mm、Z 向下降 4.30 mm、roll -36.4°，尽管末窗口仍保持 99.8% 双侧接触。因此 120 s 长时间静态保持判定失败。

## 第六步：完整整臂的简单位置验证（2026-09-28）

按当前要求先完成简单验证，再做固定抓取，最后才扩展携物、飞行或自动状态机。
本轮只验证接近、读取真实位姿和小步对齐，没有闭合、撤台或携物。

### 新工具与场景

- tests/grasp/run_full_arm_grasp.py：完整 yhang550/SO101 的专用实验入口，
  命令仍经过 PX4 的 uORB/MAVLink/Transport；未将实验逻辑写入 arm_control。
- tests/grasp/full_arm_probe.cpp：只读观察实际夹爪、方块位姿及碰撞对，
  区分双指、非夹指和支撑台碰撞，不发布任何关节命令。
- 临时场景固定机身在世界 Z=1.2 m，保留原倒挂安装、全部关节及控制器；
  只在此实验使用 ODE 40 迭代。未改正常模型、PID、摩擦、惯性或正式到位阈值。
- 方块仍是 20 mm / 10 g 动态刚体；物理支撑为 12×14×4 mm 倾斜薄台，
  与方块下表面对齐且避开夹爪本体。没有物体绑定或过程中的瞬移。
- 仿真时间决定等待、阶段和修正时序，墙钟仅作卡死保护；
  日志/源码/模型/库哈希保存在 build/full_arm_grasp/<case>/。

Gazebo pose/info 中 link 位姿是 model-relative，先乘机体世界位姿再使用；
本轮另把实际工具世界位姿与 PX4 反馈 FK 做独立比较，误差约 0.016 mm，
避免把模型相对位姿当成世界位姿。

夹持参考点为 gripper_link 中 (0.0025,-0.000218,-0.088) m，
来自先前碰撞代理实验；不是打开夹爪时两指的即时中点或独立实物 TCP 标定。
方块初始世界中心为 (0.182974,-0.005095,1.065144) m。
该场景专用坐标不能原样当成正常起飞场景的世界目标。

### 本轮结果和明确限制

full_align_01：先到 base_link=(0.17,0.035,0.05) m、夹爪保持 0.6 rad，
读取方块与夹持参考点误差，再每次最多修正 3 mm；
每条微目标须被 IK 接受并核对 ee_target，等实际反馈后再继续。
在原接受目标上增量修正，可补偿本姿态的静态偏差，不修改 PID。

- 初始参考点—方块距离 29.48 mm，最终 1.28 mm。
- 候选末端 grasp target=(0.17957,0.01586,0.06979) m，SO101 base_link 坐标。
- 最后三个观测均在 3 mm 内且关节速度较小，候选点已保存 grasp_target.json。
- success 仅指本阶段对齐完成，grasp_tested=false、flight_validated=false。
  尚未证明两指能同时夹住、离台保持或携物成功。

full_align_02 默认测量模式重复验证：候选点为
(0.17958,0.01587,0.06981) m，参考点距离 1.27 mm；
与第一轮候选目标相差约 0.025 mm。
两轮日志仿真时间均严格递增、方块位置均为有限值；
接触采样未发现非夹指碰撞或支撑台—机械臂碰撞。
测试结束后自身启动的 PX4/Gazebo 均已关闭。未执行 Git 提交或推送。

示范入口用实测末端 ≤12 mm、关节差 ≤0.08 rad、速度 ≤0.05 rad/s 连续 0.5 s
判断附近检查位停稳；严格 PX4 position_reached 仍单独记录，原 5 mm 规则不变。
附近检查位的严格到位为 false，不把 1.28 mm 夹持参考点距离冒充严格末端误差。

早期失败样本也保留：full_calibration_02 的 Z=0.10 m 目标被 IK 拒绝；
full_pickup_01 方块在靠近前掉落；full_pickup_02 起初放稳，但接近路径撞台；
full_calibration_06 延伸位实际误差约 16.7 mm，未通过其 15 mm 试验门槛。
这些均不计为抓取成功；没有据此放宽正式关节限位或调整夹爪 PID。

### 当前复现入口

```bash
# 默认只靠近、读取、微调；不闭合。
python3 src/modules/arm_control/tests/grasp/run_full_arm_grasp.py new_alignment_case --calibration build/full_arm_grasp/full_calibration_05/calibration.json

# 无方块重新标定（模型或目标改变后使用新的 case 名）。
python3 src/modules/arm_control/tests/grasp/run_full_arm_grasp.py new_calibration_case --calibrate
```

--grasp 才启用闭合与双侧接触检查；--grasp --retrieve 才启用回收。
闭合且台面仍支撑的结果只叫 bilateral_contact_verified，不证明离台抓取。
这两个后续动作本轮未执行；先复核候选点，再进入固定抓取。
已有 case 名禁止覆盖。入口拒绝占用中的 PX4/Gazebo 端口，只清理自身启动的进程。

## 第七步：固定方块双侧夹持、抓起与回收（2026-09-28）

当前优先目标是抓取 demo：真正双侧夹住、离开支撑、回收后不掉落。
小幅位移只记录，不再作为 demo 失败条件；精度验收保留为可选 strict 模式。
尚未完成飞行携物，不能用固定机身实验代替飞行验证。

### 本轮修改

- tests/grasp/full_arm_probe.cpp 增加实测夹爪角度、速度、反馈接收龄期、
  PID 力矩和两侧接触法向力、切向力、世界竖直合力。力转换采用最近物体姿态，
  属于诊断近似，不能作为高频精确测量。NaN 力矩若表示未收到消息，不等于物理 NaN。
- tests/grasp/run_full_arm_grasp.py 确认闭合后双侧接触、接触力和夹爪停稳；
  带物运动每条目标最多 3 mm，时序、等待、保持全部采用仿真时间。
  先横向退出固定台面再抬升；回收时保持侧向距离，先向机身收回再回中。
  台面不删除、不移动；方块不绑定、不瞬移，仍为动态刚体，质量保持 10 g。
  --cube-mm 支持临时尺寸 16..20 mm，同时重算实体方块惯量、台面高度及夹持参考点。
  原 test_cube/model.sdf 不变。16 mm 方块不能沿用 20 mm 的中心参考点；
  其中心 X 应到固定指内表面 -8 mm 加半宽的位置，即夹爪局部 X=0。
- --acceptance demo：连续保护检查没有明显分离（相对抓取时位移超过 45 mm）、
  没有持续超过 0.5 s 的双侧接触丢失、没有非夹指/台面干涉；保持阶段无支撑，
  双侧接触帧比例至少 90%，两侧平均法向力均大于 0.01 N，物体仍高于初始高度。
  精度和滚转仍写日志，但不因几毫米的位移中止。
- --acceptance strict 另要求末值漂移不超过 5 mm、峰值不超过 10 mm、
  保持阶段最低抬高至少 25 mm。正式 PX4 5 mm 到位规则没有放宽。
- --hold-seconds 默认 30 s，--lift-mm 默认 35 mm，--side-clear-mm 默认 70 mm；
  --retrieve-route stow 启用回收，默认 vertical 只抬升保持。
  --iterations 仅改变临时场景，默认仍为 40；另做过 quick/200 与 world 求解器对照，均未解决脱落。
  --gripper-p 只覆盖临时场景的夹爪 P；本轮对照 0.03 与 0.045，
  I=0、D=0.001、力矩上限 0.01 N·m 不变；单独提高 P 的对照不改摩擦。
  --patch-radius-mm 默认 0；显式设为 4 时仅在临时场景的两个指面增加
  torsional 摩擦：coefficient=0.8、use_patch_radius=true、patch_radius=0.004 m。
  指面线性 mu/mu2、方块与支撑面的摩擦均不改；不是刚性绑定或吸附。
  抬升前另用实测位姿和模型中的夹爪本体碰撞盒计算世界包络，
  必须与台面在侧向相隔至少 3 mm；不能只按方块中心是否离台判断。
- tests/grasp/test_full_arm_grasp.py 增加帧加权、相对坐标转换和 demo/strict 判据
  的离线测试，防止把台面支撑或单侧接触误计为成功。

未改正式模型、PID、摩擦、惯性、ROS 库或 arm_control 核心流程。
代码仍放在 tests/grasp；日志和源码/库/场景哈希保存在 build/full_arm_grasp/<case>/。

### 已完成的前置与失败样本

| 样本 | 实际结果 | 结论 |
| --- | --- | --- |
| fixed_contact_01 | 台面仍支撑时双侧接触约 99.9% | 只证明夹住 |
| fixed_lift_baseline_01 | 回收后抬高 28.18 mm；8 s 内仍有双侧接触，相对位移 17.10 mm | 当时精度门槛未通过，未做 30 s 保持 |
| slow_lift_hold30_01 | 直接竖直抬升时本体碰到台面下方，保护中止 | 倒挂布局需要先离台 |
| side_lift_hold30_01 | 退出约 41.2 mm，抬高 27.33 mm，无其他碰撞；约 7 s 后漂移超过 10 mm | 当时 strict 保护中止，不代表已经证明掉落 |
| demo_stow_hold30_01 | 已离台抬升，斜向回收时本体擦到台面，保护中止 | 回收不能直接斜穿台边 |
| demo_stow_hold30_02 | 20 mm 方块在抬升中出现跳动，接触到夹爪本体 | 非小幅滑移，保护中止 |
| demo_cube16_stow30_01 | 缩小后沿用旧中心，只有活动指接触 | 尺寸与夹持点必须一起更新 |
| demo_cube16_stow30_02 | 更新中心后双侧接触成立，但离台时真实脱落，活动指约 0.053 N | 需有限地提高夹持力 |
| demo_cube16_p045_stow30_01 | P=0.045 后离台仍保持双侧接触，但抬升时本体撞台 | 40 mm 侧移不足；离台时本体包络仍与台面侧向重叠约 13 mm |
| demo_cube16_p045_clear70_01 | 扩大侧移后仍在离台过程中翻转脱落，无其他碰撞 | 不能只归因于撞台或夹持力不足 |
| demo_cube16_p045_iter200_01 | quick 迭代增至 200，仍在离台过程中脱落 | 此例单纯加迭代无效 |
| demo_cube16_p045_world_01 | 换直接求解器后方块在闭合前被推出支撑 | 不作为成功抓取 |
| demo_cube20_p045_hold30_01 | 恢复 20 mm、P=0.045 后能离台抬升，但回收时持续滚转撞到本体 | 增大 P 仍不足以消除滚转 |

side_lift_hold30_01 的抬升后初始相对位移为 5.95 mm，已单独记录，
不能因为另设保持阶段参考位置而忽略。期间双侧竖直合力约 0.097 N，
接近重力 0.098 N，但方块有持续滚转；接触存在不等于完全无滑移。
上述中止样本均不报告完整抓取通过；未出现物体位姿 NaN。

### 指面抗扭模型与通过的实测

夹持力与竖直支持力要分开：两侧法向力主要相互抵消，不应把其模长之和
当作抵抗重力的合力。这里比较的是两指作用在方块上的世界 Z 向力之和。
10 g 方块的重力约 0.0981 N；合力接近重力只说明近似力平衡，
仍可能缓慢滑动或转动，必须再观察真实位姿、接触持续性和是否离台。

根据对照结果，本轮只在生成场景中试用有限抗扭摩擦，以限制持续翻转。
Gazebo 的此模型随法向力和有限接触半径提供抗扭能力，默认半径为零；
4 mm / 0.8 是 demo 用接触面假设，不是实物橡胶参数标定。
依据：[Gazebo 官方抗扭摩擦说明](https://classic.gazebosim.org/tutorials?cat=physics&tut=torsional_friction)。
Gazebo 11 ODE 对两个面的 patch radius 取较大值、coefficient 取较小值，
因此只给指面设置半径即可；方块—台面两面半径仍为零，没有暗中增强台面。
依据：[Gazebo 11 ODE 实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/ode/ODEPhysics.cc)。

当前验证参数：20 mm / 10 g 方块，P=0.045，力矩上限 0.01 N·m，
quick/40、1 ms 步长，两个指面有限抗扭半径 4 mm、系数 0.8；
70 mm 侧向退出，抬升目标 35 mm，经侧向中间点回收至 base=(0.12,0.015,0.03) m。
先实际横向离开固定支撑台，不删除台面、不绑定方块，之后整臂带物回收。

| 样本 | 悬空保持 | 保持阶段接触与力 | 物体保持结果 |
| --- | --- | --- | --- |
| demo_cube20_p045_patch4_01 | 原始 CSV 30.046 s | 双侧 100%，Z 向 0.09655 N | 未掉落，保持相对漂移约 1.32 mm；收尾 JSON 格式错误，不能算正常完成的 runner 样本 |
| demo_cube20_p045_patch4_02 | 正常退出，30.045 s | 双侧 100%，法向 0.1605 / 0.1617 N，Z 向 0.09659 N | 最低抬高 38.64 mm，保持相对漂移 1.19 mm，demo 与保持阶段 strict 均通过 |
| demo_cube20_p045_patch4_120s_01 | 正常退出，120.070 s | 双侧 100%，法向 0.1526 / 0.1554 N，Z 向 0.09658 N | 最低抬高 33.04 mm，相对漂移 6.96 mm；未掉落，demo 通过，strict 失败 |

复测的最初携物—回收阶段相对位置变化约 3.95 mm，单独记录，
不能因重新设置保持参考点而忽略。保持阶段没有支撑台、夹爪本体或其他组件接触，
方块位姿始终有限，仿真时间严格递增；保持期 PID 力矩峰值约 0.00956 N·m，
未提高原 0.01 N·m 限幅。Z 合力约为重力的 98.5%，是近似接触测量，不宣称精确平衡。

注意：strict_retention_passed 只对应保持阶段的漂移/抬高判据，
不代表整条运动各点均达到 PX4 正式 5 mm 到位规则。
该复测回收点实测误差约 1.71 mm，但瞬时 position_reached=false；
附近检查与离台抬升仍使用实验门槛 15 mm。正式到位规则未放宽。

收尾格式问题来自 NumPy 比较产生 numpy.bool_，在物理保持结束后不能序列化 JSON。
已将验收布尔值/抬高量转换成原生 bool/float，增加独立格式回归测试；
13 项离线测试通过。失败样本原始 CSV 不覆盖、不事后补造成正常成功。
本轮所有说明集中于本文件；没有修改 ROS 库、核心 arm_control 或正式模型。
以上抓取试验执行期间未提交或推送；之后按用户要求单独归档，见下节。

复现命令（已有 case 名拒绝覆盖，需换新名字）：

```bash
python3 src/modules/arm_control/tests/grasp/run_full_arm_grasp.py new_patch4_hold30 \
  --calibration build/full_arm_grasp/full_calibration_05/calibration.json \
  --grasp --retrieve --retrieve-route stow --hold-seconds 30 --acceptance demo \
  --gripper-p .045 --side-clear-mm 70 --arrival-mm 15 --patch-radius-mm 4
```

完整结果见 build/full_arm_grasp/demo_cube20_p045_patch4_02/summary.json；
poses.csv 是原始遥测，manifest.json 记录场景参数和源码、模型、控制库哈希。
120 s 长测已经完成，最初携物—回收相对位移 3.93 mm，之后保持阶段漂移 6.96 mm，
两段分别记录。保持期仍有缓慢位置/姿态变化，不宣称彻底消除了滚转或长期漂移；
但本次完整序列没有脱落、非夹指干涉、台面接触或物体位姿 NaN。
结果保存在 build/full_arm_grasp/demo_cube20_p045_patch4_120s_01/summary.json。
复现长测时换新 case 名，并将上述 --hold-seconds 改为 120。

当前达到的是固定机身、固定目标、整臂带物回收并保持 120 s 的物理抓取正例。
本轮启动的仿真在采样后关闭；未操作用户的其他仿真。
还没有完成全姿态、外部扰动、无人机飞行携物或自动抓取状态机。
建议保留这套已测参数作为后续流程封装的基线，暂不继续增加 P 或力矩上限，
也不为追求零滑移改变本轮 demo 判据。

## 更新归档（2026-09-28）

按用户要求将近期更新分成三项 Git 提交：测试目录整理（同时修正相对路径）、
完整整臂抓取/回收/保持实验代码、合并说明与实测结果。
原五份 SO101 说明合并为本文件，不再保留重复的独立文档。
ROS 库不提交；生成的场景、二进制、CSV 与结果 JSON 留在忽略的 build 目录。

提交前复核：13 项抓取离线测试通过，102 个位姿的 SDF/FK 对比通过，
夹指几何读取正常，到位判定 C++ 测试通过，Git 格式检查通过。
正式 Gazebo 模型子仓库保持干净；本次不改变该子模块指针或固件核心控制流程。

## 第八步：自动抓取、夹持反馈与失败退出

第 1–10 小节保留 2026-09-28 经用户校核的 v0.1 设计基线。
用户随后授权实现与测试；第 11 小节记录实际实现、物理结果及与设计的差异，
以该小节为当前完成状态。所有实验只访问 Ubuntu-20.04；ROS 库未修改。

### 1. 第一版范围与已有能力

第一版沿用固定机身、固定倾斜支撑、20 mm / 10 g 方块的已验证 bench 场景，
目标是一次请求完成接近、对齐、双侧夹持、侧向离台、抬升、回收与保持。
默认保持 30 s，长测 120 s；允许有限滑移，不增加 P 或力矩上限。
使用显式 bench_cube20_v1 配置：P=0.045、cmdMax=0.01 N·m、
两指抗扭半径 4 mm / 系数 0.8、quick/40、1 ms 步长。
这些仍是专用测试场景设置，不全局修改正常飞行世界的物理参数。

第一版不接受任意世界坐标抓取，不做全姿态 IK、飞行控制、点云、SUPER、
自动放置或自动重抓。后续先增加坐标转换与抓取姿态/路径选择，再开放任意目标。
ALIGN 使用 Gazebo 物体真值，属于显式的 simulation-assisted alignment；
不宣称由实机传感器或点云完成定位。以后定位输入可以替换，不能隐式沿用真值。

已核查可复用：arm_control 10 Hz 控制循环、五关节位置 IK、
关节位置/速度/加速度约束、目标序号、关节反馈有效性及 FK 实测末端位置。
现有 ArmControlState 的 Ready/Active/Fault 是底层控制状态，
不改成抓取阶段枚举；另设抓取状态和结果，避免改变既有语义。
现有 ArmJointStatus.current 未测量，接收端填 NaN，不能用它确认夹持。

### 2. 三层职责与单一命令所有者

```text
grasp_start / cancel / reset
             ↓
GraspStateMachine：阶段、条件、超时、结果（不直接发 MAVLink）
             ↓
ArmControl：目标校核 → 现有 IK → 现有关节轨迹 → arm_joint_command
             ↓
原 MAVLink/Gazebo 控制链 → 六关节控制器

Gazebo contacts + 实际 link/object pose
             ↓
只读 GraspFeedbackCollector → ARM_GRASP_FEEDBACK → PX4 接收校验
             ↓
arm_grasp_feedback → 状态机；原关节反馈仍走 arm_joint_status
```

状态机设计成可离线测试的 C++ 类：step(now, inputs) 返回有限的行动请求，
不持有 Gazebo/ROS 对象、不睡眠、不直接控制关节。
ArmControl 负责执行行动，并回传 accepted_target、target_sequence 和拒绝原因。
ACTIVE 抓取及带物保持/故障锁定期间，普通 arm_control target 请求返回 BUSY，
不静默抢占。重复 START 不重置阶段和计时器；CANCEL 可打断所有运行阶段。
只有一个执行器发布 arm_joint_command，测试脚本不再并行发送逐步运动目标。

拟在 Run 中依次：读取关节/抓取反馈和请求 → 计算健康与到位指标 →
状态机推进或安全退出 → 接受唯一来源的目标 → 更新现有关节轨迹 → 发布两种状态。
请求处理不能放在当前“未初始化/关节无效就提前 return”之后，
否则正好在故障时无法响应 CANCEL/STATUS。每周期动作与 IK 调用必须有界，
长路径检查分批执行，不能在 work queue 内阻塞等待。沿用 uORB 异步接口，
参考 [PX4 uORB 说明](https://docs.px4.io/main/en/middleware/uorb)。

### 3. 建议的状态与迁移条件

```text
IDLE → PRECHECK → APPROACH → ALIGN → CLOSE → VERIFY
                                              ↓
DONE ← HOLD ← SETTLE ← RETRACT ← LIFT ← EXIT_SUPPORT

任意运行阶段 → ABORTED（取消）/ FAULT（失败），锁定，不自动重新执行
```

| 状态 | 动作与完成条件 | 暂定阶段超时（仿真时间） |
| --- | --- | --- |
| PRECHECK | 检查场景配置、目标身份、固定机身、反馈能力；预检路径边界/IK，要求关节及抓取反馈连续健康 0.5 s | 5 s |
| APPROACH | 夹爪打开 0.6 rad，到已测附近点 base=(0.17,0.035,0.05) m；满足 bench_near 并停稳 | 35 s |
| ALIGN | 实测物体中心相对夹指参考点；每次最多修正 3 mm，增益 0.6，修正间隔至少 1 s；误差连续三次 ≤3 mm、速度合格 | 120 s |
| CLOSE | 固定五关节目标，夹爪目标 -0.15 rad，经现有限速；至少观察 2 s，再检查夹爪停稳和双侧接触候选条件 | 12 s |
| VERIFY | 再连续 1 s 确认同一目标的双侧接触、两侧法向力与数据新鲜；保存抓取初始相对位置和高度 | 3 s |
| EXIT_SUPPORT | 沿当前 bench 的 base +Y 退出 70 mm；带物小步移动，停稳后观察 1 s；物体确实离台、夹爪本体侧向净空 ≥3 mm | 60 s |
| LIFT | 从离台点沿 base -Z 抬升目标 35 mm；物体实际抬高 ≥25 mm、无支撑且运动停稳 | 60 s |
| RETRACT | 保持侧向距离，先到 (0.12, cleared_y, lifted_z)，再到 (0.12,0.015,0.03) m；两段分别确认接受和停稳 | 每段 60 s |
| SETTLE | 停稳并保持 2 s；仍高于初始至少 25 mm，无支撑/非夹指干涉；单独记录携物阶段相对位移 | 5 s |
| HOLD | 连续保持 30 s 或 120 s，持续检查载荷与反馈；结束时验收本阶段接触比例和保持高度 | 请求时长，不超过 120 s |
| DONE | 锁存完成结果，继续带物保持和健康监视，不自动松开；后续载荷丢失仍报告新的故障 | 无自动松爪计时 |
| ABORTED / FAULT | 取消所有未执行的路径目标，保持或停止下发，锁存原因，等待明确处置 | 不自动退出 |

阶段超时取实际 now-stage_enter_time，不用限幅后的轨迹 dt 累加。
SITL 需核对 hrt 与仿真时钟推进的一致性；暂停时阶段时间不推进，
时间倒退/世界重置视为 CLOCK_RESET，清除接触窗口并锁定退出。
外部测试器另设墙钟无进展保护；主动暂停或时钟卡住只记 STALLED，
不能把它当成物理 NaN，也不能伪造一条 PX4 已执行的退出事件。

新增 LIFT 实际抬高 ≥25 mm 的门槛已用两份既有成功 CSV 对照：
lift_arrived 时相对闭合确认高度分别抬高 29.349 / 29.344 mm，
方块—台面接触比例均为零。该检查是旧日志核对，不是状态机已经通过的测试。

### 4. 到位、对齐、夹持必须分别判定

- strict_position_reached：原 ArrivalCheck 不变，即末端 ≤5 mm、
  五关节差 ≤0.03 rad、速度 ≤0.05 rad/s、反馈新鲜并持续 0.5 s。
- bench_near：仅在已确认的 bench 配置内允许末端 ≤15 mm、
  五关节差 ≤0.08 rad、速度 ≤0.05 rad/s、持续 0.5 s。
  状态中明确记录此条件及 strict_position_reached，不冒充严格到位。
- aligned：夹指参考点与物体中心 ≤3 mm，连续三次观测，间隔至少 0.6 s，
  目标身份/位姿有效、五关节速度 ≤0.05 rad/s；不能只看 IK 求解误差。
- clamped：最近 1 s 的同目标双侧接触帧比例 ≥90%，
  两侧平均法向力各 >0.01 N、夹爪速度绝对值 <0.02 rad/s、数据新鲜，
  并满足 VERIFY 的连续确认。允许物体此时仍由台面支撑。
- payload_free：离台且实际抬高达标；这才允许设置载荷状态为 HELD。
  CLOSE 的目标角 -0.15 rad 不要求实际达到：有物体时实际角约 +0.06 rad
  可以正常夹持；空爪到达闭合角也不能判为抓取成功。

HOLD 结束的 demo 验收：整段按物理帧加权的双侧接触 ≥90%，
两侧平均法向力各 >0.01 N、支撑接触 ≤0.1%、最低物体高度仍高于抓取初值，
全程没有持续接触丢失/明显分离/非夹指干涉，结束时反馈仍健康。
不合格返回 RETENTION_CHECK_FAILED；严格漂移 ≤5 mm 仅作独立诊断，
不改变 demo 的成功结果。进入 SETTLE/HOLD 的 ≥25 mm 抬高要求仍不可跳过。
payload_state=LOST 须由有效物体位姿/明确分离证据确认，
只有接触失联时使用 UNKNOWN，不凭失联猜测物体已经掉落。

坐标约定：所有运动点在 SO101 base_link、单位 m；参考点在 gripper_link：
(0.0025,-0.000218,-0.088) m。现有 FK 末端是 gripper_frame_link，
必须使用模型固定工具变换还原 gripper_link，再计算
e_base = R_base_gripper_link * (object_center_gripper - reference_gripper)。
不用测试脚本针对倒挂固定机身的世界 [1,-1,-1] 换号写法泛化任意安装。
关节 FK 和仿真实际坐标需独立交叉核对；机身一旦解锁，此 bench 配置立即失效。

携物目标推进沿用每 0.5 s 最多 3 mm；只在前一目标已接受、反馈健康、
实测误差处于 bench 包络且碰撞保护通过时推进。接近与微调也核对
accepted_target，要求与请求小步点差 ≤0.2 mm；工作空间裁剪不算成功接受。
不能只看到 target_sequence 增加就继续，也不能每步等待严格停稳造成无限停顿。

### 5. 新增接口草案（名称/字段待最终生成校核）

保留 ARM_JOINT_COMMAND(42000)、ARM_JOINT_STATUS(42001) 的现有布局，
不往 current 填力矩或接触力。新增 ARM_GRASP_FEEDBACK，拟消息编号 42002；
本地检查未见同号 message，最终需验证 so101.xml 的完整 include 图并运行生成器。
本地 ardupilotmega 中 42002 是 MAV_CMD 枚举值，不是 message ID；
也不把项目自用编号称为全局注册号。修改方言应单独提交子模块，
遵循 [MAVLink 消息定义与生成规则](https://mavlink.io/en/guide/define_xml_element.html)。

| uORB 接口 | 建议字段及语义 |
| --- | --- |
| ArmGraspRequest.msg | 非零 request_id、action=START/CANCEL/RESET、profile_id、target_id、hold_duration_s；第一版只接受 bench_cube20_v1，不开放任意 XYZ |
| ArmGraspFeedback.msg | 本地接收 timestamp、源 sample_time_us、source_sequence、world_epoch、profile_id、target_id、source=SIM_GROUND_TRUTH、有效能力位；contact_frames、sample_window_s、两侧/双侧/台面/非夹指/台面—机械臂接触比例；两侧法向力、世界 Z 向支持力；object_center_gripper[3]、object_height_world、gripper_body_support_clearance_m、bench_valid |
| ArmGraspStatus.msg | active_request_id、最近请求 ID/接受或拒绝原因、phase、phase_elapsed_s、result=NONE/RUNNING/SUCCEEDED/CANCELED/FAILED、fault_reason、payload_state=EMPTY/CONTACT_CANDIDATE/CLAMPED/HELD/LOST/UNKNOWN、target_sequence、strict_position_reached、bench_near、alignment_error、feedback_age、接触/力诊断、initial_motion_shift、hold_drift、actual_lift |

建议命令外观：arm_control grasp_start bench_cube20 30、
arm_control grasp_cancel、arm_control grasp_status、arm_control grasp_reset。
这些子命令现已实现；执行命令返回“queued”不等于状态机已经接受。
START 必须确认当前请求已接受；单一请求者一次只保留一项未确认请求。
运行时再次 START 返回 BUSY，不能覆盖 active_request_id。
CANCEL 和结果均带请求身份；回放同一 request_id 不重新启动。
RESET 不发运动命令，也不松爪；载荷 HELD/CLAMPED/UNKNOWN 时不能直接
放开手动命令所有权，需要后续明确的人工接管/支撑确认。
本版不提供自动松爪接口；不能将 reset 设计成“恢复初始姿态”。

有效性位至少区分 CONTACT_VALID、FORCE_VALID、OBJECT_POSE_VALID、
CLEARANCE_VALID、BENCH_VALID；无能力/无测量与真实零值不同。
没有物体时正常发布新心跳：object pose 无效、接触帧仍可有有效的零接触计数。
物体消失也不能只停止发消息，否则会把“对象丢失”误诊为链路超时。
只有必需且标记有效的字段才做有限性检查；未测电流 NaN 不触发此物理故障。
第一版必须具备上述五项能力，否则拒绝自动抓取，不能降级为仅靠角度猜夹持。

### 6. 只读夹持反馈如何进入 PX4

建议在现有 GazeboMavlinkInterface 内挂接独立 GraspFeedbackCollector，
读取联系人和模型实际位姿，经既有 send_mavlink_message 路径发新反馈。
不新增 UDP 端口，不增加第二个关节命令发布器，不依赖 ROS，
也不需要为了本功能额外建立 Gazebo 自定义 protobuf 传输链。
collector 只读，不能 SetPosition/SetForce/绑定物体、移动台面或暂停物理。
默认禁用，由专用场景明确配置目标 model/ID、支撑 model 和指面 collision。
目标身份与接触对按解析后的准确 scoped 名称匹配，避免任意同名夹指/第二个物体
满足条件。两个指面接触不同物体不能记为 bilateral。

采样按物理帧去重，两个指面同一帧接触才记双侧；建议每 0.1 s 仿真时间
发布一次新窗口。状态机再按 contact_frames 加权聚合 1 s，不能按消息数平均。
空窗口、重复帧、倒序帧不得补成接触；contact_frames=0 时接触能力无效。
力转换使用时间匹配的目标 link 姿态；若无法匹配则标记 FORCE_INVALID，
Z 向力仅作诊断，不强制其与 0.0981 N 精确相等。
支持碰撞和本体干涉需单独分类，不从“夹指接触比例高”推断已经离台。
净空仍限定 bench 的退出方向和碰撞盒，不能声称完整碰撞规划器。

collector 保留样本时间、源序号与 world_epoch。桥接只发送新样本，
接收端只用新的源序号刷新接收龄期，重复旧包不能延长生命。
需核验 SITL 源时钟与 PX4 时钟对应关系，记录采样龄期与接收龄期两者；
未知同步/窗口时间过长不得标成 fresh。实现时覆盖序号回绕、世界重置、
乱序和延迟缓存回放测试，不能只以“刚收到 MAVLink”为新鲜依据。
关节反馈龄期 <0.2 s 才推进；0.2..0.3 s 暂停推进，超过 0.3 s 锁定失败。
抓取反馈采样/接收龄期超过 0.3 s 同样锁定失败；阈值需实测 10 Hz 调度裕量。
源身份、目标/配置身份变化或已验证时基倒退在运行中立即退出。

### 7. 失败退出与持物责任

| 情况 | 结果/原因 | 默认动作 |
| --- | --- | --- |
| 未初始化、场景不符、不可达/越界目标、缺必要反馈能力 | START_REJECTED，保持 IDLE | 不进入运动，不用裁剪后的另一位置替代原请求 |
| 请求动作进行中的 IK 拒绝/目标被裁剪 | FAILED / IK_REJECTED | 停止该路径，锁定故障 |
| 近点、对齐、开合或回收阶段超时 | FAILED / PHASE_TIMEOUT（保留具体阶段） | 不跳过完成条件，不进入后续阶段 |
| 空抓、单侧接触，VERIFY 到期仍不合格 | FAILED / GRASP_NOT_CONFIRMED | 保持现有臂姿和夹爪命令，不自动松开或重抓 |
| 携物相对抓取参考点分离 >45 mm | FAILED / PAYLOAD_SEPARATED | 停止臂运动，不追逐掉落物 |
| 携物双侧比例 <50% 持续 >0.5 s | FAILED / CONTACT_LOST | 锁定退出，不以一次短暂丢帧直接判掉落 |
| 非夹指/台面—臂接触比例 >20%，或抬升后再次接触支撑 >1% | FAILED / OBSTRUCTED 或 SUPPORT_RECONTACT | 停止推进，不能穿越台面继续收臂 |
| CANCEL | CANCELED | 停止未执行路径，不自动回到初始关节姿态 |
| 抓取反馈丢失而关节反馈仍健康 | FAILED / GRASP_FEEDBACK_STALE，payload=UNKNOWN | 有界停止五关节运动，保持原夹爪闭合目标和限幅，继续控制 |
| 关节反馈超时/有效字段出现 NaN/Inf | FAILED / JOINT_FEEDBACK_STALE 或 NONFINITE_STATE | 停止轨迹推进和新运动命令，不基于旧反馈继续 IK；交由既有下位超时策略，观测结果 |
| 时间/世界重置、目标/源身份变化 | FAILED / CLOCK_RESET 或 SOURCE_CHANGED | 清除窗口/未执行动作，须人工核对后重启新请求 |

关节健康时停止动作需清除尚未执行的小步及轨迹残余速度，
采用有界减速/当前实测位置附近保持，防止只是“停止状态机”而旧目标仍继续走完。
保留夹爪闭合偏置：若改成实际接触角作为目标，可能卸掉夹持力。
关节反馈不健康时，不保证持物安全；下位命令超时保持当前位置可能降低夹持力，
必须实际验证。故障试验需接物托盘和固定机身，不能将其包装成飞行安全保证。
grasp_cancel 不等于 arm_control stop；后者结束模块，不能承诺持续夹持。

没有自动复位、自动重抓、盲目撤退或空中松爪。载荷状态不可确认时是 UNKNOWN，
不是 EMPTY。成功结果锁存仅表示该请求完成了指定保持时间；DONE 后仍监测载荷，
历史 completed_success 与当前 payload_state 分别呈现，不能继续显示“当前抓取成功”
而忽略后来丢失。初始携物位移与保持漂移分别记录，demo 小滑移不触发精度失败。

### 8. 拟改文件与实施顺序

```text
src/modules/arm_control/
  grasp/GraspStateMachine.hpp/.cpp     纯状态与阶段/请求管理
  grasp/GraspChecks.hpp               窗口判据、健康与持物判断
  grasp/GraspProfile.hpp              有版本的 bench 配置与路径
  arm_control.cpp/.hpp                只增加适配、所有权、命令和反馈接入
  CMakeLists.txt                     登记实现源文件
  tests/grasp/test_grasp_state_machine.cpp
  tests/grasp/run_full_arm_grasp.py    新增 FSM 模式，不再外部逐阶段发目标
  tests/grasp/full_arm_probe.cpp      保留独立、只读的验收观察器
msg/ArmGraspRequest.msg / ArmGraspFeedback.msg / ArmGraspStatus.msg
msg/CMakeLists.txt                    注册消息
src/modules/mavlink/mavlink_receiver.cpp/.h  新反馈解码/校验/发布
```

另需修改 MAVLink 方言子仓库的 so101.xml；Gazebo 子仓库内新增
include/so101/grasp_feedback_collector.h、src/so101/grasp_feedback_collector.cpp，
并接入 gazebo_mavlink_interface.cpp/.h 与其 CMakeLists.txt。
专用场景生成器显式开启 collector 和 bench 物理配置；正式模型默认不启用自动抓取。
不修改 ROS 仓库。提交应先处理并推送 MAVLink/Gazebo 子仓库，再提交 PX4 指针，
避免再出现父仓库引用尚未上传的消息定义；本设计阶段不执行这些操作。

建议分三步实现并各自验收：
1. 只读反馈链：无任何自动动作，核对身份、窗口、坐标、力和超时，
   与独立 probe 对照；不让单一 collector 既控制又独自证明成功。
2. 纯状态机与离线故障测试：用输入序列测试迁移、计时、所有权和退出，
   完成后只做 APPROACH/ALIGN 的不闭合仿真试运行。
3. 完整闭合/离台/回收/保持与真实故障注入；暂不改无人机飞行逻辑。

### 9. 验证清单与通过条件

| 测试 | 必须看到的结果 |
| --- | --- |
| 单次正常请求，30 s / 120 s | 独立 observer 确认真实双侧抓住、物体离台抬高、回收保持，未绑定，无非夹指干涉/NaN；结果含每个迁移与阈值来源 |
| 正例新启动重复至少三次 | 独立场景恢复后每次都完成，不覆盖旧日志；明确这是新启动重复，不冒充同场景连续抓取/放置循环 |
| 空物体/偏置到单侧接触 | 不出现 HELD/SUCCEEDED；分别拒绝启动或在 VERIFY 合理失败，无后续抬升 |
| 不可达/超界目标 | 明确拒绝，不偷偷移动到裁剪后的位置；现有手动功能不受影响 |
| 停掉抓取反馈、关节反馈分别测试 | 区分两种超时；锁存原因和阶段，不自动恢复，不丢失 CANCEL 响应 |
| 重复/乱序/缓存旧包、world reset | 不能刷新生命或凑足双侧接触条件；运行中的源/时基变化退出 |
| 运行中 START/普通 target，阶段中 CANCEL | START/target 返回 BUSY，无双命令竞争；CANCEL 停止残余路径且不松爪 |
| 带物后短暂接触丢帧/持续失联/物体脱落 | 短暂缺口按去抖处理，持续丢失或明显分离退出；未把 UNKNOWN 误记 EMPTY |
| 到位门槛未满足、力测量无效或 NaN | 不靠固定等待跳过，不伪造成功，不因为未测电流而误报物理 NaN |
| HOLD 完成后再失去物体 | 保留历史完成事件，同时更新当前载荷和故障，不能静默失去监视 |

离线测试可以用合成消息检验逻辑；真实抓取成功必须由物理仿真和独立 observer
验收，禁止注入“接触为真”来证明抓取。故障注入放在测试器的桥接过滤层，
明确标记实验参数，不在正式消息中留可远程伪造成功的测试开关。
结果仍写 build/full_arm_grasp/<unique_case>/，包括源码/库/场景/配置哈希、
时钟关系、请求与阶段事件、反馈能力/龄期、力和位姿、退出后至少 2 s 观测。
墙钟保护清理只限测试器自己启动的进程，不删除物体去制造“已离台”。

### 10. 请优先校核的三项选择

1. 第一版只做 bench_cube20_v1，不先开放任意 XYZ；确认其取舍是否符合近期 demo。
2. 允许第一版 ALIGN 使用明确标记的 Gazebo 真值；以后再替换为点云/目标估计输入。
3. 取消/失败保持、锁定且不自动重抓或回收；载荷不明时不允许 reset 直接释放命令所有权。

这些边界已由用户批准。参数阈值继续以物理实测为准，
不因为状态机首次跑不通就放宽正式到位判据或调大夹持力矩。

### 11. 实现与验收记录（2026-09-29）

#### 实现位置与反馈语义

- `grasp/GraspStateMachine.hpp/.cpp`：无阻塞、无动态分配的状态机、接触窗口、
  阶段计时、所有权和锁存退出；`grasp/GraspAdapter.cpp` 负责 uORB/现有 IK 适配。
- `grasp/GraspFeedbackValidation.hpp`：能力位、有限性、比例、时间窗口、
  源序号/时间去重与 epoch 校验。草案中的 GraspChecks/GraspProfile 尚未单独建文件。
- 新增 `ArmGraspRequest/Feedback/Status.msg`；状态包含 active/last request ID、
  接受/拒绝原因、当前阶段、失败发生阶段、载荷状态、历史完成与实际抬高。
- MAVLink 子仓库新增 `ARM_GRASP_FEEDBACK(42002)`，原 42000/42001 布局不变。
  Gazebo 的 `so101/grasp_feedback_collector` 只读真实接触/位姿，不绑定物体。
- 同时接入普通 `mavlink_receiver` 和 **实际 SITL TCP 入口**
  `simulation/simulator_mavlink/SimulatorMavlink`。第一轮没有新反馈，是后者漏接；
  不是模型或 PID 故障。仍使用 development 方言，不改为另一种头文件。
- 修复 MAVLink 构建依赖：被 development 包含的 so101.xml 改变时也重新生成头文件，
  避免旧生成物掩盖新增消息。MAVLink/Gazebo 子仓库均保留未提交改动。
- `tests/grasp/auto_grasp_case.py` 仅发送 START/CANCEL，观察器独立验收真实物理结果。
  `mavlink_fault_proxy.py` 只过滤完整指定消息，保留原 CRC/其余消息/执行器命令。
  日志明确区分源采样时刻与外部观察延迟、运行错误与清理时连接关闭。

反馈由同一个目标的双侧接触、两侧法向力、实际物体相对位置和实际抬高组成，
不以空爪角度到位或“命令发出”确认抓取；电流仍未测量，继续是 NaN。
SOURCE=SIM_GROUND_TRUTH 显式标记仿真真值，不是实机电流。
只有实际抬高达标后载荷才为 HELD，完成指定 HOLD 后才 SUCCEEDED。
DONE 继续监视，`completed_success` 是历史记录，不等于当前载荷安全。
本版 profile 只允许 SITL；不能在硬件上使用 Gazebo 真值启动自动抓取。

collector 默认关闭，仅专用生成场景设 enableArmGraspFeedback=true。
实际校验固定锚点/安装、20 mm/10 g 目标、P/I/D/力矩上限、指面抗扭参数、
ODE quick/40/1 ms。目标或支撑被同名新实例替换也不能沿用原目标身份。
生产 SDF、飞行世界和 ROS 库均未修改。

#### 发现并修正的问题

1. 折叠初始 FK 的 Z≈−5 mm，原工作空间 min_z=20 mm；接近小步被裁剪，
   按“不能把裁剪当接受”的要求退出 IK_REJECTED。
   增加仅对已验证 bench APPROACH 的 **单调向内回归**：
   最大步长仍 20 mm，只减少初值到合法区间的距离，不允许越界向外走。
   普通目标、其他抓取阶段、IK/关节/速度限制和正式严格到位阈值不变。
2. EXIT_SUPPORT 的额外 1 s 改为从实测到位后起算，而不是从进入阶段起算。
   ALIGN 三次合格观察在速度/位置不合格时重新计数。
3. 接近 CANCEL 现在在首个运动目标已接受后触发，避免只测试“尚未运动就取消”。
   取消与抓取反馈超时均确认五关节目标不继续推进；夹爪保留原命令偏置。
4. 测试过滤层启用 TCP_NODELAY，避免代理的小包缓冲拖慢 lockstep。
   首轮慢速代理试验人工中止、未注入故障，保留日志且不计通过。

#### 当前已有物理结果

结果目录统一为 `build/full_arm_grasp/<case>/`：
每轮有 scene.world、manifest.json、poses.csv、px4_status.json、summary.json 及运行日志；
唯一编号不覆盖失败试验。环境固定 seed=123，未移动/删除支撑、未绑定方块。

| 已完成案例 | 验收结果 |
| --- | --- |
| fsm_feedback_20260929_02 | 5 s 内 48 个新窗口，五项能力齐全；源采样/接收/仿真时钟一致 |
| fsm_grasp_20260929_02 | 请求保持 30 s，独立观察约 32.15 s；双侧 100%，最低抬高 37.52 mm，最终相对漂移 2.07 mm，支持力均值 0.09634 N |
| fsm_grasp_20260929_03 | 独立观察约 31.94 s，最低抬高 38.38 mm，最终漂移 1.56 mm；运行中 START=BUSY，手动目标/改动 IK 的调试命令被拒绝，带物 RESET 不释放所有权 |
| fsm_grasp_20260929_04 | 独立观察约 32.05 s，最低抬高 38.35 mm，最终漂移 1.56 mm；与前两轮分别重新启动场景，均完成完整抓取 |
| fsm_grasp_120s_20260929_01 | 请求保持 120 s，独立观察约 122.11 s；最低抬高 33.63 mm，最终漂移 6.35 mm；demo 持物通过，不冒充严格精度通过 |
| fsm_cancel_approach_20260929_02 | 首个目标接受后取消；ABORTED 锁定，后续 2 s 五关节目标变化为 0；非带物阶段原开爪目标可以继续完成 |
| fsm_cancel_carry_20260929_01 | 携物阶段取消；后续 2 s 目标变化为 0，夹爪角变化约 −0.000055 rad，仍双侧夹持，不自动松爪/回收 |
| fsm_drop_grasp_20260929_02 | 过滤 42002，外部观察约 0.32 s 后 GRASP_STALE，failed_phase=LIFT，payload=UNKNOWN；后续 2 s 仍双侧夹持 |
| fsm_drop_joint_20260929_01 | 过滤 42001，外部观察约 0.533 s 后 JOINT_STALE；停止新增指令，UNKNOWN 锁定；**下位超时后物体掉落，持物安全不通过** |
| fsm_drop_joint_20260929_02 | 源 HIL 时钟锚点后首次可见故障 0.384 s，外部观察 0.427 s；failed_phase=LIFT，新关节指令时间戳不再前进；退出后物体分离 1.083 m，exit_payload_retained=false |
| fsm_drop_grasp_20260929_03 | 源首个被过滤窗口后首次可见故障 0.232 s（该窗口之前已有一个正常 0.1 s 间隔），故障时真实反馈龄期 0.340 s；外部观察 0.320 s；退出后仍双侧夹持，exit_payload_retained=true |
| fsm_missing_object_20260929_01 | START 明确拒绝，保持 IDLE，不出现 HELD/SUCCEEDED，也不进入自动运动 |

最后两轮复测独立核对 FK 与实际 link/物体坐标，最大差约 0.28 mm，
远小于 3 mm 验收限值。源锚点与外部观察延迟不是同一个指标，不能混为控制超时门槛。
上述正常/取消试验物体与机械臂均有限值，无 NaN、非夹指或支撑—臂干涉。
关节中断掉落时曾出现短暂非夹指接触（最大约 9.35%），同样保留，不隐藏故障物理后果。
故障案例的 summary.success 只表示**预期退出行为验证成功**，不是抓取/持物成功；
grasp_validated=false，新增 exit_payload_retained 单独呈现实际退出后是否仍夹住。
正常抓取真实到位正例及示范保持已跑通，不代表飞行、任意物体/姿态或重复放置循环。

#### 必须保留的限制与下一项修复

**关节反馈中断导致掉落：**PX4 在失联时停止轨迹/新命令；
当前下位插件 1 s command timeout 将夹爪目标改为实际角度，卸掉闭合偏置，
两侧法向力归零，独立 CSV 看到方块落到地面。这是实测，不是仅凭参数推断。
本次不改 ROS 库，也不能把“退出正确”包装成“失联时保证持物”。
后续需要单独设计/验证夹爪持物模式与下位看门狗策略，再进入飞行携物 demo。

紧急停止现在是健康关节反馈下重新取实测五关节位置并清掉残余轨迹速度，
保留夹爪命令；不是草案中的完整有界减速轨迹。失联时不使用旧反馈做 IK，
没有自动松爪、重抓、撤回或解除所有权。持物/未知状态不能直接 RESET。
全路径预检、完整碰撞规划、硬件夹持传感、世界重置/旧包的真实链路注入等尚未完成；
新增 42002 的旧包/非有限值/时钟倒退、状态机完成后丢物等逻辑目前由离线合成输入覆盖。
旧 42001 没有测量源时间/序号，仍只使用本地接收/读取龄期；
不能把 42002 的去重能力推广成“关节测量旧包也已防回放”。
本次只编译验证 SITL，未进行硬件版编译或实机验证。
到位严格标志是原控制周期诊断，bench 阶段检查独立使用本周期实际 FK 和新鲜反馈。

复现基准命令（端口/现有 PX4 被占用时脚本拒绝启动，不会结束你的仿真）：

```bash
python3 src/modules/arm_control/tests/grasp/run_full_arm_grasp.py <unique_case> \
  --calibration build/full_arm_grasp/full_calibration_05/calibration.json \
  --grasp --retrieve --retrieve-route stow --gripper-p .045 \
  --patch-radius-mm 4 --arrival-mm 15 --fsm --hold-seconds 30
# --fsm-test feedback_only / ownership / cancel_approach / cancel_carry
#            drop_grasp / drop_joint / missing_object
```

本次暂未 Git 提交/推送；只写这一份合并说明，不增设多份修改记录。
离线最终回归：16 项抓取脚本检查、16 项 C++ 逻辑/反馈检查、到位判定检查通过；
102 个姿态的 SDF/FK 比较最大位置差 3.533 μm，通过。PX4 与 Gazebo 插件编译通过。
所有本次测试自行启动的 PX4/Gazebo/代理/观察器均已清理，三仓库 Git 格式检查通过。

## 第九步：夹爪失联保持与界面复现（2026-09-29）

### 修改范围与保持语义

本轮针对第八步发现的“42001 中断 → PX4 停发新命令 → 下位 1 s 超时卸力”修复，
不提高 P、力矩上限，不改 IK、严格到位门槛或上层故障锁存。
仅访问 Ubuntu-20.04，没有访问 Ubuntu22，没有 Git 提交/推送。

| 位置 | 本轮作用 |
| --- | --- |
| `/home/pcz/super_ws/src/so101_gazebo/src/joint_position_controller.cpp` | 加入显式 timeout policy；区分待处理目标与已施加目标，诊断误差对应实际生效目标；校验有限 PID/正力矩上限；Reset 在更新线程清理旧目标/积分 |
| 同 ROS 包 `src/command_watchdog.hpp`、`tests/test_command_watchdog.cpp`、CMake | 独立、sim-time 看门狗与断言测试；Release 也执行断言；暂停不计超时，新命令才重新启用，时钟倒退/Reset 清除旧 episode |
| `tests/grasp/run_full_arm_grasp.py` | 专用 FSM 场景显式启用夹爪 last_target，其余关节配置不变；保存插件源码/库哈希；延长退出观测；GUI 等待入口与默认视角 |
| `tests/grasp/auto_grasp_case.py` | 退出行为与真实持物分开验收；独立检查双侧/力/位姿/非夹指干涉/力矩上限；长失联、恢复链路与命令丢失场景 |
| `tests/grasp/mavlink_fault_proxy.py` | 可过滤 42000，仍不修改帧内容/CRC，原 HIL/其余反馈正常转发；明确区分过滤命令与修改命令内容 |
| Gazebo 子仓库 `src/so101/grasp_feedback_collector.cpp` | bench 能力校验同时要求显式 last_target 配置，避免场景配置悄悄回退 |

控制插件默认仍是 `measured_position`：1 s 无 Gazebo 指令后，五关节目标取实测角。
只有关节名为 gripper 且 SDF 显式设置以下字段才允许保留最后目标：

```xml
<commandTimeoutPolicy>last_target</commandTimeoutPolicy>
```

目前该字段**只由专用 FSM 抓取场景生成器添加**，生产 so101.sdf 没改，
因此原 calibration 模型哈希仍有效。直接启动原普通 yhang550 场景不会自动启用此策略。
要部署到正式飞行场景，需显式配置夹爪并按实际参数另做验收，不能把 bench 结果冒充飞行验证。

`last_target` 保留的是最后收到、经关节限位校核、已实际施加的角度目标，
不是保持某个固定力矩，更不是自动判断“已经抓到”。闭合偏置 -0.15 rad 仍由
原 PID 与 0.01 N·m 上限约束，实际有物体时可以停在 +0.06 rad 附近。
若最后命令是开爪，则仍是开爪目标，不会因失联自行闭合。
不会自动卸力，也不会因超时自己追加闭合动作；恢复后的新有效命令可以改变目标。
软件保持依赖 Gazebo 物理与控制器继续运行，不能承诺掉电、进程崩溃或物理 NaN 后持物。

World/Model Reset 或仿真时间倒退会清除旧 command episode、待执行目标和该关节 PID
历史，以当前实测角重新设目标；**Reset 不保留旧夹持偏置**，不是带物安全操作。
这一项当前是编译/纯逻辑覆盖，尚未做整机真实 world-reset 注入，不能称物理验收通过。
42000/42001 原接口没有端到端命令确认或完整源去重；Reset 后到达的旧链路缓存指令
不能靠此看门狗识别。后续飞行版本仍需命令身份/确认与显式人工处置策略。

### 验收方式与已知边界

本轮专用退出验收至少观察 30 s，长测 120 s，不再只看退出后 2 s 的最后一帧。
持物要求：帧加权双侧接触 ≥90%、两侧法向力均 >0.01 N、无支撑接触，
相对抓取初值分离 ≤45 mm、持续双侧丢失不超过 0.5 s、无明显非夹指/台面—臂干涉，
实际关节力矩绝对值 ≤0.010001 N·m（仅数值容差），所有测量有限。
`exit_control_verified` 与 `exit_retention.retained` 必须同时满足才报告此负例通过。
PX4 在关节失联时仍是 FAULT / JOINT_STALE / UNKNOWN，保持所有权，不伪造 HELD。

五个臂关节转为实测角保持后，因重力与 PD 跟踪偏差会产生额外下沉；
这不是夹爪滑移，也不宣称末端位置保持精度通过。记录实际世界高度变化与相对位移，
用真实夹指接触/相对关系判断有没有掉物，正式严格到位阈值没有放宽。

首轮 `fsm_linkhold_joint30_20260929_01` 日志保留为失败：
双侧比例 100%、相对分离 3.38 mm、峰值力矩 0.00955 N·m，未掉物；
但新增的“相对闭合初值世界高度不得下降 10 mm”条件被实际约 14.90 mm 下沉触发。
后续删除这个混淆末端绝对位置与夹持的额外条件，保留高度作为独立诊断，
重新启动新案例验证，不覆盖或改写首轮失败结果。

直接过滤 42000 与过滤反馈不同：关节/抓取反馈仍健康，上层并没有命令 ACK
来直接识别这条链路丢指令。该案例验证下位看门狗与实际持物，
**不证明命令送达/端到端失联检测通过**，结果中 command_delivery_validated=false。

### 本轮物理结果

结果仍保存 `build/full_arm_grasp/<unique_case>/`，不绑定/移动方块或支撑。

| 案例 | 真实结果 |
| --- | --- |
| fsm_linkhold_joint30_20260929_02 | JOINT_STALE/UNKNOWN 锁存、停止新命令；退出观测 30.19 s，双侧 100%，相对分离 3.46 mm，峰值力矩 0.00949 N·m；独立持物通过 |
| fsm_linkhold_joint120_20260929_01 | 同样的关节反馈中断持续约 120.05 s，过滤 6027 个 42001；双侧 100%，相对分离 4.18 mm，峰值 0.00948 N·m，未掉落 |
| fsm_linkhold_grasp30_20260929_01 | 过滤 42002，GRASP_STALE/UNKNOWN；退出观测 30.19 s，双侧 100%，相对分离 3.45 mm，峰值 0.00948 N·m；停止路径，不松爪 |
| fsm_linkhold_cancel30_20260929_01 | 携物阶段 CANCEL 后 ABORTED 锁定，五关节目标不再推进；观测 30.09 s，双侧 100%，相对分离 3.30 mm，峰值 0.00939 N·m |
| fsm_linkhold_normal30_20260929_01 | 完整自动抓取/实际抬高/回收/保持通过；独立保持 32.15 s，最低抬高 37.96 mm，最终相对漂移 1.74 mm，strict 诊断通过，峰值 0.00953 N·m |
| fsm_linkhold_command120_20260929_01 | HOLD 后过滤 1220 个 42000，下位 1 s 看门狗真实触发；独立保持 122.01 s，双侧 100%，最低抬高 31.24 mm，最终漂移 8.41 mm，峰值 0.00949 N·m；demo 通过、strict 不通过，不能称命令送达验证通过 |
| fsm_linkhold_restore_20260929_01 | JOINT_STALE 后停止新指令，故障后约 5.02 s 恢复真实关节反馈；余下约 25.07 s 仍 FAULT/UNKNOWN/所有权锁定，target_sequence 保持 40，不自动重启路径；全退出窗口双侧 100%，相对分离 3.37 mm，峰值 0.00957 N·m |

以上七轮均无支撑/非夹指/支撑—臂干涉，物体/夹爪测量有限，CSV 仿真时间单调。
每一轮仍是独立重启场景，不冒充同场景连续抓取/放置循环。
早期案例的原 summary 不重写；用最终包含有限测力/持续接触检查的独立验收器
重新读取保存的 CSV，仍通过（不向仿真注入接触数据）。
下位日志明确五关节 `holding measured position`、夹爪
`retaining bounded last target=-0.15`，证明实际使用了不同的超时策略。

世界高度下沉诊断仍保留：普通关节失联约 14.88 / 15.56 mm，
抓取反馈失联约 14.75 mm，携物取消约 17.99 mm，
关节恢复后再按实测角重设五关节目标，累计下沉约 28.21 mm。
这提示后续要做有界停止与重力保持，而不能把这轮“夹爪不掉物”
当成整臂绝对位置安全已经解决；同样不能误把世界下沉值写成夹指相对滑移。

关节长失联负例与恢复负例均没有新成功事件，payload 仍 UNKNOWN。
120 s 命令中断案例的 DONE 只表示物理保持条件满足；其 command_delivery_validated=false。
如果需要飞行中直接报告“命令通路断了”，仍需端到端 ACK/源身份协议，不靠角度未变化猜测。

离线最终回归：18 项 Python 检查、16 项 C++ 状态机/反馈逻辑、严格 ArrivalCheck、
ROS 包登记的看门狗 CTest 均通过；看门狗 Release 使用 -UNDEBUG，断言未被跳过。
102 个 SDF/FK 姿态对照仍通过，最大位置差 3.533 μm。
更新后的 ROS 控制库与 Gazebo 接口库编译通过；GUI 场景 `gz sdf -k` 通过。
没有代开 GUI，因此窗口显示/交互要由下面的个人复现确认。

### 你亲自运行带界面仿真

本轮不替你打开 GUI。稍后在**能正常显示 Gazebo 的 Ubuntu20 终端**中运行：

```bash
cd /home/pcz/PX4-RoboMaster
python3 src/modules/arm_control/tests/grasp/run_full_arm_grasp.py gui_grasp_review_01 \
  --calibration build/full_arm_grasp/full_calibration_05/calibration.json \
  --grasp --retrieve --retrieve-route stow --gripper-p .045 \
  --patch-radius-mm 4 --arrival-mm 15 --fsm --hold-seconds 120 \
  --gui --gui-wait
```

已有 PX4/端口占用时脚本会拒绝启动，不会结束你的仿真。
若案例目录已存在，换一个新编号，不能覆盖之前的实验。
默认视角对准机身和机械臂。看到终端 `Press Enter to begin` 后，
可以先在窗口调整/放大视角，再回终端按回车，才发自动抓取 START。
等待期间物理与初始姿态保持仍在运行，并不是暂停物理；不会自动开始抓取。
观察顺序：打开夹爪 → 靠近/微调 → 双侧闭合 → 侧向离台 → 抬高 → 收臂 → 保持。
该演示机身锚定 world，**不会起飞**，没有点云/SUPER参与。
无需另开 roscore/MAVROS，沿用 PX4—MAVLink—Gazebo 独立数据链。
完成验收后脚本关闭自己启动的 GUI/后台进程；中途 Ctrl+C 同样清理。
如果没有 DISPLAY/WAYLAND_DISPLAY，则提前报错；沿用你已有的 Ubuntu20 图形配置，
不自动改显卡/显示服务，也不启动 Ubuntu22。

### 按原路线的下一阶段

先完成这一轮保持/反馈恢复回归与个人 GUI 复现，再进入飞行集成设计：
1. 将 bench 的固定安装/真值对齐约束与飞行姿态/目标坐标输入分开，制定新 profile；
   机身一解锁 bench 能力即失效，不能简单删 anchor 后继续用 bench START。
2. 先测起飞与空载折叠臂保持，再做已夹持物体的低速飞行/扰动与返航保持。
   这两项通过后再将“飞到抓取位 → 整臂抓取 → 回收 → 返航”串接。
3. 明确命令送达/反馈丢失、人工接管/安全放置与带物故障责任；不做空中自动松爪。
4. 最后接入点云目标估计、SUPER 接近规划与抓取姿态/全路径碰撞检查。

此处是后续路线，不表示飞行或世界重置已经测试成功。上述“未提交”描述的是本轮实验结束时的状态；后续阶段提交见下节。
本轮自行启动的 PX4/Gazebo/代理/观察器均已退出，4560/4562/11356 无监听；
最终四仓库格式检查通过；当时尚未提交/推送，未结束其他用户进程。

## 十、2026-09-30 阶段提交与飞行集成边界

用户已在带界面仿真中目视确认方块被夹爪夹住。这是固定机身 bench 抓取演示的结果，
不等同于飞行携物验证。提交前再次检查：18 项 Python 回归、16 项 C++ 状态机用例
通过，相关仓库差异格式检查通过。

按依赖顺序将本阶段实现分段提交：

1. MAVLink 子模块：`e5daeeef`，SO101 抓取反馈消息定义。
2. Gazebo Classic 子模块：`045351a`，接触反馈采集和发布。
3. PX4 主仓库：`8d32e26c65`，反馈传输与子模块引用；
   `459912e37a`，固定目标自动抓取状态机及故障退出测试。

ROS 工作区 `/home/pcz/super_ws/src/so101_gazebo` 的控制插件改动按约定暂不提交；
因此仅克隆上述三个 fork 仍不能重现完整的下位夹爪失联保持行为。
后续如需跨机器复现，应单独处理该 ROS 工作区依赖。

下一阶段先做飞行 profile：解除 bench 机身固定约束后，明确世界/机身/臂坐标系、
目标输入和接管时序；从空载折叠臂起飞与悬停开始，确认动力学和关节保持稳定后，
再做低速携物与返航。不要把当前 bench 抓取成功写成飞行抓取已通过。
