# SO101 控制、运动学与抓取实验（统一审阅文档）

本文件合并原 INTERFACE、KINEMATICS、ELBOW_TRACKING、TARGET_SWITCH、
GRASP 五份专项记录。后续修改和实验只维护本文；原记录可从 Git 历史恢复。
以下历史章节描述各阶段当时的结果，“未提交”和“下一步”不代表当前状态。
最新已验收抓取结果见第36节；纯雷达路线见第37节，S1/S2见第38/39节，S3a静态感知基线见第40节。
只使用 Ubuntu-20.04；历史章节的“未提交”“ROS 不提交”等仅描述当时状态。

## 目录与职责（2026-10-17）

```text
src/modules/arm_control/
  arm_control.cpp / arm_control.hpp    PX4 运行模块、IK、目标轨迹和反馈
  ArrivalCheck.hpp                    严格到位判定
  CMakeLists.txt / Kconfig             固件构建配置
  grasp/                             抓取状态机、悬停许可与无闭爪接近
  diagnostics/IKPreview.cpp           SITL 原生 IK 只读诊断，不投递目标
  tests/
    kinematics/                       FK 对照与到位判定边界测试
    grasp/                            整臂台架基线、夹持/故障回归与独立时钟
    hover/                            已知目标、飞行/许可/接近及只读 IK 检查
```

主机实验源码不加入飞控模块 CMake；diagnostics 实现仅在 SITL 预处理编译，
实机命令拒绝该入口。历史双连杆夹具和旧 SUPER 试验已归档，见第二十五步；
当前保留整臂基线和已知目标主链。运行结果放在 build 下，不放进源码目录。

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
   `459912e37a`，固定目标自动抓取状态机及故障退出测试；
   `bc1fdaaad4`，整合说明文档。

ROS 工作区 `/home/pcz/super_ws/src/so101_gazebo` 的控制插件改动按约定暂不提交；
因此仅克隆上述三个 fork 仍不能重现完整的下位夹爪失联保持行为。
后续如需跨机器复现，应单独处理该 ROS 工作区依赖。

下一阶段先做飞行 profile：解除 bench 机身固定约束后，明确世界/机身/臂坐标系、
目标输入和接管时序；从空载折叠臂起飞与悬停开始，确认动力学和关节保持稳定后，
再做低速携物与返航。不要把当前 bench 抓取成功写成飞行抓取已通过。

## 十一、2026-09-30 空载折叠臂起飞与定高基线

新增 `src/modules/arm_control/tests/flight/run_empty_arm_hover.py`：
使用生产 `yhang550` 模型，不加 bench 固定关节或方块；只启动 PX4、Gazebo、
MAVROS 与 `super_flight_manager_node`，不发送 SUPER 目标。
起飞前要求六关节相对折叠目标的最大误差 <0.05 rad、
最大关节速度 <0.05 rad/s，连续保持至少 2 s 仿真时间。
飞行中若折叠误差超过 0.10 rad 即中止，最终通过阈值为 0.05 rad。
脚本拒绝已有 ROS/Gazebo/PX4 会话，不调用
`start_super_sim.sh` 的全局 `pkill` 清理；失稳超过 25°、
横向漂移超过 1.5 m 或 12 s 未爬升时提前退出。
每轮独立目录记录 ROS 遥测、Gazebo/PX4 日志和结果摘要。

模型 SDF 质量字段合计：机身与旋翼约 2.079 kg、雷达 0.200 kg、
SO101 约 0.632 kg，共约 2.911 kg（未计嵌入模型中的其他小质量）。
四电机的 SDF 标称上限为每台 1000 rad/s、力常数 8.54858e-6；
理想静态总推力约 34.2 N，相对上述重量约 28.5 N，余量有限。
这只是模型容量估算，不代表飞控输出一定等于该推力。

第一次宽松起飞门槛下未能爬升，随后横滚和横向漂移快速增大。
该轮 PX4 ULog 在 20–30 s 请求的竖直推力持续达到上限，
2 号电机输出达到 1.0，分配器报告无法满足推力请求。
起飞前实测角曾有约 0.08 rad 的最大折叠误差；实际解锁时已降到约
0.01 rad，因此**不能仅凭此轮证明关节未收缩是唯一根因**。
该轮已按确切进程号关闭，不将其当作 PID 调参依据。

收紧“实测关节到位并静止”门槛后，三轮独立起飞与 1.5 m 定高均通过。
每轮悬停 20 s；下表高度/倾角统计取稳定判定后的最后 15 s，排除前 5 s 过渡：

| 案例 | 后 15 s 平均高度绝对误差 | 后 15 s 最大高度误差 | 后 15 s 最大倾角 |
| --- | ---: | ---: | ---: |
| folded_hover_foldguard_20260930_01 | 2.10 cm | 6.51 cm | 2.30° |
| folded_hover_foldguard_20260930_02 | 0.95 cm | 3.03 cm | 2.05° |
| folded_hover_foldguard_20260930_03 | 2.47 cm | 4.17 cm | 2.39° |

三轮起飞前/悬停时最大折叠角误差约 0.011 rad。
首轮成功案例的悬停阶段，四电机平均输出约 0.78–0.84，
无电机达到 0.95，推力分配可满足请求；悬停推力估计收敛到约 0.80。
因此当前空载定高不支持直接改 PID，但推力余量仍应限制后续载物和倾斜飞行。
这些数据只证明空载折叠臂起飞/定高，**没有验证规划到点、目标切换、
携物飞行、返航或自动抓取时机**。

下一步单独运行 SUPER 目标到达与 `FINAL_HOVER` 切换，
同时记录 SUPER 输出、MAVROS 转发设定点、实际高度、悬停推力和电机饱和。
目前 `super_navigation.launch` 设 `goal_tolerance=0.50 m`；
飞行管理器在轨迹超时且速度满足门槛时，直接将 hold 目标切到请求的目标坐标。
这可能允许接近目标但仍有高度误差时发生目标阶跃，需要用过渡试验证实，
不能把该现象预先归因于 PID。

## 十二、2026-09-30 SUPER 轨迹结束与定点悬停高度验证

本轮使用用户指定的 `/home/pcz/super_ws/src/super_px4_bridge/launch/super_navigation.launch`
启动真实传感器适配器、SUPER、MAVROS 转发器和飞行管理器。
没有伪造规划指令；仍为空载、折叠臂、自由飞行的生产 `yhang550` 模型。
没有修改或提交 ROS 工作区源码、正式 launch/YAML、PID 或生产 SDF。

### 验证脚本改动

扩展第十一节的 `run_empty_arm_hover.py`，不另增说明文档：

- `--super-goal DX DY Z`：稳定悬停后向 `/planning/user_goal` 发送真实目标；
  XY 为相对当前本地位置的偏移，Z 为本地绝对高度，默认取 ENU 方向。
- `--goal-body-frame`：将 XY 偏移按实测偏航旋转后再发送，减少初始大幅转向。
- `--super-hold-yaw`：仅在案例目录保存 launch/YAML 实验副本，开启
  `fsm.click_yaw_en` 与 `super_planner.goal_yaw_en`，目标朝向取发送时实测朝向。
  **这不是正式配置修改，也不表示全轨迹偏航已具有硬限速。**
- `setpoints.csv` 逐条保存规划器 `/planning/pos_cmd`、内部转发
  `/super/setpoint_raw/local`、最终 `/mavros/setpoint_raw/local` 的位置、
  速度、加速度、偏航、mask 和仿真时间；`events.jsonl` 保存状态切换与目标发送。
  原有遥测、日志、ULog 和摘要仍保存在各案例目录。
- 对比最后一个有效速度的跟踪指令与第一个忽略速度的终点悬停指令；
  高度差 <=0.01 m 才通过连续性检查。飞行完成但存在阶跃的案例仍记为失败，
  不把“最终能稳住”误当作“切换没有问题”。
- 新增 SUPER 阶段本地高度低于 0.5 m 自动中止；保留原有倾角、横向漂移、
  折叠关节误差和时钟停滞检查。清理只针对本轮启动的进程组。

### 已定位的两类问题

1. **规划目标与悬停目标不是同一坐标。**
   `px4_sim.yaml` 固定 `fsm.click_height=1.5`；
   `super_planner/src/super_core/fsm.cpp:184` 先覆盖请求 Z，再寻找非占据体素。
   本轮 0.3 m 地图将该目标移动到体素中心，实际规划 Z 是 **1.65 m**。
   管理器 `super_flight_manager.cpp:486` 结束时却将 hold XYZ 设回原始请求，
   两者没有终点协调或平滑过渡。请求 1.50/1.80 m 分别产生 -0.15/+0.15 m 阶跃。
   只关闭固定 click_height 仍不足以消除体素终点偏移。
2. **正式配置的偏航末端变化与推力饱和相关。**
   未启用目标偏航约束的两轮均未进入正常 `FINAL_HOVER`：
   一轮下降触地使关节误差超限，另一轮终止于 `FAILSAFE_HOVER`。
   第二轮相邻规划指令的绕角差/时间差最大约 39.27 rad/s（这是指令变化率，
   不是实测机体角速度），电机输出达到 1.0，目标发送后前 4 s 的推力请求
   可满足比例仅约 45%。`yaw_traj_opt.cpp:186` 的过速分支只打印警告，
   `return false` 被注释，配置 `yaw_dot_max=1.0` 不是硬上限。
   仅在实验副本开启目标偏航约束后，第三轮最大指令变化率约 0.40 rad/s，
   同一前 4 s 窗口电机最大输出约 0.927，推力分配全部满足请求。
   这支持先处理指令与推力余量，不能据此宣称所有飞行失稳只有一个根因。

### 实际飞行结果

案例目录统一为 `build/empty_arm_hover/<案例>/`。前三个成功走到终点的对照
均启用**实验副本**目标偏航约束、沿发送时机身朝向偏移 0.3 m；其余配置不变。
高度与倾角统计取 `FINAL_HOVER` 后 20 s 中的最后 15 s。

| 案例后缀 | 请求 Z | 规划结束 → 悬停 Z | 后 15 s 平均/最大高度误差 | 后 15 s 最大倾角 | 结果 |
| --- | ---: | --- | --- | ---: | --- |
| `goal_yaw15_20260930_03` | 1.50 m | 1.65 → 1.50 m | 2.34 / 4.50 cm | 2.36° | 飞行完成，连续性失败 |
| `goal_yaw18_20260930_04` | 1.80 m | 1.65 → 1.80 m | 0.80 / 2.00 cm | 1.64° | 飞行完成，连续性失败 |
| `goal_yaw165_20260930_05` | 1.65 m | 1.65 → 1.65 m | 0.84 / 1.98 cm | 1.94° | 高度切换与悬停通过 |

完整前缀为 `super_handover_`。原始配置失败案例：
`super_handover_height15_20260930_01`（ENU X 偏移 0.7 m），
`super_handover_forward15_20260930_02`（机身前方偏移 0.3 m）。
第三轮切换后前 5 s 实测本地高度范围约 1.447–1.611 m。
第五轮关节最大折叠误差约 0.0113 rad，未出现 NaN。
全部高度误差以 MAVROS 本地高度为基准；Gazebo 世界高度另行记录，
两者存在本地原点偏移，不直接相减判断飞控误差。

### 复现与下一步边界

关闭已有仿真后，在 Ubuntu20 中运行，例如重现高度连续正例：

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
cd /home/pcz/PX4-RoboMaster
python3 src/modules/arm_control/tests/flight/run_empty_arm_hover.py \
  super_handover_review_01 --super-goal 0.3 0 1.65 \
  --goal-body-frame --super-hold-yaw
```

案例名必须新建，脚本拒绝覆盖结果，也不会停止已有用户会话。
去掉 `--super-hold-yaw` 才使用正式 launch 原配置，不应直接把对照正例视为
正式配置已修好。1.65 m 只是本轮固定高度/地图下的诊断对齐点，**不是通用修复值**。

结论：本轮不调 PID。下一步应协调“请求目标—规划实际终点—终点悬停目标”，
明确三维目标是否允许被覆盖/移动；悬停接管应先保持有效终点，若仍需回到请求点，
用有界过渡并做安全检查，不直接阶跃。与此同时补足偏航指令连续性与硬限速，
再复测不同高度、不同方向和连续目标切换。当前通过的是一轮短程空载高度切换，
不是厘米级三维到位、长时间重复性、避障、携物或飞行抓取的验收。

## 十三、2026-09-30 实际终点协调、连续接管与偏航限速

本节修复第十二节定位的问题，使用正式 `super_navigation.launch` 复测；
未使用诊断配置副本，未改 PID、质量/惯性、推力模型或 SDF 碰撞。

### 修改位置与行为

- ROS `super_px4_bridge/include/super_px4_bridge/command_transition.h`：
  纯数学的平滑跟踪接管、五次终点停止轨迹、终点/轨迹时间判据、偏航限速器。
  配套 `tests/test_command_transition.cpp` 检查起终点位置/速度/加速度、
  过大目标投影、运动中断流、NaN、未到轨迹结束时间、跨 ±π 与暂停恢复，
  并检查 1500 个偏航速度/内部加速度限幅步骤。`CMakeLists.txt` 接入测试。
- ROS `super_px4_bridge/src/super_flight_manager.cpp`：
  `WAIT_TRAJECTORY → TRACK_SUPER` 从此前实际发布的悬停指令起始，
  1 s 连续混合到实时规划指令。持续输出仍为位置+速度，禁用加速度前馈。
  最终 MAVROS 输出统一使用偏航限制 0.35 rad/s、内部加速度限制 0.50 rad/s²；
  不直接透传规划器快速偏航阶跃。
- 正常结束新增 `SETTLE_HOVER`：根据最后有效规划位置冻结实际终点，
  经过 1 s 五次连续停止过渡；实际位置距离终点 <=0.20 m 且平移速度
  <=0.25 m/s 持续 1 s，才进入 `FINAL_HOVER`。
  最后指令必须已到达 `/planning_cmd/poly_traj` 提交的轨迹结束时间，
  且终端速度 <=0.05 m/s、终点到原请求偏移 <=0.35 m、当前到终点 <=0.50 m。
  这避免靠“速度小、离目标不远”把初始静止或中途断流误认为到点。
  接管静止判定总超时为 6 s；不满足终止条件或未能稳定均进入 `FAILSAFE_HOVER`。
- `/planning/resolved_goal`：latched 的 world `PoseStamped`，在稳定后发布
  接受的终点，让后续任务区分请求目标与地图投影目标。
  消费者必须按目标时间戳忽略上一目标 latched 消息；姿态字段是指令朝向，
  当前成功判据未包含实测偏航精确到位。
- ROS `super_mavros_bridge.cpp` 保留原始规划时间戳；管理器拒绝非有限、
  错误坐标/mask、过期/逆序/旧目标指令，不再给旧包重打新时间戳。
- 正式飞行配置移到 ROS `super_px4_bridge/config/px4_sim.yaml`，
  launch 使用包内 `config_path`。设置 `click_height=-10` 保留三维请求高度，
  同时启用 click/goal yaw 约束。外部 SUPER 原本的本地 YAML 已恢复到本轮前状态，
  不依赖提交该未跟踪文件；用户原有 SUPER CMake 修改保持不变。
- 现有 PX4 `run_empty_arm_hover.py` 增加解析终点记录、三维切换差值、
  连续目标、偏航目标偏移、最终输出限速判定和中途断流注入。
  通过 ROS 查找实际包路径；`PX4_DIR` / `ROS_WS` 指定工作区。
  每轮记录 launch/YAML 快照。测试副本也纳入新的 ROS 仓库。

正常任务时实际保持解析终点，**不把原始请求坐标强制改写到飞控目标**。
异常断流的当前实测位置悬停是安全退出，不承诺该故障路径位置指令平滑。
停止输出的判断仍基于 ROS 接收/源时间戳与已提交轨迹边界；未增加 MAVLink 协议字段。

### 复测结果

完整轨迹结束判据加入后，正式包内配置下两类正常任务和断流任务均通过：

| 案例 | 请求/解析高度 | 正常接管 XYZ 跳变 | 高度统计 | 结果 |
| --- | --- | ---: | --- | --- |
| `super_handover_guard_sequence_20260930_08` 第一目标 | 1.80 / 1.95 m | 0 m | 稳定悬停后第 5–10 s 平均/最大误差 2.36/3.02 cm | 通过，随后发送第二目标 |
| 同案例第二目标 | 1.20 / 1.35 m | 0 m | 终点悬停 20 s，后 15 s 平均/最大误差 1.35/2.57 cm | 通过 |
| `super_handover_fixed_lateral_20260930_10` | 1.50 / 1.65 m；ENU X 偏移 0.7 m | 0 m | 终点悬停 20 s，后 15 s 平均/最大误差 1.08/2.12 cm | 通过，原先失稳路径未再触地 |
| `super_handover_stream_loss_20260930_09` | 中途停止本轮转发节点 | 不适用（故障退出） | 故障悬停后约 5 s 速度 0.058 m/s、高度到 hold 误差约 0.95 mm | 进入 FAILSAFE，不误报 FINAL |

第 08 轮两次跟踪入口的位置指令差也均为 0，初始速度指令为 0；
整轮最终偏航指令变化率最大约 0.350002 rad/s（Float32 量化误差，
测试允许 0.001 rad/s 数值容差），不是实测机体角速度限幅结论。
该轮最后 15 s 最大倾角约 0.86°、最大折叠误差约 0.0114 rad。
第 10 轮最后 15 s 最大倾角约 2.53°，未出现 NaN。
两项 CTest（SO101 命令看门狗、SUPER 指令过渡）均通过；
发布副本的独立 C++ 编译与上述回归也通过。
第 06/07 轮是加入完整轨迹时间保护前的中间版本验证，也保留原始日志，
不替代第 08–10 轮的最终验证。

带偏航变化的升高窗口仍有短时单电机输出达到 1.0，约 91% 推力请求可完全满足。
因此本轮解决的是接管/指令一致性，不宣称推力余量或载物能力已解决。
所有高度误差相对接受的解析终点；**没有把 15 cm 地图投影偏移消除为精确到点**。

### ROS 仓库与回退边界

用户创建 `git@github.com:piaochenghzang/so101-px4-ros.git`，指定 `main`。
2026-09-30 已按两段提交推送到该仓库：

- `94344c8`：SO101 ROS 控制插件、看门狗/状态反馈及手动桥接基线。
- `3227f21`：SUPER 终点悬停协调、平滑接管与偏航限速，含飞行验证脚本、依赖锁定和 README。

远端 `main` 已核对为 `3227f217c1532aa5427c2095df0b815dd5ab2a36`，
发布仓库工作区干净。WSL SSH 主机校验未配置，本次通过已有 Windows Git
认证使用同仓库 HTTPS 临时推送地址；持久 `origin` 仍保留用户提供的 SSH 地址。
没有重新初始化 PX4 主仓库，也没有提交/推送其本轮说明与测试脚本改动。

三个 ROS 包与验证脚本整理到 `/home/pcz/so101-px4-ros` 发布副本，
保留 `/home/pcz/super_ws` 原包位置、原 `so101_gazebo` 未提交索引和 SUPER 仓库。
本次需回退的原文件备份在 PX4 `build/super_handover_backup_20260930/`，
不会上传备份、build/devel、ULog、bag 或实验结果。
新 ROS 仓库只有一个 README，记录安装/复现依赖和边界；详细飞行分析仍集中于本文件。
SUPER 依赖固定为 `2ad3419c127a617c6d7df6925e81a14175a9c096`，
无需把整个 SUPER 复制为另一套代码。

下一阶段应先确认粗导航到解析终点后的坐标转换、机体/臂/物体相对位姿和
抓取精确接近判据，再接自动抓取流程。当前 0.3 m 地图及 0.20 m 飞行位置判据
不满足直接抓取对齐；也未验收障碍物环境中的接管混合轨迹安全性、长时间重复性、
携物飞行或真实偏航到位。现阶段没有证据要求直接修改 PID。

## 十四、2026-10-01 飞行抓取分阶段细化方案（待校核，尚未实现）

本轮按用户要求先细化方案：只核对源码、展开 SDF、运行现有运动学/到位回归，
不启动仿真、不发关节/飞行命令、不修改 PID、SDF 或抓取状态机。
下一步先建设只读坐标与相对位姿核对，再逐步加入动作；不把设计门槛记为已验收。

### 已确认的约束

- 正式 `yhang550.sdf` 的 SO101 include 安装姿态为 `0 0 -0.05 3.1415926 0 0`。
  安装变换最终以展开 SDF 和实际 base link 位姿交叉校核；不将子模型原文件的
  model pose、include pose、机身局部原点或估计器原点简单叠加。
- PX4 位置 IK 控制 `gripper_frame_link`，不是两指夹持中心，也不保证任意末端姿态。
  当前 20 mm 方块的已验证参考点在 `gripper_link`：
  `(0.0025, -0.000218, -0.088)` m；它是本碰撞代理的抓取参考，不是通用 TCP。
  方块尺寸、指垫几何、开合角或抓取方向变化后必须重新核对。
- `super_sensor_adapter.cpp` 将 MAVROS odometry 的 frame 名改为 `world`，
  但没有做 Gazebo world 到 MAVROS local 的原点标定。因此相同 frame 字符串
  不能证明两个坐标数值可混用；雷达外参及点云/odom 时间对齐也纳入只读核对。
- 本机 MAVROS 为 1.20.1。其 odometry 位置/姿态使用 ENU / ROS base_link 约定，
  twist 使用机体坐标；raw local setpoint 插件在发 MAVLink 前做 ENU→NED 变换。
  不因消息枚举名 `FRAME_LOCAL_NED` 在 ROS 输入端再次手工换轴。
  依据：[local_position 源码](https://github.com/mavlink/mavros/blob/1.20.1/mavros/src/plugins/local_position.cpp)、
  [setpoint_raw 源码](https://github.com/mavlink/mavros/blob/1.20.1/mavros/src/plugins/setpoint_raw.cpp)。
- 现有反馈采集器的 bench 有效性明确要求 `bench_anchor`、固定基座位置和近零机身速度。
  原 profile 不能通过删除 anchor 或绕过 bench_valid 泛化成飞行 profile。
- 本轮 102 组配置 FK 对照最大位置差 3.533 μm，旋转矩阵差 0.000025006；
  原 ArrivalCheck 边界回归通过。这只是现有定义一致，不是飞行对齐通过。

### 1. 坐标合同与目标含义

固定符号：G=Gazebo 真值世界；L=MAVROS 局部 ENU（目前 SUPER 称 world）；
B=无人机机体 FLU；A=SO101 base_link；H=gripper_link；E=gripper_frame_link；
O=物体坐标系。`T_X_Y` 将 Y 坐标中的点转换到 X。

统一计算链：

`T_L_H = T_L_B * T_B_A * T_A_H(q_measured)`

`p_A_object = inverse(T_L_B * T_B_A) * p_L_object`

`p_H_object = inverse(T_L_H) * p_L_object`

全程使用实测机身四元数和关节反馈，不能只用偏航、指令关节角或固定倒挂换号。
真值物体输入必须显式经 `T_L_G` 转换；该变换用时间对齐的机体真值/估计位姿
检查原点与朝向，不直接假定 identity。重置、估计器原点变化后标定作废。
对象输入保留 target_id、来源、frame_id、源时间戳和 world_epoch；无变换则拒绝。

IK 目标若从抓取参考点构造，必须同时明确一个可实现的 H 姿态：
`p_A_E_des = p_A_object + R_A_H_des * (p_H_E - reference_H)`。
仅有物体 XYZ 不足以唯一确定五关节抓取姿态；第一轮限定已验证的方块、
接近侧和姿态族，不开放任意六自由度抓取。实测姿态不符合夹持几何则不闭爪。

粗导航目标是机体位置，不是物体中心。例如预抓取姿态 q_pre 下：
`p_L_B_des = p_L_object - R_L_B_des * (p_B_A + R_B_A * p_A_reference(q_pre))`。
再加入经过净空校核的预接近偏移；解析导航终点只证明到达粗接近区，
不能替代物体位置、精确机体目标或夹持参考点。地图投影后必须重算可达性。

### 2. 分阶段执行与退出门槛

| 阶段 | 本阶段做什么 | 通过后才进入下一步 |
| --- | --- | --- |
| A：只读坐标核对 | 采集机体/基座/夹爪/物体实测位姿，计算完整变换；输出 FK 与实际 link 残差、局部/真值原点差、参考点误差与时间差，不发布运动指令 | 不同位置、偏航及小倾角下变换闭环正确；时间、身份、重置负例被拒绝；不靠指令角证明正确 |
| B：空载精接近 | 夹爪保持打开，机身定点；固定可达方块仅作观察目标，先验证预抓取与小步接近，不闭爪、不取物 | 请求/实际 IK 接受目标一致，姿态/净空符合模板，实际参考点对齐持续满足；取消或失联不继续接近 |
| C：新飞行 profile | 保留旧 bench 回归；增加与固定世界位置无关的物体身份、接触、支持面与实际位姿能力校验 | 真正解除固定机身后，反馈健康和拒绝案例通过；没有删除旧安全检查冒充新能力 |
| D：悬停抓取与回收 | 一次固定目标：闭爪→双侧夹持→离台→小幅抬高→回收→保持；暂不同时飞行换点 | 双侧接触、两侧力、离台和持续保持均有实际证据；无自动空中松爪，失败原因明确 |
| E：已夹持低速携物 | 先短距离平移、轻微升降与停止，再返航；逐项验证推力裕度和不掉落 | 无持续饱和/高度失守、目标丢失或异常分离，取消/链路故障能安全锁存 |
| F：完整 demo | 固定物体输入→粗导航→精接近→抓取→回收→返航 | 分阶段事件、目标身份和故障日志完整；再替换为点云目标估计 |

每阶段先记录正例，再记录负例；控制路径优先分工为“机身保持、机械臂微调”，
不同时让两个位置环追同一个夹持误差。若机械臂余量不足，先退回预接近位，
收臂后单独重定位机体，再重新评估；不能在物体接触时静默重规划机体。
现有每条 IK 请求最多 20 mm 的限制保留；近物体小步先沿用 bench 已验证的
不超过 3 mm / 0.5 s 节奏，核对实际 accepted_target，不把裁剪当接受。

### 3. 接近、闭爪与抓取成功分开判定

以下是第一轮候选门槛，需用阶段 A/B 的采样、噪声和漂移结果校核，
不是放宽既有严格到位，也不是当前已通过的飞行验收：

- 输入健康：关节反馈年龄 <0.2 s；机身/物体位姿年龄拟 <0.1 s；
  参与变换的源时间差拟 <=20 ms，按共同时间插值或拒绝，不用接收时间伪造同步。
  非有限、无效四元数、旧目标、旧 epoch、重复/逆序帧均不得推进连续计时。
- 机身稳定：平移速度拟 <=0.05 m/s、角速度 <=0.10 rad/s、
  滚转/俯仰绝对值 <=3°、实测偏航到设定朝向差 <=3°，连续 1 s。
  这些条件只是稳定门槛；不能因偏航误差 3° 就认为毫米级夹持已经对齐。
- 机械臂到位：原严格 ArrivalCheck 保留，末端 <=5 mm、前五关节差 <=0.03 rad、
  速度 <=0.05 rad/s，连续 0.5 s，夹爪另判。
- 可闭爪：实测 `p_H_object-reference_H` 范数拟 <=3 mm，持续至少 0.5 s，
  期间每个有效样本满足门槛；姿态属于已验证夹持模板、物体落在开口有效区域，
  接近路径与机体/支架/非夹指净空合格。候选姿态容差需按几何测定，不凭空指定。
  普通 0.20 m FINAL_HOVER 或 IK 解误差小均不能给出 CLOSE 许可。
- 夹住：沿用同目标双侧接触窗口 >=90%、两侧法向力各 >0.01 N、
  夹爪速度 <0.02 rad/s 及反馈健康；实际闭合角未到目标不代表失败，空爪角到位不代表成功。
- 抓起并保持：实际离台且抬高 >=25 mm；持续双侧接触、无支持接触/明显分离，
  保留小滑移为诊断，不为了漂移指标牺牲“能抓起、不掉落”的 demo 目标。

计时使用 sim-time；新目标、坏样本、失联或时钟重置立即清除到位积累。
超时不得自动闭爪；无接触时退出接近并保持安全位置，有载荷时保持有界夹爪目标，
接触反馈失联记 UNKNOWN，不猜测为空载，不自动 RESET/松爪/重新抓取。
墙钟无进展保护只报告 STALLED，不能伪造物理失败或成功。

### 4. 下一小步的文件与验证边界

优先在 ROS bridge 包中增加可单测的坐标/判据函数和只读观察入口，
配套一个可定位实际 ROS 包的实验脚本；安装外参、参考点和候选阈值集中配置。
先用显式标记的 Gazebo 真值作为对象来源，后续再替换点云估计。
只读入口不能发 `/mavros/setpoint_raw/local`、关节命令或抓取 START。
这一步不新增 MAVLink 消息，不改 arm_control 行为，不绕过 bench_valid。
涉及状态机或飞行权限的新接口到阶段 C 再单独设计/审阅。

必测：变换正反往返、不同偏航/倾角、倒挂安装、非零局部原点、FK 实测残差；
错轴/错 frame、缺 link、NaN、旧目标 latched 消息、不同源时刻与世界重置。
暂拟同步真值/FK 残差 <=2 mm 为诊断门槛；超差先区分模型定义、时间错位和
状态估计误差，不扩大对齐容差掩盖问题。纯数学容差另设，不拿毫米门槛检验数学函数。
之后才运行无闭合的固定目标悬停接近，记录精接近正例及“不准闭爪”的负例。

仍保持单一集中说明；ROS 发布副本与实际 `/home/pcz/super_ws` 包分开，
实现时需核对同步差异。本轮没有新的 Git 提交/推送；上次发布的 main 保持干净。

## 十五、2026-10-01 只读坐标观察入口与被动几何核对

按审核后的范围实现观察，不接管飞行、不发关节/闭爪/抓取 START，不修改 PID、
生产 SDF、现有 SO101 控制插件、MAVLink 或 arm_control/bench 抓取行为。
本阶段完成的是只读入口及被动几何核对；真实 MAVROS 配准、飞行姿态下的动态反馈
和任意非零关节姿态的 live 验证尚未验收，不能直接进入 CLOSE。

### 修改清单与只读边界

实际源码位于 `/home/pcz/super_ws/src/super_px4_bridge`，本轮同步到 ROS Git 发布副本：

- `msg/CoordinateTruth.msg`、`src/coordinate_truth_plugin.cpp`：新增 ROS 诊断快照和
  Gazebo WorldPlugin；在同一 `WorldUpdateEnd` 读取真实 link WorldPose 与关节角/速度，
  时间戳取 world SimTime，不将 ROS 接收时间当采样时间。只向 `/grasp_audit/truth` 发布。
  包含 source_instance、world_epoch、实体 ID；时间回退、WorldReset 或实体变化使 epoch 作废。
  static 模型的关节读数明确无效；不设置关节角、位姿、速度、力矩或 PID。
- `scripts/grasp_audit/geometry.py`：独立坐标数学、真实角度 FK、安装/工具/雷达链残差、
  物体在臂基座与夹爪坐标中的位置。全四元数姿态，不采用固定倒挂换号。
  源时间插值要求两侧样本包围，最大偏离 20 ms，不外推、不重打时间戳。
- `scripts/observe_grasp_frames.py`：只订阅诊断真值、MAVROS odom、ROS joint_states、
  导航请求/解析终点和 clock，写 JSONL；没有 Publisher 或 ServiceProxy。
  拒绝错误 frame、缺 link、坏四元数/NaN、重复/过期/未来包、旧目标及身份/时钟变化。
  预留 40 ms 接收窗口取得 odom 两侧源时间样本。
- `config/grasp_frame_audit.yaml`：集中安装变换、PX4 FK 常量、20 mm 方块参考点及诊断门槛。
  FK 常量与原 PX4/SDF 链一致性测试交叉核对，不放宽 2 mm 几何残差门槛。
- `scripts/run_readonly_frame_audit.py`：隔离被动场景，独占 11357/11457；发现原仿真则拒绝启动。
  在新的 build 目录生成正式模型展开副本，移除控制/传感插件、sensor 和 collision，
  world 重力为零，车辆保持 dynamic 以读取真实 ODE 关节；方块为 static 观察目标。
  不启动 PX4、MAVROS 或 SUPER，不调用 spawn/set/reset 等模型操作，不发送动作。
  只清理自己启动的 roscore、gzserver、观察器进程组，记录 ROS 图和库哈希。
- `tests/test_grasp_frames.py`、`tests/test_frame_observer.py`：11 项数学/拒绝边界测试与
  7 项无 master 的回调测试。构建接入新诊断消息、采样器、Python 安装和 CTest，
  现有飞行节点与看门狗回归保持通过。

`close_permitted` 在所有记录中始终为 false。
`aligned_geometry_only` 只表示几何距离符合诊断门槛，不包含姿态模板、净空、
机身稳定、严格到位或真实接触许可，不得供状态机直接启动闭爪。
无对象时输出 TARGET_MISSING；无 MAVROS 时输出 ODOM_NOT_BRACKETED / local_valid=false，
不假设 Gazebo world 与 ROS local 的原点相同。

### 局部坐标与旧关节话题的测量边界

有真实 MAVROS 数据时，对同一源时刻的机体真值/估计构造 G→L 配准候选，
连续 10 个一致样本后冻结一份实际观测变换；之后只记录残差，不每帧重新拟合以消除误差。
这是仿真诊断配准，不是独立标定或现实感知能力。身份变化/重置清空配准。
`local_valid` 表示可完成已配准转换；`local_precision_consistent` 另检查残差是否达到
毫米门槛，二者不等价。配置不一致、时间错位或输入失联不产生抓取许可。

旧 `/so101/joint_states` 在 UpdateBegin 用 ROS clock 打时间戳，未在本轮修改。
与物理步读数的对照明确标注 APPROXIMATE_ROS_CLOCK，记录时间差和角差；
主要 FK 验证使用同一物理步真实关节读数，不把旧话题最近一包当精确同步角度。
雷达项用实际雷达 link 加配置 sensor offset 校核名义测量原点，
不表示点云/odom 实际同步、扫描畸变或目标检测准确度已经验证。
物体坐标是测试方块的 model 原点（该方块为中心），不是任意物体自动估计的抓取点。

### 测试夹具发现与修正

初次采用整机 static 场景：零偏航似乎通过，一次偏航测试得到约 17.47 mm 超差。
追溯原始角度后发现 static Joint::Position 缓存中出现 `5e-310` 等非实际编码器数值。
[Gazebo Classic Joint::Position 源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/Joint.cc)
对 static 模型返回 staticPosition 缓存，而不是物理引擎读数；其初始化/使用路径
不能保证未调用设置操作的 static 夹具有可用角度。采样器因此拒绝 static 关节读数，
没有用 SetPosition 初始化来让它通过，也没有把这种测试夹具问题套回历史夹爪 NaN 原因。
早期 `readonly_*` 场景不计入最终 FK 验收；保留日志供追溯。

最终使用 dynamic、零重力、无接触和无控制插件的被动场景，既无动作指令，
又保留真实 ODE 关节测量。以下每轮观察约 4 秒 sim-time：

| 案例，位于 PX4 build/frame_audit | 有效样本 | 验证结果 |
| --- | ---: | --- |
| `passive_baseline_20261001_04` | 195 | 零偏航坐标链一致 |
| `passive_yawed_20261001_04` | 194 | 非零位置与偏航 1.2 rad 下坐标链一致 |
| `passive_tilted_20261001_04` | 190 | roll=0.08、pitch=-0.05、yaw=-0.7 rad 下坐标链一致 |
| `passive_missing_target_20261001_04` | 195 | 缺对象明确报缺失，无对齐/闭爪许可 |
| `passive_wrong_mount_20261001_04` | 195 | 仅诊断配置故意改错安装 roll；chain_valid=false、观察器返回 2，被测试器认定正确拒绝 |

上述各组真实 FK 最大位置差约 4.21 μm，姿态角差约 1.81e-5 rad；
正确安装/雷达链的位置残差在浮点数舍入范围。2 mm 门槛没有放宽。
偏航与倾角是不同初始场景，不是在运行中发送移动或关节指令。
零重力被动测试不证明重力负载、接触、折叠保持、飞行稳定或抓取。
最终增加不回退时间的 WorldReset 保护和被动位移检查后，
`passive_final_20261001_05` 倾角场景再次通过，195 个有效样本，
机体位置漂移和关节角漂移均为 0；FK 残差保持上述水平。
四项 CTest（看门狗、指令过渡、11 项坐标数学、7 项观察回调）全部通过；
Git 发布副本独立运行两组 Python 测试也通过，并与 live 包内容一致。
本轮自行启动的 roscore/gzserver/观察器均已退出，没有用户进程被结束。

纯数学测试覆盖 100 组非零关节角/完整姿态、变换往返、错轴/错安装、
缺 link、坏四元数、NaN、目标缺失、旧 latched 目标、时间插值、身份/时钟重置。
回调测试核对非零局部原点、错 frame、错时刻、过期/重复/未来真值、配准清除，
并断言观察器不创建 Publisher/ServiceProxy。非零局部原点属于合成回归，
没有发布伪造 MAVROS 数据冒充实测；真实 local_frame_tested=false。

### 复现与下一小步

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
cd /home/pcz/super_ws
catkin_make -j4 so101_coordinate_truth
cd build
ctest --output-on-failure -R 'grasp_frame|super_command_transition|so101_command_watchdog'
rosrun super_px4_bridge run_readonly_frame_audit.py --name my_frame_audit_01 --case tilted --duration 4
```

案例名称需未使用；结果在 `/home/pcz/PX4-RoboMaster/build/frame_audit/`，
`PX4_DIR` / `ROS_WS` 可指定仓库/工作区。脚本通过 rospkg 找到实际包。
原正常仿真中只读观察要求 world 内显式加载以下插件，不加载控制器或改变模型：

```xml
<plugin name="coordinate_truth" filename="libso101_coordinate_truth.so">
  <vehicleName>yhang550</vehicleName>
  <targetName>test_cube</targetName>
</plugin>
```

须有 gazebo_ros API、use_sim_time=true 和 devel/lib 插件路径。
本轮没有修改生产 world/start_super_sim.sh 来自动加载它。
world 中插件已加载后，观察器可独立运行：

```bash
rosrun super_px4_bridge observe_grasp_frames.py --output /tmp/my_grasp_frames_01.jsonl --duration 20
```

输出文件不能已存在。没有源或时钟无进展时有界结束为 STALLED/WALL_TIMEOUT，
不是把卡住当物理 NaN。真实方块未生成时不替用户 spawn，不编造 object_pose。
下一小步仍是实际仿真中的只读 MAVROS/真值配准与不同实际关节姿态核对；
需在可用原场景下采样，不自动发送动作来制造正例。通过后再设计空载精接近。
上一轮结束时新增源码保留为可审阅的未提交改动。

### 本轮按授权归档提交（2026-10-01）

ROS 发布仓库 `/home/pcz/so101-px4-ros` 已将上述只读采样器、观察器、
被动验证和回归测试合为一个可独立构建的提交：
`05f3c2c5c5d4681ff66faa5381b8359714f02bcc`
（`feat: add read-only grasp coordinate audit and passive validation`）。
提交前 live 包与发布副本一致，diff --check 通过，相关 CTest 4/4 通过。
本集中说明同时归档此前阶段 11–15 的说明更新，不包含新的飞行控制修改。
PX4 中此前未跟踪的 `src/modules/arm_control/tests/flight/` 保留，
不混入本次只读功能提交。此次仅本地提交，未推送。
后续正常仿真只启动未解锁 PX4/Gazebo/MAVROS，保留原有初始关节保持，
不启动自动起飞/导航管理器，不发送运动或闭爪命令；实际核对结果另行记录。

## 阶段 16：正常物理仿真中的只读局部坐标核对（2026-10-01）

### 边界与场景

用户明确授权由代理启动未解锁 PX4/Gazebo/MAVROS，保留原有初始关节保持。
仅使用 Ubuntu-20.04，未访问 Ubuntu22。没有启动 flight_manager/SUPER，
没有发布飞行设定点、机械臂目标或闭爪指令，没有解锁、抓取或注入传感器数据。
PX4 既有 airframe post 启动 arm_control，原有折叠目标和夹爪初始保持仍正常运行；
这不代表整个场景无控制器，仅代表观察任务没有新增动作命令。

生产 empty.world 的场景副本保留重力、地面、碰撞、传感器与全部原控制插件，
通过 include 加载生产 yhang550，使用 sitl_run.sh 原有出生位置
`1.01 0.98 0.83 0 0 0`，只新增 CoordinateTruth world 插件。
本轮未修改生产 SDF、PID、PX4/ROS 数据流频率或启动脚本；未开 GUI。
生产场景没有 test_cube，本轮没有另行生成，全部记录 target_present=false。
因此物体相对位姿、解析规划终点和抓取对齐仍未实测。

证据统一保存在 `build/frame_audit/normal_unarmed_20261001_01/`：
`normal.world`、`manifest.json`、`raw_sources.jsonl`、两段 `frames_*.jsonl`、
前后 ROS graph、PX4 ULog、进程日志、`coordinate_result.json`、
`estimator_result.json` 与两个离线 residual.csv。
`run.py`、`analyze.py`、`analyze_ulog.py` 是该 build 目录中的一次性实验工具，
不混入生产源码或前一轮提交。

MAVROS connected=true、armed=false；守卫没有观察到解锁，ROS graph 没有
机械臂 cmd_pos、飞行 setpoint_raw/local 或 planning user_goal/click_goal 发布者。
落地后从约 36–78 秒仿真时间观察两段各 20 秒窗口，耗时约 90 秒墙钟。
采样完成后只退出本轮自己启动的进程，没有结束用户的既有进程。

### 实测结果：坐标链通过，但局部精度未通过

| 项目 | 实测 | 判定 |
|---|---:|---|
| 同物理步真值/坐标链 | 第一段 977/977，第二段 961/961 有效 | 该折叠姿态下通过 |
| 末端实际关节 FK 与实际 link TCP 位置最大差 | 0.07485 mm | 小于诊断门限 2 mm |
| FK 旋转最大差 | 0.000378 rad | 小于诊断门限 0.001 rad |
| 机体—机械臂安装位置最大差 | 0.03743 mm | 小于诊断门限 |
| 雷达名义传感器偏移位置最大差 | 0.001218 mm | 仅 link 加配置偏移核对 |
| 实际机体平移各轴范围 | 0.1948 / 0.0345 / 0.00304 mm | 实际机体近似静止 |
| 在线 local_valid | 两段均为 0 | 配准没有完成，不能声称在线核对通过 |
| 离线冻结原点后机体位置残差 | 中位 0.2581 m，最大 0.3182 m | 不满足抓取精定位 |
| 离线冻结原点后机体姿态残差 | 最大 0.15675 rad（约 8.98°） | 不满足精确姿态核对 |
| 离线局部估计链 TCP 位置残差 | 中位 0.2597 m，最大 0.3120 m | 不满足抓取精定位 |

在线观察器的 OBSERVATION_COMPLETE/返回 0 只说明采样完成且存在有效坐标链，
不是 local 配准或抓取验收成功。第一段 SOURCE_SKEW=693、
ORIGIN_CALIBRATING=281、ODOM_NOT_BRACKETED=3；第二段分别为 711/249/1。
MAVROS odom 典型源时间间隔 32 ms，真值约 20–21 ms。
20 ms 的双侧支持门限只接受部分真值时刻；每次 SOURCE_SKEW 又清空 origin，
导致十个连续校准样本无法积累。此前合成的更密集 odom 回归没有覆盖此正常流量情形。
这属于观察基础设施问题，本轮只读发现，没有修改门限或代码来将结果强行变成通过。

### 离线交叉核对与限制

为区分时间拒绝与真实估计误差，离线在 odom 源时间戳上使用双侧真实机体
位姿插值，仍保留原 20 ms 支持门限，不外推、不重标时间、不发布伪造 ROS 消息。
第一次十个合格配对（36.045–36.345 s）选中间实际样本冻结 G→L 变换；
之后不重新拟合来消除漂移。变换平移约
`[-0.999284, -0.967238, -0.346787] m`，初始偏航约 `0.006168 rad`。
这是吸收当时估计偏差的诊断配准，不是已标定的导航 TF，不用于抓取许可。

原始数据有一对不同 pose 共用 stamp=57.772 s；离线明确排除两条重复样本，
保留原值在结果文件中。在线第二段观察器在该事件之后启动，故不能把在线
配准失败都归因于这次重复时间戳。其余 1238 个机体配对、551 个 TCP 配对
用于冻结变换后的残差统计；不同方向插值与在线配准属于不同诊断，不冒充在线验收。

PX4 ULog 中 vehicle_local_position 在同窗口自身就存在位置变化；
xy/z/heading reset counter 没有变化，不能解释为本窗口内原点 reset。
将原始 NED 位置换成 ENU，与同源时间 MAVROS 核对 1247 个非重复样本：
位置差中位 0.323 mm、P95 1.034 mm、最大 1.666 mm，源时间支持最大 4 ms；
偏航用 `yaw_ENU = pi/2 - heading_NED` 核对，最大差 0.001045 rad。
这支持 MAVROS 的轴/位置转发基本一致，大量残差已经存在于 PX4 局部估计，
不是仅靠修正机械臂安装或把 world 标签改成 map 就能解决。

ULog 显示 GNSS 位置/速度、磁航向和气压高度融合有效，位置有效标志为 true；
eph 约 0.443–0.506 m、epv 约 0.276–0.302 m，不能将 valid=true 当成毫米精度。
cs_in_air=false、cs_vehicle_at_rest=false、filter_fault_flags=0。
GPS 模型开启噪声，但本轮没有做单因素试验，不能认定具体传感器噪声、接触振动
或某项 EKF 参数是唯一原因。也没有证据要求直接修改飞行 PID。

下一步建议仍先完善观察器的时间配对/连续配准规则，覆盖 32 ms odom、
21 ms truth、缺样/重复时间戳，并严格区分采样成功与精度验收成功。
真实源身份/时钟/估计原点 reset 应清除配准，但单个不满足配对支持的样本
不应通过反复重拟合掩盖后续残差。随后再只读复测局部漂移并核对固定方块，
明确粗导航用局部估计与精接近用相对观测的边界，暂不接闭爪/自动抓取。
本轮仅将结果补入本集中说明，保留为待审阅更新；未新增推送。

## 阶段 17：观察器时间配对修复与正常场景只读复测（2026-10-07）

### 修改范围及原因

本轮按审核意见实施观察基础设施修复，不改运动、抓取、PID、碰撞、传感器噪声
或 MAVLink 数据流参数。只访问 Ubuntu-20.04，未访问 Ubuntu22。
ROS live 包与 Git 发布副本一致，修改集中在：

- `scripts/grasp_audit/geometry.py`：提取严格源时间支持检查；拒绝重复/倒序、
  非法时间戳和无双侧支持。仍只在线性位置/最短弧 SLERP 上做有界插值，
  不外推、不改写原时间戳，双侧各自不得超过原 20 ms 门限。
- `scripts/observe_grasp_frames.py`：保留真实物理步的关节 FK/安装/雷达核对，
  新增有界等待未来 odom 端点；40 ms 后如果未来端点仍未收到，最多等待至原
  100 ms 真值新鲜度门限的 80%（80 ms），不会放宽支持或精度门限。
  SOURCE_SKEW、缺样、迟到、无效单条数据和过期/重复真值只拒绝样本，
  不清除候选/被冻结的原点；过期真值也不再推进身份时间防护。
  未冻结时仍检查原有 20 mm / 0.020 rad 候选散布，允许重新积累一致候选；
  已冻结后绝不按残差重拟合，所以后续真实漂移仍可见。
- odom 相同源时间戳拒绝两条记录并记住被拒绝的 stamp，迟到记录不插回缓存。
  若冲突 stamp 属于建立原点时使用的支持端点，则锁定
  CALIBRATION_SOURCE_CONFLICT，不自动重新拟合；须明确边界重置或新观察会话。
  时钟回退、采样实体/源世代变化、frame 错配以及已连通 FCU 断联会清除配准。
  `/mavros/state` 仅作为订阅输入，没有调用任何飞行服务。
- 无方块时仍输出机体与 TCP 局部链残差/精度结果，不生成目标、不假报抓取对齐。
  有方块时再增加物体相对位置核对；所有记录仍为 close_permitted=false。
- summary 分开给出 observation_complete、local_pairing_observed、
  local_registration_current、local_precision_verdict 和拒绝原因/冻结次数。
  只要已测到越限就报 FAIL，不因样本尚少而把失败藏为未评估。
  新增 `--require-local`：当前世代至少十个合格配对、配准未失效时才允许返回 0；
  返回 0 只表示观察/配对基础设施成功，不表示精度或抓取成功。
- `tests/test_grasp_frames.py`、`tests/test_frame_observer.py`：补回归。
  现有 ROS `README.md` 补入口契约和结果，不新增分散的说明文件。

仍没有接入能证明 EKF 原点重置的 reset-counter 元数据，summary 明确标为
estimator_reset_metadata_available=false。frame 标签或普通 Odometry 自身不能
证明估计世代连续；本变换只用于诊断，不能当导航 TF/闭爪许可。局部坐标跃变
不会被静默重配准消除，后续应单独完善估计状态/重置观察。

### 回归与实测

数学 12 项、回调 19 项全部通过；live 和发布副本均独立运行通过，CTest 4/4。
新增覆盖 32 ms odom / 21 ms truth、35 ms 接收延迟、连续缺两条消息、
单次时差拒绝前后保持冻结、真实漂移仍报 FAIL、重复支持锁定、迟到/未来数据、
FCU 断联、时钟/实体重置、新世代不足样本及无目标时的机体/TCP 精度报告。
测试在进程内构造消息，不发布到真实仿真；数学精度正例不是物理抓取正例。

正常场景复用阶段 16 的生产 empty.world/yhang550 include、原出生位置、
重力/碰撞/控制插件和 seed=123；独立 rootfs 启动未解锁 PX4/Gazebo/MAVROS，
不启动 SUPER/flight_manager，只保留既有折叠关节保持。没有 GUI、方块或新命令。
一次观察器连续运行 40 s 仿真时间，不在中途重启观察器以重拟合原点。
完整产物在 `build/frame_audit/normal_unarmed_20261007_01/`，包含场景/manifest、
原始来源数据、JSONL、ROS graph、ULog 和 `coordinate_result.json`；
manifest 保存实际观察器/几何模块及真值库 SHA256。本轮自己的进程采样后均退出。

| 检查项 | 实测 | 判定 |
|---|---:|---|
| 有效原生真值/关节坐标链 | 1941/1941 | 该折叠姿态下通过 |
| 合格局部配对 | 467/1941（24.06%） | 由原 20 ms 双侧支持严格筛选 |
| 原点首次冻结 | sim 41.361 s | 只冻结一次，全部有效样本矩阵相同 |
| 原点世代重置 | 0 次 | 本次会话连续 |
| 配对支持、新鲜度、无命令/解锁等基础设施检查 | 全部满足 | infrastructure_pass=true |
| 原生 TCP FK 最大位置差 | 0.07468 mm | 小于原 2 mm 门限 |
| 原生 FK 最大旋转差 | 0.0003772 rad | 小于原 0.001 rad 门限 |
| 实际机体平移各轴范围 | 0.1855 / 0.03362 / 0.002667 mm | 近似静止 |
| 局部机体位置残差中位/最大 | 0.1731 / 0.3299 m | 精度不通过 |
| 局部 TCP 位置残差中位/最大 | 0.1578 / 0.3218 m | 精度不通过 |
| 局部链最大旋转残差 | 0.24793 rad（约 14.21°） | 精度不通过 |
| 局部精度达标的配对 | 0/467 | local_precision_verdict=FAIL |

真值窗口约 36.544–76.409 s，启动/采样/清理墙钟约 94.3 s。
拒绝原因 SOURCE_SKEW=1409、ORIGIN_CALIBRATING=65；开始阶段候选散布
检查重新积累候选，不等于已冻结后重置。回调另拒绝未来真值、重复 odom 和
过期/未来 odom 各 1 条；未因此清空被冻结的配准或改写源时间。
MAVROS odom 中位间隔仍为 32 ms，truth 为 21 ms。低配对覆盖率是严格门限
下真实记录的限制，不冒充高频精接近控制能力，也没有把被拒绝帧算作精度通过。

结论：本阶段的时间配对与漂移可观测性修复已通过回归及正常场景验证；
真实局部估计精度没有因此改善，不能接飞行抓取。此次未重复解析 ULog 来
认定新的具体传感器根因，仍沿用阶段 16 已验证的 PX4/MAVROS 转发交叉证据。
没有物体相对位姿、飞行中接近、闭爪或携物验证，也没有飞行 PID 修改。
下一步仍宜只读核对传感器/估计状态、原点重置元数据与目标相对观测边界，
将粗导航坐标与精接近坐标分开验收后再推进抓取。
阶段 17 结束时修改保留未提交供审阅。2026-10-07 按用户审核后的授权，
ROS 本地仓库已提交 `78c9628de0abd61402081edf2016b7b184c9b17e`
（`fix: preserve frozen frame registration across rejected time pairs`），
提交前 CTest 再次 4/4 通过。本说明同时归档阶段 16–17 的实测与修改记录，
与 ROS 修复分开提交；未混入此前未跟踪的 flight 测试副本，未推送。
后续只读传感器/EKF 检查另行记录，不修改上述已验收的观察器或控制参数。

## 阶段 18：传感器/EKF 状态与原点重置只读核对（2026-10-07）

上一部分已按授权分段本地提交：ROS `78c9628de0abd61402081edf2016b7b184c9b17e`，
PX4 集中说明 `9653ccd31fa3ab1d318592c8b92230731a559cd1`；均未推送。
此阶段只读分析阶段 17 已保存的完整 ULog、原始 MAVROS 数据和当前仓库源码，
没有必要重新启动仿真，没有参数写入、运动/闭爪命令或生产源码改动。
只访问 Ubuntu-20.04。新增分析工具与结果位于
`build/frame_audit/ekf_readonly_20261007_01/analyze.py` / `result.json`，
保存源 ULog SHA256、字段清单、重置时间线、传感器/融合统计和所读源码指纹。

### 1. 原点重置与连续漂移已分开

稳定窗口 36.544–76.409 s 的 4984 条 vehicle_local_position 中：
xy/z/vxy/vz/heading reset counter 分别一直为 `3/2/2/1/1`；
ref_timestamp=`1185000 us`、参考经纬度和高度均未变化。
主 EKF 为 instance 0，selector 在窗口内未切换；实际日志记录的是
estimator_odometry（多实例 EKF 输出），不是 vehicle_odometry。
其 3987 条窗口数据 reset_counter 一直为 9，没有重置事件。
不能把没有记录 vehicle_odometry 误判为没有估计里程计输出。

日志开始后可见的启动事件：

| 仿真时刻 | 事件 | 主 EKF estimator_odometry 计数变化 |
|---|---|---|
| 1.337 s | 全球参考首次建立，XY/Z 计数改变 | 4 → 6 |
| 1.633 s | XY/水平速度计数改变 | 6 → 8 |
| 9.593 s | 航向计数改变 | 8 → 9 |

这些均发生在正式窗口之前。首次参考建立前 ref_lat/lon/alt 是未初始化值，
分析结果明确用 null 记录，不将其当作 Gazebo 求解器 NaN。
delta_z=-488.3616 m 是当次启动高度参考变换的残留记录；没有计数改变时，
不能仅凭单帧仍包含这一 delta 就断言刚发生了同样跳变。
结论：本次稳定窗口的位置/姿态误差是连续估计变化，不能用反复原点 reset 解释。

### 2. 找到 at_rest=false 的直接机制线索

事件型 sensor_selection 在 0.609 s 选中 accel/gyro device 1310988，窗口内无切换；
对应 vehicle_imu_status instance 0 的 device ID 相同，传感器速率字段均为 250 Hz。
land_detector 的窗口快照 landed=true、ground_contact=true、水平/垂直/旋转
movement=false，但 at_rest=false；EKF 的 cs_vehicle_at_rest 同样为 false。
这些状态来自不同判据，不是互相矛盾的同一个运动标志。

`LandDetector.cpp:246` 的静止判据检查陀螺振动指标 >0.02，或加速度振动指标 >1.2；
每次超限刷新最后运动时刻，超过 1 s 未再超限才能恢复静止。
`VehicleIMU.cpp:742` / `:751` 表明指标是相邻传感器向量差的模长再低通，
不是实际位移，也不能直接当成机械振动幅度。

| 指标 | 本窗口已记录快照 | 对比静止门限 |
|---|---:|---|
| gyro_vibration_metric 中位/最大 | 0.021747 / 0.023872 | 39/40 个快照 >0.02 |
| accel_vibration_metric 中位/最大 | 0.36550 / 0.42874 | 均 <1.2 |
| 加速度 clipping、传感器错误计数 | 0 | 未见饱和/错误证据 |

日志对此话题按 1000 ms 间隔记录，39/40 是快照比例，不是全部运行周期的比例。
仍足以与陀螺指标频繁触发静止抑制的代码机制相互支持，不能据此精确重建每个
高频触发时刻，也不能认定它是全部漂移的唯一根因。
sensor_combined 陀螺各轴标准差约 0.0165–0.0169 rad/s；加速度约
0.302/0.393/0.279 m/s²。当前 SDF 没有显式覆盖 IMU 噪声参数，源码默认 gyro
noise density=0.0008726646、accel=0.00637，随机游走/turn-on bias sigma 为 0。
默认噪声与接触/关节/imu_link 高频运动各自的贡献还没有做单因素隔离，
源码默认值也不冒充本轮直接读取的插件内部运行参数。

静止状态不成立会阻止 ZeroGyroUpdate 进入静止陀螺偏置观测；
但不能声称仅修复 at_rest 就必然启用零速度约束：ZeroVelocityUpdate 还要求
没有有效垂直速度辅助或倾角尚未对齐。本次 GNSS 速度融合有效、倾角已对齐，
因此需要分别检查这些约束，不应直接放宽静止门限强行让状态通过。

### 3. 融合及传感器链不是毫米级真值定位

GNSS 位置/速度、GNSS 高度、气压高度、磁航向融合标志有效；外部视觉
位置/速度/高度/偏航融合标志均为 false。日志参数 EKF2_HGT_REF=1（GPS），
EKF2_GPS_CTRL=7、EKF2_BARO_CTRL=1、EKF2_EV_CTRL=0。
SDF 的 send_odometry=1 只是 Gazebo→PX4 里程计报文封装方向的选择，
不等于已经有外部里程计生产者，更不等于 EKF 开启了外部视觉融合。
PX4→MAVROS 的估计 ODOMETRY 输出是另一条方向相反的链。

本窗口 filter_fault_flags、GPS 检查失败标志、time_slip 均为 0；
已记录的 heading/velocity/position/height test ratio 最大约
0.0436/0.1448/0.0232/0.0208，未见这一窗口内大创新拒绝证据。
但 EKF 的 eph 约 0.443–0.505 m、epv 约 0.277–0.304 m，valid=true
不能解释为毫米精度。GNSS 实测经纬度在该窗口量化值不变，气压高度标准差
约 0.0493 m；磁偏置 X 估计从约 +0.00744 G 变为 -0.01885 G。
磁偏置变化与航向变化是后续只读核查线索，不是已经证明的唯一因果关系。

本轮再次用这份 ULog 与同次 MAVROS 原始记录进行 1194 个源时间配对，
支持时差最大 4 ms。NED→ENU 后位置差中位 0.294 mm、最大 1.428 mm；
偏航差最大 0.001265 rad。这继续支持大误差已经存在于 PX4 估计，
而非 ROS 转发引入数十厘米轴错配。没有关闭噪声或注入真值来消除误差。

### 4. 在线重置元数据下一步边界

源码确认：EKF2 的主 estimator_odometry reset_counter 为姿态、位置和速度
重置计数合成；EKF2Selector 转发时另维护自身计数，故主 EKF 与最终 MAVLink
计数不保证数值相等，应检测同一输出流的变化、回绕及启动世代，不能要求等值。
MAVLink ODOMETRY 明确发送 reset_counter，而 LOCAL_POSITION_NED 没有该字段；
当前 ROS nav_msgs/Odometry 同样没有 reset-counter 字段。
现有观察器 estimator_reset_metadata_available=false 仍是准确状态，没有改成 true。

建议下一小步只读补采：同源 IMU link 角速度/加速度真值与传感器输出，
并核实 `/mavlink/from` 原始 ODOMETRY 的 sysid/compid、源时间和 reset_counter
是否实际可用，再设计失联/重启/回绕时清除配准的元数据订阅回归。
未接入或失效时必须明确输出未知，不编造 counter=0。
先区分模型接触高频抖动与传感器噪声，再决定是否另行授权单因素参数试验；
本轮不修改 PID、传感器噪声、EKF 融合参数或闭爪流程。
此阶段仅新增离线 build 产物与本说明记录，保留待审阅；没有新增提交或推送。

## 阶段 19：无雷达组合模型最小抓取 demo（2026-10-07）

本轮按用户重新收束的主线，停止扩展传感器/EKF 小方向，完成一次实际抓取。
直接复用已有组合模型、IK、固件自动抓取状态机和真实接触反馈，未重写控制器。
范围是固定无人机、固定可达方块的台架 demo：无人机在世界 Z=1.2 m 处用
bench_anchor 固定，机械臂正常运动；方块为 dynamic，不绑定、不传送、不删除支撑。
不解锁/飞行，不启动雷达、ROS、MAVROS 或 SUPER，不将该结果称为悬停抓取。

### 修改与入口

仅改 `tests/grasp/run_full_arm_grasp.py` 和 `test_full_arm_grasp.py`：
新增 `--no-lidar`，只从实验副本移除 velodyne link/sensor 和相关 mount joint，
保留机械臂安装与 PX4 IMU/GPS 等必要仿真传感器。生产 SDF/子模块/ROS 库不变。
校准记录新增 lidar_enabled，拒绝雷达配置不同的校准文件；
同时拒绝已有 gzserver/gzclient，不按进程名清理他人仿真。
使用既有 bench_cube20 配置：20 mm/10 g 方块，夹爪 P=.045、原力矩上限 .01 N·m，
指面抗扭 patch=4 mm、coefficient=.8，ODE quick/40/1 ms。
这些是此前已验证的台架配置，不是本轮新增 PID/摩擦参数搜索，也不改飞行世界。
场景生成/检查 Python 回归 19 项通过，纯状态机/反馈 C++ 回归 16 项通过。

状态机入口为 `arm_control grasp_start bench_cube20 10`；脚本只发送一次 START，
后续运动均由固件执行：PRECHECK→APPROACH→ALIGN→CLOSE→VERIFY→
EXIT_SUPPORT→LIFT→RETRACT_CLEAR→RETRACT→SETTLE→HOLD→DONE。
目标坐标是 SO101 base_link、单位 m；模板接近点 (.17,.035,.05)，
校准名义 pickup 点 (.18,.015,.07)，最终收回点 (.12,.015,.03)。
闭爪前按实际夹指参考点进行相对位置微调，连续三次满足 3 mm 对齐条件，
再通过双侧接触/法向力确认夹持，不靠夹爪到达空载闭合角来判成功。

### 本次位置、姿态与结果

新校准案例 `demo_combo_nolidar_cal_20261007_01` 没有方块，完成 IK 实际 pose 校准。
PX4 FK 与实际工具位置差约 0.0375 mm；校准阶段名义 EE 误差 13.54 mm，
只满足既有 benchNear，不冒充该阶段严格到位。之后 ALIGN 独立核对真实对齐。
方块实际放置点：世界 (0.183279,-0.005130,1.064362) m，
对应本固定安装的臂基座 (0.183279,0.005130,0.085638) m。
方块 RPY=(-2.586224,-0.025376,1.557754) rad，按本次测得夹指姿态设置；
这是固定物体姿态模板，不声称任意物体/任意 6D 姿态抓取已经通过。

完整抓取案例 `demo_combo_nolidar_grasp_20261007_01`：

| 验收项 | 实测 |
|---|---:|
| 进入闭爪时真实对齐误差 | 约 1.60 mm |
| 最终状态 | DONE / SUCCEEDED / HELD，fault=0 |
| 收回后独立无支撑保持 | 11.913 s（请求 10 s，加完成后继续观察） |
| 保持期间最低实际抬高 | 40.29 mm |
| 双侧接触比例 | 100% |
| 方块—支撑／非夹指／支撑—臂接触 | 均为 0 |
| 两侧平均法向力 | 0.15947 / 0.15994 N |
| 平均竖直接触合力／物重 | 0.09664 / 0.098066 N |
| 保持末期相对滑移 | 0.353 mm（记录，不作为额外调参方向） |
| 无雷达、动态方块、未解锁、完整阶段及物理验收 | 独立检查全部通过 |

全过程约 78.5 s 仿真时间、80.8 s 墙钟。结果 success/grasp_validated=true，
strict_retention_passed=true；这个 strict 是保持漂移的独立诊断，不表示整个
接近阶段均达到严格 EE 判据。最终收回/HOLD 的 strict_position_reached=true。
ULog 独立核对全程未解锁/未起飞；物体有限、遥测时间单调，未观察到 NaN。
本轮进程已自行退出，仅清理本轮启动的进程。

### 复现（名称必须未使用）

```bash
cd /home/pcz/PX4-RoboMaster
python3 src/modules/arm_control/tests/grasp/run_full_arm_grasp.py demo_cal_new \
  --calibrate --no-lidar --gripper-p .045 --patch-radius-mm 4 --arrival-mm 15
python3 src/modules/arm_control/tests/grasp/run_full_arm_grasp.py demo_grasp_new \
  --calibration build/full_arm_grasp/demo_cal_new/calibration.json \
  --no-lidar --fsm --grasp --retrieve --retrieve-route stow --hold-seconds 10 \
  --gripper-p .045 --patch-radius-mm 4 --arrival-mm 15
```

若想亲自看界面，第二条增加 `--gui --gui-wait`；图形 WSL 终端中运行，
界面准备完成后按回车才发送 START。必须先退出已有仿真；脚本不会代为全局杀进程。
本次产物统一在 `build/full_arm_grasp/<case>/`，包括校准、场景、manifest、
真实 pose/contact CSV、阶段事件、PX4 状态和 ULog；抓取案例另有独立
`verify_demo.py` / `demo_checks.json`，检查全部通过。

结论：无雷达、固定无人机的组合模型最小抓取闭环已实际跑通。
不要求为微小滑移继续调参；下一阶段再设计悬停版抓取入口和安全门控。
当前 bench_cube20 的能力检查依赖固定锚点，不能直接解除锚点后用同一 profile
冒充飞行抓取；悬停抓取与携物飞行仍未验收。
阶段 19 结束时改动留待审阅。按用户随后授权，测试入口/回归已单独本地提交
`59bc0e73ca7b0f984226b8e661f6a4e8683ad7ab`
（`test: add lidar-free combined-model grasp demo`），提交前 Python 回归再次 19/19 通过。
阶段 18–19 记录及以下设计另作说明提交，未推送；未跟踪 flight 副本不改动。

## 阶段 20：悬停版抓取入口与安全门控设计草案（2026-10-07）

本节只有设计：没有实现 hover profile、解除锚点、发送动作或试飞。
用户此次授权提交草案不等于授权执行飞行。当前可运行、已验收的仍是阶段 19 台架 demo。

### 主线与入口

第一版仍只做固定 20 mm/10 g 方块、固定偏航、无雷达、无 SUPER/避障、Gazebo 真值
相对定位的 SITL demo。航控仍使用原 PX4/MAVROS，不向 EKF 注入真值、不调飞行 PID。
不先扩展任意物体、任意姿态、返航或携物航线。

保留 `bench_cube20`（profile 1）及其固定锚点/坐标/物理配置校验；
拟新增独立 `hover_cube20_v1`（profile 2，待实现注册），不把 CAP_BENCH 伪装成有效。
当前 CLI/反馈校验只接受台架 profile，新的入口现在不存在，不能直接执行。
原有 ARM_JOINT_COMMAND/STATUS、bench 请求/反馈语义与枚举编号不改写。

最小控制分工：

1. 专用 demo 协调器复用现有飞行管理器起飞/定点悬停，获取匹配目标的抓取悬停租约。
2. 飞行管理器是唯一飞行设定点发布者；租约存续时锁定位置/偏航目标，拒绝新的
   导航目标，不允许 pending_goal 或 SUPER 回调切换航线。完成但仍 HELD 时也不解锁导航。
3. ArmControl 是唯一机械臂目标发布者；接受一次匹配任务/profile/目标的 START 后，
   复用现有 APPROACH→ALIGN→CLOSE→VERIFY→离台→回收→HOLD，重复 START 仍 BUSY。
4. Gazebo collector 只读真实 pose/contact，增加同物理步悬停几何上下文；
   原接触/力/物体字段可以复用，但台架 world +Y clearance 不改名充当三维净空。
   新上下文单独携带体/臂/物体姿态、机体速度/角速度和三维净空有效性。

悬停租约是持续刷新的任务授权，不是永久 hover_ready 布尔值；任务 ID、profile、
目标身份、源时间/序号/世代均须匹配。旧 latched 数据或转发旧包不能续期。
具体消息布局/编号和本机 SITL 入口实现时再校核，不能绕过现有双向链直接发关节目标。

### 坐标与路径

同一物理步直接用实际 T_G_A（world→臂安装位姿）和 T_G_O（物体位姿），
计算 `T_A_O = inverse(T_G_A) * T_G_O`。不依赖冻结的 MAVROS/Gazebo 原点配准，
不照搬世界 `[x,-y,1.15-z]` 或台架固定点作为飞行目标。
Gazebo 真值明确标注 SIM_GROUND_TRUTH，只用于这版 demo 的相对抓取与验收。

固定物体抓取参考与预接近/侧向离台/世界向上抬高路径按支撑/物体坐标定义，
每步转换到当前臂基座。完整刚体姿态参与换算，不只处理平移或 yaw。
保留当前腕姿态模板，不宣称 5DOF 机械臂已经能跟踪任意 6D 姿态；
相对抓取朝向不可实现时拒绝，不靠移动/重放置方块制造对齐。
回收使用已验证分段路线和场景已知几何的扫掠净空检查，不直接对角线穿过支撑。

关节角/角速度/目标限速、IK 与工作空间限制继续生效。初始折叠 EE 的 Z 可能
低于工作空间 min_z；只对经过 hover gate 的已知初始状态，复用已有有界、单调
向合法域内回归规则，不开放一般越界目标或解除限制。

### 首轮候选门限（设计值，尚未由悬停实测验收）

| 条件 | START / 闭爪前要求 |
|---|---|
| 权威飞行状态 | PX4 armed、OFFBOARD、非 landed、无已处理电机故障；飞行状态数据 ≤1 s，offboard 控制/租约 ≤0.3 s |
| 相对几何/反馈 | 真值几何 ≤0.1 s、配对支持 ≤20 ms；关节 ≤0.2 s、接触窗口 ≤0.3 s；源身份、目标、时钟一致 |
| 悬停稳定 | 真值速度 ≤0.05 m/s、角速度 ≤0.10 rad/s、roll/pitch 各 ≤5°，相对任务悬停参考位移 ≤3 cm；START 前连续 2 s |
| 推力分配 | 当前分配的 thrust/torque 均 achieved；四个有效电机指令 <0.95，连续满足悬停窗口；只代表分配余量代理，不冒充实测推力 |
| 可达/净空 | 目标处于合法工作空间、IK 被原样接受；已知几何三维/扫掠净空有效，非夹指路径候选最低 10 mm；未知不放行 |
| 闭爪对齐 | 真实夹指参考误差 ≤3 mm，三次新观察间隔 ≥0.6 s；相对目标速度 ≤0.005 m/s，抓取朝向误差 ≤5°；期间持续满足 hover gate |
| 夹持/离台 | 复用双侧接触 ≥90%、每侧法向力 >0.01 N 等确认；实际离台抬高 ≥25 mm，支撑接触 ≤0.1% 才能 HELD |

门限的实现应分清启动稳定驻留、软退化消抖与硬故障；不能用一次异常包重拟合目标。
硬故障（非有限值、模式/租约丢失、时钟/身份/重置、明确碰撞/分离）立即禁止新增运动。
候选姿态硬限为 10°；连续分配不足/接近上饱和达到 0.3 s 则退出抓取。
姿态硬限不替代 PX4 原有保护；这些设计值不可直接当作已校准的飞行安全范围。

ArmControl 直接读取 PX4 vehicle_status/control_mode/local_position 重置计数和
control_allocator_status，不能把来自 Gazebo 的几何能力位当飞行授权。
PX4 重置/实例切换须撤销当前租约与抓取，冻结待人工重新授权；不用先完成通用
传感器/EKF 改造项目，也不能在元数据未知时假定没有重置。
同物理步上下文采用统一 producer 世代；不同采样器的 epoch 数字不能直接硬等。

### 异常退出与完成

闭爪前：停止新的臂目标并有界减速/实测位姿附近保持，飞行侧继续安全定点悬停，
任务标记失败；不自动换目标重试。夹持后：保持已有闭合偏置、不自动松爪，
停止臂路径推进，按真实观测保留 HELD 或 UNKNOWN；仅有明确分离证据才 LOST。
请求飞行管理器保持安全悬停，禁止自动返航、降落或恢复任务。
FCU/飞行估计不可用或已经失控时尊重 PX4 自身 failsafe，不能承诺软件必能维持悬停。
不在空中 disarm，不强制夺回模式，不直接把折叠轨迹当安全退出路径。

成功标准仍是实际抓取、收回、无支撑保持 10 s 且未掉落；完成后继续监视接触，
保持夹持/飞行目标所有权，直到另行授权下一任务。小滑移只记录，不作为调参支线。

### 实施顺序与改动落点

先实现 profile 分流、`grasp/HoverGate` 与只读几何/租约输入，回归证明无许可、
旧许可、模式切换、重置及不可达目标均不闭爪；bench 原回归不能退化。
再在无锚点、折叠臂的空载悬停中只观察 gate，不发送抓取 START。
通过后做 APPROACH/ALIGN 的 no-close 联调，再执行一轮悬停抓取和 10 s 保持。
只有这轮通过后才另行设计携物位移/返航；不在同一轮扩展路线规划。

生产代码计划留在现有 `arm_control/grasp/`、Gazebo `so101/` 和 ROS 飞行管理器
的小型协调模块中，不另建一套机械臂/飞行控制器。消息/状态新增枚举只追加、
不重编号；先明确所有权和输入契约，再实现和分别提交子模块/主仓库改动。
以上仅设计归档；本轮未编写或执行悬停版控制代码，未试飞。

## 阶段 21：已知目标输入与坐标转换（2026-10-08）

按用户要求拆成小阶段。本轮检查发现上次额度/工具中断前，已知目标文件尚未落盘，
不将其算作已完成。此前刚开始、未验证的悬停固件接线已经撤回；
差异和两个新增文件备份保存在 `build/grasp_demo_stage21/`，没有激活或编译。
已提交的阶段 19 抓取 demo、台架 profile 和此前未跟踪 flight 副本未改动。
只使用 Ubuntu-20.04，未访问 Ubuntu22。

新增 `src/modules/arm_control/tests/hover/known_target.py`、`known_target.json`、
`test_known_target.py`。输入必须显式标记 gazebo_world、位置单位 m、四元数 wxyz；
拒绝 map/world 模糊别名、非法形状、非有限值、零/错误归一化四元数。
候选悬停位根据机体—臂安装与已校准物体中心参考偏移推算，不直接等于物体位置。
预览始终标记 planned、flight_ready=false、close_permitted=false，
不发布命令、不生成 MAVROS 目标、不声称 IK 可达性已测试。
可对保存的实际机体/臂 world pose 做完整刚体逆变换，但不重标旧采样时刻或声称实时新鲜。

默认示例沿用阶段 19 的 20 mm 方块姿态/夹指参考模板，将世界目标改为
`(0.6, 0, 1.364362) m`；推得机体悬停位约
`(0.416721, 0.005130, 1.5) m`、世界 yaw=0。
名义物体中心在臂基座为 `(0.183279, 0.005130, 0.085638) m`，
不是 FK 工具目标点。默认安装矩阵与当前生产 yhang550.sdf 的 so101 include 一致。
数学/输入测试先 9 项通过；加入后续轨迹/局部控制转换回归后为 11 项通过，
包括非零 yaw、完整 roll/pitch/yaw 快照往返和不能把计划值冒充实测值。

```bash
cd /home/pcz/PX4-RoboMaster
python3 src/modules/arm_control/tests/hover/test_known_target.py
python3 src/modules/arm_control/tests/hover/known_target.py
```

## 阶段 22：已知目标飞行接管与粗到位，不闭爪（2026-10-08）

### 实现范围与输入边界

复用 ROS 现有 super_flight_manager，不另写飞行控制器。新增显式参数
known_goal_mode（默认 false，要求 use_sim_time=true）、KnownFlightSetpoint 消息、
known_flight_handover.h 输入租约及回归。只有专用模式才跳过雷达等待，并从
`/grasp_demo/known_setpoint` 接收 map/ENU 指令；保留 MAVROS 既有
FRAME_LOCAL_NED 枚举/ROS ENU 转换约定，不额外手动翻轴。

源任务首次接受后唯一绑定，旧/重复时刻、换任务、错 frame、非法 mask、
非有限值、过期/未来包不续期；输入 ≤0.3 s，控制位置离当前 odom ≤1 m、
速度 ≤1 m/s、局部高度 0.3–2.5 m。流失效进入 FAILSAFE_HOVER 并锁定，
不能靠后续正常包自动恢复；模式/解锁丢失后不自动夺回/重新解锁。
该模式拒绝普通用户规划目标及 SUPER 指令，管理器仍为唯一 MAVROS 飞行发布者。
正常导航默认逻辑没有关闭雷达等待或改变恢复策略。

CoordinateTruth 仅追加同物理步机体线/角速度世界真值和有效标志；
旧 FK 观察逻辑不变，采样器没有控制动作。消息生成/导出依赖补齐，首次编译因
旧 CMake 规则未生成新头失败，强制重新生成后管理器、真值库及测试编译成功。

PX4 测试目录新增 run_known_target_flight.py，默认只预览；只有 --execute-flight
才启动本机 PX4/Gazebo/MAVROS 与已知目标管理器。拒绝已有会话/占用端口，
父节点与子进程固定使用同一个本地 master，只清理自己启动的进程。
从生产组合模型扩展生成实验副本，移除雷达，不添加 bench_anchor；
物体 dynamic、支撑 static，不绑定或运行中传送物体。实验 ODE quick/40，
没有修改生产 SDF、飞行 PID、EKF 参数或 arm_control。
必要传感器和原初始折叠保持照常运行。

### 真值辅助控制是本版 demo 的明确假设

位置转换每次采用 `T_L_G = T_L_B * inverse(T_G_B)`，将 world 的 6 s 平滑
短轨迹采样转换为当前局部控制坐标；它不是已标定的导航 TF，更不是阶段 17
的冻结原点诊断。速度前馈为
`v_cmd_L = R_L_G * (v_des_G - v_true_G) + R_L_B * v_est_B`。
这里 nav_msgs/Odometry twist 是 body FLU，先按其姿态旋转；偏航也按当前
world/local 姿态换算后走原限速器。

这明确使用 Gazebo 机体真值参与控制坐标及估计速度偏差补偿，是仿真定位输入，
不是只给物体坐标就证明了真实导航定位能力。没有向 EKF 注入真值，但闭环性能
也不能冒充原 GPS/IMU 定位精度已改善或将来实机必能达到同样结果。

### 分级验收与证据

原测试 CTest 4 项加接管租约为 5/5 通过，已知目标 Python 11 项、
原抓取场景 Python 19 项通过。test_known_flight_ros.py 在独立 11361 master
构造消息，6 项接口检查通过：错 frame、有效任务、换任务、旧包、断流退出、
不自动恢复。该测试不启动/连接 PX4、Gazebo、MAVROS；只验证接口，不冒充物理结果。

两次实际无雷达、无锚点、空载折叠臂短程飞行结果：

| 案例 | 到位后保持 | 世界位置平均/最大误差 | 结论 |
|---|---:|---:|---|
| known_goal_no_close_20261008_01 | 10.001 s | 2.23 / 3.50 cm | 本阶段粗到位通过 |
| known_goal_no_close_20261008_02 | 10.003 s | 2.30 / 3.52 cm | 补充物体/折叠证据后通过 |

结果目录 `build/known_target_flight/<case>/` 保存场景、manifest/源码哈希、
逐帧真值/源年龄、ROS graph、日志、ULog 和 result.json。
第二轮起飞前核对实际方块位置与已知输入相差 ≤4 mm；保持期间最大折叠角误差
约 0.02130 rad。唯一飞行发布者 /known_flight_manager_probe，
arm_motion_requests_sent=false、gripper_requests_sent=false、grasp_tested=false。
两个会话均在完成保持后停止本轮仿真进程，没有验证降落、返航或携物。

进入保持前连续 2 s 要求位置 ≤3 cm、速度 ≤0.05 m/s、姿态 ≤5°；
本阶段后续 10 s 粗到位包络为 ≤5 cm。最大 3.52 cm 超过阶段 20 草案的
连续 3 cm 抓取许可要求，因此明确 hover_grasp_permission_validated=false，
不能把 success=true 当作允许闭爪。下一小阶段才接实际悬停许可、相对对齐和拒绝用例，
许可不达标则不启动抓取，不在这一轮把更严格许可改成已经通过。

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
cd /home/pcz/PX4-RoboMaster
python3 src/modules/arm_control/tests/hover/run_known_target_flight.py my_known_goal_case --execute-flight
```

名称必须未使用；不加 --execute-flight 时不会启动仿真或飞行。
阶段 21–22 结束时改动同步到 ROS 发布副本并留待审阅。按用户审核后的授权，
ROS 本地提交 `3f6efbc4c0de15b68860aa5cddce99d005c48c85`
（simulation-only known target flight handover），PX4 测试提交
`753fca0f30eb1756cb8a6bc2f2f7c3ff2a92de51`（known target coordinates/no-close trials）。
提交前 CTest 5/5、坐标 Python 11 项通过；去掉新增文件末尾空行后 diff --check 通过。
本集中说明另作提交，未推送。
旧 flight 未跟踪副本未改动；悬停抓取入口/实际许可仍未实现，闭爪与携物尚未执行。

## 阶段 23：悬停准备许可、拒绝与撤销（2026-10-15）

本轮只访问 Ubuntu-20.04。继续无雷达、动态飞机、折叠臂的已知目标 SITL；
没有发送机械臂运动或闭爪命令，没有启动 hover 抓取 profile，也未调飞行 PID。
完成后按授权本地提交，未推送：

- ROS `0b5b426448e5acd971a346423d97610dc8f89e1b`：真实夹爪/物体速度采集。
- PX4 `200c60eec2626a457f76b1384016b3f605bd39ab`：可撤销 PREPARE 与无动作验收。
- 本集中说明另作提交。原未跟踪 tests/flight/ 副本保持不动，子模块未改动。

### 接口及修改范围

新增 ArmHoverContext/Request/Status 三个 uORB 话题，独立于已有 ARM_JOINT 和
bench 抓取消息。SITL 本地入口 `arm_control hover_context ...` 只发布观测上下文；
`arm_control hover_grasp_prepare <task>` 只申请准备票据。
两者在非 SITL 构建中拒绝。没有新增 MAVLink 抓取执行入口，也不接受此票据发动作。

`grasp/HoverPermit.hpp` 分离悬停条件与准备票据；`HoverPermitAdapter.cpp`
读取 PX4 原生状态并接入 ArmControl。只有悬停条件合格、任务匹配、关节反馈健康且
原抓取流程未占有控制权，才能接受准备。任一条件退出、关节反馈过期、换任务或
抓取流程占有控制权时撤销；恢复不自动准备，旧 request ID 不能再次生效。
`motion_authorized` 和 `close_authorized` 全程硬编码 false。

ROS CoordinateTruth 追加夹爪 link 原点和目标 model 原点的世界线/角速度；
与 pose 同物理步采集，缺失/不支持的目标明确 invalid，不补零。
测试参考点速度使用 v_link + omega × R*offset，不能把 link 原点速度当工具点速度。
偏移仍是现有 cube20 碰撞模板 (-.0079,-.000218121,-.0981274) m，不是任意 IK TCP。
已同步运行工作区和 ROS Git 副本；现有台架控制/夹持参数不变。

### 本轮门限与边界

准备候选连续 2 s 要求：世界悬停位置误差 ≤3 cm、线速度 ≤0.05 m/s、
角速度 ≤0.10 rad/s、真实总倾角 ≤5°；四电机控制量有限、非负、最大 <0.95，
原生分配器推力/力矩达成且无已处理电机故障/停转掩码。
PX4 必须 ARMED + OFFBOARD、未落地、非 failsafe，本地位置/速度有效并且
heading_good_for_control=true。位置/速度来自 Gazebo 真值，不声称 EKF 精度已验收。

源物理时间年龄 ≤100 ms、未来偏差 ≤20 ms，重复序号不能刷新接收年龄；
新任务前不接受时钟回退、世代或原生 xy/z/vxy/vz/heading reset counter 变化。
原生位置/电机/分配器数据 ≤300 ms、vehicle_status ≤1 s、关节反馈 ≤200 ms。
land_detected 发布约 1 Hz，实际允许年龄 1.2 s 而非草案 1 s，容纳周期与排队；
这不改变几何/速度门限。模块更新周期 100 ms，延迟报告采用仿真时间观测值，
不把观察器轮询延迟冒充精确控制反应时间。

close_geometry_ready 还检查参考点对齐 ≤3 mm、相对速度 ≤5 mm/s、姿态差 ≤5°；
它只是几何诊断，本轮保持 false，也不能触发闭爪。start_ready 只表示悬停候选，
不等于完整抓取授权。当前任务 ID 仅绑定本地上下文与准备请求，没有完成与
ROS 飞行租约/profile/目标实体的执行级绑定；这些是接 hover 执行入口前的剩余项。

### 实测发现及修复

原 1.5 m 场景 `hover_prepare_20261015_01` 已到位但一直拒绝准备。
只读核对发现原生 heading_good_for_control=false。当前
EKF/ekf.h 的 isYawFinalAlignComplete() 要求飞行中磁场对齐完成；
aid_sources/magnetometer/mag_control.cpp 的 checkHaglYawResetReq()
在估计 HAGL >1.5 m 才请求该对齐。世界高度 1.5 m 不代表估计 HAGL 已越过门限。
本轮未关闭此检查、未写 EKF 参数，也不把这一拒绝记作通过。

新增 `known_target_high.json` 对照：方块世界 z=1.7643621485 m，
派生悬停机体原点 (0.416721237,0.005129812,1.9) m；原配置未改。
协调器等待原生航向最终对齐后才发送第一个任务上下文，避免把正常起飞对齐的
reset 当作已经绑定任务后的异常；绑定后再发生 reset 仍锁止，不能自动恢复。

`_02` 原型完成整组接受/撤销；随后补强了持续检查关节反馈的撤销条件。
`_03` 在 72.003 s 捕获实际位置误差 0.03040 m，准备刚接受就被原生门控撤销。
初版测试把“下次读取前已撤销”误报异常；修正后该情况只记撤销、等待重新合格，
不计接受正例。补了对应观察竞态单元测试；保留失败产物，不回写成通过。

最终 `hover_prepare_20261015_04` 完整通过，独立 `verify_hover_case.py`
交叉检查 result、原始 hover_status.jsonl、telemetry.jsonl、初末目标与事件：

| 情形 | 实测结果 | 撤销观测延迟（仿真秒） |
|---|---|---:|
| 无上下文/未解锁、任务不符 | 拒绝；不产生准备 | — |
| 正常真实悬停 | 5 次 PREPARE 接受，无动作授权 | — |
| invalid 输入 | 撤销并拒绝重试 | 0.100 |
| 速度注入至至少 0.06 m/s | 撤销并拒绝重试 | 0.103 |
| 位置误差注入至至少 0.04 m | 撤销并拒绝重试 | 0.107 |
| 观测上下文断流 | 过期撤销并拒绝重试 | 0.201 |
| 重发原始序号/采样时间 | 不能续期，撤销并拒绝重试 | 0.210 |

五次恢复后均不自动准备。注入只改变许可测试输入，不改变飞机、物体或关节状态。
关节链实际断开/飞行模式丢失没有在空中注入；原生条件拒绝、票据撤销/重放/任务变化
等纯逻辑已覆盖，不能据此声称实际拔断整条 MAVLink 链也已验收。

到位后观察 19.751 s，世界位置平均/最大误差 1.894/3.584 cm，最大折叠误差
0.02141 rad。粗包络通过不代表持续处于 3 cm 抓取许可内，短暂越界仍会撤销。
初末 target_sequence=0，六关节 joint_target 均为
[-.07,-1.175,1.45,1.3,-1.57,.3]，抓取状态始终 IDLE、不占有控制权。
唯一飞行发布者 /known_flight_manager_probe；无绑定物体、锚点、雷达或闭爪。
hover_prepare_validated=true、hover_grasp_permission_validated=false、
grasp_tested=false。测试自行清理本轮进程，没有验证降落、携物或飞行抓取。

### 回归、复现和后续

最终 SITL/ROS 编译成功；纯 C++ 门控/票据 26 项、观察器 Python 5 项、
已知目标坐标 11 项、原台架场景 19 项、原抓取逻辑 16 项及 ROS CTest 5/5 通过。
独立离线核验 verified=true；提交检查仅修剪新增文件末尾空行，不改变逻辑。

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
cd /home/pcz/PX4-RoboMaster
python3 src/modules/arm_control/tests/hover/run_known_target_flight.py my_prepare_case \
  --execute-flight --observe-hover-permit \
  --target src/modules/arm_control/tests/hover/known_target_high.json
python3 src/modules/arm_control/tests/hover/verify_hover_case.py \
  build/known_target_flight/my_prepare_case
```

名称必须唯一、已有仿真会拒绝启动。结果目录保存场景、manifest/二进制与源码哈希、
原始状态/真值、ROS graph、日志/ULog、result.json；最终轮另存 verification.json。
本阶段只交付可撤销的“准备”接口。下一步再接执行级飞行租约/目标绑定、独立
hover profile 和精确接近入口，继续先无闭爪；不能直接将 bench profile 解除锚点。

## 阶段 24：同任务飞行衔接、折叠恢复与无闭爪接近（2026-10-15）

### 当前结论与范围

本轮仅访问 Ubuntu-20.04。折叠姿态到合法工作空间的恢复已实测通过，
错误计划拒绝、正确计划进入原生接近、侧向展开及暂停保持也有正例。
**整条接近流程尚未验收通过**：最终第 10 轮末段的腕部净空降到约 15 mm，
原 16 mm 几何保护锁止；result.json 保持 success=false，不能记作抓取成功。
夹爪目标始终 0.3 rad，未发送闭爪或携物命令；采样中未见外部接触或 NaN。
没有修改关节位置/速度/加速度限值、飞行 PID、EKF 或原悬停门限。
本阶段按用户授权分段本地提交，未推送；ROS 运行源码与发布副本已同步，忽略解释器缓存后一致。
原未跟踪 tests/flight/ 副本及 Git 子模块未改动。

### 接口与文件改动

- ROS KnownFlightSetpoint 增加 plan/target/scene ID 和 terminal_hold，新增
  KnownFlightStatus。super_flight_manager 仅在同一飞行任务中接受不可换绑的元数据，
  发布原始命令时刻、实际租约和终端保持状态；拒绝包不能续租，旧普通入口保持兼容。
  修改集中在消息、known_flight_handover.h、super_flight_manager.cpp 及其单元测试。
- CoordinateTruth 与 coordinate_truth_plugin.cpp 增加显式 approachGuard 观测，
  输出实际 SO101 盒体对方块、支撑、地面/asphalt 的净空及外部接触标志。
  未知模型、形状、帧或不完整几何不补零，拒绝起飞/接近。没有模型位姿写入或运动指令。
- PX4 新增 ArmApproachRequest/Status，ArmHoverContext 追加执行级绑定、飞行租约、
  实际臂系目标与几何字段。grasp/HoverApproach.hpp 为纯逻辑，
  HoverApproachAdapter.cpp 单独接入 ArmControl，不把台架 fixed-anchor profile 用于飞行。
  入口仅限本地 SITL；未增加 MAVLink/实机抓取执行授权。
- hover_approach_probe.py 只协调原生请求，不直接发布关节目标。
  同一组 task/plan/target/scene 绑定飞行接管、准备票据和接近请求；计划 ID 包含整条
  世界系路线。每次用实际 arm link 完整位姿求逆变换，而不是假设机体水平或手工翻轴。
  这是受信任的本地仿真协调链，不是对任意外部计划内容的密码学认证。
- run_known_target_flight.py 默认仍只预览；显式 --execute-flight --execute-approach
  才执行无闭爪接近。加入只读离线 check_approach_route.py、独立
  verify_approach_case.py 及对应单元测试。后者只接受完整正例，不能给本轮负例盖章。

### 折叠恢复与暂停问题的实际修复

1. 折叠 FK 的 z 约 -5.02 mm，低于普通工作空间 min_z=20 mm。
   复用现有单调向内恢复，只允许受保护的 hover 接近逐步回到工作空间；
   普通手动目标不获得例外。请求点与实际有界 IK 后的 FK 都必须通过检查。
2. 原预算缩放从实测角度出发；wrist_roll 实测约 -1.5737 rad，略低于
   -1.57 rad 命令下限，缩放反而生成非法指令。现从“上次已接受的合法目标”
   向合法候选插值，先为目标与实测之间尚未完成的运动预留预算。
   不钳改真实反馈，也不放宽命令下限。随后修正了从实测重新投影导致 FK 小幅回退、
   被单调恢复正确拒绝的问题。
3. 单步工具点最多 3 mm、每 100 ms 更新；候选最大关节增量 0.02 rad。
   后续仍走原关节速度/加速度限制。整臂扫动半径由生产 SDF 的子树平移和所有盒体角点
   三角界推导，上界为 [0.51,0.45,0.33,0.20,0.13] m，测试检查运动与恢复两处一致。
4. 预算不足与真正 IK/越限拒绝分开：前者进入 PAUSED，不冒充 IK 失败。
   额外要求总净空至少 32 mm，且在原 16 mm 门限和待跟踪位移外仍有 8 mm 余量，
   才显示 resume_ready。必须再次显式同任务 RESUME，条件恢复不自动运动。
   净空本身低于原门限、失联、身份变化、原生飞行故障或重置仍锁止。
5. 悬停时关节须运动补偿机体摆动，不能用“各关节几乎静止”定义世界末端到位。
   无闭爪偏置接近改为实际工具点相对速度 ≤10 mm/s、位置误差 ≤15 mm 持续 1 s。
   这不是闭爪许可；原闭爪几何诊断仍是 3 mm、5 mm/s、5°，并未改动。
6. 多次暂停原先把目标重置为实测角度，会卸掉承载所需的伺服偏差，逐次下垂。
   hover 专用停止现冻结最后实际发出的五关节设定值并清零轨迹速度，不保留未来 IK 轨迹，
   也不重置为实测角度；夹爪目标不变。此轮没有改动既有台架停止策略。

### 路线、假设与未通过的边界

新增 known_target_approach.json，沿用高位试验的同一方块世界位置/姿态，
只将计划悬停机体 x 从 0.416721237 后移到 0.386721237 m，高度仍为 1.9 m。
原 known_target_high.json 保留，准备许可回归不被改写。
名义臂系路线为 (0.10,-0.10,0.03) → (0.19,-0.06,0.10) m，
最终世界点约 (0.576721237,0.065129806,1.750000000) m，
距物体中心约 70.6 mm，只是偏置接近点，不是夹爪已对齐的 grasp target。

原直线的主要障碍是 wrist_body_collision，而非手指。
第 5 轮近点处 AABB 距离约 19.06 mm，定向盒投影下界约 19.12 mm，
不能靠换更紧的包围盒就合理绕过拒绝。也没有删碰撞、调软求解器或增大关节范围。

离线检查只是采样运动学和已知盒体检查，不含实际跟踪/飞行扰动。
补齐上下文工作空间检查后，机体高度偏低 30 mm 的采样会使第一点落在 z=0，
必须拒绝；因此不能声称整个 3 cm/5° 悬停许可包络中任何姿态都可执行本路线。
运行时仍按实际臂位姿与源时间检查。盒体预算也不是任意场景、机体/臂自碰撞、
所有未来飞行运动的连续碰撞证明；10 mm 几何余量加 6 mm 离散采样预留不作这种保证。
本轮仍使用之前说明的 Gazebo 真值辅助控制变换，不证明实机定位能力。

### 实测轨迹与保留的失败产物

所有轮次在 build/known_target_flight/no_close_approach_20261015_XX/ 保留，未回写失败结果：

| 轮次 | 实际结果与定位 |
|---|---|
| 01 | 几何守卫识别到场景另有 asphalt；拒绝起飞，随后纳入实际地面盒体 |
| 02–03 | 飞行/准备/错计划拒绝通过，首步因实测角度插值越过命令边界而拒绝 |
| 04 | 接受 4 次更新，实测重新投影使 FK 向外回退约 54 μm，被单调恢复拒绝 |
| 05 | 合法目标锚定后接受 9 次更新，直线路线腕部净空约 19 mm，预算不足 |
| 06 | 侧向路线接受 49 次更新，但原悬停位下净空仍耗尽 |
| 07 | 后移悬停位后完成侧向点，末段预算不足；此时仍被粗报为 IK 故障 |
| 08 | 加入预算暂停后未坍缩，但中间点沿用关节静止判据而超时 |
| 09 | 修正世界末端到位后进入后半段，反复“保持实测角度”使臂逐步下垂，最终超时 |
| 10 | 冻结已发设定值后恢复与侧向点通过；末段真实净空不足触发几何锁止 |

第 10 轮逐帧证据：60.456 s 错计划被拒且目标序号不变，60.658 s 正确计划进入接近；
77.253 s 已接受 FK 为 (0.10478,-0.06772,0.02031) m，恢复进入合法工作空间；
85.901 s 到达侧向点。累计 163 次目标更新，3 段可核对的暂停中，
跳过转换采样后目标角最大变化为 0 rad，不再随实测误差下垂。

104.133 s 原生 phase=FAULT、fault=Geometry，净空 0.0149854 m；
实际工具点到最终偏置点仍差 0.0337054 m。夹爪命令目标全程 0.3 rad，
采样外部接触数为 0，FK/真值最大位置残差约 13.7 μm、旋转残差 67.6 μrad。
展开期间飞行位置最大误差约 4.59 cm、原生总观测倾角约 0.39°；
超过原 3 cm 许可会暂停，并未改为允许继续。实际关节反馈存在承载跟踪偏差，
不能把“命令合法”表述为物理反馈从不越过命令边界。
本轮已停止自建仿真进程；没有验证降落、闭爪、抬起、回收或携物飞行。

### 回归与下一步

SITL 和 ROS 编译通过。原生接近/预算/冻结 25 项（含 10,000 组预算随机不变量）、
准备许可 26 项、坐标/观察器/计划 22 项、台架逻辑 16 项、台架场景 19 项、
ROS CTest 5/5 通过。新增脚本语法检查通过。
独立正例核验器未对本轮输出 verified=true，因为没有完整正例；不得省略其验收条件。

按用户本轮确认，下一步收敛为最小固定点 demo：
**给定固定抓取目标 → 复用现有 IK 到位 → 闭爪并短距离抬起验证。**
先固定方块和悬停位，选一组合法、可达、不与当前模型碰撞的末端目标，
优先复用已有可抓姿态及到位/夹持反馈；不新增通用走廊规划器、雷达感知或复杂抓取策略。
原关节限位、反馈、悬停与净空保护保留。当前偏置接近点不是已验收 grasp target，
需要先完成这一固定目标的到位核对，再进入闭爪，不直接绕过本轮真实几何拒绝。
本轮证据还不足以要求先修改飞行 PID。

本地提交记录（本轮未推送）：

- ROS `8eb067c`：飞行任务/计划绑定和终端租约状态。
- ROS `40b4aa2`：整臂净空与外部接触观测。
- PX4 `1f1a66985d`：原生无闭爪接近、折叠恢复及暂停保持修复。
- PX4 `01caba664c`：坐标、预算、冻结保持回归及有界试验/独立核验脚本。
- 本集中说明另作本地文档提交；失败产物仍在 build 下，不作为成功正例提交。

```bash
# 离线采样检查，不连接 ROS/Gazebo
python3 src/modules/arm_control/tests/hover/check_approach_route.py

# 明确授权飞行及不闭爪接近后才使用；当前路线尚未验收，可能保护退出
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
python3 src/modules/arm_control/tests/hover/run_known_target_flight.py UNIQUE_CASE \
  --execute-flight --execute-approach \
  --target src/modules/arm_control/tests/hover/known_target_approach.json
```


## 第二十五步：原生 IK 只读检测与主链清理（2026-10-17）

### 本轮边界与结论

按最新审阅意见，本轮只检查原生 IK、拒绝用例及检查前后目标不变。
所有部位仍保留 16 mm 净空，不增加手指接触豁免，不闭爪。
没有发送飞行、PREPARE、展开、笛卡尔目标或夹爪请求；原有折叠保持照常运行。
只使用 Ubuntu-20.04，未访问 Ubuntu22，未改 PID、限位、SDF 或 ROS 源码。

`fixed_target_readonly_20261017_03` 和最终整理后的 `_04` 均通过只读检测。
最终证据：`build/fixed_target_checks/fixed_target_readonly_20261017_04/`。
`inspection_verified=true`，`final_armed=false`，无飞行设定值发布者；
`flight_tested=false`、`grasp_tested=false`，不能当作到位、抓取或飞行正例。

### 原生入口和坐标转换

新增 `diagnostics/IKPreview.cpp`，命令为 `arm_control ik_check x y z`。
仅 SITL 可用；从新鲜 arm_control_status 快照取得合法种子，调用现有
solvePositionIK，结果只写日志。不会调用 updateIKTarget/sanitizeCartesianTarget，
不会写 IK 缓存、目标、到位计时或任务所有权，也不会投递关节命令。
诊断成功执行不等于解算成功：正常拒绝可返回 0，必须读取
workspace_valid / ik_evaluated / ik_success；参数格式非法返回非零。

`tests/hover/fixed_grasp_target.py` 只做坐标转换与严格结果解析。
继续使用已校核的 pad 参考点和 FK 工具偏置，不把方块中心直接当工具原点：

- 由世界物体姿态减去旋转后的夹持参考偏置，得到夹爪坐标。
- 乘以 gripper_frame 工具偏置，再转换到计划或实测 SO101 基座坐标。
- 位置 IK 仍是位置 IK；该参考姿态不是原生 IK 已满足完整抓取方向的证明。

`check_fixed_target_native.py` 默认只打印预览。显式 `--native-sim` 才会
启动其拥有的未解锁仿真；拒绝占用已有仿真，退出只清理自建进程。
每条查询先保存原始输出，再解析，并在每个用例后读取三类状态进行比较。
新增共享 `native_client.py`，替代借用 HoverPermitProbe 未初始化实例的做法；
已有准备许可观察器复用它，不改变准备/拒绝逻辑。

修复三个检测基础问题：

- PX4 日志单条文本有限，原长 JSON 会截断；现在拆为短记录，缺失或截断即拒绝。
- 空闲状态的未测量 inf 保留为 JSON 字符串标记；反馈、关节目标和 IK 结果仍严格校验。
- 原生 -1.57f 输出为 -1.57000005；解析按相同 float32 比较，不扩大真实关节范围。

### 最终实测用例

| 检查 | 实际结果 |
|---|---|
| 计划悬停假设下固定工具点 | 臂系约 (0.218480, 0.015814, 0.077299) m；原生 IK 成功，残差 0.073921 mm |
| 同一点重复查询 | 解、误差及种子序号完全一致 |
| 未解锁实际机体位姿下的同一世界点 | 在臂工作空间外，未进入 IK；不能冒充飞机已到计划悬停位 |
| 盒外 (0.5, 0, 0.05) m | workspace_valid=false，未进入 IK |
| 盒内远点 (0.39, 0.24, 0.39) m | 原生求解失败，残差约 77.709 mm；未接受或发布目标 |
| NaN、Inf、尾随字符、缺参数 | 全部返回非零，拒绝输入 |

9 次查询/拒绝后及最终快照共 10 个不变性检查点：
六关节 joint_target 最大变化 0 rad，target_sequence、ik_status、ik_error 不变；
抓取/接近 phase、last_request_id 不变，所有权始终未锁定，夹爪目标始终 0.3 rad。
每项状态和输出分别保存在 native/、invariants.json、before.json、after.json。
上述 0 rad 是原生 listener 状态浮点读出精度下的差值，不是额外的位级测量；
同时通过目标序号/缓存/所有权核对与只读实现审阅确认未投递或写入新目标。

这里证明的是“只读解算/拒绝不改变控制目标”，不是物理运动全程安全。
本轮没有执行净空路径审查、碰撞接触、完整朝向到位或抓取。
原 16 mm 拒绝仍有效，不能因位置 IK 成功就直接闭爪。

失败尝试也保留：20261015_01 为日志解析失败；20261017_01 为空闲 inf 保存失败；
20261017_02 为 float32 边界误报。未覆盖或改写为成功。

### 清理范围、保留主链与恢复

删除 5 个历史源文件，共 1407 行原文件内容：

- tests/grasp/grasp_fixture.py、grasp_probe.cpp、grasp_load_plugin.cpp、run_grasp_case.py：
  旧双连杆夹具、载荷与探针实验。没有当前运行主链引用。
- tests/flight/run_empty_arm_hover.py：早期 SUPER 接管试验，原为未跟踪文件；
  当前已知目标流程不依赖 SUPER/雷达，不继续保留这套独立试验入口。

删除前已将上述五项及旧 run_full_arm_grasp.py 一并归档：
`build/cleanup_backup_20261017_01/legacy_experiments.tar.gz`。
SHA-256：
`0d2635f7a8fa6de0a86b6169de7d0889dabfded2fd9650fb04bec31320886d56`。
四个已跟踪旧文件还可从 Git 历史恢复；未跟踪飞行脚本必须使用该备份。
需要恢复时先解压到新的审阅目录，不直接覆盖当前源码。
备份在被忽略的 build 内，尚未上传；清理 build 前应另存。

旧 grasp_probe.cpp 的时钟分支仍是全臂基线依赖，不能整段丢弃：
已原样抽成 grasp_clock_plugin.cpp，更新整臂编译和源码指纹入口，
仍为不超过 100 Hz 的 Gazebo 仿真时间，无控制命令。
另删除本轮未验收、超出当前范围的假设终态碰撞审计分支，避免混淆只读 IK 结论。

当前保留的工作链：

`已知目标/坐标转换 → 飞行目标接管 → 悬停许可 → 原生不闭爪接近`；
后续经审阅后再接闭爪、抬起与回收。
台架整臂抓取基线、夹持反馈、失联保持、取消/失败退出和关键回归全部保留；
台架锚点 profile 不作为飞行入口。旧章节仍保留实验历史，
其中已删除入口的命令仅是历史记录，不再是当前推荐运行方式。

### 回归和复测入口

最终 SITL 构建通过；只读/坐标/计划/观察器 Python 35 项、全臂场景 19 项、
原生接近、悬停许可和 16 项抓取逻辑通过；102 组 SDF/FK 对照与到位边界通过。
抽取后的独立时钟插件编译通过。未重跑会驱动夹爪的物理台架实验。
本轮未新增其他说明文档，未自动提交或推送。

```bash
# 无 ROS/控制连接的默认预览
python3 src/modules/arm_control/tests/hover/check_fixed_target_native.py

# 独立未解锁只读复测，不执行飞行/展开/闭爪；需使用唯一名称
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
python3 src/modules/arm_control/tests/hover/check_fixed_target_native.py UNIQUE_NAME --native-sim

# 不启动仿真的离线回归
cd src/modules/arm_control/tests/hover
python3 -m unittest test_fixed_target_native test_known_target test_hover_probe test_approach_plan -v
```

## 阶段 26：固定点悬停抓取最小链路（2026-10-19，物理验收尚未通过）

本轮继续“固定目标 → 现有 IK 接近 → 受控闭爪 → 世界向上 30 mm 抬升”。
仅 Ubuntu-20.04、本地 SITL；不接雷达，不改飞行 PID，不扩大关节/工作空间限值。
上一轮只读和清理工作已经本地分段提交：`0844e2c7f2`、`0dfe0acd41`、`c55299f37f`。
本阶段新增代码尚未提交或推送，不能视为完成抓取验收。

### 授权边界及实现

- 仅两个指垫与指定 `test_cube` 允许接触；精抓阶段指垫与支撑至少 3 mm，仍禁止接触。
  其余臂部/地面/非目标物保持 16 mm。展开阶段尚未进入精抓，全部接触对（包括指垫/目标）仍为 16 mm，
  禁止任何外部接触；原无闭爪入口及其保护不变。
- 闭爪要求物理对齐误差 ≤3 mm、相对速度 ≤5 mm/s、朝向误差 ≤5°，
  同时保留飞行/悬停/任务租约/反馈新鲜度/控制器分配状态等原生门控。
- `grasp/FixedGrasp.hpp`、`FixedGraspAdapter.cpp` 新增独立 SITL 状态机：
  `Approach → Align → Close → Verify → Lift → Hold → Done`。
  四项任务绑定一致才允许请求；取消/故障保留所有权锁止，保持夹爪偏置，不自动打开或重启。
  暂停后必须同任务显式恢复；故障及取消不能恢复。
- 展开先用固定小步把 wrist_flex 从 +1.3 rad 恢复至 -1.3 rad，选择已有位置 IK 的向下分支；
  每步最多 0.02 rad，复用原关节限位、单调工作空间恢复、实际跟踪误差和扫掠净空预算。
  随后进入原生位置 IK，不新增通用姿态规划器。生产初始折叠配置不变。
- `FixedDemoPlan` 仅为第一次固定点实验模板。根据该 IK 分支在仿真启动前确定方块朝向和悬停位；
  运行后不旋转/移动/绑定方块。固定方块中心为 world `[0.6,0,1.5] m`，
  当前 IK 接近点为 arm `[0.26,0.015,0.36] m`，抬升为世界向上 30 mm。
  在此之前先到 arm `[0.23571021,0.01420103,0.32822945] m` 的约 40 mm 预抓取点，
  到位稳定 0.5 s 后沿指长方向接近；两点都用现有位置 IK，不增加通用规划器。
  精接近轨迹在世界系按每 100 ms 最多 1 mm 推进，仍保留原速度/跟踪保护。
  `_21` 起最后约 40 mm 插入改为 2 mm/s（每 100 ms 至多 0.2 mm），
  初段仍为 10 mm/s；夹指方向使用方块真实旋转投影而非只看中心。
  开口侧向间隙不足 1 mm 或朝向超过原 5° 时，不继续插入，只允许原预算内横向小步对齐。
  微调每次最多 0.1 mm、总量仍不超过 5 mm，跟踪不足或相对速度偏大时不累计，防止追滞后积分。
  不能在机体系直接采用同样慢速：那会占用抵消飞机微动的速度容量。
  世界系推进点先按实测全姿态转到臂基座，再由原生关节/净空预算接管。
  `_18` 起仅新增精抓 profile 在展开完成后使用 20 ms 调度，
  Cartesian 每步 0.6 mm、关节每步 0.004 rad、夹爪每步 0.004 rad；
  与原 3 mm/100 ms、0.02 rad/100 ms 保持相同每秒速率容量，无长间隔追赶。
  固定展开、原无闭爪模式及台架模式仍使用原 100 ms；原力矩、轨迹速度/加速度上限不变。
  暂停、跟踪不足时世界系轨迹也停止；时间间隔变长不允许一次补跳。
  起飞前必须同时通过接近和整段抬升 IK 检查；仅接近可达不够。
- `run_known_target_flight.py --execute-fixed-grasp` 显式开启新入口，
  与原许可观察/无闭爪接近模式互斥，仍须另给 `--execute-flight`。
  默认预览无运动权限，检测已有仿真即拒绝，不抢占其他实例。

### 临时场景及反馈证据

生产模型和 Gazebo 子模块源码未修改。生成的实验世界不含台架锚点，方块没有附着关节：
20 mm / 10 g 动态方块，4×24×4 mm 静态小支撑；因放置面倾斜，
仅实验副本的方块/支撑线性摩擦系数为 1.2。质量、惯性不变。
临时夹爪行程收窄至 `[0.04,0.30] rad`，P=0.25，D=0.001，I=0，
力矩上限仍为 0.01 N·m，失联策略 `last_target`；指垫采用既有 4 mm 扭转摩擦补丁。
`_10` 的展开中段测得 wrist_flex 静差约 0.0737 rad、保持力矩约 0.088 N·m，
没有达到 0.22 N·m 上限，但末端约 15.9 mm 误差触发原 15 mm 跟踪门控而停滞。
SDF 质量/质心估算整个接近段肩抬升/腕弯曲最大重力保持需求约 0.116 / 0.111 N·m。
因此仅实验副本加入 shoulder_lift I=1.0、iMax=0.18 N·m，
wrist_flex I=0.8、iMax=0.14 N·m；P/D 和 0.8 / 0.22 N·m 原力矩上限不变。
`configure_arm_hold()` 检查原力矩上限不符就拒绝，不回写生产 SDF。
空载接近时方块中心取 gripper `[0.005,-0.000218,-0.102] m`，与固定指侧约留 3 mm，
而不是直接使用夹紧后靠固定指的中心。支撑沿方块局部 X 偏移 -1.5 mm，
离线检查开口放置和闭合约 3 mm 位移后的重心投影均在原 4 mm 支撑宽度内。
相对旧参考点抓在同一方块上高 14 mm，方块和支撑尺寸不变；固定指名义轴向重叠约 9 mm，
完整夹爪行程最坏支撑距离约 7.97 mm，仍使用原 3 mm 要求，不扩大接触例外。
采样预检还要求整条空载接近路径的两指垫与目标保持正间隙；
实际接近若在误差 >6 mm 或朝向 >5° 时已经出现指垫接触，则立即终止，不继续顶推。
闭爪请求后冻结最多 5 mm 的世界位置微调，夹紧过程不追着目标移动。
悬停暂停或 start_ready 撤销时同样冻结微调，不能在暂停期间累计待恢复目标。
起飞前要求方块位置偏差 ≤4 mm、姿态沉降 ≤2°，不以放宽朝向要求掩盖支撑不稳。

ROS `CoordinateTruth.msg` / `coordinate_truth_plugin.cpp` 新增显式精抓诊断字段。
旧无闭爪字段语义不变。新净空值是“各接触对最小剩余裕量 +16 mm”，不是所有对统一原始距离。
只读观察器使用世界和方块坐标投影所得保守距离的较大值，分别给出：
`fixed_grasp_current_clearance`（实际臂几何并预留夹爪回到张开目标的位移）和
`fixed_grasp_clearance`（完整夹爪行程并预留角采样区间位移）。两者均使用同一 3/16 mm 物理门限。
张开接近只使用当前/待张开证书；完整行程证书必须在原生 close_entry_ready 判定中
通过待执行臂跟踪预算，才允许进入闭爪。Close/Verify/Lift/Hold/Done 的臂运动预算、
夹爪步长和独立物理验收继续使用完整行程证书；不能以张开证书代替闭合证书。
接触白名单仅含准确的两个指垫与准确目标碰撞体。
实际物理步另行统计双侧接触和支撑接触，不能用目标误差代替抓取成功。
两处 ROS 改动已同步至 `/home/pcz/so101-px4-ros/super_px4_bridge/`，与运行工作区内容一致，尚未提交。

`fixed_grasp_probe.py` 不直接发送关节目标，原生状态机拥有关节控制。
10 Hz MAVLink/uORB 抓取反馈按原始 Gazebo 采样时间，与同身份真值的左右两帧插值比对；
不外推、不重打时间戳，最大时间偏离 25 ms，位置残差上限仍为 3 mm。
有界队列保留尚未取得右括号的反馈，旧证据不能续期；有效证据时效 150 ms。
高频只读反馈采集与慢速状态查询分离，避免串行诊断查询阻塞配对。
50 Hz 真值帧只计算一次 FK/几何，反馈读取间隔 40 ms；上下文 100 ms 和配对证据 150 ms 时效不变。
重复真值帧必须在消费状态查询序号之前跳过，避免吃掉接近请求回执而误触发折叠姿态检查。
本地频繁诊断复用 PX4 已有 Unix daemon 协议，不新增控制服务或绕过原生门控：
仅精抓观察器显式启用，逐连接核对 SO_PEERCRED 为本轮拥有的 PX4 PID 和当前 UID，
同一 listener 解析器、原生返回码、4 s 超时和有界消息；异常不静默回退。
启动时与官方 CLI 的 joint_target 快照比较一致才允许本轮飞行。
其他入口仍默认使用原进程式客户端。6 项 mock 测试覆盖协议分片、拒绝、错误身份和截断。

验收必须同时具备：真实双侧接触及两侧法向力、实际脱离支撑、物体实际抬高至少 25 mm、
原生保持 5 s 后再独立观察 5 s、期间无禁止接触、无非法几何/反馈失效。
当前实现不是全姿态抓取、携物飞行或返航验收。

### 本轮已验证及尚未完成

离线 55 项（含新固定点、抬升不可达拒绝、时间配对竞态、重复帧不能消费请求回执、积分副本不变性、世界系推进、插入侧向正间隙、暂停微调冻结及 IPC 协议）、全臂场景 19 项通过；
固定抓取原生逻辑 21 项（含等速率高频步长、缺少完整行程证书拒绝闭爪、携物证书失效故障退出）、既有接近/许可/台架抓取逻辑通过。
SDF/FK 102 组对照最大位置差 3.533 μm，到位判据回归通过。
PX4 SITL 与 ROS 观察插件均构建通过。以上不能代替真实闭爪/抬升证据。

物理证据统一位于 `build/known_target_flight/fixed_point_demo_20261019_XX/`：

- `_01`：临时支撑上方块稳定，起飞后因最近帧坐标比较失败退出，未展开/闭爪。
- `_02`：配对修正后起飞数据残差约 0.53 mm；额外抬升 IK 预检拒绝旧点，主动停止，未闭爪。
- `_03`：新可达点支撑沉降约 3.2°，未通过起飞前 2° 检查，未解锁。
- `_04`：支撑加宽后稳定；飞机到位并取得准备许可，首次展开因观测时序保护锁止，未闭爪。
- `_05` 至 `_08`：逐步定位为反馈配对/查询时序问题，而非净空不足；
  仍保留原阈值，均安全退出且未闭爪。`_08` 证实真值帧间隔正常（最大约 21 ms），
  未配对反馈等待右括号时被慢查询拖延，旧证据达到 0.21 s。
- `_09`：反馈时效稳定；仍因慢诊断导致上下文年龄 123 ms 超过原 100 ms 门限而退出，未闭爪。
  随后将慢诊断轮询也与上下文发布分离，没有扩大时效门限。
- `_10`：观测时序已稳定运行超过 150 s 仿真时间；因上述真实静差停在展开中段，主动停止，未闭爪。
- `_11`：临时有界积分将展开跟踪误差降至约 0.41 mm，但旧候选点的展开末段有打开的指垫扫到方块，
  反馈残差检查触发退出，未闭爪；这不是成功或受控抓取。
  原因是精抓的指垫/目标例外过早覆盖了展开段。现已改为展开时复用原全对 16 mm 原始距离及禁止接触观察器，
  原生确认展开完成才开放精抓例外；新固定点展开名义全对最小净空约 52.8 mm，旧危险点离线预检直接拒绝。
- `_12`：全保护展开完成；直接 IK 路线在约 19 mm 对齐误差时固定指先接触，随后将方块顶离支撑，
  净空保护锁止，未闭爪。后续区分空载开口中心与夹紧参考，末段减速，增加过早接触拒绝。
- `_13`：过早接触在约 28 mm 误差处被新检查拒绝，方块当时仍有支撑，未发送闭爪。
  直接到点仍会侧扫，因此离线选择上述标准预抓取点；空载接近采样全程保留约 3 mm 指侧间隙。
- `_14`：两段接近未再顶走方块，但预抓取到位稳定条件未满足，主动停止，仍未闭爪。
  检查发现慢速推进错误地作用于机体系，也限制了抵消飞机微动的能力。
  下午继续时已加入独立世界系推进点和暂停/长间隔不补跳测试，原生相对机体的 3 mm 步长保持原有上限。
- `_15`：恢复世界系推进后，仍出现进程式频繁诊断导致的上下文时效失败，未闭爪。
- `_16`：复用既有 daemon 协议后时序稳定，预抓取点实测误差约 0.243 mm；
  末段约 19 mm 误差时指垫提前接触，立即退出，未闭爪。
- `_17`：预抓取点实测误差约 2.98 mm；横向微调加入后，末段仍有毫米级跟随滞后并提前接触，
  保护退出，未闭爪。微调仅沿实测指间方向，单次不超过 0.75 mm、总量不超过 5 mm，
  闭爪请求后冻结，不将剩余插入距离算成微调。
- `_18`：高频小步使原生末端跟踪约 1.3 mm；预抓取误差约 3.49 mm。
  末段存在飞行位置超限暂停和朝向一度超 5°；冻结臂后机体仍继续漂移，最终在约 23 mm 对齐误差时提前接触退出，未闭爪。
- `_19`：验证暂停目标冻结；预抓取误差约 3.11 mm，末段首次几何暂停时悬停 denial=0，
  随后提前接触退出，未闭爪。只读重放 125.108 s 真值：张开状态最坏对为固定指/支撑，
  实际距离约 16.84 mm（要求 3 mm、剩余 13.84 mm）；未请求的完整闭合行程最坏距离仅约 7.48 mm，
  减去 3 mm 和角采样保留量后，剩余小于约 4.60 mm 的待执行臂跟踪上界。
  因此不能把“全行程虚拟闭合净空”提前作为张开接近预算。双证书修正保留所有真实距离要求。
- `_20`：双证书复测不再在原位置几何暂停；末段约 17.43 mm 对齐误差、1.65° 朝向、
  悬停 denial=0 且未暂停时，固定指仍提前接触，立即退出，未闭爪。
  失败事件新增保存首次接触的真实误差、接触比例、原始反馈时间和两种净空值。
  此次原生臂跟踪约 2–6 mm，10 mm/s 插入叠加机体微动仍存在侧扫；
  因此不提高原生速度容量，先改为上述慢速插入和侧向间隙不足不插入。
- `_21`：2 mm/s 插入和侧向间隙检查下，预抓取误差约 1.26 mm；
  固定指仍在约 10.14 mm 对齐误差时提前接触，立即退出，未闭爪。
- `_22`：提高同一方块上的抓持位置后，预抓取误差约 0.986 mm；没有记录提前接触，
  但在约 29.07 mm 对齐误差时上下文超时，原生 Contract 故障退出，未闭爪。
- `_23`：原生已接受接近并开始合法展开；测试观察器的重复帧过滤吃掉状态回执，
  仍按折叠目标检查，在展开约 0.08 rad 时退出，未闭爪。
  2026-10-21 只修正回执消费顺序，不放宽折叠阈值或原生保护；重新运行 `_20261021_01`。

2026-10-21 复测位于 `build/known_target_flight/fixed_point_demo_20261021_XX/`：

- `_01`：回执修正后，展开接管正常，状态最大记录间隔约 228 ms；
  135.675 s 到达预抓取点，实测误差 1.361 mm。159.622 s 在 11.581 mm 对齐误差、
  3.393° 朝向误差时固定指提前接触，立即退出，未发送闭爪，无抬升证据。
  本轮自身进程已全部结束；没有修改生产 SDF、飞行 PID 或速度/接触要求。
- `_02`：使用同一物理参数复测最终回执游标版本，接管回执正常；
  120.434 s 到达预抓取点，实测误差 1.357 mm。159.746 s 在 10.120 mm 对齐误差、
  4.648° 朝向误差时固定指提前接触，保护退出，未闭爪。自身进程已全部结束。
  两轮均未产生物体抬升/保持验收证据，不能标记最小抓取 demo 完成。

对 `_01` 末段 118 个状态样本做只读重放，匹配到 ±40 ms 内的机体真值速度。
世界固定末端的机体系补偿速度为 `-R_A^T * (v_body + omega_body × (p_tool-p_body))`，
只计算抵消机体运动，尚未加上插入速度或夹爪朝向补偿。
所需末端速度中位 17.327 mm/s、90 分位 31.328 mm/s、最大 39.259 mm/s；
11.86% 样本超过现有 30 mm/s 目标容量。现有跟随误差中位 3.180 mm、90 分位 5.362 mm、
最大 6.940 mm，已经不可直接保证名义 3 mm 侧向余量。
另外以现有 FK 的中心差分雅可比（步长 1e-5 rad）求局部位置补偿所需最小关节无穷范数，
即使不约束 IK 零空间，46.61% 样本仍需要超过 0.1 rad/s；
这是局部必要速度估计，不是动态执行证明。现有原生步进又要求实测关节速度 ≤0.1 rad/s。
因此当前卡点不能归因于夹爪力度，后续必须协调精抓悬停的实际微动与补偿容量，
不能通过扩大白名单、放宽 3/16 mm 净空或绕过闭爪判据宣称成功。
本轮未据此修改飞行 PID，也没有擅自提高速度限制。
下一步“精抓悬停微动抑制／补偿容量协调”已于 2026-10-21 获用户授权，实施记录见下一节；
不能把现有粗悬停许可等同于毫米级精抓稳定性。新增工作本轮未提交或推送。

每轮保存临时世界、源码/构建哈希、`preflight_state.json`、飞行/抓取逐帧记录和结果。
只结束自身创建的进程，不全局清理 Gazebo/PX4，不修改 Ubuntu22。

```bash
# 离线预检：无 ROS 连接、无运动权限
cd /home/pcz/PX4-RoboMaster
python3 src/modules/arm_control/tests/hover/fixed_demo_plan.py

# 唯一名称；显式允许已审核的飞行、接近、闭爪和 30 mm 抬升实验
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
python3 src/modules/arm_control/tests/hover/run_known_target_flight.py \
  UNIQUE_NAME --execute-flight --execute-fixed-grasp
```

## 27. 精抓悬停与补偿协调（2026-10-21）

本阶段保留原有 3/16 mm 净空、3 mm /5 mm/s /5° 闭爪判据、反馈时效、
0.1 rad/s 原生跟随门槛和既有关节/力矩限制。没有修改 PX4 飞行 PID 或生产 SDF。

### 对照实验中的方案（附加飞行外环已撤回，见本节末尾的当前实现）

- `tests/hover/known_target.py`：新增仅用于 terminal fixed demo 的有界世界系外环。
  已知世界位置目标、目标身份和偏航不变，依次试过回位/阻尼增益 1/1、3/2、1/2，
  向量范数不超过 25 mm/s、变化率不超过 50 mm/s²；超过 100 ms 更新间隔或时钟倒退拒绝继续。
  仍显式使用原 SIM 真值控制变换，不注入 EKF，也不声称具备实机定位能力。
  这是显式改变末段外环设定点策略，不等于“没有改变控制”；PX4 内部 PID 参数未改。
- `tests/hover/run_known_target_flight.py`：仅固定抓取模式的六秒航段结束后启用阻尼；
  原只读/许可/无闭爪入口不启用，逐帧记录阻尼值，manifest 记录参数和作用范围。
- `grasp/FixedGrasp.hpp` / `FixedGraspAdapter.cpp`：精抓工作项本已 20 ms 调度，
  原额外 20 ms 整数门槛遇到 19 ms 调度间隔时会漏一次更新。
  现允许至少 16 ms 的精抓回调，但使用实际经过时间预算，且最多一周期预算，禁止迟到补跳。
  末端仍不超过 30 mm/s，关节目标和夹爪仍不超过 0.2 rad/s，单次仍最多 0.6 mm /0.004 rad。
  展开仍采用原 100 ms 节拍。原跟随、IK、扫掠预算及所有安全判据均保留。
- `tests/hover/fixed_grasp_probe.py`：按同一步机体线速度、角速度和实际 TCP 杠杆，
  计算固定世界 TCP 的补偿需求。预抓取实测误差与跟随误差各 ≤3 mm、需求 ≤20 mm/s、
  实测关节速度 ≤0.08 rad/s、飞机距名义悬停位 ≤5 mm 等条件连续满足 1 s，才进入末段插入。
  末段余量不足时只冻结世界任务推进，仍把固定世界推进点变换到最新臂坐标补偿机体运动。
  不把“暂停插入”误做成“冻结机械臂补偿”，不改变目标或假造到位证据。
  暂停/撤销准备许可时也不累计预抓取稳定时间或切换路线段。
- 回归补充至原 `test_known_target.py`、`test_fixed_demo.py`、`test_fixed_grasp.cpp`，不新增说明文件。
  插入段在原逐帧文件密集记录物理真值；慢速原生状态保留原 timestamp，并显式标记是否本帧新读到，
  不能把旧状态当成新的物理证据。

### 单变量物理对照与后续复测

`precision_hover_20261021_01` 只加入阻尼，尚未修正节拍和推进余量。
末段补偿需求中位/90 分位/最大为 10.64/19.58/26.88 mm/s；两轮旧基线分别为
17.33/31.33/39.26 和 15.26/26.06/43.48 mm/s。
本轮末段跟随误差中位/90 分位/最大为 2.46/3.98/4.67 mm，仍不足以保证精抓间隙。
109.785 s 到达预抓取，误差 2.043 mm；130.079 s 在 10.843 mm 对齐误差、
5.563° 朝向误差时固定指提前接触，保护退出，未闭爪，自身进程已结束。
这些阶段匹配数据表明阻尼有改善，但不能证明全姿态/长时间稳定，更不是抓取成功。

`precision_hover_20261021_02` 使用纯速度阻尼、按时间计量步长和补偿余量检查。
112.077 s 到达预抓取，误差 1.320 mm；161.830 s 在 8.596 mm 对齐误差、4.712° 朝向误差时
固定指提前接触，保护退出，未闭爪，自身进程已结束。末段补偿需求中位/90 分位/最大为
10.63/18.91/34.84 mm/s；跟随误差 2.39/5.658/8.50 mm，推进余量通过比例约 40.1%。
只加速度阻尼仍允许约 2 cm 悬停位置偏差；122.160 s 真值中，实际偏航接近零，
肩转实测 -0.1322 rad，而静止预抓取参考约 -0.0329 rad。位置 IK 在补偿横向偏差时改变了夹爪朝向，
所以不能把 EKF heading_good_for_control 或粗悬停等同于抓取朝向到位。

`precision_hover_20261021_03` 在相同 25 mm/s /50 mm/s² 附加速度限制内加入回位项，
第一组回位/阻尼增益为 1/1，并要求 ≤5 mm 真正精抓悬停位置。
266.669 s 才通过连续稳态，预抓取误差 1.027 mm；283.375 s 按原有 240 s 原生总时限退出。
未提前接触、未闭爪，自身进程已结束；不能通过扩大时限或放松 5 mm 要求把它标记为通过。

`precision_hover_20261021_04` 使用 3/2 配比，悬停误差反而扩大，227.930 s 人工中止实验；
仅停止该实验自身进程，结果记录 `own_processes_stopped=true`。这不是原生保护自动失败，也不是通过。
`precision_hover_20261021_05` 使用 1/2 配比，最后遥测为 234.414 s，未留下完整 `result.json`；
最后精抓记录仍处于接近阶段、未闭爪，不能计为完成或声称完成了正常退出验收。
再次检查时上述 PX4/Gazebo/MAVROS 进程均未运行。没有启动第六轮增益试验。
只有实际物体双侧夹持、脱离支撑、抬高 ≥25 mm 并完成独立保持证据，才能计为抓取完成。

### 本轮收束后的当前实现（2026-10-21）

按用户要求停止叠加控制层，当前只保留一条链路：
`既有飞行目标接管 → 展开后机体系保持并等待稳定 → 固定世界目标的现有 IK → 受保护的闭爪/抬升`。

- 删除 Python `PrecisionHover` 附加 PD 及其专用测试。恢复既有 `local_motion` 坐标/速度偏置转换，
  不更改 PX4 内部飞行 PID，不修改生产 SDF，不再继续增益组合试验。
- 删除新增的 `ArmFixedGraspStatus.msg` 和专用发布器；精抓复用 `ArmApproachStatus.msg`，
  `mode=0` 为原无闭爪接近，`mode=1` 为固定点精抓。不同模式分别解释阶段/故障编号，
  每周期仅当前拥有者发布；观察器拒绝未知模式及其他拥有者，旧无闭爪观察器拒绝精抓模式。
  相对本轮之前的主仓库，机械臂消息总数不增加；既有 MAVLink 关节命令/状态接口不变。
  已验证的台架反馈和许可接口暂不做大范围协议迁移。
- 展开完成后，保持当前合法的机体系末端目标，避免立即追赶展开瞬态。
  真正悬停误差 ≤5 mm、机体速度 ≤5 mm/s、角速度 ≤0.02 rad/s、臂跟随误差 ≤3 mm、
  关节速度 ≤0.05 rad/s，且许可/反馈/净空有效，连续满足 1 s 才开始世界系 IK 路线。
  任何源间隔超过 100 ms 重置等待，不跨缺帧补计稳定时间。
- 世界目标补偿只做同一个目标向最新臂坐标的变换，不增加独立补偿状态机。
  末段推进仍为 2 mm/s，补偿余量不足则停止推进；关节/力矩/时间限制不变。
- 保留指垫—支撑至少 3 mm、其余至少 16 mm；闭爪仍需真实 3 mm 对齐、5 mm/s 相对速度、
  5° 朝向、完整行程净空和悬停条件。没有用“精简”绕过接触、失联或失败锁止保护。

代码回归：58 项 hover 脚本、19 项整臂脚本、24 项原生精抓、16 项台架状态机用例通过；
原有悬停许可及无闭爪接近原生测试通过。PX4 SITL 重新构建通过（包含共用状态消息生成）。
以上逻辑测试不是实际抓起物体的证据。上述实现已并入下节所列本地提交，未推送；所有说明仍集中于本文件。

单次物理复测 `build/known_target_flight/minimal_hover_20261021_01/`：

- 60.829 s 原生接受接近，92.992 s 展开后进入机体系保持；共用状态 `mode=1` 正确，
  没有启动世界系接近，没有请求闭爪。300.003 s 按原仿真总时限结束，
  `error=Simulation-time limit exceeded`、`own_processes_stopped=true`，并非精抓验收通过。
- 100.055–299.875 s 的 992 个保持阶段状态样本中，臂跟随误差中位/90 分位/最大
  为 0.190/0.230/0.310 mm，关节速度中位/90 分位为 0.006/0.013 rad/s；
  悬停位置误差则为 18.470/34.358/52.430 mm，机体速度为 12.345/22.154/35.450 mm/s。
  5 mm/5 mm/s 连续稳定条件没有通过，因此等待和闭爪保护按预期阻止继续。
- 同期密集飞行真值中，X/Y/Z 绝对误差中位为 11.057/10.444/0.817 mm，
  90 分位为 25.490/26.353/1.879 mm。当前主要是水平悬停误差，不是高度跟踪失败。
- 同一日志只读对照 `vehicle_attitude` 与真实机体姿态，转换 NED/FRD 与 ENU/FLU 后，
  机体系重力方向差中位约 3.120°、90 分位约 3.336°；
  `vehicle_attitude_groundtruth` 与真实机体均接近竖直。该结果提示先检查姿态估计/传感器，
  不能直接认定为 PID 或 IMU 安装关节错误，也不能用附加补偿把它掩盖。
  此次未改传感器、估计器、飞行 PID 或生产模型参数。

本阶段已完成代码收束、编译和一次有界复测，但精抓悬停正例仍未通过。
下一步只定位上述姿态差与水平悬停偏差，不新增 uORB 消息或抓取状态机。

### 提交前最终清理

用户授权先清理、提交当前，再继续悬停排查。本次另删除接近末端的第二套三维反馈微调及其计时器，
只保留经过边界测试的横向有界对中，不再叠加调整同一目标；删除未使用的 CSV 导入，
修正精抓模式下“无闭爪保持”的误导性日志文案。没有删除保护检查或历史回归基线。
本地提交包括已审核的原生精抓主链、接口复用、脚本和回归；不提交 build 实验结果、ROS 工作区或子模块改动，
本次不推送远端。真实悬停正例和物体抬升/保持仍需后续验证，提交代码不代表演示验收完成。

## 28. 主线精简提交与水平悬停诊断（2026-10-21）

### 已提交的精简

本地提交 `2f650d4f4f`（`feat(arm): checkpoint minimal guarded flight-grasp workflow`）。
删除附加飞行 PD、新增精抓专用状态消息和重复三维末端微调，只保留既有飞行接管、
IK、受保护闭爪与抬升主链；保留原有回归及净空/反馈/限位保护。未推送，也未提交 ROS 或子模块。
提交前 PX4 SITL 构建、58 项 hover 脚本、19 项整臂脚本和 24 项原生精抓测试通过。

### 有界物理对照，不作为抓取正例

以下指标统一使用 100–250 s 仿真时间窗；单位分别为 mm、mm/s、度，均为中位数/90 分位。
姿态差使用日志有效 `timestamp` 最近邻配对（差 <20 ms），比较机体系重力方向，避免偏航原点差影响。

| 实验目录（`build/known_target_flight/`） | 临时变化 | 悬停位置误差 | 机体速度 | 估计/真值姿态差 |
| --- | --- | --- | --- | --- |
| `minimal_hover_20261021_01` | 原参数、原 GPS 噪声 | 17.725/33.217 | 11.804/22.301 | 3.140/3.352 |
| `hover_accel_bias_20261021_01` | 仅 `EKF2_IMU_CTRL=5` | 16.390/29.768 | 11.985/21.978 | 0.432/0.659 |
| `hover_native_d0_20261021_01` | 上一行条件再仅设水平 D=0 | 14.583/26.338 | 10.296/18.831 | 0.321/0.513 |
| `hover_gps_ideal_20261021_01` | 恢复原参数，仅关闭场景副本 GPS 噪声 | 17.861/33.718 | 11.933/22.236 | 3.246/3.463 |

- 原场景静止 IMU 水平加速度接近零，但 EKF 学到约 `[-0.36, 0.40] m/s²` 的水平加速度偏置。
  `EKF2_IMU_CTRL=5` 仅临时关闭加速度偏置估计，保留陀螺偏置与重力融合；姿态差明显减小，
  但并未使真实悬停达到门控。因此不能把“姿态估计改善”等同于“悬停问题已解决”，也不作为实机设置建议。
- 已核对原水平 D 输入是估计速度的滤波导数，并非直接使用日志 `ax/ay`。
  关闭 D 的改善不足以通过 5 mm /5 mm/s 连续 1 s 判据，未保留该参数改动。
- 关闭 GPS 噪声本身也没有解决问题；未保留该生成场景改动，生产 SDF 从未修改。
- 最后一轮 `hover_gps_bias_20261021_01` 在无 GPS 噪声副本中仅设 `EKF2_IMU_CTRL=5`，
  94.461 s 因配对反馈证据年龄 160 ms 超过原 150 ms 有效期而拒绝上下文，94.474 s 锁止退出。
  当时原始几何有效、无禁止接触，未达到稳态统计时间窗；该轮不能用来评价稳态悬停改善。
  没有延长时效或放松锁止以继续实验。

所有上述轮次均 `success=false`、`gripper_requests_sent=false`、`own_processes_stopped=true`。
没有闭爪、抬升或携物成功证据，不能以关节目标到位代替实际抓起物体。

### 最终保留的增量与后续边界

只给原运行脚本补充 `PX4_PARAM_*` 环境覆盖记录，并据此如实标记是否临时修改原生飞行参数；
不新增消息、状态机、控制器或默认参数覆盖。早期对照以各轮 ULog 的实际初始参数为准，
不能依赖旧清单恒定的 `flight_pid_modified=false` 字段判断是否调参。
全部诊断设置已撤回；下一步只核对既有速度偏置转换的反馈时序，不继续堆叠外环或盲试增益组合。

成功标准维持：实际双侧夹持、物体离开支撑、抬高至少 25 mm 并完成保持且不掉落。
指垫—支撑至少 3 mm，其余至少 16 mm，原闭爪对齐/速度/朝向和反馈保护全部保留。

## 29. 原生悬停参数短对照与 NN 适用性（2026-10-22）

### 本轮保留的代码范围

仅在 `tests/hover/run_known_target_flight.py` 的既有逐帧日志中增加原始局部位姿、
估计机体系速度、最近一次实际发送的局部位置/速度设定点及其输入时间戳。
最近命令只作审计，不能作为反馈、不能用于续租；控制变换、调度、消息和状态机均不变。
遥测帧和最近命令可能不是同一时刻，分析须使用各自记录的消息时间戳，不能视为同步测量。
58 项现有 hover 脚本回归通过。未改默认飞行 PID、生产 SDF、ROS 源码或子模块。

只读复核上一轮 `minimal_hover_20261021_01` 的 100–250 s 数据：
按真值消息时间插值并统一 NED/FRD、ENU/FLU 后，原生速度误差与由真值推算的
`位置 P × 位置误差 − 实际速度` 的水平差值，中位/90 分位约 4.1/7.8 mm/s。
这支持原有速度偏置转换大体正确，但不是零延迟或所有坐标转换均已验收的证明。
水平位置变化主要在低频，不能据此断定需要更换控制器或提高角速度环增益。

### 仅折叠臂飞行的短时参数筛选

先尝试 `hover_xy_p15_20261022_01` 精抓入口，临时设置 `MPC_XY_P=1.5`。
59.117 s 因 `FixedFaultContract` 锁止：日志为 `context=0, lease=1, hard_hover=0, binding=1`，
当时几何有效、无禁止接触、未闭爪；没有足够稳态数据评价 PID。未延长反馈有效期。

随后使用原 `--execute-flight` 入口和同一 `known_target_fixed.json`，保持机械臂折叠，
不发送机械臂运动或闭爪请求。以下均为粗到位后约 10 s 的 Gazebo 真值悬停统计，
参数已与各轮 ULog 的实际初始值核对；未列出的参数均为原值。
各轮稳态开始时刻不同、带原有传感器噪声，仅用于候选筛选，不是重复性或精抓验收。

| 目录（`build/known_target_flight/`） | 临时参数 | 平均/最大位置误差 mm | 速度中位/90 分位 mm/s |
| --- | --- | --- | --- |
| `hover_fold_baseline_20261022_01` | 原值：位置 P=0.95，速度 P/I/D=1.8/0.4/0.2 | 44.4/65.7 | 13.2/20.5 |
| `hover_fold_p15_20261022_01` | 仅位置 P=1.5 | 17.8/38.0 | 12.8/24.8 |
| `hover_fold_p15_i08_20261022_01` | 位置 P=1.5，再仅速度 I=0.8 | 17.5/34.4 | 12.4/21.2 |
| `hover_fold_p15_vp30_20261022_01` | 位置 P=1.5，恢复 I=0.4，再仅速度 P=3.0 | 25.5/36.2 | 7.8/13.0 |

位置 P=1.5 的短时位置回位有改善，但速度未同步达到原 5 mm/s 精抓门控。
积分加倍没有明显的平均位置收益，不能作为可靠改善保留。
速度 P=3.0 的本次速度较小，但位置平均误差反而较大，不能仅凭单次短窗口认定更优。
这些轮次的 `flight_goal_validated` 仅表示原 50 mm/5° 粗悬停判据，
绝不等于通过 5 mm/5 mm/s 精抓条件、悬停抓取许可或真实抓起。
四轮飞行短对照均没有机械臂/闭爪请求，五轮实验均记录自身进程正常清理；
没有永久保存上述临时参数。本轮修改尚未提交或推送。

下一小步限于原生控制链：用同一稳定观测窗较长时间复测位置 P=1.5 的候选，
再回到展开机械臂场景，核对真实位置和速度能否同时达到原精抓条件。
不把粗悬停通过或偶然一次进入门槛等同于精抓就绪，不继续增加外环、消息或状态机。

### 为什么本轮不接入 `mc_nn_control`

[PX4 官方指南](https://docs.px4.io/main/en/neural_networks/mc_neural_network_control)
将其标为实验模块，示例网络针对 X500 V2，其他机型建议进行辨识和重新训练。
它直接输出电机指令，替换传统控制链，并非原 PID 的附加补偿。

本地代码的兼容性问题比单纯启用模块更重要：

- `mc_nn_control.cpp` 注册独立飞行模式，并关闭 OFFBOARD、原位置控制和控制分配；
  当前抓取许可和飞行管理器则要求 OFFBOARD。
- `mavlink_receiver.cpp` 只在 OFFBOARD 下把现有 MAVROS 局部目标发布到 `trajectory_setpoint`。
  直接切换到 NN 模式不能继续假定当前目标输入有效。
- 本地 NN 输入只读取轨迹位置，不使用当前坐标/速度偏置转换所需的速度设定点；
  也仍依赖原估计状态，不会自行消除定位或姿态估计偏差。
- YHANG550 加臂后的惯量、推力缩放和展开工况尚未验证与示例网络匹配。

因此本轮没有编译/启用 NN，没有修改 OFFBOARD 或任何抓取保护以兼容它。
如果日后单独研究，须先做机型匹配和独立飞行验证，不作为最小抓取 demo 的快捷修复。

## 30. 全链路悬停检查：不只调整位置 P（2026-10-22）

### 模型、执行器、估计与时序的证据

- 组合模型质量约 2.726 kg，机械臂约 0.632 kg；展开后的单 link 惯量均为正定，
  主惯量满足三角不等式。这只是基本合法性检查，不是惯量、质心与实机一致性的证明。
- 前述展开臂基线的 100–250 s：电机指令范围 0.721–0.843，分配器推力/力矩均达成，
  未分配力矩最大约 9.4e-8。当前没有持续饱和证据，但不能由此宣称携物/抗扰推力余量充足。
  模型电机命令还经过 `1000 × command + 100`、最大转速 1000 的映射，不能把指令距离 1
  简单当成物理推力余量。此阶段不改电机常数、惯量或质量以获得更好的统计。
- 同窗估计姿态相对姿态目标的误差中位/90 分位为 0.125/0.207°，
  估计与真值的重力方向差却为 3.140/3.352°。姿态环跟随和姿态估计精度必须区分。
  上一轮关闭加速度偏置估计虽改善姿态差，却没解决真实悬停，不能把它视作唯一根因。
- 同窗无 EKF 故障标志、IMU 加速度剪切或位置/速度/偏航重置计数变化；
  局部里程计和真值消息年龄 90 分位约 30/18 ms。稳态未见重置不代表展开期间上下文不会过期。
- 本地 SDF 的 `send_odometry=1` 只有收到 vision 回调才会发送数据；日志没有
  `vehicle_visual_odometry`，实际 `EKF2_EV_CTRL=0`。目前没有融合外部里程计。
  原测试已显式使用仿真真值转换目标，但未将真值注入 EKF，不能等同于已有精确导航定位。
- 角速度/力矩频谱在 10–20 Hz 有较多能量，估计速度则主要是低频变化。
  不能仅凭频谱断定是旋翼、传感器噪声或闭环振荡；滤波也会增加时延，未盲目修改截止频率。
  控制分配器还对各轴做归一化，不能仅凭 SDF 与 CA 系数数值不同就认定映射错误。

### 统一较长观测窗的三轮物理对照

运行脚本增加仅飞行入口的 `--hold-seconds`，默认仍为 10 s，可选 10–60 s；
非有限值、缩短到 10 s 以下或加长机械臂/许可/抓取入口均拒绝。
本轮均观测到位后 60 s，分析统一取 50–90 s 仿真时间，避免不同到位时刻的短窗口比较。
保留原 120 s 仿真和 260 s 墙钟上限、失联/倾斜保护及所有机械臂保护。
全部保持折叠臂，没有发送机械臂运动或闭爪请求；不新增飞行外环、消息或状态机。

下表均为中位数/90 分位；每轮实际参数已与 ULog 核对，未列出的参数维持原值。

| 目录（`build/known_target_flight/`） | 临时设置 | 位置误差 mm | 真实速度 mm/s | 真实角速度 rad/s |
| --- | --- | --- | --- | --- |
| `hover_long_p15_20261022_01` | 位置 P=1.5 | 14.69/24.87 | 11.81/20.40 | 0.0155/0.0277 |
| `hover_long_p15_d0_20261022_01` | 上行条件，仅水平速度 D=0 | 13.60/24.80 | 10.07/16.93 | 0.0139/0.0249 |
| `hover_long_p15_rate07_20261022_01` | 恢复原速度 D，仅横滚/俯仰角速度 K=0.7 | 14.11/26.62 | 11.23/19.89 | 0.0123/0.0226 |

仅检查真实位置 ≤5 mm、速度 ≤5 mm/s、角速度 ≤0.02 rad/s 时，三轮最长连续时间分别为
0.208/0.167/0.167 s，均未达到原 1 s 条件；这还不是包含许可、臂反馈和净空的完整精抓判定。
三轮分配均无失败、稳态无重置、估计重力方向差仍约 3°。
去掉速度 D 略减速度波动，降低角速度环整体增益略减真实角速度，但均未根治厘米级漂移。
不能只凭某一项变好就保留参数，更不能把 `flight_goal_validated=true` 的粗悬停结果算作精抓通过。

对基准数据进一步检查，实际原生速度误差与真值推算值的水平残差中位/90 分位约
4.7/8.9 mm/s；其与单一固定输入延迟的相关性较弱。
该残差受日志采样、时间配对和估计状态影响，不能直接认定为通信延迟，也不据此增加预测补偿。

### 结论与保留范围

当前证据不支持单独提高 P、加倍积分、删除 D 或降低内环整体增益就能完成精抓悬停。
下一小步应优先核实估计状态与实际闭环反馈的一致性、精定位来源，再联合核验速度环与内环，
而不是继续无界增益组合。若改用模拟视觉/里程计融合，须作为明确的定位方案变更单独验收，
不能偷偷注入理想真值后宣称原 GPS 控制已经达到毫米级精度。

本轮仅保留测试观测时长选项与本说明；上一轮新增诊断字段继续保留。
58 项原有回归通过，另实际检查 6 个时长/入口拒绝用例及 1 个无命令预览正例。
三轮均清理自身进程，默认飞行参数未修改，临时参数未永久保存，未提交或推送。
机械臂展开、实际闭爪、离架抬升与保持成功仍未在本轮验证，现有净空和闭爪保护全部保留。

## 31. 最小模拟里程计入口：接入成功，精抓悬停尚未通过（2026-10-22）

按用户授权，仅复用已有 Gazebo vision 插件和标准 MAVLink ODOMETRY/EKF 融合，
不引入新消息、控制器、状态机或外部定位模块；生产 SDF 和默认飞行参数不修改。

### 保留的最小改动

- `run_known_target_flight.py --sim-vision`：在本次生成的场景副本加入唯一的
  `libgazebo_vision_plugin.so`，配置 50 Hz、`noiseDensity=0.001`、随机游走为零；
  已测输入频率约 41.7 Hz，消息有正的 1e-6 方差，并非声明零噪声定位。
  本模式明确使用仿真真值派生的模拟视觉，清单记录 `ekf_truth_injected=true`，不作为实机定位能力。
- 仅本次运行覆盖：`EKF2_EV_CTRL=15`、`EKF2_HGT_REF=3`、GPS/baro/range 融合关闭、
  `EKF2_EV_DELAY=0`、EV 位置/速度噪声下限均为 0.01。
  `EKF2_IMU_POS_Z=-0.02` 对应已核对的 IMU 在 base_link 上方 20 mm；模拟视觉原点为 base_link。
  不改飞行 PID，也不永久写入其他 rootfs 的参数。
- 起飞前读取已有 `vehicle_visual_odometry` 和 `estimator_status_flags`：
  检查有效数值、时效、NED/机体 FRD 帧、四项 EV 融合均启用且 GNSS/baro 未混入；
  再核对 MAVROS 局部原点与物理世界原点误差不超过 20 mm。失败则不启动飞行管理器。
- 生成场景的模型初始 pose 为世界零点，插件 ENU 经标准 MAVLink/MAVROS 变换后仍为相同 ENU 轴。
  新模式直接发送固定 ENU 位置、轨迹速度和偏航，不再每帧做真值偏置抵消；旧模式不变。
  不能用初始机体 roll/pitch 误差去旋转两个重力对齐的导航坐标系。
- `native_client.py` 修正带 `(Roll: ...)` 注释的四元数数组解析，补充原文件内回归；
  Gazebo 子模块仅将 `mavlink_odometry_t odom` 改为零初始化，避免未赋值的 reset/quality 字段随机。
  已仅重编该插件，无需修改 PX4 控制器或消息定义。

### 实验结果与淘汰的写法

所有轮次均无机械臂运动/闭爪请求，车辆和物体没有人工固定或吸附，自身进程均已清理。

- `vision_hover_20261022_01` 在起飞前因原读取器无法解析四元数注释退出，未解锁；解析问题已修复。
- `_02` 验证四项融合全程生效、EV reset_counter=0；但仍走旧的在线偏置补偿，
  50–90 s 位置/速度中位数为 10.72 mm /9.90 mm/s，最长运动门槛窗口仅 0.167 s，不通过。
- `_03` 的初始完整姿态对齐引入约 17.7 mm 平均 Y 偏差，这个写法及新增转换函数/测试均已删除，
  不保留第二套坐标转换。该轮不是有效的调参基线。

以下两轮使用最终的直接 ENU 固定目标，统计统一取 50–90 s；均为中位数/90 分位：

| 目录（`build/known_target_flight/`） | 设置 | 位置误差 mm | 真实速度 mm/s | 真实角速度 rad/s |
| --- | --- | --- | --- | --- |
| `vision_hover_20261022_04` | 模拟里程计、原始飞行 PID | 20.01/39.50 | 20.78/34.81 | 0.0163/0.0290 |
| `vision_hover_acc01_20261022_01` | 仅临时 `EKF2_ACC_NOISE=0.1` | 17.25/33.06 | 22.86/40.77 | 0.0178/0.0316 |

四项 EV 融合均全程有效，无稳态重置或分配失败；估计重力方向误差中位约 0.41°，
比先前约 3° 改善，但两轮均未形成满足 5 mm/5 mm/s/0.02 rad/s 的连续运动窗口。
原参数轮 60 s 保持的平均/最大误差为 21.89/52.64 mm，连原 50 mm 粗悬停也未完整通过；
过程噪声轮为 19.42/43.30 mm，仅粗悬停通过，不是精抓许可或抓取成功。
不保留 `EKF2_ACC_NOISE=0.1`：位置变化不足以证明收益，速度反而增大。

59 项现有回归（含新解析用例）通过；另检查场景只有一个模拟定位源、原点及 IMU 偏移匹配、
无固定锚点/物体吸附、清单如实标注模拟定位、无命令预览。尚未提交或推送。
新开关目前只接受仅飞行试验，不允许搭配机械臂/准备许可/闭爪入口；本轮未展开、闭爪或抬升。
下一步若继续，仅以这个标准定位入口整定原生飞行闭环，不再叠加真值速度补偿或扩大控制架构。

仅飞行复现（使用尚不存在的新 case 名，启动前不得已有另一套仿真）：

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
python3 /home/pcz/PX4-RoboMaster/src/modules/arm_control/tests/hover/run_known_target_flight.py \
  vision_hover_manual_01 --sim-vision --execute-flight --hold-seconds 60 \
  --target /home/pcz/PX4-RoboMaster/src/modules/arm_control/tests/hover/known_target_fixed.json
```

## 32. 最小 demo 续测：先压低悬停误差，再复用原生抓取入口（2026-10-22）

本轮只访问 Ubuntu-20.04，不改 ROS 工作区、生产 SDF、PX4 消息或状态机。
继续使用第 31 节的模拟视觉定位；控制仍是原生 PX4，未叠加外部位置/速度补偿。

### 悬停对照与保留参数

下表为 `build/known_target_flight/` 中各次隔离运行，统一统计 50–90 s，
位置/真实速度均为中位数/90 分位；全部为折叠臂、无闭爪请求。

| case | 相对原模拟视觉基线的修改 | 位置 mm | 真实速度 mm/s |
| --- | --- | --- | --- |
| `vision_hover_20261022_04` | 第 31 节基线 | 20.01/39.50 | 20.78/34.81 |
| `vision_tau005_20261022_01` | 仅 `EKF2_TAU_VEL=.05` | 15.00/30.08 | 14.87/25.36 |
| `vision_delay40_20261022_01` | 请求缓冲 40 ms，**实际被抬到 100 ms**，其余原值 | 16.82/32.10 | 16.87/27.94 |
| `vision_shortbuffer_20261022_01` | 实际缓冲 40 ms、空速延迟 0、速度输出时间 .05 s | 10.53/21.24 | 9.71/17.98 |
| `vision_short_p15_vp3_20261022_01` | 上一行加水平位置 P=1.5、速度 P=3 | 5.44/11.53 | 7.77/13.66 |
| `vision_short20_p15_vp3_20261022_01` | 上一行缓冲降至 20 ms、速度输出时间 .02 s | 5.05/9.87 | 6.00/11.03 |

实际参数均经 ULog 核对，不以环境变量请求值代替实值。
本地 `EKF2.cpp` 会比较所有编译进来的传感器延迟，包括没有融合的空速；
因此仅降低 `EKF2_DELAY_MAX` 不够，需在本次无空速的隔离场景将 `EKF2_ASP_DELAY` 置 0。
最后一轮四项 EV 融合连续有效、推力/力矩分配成功，没有稳态重置。
60 s 保持的平均/最大位置误差为 6.00/15.04 mm，仍不是抓取成功；
20 Hz 审计记录中的最长精细运动学窗口约 0.905 s，也不能代替原生许可。

保留在既有 `run_known_target_flight.py --sim-vision` 配置中的新增参数仅为：
`EKF2_ASP_DELAY=0`、`EKF2_DELAY_MAX=20`、`EKF2_TAU_VEL=.02`、
`MPC_XY_P=1.5`、`MPC_XY_VEL_P_ACC=3`。
前两项匹配当前无延迟模拟定位场景，不是可直接复制到实机的设置。
I/D、姿态/角速度增益和生产默认参数暂不改；清单正确标记 `flight_pid_modified=true`。
不改物理质量、惯性、传感器噪声或任何净空阈值。
原生 EKF 延迟/输出预测机制也见 [PX4 EKF 文档](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf)。

### 完整入口与反馈读取修正

`--sim-vision` 现在可与现有显式执行入口组合；仍必须额外指定 `--execute-flight`，
闭爪还必须明确指定 `--execute-fixed-grasp` 并通过全部原生反馈/悬停/几何判据。
没有新增流程、接口或自动放宽条件。

`vision_fixed_grasp_20261022_01` 完成飞到位、任务绑定与展开，69.390 s 开始机体系保持；
77.349 s 观察器已配对证据年龄达到 152 ms，随后原生以 Geometry 故障锁止。
当时原生反馈年龄 92 ms、任务/租约/硬悬停均有效，净空约 66.8 mm，无禁碰、无闭爪。
不能把这次记录描述为真实夹爪失联，也不能因为几何本身安全就跳过时间配对。

只在原有 `fixed_grasp_probe.py` 做两点小修正：反馈轮询 40→20 ms；
待配对样本在主观察循环取得新真值后立即复核，不等下一次 CLI 读取返回才复核。
150 ms 时效和 25 ms 配对跨度均保持不变，证据仍使用原始源时间戳，不续写时间戳。
在既有回归内补充“没有新反馈、只有新真值右括号到达”也应及时配对的检查；
原有陈旧反馈/越界/非目标接触拒绝用例继续保留。

`vision_fixed_grasp_20261022_02` 的展开后保持延续到约 227.5 s，未再出现首轮的配对年龄退出，
但没有通过展开后连续 1 s 的精细稳定条件，随后因上下文源时间超时触发 Contract 保护。
75–200 s 的真实位置/速度中位数约 4.84 mm/6.67 mm/s；高度绝对误差 90 分位约 1.00 mm，
主要余量在水平速度和角速度，而不是高度稳态偏差。没有闭爪或携物成功。
`vision_fixed_grasp_dhalf_20261022_01` 仅临时将 roll/pitch rate D 从 .003 减半到 .0015，
84.355 s 同样因上下文超时退出；短窗口不足以证明收益，**不保留 D 修改**。
两轮退出时仍无非目标碰撞、NaN 或物体吸附，任务绑定/飞行租约保持有效。

`vision_fixed_grasp_att6_half_20261022_01` 临时使用已有半速选项，并仅将 roll/pitch 姿态 P 设为 6；
仍在 72.264 s 以相同上下文时效条件退出，无闭爪。
该短窗口不证明姿态参数收益，**不保留姿态 P 修改**。
不能将问题简单归因为 WSL 性能；后续传输修正后的半速对照见下文。

在既有 `FixedGraspAdapter.cpp` 故障日志中只增加源/接收年龄及拒绝位，不改控制行为。
`vision_fixed_timing_20261022_01` 捕获 `sample_age=.105 s`、`receipt_age=.052 s`、`denial=Stale`，
明确是源样本超过 100 ms，并非接触、反馈失联或推力不足。
同一轮 MAVROS 在约 51.185/101.723 s 报时间同步重置；这是需排除的时钟配置问题，
不能仅凭日志同时出现就断言已经证明所有超时的唯一原因。

本脚本始终使用 Gazebo `/clock` 和 PX4 lockstep 的同一仿真时基。
按已安装 MAVROS 1.20.1 核对后，改用 `PASSTHROUGH` + `conn/timesync_rate=0`，
保留 FCU 源时间戳而非用 ROS 接收时间伪装新鲜；`NONE` 会走接收时刻回退，因此不用。
见 [MAVROS 时间戳实现](https://github.com/mavlink/mavros/blob/1.20.1/mavros/src/lib/uas_timesync.cpp)。
只生成本次目录内的配置/launch 副本并纳入清单哈希；ROS 安装和源码均未改。
起飞前核对实际模式/同步频率，不满足则拒绝起飞。

### 时序续测与小幅摆动对照（2026-10-23，尚未验收抓取）

`vision_fixed_sharedclock_20261022_02` 在 PASSTHROUGH 下仍发生源年龄 107 ms 退出，
因此时钟配置不是超时的唯一原因。`_hover_context_sub` 为普通 Subscription，没有 100 ms 订阅节流。
在已有观察器中记录计算/命令墙钟耗时、发送前后仿真源年龄，并密集记录同一步物理运动量；
没有新消息、状态机、控制器或重写源时间戳。

ROS 真值/里程计订阅启用 `tcp_nodelay`。抓取观察器只要求其实际使用的真值新鲜；
导航入口仍要求真值和 ROS 里程计均新鲜，原生 PX4 仍独立检查 EKF、飞行与分配状态。
新增四项回归检查：陈旧/缺失里程计不能准入导航、不能阻塞独立真值；
真值陈旧/未来时间/错误坐标系、非有限运动量仍拒绝。合计 63 项回归通过。

`vision_fixed_nodelay_20261022_01` 的典型发送前/后源年龄为 4/5 ms，
但 183.3 s 仍以 103 ms 超时退出；`vision_fixed_truthstream_20261022_01` 仍在约 66.6 s 退出。
后者局部记录显示 3–5 ms 墙钟命令内仿真时间可前进 20–31 ms，源年龄偶发升至 48–61 ms；
不能将问题归为 CLI 长时间阻塞，也不能据此宣称传输问题已彻底修复。

后续使用已有 `PX4_SIM_SPEED_FACTOR=.5`；脚本仅接受 .5..1，记录实际请求倍速并同比缩放总墙钟看门狗，
300 s 仿真时限、物理步长、全部源时效/反馈/净空和连续稳定判据不变。
`vision_fixed_vp5_half_20261023_01` 仅把水平速度 P 从 3 对比到 5，
80–180 s 的真实位置误差中位/90 分位为 3.20/5.95 mm、真实速度为 5.14/8.52 mm/s。
展开后保持至主动结束的 201 s，没有上下文故障，但最长实际精细稳定窗口仍不足 1 s，未闭爪。

`vision_fixed_vp5_dhalf_20261023_01` 在相同条件下只把 roll/pitch rate D 减半；
ULog 确认实际值 .0015，最长完整准备窗口约 .77 s，收益不明显，**不保留 D 修改**。
`vision_fixed_vp5_gyro10_20261023_01` 恢复原 D，只将 gyro cutoff 从 40 对比到 10 Hz；
真实角速度反而增大，**不保留滤波修改**。这些试验均未发送闭爪请求，主动停止后清理了自建进程。

`vision_fixed_vp5_abias_20261023_01` 对照 `EKF2_ABIAS_INIT=.01`（原值 .2）。
依据是当前实际 IMU 插件启动偏置与随机游走默认均为 0（模型未覆盖），
而上一轮 EKF 学到约 -0.076/+0.069 m/s² 水平偏置。
估计水平偏置降至约 -.015/+.018 m/s²，但真实运动未明显改善，**不保留该修改**。
传感器噪声、偏置学习开关、物理模型均未改。
不能将这些模拟定位/先验参数直接用于实机，也不能把粗悬停或展开成功称为抓取成功。

`vision_fixed_vp5_att6_20261023_01` 恢复其他参数，只对照姿态 P=6；
在 68.992 s 主动停止以切换审核后的接管策略，窗口太短，不能判断其收益，**不保留姿态 P 修改**。

### 已批准的最小接管调整

用户明确批准：只将额外的展开后速度/角速度接管条件改为“实际 TCP 补偿需求≤20 mm/s”。
仍需真实机体位置误差≤5 mm、臂跟踪误差≤3 mm、关节速度≤.05 rad/s，
原生悬停许可/几何/反馈均有效且连续保持 1 s；源间隙超过 100 ms 仍重置该窗口。
补偿需求沿用同一步真值计算 `|v_body + omega_world × (p_tcp - p_body)|`，
从原生 30 mm/s 能力中保留至少 10 mm/s；不是简单放宽机体速度后任意接近。
没有改任何原生接近速度/关节步长、16/3 mm 净空、25 ms 配对、150 ms 反馈时效、
100 ms 上下文时效，也没有改闭爪的 3 mm 相对对齐、5 mm/s 相对速度、5°朝向保护。
在现有文件新增接管边界与非有限值检查，合计 64 项回归通过。

模拟视觉 profile 保留有实测收益的水平速度 P=5（位置 P=1.5）；
所有姿态/角速度增益、gyro cutoff 和启动偏置先验恢复生产原值。
`vision_fixed_capacity_20261023_01` 用该配置和半速运行批准后的完整入口：
43.564 s 原生接近启动，70.847 s 通过展开接管，113.159 s 到第一接近点（真实误差 1.14 mm）。
213.525 s 插入时固定夹指提前接触方块，中心仍差 10.10 mm，观察器立即退出，**未请求闭爪**。
接触前逐帧侧向间隙从 1.30 mm 降至 -1.34 mm，说明窄侧余量被实际跟踪偏差吃掉，
不是夹持力不足；该次没有进入夹持力或抬起验证，不能称为抓取成功。

只在现有 `fixed_demo_plan.py` 增加一个居中插入途经点：
原第一接近点 → 张开夹爪中部 → 侧移 4 mm 到原来的最终抓取点。
原最终抓取点、方块/支撑尺寸质量、关节限位和全部净空/闭爪保护均不改。
没有新增入口、消息或状态机；任务 ID 继续绑定实际途经点，旧计划不能复用。
离线检查覆盖新路线、完整闭爪行程及原 30 mm 抬起，最小采样余量约 4.97 mm，
只证明数值几何可行，不代表连续安全或实际抓取已成功；64 项回归通过。
`vision_fixed_center_20261023_01` 已实测该修正：没有触发上一轮的提前夹指接触退出，
但在居中点附近发生多次原生净空余量暂停，例如可用预算 9.90 mm、待消化关节位移预算 11.60 mm。
214.101 s 原生因上下文源/接收年龄均达到 108 ms，以 Contract/Stale 退出；
同段观察器已配对证据年龄也达到 163 ms，未以当前几何安全为由忽略任何时效条件。
退出时真实中心仍差约 9.66 mm，原生末端目标误差约 8 mm，不能把“已发到途经点”当成真实到位。
共 54 次原有接近/恢复请求、**0 次闭爪请求、0 次真实抬起**；自建进程均已清理。
该路线数值可行且本轮未出现同类提前接触，但并未证明重复成功或完成最小 demo。

本阶段验收边界：批准后的展开接管已通过；固定目标飞行、展开和第一接近点已实测。
剩余只集中在末端动态跟踪余量及上下文发布的偶发时间缺口，不再保留无收益的 PID/滤波/先验对照。
最终仍必须以真实双侧夹持、物体离开支撑至少 25 mm 且保持不掉落验收；
不能用放宽净空、续写源时间戳、吸附物体或仅观察已发送目标来代替。
所有修改当前未提交/推送；ROS 工作区没有修改，没有新增消息或状态机。

## 33. 两处小修正与居中点等待简化（2026-10-23）

不再叠加飞行 PID、滤波或新入口。本轮修改均在现有试验代码内：

| 文件 | 修改 | 不变项 |
| --- | --- | --- |
| `tests/hover/fixed_demo_plan.py` | 肩部俯仰 `iMax` .18→.25 N·m，消除明显重力静差 | P/I/D 增益、最大力矩 .8 N·m、生产 SDF 不变 |
| `tests/hover/run_known_target_flight.py` | 同一不可变 ROS 真值帧只转换/校验一次，缓存数组只读 | 每次读取仍核对原始源年龄、连接及坐标系；新帧重新校验，不续写时间戳 |
| `tests/hover/fixed_grasp_probe.py` | 用户批准仅将新增居中途经点的额外等待设为 0 s | 原第一接近点等待、居中点全部即时距离/速度条件、最终 2 mm/s 接近及原生闭爪连续 1 s 判定不变 |

旧记录显示主要关节静差集中在 shoulder_lift，约 .012 rad；
在旧积分限幅上仍需 P 分量补约 .035 N·m，故先补足积分余量，不提高最大力矩。
`vision_fixed_integral025_20261023_01` 已运行至 283.549 s 的原生 240 s 总时限，
本轮没有再次因上下文源超时锁止，但仍不能宣称长期时序已彻底解决。
该轮停在居中途经点：107 s 等待期间最长连续满足 .96 s，**未闭爪、未抬起**。

同一路线 150–205 s 的观察区间，真实世界 TCP 跟踪误差中位/90 分位：
旧 `vision_fixed_center_20261023_01` 为 5.70/9.00 mm，新积分余量轮为 1.78/3.83 mm；
肩部目标减实测均值约 .01375→.00044 rad。
两轮推进进度并不完全相同，以上是运行表现对照，不是相同稳态姿态的严格 A/B 结论。
离线同帧读取耗时约 80→13 μs，也不等于已证明所有实时抖动消失。

新增回归覆盖缓存不续期、缓存数组只读、新消息重新校验，
以及只有三点路线中的新增居中点可取消额外等待；合计 67 项回归通过。
`vision_fixed_centerpass_20261023_01` 已通过新增居中点：
164.138 s 第一接近点、172.454 s 居中点、185.348 s 进入原生 Align。
该入口请求只开启对齐判定，夹爪仍维持张开 .3 rad；没有进入实际 Close。
最终对齐阶段相对速度超过 5 mm/s 的样本约 80%，最长距离/速度/朝向/机体位置同时达标约 .285 s，
原生连续 1 s 闭爪条件未通过。约 224.8 s 后原生日志记录源年龄 118 ms 的 Contract/Stale 故障。
该次目录缺少最终 result.json，恢复工作时已无其运行进程；不能伪记为正常完成或成功清理报告。

### 剩余动态偏差的小幅补偿复测

积分余量修正后，150–260 s 真实世界 TCP 静差均值已接近零，
但仍有随机体运动变化的动态滞后。对既有日志做离线线性对照，等效滞后约 86 ms；
这只用于选择保守试验值，不是闭环有效性的证明。
在同一 `fixed_grasp_probe.py` 的世界目标转换中加入 80 ms 速度超前，向量长度最多 2 mm：
`command_world = goal_world - clamp_norm(.08 * (v_body + omega × lever), .002)`。
只在展开稳定接管后启用，抓取/抬起目标使用同一个偏移，原 30 mm 抬起量严格不变。
不预测或重写反馈/时间戳，实际距离/相对速度/净空仍按原始真值计算；
原生关节步长、30 mm/s 能力、100 ms 时效、16/3 mm 净空及连续 1 s 闭爪条件均不改。
新增零速度、平移/旋转方向、2 mm 限幅、抬起量和非法输入回归，合计 68 项通过。
`vision_fixed_lead80_20261023_01` 232.955 s 请求原生 Align，283.549 s 达到总时限后退出，
自建进程清理完成，没有进入实际 Close 或抬起。
Align 段相对速度中位/90 分位为 8.31/16.51 mm/s（无超前轮 8.32/16.99），
对齐误差约 1.33/3.44 mm（无超前轮 1.60/3.65）；不同运行窗口不能视作严格 A/B。
位置有所改善但尚无相对速度收益，关节速度快变增加，不能据此宣称超前补偿已经有效。

进一步核对现有控制链：精抓原来以 20 ms 间隔发送位置阶梯，
Gazebo JointController 则在 1 ms 物理步内计算位置误差 PID，可能放大位置变化的瞬时响应。
只将 `FixedGraspAdapter.cpp` 展开后的轨迹输出改为 4 ms，与既有 250 Hz 仿真传输匹配；
不新增控制器、插值模块或消息，继续使用原有加速度有界关节轨迹。
IK 仍至少间隔 16 ms，Cartesian/joint/jaw 每秒预算分别仍为 30 mm/.2 rad/.2 rad；
原生闭爪连续 1 s 判定不变。`FixedGrasp.hpp` 只同步注释，无保护逻辑修改。
新增 250 Hz 调用累计预算及闭爪计时回归；26 项独立 C++ 逻辑用例通过，SITL 编译通过。
`vision_fixed_trajectory4_20261023_01`（仍含速度超前）在 82.326/89.263 s 通过第一/居中点，
95.035 s 首次请求 Align，推进明显更快，但尚无重复性证明。
之后保守的待跟踪关节位移预算多次超过可用净空，引起暂停；
103.867 s 固定夹指提前接触，真实对齐误差 8.03 mm，观察器按原规则退出，自建进程清理完成。
没有进入实际 Close；清理前原生日志还记录了上下文时效故障，不能将它遗漏为正常抓取结束。
不采用减小抓取深度或缩小安全间隙来回避该问题。
80 ms 超前没有显示相对速度收益，且增大了快变关节响应，故已删除其代码和专用回归，
不留下无收益的配置开关。当前仅保留原始同帧位姿补偿，Python 回归仍为原有 67 项。
`vision_fixed_trajectory4_only_20261023_01` 在 94.909/100.910 s 通过两个途经点，
112.930 s 请求 Align；119.846 s 因固定夹指提前接触而退出（对齐 4.09 mm、朝向 5.034°），
仍未实际闭爪；自建进程已清理。退出附近亦有源时效故障，不记为成功。
最终仍须真实夹住、离开支撑至少 25 mm 并保持不掉落才算成功。

### 按最小 demo 收束等待条件

用户再次明确允许适度放松、避免复杂化。本次不增加消息、状态机或预测补偿：

- 空爪的两个途经点均取消额外连续 1 s 等待，原有即时距离/速度/反馈/净空条件保留。
- 仅 SITL 固定点抓取的 Align 连续到位等待由 1 s 缩短为 .2 s，
  相对速度由 5 放到 10 mm/s（这段等待中相对行程最多约 2 mm）；
  仍每周期核对原 3 mm 对齐、5°朝向、5 mm 机体误差和完整闭爪扫掠净空。
  不满足会重置等待，闭爪期间不满足仍停止继续收紧；双侧夹持确认、抬起和保持验收不变。
  通用 `HoverPermit` 的 5 mm/s 判据不变，这一调整仅用于已隔离的固定点 SITL 入口。
- 试验副本中 shoulder_lift/elbow_flex/wrist_flex 的 P 分别为 6/5/2.4；
  D 与最大力矩仍取原值，生产 SDF 未改。这是关节跟踪试验设置，不是飞行 PID 修改。

`vision_fixed_tracking2_20261023_01` 的首途经点因旧 1 s 等待拖至 217.652 s，
229.339 s 进入 Align；272.752 s 按本轮简化方向主动停止，进程清理完成，没有实际 Close。
前段世界 TCP 跟踪中位/90 分位约 1.22/2.42 mm，并不等于已抓取成功。
仅取消途经等待的 `vision_fixed_passage_20261023_01` 在 78.034/85.160 s 通过两个途经点，
约 93.846 s 首次请求 Align，103.847 s 固定夹指提前接触退出（真实对齐 7.75 mm），
没有实际闭爪，进程清理完成。缩短最终等待的回归共 27 项 C++ 逻辑用例通过，SITL 编译通过；
Python 回归 67 项通过。单元测试不能替代真实夹持与离开支撑的物理验收。

仅缩短 Align 等待的 `vision_fixed_shortalign_20261023_01` 仍在约 103.85 s 触发
观察器的“提前夹指接触”额外退出，未实际闭爪。随后删除该观察器单独的一票否决：
精抓阶段原先已批准指定夹指与指定方块接触，其他配对仍由真实碰撞分类与净空保护拒绝，
展开阶段仍不能使用该接触例外。新增回归后 Python 68 项通过。
`vision_fixed_contactpass_20261023_01` 在 94.263 s、对齐约 .80 mm 时请求闭爪入口，
但原 5 mm/s 条件最长连续仅约 .166 s；夹爪仍张开。
103.846 s 单侧接触将方块推离支撑，104.471 s 原生 Geometry 保护退出，进程清理完成。
这说明不能把“门限不放行”误认为已经稳定完成抓取，也不能靠删除退出规则宣称成功。
在同一轮物理日志中，10 mm/s 条件可于约 91.534 s 满足 .2 s 连续窗口，
因此仅按此证据小幅调整固定点入口，而非继续增改飞行 PID。
最终门限的 28 项 C++ 逻辑用例及原通用悬停 26 项用例通过，SITL 重编译通过。

`vision_fixed_practical_20261023_01` 已实际进入 Close（约 88.242 s），
93.263 s 获得可持续的双侧接触，94.867 s 进入 Lift，说明无需再放宽到位或飞行判据。
但这轮没有抓起：物体仍在支撑上滑移，随后掉落，96.263 s Lost 保护退出，进程清理完成。
抬起初期双侧法向力约 .043/.031 N，测得竖直接触合力最高约 .087 N，
随后下降至约 .031 N，低于 10 g 方块约 .098 N 的重力；双侧接触本身不等于足够夹持力。
当时夹爪目标 .04 rad、实测约 .049–.052 rad，P=.25，未充分利用 .01 N·m 力矩上限。
因此只将试验副本的夹爪 P 改为 .75，并由同一个 `GRIPPER_P` 常量写入模型和试验记录；
原生产 SDF、D、.01 N·m 上限、夹爪速度、方块质量/摩擦及净空均不改。
没有增加新控制器或以粘连/外力代替夹持。Python 68 项回归通过。

### 精抓接触时不再反复追逐机体微动

P=.75 的 `vision_fixed_grip075_20261023_01` 未进入机械臂运动：
原生定时任务在约 22.209 s 后停更，关节反馈与飞行回路仍更新；
159.988 s 主动结束并清理。这不是夹持成功记录，也没有证据归因为夹爪 P。
相同参数重启的 `_02` 已闭爪，但仅形成单侧接触，方块被顶转约 30°，
103.702 s 触发 Geometry 退出并完成清理，没有抬起。提高 P 本身尚不能解决这一轮的问题。

为收束最小 demo，只在既有观察脚本内改目标保持方式：
原生 Align 接受后锁存当时的臂基座目标，接触、闭爪和抬起阶段不再逐帧追逐机体抖动。
飞行器仍自由悬停，没有锚定；真实世界距离、速度、朝向、碰撞、反馈时效判据仍按新真值帧核对。
抬起向量仍由当前姿态将世界向上 30 mm 转入臂基座，未改抬起量或成功标准。
记录中明确区分 `BASE_HOLD_DURING_GRASP` 与原世界 TCP 补偿，便于审阅。
未增加消息、控制器或状态机；68 项 Python 回归通过。
补充闭爪途中超限停止收紧的用例后，独立固定抓取 C++ 回归为 29 项，通用悬停为 26 项。

`vision_fixed_basehold_20261023_01` 在 94.492 s 实际进入 Lift，
99.297 s 记录实际抬起 22.54 mm、双侧法向力约 .074/.069 N，支撑接触为 0，
物体仍被夹住，并非再次滑落。但是源年龄偶发达到 102 ms，超过原 100 ms 门限，
原生 Contract/Stale 退出；本轮自建进程已清理。
尚未达到 25 mm 与长时间保持的完整成功条件，不能将此记录标记为 demo 成功。
下一轮保持同样参数，不放宽时效；减少运行中整份日志读取，避免给测试增加非必要负载。

相同参数的 `vision_fixed_basehold_20261023_02` 在 93.284 s 因净空指标 15.77 mm
低于原 16 mm 门限退出，自建进程已清理；该轮没有完成抬起。
因此“锁存臂基座目标”仍是待完善的试验方案，不能宣称已稳定复现或已完成最小 demo。
当前保留 100 ms 时效与 16/3 mm 净空，不再新增控制器、消息或状态机。
剩余集中在固定抓取位与支撑的实际余量、以及自由悬停接触的可重复性，
不是再扩大 PID/规划参数试凑。当前修改未提交/推送，ROS 工作区未改。

## 34. 最小 demo：只修正固定抓取几何余量（2026-10-23）

独立回放 `_basehold_02` 的最后实测关节和臂基座姿态，确认最坏配对是
`moving_jaw_collision ↔ cube_support::link::collision`，不是飞机或臂身。
93.263 s 完整闭爪行程最坏 q=.04 的保守间距约 2.985 mm，
扣除原 3 mm 门限与约 .220 mm 角采样预留，净裕量约 −.235 mm。
插件输出是“配对净裕量＋16 mm”，重建 15.765 mm 与记录 15.768 mm 相差约 3 μm。
当时实际夹爪仍张开约 .2926 rad，所以是完整闭合行程证书不足，不能说已经发生穿模。

当前文件的支撑曾偏至 −2 mm，但最坏实测帧重力投影会超出支撑 X 边缘约 .1 mm，
继续侧移容易先把方块弄翻。因此本轮只做两处固定布局修正：

- 支撑局部 X 恢复 −1.5 mm，保留承载余量，不缩小或取消支撑。
- 抓取参考由 `[.0045,-.000218,-.102]` 改为 `[.0045,-.000218,-.103]` m，
  即轴向让开 1 mm；夹指与方块轴向覆盖仍有 8 mm，侧向张开间隙仍为 2.5 mm。

按旧最坏帧同样位移回放，完整闭爪间距约 4.046 mm，扣原门限与采样预留后余量约 +.827 mm。
名义路线、闭爪和抬起 IK 预检通过；固定物体世界位置仍为 (.6,0,1.5) m，姿态不变。
测试只调整覆盖边界到“至少 8 mm”并让横向死区测试使用当前参考，未放宽控制死区。
68 项 Python 回归通过。没有新增消息、状态机、控制器或飞行 PID 修改，
100 ms 源时效与 16/3 mm 净空仍保留；离线通过不是动态抓取成功证明。

`vision_fixed_axial103_20261023_01` 在 84.054 s 退出，没有完成抓取。
回放确认 20.063–83.034 s 方块静止且受支撑，83.117 s 开始仅固定指接触，
83.306/83.638/83.701 s 位移超过 .5/2/5 mm，随后翻落。
83.264 s 的闭爪入口请求未被接受，仍为 Approach；最后反馈配对误差是快速掉落后的表现。

因此本轮第二个物理修正仅将支撑总宽由 4 增至 8 mm（另外两维仍24×4 mm）。
当前布局名义完整闭爪间距约 8.005 mm；旧最坏帧回放扣3 mm门限及采样预留后仍余约 .211 mm。
方块初始重力投影距支撑X边缘从 .713 增至2.713 mm，先改善轻触时的承载稳定性，
而非再放宽到位或碰撞条件。
实际使用的 ROS `super_px4_bridge/src/coordinate_truth_plugin.cpp` 仅同步允许明确的8×24×4 mm
支撑尺寸（兼容原4×24×4 mm），仍按实际包围盒计算净空；没有新增消息或跳过碰撞检查。
`so101_coordinate_truth` 定向重编译和68项 Python回归通过。ROS源文件本轮有这两行相关修改，
未提交、未推送；该修改不自动同步到另一个源码镜像。

`vision_fixed_support8_20261023_01` 在65.013 s、仍在第一接近段即因源年龄101 ms退出，
当时净空约83 mm、无夹指接触，不能据此评价新支撑抓取表现；自建进程已清理。
启动器仅将已有 `PX4_SIM_SPEED_FACTOR` 可用下限从 .5 调至 .25，墙钟看门狗原本已随该系数缩放。
下一轮按 .25 执行，仿真时间内的轨迹速度、源时效100 ms、物理步长、净空与验收标准均不改。

## 35. 最小 demo：只增加工具提取余量（2026-10-24）

复核 `vision_fixed_basehold_20261023_01` 的实际反馈：工具距原30 mm抬起目标仅约 .94 mm，
方块实际抬起22.54 mm，双侧接触约 .990，支撑接触为0，竖直接触合力约 .097 N。
这不是 IK 没走到或已掉落；指间相对转动/偏移消耗了约7 mm的有效抬升。

本轮仅将既有工具向上提取行程从30改为40 mm，不新增消息、状态机、规划或控制器。
Python 预检、实时目标变换及原生距离合同同步修改，避免新旧行程混用；
物体实际至少抬起25 mm、支撑接触为0、双侧夹持与持续保持的验收均不变。
30 mm/s轨迹速度、力矩上限、100 ms源时效以及16/3 mm净空也未放宽。

按当前8×24×4 mm支撑布局重新离线检查40 mm提取：
16个原位置IK样点，每点53个完整夹爪行程采样，方块静止及随工具刚性移动两种情形均通过；
末端误差约 .096 mm，全部在原工作空间与关节限位内，最小关节边距 .01676 rad。
最坏名义配对仍是起点 moving_jaw/support：间距8.005 mm，扣3 mm门限与.220 mm采样预留，
余量4.786 mm；非夹指最坏 gripper_body/target 间距约42 mm，仍要求16 mm。
这些离线结果不证明动态夹持成功。31项独立C++固定抓取逻辑用例、68项Python回归通过，SITL编译通过。

首次 .25 速30 mm试验 `vision_fixed_support8slow_20261024_01` 在44.195 s主动结束，
随后统一改用40 mm，不将该中止记录作为抓取结果。
`vision_fixed_lift40_20261024_01` 在约82 s进入Align，但旧逻辑此时已锁存臂基座目标，
之后机体小幅漂移导致约6 mm对齐偏差，夹爪一直维持张开 .3 rad。
121.034 s方块被碰离支撑，原生Geometry退出，自建进程已清理；没有实际闭爪或抬起。
所以这一轮未检验到40 mm提取效果，不能将失败归因为提取行程。

最小修复只将观察器的锁存时机从Align(2)后移到原生真正Close(3)：
Align仍跟踪固定的世界目标，实际开始闭爪后再保持相对机体的臂目标。
不增加新的流程或状态；原有到位、时效、关节及净空判据均保持。
对应记录的补偿模式同步更正，68项Python回归通过。

`vision_fixed_closehold_20261024_01` 又出现原生状态停在启动约22.165 s、
而仿真与关节反馈继续更新的现象，在机械臂尚未运动时主动结束；不将此计为抓取试验。
源码审查未发现状态读取接口直接修改HRT/lockstep的路径，原因尚未定位，不为此改飞控平台。
试验观察器仅新增“原生状态落后当前仿真时间超过 .5 s 即退出”，避免长时间等待失效的许可；
不会自动重启或发送额外控制命令。新用例后69项Python回归通过。

### 首次端到端最小 demo 通过

`vision_fixed_closehold_20261024_02` 完整结果为 success=true、grasp_validated=true：

| 仿真时间 | 实测过程 |
| --- | --- |
| 43.503 s | 飞行到位并获许可，原生机械臂接近开始 |
| 62.747 s | 展开后稳定核对通过 |
| 71.305 / 78.805 s | 两个空爪接近点通过 |
| 85.596 s | 请求原生对齐/闭爪入口，当时实际对齐约1.70 mm |
| 91.409 s | 已形成真实双侧夹持，开始物理提取 |
| 105.221 s | 原生Done后再通过独立5 s保持观察 |

独立保持窗口累计5020个接触物理步，双侧夹持比例1.0、支撑接触比例0.0，
窗口内实际抬起最低39.593 mm，末帧约41.071 mm，未掉落；原生末态Done、fault=0。
双侧法向力约 .106/.106 N。飞机未固定、物体未粘连，飞行指令源只有既有测试管理器。
本次未发生保护锁止，末态净空指标约40.73 mm；自建PX4/Gazebo/ROS进程已清理。

独立流式复核3216条记录：Align全部使用WORLD_TCP，Close至Done全部使用基座保持，
模式错配0；NativeDone始于100.221 s，独立窗口100.221–105.221 s共241帧，
窗口内禁碰与无效几何均为0，按真实物体高度复算的最低升高与结果完全一致。
另只读核查当前Linux进程，本次PX4/Gazebo/ROS/MAVROS及启动器残留均为0。

本轮验证范围仅为已知固定目标的“飞到悬停位→展开/接近→闭爪→提取→原地保持”。
这是单次端到端物理成功，不等于多次重复、任意姿态、收臂回收或携物飞行已验收。
仍未提交或推送；没有新增消息/状态机/控制器，也未继续改飞行PID。
本轮说明继续集中在本文件，不新建多份文档。

复现采用现有有界启动器（确保其他PX4/Gazebo/MAVROS实例已退出，使用新case名）：

```bash
cd /home/pcz/PX4-RoboMaster
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
PX4_SIM_SPEED_FACTOR=.25 python3 src/modules/arm_control/tests/hover/run_known_target_flight.py \
  fixed_demo_replay_01 --sim-vision --execute-flight --execute-fixed-grasp \
  --target src/modules/arm_control/tests/hover/known_target_fixed.json
```

结果位于 `build/known_target_flight/<case>/result.json`；要同时看到 success=true 与
fixed_grasp.grasp_validated=true，不能仅凭gripper_requests_sent或已进入Align判断成功。

## 36. 复用现有导航链的多障碍抓取扩充（2026-10-25）

当前结果（2026-10-26）：`super_obstacle_grasp_20261026_07`完整通过。
SUPER实际绕障到悬停位后，现有位置IK完成接近、闭爪和提取；物体未粘连、飞机未固定。
原生Done后独立保持5.000 s，最低真实抬升30.714 mm，双侧夹持99.62%、支撑接触0。
障碍最小包络间距186.943 mm、绕行偏移最大1.844 m，无NaN/禁碰，末态fault=0。

最终只保留现有链所需改动：雷达/三障碍导航与终点接管、空载展开和接近提速、
实时物体中心对齐、Close世界补偿至Verify后基座保持，以及更垂直的固定抓取模板。
最后模板三点整体平移`[-.10,0,+.03] m`，末端`(.16,.015,.39)`，
夹具倾角14.14°、支撑8×24×4 mm且横向偏移0；P1.5、扭矩上限.01 Nm。
最后插入2 mm/s、16/3 mm净空及原生闭爪/失夹保护保留，未新增PX4控制消息或状态机。
以下按实验顺序保留失败定位依据，不能把中间试验参数当作最终配置。

本轮沿用 `super_px4_bridge/scripts/start_super_sim.sh` 对应的 Gazebo/VLP16 链，
以及 `super_navigation.launch` 中的 sensor adapter、SUPER 和 MAVROS bridge。
有界演示启动器单独持有飞行管理器，导航 launch 增加 `start_flight_manager` 开关避免重复启动。
未增加 PX4 消息、状态或控制器，生产模型不改；沿用SIM_VISION_PARAMS，
最终未保留试验性的PID切换，MPC_XY_P仍为1.5。

- `run_known_target_flight.py --super-obstacle` 保留雷达，生成三处有真实碰撞的静态障碍：
  墙中心 `(1.5,0,1.3)`、尺寸 `.3×1.6×2.6 m`；两根柱中心
  `(3.4,2.6,1.3)`、`(3.4,-2.6,1.3)`，尺寸均 `.7×.7×2.6 m`。
  原固定方块平移到 `(5,0,1.5) m`，夹持姿态及局部 IK 路线沿用已验证模板。
- SUPER 先规划到抓取悬停位前方 `.6 m` 的停靠点；原管理器完成
  `TRACK_SUPER → SETTLE_HOVER → FINAL_HOVER` 后，才接受现有 known-target 租约进行精确接近。
  `allow_terminal_known_handover` 默认关闭，仅本演示显式开启。
- 实验副本 `super_demo.yaml` 将规划速度/加速度收紧到 `.5 m/s、.6 m/s²`，
  时间代价 `exp_traj/penna_t=1e5`。原 `px4_sim.yaml` 不修改，原优化可行性判据不放宽。
  轨迹与机体实际位置均来自运行中的传感器和真值记录，没有用预设绕行点冒充规划。
- 机械臂原生关节目标限速 `.2→.4 rad/s`，展开周期 `100→50 ms`，
  笛卡尔目标限速 `30→50 mm/s`；世界系空爪路线 `10→20 mm/s`，最后插入保留原 `2 mm/s`。
  夹爪仍 `.2 rad/s`，原力矩、关节限位、源时效、16/3 mm 净空及闭爪许可均保留。
- truth 插件将明确命名的 `demo_obstacle_*` 静态碰撞体纳入接近和精抓净空计算，
  仍拒绝未分类模型；飞行观察另核对半径 `.45 m` 机体包络到障碍的实际间距。

核对保存场景还发现一处历史记录差异：第34/35节虽描述8 mm支撑，旧生成器仍实际输出4 mm。
旧成功结果应以其保存的 `scene.world` 为准。首次按离线尺寸生成8 mm支撑的`_04`在提取时触发净空退出；
`_05`恢复4 mm后在闭爪轻触时方块翻离支撑，未完成提取。该阶段试用8×24×4 mm支撑，
并将 `OPEN_REFERENCE.z` 从−.103改为−.104 m，轴向让开1 mm，名义夹指覆盖仍为7 mm。
`SUPPORT_HALF` 与生成器同源，使实际碰撞、离线计划和插件尺寸一致；
不将旧4 mm物理结果冒称为8 mm验证。原16/3 mm门限不改。

当前20中的 SUPER 目录没有找到 FAST-LIO 启动文件，但已在 `rog_real_bags/R1、R2` 找到
此前 Jetson/Mid360 FAST-LIO 的真实输出归档。导航 launch 提供 `use_sensor_adapter`、
`map_cloud_topic` 和 `map_odom_topic` 开关，可复用其已注册点云与里程计输入；
外部 FAST-LIO 的数值原点/方向必须与规划坐标一致，不能仅凭重命名话题认为坐标已对齐。
本轮运行的地图来源明确记录为 VLP16+MAVROS 里程计适配，精抓目标继续用 Gazebo 真值。

阶段验证：73项 Python、31项独立 C++ 固定抓取逻辑、两项原飞行接管回归通过，
SITL、truth插件和飞行管理器均编译通过。
`super_obstacle_grasp_20261025_02` 在规划阶段失败，未展开/闭爪；
`_03` 降低时间代价后生成有效轨迹，但以1.5 m/s、2 m/s²转弯时触发15°倾角保护，已退出并清理。
因此该轮只收紧演示的规划速度/加速度，未放宽倾角保护，也未据此调飞行PID。
`_04`、`_05`均完成实际绕障和终点接管；`_06`在原紧凑通道中规划指令断流，
安全悬停退出，机械臂未动。最终将两柱移到上述更宽的两侧位置，保留正面墙阻断直线，
避免为演示引入狭窄回头通道。

按`_04`末帧保持原支撑世界位姿回放，轴向让位1 mm后moving_jaw/support间距
2.788→3.733 mm，扣3 mm门限与约.22 mm采样预留后余量+.513 mm。
这是单帧离线回放，只说明修正方向合理，动态成败仍以完整实验记录为准。

`_07`在宽场景完成导航，但停在Align，夹爪目标一直为.3 rad，没有进入Close。
97.055 s请求时机体误差3.89 mm，97.513 s超过6 mm，103 s约11.9 mm；
额外5 mm机体门槛持续阻止闭合，103.867 s先发生固定指单侧接触，104 s方块被推离支撑。
因此不继续盲移支撑，也不改变碰撞门槛：仅将SITL精抓闭爪接管的机体位置门槛5→10 mm，
保留实际夹爪入口对齐≤3 mm、相对速度≤10 mm/s、朝向≤5°、完整夹爪行程净空与反馈时效。
展开/世界路线推进原5 mm条件保持。最后插入恢复2 mm/s，空载段仍提速。
对应10 mm边界拒绝测试、31项原生固定抓取及71项Python回归通过，SITL重编译通过。

`_08`在最后接近仍出现机体暂停，未闭爪。90–93 s误差分解为：
X由.15升至10.24 mm，Z为约3–5 mm；主要是机械臂移动时的水平偏差。
障碍demo因此仅试调`MPC_XY_P: 1.5→2.0`，位于当前源码声明的0..2范围；
其余MPC/MC增益不变，不改生产参数文件。

`_09`在机械臂未动的远场绕障阶段触发精抓局部坐标核对，未检验到上述增益的抓取效果。
精抓观察器现改为在同一目标任务的`terminal_hold`生效后开始配对核对；
导航阶段继续保留源有效性、倾角与障碍包络检查。精抓开始后坐标误差仍严格按原3 mm退出，
租约撤销后也不停止核对。新回归覆盖错误任务、未终点保持时不启用，以及开启后仍拒绝错误帧。
72项Python通过；外部FAST-LIO模式launch节点展开及启动脚本语法检查通过。

`_10`全程较高增益在绕墙阶段触发机体包络保护，未进入抓取。
`_11`尝试导航1.5、终点保持后2.0，仍未进入闭爪。101 s时实际夹爪对齐误差约.5 mm，
观察器却继续等待虚拟路线末端约3.4 mm余量耗尽；持续微调随后推动方块。
因此最终撤销PID切换与临时参数命令权限，保留原1.5。
只简化闭爪衔接：实际对齐≤3 mm、朝向≤5°及原生close_entry_ready满足后，
以实时物体中心校正入口残差、冻结世界目标并请求原生闭爪，不再等待虚拟路线余量归零。
世界路线推进仍用5 mm机体条件，闭爪接管与原生已批准的10 mm条件一致；
请求等待期间停止额外插入，被拒绝后可恢复原接近。物理净空、反馈和原生闭爪保护不改。
修改后72项Python回归通过。`_12`中断于绕障阶段，无最终结果，不计为通过。

2026-10-26收尾审查另修复两处兼容性：启动脚本先保存命令行参数并清空位置参数，
加载ROS环境后才启用未定义变量检查，避免`--gui`被catkin误读或ROS_DISTRO未定义导致退出；
飞行管理器保留原known-goal模式全阶段失联退出行为，终点接管时沿用同一规则。
脚本实际`--help`、31项原生抓取回归、原悬停/接近回归及两项ROS接管测试通过，管理器编译通过。

`super_obstacle_grasp_20261026_01`完成SUPER绕障和终点接管，实际机体包络距障碍
最近153.863 mm，直线绕行偏移最大2.823 m。111.826 s请求闭爪，119.242 s开始提取，
122.451 s进入Hold，126.659 s仍双侧接触且离开支撑、真实提取约39.9 mm；
随后方块滚转滑脱，127.263 s由Lost保护退出，未到Done，不计成功。
法向力由约.132/.129 N降到.091/.087 N，夹爪实测角逐渐接近.04目标，
P误差减小使保持力下降；同时提取过程中相对滚转增至约19°，不能只归因于增益。
下一轮只将演示夹爪P从.75调到1.5，扭矩上限仍.01 Nm、限位及摩擦不变，
先验证能否延长饱和保持区，不增加新控制或缩短物理验收窗口。

`_02`未进入Close，未检验到新的P：99.409 s请求时对齐2.815 mm，
将目标重置为实测末端引入1.56 mm命令变化，随后对齐升至4.335 mm，原生闭爪拒绝推进。
104.2 s后单侧接触推翻方块，104.783 s配对坐标核对退出。
回放还发现原名义目标比实时居中目标偏约3.4 mm，所以不继续等待旧目标或冻结实测点。
最终仅将冻结目标改为`实际TCP + 实际夹爪旋转 × (物体局部中心−抓取参考)`，
在已满足≤3 mm的入口内消除残差，再由原生IK/限速/净空执行；无新增修正循环或状态。
新增旋转坐标下消除残差的回归，合计73项Python通过。

`_03`已实际进入Close并建立双侧夹持，102.846 s进入Lift；但被完整夹爪行程的
moving_jaw/support净空拦住，107.846 s退出，未实际提取，不计成功。
独立回放精确匹配该帧：8 mm支撑间距2.350 mm，扣3 mm门限与.218 mm采样预留后为负；
恢复旧成功实验的4×24×4 mm支撑，同一帧间距增至3.908 mm，剩余+.690 mm。
该轮只恢复4 mm支撑，仍保持实际/离线生成尺寸一致，−.104轴向参考与P1.5均不再改；
不修改净空门槛、不跳过完整行程检查。

`_04`恢复窄支撑后未再被净空阻挡，但125.846 s实际滑脱，由Lost退出。
独立复核找到更早的原因：111.827 s开始Close时姿态误差仅.45°；
此时脚本立即切成基座保持，机体漂移不再由世界TCP补偿，112.221 s尚未接触就偏离5.306 mm。
113.221 s移动指先单侧接触，115.827 s方块已滚转约30°才建立双侧夹持；
117.034 s进入Lift时已经歪夹，且P1.5始终受.01 Nm饱和限制，因此不再盲目增P或只改提取速度。
下一轮只把现有WORLD_TCP→BASE_HOLD切换点由Close改到Lift：Close和Verify仍固定世界目标，
夹稳并进入提取才随机体保持。没有增加状态、消息或控制器，原净空/闭爪/失夹保护均保留。

`_05`证实Close期间世界补偿使方块夹正：102.201 s进入Verify时仅3.23°。
但继续世界补偿到Lift过迟：104.012 s夹指竖向合力变成向下−.0635 N，
位置误差约5.5 mm、净空反复暂停，105.013/108.013 s已绕夹持轴X滚转17.14/28.30°。
127.638 s仍接触支撑、实际提取仅1.39 mm，由净空保护退出，不计成功。
因此最终只把切换点从Lift提前到Verify：Close仍WORLD_TCP，首次原生双侧夹持确认后
Verify/Lift/Hold/Done使用BASE_HOLD。P、速度、物体、支撑与全部保护均不再改。

`_06`仍在Close阶段先遇到完整行程净空：104.033 s已双侧、对齐误差2.50 mm，
但余量比门限少约.04 mm，未能进入Verify，不计通过。该倾斜模板的几何余量不足以稳定复现。
因此不再在边界反复改时序，改用同一展开/位置IK的更垂直固定模板：三点整体平移
`[-.10,0,+.03] m`，末端为`(.16,.015,.39)`；支撑8×24×4 mm，横向偏移0。
目标物世界位置仍为(5,0,1.5)，现有坐标转换自动计算相应机体悬停位，未增加规划算法。
离线夹具倾角14.144°、完整行程最小距8.434 mm、路线夹指/支撑最小距15.533 mm，
展开最小距84.486 mm；初始与闭合−3 mm后的重心投影均位于支撑内，40 mm提取IK误差.094 mm。
保留−.104抓取参考、P1.5、.01 Nm力矩上限及16/3 mm保护；物理成败仍以新一轮完整实测为准。

`_07`实测与独立复核：36.083 s开始TRACK_SUPER，55.483 s停靠，58.483 s经FINAL_HOVER
转TRACK_KNOWN；67.159 s开始原生接近，92.201 s请求闭爪，97.013 s开始提取，
108.430 s原生Done，113.430 s通过独立保持。独立窗口241条记录、5021个接触物理步，
按实测pickup_z=1.499990042 m复算，最低物体z=1.530704073 m，实际提取30.714031 mm。
双侧加权比例.996215893、支撑0、禁碰0、无效几何0，metric净空最低35.890 mm，
所有原生fault=0；Close全部WORLD，Verify到Done全部BASE，模式错配0。
唯一飞行发布者`/known_flight_manager_probe`，自建PX4/Gazebo/ROS进程清理完毕。

本次验收针对已知固定方块的抓起与原地保持：物体仍有滚转，保持窗口内相对夹爪姿态
约37.34–37.56°，不能称姿态精确抓取；未验收长时间重复或携物飞行。
`hold_max_fold_error_rad`在展开后仍沿用fold名称统计，不表示本轮应保持折叠姿态；
本轮展开已获原生许可。地图仍由VLP16+MAVROS适配输入ROG-Map/SUPER，未启动FAST-LIO。

有界面的复现入口（先退出其他PX4/Gazebo/MAVROS实例）：

```bash
bash /home/pcz/super_ws/src/super_px4_bridge/scripts/start_obstacle_grasp_demo.sh --gui
```

脚本默认 `.25` 仿真速率、生成唯一case名，并沿用原启动器的进程归属与退出清理。
结果与轨迹保存在 `build/known_target_flight/<case>/`；成功仍必须同时满足绕障验证、
实际双侧夹持、物体离开支撑至少25 mm并独立保持5 s。

## 37. 纯 MID360S 感知抓取实现路线 V1（2026-10-26；S1结果见第38节）

本节是实施与实验路线，不表示所有功能已经实现；分阶段结果从第38节记录，保持第36节已验收演示不变。
后续每次只完成一个可独立验收的小阶段，修改和实验说明继续集中在本文。

### 37.1 范围、起点与最小链路

- 目标感知只使用MID360S激光点云，不加入相机；定位可使用雷达自带IMU。
- 雷达先保持顶部居中正装。第一版只处理静止、已知几何模型的刚性目标，不要求任意物体通用识别。
- 目标采用较大可观测主体与适配爪口的窄抓取部位，二者属于同一刚体。
  可先试外包络约8–12 cm的非对称主体与约20 mm抓取柄，尺寸经点云与爪口实验后定版。
  外形放大不按体积同步增加质量；从现有10 g载荷附近的轻质结构起步，质量、重心、惯性需另验收。
- 只使用Ubuntu-20.04；复用既有飞行管理、任务绑定、悬停许可、位置IK和抓取逻辑，不先增加PX4消息或状态机。
- 已有基线仍是VLP16+MAVROS适配导航、已知真值目标抓取；不是MID360S感知闭环，也未验收携物飞行。

MID360S官方规格为水平360°、垂直−7°至52°、典型10 Hz、20万点/s。
官方明确近场细小目标存在检测与精度限制，点率不等于毫米级位姿精度，最终以实测为准。
依据：[MID360S规格](https://www.livoxtech.com/mid-360s/specs)。

```text
MID360S点云 + 内置IMU
    ├─ LIO定位/去畸变 → ROG-Map → SUPER避障导航
    └─ 高分辨率目标点云 → 分割 → 已知模型配准 → 目标位姿与质量判定
                                                   ↓
              观察位测量 → 锁定同一目标任务 → 抓取悬停位 → 现有IK接近
                                                   ↓
                            闭爪反馈 → 提取/保持 → 携物与返航
```

导航地图与目标感知点云分开使用；当前0.3 m导航栅格不能直接用于小目标精定位。

### 37.2 分阶段实现与验收

| 阶段 | 本阶段只实现什么 | 验收与输出 |
|---|---|---|
| S1 几何与观察位预检 | 候选目标外形、顶部正装视场、观察位/抓取位关系；离线计算 | 目标参数与可见距离/角度表，选一个观察位及备用位；检查爪口、工作空间和净空可行性 |
| S2 MID360S静态点云链 | 核对或建立Gazebo Classic扫描近似、点时间/IMU接口，静态记录原始点云并对照实测bag | 不同距离/视角的目标点数、覆盖率、时间差及噪声；说明模型近似项，不能只把VLP16改名 |
| S3 分割与物体位姿 | ROI、支撑面剔除、聚类、已知模型粗配准与精配准；先在雷达坐标输出 | 位姿、采样时刻、坐标系、有效性和质量；统计平移/旋转误差、有效率/误检及耗时，不只检查点云均值或配准残差 |
| S4 LIO定位独立验收 | MID360S/IMU接入兼容FAST-LIO，核对外参/原点，单独替换sim-vision | LIO定位、去畸变点云与SUPER绕障通过；无重复定位源，坐标重置可识别 |
| S5 感知驱动无闭爪接近 | 感知快照绑定既有任务；观察位重新定位，再飞到抓取悬停位并IK接近 | 同一目标/任务一致，误差预算满足夹取容差；错目标、过期、丢失和不可达均能拒绝；先不闭爪 |
| S6 新目标物理抓取 | 大主体/窄柄的目标几何与接触分类，接回既有闭爪、提取和保持 | 实际夹到指定部位，物体离开支撑≥25 mm并保持5 s；先单次，再预先固定一组重复案例统计，记录滚转/滑移 |
| S7 携物基线 | 从原地保持、低速直线到转弯/绕障，再到返航降落，逐项增加 | 位置/姿态误差、物体保持、任务时间与失败原因；前一项不能重复通过就不扩复杂任务 |
| S8 实机分级复现 | 静态雷达与机械臂台架，再空载飞行、无闭爪接近、单次抓取及低速携物 | 实测标定、真实可得反馈和硬件入口分别验收；仿真通过不直接等同实机通过 |

每阶段完成后在本文追加修改文件、实际条件、结果与失败原因；点云包和实验日志放独立数据/build目录。
大主体尺寸、安装倾角、控制参数不同时改动；只有上一项明确通过或找出失败原因，才决定下一项修改。

### 37.3 S1实施边界

1. 提出一个非对称刚体目标及抓取坐标，记录主体/夹柄尺寸、候选质量和模型坐标；先不承诺载荷能力。
2. 以现有安装变换为仿真初值，按MID360S外形/传感器原点核对；实机外参另行标定。
   离线检查观察位与抓取位，不启动飞行或发关节/闭爪命令。
3. 初始几何采样可取水平距离0.8/1.5/3/5 m、雷达高于目标0/0.1/0.3/0.6 m，
   检查目标整体及可配准面的仰角与遮挡风险。这些是测试点，不是预先承诺的工作距离。
4. 产出一份结果表和目标参数，选一个观察位及一个备用位，确认新目标没有明显爪口/工作空间/净空冲突。
5. 几何可见只用于预筛，不代表有足够回波或定位精度；通过后再做S2静态点云实验。
   小文件只在实施时按需增加到ROS侧perception目录，不提前铺设大量节点、消息或目录。

视场判定先用完整位姿把目标/可配准面变换到雷达坐标：`p_L=inverse(T_M_L)*p_M`，
再计算`beta=atan2(p_L.z, hypot(p_L.x,p_L.y))`与−7°至52°比较，计入机体roll/pitch和安装外参。
仅在雷达水平时，可近似用世界高度差与水平距计算仰角。
按第36节最后抓取位作这一水平近似估算：雷达z约2.106 m、目标z1.5 m、水平距.163 m，仰角约−75°；
相同高度差需约4.93 m水平距才到−7°边界。因此观察时可先降低雷达相对目标高度，再改变位置/高度去抓取。
增大目标增加可见面积，不改变视场。第一版静止目标允许受控的末段短时出视场，
但定位漂移、目标估计年龄/不确定度须有界；超过夹取容差应重新观察/退出。
若目标会移动，不能直接复用静态目标快照。只有实验证明必须持续观测且正装无法满足，才评估倾角和支架，
计算质心/惯量变化并重新标定外参，不先因倾角假设改飞控PID。

### 37.4 位姿、坐标与误差预算

控制地图坐标记M，雷达L、机体B、物体O、机械臂基座A、抓取坐标G。已知刚体模型定义T_O_G：

```text
T_M_O = T_M_L(t_sample) * T_L_O
T_M_G = T_M_O * T_O_G
T_M_A(t) = T_M_B(t) * T_B_A
T_A_G(t) = inverse(T_M_A(t)) * T_M_G
```

目标中心不等于抓取位姿，抓取坐标也不是当前IK工具原点；两者间需校核既有工具/夹持中心标定。
当前IK是位置IK，第一轮只接受已验证抓取方向，不声称任意六自由度末端姿态均可执行。
对称目标应报告等价姿态或不可观测方向。可复用[PCL配准](https://pointclouds.org/documentation/tutorials/registration_api.html)，
初始搜索区域与模型先验需注明，不以真值位置裁剪精确ROI或初始化ICP。

先按距离、视角、积累时长分组统计误差中位数/P95、有效率、误检与处理时间。
0.2/0.5/1 s可作积累时长对照，运动时须去畸变并使用对应采样时刻的变换。
正式接抓取前，由爪口、抓取柄与净空共同确定容差，计入目标平移误差、旋转引起的抓取点位移、
外参/工具误差、悬停及机械臂跟踪误差。配准残差小不能替代真实位姿误差验收。
若总误差无法满足，应改观察位/目标几何/估计方法，不靠放宽保护制造通过。
现有3 mm/5°是仿真精抓入口条件，不是MID360S实机已达到的感知精度。
锁定任务后保留原始观测时刻；控制心跳或重复发布不能把旧感知伪装为新观测。

### 37.5 去真值与实机迁移的两道验收

定位源替换：当前`--sim-vision`用Gazebo里程计进入EKF，S4需关闭该源后接LIO，不能混用来源而不区分。
MID360S使用[官方driver2](https://github.com/Livox-SDK/livox_ros_driver2)，本地FAST-LIO仍依赖driver1的CustomMsg。
先核对字段、MD5、点时间单位和兼容版本再选择最小适配，不先假设需要新建桥接节点。
旧`blind:0.5`与0.5 m滤波配置不能直接用于目标精定位；导航和目标感知可使用不同分辨率。
LIO原点/重置、传感器与机体外参须明确转换，不能只改frame_id。
既有适配器用最新odom且超时仅告警，运动感知必须补按采样时刻配对/插值与无效拒绝。

目标/抓取反馈替换：Gazebo真值保留作独立评分，不进入检测器、配准器或正式抓取目标。
当前精抓还依赖物体位置、净空、接触、提取高度等仿真数据，后续要区分感知几何、关节/电流反馈与评分通道。
仿真真实接触可检验物理成败，实机没有对应传感器时不能宣称具有同样反馈能力。
电流/负载需标定空载摩擦、末端限位与夹持情况；电流升高本身不能充分证明抓住物体。
可结合夹爪角度、持续负载和提取后的雷达再观察，设计实机可实现的确认；不确定时明确报告不确定。

大主体+窄柄需独立几何配置：现有fineGeometry硬检查20 mm单box，不能直接缩放SDF；
接触统计也须避免把夹到主体当成功，仅夹指接触指定抓取部位受允许，其余主体/支撑/障碍继续分类保护。
重新验证完整夹爪行程、组合质量/惯性，保留16/3 mm净空基线；不删除碰撞组件或伪造能力位。
当前实机明确拒绝fixed_grasp/context，SITL授权还要求SIM_GROUND_TRUTH。
迁移必须先实现真实可得输入与对应能力判定，再分级复用既有控制核心，不能直接删除宏开放演示入口。

### 37.6 论文与扰动研究

暂定主线：纯雷达物体位姿估计→可视性约束观察/接近→可达抓取坐标转换→飞行/抓取交接与反馈。
明确已知几何、静止目标、模型先验、对称性、出视场假设，以及目前尚未解决的姿态精抓问题。
FAST-LIO、ROG-Map、SUPER、PCL作为引用基础；可视性/可达性策略、交接规则和扰动补偿是否构成贡献，
由对照与消融实验支撑，不把系统接通直接称为算法创新。

从各阶段持续记录定位误差、处理延迟、悬停位置/姿态误差、最小净空、任务时间、
成功率、提取高度及滑移/滚转。真值/感知目标、时间配对、观察位策略可逐项对照；
重复次数与场景分组事先固定，报告成功数/总数及失败原因，不只选择成功样本。
仿真采样/噪声近似要用设备来源明确的MID360S实测bag校核。
旧R2归档是Livox历史数据，可验证处理链，不能自动认定为MID360S性能验收数据；
旧VLP16演示也不能作为MID360S精度或纯雷达自主抓取证据。
S7取得空载/携物基线后，再单独研究机械臂运动与载荷扰动到PX4的反馈/补偿，
在相同任务下做补偿开关与空载/携物对照，先仿真再复现实机，不在感知初期同时引入新控制器。

S1几何、S2扫描近似和S3a静态分割/位姿基线已逐项实施，结果见第38–40节。
MID360S实测校准、LIO定位替换及感知驱动抓取尚未验收。

## 38. S1：顶部正装安装尺寸与参数化几何预检（2026-10-27）

本轮只执行离线计算，无飞行/关节/闭爪命令；原VLP16组合模型、PID、PX4消息、状态机和运行抓取保护均未修改。
结果是下一步静态点云实验的候选，不是点云精度、制造安装、动力学稳定或实机载荷验收。

### 38.1 修改文件与参数入口

实际ROS包 `/home/pcz/super_ws/src/super_px4_bridge` 新增三份文件，并同步到Git镜像 `/home/pcz/so101-px4-ros/super_px4_bridge`：

- `config/perception/mid360s_s1.yaml`：统一输入，长度m、质量kg、角度**度**。
- `scripts/perception/check_installation.py`：读配置，复用既有离线位置IK分支与SDF碰撞盒，生成JSON/CSV；不含控制发布接口。
- `tests/test_mid360s_s1.py`：坐标、遮挡及错误输入/几何拒绝验证。

不新增ROS节点/消息，也不把配置自动写回当前飞行与抓取控制。这样测量值可先离线核对，再按S2/S5/S6逐项接入。
检查器复用PX4仓库的 `fixed_demo_plan.py` 和现有 `grasp_frame_audit.yaml`，不是独立原生IK执行器；
当前工作空间/关节范围不扩展。报告保存所用配置及几何来源文件哈希，便于复查同一组尺寸。

### 38.2 选定的第一版尺寸

| 参数 | 候选值 | 来源/含义 |
|---|---|---|
| 雷达外壳 | 65×65×60 mm，265 g | MID360S官方规格 |
| 安装平板 | 120×120×3 mm，铝材密度2700 kg/m³ | 本轮候选；板质量116.64 g |
| 安装方式 | 顶部居中、正装、无俯仰倾角 | 底面中心F的轴按官方O-XYZ定义；雷达+X朝机头，侧连接器朝后 |
| 机体B到雷达底面F | xyz=(0,0,113) mm，rpy=(0,0,0)° | 仿真候选；B为机体base_link，不是地面或机架最高点 |
| 底面F到雷达输出坐标L | xyz=(0,0,47) mm，rpy=(0,0,0)° | 官方尺寸图第16页；坐标方向见第19页 |
| 合成B到L | xyz=(0,0,160) mm | `T_B_L=T_B_F*T_F_L`；没有把60 mm外壳高度当原点高度 |
| 机体B到机械臂A | xyz=(0,0,-50) mm，rpy=(180,0,0)° | 复用现有倒挂安装 |
| 目标主体 | 120×80×60 mm | O中中心(-20,0,-90) mm，供雷达观察 |
| 非对称凸块key | 30×25×20 mm | O中中心(25,25,-50) mm，帮助区分主体姿态；不承诺实际配准精度 |
| 抓取柄handle | 20×20×70 mm | O中中心(0,0,-25) mm，截面保持已验证20 mm；下端与主体连接 |
| 目标参考O/抓取G | 柄顶以下10 mm处，`T_O_G=I` | 此点不是大主体中心，也不是IK工具原点 |
| 夹持参考在gripper_link中 | (4.5,-0.218,-104) mm | 复用已验收开爪接近参考；作为可测量参数保留 |
| 目标总质量 | 暂定10 g轻质/空心候选 | 仅记录目标，不按实体体积推算，也未验收新目标质心/惯性/夹持力矩 |

安装依据：[MID360S官方规格](https://www.livoxtech.com/mid-360s/specs)、
[官方手册](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/UM/20260601/Livox_Mid-360s_User_Manual_en.pdf)第15–16、19页。
官方建议平金属安装面，厚度至少3 mm，并留散热空间；本板按扣除65×65 mm外壳投影后，
顶面剩余10175 mm²，满足配置的10000 mm²预检；侧向余量27.5 mm。
板与雷达合计381.64 g，不含立柱、线束、螺钉；未据此宣称完整无人机重心或载荷通过。
四个底部M3安装孔最大深度5 mm；孔位须用官方CAD及实物核对，本轮没有生成可直接加工的孔图。
平板与现有中央机体碰撞盒间隙78 mm，支撑立柱/线缆/电气设备/桨盘与散热仍需实际装配复核。

### 38.3 观察位与抓取位结果

目标O位置沿用(5,0,1.5) m，朝向沿用第36节已验证位置IK分支。
观察高度差以**大主体中心**为参考；该中心世界z=1.412546 m，不是抓取点z=1.5 m。
视场计算完整使用安装旋转、机体姿态、杆臂和目标旋转；检查主体/key的面网格、相互遮挡、
中央机体/安装板/折叠臂碰撞盒遮挡。每面5×5是几何采样，绝不是预期雷达点数或实测覆盖率。

| 位置 | 相对主体水平距 | 雷达高于主体中心 | 机体B世界位置m | ±3° roll/pitch九组采样最小视场余量 |
|---|---:|---:|---|---:|
| 首选观察位 | 1.5 m | 0 m | (3.520020,-0.021236,1.252546) | 2.3613° |
| 备用观察位 | 3.0 m | 0.1 m | (2.020020,-0.021236,1.352546) | 1.2658° |
| 抓取悬停位 | 约0.163 m，按抓取点计 | 雷达高出抓取点约0.606 m | (4.837375,0.002825,1.945582) | 主体仰角约-78.41°至-70.98°，不可见 |

首选观察位机体yaw=0°；雷达z=1.412546 m。备用雷达z=1.512546 m。
两观察位均超过额外1°视场边界余量，主体与key在全部采样姿态均有几何可见面。
它们不是飞行目标授权：本轮未检查场景障碍路径、连续运动包络或真实扫描回波。
16组距离/高度组合中5组通过这项几何候选检查；完整范围表见输出CSV。
0.8 m同高位在姿态采样后只剩0.8847°余量，未达到配置1°要求，不选作首轮观察位。

目标尺寸没有消除近端下视盲区。下一阶段仍按“先观察静止目标，再转入已知抓取位”验证；
连续可观测、位姿老化和实际误差预算分别验收，不把当前观察位直接接成自动飞行命令。

新目标在253组展开/接近姿态、53组夹爪行程采样及既有抬起IK采样中：

- 主体/key到机械臂最近保守距离38.1087 mm，最差为活动夹指到key，超过16 mm要求22.1087 mm。
- 开爪路径中夹指到handle最小2.5001 mm；只在精抓段允许两指对handle受控接触的几何例外。
- 其他部位/非目标部位仍16 mm，精抓夹指到支撑仍3 mm。该候选未更新运行保护或接触分类。
- 名义TCP在A中约(0.159995,0.015080,0.390000) m；使用原位置IK分支，目标方向残差约0.000003°。
- 上述均为离散盒几何和离线近似IK，不是连续扫掠、动态接触、原生IK新目标或真实抓取验收。

### 38.4 实机测量值如何输入

坐标约定：M为局部ENU，B为前/左/上FLU；`T_parent_child`把child坐标点变到parent。
不能把PX4的NED/FRD数值原样填入。实机先明确B与飞控IMU的关系，传感器外参须对应实际点云输出坐标。

1. 复制候选YAML另存为测量配置，保留原基线。修改机体到安装面的 `lidar.body_from_base_xyz_rpy_deg`；
   47 mm内部原点偏移保留官方名义值，实际安装yaw/roll/pitch填测量值。
2. 若已标定完整 `T_B_L`，直接填 `lidar.body_from_lidar_override_xyz_rpy_deg` 六个数，
   替代前述两段组合；不能同时在驱动和本检查中重复施加外参。物理底面/平板参数仍需按实物填写，用于装配盒检查。
3. 机械臂安装填 `vehicle.body_from_arm_xyz_rpy_deg`；工具/夹持参考填 `vehicle.grasp_reference_gripper_m`。
   目标主体/key/柄的尺寸和相对位置、`target.object_from_grasp_xyz_rpy_deg` 都在同一文件。
   改成非20 mm截面或使抓取参考离开柄部会拒绝当前分支，不能因此认为任意尺寸已经验证可抓。
4. 再运行离线预检，检查报告；不会自动启动仿真、修改模型或发送控制命令。

```bash
python3 /home/pcz/super_ws/src/super_px4_bridge/scripts/perception/check_installation.py \
  --config /home/pcz/super_ws/src/super_px4_bridge/config/perception/mid360s_s1.yaml \
  --px4-root /home/pcz/PX4-RoboMaster \
  --output /home/pcz/PX4-RoboMaster/build/mid360s_s1/baseline

python3 /home/pcz/super_ws/src/super_px4_bridge/tests/test_mid360s_s1.py \
  /home/pcz/PX4-RoboMaster -v
```

结果目录包含 `report.json`、`observations.csv`、`profile_used.yaml`，后者固定本次实际输入。
`--config`可以指向自己的测量文件，`--output`另选目录以保留不同装配对照。

### 38.5 验证与下一步

11项离线测试通过：底面/原点组合与标定覆盖、全姿态逆变换、倾角影响、旋转盒/端点遮挡、
视场内但完全遮挡拒绝、原分支候选/近端盲区、测量臂安装改变悬停位置、主体过大/错误目标朝向拒绝、
安装板过小/装配重叠拒绝、未验证柄宽及柄外抓取参考拒绝，以及NaN/负尺寸/反向视场等输入拒绝。
本轮S1离线预检通过；代码已提交至ROS仓库 `92cd89ffeb32429e0f770d3f7aa3af40d7b0de1a`，未推送。

下一小步只做S2静态MID360S扫描与点云记录：由这份尺寸配置建立静态目标及雷达近似，
核对1.5 m首选位与3 m备用位回波，再决定点云积累时长和分割参数；暂不增加飞行状态机或闭爪联调。

## 39. S2：静态扫描、点时间/IMU与回波记录（2026-10-27）

S2的静态仿真链已跑通：使用S1几何生成静态目标、支撑、中央机体/折叠臂/雷达安装盒和背景场景，
在独立ROS/Gazebo端口记录原始点云、理想静态IMU和时钟。没有启动PX4、飞行管理或抓取控制。
MID360S实测扫描/噪声校准仍待设备来源明确的bag，当前结果不能标为MID360S实机性能验收。

### 39.1 实现文件与扫描近似

实际包 `/home/pcz/super_ws/src/super_px4_bridge`，源码同时同步到ROS Git镜像：

- `src/perception/mid360s_static_plugin.cpp`：Gazebo Classic/ODE任意方向射线，按仿真时间生成每点回波和200 Hz理想静态IMU。
- `scripts/perception/run_static_scan.py`：复用S1参数/已有离线IK结果生成静态场景，依次录制和清理各实验，仅用源仿真时间判断采样结束。
- `scripts/perception/scan_metrics.py`：坐标、逐点时间、IMU括号配对、表面回波/覆盖统计，使用独立真值评分。
- `config/perception/mid360s_static.yaml`：扫描速率、采样窗口、噪声敏感性、随机种子、端口及数据来源/哈希。
- `models/perception/mid360.csv.gz` 与 `LICENSE.livox`：保留官方扫描CSV原始内容与MIT许可，gzip用于减小仓库体积。
- `tests/test_mid360s_static.py`：8项纯离线边界测试；CMake注册 `mid360s_static_math`。
- `CMakeLists.txt/package.xml`：仅新增静态插件目标、zlib及记录bag/服务所需依赖；没有新消息定义。

扫描轨迹来自 [Livox官方仿真仓库](https://github.com/Livox-SDK/livox_laser_simulation/tree/1cce1073633a062b92e30243a4c2920e45551bb5)，
固定提交 `1cce1073633a062b92e30243a4c2920e45551bb5`，文件为 `scan_mode/mid360.csv`。
解压内容SHA256为 `aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a`。

该文件是**MID360轨迹代理**，不是MID360S实测轨迹。800000行第一列虽标Time/s，实际为1..800000序号，
本实现不把它当物理秒；按声明的200000次射线/s排列，轨迹4 s后循环，帧率10 Hz。
仰角按官方方向换算为 `90°-Zenith`，直接裁掉不在S1配置−7°..52°中的方向，不重映射或向目标补点。
整个周期1742/800000条方向被裁掉，占0.21775%；200 k是裁剪前发射采样率，实际返回点数取决于场景。
没有把VLP16改名，也没有对目标区域增加采样密度。

场景全部刚性静止，光学窗口仅可视；低于原点的39.5 mm下壳保留碰撞盒，避免用完整外壳盒把雷达原点包在内部，
导致射线误击自身。环境中的目标、支撑、机体和背景都有真实ODE射线遮挡。
距离噪声为固定种子的独立高斯σ=0/10/20 mm敏感性实验；没有标定反射率、材质漏检、光斑、角误差或真实MID360S扫描相位。
intensity=100、tag=0及line序号均为模拟占位，不代表实机反射率或已验证通道。
插件只供本节静态场景使用，加载时拒绝动态模型；传感器移动或仿真时钟回退时停止发布，不承诺运动去畸变或自动恢复。

### 39.2 点云、IMU与真值接口

| 话题/字段 | 本轮约定 |
|---|---|
| `/mid360s_sim/points` | 原始PointCloud2，坐标为L=`mid360s_lidar`，不含目标类别/物体位姿 |
| x/y/z/intensity/tag/line/timestamp | 前七字段沿用driver2 PointXYZRTLT的字段名、类型和偏移；额外添加offset_time |
| PointCloud2.header.stamp | 本帧计划开始时刻，ROS秒/纳秒，来源为Gazebo仿真时钟 |
| timestamp（FLOAT64） | **点的绝对时间，纳秒**；不是秒，也不是帧内偏移 |
| offset_time（UINT32） | 相对帧header时刻的纳秒偏移；`point_ns=header_ns+offset_time` |
| `/mid360s_sim/imu` | 200 Hz，sensor_msgs/Imu；L中理想静态比力m/s²、角速度rad/s，姿态未提供（orientation_covariance[0]=-1） |
| `/mid360s_s2/ray_truth` | 仅评分：无噪声交点、碰撞部位标签、采样序号与点时间；不得作为S3感知或S4定位输入 |
| `/clock` | 记录仿真时钟；墙上时间仅用于启动等待/超时退出，不决定实验积累时长 |

接口参照 [driver2固定版本的lddc.cpp](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/lddc.cpp)。
PointCloud2.timestamp直接写入内部点纳秒时间；CustomMsg则使用timebase与相对offset_time。
本轮先复核并修正了最初把PointCloud2.timestamp写成秒的实现，最终验收数据使用上述纳秒约定。
这是字段/时间约定对照，尚未生成driver2 CustomMsg或接入本地仍依赖driver1的FAST-LIO。
IMU采用与L重合的理想静态坐标，真实内部IMU外参、噪声、偏置和运动响应留到S4/实测标定。

### 39.3 实验矩阵与最终结果

最终数据：`/home/pcz/PX4-RoboMaster/build/mid360s_s2/static_matrix_20261027_03`。
每组先跳过0.2 s，再按源时间取2.0 s/20帧，分别统计不重叠0.2/0.5/1.0 s窗口。
原模型姿态与安装参数均继承S1；所有实验共用同一几何和固定随机种子19。

| 实验 | 距离/条件 | 2秒目标总回波 | 结果 |
|---|---|---:|---|
| primary / primary_repeat | 1.5 m，σ=0 | 各373 | 通过，两次采样点字节哈希一致 |
| backup | 3 m，雷达高出主体0.1 m，σ=0 | 120 | 通过，但小特征较稀疏 |
| primary_noise / primary_noise2 | 1.5 m，σ=10/20 mm | 各373 | 接口/噪声注入通过，未进行物体位姿估计 |
| backup_noise / backup_noise2 | 3 m，σ=10/20 mm | 各120 | 接口/噪声注入通过，未进行物体位姿估计 |
| occluded | 1.5 m，中间放400 mm遮挡盒 | 0 | 符合预期，目标被遮挡 |
| blind | S1抓取悬停位 | 0 | 符合预期，目标位于顶部正装雷达下视盲区 |

全部9组静态用例通过。帧间隔100 ms、IMU间隔5 ms、点的时间单位/顺序与对应帧一致；
所有点都有IMU源时间括号，最大单侧间隔5 ms。清洁交点回到场景表面的最大残差约1.15e-7 m，
这只是射线坐标变换一致性，**不是毫米级感知精度证据**。
σ=10/20 mm时绝对测距扰动P95分别约19.60/39.20 mm，符合声明的敏感性设置；未据此校准真实雷达噪声。

| 窗口 | 1.5 m主体平均点数 | 1.5 m key平均点数 | 3 m主体平均点数 | 3 m key平均点数 |
|---|---:|---:|---:|---:|
| 0.2 s | 30.1 | 2.7 | 10.0 | 0.9 |
| 0.5 s | 75.25 | 6.75 | 25.0 | 2.25 |
| 1.0 s | 150.5 | 13.5 | 50.0 | 4.5 |

3 m的0.2/0.5 s窗口出现过key零回波；因此几何可见并不保证有足够点约束姿态。
1.5 m、1 s作为S3首轮静止目标积累条件，0.5 s作对照；3 m保留为较远观察/鲁棒性对照，暂不认定为精定位位。
这是当前MID360代理的实验选择，实机MID360S需重新测量后决定。

覆盖率采用5 mm面网格，在独立真值中统计“有交点的可见网格/几何可见网格”。
1 s时1.5 m主体/key平均覆盖约17.82%/16.22%，3 m约6.80%/6.08%。
该指标用无噪声交点评分，故本轮改变距离噪声不改变这项采样覆盖；不能用它证明带噪分割/配准准确或完整表面覆盖。

### 39.4 历史实测bag接口对照与尚未验收项

只读对照：`/home/pcz/rog_real_bags/R2/r2_formal_04_20260724_103919/r2_formal_04_0.bag`。
其归档未给出可核实MID360S设备来源，不能作为MID360S性能/新目标抓取精度数据。
只检查前约3 s：29帧driver2 CustomMsg，点数19392–20640/帧，29帧内offset_time均非单调，
28帧最大偏移略超过100 ms；IMU源时间中位间隔5.036 ms，P95约5.896 ms。
header与timebase差值为−173..146 ns，与绝对时间转换精度需一并考虑，不能要求它们字节一致。

结论：模拟字段与真实CustomMsg时间单位可对应，但本节“严格有序固定100 ms”检查只用于理想静态仿真。
S4接真实数据时，需处理实际帧跨度、点时序、header/timebase选择与精度，再作对应时刻配对/去畸变；不能直接套本节校验器。
本轮未替换sim-vision、未启动LIO、未做目标分割/位姿估计；硬件入口与抓取能力判定均未放开。

### 39.5 复现、验收与下一小步

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
cmake --build /home/pcz/super_ws/build --target so101_mid360s_static -- -j2

# --output必须选新目录；省略--cases会执行完整9组矩阵。
python3 /home/pcz/super_ws/src/super_px4_bridge/scripts/perception/run_static_scan.py \
  --px4-root /home/pcz/PX4-RoboMaster \
  --cases primary backup primary_noise backup_noise occluded blind \
  --output /home/pcz/PX4-RoboMaster/build/mid360s_s2/my_static_run

python3 /home/pcz/super_ws/src/super_px4_bridge/tests/test_mid360s_static.py -v
cd /home/pcz/super_ws/build
ctest -R mid360s_static_math --output-on-failure
```

加 `--gui` 可查看静态Gazebo场景；每组结束自动关闭本轮拥有的Gazebo进程。
`--real-bag 路径`只读生成接口对照，不改变实测数据。ROS端口11411、Gazebo端口11355，已占用则拒绝启动，
不会杀掉用户已有仿真。全部对象静止，不加载飞行或关节控制插件；本轮所启动的进程结束后均已清理。
每组保留 `scene.world/scene.json/raw.bag/result.json/gazebo.log`，总目录保存配置快照、来源哈希、覆盖CSV和历史bag接口报告。
同目录 `static_points.png` 展示1 s回波与点数对照；图中目标隔离、颜色及边框使用独立评分真值，不是分割识别效果。
评分真值话题与原始点云分开，后续S3输入只能使用raw点云及允许的坐标/模型先验。

插件编译通过，8项边界测试与CTest通过：错时间单位/重复点时间、错布局、缺帧、失去IMU括号、
静态比力符号错误、表面位姿不符、预期/非预期目标缺失、场景全部静态及link内名称唯一。
`first_primary`是修复场景重名前的探查，`static_matrix_..._01`是纳秒接口修正前的历史结果；最终以03及其manifest为准。
S1/S2代码已分段提交；S2 ROS提交 `af0c4f1ac821f58448c865e250ca2c29e2cb31cd`，本轮未推送。

下一小步为S3静态目标分割与模型位姿：先用1.5 m/1 s原始点云做支撑/背景剔除、目标聚类与模型配准，
再统计位姿误差和拒绝用例；不先接飞行、闭爪或机械臂扰动控制。真实MID360S扫描/噪声校准继续保留为未验收项。

## 40. S3a：静态已知目标分割与位姿基线（2026-10-27）

本轮先分段提交S1、S2，再完成S3第一小步。ROS提交依次为S1 `92cd89f`、S2 `af0c4f1`、
S3a `262662085535174f07aa833e396968e5544bbe26`；PX4仓库的路线/S1、S2说明分别提交为 `07a05b1a75`、`34f1f230fb`。
本节随S3a说明另行提交，全部为本地提交，本轮未推送。

结论：静态原始点云→分割→已知模型位姿→独立评分的链路已跑通。
无测距噪声的两个1.5 m窗口误差约1.82–1.90 mm；σ=10 mm时仍有7.16–9.59 mm偏差，
因此只验收**静态感知基线**，不宣称带噪精抓精度达标或S3整体完成。没有启动飞行或闭爪，也未增加PX4消息/状态机。

### 40.1 实现与输入边界

实际ROS包 `/home/pcz/super_ws/src/super_px4_bridge` 与Git镜像 `/home/pcz/so101-px4-ros/super_px4_bridge` 同步：

- `scripts/perception/static_target.py`：纯NumPy几何估计核心，负责连通大平面剔除、欧式聚类、多初值刚体表面拟合和质量拒绝。
- `scripts/perception/run_static_target.py`：只回放现有bag，不创建ROS发布器；写出估计结果后再读取独立评分真值。
- `config/perception/static_target.yaml`：共同粗搜索区、点数/几何阈值、姿态先验、随机种子等；两观察位及所有噪声/负例保持同一参数。
- `tests/test_static_target.py` 与CMake中的 `static_target_math`：7项离线边界测试。

没有安装新计算库。当前Ubuntu-20.04系统没有SciPy，本轮使用已有NumPy完成原型；
平面/聚类思路参照[PCL平面分割](https://pointclouds.org/documentation/tutorials/planar_segmentation.html)与
[欧式聚类](https://pointclouds.org/documentation/tutorials/cluster_extraction.html)，本实现并未调用PCL ICP。

估计器只得到：原始XYZ、IMU静态比力给出的上方向、目标局部body/key/handle尺寸与相对位置。
局部模型通过白名单提取，**不读取目标世界位置/四元数、观察位或机械臂安装数据**；单元测试把这些字段改为无效值，局部模型仍完全一致。
粗搜索区固定为L中x=0.4..4.0 m、y=±1.2 m、z=±0.6 m，不随目标真值重心移动。
第一窗1.5 m带噪输入约3.30万原始点、粗搜索区约1.63万点，经过背景剔除/聚类选择193点候选；不是用真值裁出目标点。

本轮明确采用“已知目标近似立放”的先验：目标局部+Z与实测IMU上方向夹角≤35°。
IMU不提供目标位置或偏航；偏航以15°间隔种子及数据PCA初值搜索，再做六参数局部拟合。
全姿态任意目标、任意物体分类和运动观测不在本轮已验收范围内。

### 40.2 处理链与输出含义

```text
原始PointCloud2 + IMU（按源时间排列）
        ↓
固定粗搜索区 → 5 mm体素选点 → 连通大平面剔除 → 25 mm欧式聚类
        ↓
点数/外形筛选 → PCA/偏航多初值 → 已知盒组合的刚体表面距离拟合
        ↓
可见模型反向距离 + 非对称key支持 + 不同位姿解的分数间隔
        ↓
L中物体O位姿、valid/reasons、原观测时段；始终无控制许可
```

首次实验发现全局RANSAC会把远处共面的散点与小目标片段合并，导致第二个窗口目标被误删。
现改为仅删除空间连通且尺寸足够大的平面片段，小目标大小的片段保留；配合IMU上方向先验，解决了初版翻转解与漏检。
拟合采用局部几何的可见面和非对称凸块证据，不以场景真值姿态初始化。

`best.R`、`best.position_lidar_m` 定义 `p_L=R_L_O*p_O+t_L_O`。
位置对应物体模型O参考（本目标的抓取参考），不是点云均值、大主体中心或IK工具原点。
`valid`只表示本轮模型/质量条件通过；`best`在拒绝时仍可作为诊断候选，不能当已接受目标。
所有输出的 `control_authorized=False`、`grasp_ready=False`；不生成飞行、IK或闭爪请求。
`source_observation`保留源时钟、观察开始/结束纳秒与输入话题，回放和心跳均不刷新观测时间。
当前分数/解间隔尚未标定成统计置信度或协方差。

回放额外发现bag的接收时间排序可把早期IMU记录排到末尾：例如一段记录中源时刻2.331 s之后出现0.001 s，
但按源header排序后全程仍为5 ms间隔。现读取器按源header与点时间配对，继续拒绝真实缺帧、重复帧及不完整窗口，
不通过重打时间戳或补造样本规避错误。固定100 ms帧/5 ms IMU的检查仍只适用于S2理想仿真，不能直接移作实机校验器。
静止前提来自S2静态场景；角速度检查本身不能证明没有平移，运动去畸变需后续LIO链路。

### 40.3 固定矩阵与独立评分结果

正式结果：`build/mid360s_s3/static_pose_20261027_04`，父数据为第39节最终 `static_matrix_20261027_03`。
各组使用[0.2,1.2)与[1.2,2.2) s两个不重叠源时间窗口，实际纳秒时段以各帧初始epoch加此偏移为准。
估计器在读取 `scene.json` 或 `/mid360s_s2/ray_truth` 之前先写出 `estimate.json`。
评分再单独核对物体O真值、目标是否有回波，以及选中簇的目标点比例。

| 条件（均1秒积累） | 质量接受数/窗口数 | 位置误差 | 姿态误差 | 判断 |
|---|---:|---|---|---|
| 1.5 m，σ=0 | 2/2 | 1.82、1.90 mm | 1.64°、1.86° | 静态基线通过；两窗数值均在3 mm/5°对照范围内 |
| 1.5 m，σ=10 mm | 2/2 | 7.16、9.59 mm | 2.00°、5.38° | 模型质量通过，但精度不能直接支撑当前精抓 |
| 1.5 m，σ=20 mm | 0/2 | 拒绝候选约12.68、16.08 mm | 约3.36°、6.65° | 拟合质量/位姿歧义拒绝 |
| 3 m，σ=0/10 mm | 0/4 | 仅诊断候选，不采纳 | 仅诊断候选 | 非对称特征不足或点数不足拒绝 |
| 完全遮挡、抓取盲区 | 0/4 | 无有效位姿 | 无有效位姿 | 本组4窗无目标时均未误接受 |

以上14窗中4窗通过模型质量，只有2个无噪声窗达到3 mm/5°数值对照。
这个对照不是控制许可，也没有为外参、悬停或机械臂误差分配额外余量。
样本数少且场景混合，不能把4/14或其P95当作通用检测成功率/实机精度。
选中簇的独立目标点比例约86%–94%，仍含少量非目标点；这和物体边缘/遮挡共同提示后续应检查分割混入与表面拟合偏差，
尚未证明其中某项就是全部误差来源。

冻结相同算法/参数补做积累时长对照：

- `build/mid360s_s3/accumulation_2s_20261027_02`：0.2 s起积累2 s，σ=0误差1.67 mm/1.55°；
  σ=10 mm为6.33 mm/1.61°，仍超过3 mm位置对照；σ=20 mm仍拒绝。
- `build/mid360s_s3/short_window_20261027_02`：0.2 s短窗口的近距有/无噪声与两个负例共4窗均拒绝，未放低点数阈值强行通过。

延长积累能改善这组带噪结果，但单靠2秒积累尚未达到所需精度。每个发生拟合的1秒窗口处理耗时约0.6–2.2 s，
本轮为离线原型，不是10 Hz实时感知或现有反馈时效已经满足的证据。
父数据仍是MID360扫描轨迹代理及简化噪声，不替代MID360S来源明确的实测校准。

### 40.4 复现、提交与下一小步

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 /home/pcz/super_ws/src/super_px4_bridge/scripts/perception/run_static_target.py \
  --dataset /home/pcz/PX4-RoboMaster/build/mid360s_s2/static_matrix_20261027_03 \
  --output /home/pcz/PX4-RoboMaster/build/mid360s_s3/my_pose_run

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 /home/pcz/super_ws/src/super_px4_bridge/tests/test_static_target.py -v
cd /home/pcz/super_ws/build
ctest -R 'mid360s_static_math|static_target_math' --output-on-failure
```

`--window 2.0 --starts 0.2` 可做2 s对照；`--cases primary primary_noise` 可只回放两组。
每轮输出必须用新目录，包含参数快照、只含局部形状的 `model_used.yaml`、源bag/评分文件/源码哈希，
以及各窗 `estimate.json/evaluation.json/segmentation.npz`。独立评分文件和估计结果分开保存。
正式目录的 `static_pose.png` 展示共同搜索区、实际分割簇/估计模型及误差。

7项边界测试通过：世界位姿/安装字段隔离、远处共面小目标保留、独立合成位姿恢复、缺非对称特征拒绝、
稀疏/空/非有限数据拒绝、无效重力/参数拒绝，以及只读测量话题、接收顺序变化不改变观测、真实缺帧拒绝。
S2的8项测试与S3a的7项测试经CTest同时通过；源工作区与ROS Git镜像一致。

下一小步继续S3b：先核对分割簇的支撑混入，再处理带噪边缘表面拟合偏差，用同一组窗口前后对照，
保持已有3 mm/5°数值对照与所有控制保护。先收敛静态测量误差，再进入实时LIO和感知接近流程。

## 41. S3b：粗筛减支撑混入、射线精修与最小 demo 收束（2026-10-27）

复核完成，ROS 本地提交为 `e1f79ee0f783896399509a7b012207daa7902f38`。
S1/S2/S3a 已按前轮授权推送，远端 ROS 为 `2626620`、PX4 为 `e2a09a88f8`；
本节的 S3b 代码与说明是之后的新提交。本轮只做离线回放，没有启动仿真、飞行、IK 或闭爪。

### 41.1 修改与复核边界

仍只修改 ROS 包已有的四个文件，没有新增节点、消息或状态机，也没有修改 PX4 控制器、SDF、PID 或接触保护：

- `scripts/perception/static_target.py`：在候选簇内增加有界底部薄层剔除；
  用实测 IMU 上方向计算点高度，默认排除最低约 15% 的点，薄层高度超过 20 mm 则保留整簇。
  不读取支撑尺寸/姿态或语义标签。这是固定近似立放目标的几何粗筛，不是通用支撑语义分割；
  可能连同部分主体底边一起剔除，因此同时统计目标点保留率。
  原表面距离拟合之后增加射线首次相交的测距残差精修，匹配 S2 沿测距方向施加噪声的观测模型；
  未相交的射线以表面距离引导初值，并单独检查模型射线命中比例，不把“没命中”当作零测距误差。
- `config/perception/static_target.yaml`：增加两个粗筛参数和一个射线精修开关。
  原点数、表面拟合、非对称特征、位姿歧义和控制保护阈值未放宽。
- `scripts/perception/run_static_target.py`：记录粗筛前后候选、独立标签计数、目标点保留率、
  支撑剔除率及不重叠观测窗口的位姿差；补齐观测起点与终点两侧的 IMU 括号检查。
- `tests/test_static_target.py`：补充薄层上限、坐标变换、射线平行/遮挡及可见首次回波的几何恢复。
  原完整模型采样含隐藏背面，其恢复测试明确使用表面模式；新增可见首次回波测试覆盖射线模式。

估计器仍只得到 raw XYZ、局部 body/key/handle 模型和实测 IMU 上方向。
估计落盘后才读取世界真值及 ray_truth 评分；没有把分割真值裁剪作为正式估计输入。
`valid`仍只表示模型质量条件通过，始终 `control_authorized=False`、`grasp_ready=False`；
配准残差、射线命中比例、重复窗口差均未标定为统计置信度。

### 41.2 冻结数据前后对照

正式目录：`build/mid360s_s3/s3b_review_20261027_01`。
父数据仍为第39节 `static_matrix_20261027_03`。
三组分别为 `baseline`、`coarse_only`、`final`，每组同样的14个窗口；
baseline 关闭底部剔除与射线精修，重现 S3a 结果。
`review.json`确认源码、配置、输入 bag 和评分文件哈希相符；源观测时间未刷新。

| 1.5 m / 1秒窗口 | 原方法位置误差 | 仅粗筛 | 粗筛＋射线精修 |
|---|---:|---:|---:|
| 无噪声，0.2 s起 | 1.822 mm | 0.421 mm | 0.041 mm |
| 无噪声，1.2 s起 | 1.895 mm | 0.125 mm | 0.005 mm |
| σ=10 mm，0.2 s起 | 7.165 mm | 11.202 mm | 4.718 mm |
| σ=10 mm，1.2 s起 | 9.594 mm | 16.057 mm | 4.000 mm |

无噪声中接近零的误差是同一理想盒几何/无角误差下的仿真数值结果，不是 MID360S 实机精度。
仅删除底部点会使带噪位姿变差，说明“候选更干净”不等于“估计更准确”；本轮保留粗筛与射线精修的组合。

四个近距窗口的候选目标比例由约86%–94%升至99.32%–99.44%，
支撑点剔除率91.7%–96.3%，目标点保留率89.4%–98.2%；
本组非对称 key 与 handle 点数均未减少，损失主要在主体底边。
σ=10 mm 两窗姿态误差3.87°/3.21°，窗口间位置差4.15 mm、姿态差4.07°；
仍未满足既有3 mm位置数值对照，也没有证明重复性或总抓取误差预算达标。

σ=20 mm 两窗仍因表面拟合/歧义拒绝；遮挡与盲区4窗均未误接受。
3 m 四窗中仅一个无噪声窗口通过，另外三个点数不足拒绝，不能据此验收3 m稳定精定位。
最终发生拟合的窗口约0.9–2.9秒处理时间；仍为离线原型，不作为10 Hz实时链路。

S2的8项与S3的10项测试经 CTest 全部通过；源工作区与 ROS Git 镜像的四个修改文件一致。
探索目录 `s3b_support_20261027_01` 中的真值剔除诊断只用于误差归因，
不是正式算法、不是控制输入；正式结果以上述 review 目录为准。
本轮沿用开发数据做回归和对照，未声称独立实测或未见场景泛化验收。

### 41.3 下一步只走最小 demo 主线

本阶段停止扩展感知方法。第一版优先使用已知固定物体、1.5 m稳定观察和受控的理想仿真回波，
先锁定一次感知快照，再复用已有接近、闭爪、抬起保持流程。
带噪4 mm量级偏差继续如实记录；不通过修改评分阈值把它包装成精抓通过。

现有可成功抓取的飞行 demo 仍在运行时生成旧的20 mm `test_cube`，
而本轮感知对象是“大主体＋非对称凸块＋窄柄”；物体O又定义在抓取参考处。
因此不能直接把新估计位置填成旧方块中心并启动闭爪。下一小步只处理这个实际接口差异：

1. 让运行时实体与已验证的感知局部模型一致，核对 O、夹柄和现有抓取参考的对应关系。
2. 用同一观测时刻的机体位姿和安装外参，将 `T_L_O` 转为既有任务需要的目标；
   保留源时间，目标世界真值只用于独立核对。
3. 先验证感知目标能进入现有无闭爪接近入口，再沿原有闭爪、抬起≥25 mm、保持5秒流程验证实际抓住不掉落。

现有飞行定位可作为显式标注的 SIM_VISION 演示条件先复用，避免等待整套 LIO 改造；
这只能称为“雷达目标感知驱动的仿真 demo”，不能宣称纯雷达定位飞行或实机已验收。
FAST-LIO定位替换、真实MID360S噪声校准与多姿态重复性放在最小闭环完成后逐步完善，
原有净空、反馈时效和闭爪保护继续保留。


## 42. S4：雷达目标到飞行抓取的最小 demo（2026-10-28）

### 本轮结果

`build/known_target_flight/lidar_grasp_20261028_14/result.json` 已通过：
观察悬停 → 原始点云识别并锁定目标 → 飞到抓取悬停位 → 原生 IK 接近 → 闭爪 → 抬起 → 保持。

- 独立验收的5秒保持窗口内，物体最小抬升 **35.964 mm**，满足至少25 mm；
  5021个物理接触步中双侧接触比例 **1.0**，支撑接触比例 **0.0**。
- 目标和飞机均为动态刚体，没有固定飞机或把物体附着到夹爪；
  保持窗口的净空检查通过，无禁止接触。
- 理想测距条件下，目标位置/姿态独立评分误差约 **0.025 mm / 0.043°**。
  此数值只说明这次理想仿真的接口与几何配对，不能作为实机精度。
- 飞行终端保持期间位置误差均值约5.08 mm、最大约11.93 mm。
  最终启动脚本复现也已通过：5.021秒保持窗口内最小抬升37.625 mm，双侧接触1.0、支撑接触0.0。

### 修改位置与原因

1. PX4 测试目录 `src/modules/arm_control/tests/hover/lidar_grasp_demo.py`：
   复用S1的 body/key/handle 几何；物体为10 g候选刚体，按分块体积计算质心和惯量。
   抓取参考O与质心分别处理。顶部雷达和安装板加入约381.64 g实际仿真质量。
   雷达仍是MID360轨迹/MID360S视场代理，测距噪声为0。
2. `run_known_target_flight.py --lidar-target`：
   先在约1.25 m观察高度悬停，使用1秒原始点云，按源时间配对机体位姿并以5 ms分箱补偿微小运动。
   目标通过粗筛和精评后才创建飞行/抓取任务；离线检查结束后重新读取时间与机体位置，
   避免旧时刻导致设定点跳变。原方块和SUPER入口保留。
3. `known_target_lidar.json`：候选目标放在世界坐标约[1.480, 0.021, 1.500] m，
   使用既有可达抓取姿态；观测姿态进入计划后不再被离线IK标定姿态覆盖，超过5°兼容范围则拒绝。
4. `fixed_grasp_probe.py`：
   物体仍在支撑上的Close/Verify阶段持续保持世界坐标TCP；
   仅在原生夹持确认完成、进入Lift后锁定机体相对携物目标。
   闭合前固定指预靠近2.3 mm，仍在原3 mm入场误差预算内。
5. ROS `src/coordinate_truth_plugin.cpp`：
   显式识别三碰撞体目标，只有两个指定夹指对指定夹柄允许接触；
   主体、凸块、支撑和地面继续检查，其他对16 mm、精抓夹指对支撑3 mm。
6. ROS `src/perception/mid360s_static_plugin.cpp`：
   新增显式moving demo开关，默认静态S2约束保留。
   动态模式按物理步姿态发射射线，保留源点时间；
   锁定目标后取消订阅并停止这次快照射线计算，减少控制阶段负载。
   IMU仅提供理想重力/角速度线索，不是可直接用于LIO的真实加速度模型。
7. ROS `scripts/start_lidar_grasp_demo.sh`：单个复现入口，支持`--gui`。
   ROS源码已同步到`/home/pcz/so101-px4-ros`，解释集中维护在本节。

### 最终保留的接触设置

新物块主体/凸块摩擦为0.4，夹柄摩擦为1.2；
夹柄接触kp=500 N/m、kd=10 N·s/m、min_depth=0.05 mm、max_vel=0.01 m/s。
这些是演示的柔顺接触候选值，尚无材料实测标定。

前期版本能形成双侧接触，但夹爪微振动使连续验证无法完成，并可能进一步触发推力余量保护。
本轮按原始角度/速度记录调整夹柄柔顺性。
增大关节物理阻尼、改变物理步长和夹爪PID的试验均已撤回。
最终仍使用原1 ms步长、40次求解、夹爪P=1.5/D=0.001、0.01 Nm力矩上限；
未增加PX4消息、飞控状态机或修改飞控PID。
连续夹持1秒确认、闭爪姿态/速度、反馈时效和原净空保护保留。

### 复现与验收记录

在Ubuntu-20.04中执行；先结束已有仿真：
```bash
bash /home/pcz/super_ws/src/super_px4_bridge/scripts/start_lidar_grasp_demo.sh --gui
```

脚本会启动ROS、Gazebo、PX4和MAVROS，输出本次结果目录。
默认仿真速度因子0.5；已有实例会使启动被拒绝，脚本只清理自己启动的进程。
成功必须同时满足`result.json`中的`success=true`和`fixed_grasp.grasp_validated=true`，
并检查最小抬升、双侧接触、支撑接触与保持时长，而不是只看闭爪动画。

已有抓取/物块30项回归及感知两套测试（18项）通过。
静态S2扫描回归目录为`build/mid360s_s2/demo_regression_20261028_01`；
primary/primary_repeat均为20帧、373个目标回波，点云字节哈希完全相同。

| 完整抓取用例 | 最小保持抬升 | 保持时长 | 双侧接触比例 | 支撑接触比例 |
| --- | ---: | ---: | ---: | ---: |
| `lidar_grasp_20261028_14` | 35.964 mm | 5.000 s | 1.0 | 0.0 |
| 最终脚本 `lidar_grasp_20261028_115005` | 37.625 mm | 5.021 s | 1.0 | 0.0 |

两次`result.json`均为`success=true`、`fixed_grasp.grasp_validated=true`，且自身进程已清理。
最终脚本复现的目标评分为0.012 mm / 0.031°，飞行终端保持误差均值5.12 mm、最大12.05 mm。

本节对应ROS提交为`a75292b9aedb752c8040b934ec0ac157e93ff78d`；Gazebo子模块没有新增修改。

### 当前数据来源边界

估计器只接收原始点云、理想重力线索以及显式的仿真机体世界位姿，不读取物体位姿、关节或ray标签；
目标估计的评分在估计落盘后进行。
启动时的场景核对、后续精接近居中及几何/接触许可仍沿用既有仿真真值反馈，飞行定位使用原SIM_VISION融合。
因此当前通过的是雷达目标初始化、仿真定位与精接近辅助的抓取联调。
本轮没有验收FAST-LIO定位、SUPER多障碍与该新目标的完整组合或携物飞行。

## 43. 抓取后携物与 FAST-LIO / SUPER 联调（2026-10-28）

目标链路：雷达观测目标 → FAST-LIO 建图 → SUPER 绕障到抓取位 → 现有 IK 接近、闭爪、抬起 → 携物飞到指定点。继续复用原生 arm_control，未新增 PX4 / ROS 自定义消息或抓取状态机。

### 43.1 修改范围

- `tests/hover/carry_demo.py`：分为先上升、后平移的短程轨迹；默认相对抓取悬停位平移 Y=0.5 m、上升 Z=0.12 m，命令峰值速度 0.03 m/s。支持 `--carry-to X Y Z`，限制在起点 0.9 m 范围内并至少上升 80 mm。
- `fixed_grasp_probe.py`：通过抬起和 5 秒保持后，继续发送同任务的实时几何与接触反馈。携物时维持臂相对机体的姿态；已经通过的静态保持窗口冻结，不因后续接触波动被误清空。
- ROS `super_flight_manager.cpp` / `known_flight_handover.h`：增加默认关闭的私有 `begin_slow_carry` 服务，供已验证抓取的协调器切换任务到慢速航段。沿用任务 ID、反馈时效和租约，航速上限 0.04 m/s、范围 1 m、超时 120 秒；原生抓取的 0.05 m/s、倾角、力矩分配及接触保护保持生效。
- `fastlio_sim_adapter.py`、`fastlio_sim.yaml`、`fastlio_sim.launch`：保留逐点时间，将 offset_time 从纳秒转为秒，使用 FAST-LIO 既有 PointCloud2 接口。首次静止时仅用一次 MAVROS 位姿对齐地图原点，之后不使用真值持续修正 LIO。
- 雷达仿真插件新增显式 `demoLioImu` 开关：在传感器安装点对速度作时间差分，输出含平动与杆臂运动的比力和角速度。原静态扫描及旧的重力提示模式保持默认行为。
- 生成世界加入三处障碍与五个定位墙面。墙面使用已有 `demo_obstacle_` 名称契约，因此同样进入原有全臂 / 全夹爪行程的碰撞检查。小场景地图分辨率 0.15 m、膨胀 0.6 m，与 SUPER 的 0.5 m 机体半径一致。

### 43.2 启动初始化

原生成世界在机体 Z=0 时启动物理，起落架已经深入地面，可能产生不同的落地位置、偏航和起飞电机饱和。直接把 SDF 出生高度改为 0.45 m，又会使现有视觉定位插件把这个高度当作局部原点。

当前流程在暂停状态加载零原点世界；确认 `/gazebo/default/yhang550/vision_odom` 已广告、视觉原点已经保存后，把整机设置到 Z=0.45 m，再解除暂停。仅发生在第一个物理步之前。预检仍要求世界 / 局部原点误差不超过 20 mm，起飞 30 秒未完成则退出。07 预检误差 0.545 mm，并正常起飞。

### 43.3 已验收的独立携物实验

结果：`build/known_target_flight/lidar_carry_20261028_02/result.json`，success=true。

- 携物实际位移 0.5147 m；终点误差 2.062 mm，到位后保持 5 秒。
- 43.726 秒携物观察、43747 个接触物理帧，双侧接触比例 1.0，支撑接触比例 0.0。
- 物体相对夹爪的最大偏移约 4.25 mm。物体为自由动力学实体，无固定连接或抓取吸附。
- 新增轨迹、携带证据和雷达时间格式检查已通过（33 个机械臂测试、2 个适配测试，以及任务绑定 C++ 检查）。

### 43.4 运行与范围

首次需要编译现有 FAST-LIO 仿真依赖：

```bash
bash /home/pcz/super_ws/src/super_px4_bridge/scripts/perception/build_fastlio_sim.sh
```

该脚本复用 `/home/pcz/FAST_LIO`，只从官方 Livox 源码提取既有消息定义作为编译依赖，不安装设备驱动。FAST-LIO 源码版本和 Livox 消息版本均固定记录；依赖和构建产物放在 PX4 的 build 目录。

正式入口：

```bash
bash /home/pcz/super_ws/src/super_px4_bridge/scripts/start_lidar_grasp_demo.sh --gui
# 可选：指定携物目的地（世界坐标，需满足短程上升与净空条件）
# bash .../start_lidar_grasp_demo.sh --gui --carry-to 1.317 0.524 2.066
```

当前 FAST-LIO 为 SUPER 提供地图和里程计；PX4 EKF 仍保留已验证的仿真视觉定位。目标估计的坐标配对和末端精接近仍含仿真真值辅助。当前雷达是理想扫描 / IMU 代理，尚不能据此宣称 MID360S 实机精度或完成了纯雷达飞行定位。携物航段为有界低速点到点，不等同于长航程携物规划。

参考依赖：[FAST-LIO](https://github.com/hku-mars/FAST_LIO)、[Livox ROS driver 消息定义](https://github.com/Livox-SDK/livox_ros_driver)。

### 43.5 正式入口整链路验收

正式入口案例：`build/known_target_flight/lidar_grasp_20261028_135132/result.json`。success、super_obstacle_navigation_validated、fixed_grasp.grasp_validated、carried_flight_validated 均为 true。所有自有进程已清理。

| 项目 | 本轮实测 |
| --- | --- |
| 三障碍绕行 | 机体 0.45 m 保护包络至障碍最小净空 149.3 mm |
| SUPER 航段 | 对起终点直线最大偏离 1.058 m，确认实际绕行 |
| FAST-LIO 对独立真值的配对核对 | 3037 帧，平均位置误差 10.07 mm、最大 18.02 mm |
| 抓取后保持 | 实际最小抬起 35.19 mm，保持 5.026 秒，双侧接触 1.0、支撑接触 0.0 |
| 携物 | 机体位移 0.5135 m、物体位移 0.5118 m |
| 携物到位 | 终点误差 0.809 mm，随后保持 5.003 秒 |
| 携物反馈 | 43.726 秒、43747 物理接触帧，双侧 1.0、支撑 0.0，相对滑移最大 6.89 mm |
| 指令源 | MAVROS 最终设定点只有一个发布者；地图与里程计的实际发布者均已核对为 FAST-LIO 链路 |

ROS 配套提交：`09aec79685c02b37bf2e4bb80e6a60e0bc088f6e`。上表为一次正式整链路运行，加上 43.3 的独立携物正例；尚非长时间重复统计或实机验收。

## 44. 六柱交错通道：穿行后识别再抓取（2026-10-28）

本步将场景扩为三排六根宽方柱，并把目标观测移到通道末端：折叠臂起飞 → SUPER 穿过三道交错开口 → 观测区悬停 → 新点云窗口识别目标及位姿 → SUPER 到待机位 → 原有精接近、闭爪、抬起与携物。

### 44.1 场景与修改

- 方柱尺寸 1.2 × 1.2 × 3.4 m，三排 X=2.0、4.8、7.6 m；开口宽 1.8 m，中心 Y 依次为 +0.6、−0.6、+0.6 m。
- 侧墙 Y=±2.4 m，外侧窄缝不能供机体通过；观测区约在 X=9.3 m，目标参考点 X=10.8 m、Z=1.5 m。
- `tests/hover/pillar_course.py` 集中定义立柱、墙面、观测航点和过桩审计；`known_target_pillars.json` 保存该场景目标。目标、摩擦、夹爪和 IK 参数沿用前轮。
- `run_known_target_flight.py` 增加 `--pillar-course`，复用现有多目标飞行管理入口；没有新增原生 PX4 消息或抓取状态机。
- 正式 `start_lidar_grasp_demo.sh` 默认启用新场景；旧基线可直接运行 Python 入口、不加 `--pillar-course`。

四个粗导航航点是任务输入，SUPER 根据 FAST-LIO 的实时点云地图计算轨迹；未预先灌入立柱占据地图。立柱位置另用于独立碰撞与穿越审计。

### 44.2 识别顺序

全部开口通过后才创建观测器。以真实机体轨迹与各排 X 平面求交，检查交点 Y 和高度，避免把绕到末端当作穿桩。未通过则不识别或展开机械臂。

识别保留原始点云的一秒来源窗，最终检查窗口开始时间晚于观测区到达事件，防止复用起飞点或穿行中的旧结果。目标真值只评分，识别器不读取目标真值；机体配对与精接近的仿真辅助边界仍见 43.4。

记录：`pillar_passages.json`、`navigation_events.json`、`perception_attempt_*/observation.bag`、`perceived_target.json`。

### 44.3 配置与运行

虚拟顶高曾设为 3.0 m，经 ROG-Map 为膨胀收缩边界后，1.95 m 目标被投影为 1.425 m，原接管检查正确拒绝。当前生成配置设为 4.2 m，物理屋顶为 4.5 m；机体仍限于 2.5 m 以下，不能越柱顶，终点投影阈值未放宽。

默认按 0.35 倍仿真时间推进，为 WSL 点云、物理和反馈配对留调度余量；名义飞行、关节参数及原时效、接触、净空保护保持。

```bash
bash /home/pcz/super_ws/src/super_px4_bridge/scripts/start_lidar_grasp_demo.sh --gui
```

### 44.4 正式入口验收

案例：`build/known_target_flight/lidar_grasp_20261028_145503/result.json`。全流程 success=true，三道通道、后置识别、原生抓取和携物验证均通过。

| 项目 | 结果 |
| --- | --- |
| 三道通道交点 Y | +0.477、−0.508、+0.588 m，全部从开口内通过 |
| 机体保护包络至六柱最小净空 | 188.5 mm |
| 后置识别 | 到达观测区 97.772 s；原始点云窗 111.138–112.138 s，时间顺序检查通过 |
| 理想点云位姿评分 | 位置误差 0.054 mm、姿态误差 0.0395°；不代表实机精度 |
| 夹起保持 | 最小抬起 34.445 mm，保持 5.006 s |
| 携物 | 机体位移 0.517 m，物体位移 0.510 m，终点误差 5.66 mm，再保持 5 s |
| 物理夹持 | 携物观察 43.723 s / 43747 帧，双侧接触 1.0、支撑 0.0，无固定连接 |
| FAST-LIO 配对核对 | 平均位置误差 51.8 mm、最大 58.9 mm，未作连续真值修正 |
| 检查 | 36 项离线几何 / 流程测试通过，Python 编译、启动脚本和 Git 差异检查通过 |

时间均为仿真时间。ROS 配套提交：`c15c06834497a2d89e9337f526b42b70a77cdfce`。实际轨迹图保存在案例目录的 `pillar_flight_top_view.png`。全部自有仿真进程已退出。

本场景在 0.5 倍推进时曾因反馈配对短时超过原窗口而在携物后段退出；默认 0.35 倍推进已完成本次整链路正例，保护窗口未放宽。当前不是长时间重复统计或纯雷达飞行定位验收。

## 45. 移除定位真值辅助：独立 LIO 飞行第一步（2026-10-28）

本轮先验证最小独立链路：原始雷达 / IMU → 原版 FAST-LIO → PX4 EKF → 起飞、悬停、局部平移。未运动机械臂、未闭爪，未修改 PX4 原生模块或增加消息。第 44 节已通过的完整 demo 默认入口保持原行为，不能将本节结果称为整链路无真值抓取。

### 45.1 改动与坐标约定

- ROS `fastlio_sim_adapter.py` / `fastlio_sim.launch` 新增 `alignment_source:=lio_origin publish_to_fcu:=true`。新模式用首帧 LIO 机体位姿确定一次局部原点与初始航向，不订阅 MAVROS 里程计对齐自身；`map` 保留重力方向。位置零点是初始化时的机体位置，不能直接沿用旧 Gazebo 世界坐标目标。
- 始终保留 LIO 源时间戳；只向 `/mavros/vision_pose/pose_cov` 发布新鲜的位姿。位置 / 角度协方差下界分别为 0.05 m / 0.03 rad，属于当前仿真实验设置，未完成实机标定。差分速度未输入飞控。
- 拒绝旧 MAVROS 对齐模式同时反馈 FCU，避免形成定位对齐环路。旧模式仍为默认，未给旧完整 demo 更换定位。
- 新建单一 `tests/hover/run_lio_hover.py` 作为独立验收入口，复用既有模型、飞行管理器、安装外参与进程清理。世界中没有仿真视觉定位插件，不发送模拟里程计或物体抓取真值给 PX4。真值只由记录器保存；控制结束、全部自有进程停止后，才打开 bag 做评分。
- EKF 实际检查融合外部位置、高度、偏航（EV_CTRL=11），不融合外部速度、GNSS、气压高度或磁力计。保留常规机载 IMU；模拟雷达 / IMU 仍是理想传感器代理，不能据此承诺 MID360S 实机精度。
- 验收要求 LIO / EKF 够新、维持已解锁 OFFBOARD、起飞 30 秒内完成、姿态与位置在有限范围内。达到局部 `[0.3, 0, 0.9] m` 附近后，估计位置误差小于 50 mm、速度小于 80 mm/s 连续保持 8 秒；最终实际位置最大误差要求小于 120 mm。真值评分只作一次离线原点 / 偏航对齐，不逐帧修正漂移。

### 45.2 起飞推力与验收

02 中 EKF 融合已成功，但飞机在地面阶段出现偏航和控制分配饱和，未起飞。随后只在本实验 rootfs 参数中设置 `MPC_THR_HOVER=0.84`，未改生产 airframe 或 PID。依据：组合模型质量 3.108 kg、电机映射 `ω=min(1000, 1000u+100)`，对应静态平衡输入约 0.844；也与前一完整 demo 实测悬停集体输入约 0.843 一致。03–05 均完成起飞与保持。这支持当前设置有效，尚不构成完整的地面瞬态根因验收。

总最大静态推力约 34.19 N、重力约 30.48 N，推重比约 1.122。当前结果仅覆盖低速空载带臂飞行，不等于已验证更重载荷、强扰动或长航程。

结果位于 `build/lio_hover/lio_local_20261028_0{3,4,5}/result.json`：

| 案例 | LIO 位置最大误差 / mm | 最后 5 秒实际飞行目标最大误差 / mm | 起飞、悬停、0.3 m 平移及保持 |
| --- | --- | --- | --- |
| 03 | 3.90 | 77.77 | 通过 |
| 04 | 3.90 | 77.63 | 通过 |
| 05 | 3.75 | 77.46 | 通过 |

05 为加入 OFFBOARD 状态时效和评分首帧完整性检查后的最终版本。05 的 LIO 平均位置误差为 2.92 mm，最终实际飞行目标平均误差为 60.89 mm；二者不能混为同一个指标。仍需改善飞行跟踪，不能把这组厘米级飞行误差用于宣称毫米级抓取对齐。10 项原点、时间字段与对齐环路测试通过；默认启动行为回归通过。三个正例全部自有进程已清理。

ROS 配套提交：`f4dcf5a`。

独立验收（先关闭其他仿真；名称不能重复）：

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
cd /home/pcz/PX4-RoboMaster
export PYTHONPATH="$PWD/src/modules/arm_control/tests/grasp:$PWD/src/modules/arm_control/tests/hover:${PYTHONPATH:-}"
PX4_SIM_SPEED_FACTOR=.35 python3 src/modules/arm_control/tests/hover/run_lio_hover.py UNIQUE_CASE --gui
```

### 45.3 后续仍需移除的依赖

按原主线继续，每步只替换一个输入：

1. 目标识别的点云 / 机体位姿配对改用同一 LIO 地图；目前观察器仍使用机体真值。目标物位姿已经来自点云，不能因此忽略机体配对的真值依赖。
2. 用 LIO 机体位姿、编码器关节角和固定外参计算末端及目标的相对关系，替换悬停许可、精接近、净空检查中的机体 / 夹爪真值。已知模型几何和参数可保留，但不能读取实际物体位姿冒充传感器。
3. 用双侧夹指接触、接触力、夹爪开度与关节状态验证夹持；物体真实离架、真实滑移和携带轨迹保留为独立离线评分。当前原生抓取反馈仍显式要求仿真真值来源，尚未替换。

顶部正装雷达在最终近距离下方抓取区域有视场盲区。首个无真值版本限定静态目标：在可见观测位估计并锁定目标，再通过共同的 LIO 地图接近；不能称为盲区中的实时目标跟踪。现有精抓夹指—支撑 3 mm 净空也不能直接视作 LIO 不确定度预算，后续需用实际估计误差核对几何余量，保留原接触和净空保护。

## 46. 目标点云与 LIO 机体位姿配对（2026-10-29）

完成第 45 节后的一个小阶段：独立 LIO 定位起飞 → 原有局部航点悬停 → 原始点云识别目标 → 输出 LIO `map` 中的目标位姿。不发送机械臂接近或闭爪请求，未增加消息、状态机或调整 PID；完整抓取默认入口仍使用旧的仿真辅助路径。

### 46.1 修改范围

- `tests/hover/lidar_grasp_demo.py`：`TargetObservation(..., body_source='lio')` 只订阅原始点云、IMU 和 `/lidar_slam/odom`，验证 `map → base_link`。局部形状仍通过既有白名单提取，绝对目标位姿、标签和实际夹爪位姿不进入估计器。默认 `sim_truth` 模式保留，旧插值容差仍为 0.05 秒。
- LIO 为约 10 Hz：按原始 header 与逐点时间配对，检查整段一秒点云窗口被机体位姿前后包围；限制相邻位姿间隔 ≤0.125 秒，禁止外推。缺一帧形成 0.2 秒间隙、重复时间和非有限数据均拒绝。5 ms 去畸变分箱仍使用插值，不能称为 200 Hz 位姿测量。
- 保留一秒内机体位移 ≤10 mm、转角 ≤1° 的稳定门槛。修正 LIO 模式的等待计数：不稳定窗口只等待，不消耗四次目标拟合机会；重试使用不重叠的新窗口，受原有 110 秒仿真时间总期限约束。01 曾因旧逻辑把不稳定等待计作拟合失败而提前退出，未进入识别；02 修复后通过。
- 输出 `perceived_target_map.json` 明确 `frame=map`、`map_from_object`；报告使用 `body_map_at_reference` / `lidar_map_at_reference`，不沿用世界坐标字段。观测 bag 只保存原始点云、IMU 和 LIO 里程计。
- `run_lio_hover.py --observe-target` 复用同一飞行入口，新增观测后的离线目标评分；真值 bag 只在自有进程停止后读取。新 `test_lio_observation.py` 集中覆盖源隔离、时间覆盖和等待回归。

### 46.2 验收与结果

案例：`build/lio_hover/lio_target_20261029_02/result.json`，success=true。控制 / 观测运行图中 `/grasp_audit/truth` 的唯一订阅者是记录器；观测 bag 只有 `/mid360s_sim/points`、`/mid360s_sim/imu` 和 `/lidar_slam/odom`。全部自有进程已退出。

| 项目 | 本轮结果 |
| --- | --- |
| 识别来源窗 | 71.472–72.472 秒，10 帧点云，源时间未重写 |
| 实际最大配对间隔 | 0.100005 秒，满足 0.125 秒上限 |
| 窗口内机体运动 | 最大平移 6.429 mm、转角 0.3833° |
| 地图中目标位置误差 | 2.117 mm（仅评分时作一次初始地图原点 / 偏航对齐） |
| 目标姿态误差 | 0.1778° |
| 机体相对目标位置误差 | 0.342 mm，按同一观测参考时刻离线核对 |
| 飞行定位误差 | LIO 最大位置误差 3.603 mm |
| 最后 5 秒实际飞行目标误差 | 平均 38.43 mm、最大 54.65 mm，与识别误差不同 |
| 检查 | 11 项 LIO 配对 / 来源 / 等待测试、36 项既有几何测试通过 |

`target_offline_score.json` 和 `offline_replay_score.json` 包含最后补充的机体相对目标坐标核对；使用的是同一次已经停机的 02 记录，没有把评分原点或真值写回在线观测。原始运行 `result.json` 的目标地图误差、姿态误差与重放一致。

当前是单个理想雷达 / IMU 代理仿真正例，局部形状已知，不能据此承诺实机毫米级精度，也不构成精抓或支撑净空许可。稳定窗口出现前仍需等待，本轮未解决更高要求的飞行定点控制。

### 46.3 复现及下一步

先关闭其他仿真，按第 45 节设置 ROS 与 PYTHONPATH 后运行：

```bash
PX4_SIM_SPEED_FACTOR=.35 python3 src/modules/arm_control/tests/hover/run_lio_hover.py UNIQUE_CASE --observe-target --gui
```

目标位置在 LIO 局部地图中，不能直接作为旧 `gazebo_world` 配置使用。下一步将末端相对关系改由 LIO 机体位姿、安装外参和编码器正运动学计算，再核对接近 / 净空判定；原生抓取反馈的真值来源限制、夹持和携物判断仍需单独替换。继续保留现有碰撞、净空和闭爪保护。

## 47. LIO、安装外参与编码器正运动学（2026-10-29）

完成折叠姿态下的只读末端坐标链：LIO 机体位姿 + 安装外参 + 关节角反馈 → 末端 / 抓取参考点相对已识别目标的位姿。未请求展开、IK 接近或闭爪，未新增 PX4 / ROS 消息、状态机或修改 PID；已有接触与净空保护未接管或放宽。

### 47.1 修改及坐标定义

- 新 `tests/hover/lio_tool_observer.py`：复用 ROS `grasp_audit.geometry` 的纯数学 FK，仅订阅 `/lidar_slam/odom` 和 `/so101/joint_states`。关节按六个名称重排；FK 仅用前五个转轴，夹爪开度不改变 `gripper_link`。
- 配置复用 `grasp_frame_audit.yaml` 的 `joint_origins`、`mount_pose`、`tool_pose`；启动时核对机械臂安装外参与 MID360 profile 一致。抓取参考点明确使用 MID360 profile 的 `[0.0045, -0.000218, -0.104] m`，不使用旧方块的 reference_gripper。
- 坐标链：`T_M_G = T_M_B(LIO) @ T_B_A @ T_A_G(q)`。LIO 输出已经是 `map → base_link`，不再次应用雷达外参。原生 TCP 为 `T_M_G @ tool_pose`；抓取参考点为 `T_M_G @ translation(reference_gripper)`，其方向沿用 gripper_link。两者明确分别输出，参考点不是随开度变化的实际双指中心。
- 目标只从第 46 节的 `perceived_target_map.json` 锁定一次，矩阵复制并设为不可写。按关节消息源时间插值 LIO，要求前后样本包围、间隙 ≤0.125 秒、配对年龄 ≤0.2 秒；缺端点时等待，缺帧、过时、非有限值、重复 / 回退时间或错误坐标系拒绝，不外推。
- JointState 当前在 Gazebo UpdateBegin 读取角度，但以 `ros::Time::now()` 标记时间；记录明确为 `ROS_SIM_CLOCK_APPROXIMATE_PHYSICS_SAMPLE`，不声称精确物理步同步。
- `run_lio_hover.py --observe-target` 在目标锁定后追加 3 秒只读 FK 观察。输出 `tool_geometry_used.json`、`tool_relative.jsonl`；真实连杆 / 目标位姿只在停机后的 `tool_offline_score.json` 中核对，不反馈在线计算。

### 47.2 实验与误差

案例：`build/lio_hover/lio_fk_20261029_01/result.json`，success、target_observation_completed、fk_observation_completed 均为 true。采样 73.38–76.38 秒，共 31 帧；全部自有进程已退出，真值仅进入记录器。

| 核对项目 | 本轮最大误差 |
| --- | --- |
| 地图中的原生 TCP | 2.725 mm / 0.0494° |
| 地图中的抓取参考点 | 2.729 mm / 0.0494° |
| 抓取参考点相对目标的估计 | 0.346 mm / 0.1308° |
| 去除机体定位影响后的 FK 链 | 0.0106 mm / 0.00430° |
| 配对消息年龄 | 0.110 秒 |

目标矩阵在全部 31 帧中保持不变；运动与闭爪许可字段均为 false。最后一帧抓取参考点距目标约 **1.110 m**：相对位姿估计误差小，不等于实际已经对齐到目标。当前仍是折叠臂、远距观察姿态。

独立评分使用同一消息参考时刻：实际连杆姿态加相同 TCP / 参考点外参，对照 LIO + 编码器 FK；参考点相对目标再用实际连杆与物体姿态独立计算。只作一次初始地图原点 / 偏航对齐，未逐帧修正。核对结果仅支持该理想传感器仿真的单次折叠姿态观测，不代表展开全工作空间、实机标定精度或抓取成功。

检查：15 项新增数学 / 来源隔离 / 锁定与配对测试通过，12 项既有坐标测试通过，102 组原生 FK / SDF 姿态一致性检查通过（最大位置差 3.533 微米）。还未验证运动中的 ROS 关节时间误差预算。

### 47.3 复现及下一步

沿用第 45 节 ROS / PYTHONPATH 设置；关闭其他仿真后运行：

```bash
PX4_SIM_SPEED_FACTOR=.35 python3 src/modules/arm_control/tests/hover/run_lio_hover.py UNIQUE_CASE --observe-target --gui
```

下一步可将这组末端 / 目标相对关系用于现有 IK 输入核对与接近检查，先验证拒绝和净空判断。悬停许可、精接近几何、夹持 / 携物反馈中的原有真值依赖仍需逐项替换；本轮坐标结果不直接授予接近或闭爪许可。

## 48. LIO 相对位姿的原生 IK 输入与局部接近保护检查（2026-10-29）

沿用第 47 节折叠臂观测，核对当前识别目标到原生 TCP 输入的转换，以及原生只读 IK 和局部净空拒绝。未发送目标设定、准备、接近或闭爪请求；未修改原生控制器、消息或保护门槛。

### 48.1 修改与检查范围

- 新 `tests/hover/lio_ik_inspection.py` 复用 `arm_control ik_check`、`snapshot/assert_unchanged`、生产 SDF 盒体和既有 `box_gap`。输入是带源时间戳的 LIO / 编码器冻结快照；查询期间飞机继续保持悬停续租，冻结输入不代表实时接近许可。
- TCP 目标转换：沿用本 demo 的“物体方向作为期望 gripper_link 方向”约定，先用 `t_M_G = t_M_O - R_M_O · reference_gripper` 定位夹爪连杆，再乘原生 `tool_pose`，最后乘 `inverse(T_M_A)` 得到机械臂坐标中的 TCP 目标。参考点回投核对通过；物体参考点不能直接当作原生 TCP。
- 原生 IK 只求 TCP 位置。本轮未用 XYZ 成功冒充任意六维姿态到位或抓取许可。独立可达对照来自已有 `known_target_approach.json` fixture，明确不代表当前观测物体已可达。
- 局部几何只检查本帧 LIO 机械臂盒体与锁定目标的局部 body / key / handle 形状及已知支撑 profile，全部要求 16 mm，双夹指无接触例外。支撑位姿是假定的静态 fixture 相对关系，不是新测得的支撑面。
- 数值拒绝反例仅平移内存中的目标副本，使其 body 与一个机械臂盒体同心；不移动仿真物体，不修改锁定目标。`box_gap=0` 表示不能证明盒体分离，不当作精确穿透深度。
- 结果明确列出未检查地面、机体、其他障碍、整条轨迹及估计不确定度；`complete_approach_safety_validated=false`，运动 / 闭爪许可始终 false。原生扫掠半径上界与 SDF 核对通过；没有因此授予路线执行权限。
- `run_lio_hover.py` 增加独立 `--inspect-ik` 开关，要求同时启用 `--observe-target`。查询在只读线程中运行，主循环继续续租；旧观测入口保持原行为。`test_lio_ik_inspection.py` 集中覆盖转换和局部保护反例。

### 48.2 本轮结果

案例：`build/lio_hover/lio_ik_20261029_01/result.json`，success=true，全部自有进程已退出。输入快照时间 76.72 秒；原生记录位于 `ik_inspection/`。

| 检查 | 结果 |
| --- | --- |
| 当前识别目标对应 TCP | 机械臂坐标约 `[1.18805, -0.01905, -0.27999] m`，工作空间拒绝，未调用求解器 |
| 已有独立可达对照 | `[0.218480, 0.015814, 0.077299] m`，原生 IK 成功，位置残差 0.0739 mm |
| 越工作空间对照 | 拒绝，未调用求解器 |
| 矩形工作空间内但不可达对照 | 原生 IK 失败，未产生目标设定 |
| 重复可达查询 | 结果与首次相同 |
| 本帧局部净空 | 保守下界约 1.05248 m，通过本帧目标 / 支撑局部检查 |
| 数值侵入反例 | 下界 0，按 16 mm 规则拒绝 |
| 五次查询及最终核对 | 关节目标最大变化 0 rad；目标序号、IK 缓存、请求 ID、phase 和所有权均未改变 |
| 输入与许可 | 锁定目标未改变；运动 / 闭爪许可 false |

当前拒绝是正确结果：飞机仍位于远距观察点，目标在当前机械臂工作空间之外。不能将独立对照 IK 成功或远距局部净空通过解释为已识别物体当前可抓。

检查：9 项新 TCP / 净空测试、13 项既有原生诊断 / 不变性测试通过；编译并执行既有 C++ `test_hover_approach.cpp` 的 25 个保护用例通过。没有新增仿真真值在线输入；真值仍仅由记录器保存并在停机后评分。

### 48.3 复现及下一小步

沿用第 45 节 ROS / PYTHONPATH 设置，关闭其他仿真后运行：

```bash
PX4_SIM_SPEED_FACTOR=.35 python3 src/modules/arm_control/tests/hover/run_lio_hover.py UNIQUE_CASE --observe-target --inspect-ik --gui
```

下一步应从已识别目标反算抓取悬停位，先验证候选位下的原生 IK、期望姿态与净空；当前物理位姿拒绝保持。完整接近路线及地面 / 周边障碍覆盖尚未通过，不进入展开或闭爪。

## 49. 从识别目标反算候选悬停位（2026-10-29）

完成候选几何与只读查询；结果尚不允许飞到候选位或展开。模板预接近可行，但当前折叠种子得到的位置 IK 解方向错误，仍被 5° 判据拒绝。没有通过修改目标、关节限位或净空来掩盖分支问题。

### 49.1 修改及反算方法

- 新 `tests/hover/candidate_hover.py` 复用已验证 demo 的 `LidarDemoPlan.expected_q` 和安装 / 工具外参。设 `B_G = B_A @ FK(q_reference)`，由 `R_M_O @ R_B_G.T` 只取偏航，机体横滚 / 俯仰保持水平；平移按 `t_M_B = t_M_O - R_M_B · (B_G @ reference_gripper)` 反算。目标输入来自锁定点云结果，不读取物体真值。
- 保持这一个机体悬停位；最终抓取参考点在目标参考点，预接近参考点为目标沿自身 +Z 后撤 40 mm。两个 TCP 都通过现有工具偏移转换，不另抬高悬停位，避免把最终点推到 Z=0.4 m 工作空间之外。
- 分开记录模板 FK、既有数值 IK 近似和当前原生 `ik_check` 三类证据。模板 / 数值近似不是原生预演；原生输出 TCP 四元数与期望 TCP 方向比较，不拿位置有解替代姿态到位。
- 预接近 / 最终点均复用第 48 节局部盒体检查，全部 16 mm、双夹指无例外。最终点小净空被拒绝是本轮无接触规则的正确结果，不自动进入精抓接触阶段。
- 仍使用 `run_lio_hover.py --observe-target --inspect-ik`，原五项检查后追加候选查询；主循环维持原观察点的飞行租约，未发布候选航点或机械臂请求。新增 `test_candidate_hover.py` 集中覆盖手算位姿、水平机体、同一悬停位的双 TCP、坐标等变和非法输入。

### 49.2 反馈时间戳最小修复

01 在候选查询之前因 `JOINT_SOURCE_TIME` 退出；停机记录确认有两条关节消息的源时间同为 5.499 秒。旧发布器在物理更新中用 `ros::Time::now()` 盖章，ROS /clock 延迟会令不同物理样本共用时间。

只修改 ROS `so101_gazebo/src/joint_state_publisher.cpp` 的 header 时间，改为 `ros::Time(info.simTime.sec, info.simTime.nsec)`，随后增量重建发布器。重复 / 回退拒绝和原有时效窗口均保留。02 的 3855 条关节消息未出现重复或回退。采样仍发生于 UpdateBegin，独立真值在 WorldEnd；观察器保留近似物理配对标注，不将它宣称为精确同一物理步。

ROS 配套提交：`0145b9d`，已推送。

### 49.3 本轮只读结果

案例：`build/lio_hover/candidate_hover_20261029_02/result.json`，success=true 表示观察与检查完整执行，不表示候选已可执行。全部自有进程已退出。候选数据在 `candidate_hover/`。

| 项目 | 结果 |
| --- | --- |
| 候选机体位置（LIO map） | `[1.31867, 0.02372, 1.59149] m` |
| 候选机体偏航 | −0.000983 rad（约 −0.0563°），横滚 / 俯仰为零 |
| 模板抓取方向残差 | 0.1195° |
| 预接近 TCP（臂坐标） | `[0.15033, 0.01450, 0.35119] m`，工作空间内 |
| 最终 TCP（同一臂坐标） | `[0.16001, 0.01508, 0.39000] m`，工作空间内 |
| 既有数值近似预接近 | 位置残差 0.0990 mm、方向残差 0.6488°、局部净空下界 30.723 mm |
| 当前折叠种子的原生预接近 IK | 位置成功（残差 0.0785 mm），方向偏差 64.594°，拒绝 |
| 当前折叠种子的原生最终 IK | 位置成功（残差 0.0874 mm），方向偏差 53.808°，拒绝 |
| 最终模板点的全 16 mm 检查 | 下界约 2.500 mm，不通过无接触检查 |
| 查询不变性 | 三次查询及最终核对中，目标、关节目标、序号、IK 缓存和所有权均未改变 |

当前原生分支的方向不合格，`native_current_seed_pose_acceptable=false`；候选位尚未成为运动或闭爪许可。原有最终 TCP 距 Z 上限约 10 mm；机体飞行误差、展开扰动及估计不确定度预算仍未验收。此候选也超出原 0.3 m 局部飞行试验范围，未直接扩大飞行边界。

地面、机体、周边障碍、整条展开 / 接近路线仍未覆盖。8 项新增候选测试通过，错误坐标系在 IO 前拒绝；本步未修改 PX4 原生消息或控制器。

### 49.4 复现及下一步

按第 45 节设置 ROS / PYTHONPATH，并先重建包含 ROS 时间戳修复的 `so101_gazebo` 后，关闭其他仿真运行：

```bash
PX4_SIM_SPEED_FACTOR=.35 python3 src/modules/arm_control/tests/hover/run_lio_hover.py UNIQUE_CASE --observe-target --inspect-ik --gui
```

界面仍停在观察位，候选位只写入报告。下一步应复用既有“先受保护展开、再接近”的顺序，核对进入正确 IK 分支的过程与整段净空；不能从当前折叠种子直接跳到这两个点，也不需为本步另加控制器或状态机。

## 50. 复用展开顺序选择原生 IK 分支及整段局部净空（2026-10-29）

本轮完成只读预演，未发送机械臂展开、接近或闭爪请求。沿用既有腕关节展开和首个过渡点后，编译后的原生位置 IK 可进入方向合格的预接近分支；不再用数值近似代替原生求解证据。没有新增消息、状态机或控制参数。

### 50.1 最小修改

- SITL 诊断保留 `arm_control ik_check x y z` 原行为，增加可选的五个只读种子角 `q0 q1 q2 q3 q4`。只接受完整参数个数、有限数和现有关节范围；种子先检查，越工作空间目标不能掩盖非法种子。种子只复制到局部数组，不发布目标，也不修改控制缓存或所有权。输出的 `seed_sequence` 仍表示控制状态快照序号，显式种子另记在外部报告。
- 新 `tests/hover/unfold_precheck.py` 复用 `START → UNFOLDED → ROUTE[0] → 识别目标的 40 mm 预接近点`。先将 wrist_flex 从 +1.3 rad 分 130 步变到 −1.3 rad，随后沿原过渡点逐步调用原生 IK；每次以此前的假想关节状态为种子，笛卡尔请求不超过 1 mm，关节增量不超过 0.02 rad。后两个旧抓取路点不属于本轮无接触路线，不加入。
- 每次原生查询后比较控制目标、目标序号、可见 IK 缓存、phase、请求 ID 和所有权；比较原生报告的 FK 与安装 / 工具外参所用 FK。预接近终点仍要求位置残差小于 0.1 mm、方向误差不超过 5°。正例重复以及非法种子拒绝后再次查询正例，均要求确定性与控制状态不变。
- 全部生产碰撞盒体对识别目标 body / key / handle 及静态支撑 profile 保持 16 mm，双夹指无例外。对每个名义线性关节段，使用经生产 SDF 核对的后代角点半径 `[.51, .45, .33, .20, .13] m`，从端点间距下界减去 `sum(radius * abs(delta_q))` 得到整段真实分离距离的保守下界；不够证明时二分，达到深度上限仍不能证明则拒绝。不是只检查路点。
- 已有观察器入口 `--observe-target --inspect-ik` 自动追加此项，飞行主循环继续维持远距观察位。夹爪命令固定为原控制目标 0.3 rad，整个展开仅存在于诊断种子和报告中。16 项新测试覆盖中点碰撞、半径求和、递归证明 / 深度不足拒绝、非法输入、不变性及原单调工作空间恢复规则。

### 50.2 原生仿真验证

案例：`build/lio_hover/unfold_branch_20261029_01/result.json`，success=true，全部自有进程已退出。LIO 目标 / 编码器冻结快照时间 76.38 秒；路线、244 段证书和 116 次原生查询在 `unfold_precheck/`。真值仍仅在停机后评分，不进入目标、候选或 IK / 净空输入。

| 项目 | 结果 |
| --- | --- |
| 新观测候选悬停位置（LIO map） | `[1.31909, 0.02309, 1.59148] m`，未发布为航点 |
| 当前折叠种子直接查询预接近 | 方向误差 64.786°，继续拒绝 |
| 展开顺序 | 腕关节 130 段 → 既有过渡点 95 段 → 预接近 18 段 → 终点复核 1 段 |
| 正确分支预接近 TCP（臂坐标） | `[0.150469, 0.014494, 0.351151] m` |
| 原生位置 / 方向残差 | 0.0826 mm / 0.7583°，符合原判据 |
| 最终五关节种子 | `[-0.060476, -1.164353, 1.127823, -1.282406, -1.567938] rad` |
| 腕关节展开扫掠下界 | 80.499 mm |
| 过渡点路线扫掠下界 | 25.007 mm |
| 预接近路线扫掠下界 | 25.142 mm |
| 全路线最小采样 / 扫掠下界 | 26.803 mm / 25.007 mm，均大于 16 mm |
| 最大相邻 TCP 位移 | 3.1845 mm，原 20 mm 步长和逐命令工作空间恢复检查通过 |
| 非法种子拒绝 | 越关节范围、越工作空间目标携带非法种子、NaN、参数缺失全部拒绝；之后正例仍相同 |
| 不变性 | 关节目标最大变化 0 rad；锁定目标、序号、可见 IK 缓存、请求与所有权未变；未发闭爪请求 |

检查：SITL 增量编译通过；16 项新扫掠 / 恢复测试、8 项既有候选坐标测试及 13 项既有原生诊断测试通过。本轮未修改 ROS 工作区或 Gazebo 控制器 / SDF。

### 50.3 结果边界与复现

这里的“整段”覆盖固定候选机体、固定开度及静态目标 / 已知支撑 profile 下，从折叠到 40 mm 预接近的全部名义关节段。地面、机体 / 自碰撞、其他障碍、估计不确定度、实际跟踪误差和展开时飞机运动尚未验收；工作空间检查为逐命令端点恢复规则，不宣称连续关节插值的工作空间证明。支撑位置仍来自已知相对 profile，而非新感知平面。

因此本轮证明了原生分支选择及整段局部几何，未证明实际动态展开安全；`motion_permitted=false`、`closing_permitted=false`、`complete_approach_safety_validated=false`。最终接触点的全 16 mm 拒绝仍保留。下一步可在补足候选处机体 / 周边净空后，复用这条顺序做有反馈的无闭爪展开验证，核对实际到位及展开扰动。

沿用第 45 节 ROS / PYTHONPATH 设置，先构建当前 SITL，关闭其他仿真后运行：

```bash
make px4_sitl_default
PX4_SIM_SPEED_FACTOR=.35 python3 src/modules/arm_control/tests/hover/run_lio_hover.py UNIQUE_CASE --observe-target --inspect-ik --gui
```

界面中机械臂仍保持折叠；只读展开结果见该案例的 `unfold_precheck/result.json`、`route.json`、`certificates.json` 和 `native_queries.log`。

## 51. LIO 识别目标接管实际候选悬停导航（2026-10-29）

本阶段把第 50 节的候选位置接入实际飞行，机械臂保持原折叠目标。当前入口是简单场景的短距离候选导航；完整无真值抓取、复杂障碍 SUPER 航线和携物运输仍需后续验收，不以本轮到位代替抓取成功。

### 51.1 修改及数据来源

- `run_lio_hover.py` 增加显式 `--navigate-candidate`，要求同时开启 `--observe-target --inspect-ik`。原观察模式保持原入口；新模式先完成 LIO 点云识别、FK 观察和原生展开分支只读检查，随后才接管候选位置。候选来自锁定的感知目标及既有抓取姿态，不读取场景中物体的位置参数。
- 新 `lio_candidate_route.py` 复用五次平滑轨迹与现有 `KnownFlightSetpoint` 续租：保持折叠，先原地升至候选高度，再水平移动。垂直速度不超过 0.10 m/s，水平不超过 0.15 m/s，偏航采用最短方向且不超过 0.15 rad/s。执行的是预检查的同一条曲线，只重置开始时间，不用更新起点偷偷换一条未经检查的路线。
- LIO 原点仍只由首个 LIO 机体估计确定，经现有外部位姿接口供 EKF 使用。时效、源时间差及 LIO / EKF 同源机体坐标差持续检查，超限即退出；不调用旧 `known_target.local_motion()` 的真值位置 / 速度抵消，也不反向从 MAVROS 对齐 LIO。
- 新 `lio_route_clearance.py` 从配置模型读取几何常量，覆盖机体、起落架、旋翼全转角包络、雷达安装及 FK 折叠臂共 25 个碰撞盒体。忽略模型的世界 pose；核对机械臂关节树、原点、轴、连杆帧及安装关系与生产模型一致，拒绝无法解释的改动。
- 净空输入为 `/cloud_registered` 的静态 LIO 回波。连续路线按平移和偏航上界采样，在拐点分段，从端点点云 / 盒体间距减去整段刚体扫掠量、90 mm 跟踪预算、40 mm 点云 / 体素余量和 5° 姿态位移余量，结果仍须至少 16 mm。飞行中把 LIO 机体位置跟踪误差与实测折叠臂角点偏差之和限制在该 90 mm 预算内，继续核对 5° 姿态及输入时效。
- 这是对已观测静态回波的检查，未知、遮挡及动态空间不因此变成已证明的自由空间；本轮没有宣称完成复杂环境避障规划。最终粗到位要求 LIO / EKF 位置均在 50 mm 内、速度小于 50 mm/s、偏航误差小于 5° 并持续 5 秒，仍不授予闭爪许可。
- 在线图检查要求 `/lidar_slam/odom`、`/cloud_registered` 和飞控外部位姿都只由既有 LIO 适配器发布。`/grasp_audit/truth` 只允许停机评分记录器订阅，旧 Gazebo 视觉定位及机械臂真值反馈转发均关闭。仿真生成雷达 / IMU 和关节反馈属于传感器模型；本轮控制没有读取物体或机体真值 pose。

### 51.2 调试记录与验证

01：`build/lio_hover/lio_candidate_nav_20261029_01/`。路线点云检查通过，37,149 个回波、25 个盒体；扣除当时 60 mm 跟踪预算及其他余量后下界 58.778 mm。爬升约 6.36 秒时触发旧 60 mm 综合预算并退出：最后通过样本的机体位置偏差约 58.46 mm，折叠臂角点偏差约 1.54 mm，姿态仅约 0.38°。误差主要为 X/Y 漂移，不是关节或姿态错误。本轮未据此调 PID。

将该导航预算增加到 90 mm 后必须重新预检查；01 同一数据换算仍剩 28.778 mm，大于原 16 mm。该调整只用于粗导航的已观测路线，不改变机械臂限位、5° 抓取朝向、原生 IK 或精抓净空规则。02 按新预算重新启动独立仿真。

02：`build/lio_hover/lio_candidate_nav_20261029_02/result.json`，success=true，全部自有进程已退出。实际仿真飞行完成“观察 → 点云识别 → 原生只读分支检查 → 先升高再平移 → 候选位保持”，机械臂和夹爪目标未改变；没有发送准备、展开或闭爪请求。

| 项目 | 结果 |
| --- | --- |
| 候选悬停位（LIO map） | `[1.318618, 0.023574, 1.591437] m` |
| 导航及最终保持 | 30.828 仿真秒，末端连续满足粗到位条件 5 秒 |
| 最终 LIO / EKF 位置误差 | 35.926 mm / 20.565 mm |
| 最终速度 / 偏航误差 | 33.458 mm/s / 约 0.103° |
| 注册点云路线检查 | 37,315 个体素代表回波，25 个盒体，390 个路线采样 |
| 扣除所有预算的连续回波净空下界 | 16.293 mm，通过 16 mm；只认证已观测静态回波 |
| 最大实测位置跟踪＋折叠臂角点偏差 | 88.907 mm，小于 90 mm 预算 |
| 最大姿态偏差 | 1.299°，小于 5° 预算 |
| 最终不变性 | 关节目标变化 0 rad；序号、IK 缓存、请求及所有权未变 |
| 在线真值订阅 | 只有 `/lio_hover_audit_recorder`，用于停机后评分 |
| 停机后独立评分 | LIO 全段平均位置误差 2.888 mm；物理末端保持位置误差均值 / 最大值 24.953 / 35.354 mm；点云目标位置 / 方向误差 2.266 mm / 0.101° |

到位是粗导航级别，35.9 mm 残差不能直接当作精抓对齐；本轮未用该判据批准闭爪。下一步须在实际悬停位利用实时 LIO＋FK 更新臂坐标目标，检查 IK / 净空并进入有反馈的展开与预接近，再迁移闭爪及携物判断。原生共享展开入口尚未在本轮实现。

新增 11 项路线 / 原点检查测试和 14 项碰撞模型 / 点云路线测试；合并第 50 节相关回归共 62 项通过。没有新增 ROS / uORB / MAVLink 消息或原生状态机，本轮未修改 ROS 仓库。

### 51.3 余下真值迁移与复现

| 环节 | 当前处理 |
| --- | --- |
| 外部环境与目标位姿 | LIO 雷达 / IMU 定位，点云目标识别；FK 使用安装外参与关节编码器 |
| 复杂障碍 SUPER 航线 | 已有 LIO cloud / odom 接口可复用，但须统一遗留 `world` 与独立 LIO `map` 标签并复测 |
| 实际无闭爪展开 | 下一步复用既有腕展开 helper 到无真值 Approach 入口；本轮仍折叠 |
| 精接近与闭爪 | 旧 FixedGrasp 仍硬绑定真值 source / capability；需替换为 LIO＋FK 几何及诚实的接触 / 夹爪反馈 |
| 抓起与携物 | 静态锁定的目标在抓起后不能作为真实物体轨迹，更不能随 FK 人工绑定；需点云重观测或夹持反馈估计，真值只做停机后验收 |

原 `start_lidar_grasp_demo.sh` / `run_known_target_flight.py` 的完整抓取入口仍含 sim-vision、真值 `Sources` 及物体真值反馈，不能作为本轮无真值入口。Gazebo 抓取 collector 的接触力统计与直接读取 WorldPose 的物体位置 / 支撑净空混在同一反馈中，后续须按来源区分，不能只删除 ROS 真值订阅就称为完成替换。

沿用第 45 节 ROS / PYTHONPATH 设置，关闭其他仿真后运行：

```bash
PX4_SIM_SPEED_FACTOR=.35 python3 src/modules/arm_control/tests/hover/run_lio_hover.py UNIQUE_CASE --observe-target --inspect-ik --navigate-candidate --gui
```

## 52. LIO＋FK 驱动的无闭爪展开与预接近（2026-10-29～30）

### 52.1 当前状态

本阶段使用仿真雷达 / IMU → FAST-LIO、安装外参及关节反馈 FK，已实际完成目标识别、候选悬停导航、原生准备许可、腕展开和臂坐标过渡点。**40 mm 预接近终点尚未验收，未闭爪，也未验证无真值抓起或携物。** 夹爪目标保持 0.3 rad，所有部位仍保留 16 mm 净空。

当前主要阻塞是展开后的真实悬停摆动：它会触发原速度 / 角速度暂停，并消耗目标附近的净空，不能只看最终机体位置或朝向误差。case 24 在远点途中因真实下移触发净空退出；case 26 已复测 EKF/LIO 同时刻配对，但真实悬停运动仍使净空退出；不能据离线配对改善宣称闭环已稳定。运行条件与历史失败见下表。

### 52.2 本轮修改

| 部分 | 改动与保留的约束 |
| --- | --- |
| 原生展开 | `grasp/WristUnfold.hpp` 复用 FixedGrasp 已验证的腕展开；generic Approach 只在已知折叠命令 / 反馈初态选用。展开每 50 ms 最多 0.02 rad，保留关节限位、工作空间单调恢复、悬停许可、反馈时效及扫掠 / 待跟踪预算；展开和 generic Cartesian 都只用一半当前自由预算。 |
| 折叠恢复 | 小步腕圆弧最初会下沉最多 0.814 微米；只在 generic 恢复 TCP Z＜22 mm 时暂将肘角由 1.450 调为 1.449 rad，进入工作空间 2 mm 后恢复。未降低恢复精度或放宽关节限位。名义 130 段连续机体净空下界 16.079 mm；0.25 倍步长 543 段为 18.719 mm。 |
| 动作衔接 | 命令距展开目标＜1 mrad、反馈误差＜30 mrad、关节速度＜0.05 rad/s 后才进入 Cartesian IK；复用既有 phase / `unfold_complete`，未新增消息字段或原生状态机。任务沿用飞行锁定的 task=1，感知目标 / 计划 / 运行分别绑定非零 target / plan / scene。 |
| LIO＋FK 控制采样 | `lio_control_sample.py` 将两帧 LIO 的短时恒定速度 / 角速度预测到真实编码器时刻，最多 120 ms；保留真实源时间和预测量。关节年龄≤50 ms、LIO 年龄≤150 ms，异常运动或时间回退拒绝；ROS 回调领先当前 /clock 时从缓冲选择未过期历史实测；预测所需时长超过 120 ms 时暂不生成样本。只读观察仍要求前后插值配对。末端速度由机体线速度、角速度叉力臂及实测关节速度 Jacobian 得到。 |
| 接近路线 | `lio_approach_probe.py` 适配既有入口：腕展开 → 臂坐标姿态点 `[.15,.015,.325] m` → 地图固定 55 mm 对齐点 → 原 40 mm 终点。原 70 mm 中间点贴肩关节 −1.2 rad 下限，已撤换；终点不改。进入终段前仍要求 5 mm 位置 / 3 mm 横向误差、≤10 mm/s 和原生 IK 5° 朝向检查；仅中间远点按已获授权采用即时条件，不额外等 native Holding。最终到位判据不改。两段地图目标已加 quintic 过渡，峰值前进速度≤10 mm/s、至少 4 s；原生 30 mm/s 能力仍用于机体运动补偿。generic 更新已由每100 ms一次3 mm改为实际elapsed小步：间隔≥16 ms、最多预算20 ms，即每20 ms最多0.6 mm；关节预算同时≤0.2 rad/s，最多0.004 rad，不因暂停或迟到追赶。展开后generic调度20 ms，FixedGrasp原4 ms恢复不变。 |
| 到位与取消 | 终点按实测末端≤5 mm、≤5°、≤10 mm/s 持续 2 s 验收。随后显式取消，核对 Approach 模式、四个绑定 ID、新且已接受的请求、phase 5、闭爪许可关闭和夹爪目标不变。运动故障与成功取消分别记录，不再把原故障重复写成清理错误。 |
| 几何与时延 | 检查目标 body / key / handle、已知相对支撑 profile、注册点云及机械臂到机体 / 起落架 / 旋翼包络；双夹指也无接触例外。外部几何扣除预测 / 定位与细体素余量，自几何消去共同 LIO 误差。批量 `lio_box_gaps.py` 与原计算等价，168 对盒体约 32.04→0.39 ms、数值差＜4e-16 m；整次几何约 36.1→1.55 ms。原生样本 100 ms / 接收 300 ms 时效不变。 |
| 飞行设定点 | EKF与LIO先配对到真实滤波样本时刻，再自首段飞行起使用有界 `p_EKF-p_LIO` 修正（≤60 mm、变化≤20 mm/s）；另保留有界 `v_EKF-v_LIO` 残差（≤50 mm/s、变化≤50 mm/s²），不重复叠加 `b_dot`。粗到位分别核对补偿后的飞控参考与独立 LIO 位置 / 速度，实际路线 90 mm 综合跟踪预算仍受监测。 |

标准外部里程计试验通过 `--fuse-lio-velocity` 选择：两帧 **base_link** LIO 位姿的区间速度，与源时间中点的插值位姿配对，不重盖接收时间、不重复扣雷达杠杆臂。MAVROS 1.20.1 的 ROS→FCU 入口为 `/mavros/odometry/out`，采用合法 LOCAL_FRD / BODY_FRD，关闭旧 VISION_POSE 输入；未修改 FAST-LIO、ROS 源码或新增 PX4 消息。[对应版本 ODOM 实现](https://github.com/mavlink/mavros/blob/1.20.1/mavros_extras/src/plugins/odom.cpp)

- 时间链：PX4 需要 500 个有效往返才保留外部源时间，试验将既有 14580 链路 TIMESYNC 提至 100 Hz。起飞前要求连续三个不同 VO 样本与已发送中点时间＋同步偏移相差≤2 ms，并保留 40～250 ms 源年龄。case 15 后续 305 帧最大匹配误差约 4.8 微秒、年龄 57～61 ms；case 16 预检约 0.5 微秒 / 60 ms。删除了本仓始终报告 0 的 `source_protocol` 误拒条件，保留直接时间核验；不能据起飞初期的到达时间回退断言后续仍未同步。MAVROS PASSTHROUGH 响应使用 ROS 时钟，WallTimer 仅调度。[同步实现](https://github.com/mavlink/mavros/blob/1.20.1/mavros/src/plugins/sys_time.cpp)
- 局部航向：本仓 EKF 对有效 LOCAL_FRD 外部航向有意保留 `yaw_align=false`。原许可因此误报 Flight；现保留全球航向路径，并增加有效局部 EV 路径：FRD、位置 / 高度 / 速度 / 航向融合有效、无 yaw fault、源≤250 ms / 接收≤300 ms、有效单位四元数、两路航向方差≤5°²、航向差≤5°。flags 按变化或 1 Hz 发布，独立要求≤1.2 s；局部位置仍≤300 ms。多 EKF 跟随新鲜 selector 的 primary_instance，且局部位置须晚于切换；原 EKF reset counters 继续锁止。未伪报全球对北或 NED，未放宽原悬停、分配器、反馈和净空保护。
- 脚本瘦身：`sim_scene.py` 统一场景 / 路径 / 进程清理，`grasp/sim_model.py` 统一模型修改；`run_lio_hover.py` 已切断对两个历史大型运行器的导入；`--approach-lio` 自动包含目标观测、只读IK、候选导航与已验证的速度融合前置，减少命令选项。旧 CLI 保留公共名字重导出，移走了重复函数体，未删除仍在用的数学 / IK / 净空核对。

### 52.3 验证、失败结论与运行命令

以下均为无闭爪试验。真值只由 recorder 订阅，在自有进程停止后作独立评分，不能作为运行时控制输入。

| 轮次 | 结果与处理 |
| --- | --- |
| case 01～06 | 修复额外点数误拒、粗体素预算、缩步恢复和几何计算时延；已推进到实际展开 / IK。之后因真实悬停偏差约 55 mm、净空约 15.5 mm 退出，未放宽时效或 16 mm 下限。 |
| case 07～13 | 改用高净空臂坐标过渡点，删除重复速度前馈，将中间远点 70→55 mm，并提前启用同一坐标补偿。展开和首点通过；仍有机体摆动、IK 肩限或净空退出，40 mm 终点未通过。case 11 停机后同窗速度 RMS：LIO 1.16 mm/s、EKF 39.2 mm/s，不能把实际运动误当 LIO 噪声而随意加慢低通。 |
| case 14～17 | 接通标准 ODOM 速度与源时间核验，修复无效元数据检查及局部航向许可。起飞派生速度异常上限与悬停预测上限分开；暂时无有效 LIO 配对时不作到位或入口决策，原 200 ms 源失效条件仍退出。 |
| case 18 | LIO 时间 / 速度融合、候选导航、局部许可、实际展开及臂坐标首点通过。55 mm 点最小位置误差约 5.3 mm；随后摆动消耗净空，扣约 7.6 mm 余量后仅 14.6 mm，按原 16 mm 退出。 |
| case 19 | 保留外部速度融合，恢复有界速度残差。展开阶段机体误差最大约 26.7 mm（上一轮该阶段约 39 mm）；远距段仍出现约 61 mm/s / 0.120 rad/s，触发暂停和净空退出，终点未通过。 |
| case 20 | 仅将 `MPC_XY_VEL_P_ACC` 从 5 改为 3，仍出现约 54 mm/s 和净空退出，未证明改善，**已撤销，恢复 P=5 基线**。 |
| case 21 | 仅平滑地图固定接近目标，仍在远点前因约 60 mm/s 和净空不足退出；**平滑切换未解决悬停问题**。未闭爪，取消和自有进程清理完整。 |
| case 22 | 速度标准差输入为 5 mm/s，但本仓 `EKF/aid_sources/external_vision/ev_vel.h` 的硬下限使实际有效标准差仍≥**10 mm/s**。展开完成，尚未完成首点 Holding；因 `Registered navigation returns lost` 退出，当时未记录实际 age，不能断言是真超时。末帧机体误差约 12.3 mm、速度约 5.8 mm/s；取消确认完整，新版顶层汇总保留 ACK，无重复 cleanup_error。 |
| case 23 | ODOM 与 `EKF2_EVV_NOISE` 输入统一为 10 mm/s，与 case 22 的 EKF 有效下限相同。注册点云改用检查时的新 ROS 时钟，避免复用循环旧 now；≤2 ms 未来时间竞争只暂缓判断，300 ms 源时效不变，异常记录实际 age。其余融合参数、PID、路线和保护不变。远点最小误差约 0.57 mm；两帧达到即时条件（误差1.9 / 2.4 mm、速度10.0 / 7.3 mm/s、横向≤3 mm），仍被 native 额外连续等待挡住；后来净空退出，取消及清理确认，终点未通过。 |
| case 24 | 已应用中间远点即时通过条件，但尚未到达远点便因净空退出。停机后独立核对：138.121～139.080 s 真实机体下移25.67 mm、TCP下移28.22 mm；上下文曾到15.44 mm，原生正确锁止。末段补偿需求最高52.8 mm/s，超过当前臂30 mm/s能力；不是重复扣除净空或显示原点问题。取消 ACK、夹爪保持和自有进程清理确认。 |
| case 25～26 | 修复当前 EKF 与旧 LIO 位置直接相减的问题，复用有界预测配对到实际 `odom.header.stamp`，保留源时刻与最多120 ms预测记录；速度也取同一对LIO位姿。case25发现启动前空样本分支，未发臂动作且完整清理，已修正；case26完成导航、展开与首点，远点途中上下文净空14.64 mm触发退出；未闭爪，取消和清理确认。同时刻配对减少估计误差，但单独未解决闭环悬停。失配时连续粗到位计时重置，保持原连续等待语义。 |
| case27 | 在case26上仅改善generic更新节拍，30 mm/s Cartesian、0.2 rad/s关节累计预算和原保护均不变。完成展开与首点，远点途中原生上下文净空14.96 mm退出；更细节拍单独未解决悬停跟踪。使用单一 `--approach-lio` 入口；取消与自有进程清理确认。 |
| case28 | `EKF2_TAU_VEL` .15→.05 s 的独立试验；完成展开与首点，远点尚未验收便在122.489 s因编码器时间检查退出。停机重放未发现该段编码器源断流，发现 ROS /clock 与回调到达顺序竞争。新版从缓冲选择未过期实测，不重盖时间；当前参数仍是仿真试验，不能预先认定悬停稳定。 |

关键离线定量审计：

| 数据 | LIO | EKF 输出 | 独立真值（仅停机后评分） |
| --- | --- | --- | --- |
| case 18，93.8～143.1 s、493 个同源 100 ms 窗；速度误差 RMS | 1.16 mm/s | 22.0 mm/s | 共同参考；单一时间平移不能消除 EKF 残差 |
| 同段悬停位置误差 RMS / 最大值 | 17.0 / 37.8 mm | 11.0 / 24.6 mm | 17.2 / 37.5 mm，说明真实摆动存在，不能只按 EKF 位置判断稳定 |
| case 19，恢复有界速度差补偿后悬停位置误差 RMS / 最大值 | 10.8 mm / — | — | 11.0 / 25.7 mm（case18为17.2 / 37.5 mm）；速度尾部仍超限，不等于稳定验收 |
| case 22，速度噪声有效下限10 mm/s后的同窗速度误差 RMS | 1.31 mm/s | 17.8 mm/s | 仅该轮展开区间，不能与前轮不同航段作严格A/B；未通过实际终点 |
| case 19 / 20，角速模长 RMS 误差 | 0.00105 / 0.00096 rad/s | — | case 19 两次 LIO＞0.1 rad/s 时真实窗口均超限；未据此加滤波或放宽门槛 |
| case24接近段1231个EKF源样本；位置 RMS / P95 / 最大误差 | 旧latest：3.433 / 5.003 / 7.849 mm；同时间预测：2.948 / 3.579 / 4.486 mm | 使用滤波样本真实时刻，仅改变LIO配对 | 仅停机后录制轨迹重放，不能宣称新闭环改善 |

已完成的相关检查包括：原生 SITL 编译、36 项许可检查、批量几何等价性及展开 / IK / 脚本回归；公共模块抽取相关 138 项检查通过；后续接触模式加入后的集中回归见第53节，原无闭爪逻辑和所有16 mm下限保持。实际展开模型的 plain、vision_obstacles、lio、lio_pillars 四类场景及物理配置，抽取前后逐字节一致。这些检查不能代替 40 mm 实际终点或抓取验收。

关闭其他仿真后，在 Ubuntu-20.04 使用唯一案例名运行当前实验入口（该命令仍只展开和预接近，不闭爪）：

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
cd /home/pcz/PX4-RoboMaster
export PYTHONPATH="$PWD/src/modules/arm_control/tests/hover:$PWD/src/modules/arm_control/tests/grasp:${PYTHONPATH:-}"
PX4_SIM_SPEED_FACTOR=.35 python3 src/modules/arm_control/tests/hover/run_lio_hover.py UNIQUE_CASE --approach-lio --gui
```

结果位于 `build/lio_hover/UNIQUE_CASE/`，以 `manifest.json`、`result.json` 和接近记录共同核对实际参数、源时间、阶段结果、取消确认及进程清理。当前 10 mm/s 速度标准差是理想仿真试验假设，不是 FAST-LIO 内部协方差或实机标定结论；case 22 的 5 mm/s 只是输入值，受原生 10 mm/s 硬下限约束。

### 52.4 剩余边界

1. 原无闭爪入口保持独立；允许接触的精抓入口及其验收见第53节。当前不能宣称完整去真值抓取 / 运输通过。
2. 旧 source1 台架保持原语义；source2 在第53节单独提供接触反馈，明确禁用物体位姿 / 高度 / 支撑净空能力。不能仅去掉 ROS 真值订阅或更名 source 就称为完成替换。
3. 抓起后，静态锁定目标不能跟随 FK 人工绑定成物体轨迹。离开支撑与持续携物判断仍需可靠重观测或明确的夹持估计，真值只用于离线核验。
4. 支撑目前仍是假设已知的相对 fixture，点云无回波、遮挡和动态空间不被证明为空闲。复杂障碍 SUPER 航段及任意原始 LIO 重启也不在本轮验收范围。


## 53. 去除抓取控制中的物体位姿真值（2026-10-30）

### 53.1 修改范围与反馈含义

复用 `run_lio_hover.py`、原生 FixedGrasp 和现有 ArmGraspFeedback / MAVLink消息，不增加消息字段或状态机。新 `--grasp-lio` 入口自动包含目标点云识别、LIO导航、IK核对及原生准备许可；路线仍是腕展开、臂坐标过渡、55 mm对齐、40 mm预接近、精接近，再由原生入口决定闭爪及40 mm工具抬升。

- Gazebo仅在显式 `armGraspContactOnly=true` 时使用 source2。该路径读取接触身份、接触比例和夹指本地压缩力，不读取目标WorldPose，不调用台架几何校验或真值包围盒。只有指定夹指与目标handle碰撞体的接触可算有效夹持，目标body/key接触仍报禁止接触。
- source2只允许CONTACT / FORCE能力。物体局部位置、世界高度、真值支撑净空及世界竖向力一律NaN；没有有效力样本时力也为NaN。接收端拒绝source2夹带真值字段、非法能力、来源切换、重复 / 过期样本和目标重建。source1保持原兼容行为。
- 接近与闭爪仍用LIO＋外参＋编码器FK计算工具与锁定目标关系。闭爪后不把FK工具轨迹伪装成物体轨迹；`actual_lift`保持NaN。抓起判据改为双侧压缩力持续、目标支撑接触消失、LIO/FK工具沿抬升方向实际前进至少25 mm并到达抬升点，随后保持5 s。这是接触与抽离证据，不是直接测得物体高度。失去接触或反馈过期仍退出，取消保持当前夹爪偏置。
- 保留5°闭爪姿态、原悬停许可、关节 / 工作空间、反馈时效、扫掠限制。精抓只允许指定夹指接触handle；夹指到已知支撑至少3 mm，其他对仍16 mm。已知支撑相对fixture仍是场景假设，未知空间未认证。
- 录制器可记录真值，但必须确认所有自有控制和仿真进程停止后才离线读取；独立核对目标确实离开支撑至少25 mm并持续保持。离线评分不推进任何在线动作。

### 53.2 时序与计算修正

1. case29证实source2接入PX4且全部物体位姿字段未知，但没有启动机械臂。原因是把LIO共同运动的时延余量重复扣到了臂—机体相对净空；现只对外部几何扣运输余量，保留真实自几何下界。
2. 夹爪全行程改为一次批量几何：固定盒体只算一次，53个夹爪位置共用同一生产SDF父变换，仍扣除采样间连续扫掠余量。与53次完整计算在两组合法姿态及两种接触模式下数值等价；计算约从89～227 ms降至5 ms。展开阶段不做与闭爪无关的全行程检查，闭爪前仍须检查完整行程。
3. case30已取得准备许可并进入原生展开，夹爪未闭合；93.9 s附近短暂LIO延迟触发120 ms预测上限，取消确认正常。新版不生成超限预测，短缺样时不发布上下文，恢复后只用新实测；原生100 ms源 / 300 ms接收时效保护照常执行。预测函数对超限、异常导数和时间回退的硬拒绝仍保留。

### 53.3 验证与剩余工作

SITL与Gazebo MAVLink插件编译通过。当前集中Python回归164项、原生固定抓取49项、许可36项、接近41项、反馈与旧抓取19组、夹指力13项通过；这些是逻辑检查，不能替代物理抓取验收。

case31已完成实际展开、臂坐标过渡和55 mm对齐（118.980 s，工具误差3.423 mm）。进入40 mm段后原生上下文净空14.93 mm触发保护，未闭爪；取消确认、自有进程清理完成。回放124.821～125.821 s的12帧，最小pair均为活动夹指/handle，盒体下界27.40～33.02 mm；非豁免最小pair为活动夹指/key，77.40～85.42 mm。125.821 s帧扣预测定位和计算运输后普通净空16.220 mm，同一帧精抓策略为57.196 mm；这不是先前触发14.93 mm的同一源帧，不能混为一个数字。

case32仅将已批准的精抓接触策略提前至55→40 mm段（`--grasp-lio` index2）。指定夹指/handle可接触，夹指/支撑仍3 mm，所有其他对仍16 mm；不改变NoClose、悬停、预测、IK、闭爪门控或PID。case32仍在55 mm对齐段退出：115.3 s附近原生几何保护触发，未闭爪，取消 / 清理正常。最后12帧最小对仍全是指定夹指/handle，精抓策略重算其他保护下界58.00～72.78 mm，表明应从55 mm对齐段开始应用已获允许的接触策略，而非到40 mm途中才启用。

case33只作两项一致性修正：grasp从index1起使用同一精抓接触规则；到位对齐比较实测OPEN夹爪参考与锁定目标+2.3 mm X的固定seating目标（等价于目标朝向下的close reference），而非未经偏移的中心。阈值仍3 mm、方向仍比锁定目标、速度仍来自相同实测参考；绝不把far/pre移动途经点当闭爪目标。新测试核对两个不同途经点的闭爪参考不变，NoClose原参考不变，目标未修改。case33通过了先前夹指/handle净空瓶颈，但113.45 s在55 mm段肩关节到−1.2 rad下限后IK拒绝（误差0.890 mm），原生fault4；未闭爪，取消 / 清理正常。未放宽限位或IK精度。无真值抓取尚未验收，携物导航到指定点仍未接入此入口。携物后物体姿态没有被直接观测，因此不能据接触成功宣称携物包络或复杂障碍安全已经验证。

在第52节环境设置下运行：

```bash
PX4_SIM_SPEED_FACTOR=.35 python3 src/modules/arm_control/tests/hover/run_lio_hover.py UNIQUE_CASE --grasp-lio --gui
```

审阅同一案例目录中的 `contact_preflight.json`、`lio_approach/result.json`、`result.json`；只有全部自有进程已停止且完成物理评分时，才会生成 `grasp_offline_score.json`。不要把原生Done或命令发送成功当作物体实际抓起。


### 53.4 速度补偿对照与精接近点云来源（2026-10-31）

case34在case33上只把速度补偿slew由0.05提到0.20 m/s²，幅值仍50 mm/s；位置补偿slew20 mm/s、PID与全部保护不变。已完成55 mm对齐、40 mm预接近并进入精接近（142.903 s）；未闭爪。168.5 s原生Contract退出的具体原因是推力分配保护（hard_hover=0、denial16），context / lease / binding / geometry均有效，sample年龄17 ms、接收年龄8 ms。单侧夹指压缩反馈0.07871 N、另一侧0；取消与清理正常。不能把单侧接触或移动途经点的0.95 mm最小误差称为最终到位 / 抓起。

前两轮停机数据重放表明，旧速度slew在far阶段99.47～100%更新被裁剪，50 mm/s幅值很少触顶；速度补偿剩余误差均值10.54 / 16.09 mm/s。离线重放用200 mm/s²可降至1.84 / 2.31 mm/s，但这不等于真实飞行改善。case34推进更远是单轮实测，仍未通过实际抓起，保留为有待复测的demo参数，不作为PID或实机标定结论。

继续检查发现几何输入存在分辨率不一致：`fastlio_sim.yaml`的surf/map体素均0.15 m，dense_publish_en=false。实际FAST-LIO `publish_frame_world()`在false时发布feats_down_body（PCL 15 cm体素质心），true时发布feats_undistort。下游2 mm格子筛选不能恢复已经丢失的细节，因此旧输出不支持按4 mm点误差做夹爪附近净空判断，亦不能仅通过放大handle掩膜忽略这些点。

本轮仅在LIO精接近入口生成案例自己的`fastlio.launch`，加载原配置后设置全局`/publish/dense_publish_en=true`。既有FAST-LIO节点、IMU/点源、时间戳、SLAM surf/map滤波参数均不改；发布改用现有去畸变原始点。`roslaunch --dump-params`逐键核对，原/新启动参数仅该一个布尔值不同。起飞前再次检查参数并保存`lio_cloud_source.json`；未启用dense就拒绝精接近。原共享launch、ROS工作区未修改。

相关测试核对案例副本不修改源文件、参数覆盖在原配置之后、重复生成无重复覆盖、错误节点作用域拒绝；集中Python回归164项通过。case35已确认起飞前dense参数生效、surf/map仍0.15 m；但94.9 s初始展开时因上下文超时退出（sample_age117 ms、receipt116 ms、denial2），未闭爪，取消 / 清理完成。该轮不是Allocation拒绝，不能合并为上一轮推力问题。

原始点源重放进一步支持修正：case34精评222点的拟合RMS0.455 mm，其中25个handle点最大表面残差0.651 mm。旧registered约束点超出handle边界13.319 mm、离最近原始精评点14.070 mm。固定同一LIO/FK姿态、同一几何门槛并置prediction=0以仅隔离点源，完整夹爪行程下界：旧质心20.21 mm、原始点35.67 mm；数值推进至固定机体最终点后分别17.86 / 33.16 mm。此离线比较不是在线通过，也没有放大4 mm掩膜。

### 53.5 状态快照时刻与重测（2026-10-31）

`HoverPermitAdapter.cpp`原先先读hrt时刻，再复制allocator、motors、selector等快照。SITL在复制期间可能走到下一tick，新状态会被旧时刻误判为未来。现把所有快照与请求复制完成后再取统一检查时刻；selector primary绑定、实例切换、reset、所有新鲜度/分配阈值均保留，没有容忍未来时间或重盖样本时间。Allocation拒绝上升沿增加两行本地诊断，区分age、achieved、masks与motor_max，不新增消息/状态机。SITL编译与既有许可逻辑检查通过。

这不是case34已证实的唯一根因：ULog在该区间未录到>=0.95的电机值（最大0.93395）或失配，但motor只100 ms、allocator只200 ms记录，无法排除168.457 s的瞬态。后续据新日志定位，不据此提高0.95阈值。

case36将仿真倍率由0.35降到0.25，为原始点处理和ROS调度留出墙钟计算时间。所有运动速率仍按sim-time、100 ms上下文与120 ms预测上限不变；尚未完成闭爪、抓起和保持的在线验收。现有16 mm / 3 mm净空、3 mm闭爪到位、5°、反馈时效、限位及推力分配保护不变。


### 53.6 样本配对与重复观察收束（2026-10-31）

case36在0.25倍率、状态快照修复下仍于91.5 s初始展开退出：原生context sample年龄105 ms、receipt36 ms，表示原生接受时该样本已约69 ms；不是Allocation拒绝。取消、夹爪偏置保持和进程清理通过，未闭爪。不能仅凭这两个年龄断言69 ms全是某次IPC耗时。case35/36的LIO源仍100 ms、joint仍20 ms，但接收出现约62～63 ms突发延迟；几何wall最大21 ms，满缓冲audit锁内FK重放最大1.55 ms，均不足以单独解释故障。

case37进行不改保护的传递链修正：

- 控制配对从真实缓冲中选择同时满足当前时间、关节年龄≤50 ms、LIO预测≤120 ms的最新编码器帧。旧代码先挑更新但不可配对的帧，可能丢掉仍有效的历史配对。没有重盖时间、重复刷新旧context或延长任何界限；无合法配对仍暂缓，由原生100 ms看门狗处理。
- probe启动后停止主循环重复的只读FK审计和折叠臂旧值检查；候选飞行全过程仍保留该检查。此后折叠偏差诊断为null，真实臂反馈/净空保护继续由probe与原生控制器执行。
- 同一案例新增`lio_approach/timing.jsonl`，记录每次成功context的源时间、采样/几何wall耗时、发送/返回ROS时刻与IPC wall耗时，以及每次诊断RPC起止。暂缓/重复样本等早退不落该日志，故空窗仍须结合bag核对，不能把它当全部线程调度跟踪。

新增配对边界/旧帧过期拒绝测试通过，相关67项测试和Python语法检查通过。case37在0.25倍率使用同一原始注册点源重测，实际闭爪 / 抓起尚未验收。


### 53.7 拒绝回执和同一时刻的飞控配对（2026-10-31）

case37没有取得固定抓取所有权，却一直等待start回执：旧adapter仅在owns时发布status，被拒绝的request_id/result未对外可见。已人工中止，所有自有进程停止；旧取消逻辑还报未确认，不能把这轮写成取消ACK通过或机械臂实际展开。

修正复用既有status：`reportsStatus()`在当前fixed上下文收到请求后即报告拒绝，caller避免再以默认generic状态覆盖；已有generic owner优先。未持权时motion_authorized必须false，source2的actual_lift保持NaN。未取得所有权时，只有本次新cancel回执明确Idle/Rejected/noOwner且全部关节目标未变，外部才报告motion_not_acquired；旧回执或目标变化均拒绝，该正反用例已通过。

直接gzserver曾绕过标准ROS launch的时钟设置；本demo改用`/gazebo/pub_clock_frequency=100`，与已安装empty_world.launch默认一致。[Gazebo ROS参数读取实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp)。运输余量和计算年龄额外保守计入10 ms时钟周期，原生100 ms样本限制不变。case38出现正常回调比100 Hz本地clock略早，latest filter帧暂时不可配对并反复清零8 s观察位到位计时；尚未启动臂，已中止。

case39使用真实飞控历史帧配对：保留最新源的原[-20 ms,200 ms]有效性检查，再从50帧缓冲选不晚于当前clock的最新原消息；选中帧仍须通过原200 ms检查，不重盖时间、不隐藏过期。只读FK同样只选择不晚于clock且有LIO前后括号的真实joint帧。没有缩短原到位连续等待，也未改变悬停/闭爪阈值。新增测试证明对象和源时间不变、未来帧未采用、过期仍由原检查拒绝。

当前相关Python99项、原生Fixed50项、SITL编译通过。case39继续验证同一无物体位姿真值链；在线闭爪、实际抬起和保持仍待这轮结果，不以静态IK或命令回执代替物理验收。
