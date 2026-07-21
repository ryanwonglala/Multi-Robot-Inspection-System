# 排障记录汇总（Troubleshooting Log）

> RoboInspect 多机巡检系统 · 排障/Debug/问题记录索引
>
> 本文件由散落在各处的内部排障笔记（早期 `dbg/` 逐次记录、AI 协作 cycle 往返实录、DEVLOG 排障流水、
> 手动中止运行记录）**汇总整理**而成，作为对外可分享的单一索引。原始明细文件仍保留在本地
> `doc/`（`_archive/ros_ws_doc_dated/*/dbg/`、`ai_collab/cycle_*/RECORD.md`、`DEVLOG.md`）。
>
> **时间跨度**：2026-05-26 ～ 2026-06-24（仿真主线封版）。
> **平台演进**：早期为 TurtleBot4 / `tb4_ws`（05-26~05-31 搭建期），后迁移至 TurtleBot3 burger 双机
> （`tb3`=侦察机/Robot B、`arm`=操作臂/Robot A）。

---

## 阅读方式

每条记录采用统一结构：

- **症状** — 观察到的失败/异常现象
- **根因** — 诊断出的原因（若已定位）
- **修复** — 解决该问题的改动
- **状态** — `已修复` / `已缓解` / `已回退` / `已知弱项`（留档不修） / `观察项`

分类见下方速览表，可按需跳转。

---

## 速览索引

| 类别 | 条目数 | 关键问题 |
|---|---|---|
| [A. 早期仿真搭建期（TB4）](#a-早期仿真搭建期tb4阶段) | 8 | Gazebo 退出、teleop 不动、SDF 可编辑性、地图安装路径 |
| [B. 多机 / TF / 命名空间](#b-多机--tf--命名空间) | 4 | odom TF 漏发根 /tf、嵌套 include namespace 覆写、QoS 不对齐 |
| [C. SLAM / 建图 / 定位](#c-slam--建图--定位) | 7 | AMCL 长轴失约束、贴箱失锁、墙角画薄刮蹭、拍照朝向瞬态偏移 |
| [D. 导航 / Nav2 / 避障](#d-导航--nav2--避障) | 13 | 二元代价地形贴墙崩溃、scan 死信地址全聋、B 轴取消/超时竞态、半栈 server_unavailable |
| [E. 仿真 / Gazebo / SDF 模型](#e-仿真--gazebo--sdf-模型) | 7 | 双车激光互不可见互撞、资产穿地、相机保真度、D436 shape 不符 |
| [F. 感知 / 异常检测 / photo-diff](#f-感知--异常检测--photo-diff) | 16 | 激光检测器与 AMCL 根本矛盾（废弃）、相位相关伪峰、no_baseline 伪装、配准偏移墙边 FP |
| [G. 任务分配 / allocator](#g-任务分配--allocator) | 7 | 撞家 bug、门口会车卡死、返航兜底超时、路径代价分配 |
| [H. 报告 / evidence](#h-报告--evidence) | 5 | 报告未合并、取证照格式、报告堆积清理、RViz 截图剥离 |
| [I. 构建 / 环境 / 工具链](#i-构建--环境--工具链) | 8 | 地图路径硬编码、残留 gzserver、GUI 吞行死循环、沙箱禁 socket |
| [J. 其他 / 方法论](#j-其他--方法论) | 4 | 检测-处置物理矛盾、AI 署名清理、死代码清理 |

贯穿性的**救援铁律 / 排障方法论**见文末[附录](#附录贯穿性救援铁律--方法论)。

---

## A. 早期仿真搭建期（TB4 阶段）

> 来源：`doc/_archive/ros_ws_doc_dated/2026052{6,7},20260531,20260603/dbg/`。此阶段基于 TurtleBot4 / `tb4_ws`，
> 是整个项目最早的仿真世界与建图搭建，后续迁移到 TurtleBot3 burger 双机。

**A-1 手动 launch 后 Gazebo 几秒退出、RViz 报 map frame 缺失**（2026-05-27）
- **症状**：Gazebo 启动显示自定义地图+机器人数秒后 GUI 退出；RViz 保持但报 `map` frame missing、机器人模型加载不正确；旧日志出现畸形 bridge 话题 `/world//home/ryan/tb4_ws/.../map.sdf/...`。
- **根因**：bridge 收到的是世界**文件路径 / `.sdf` 值**而非 Gazebo 世界名；且自定义 world 启用了 world-level `Sensors`(ogre) 插件，与 TB4 相机/LiDAR 栈或 Harmonic 渲染假设冲突。
- **修复**：`map.launch.py` 显式向官方 spawn/bridge 传 `world`；world 文件去掉 world-level `Sensors`(ogre)、补上官方一致的 `Contact` 系统插件。（`已修复`）

**A-2 diffdrive_controller cmd_vel 超时警告**（2026-05-27 跟进）
- **症状**：world 修复后 Gazebo 不再退出，但反复 `Ignoring the received message ... older than current time by 0.5s`。
- **根因**：速度指令到达时间处于/略超 `cmd_vel_timeout: 0.5` 门限。
- **修复**：判定为空闲/低频发布期的噪音——机器人正常运动即可忽略；仅当不动/卡顿才需追指令发布率/sim time/bridge 时序。（`观察项`）

**A-3 Gazebo GUI 键盘 teleop 不动车**（2026-05-27）
- **症状**：GUI 里 `/cmd_vel` teleop 键盘控制不动，但直接 ROS 速度指令能动。
- **根因**：Gazebo GUI teleop bridge 接线不可靠。
- **修复**：本项目路径改用 ROS 侧 teleop / 直接 `/cmd_vel`，不依赖 GUI teleop。（`已缓解`）

**A-4 自建 ctl.py teleop GUI 不动车**（2026-05-27）
- **症状**：新建的 tkinter teleop GUI 打开但不动车；手动 `TwistStamped` 指令可动。
- **根因**：GUI 发的是 `geometry_msgs/msg/Twist`，而官方 Create3 Gazebo bridge 期望 `/cmd_vel` 为 `TwistStamped`（`irobot_create_gz_bringup` 确认）。
- **修复**：`ctl.py` 改发带 header 时间戳的 `TwistStamped`。（`已修复`）

**A-5 手动 SLAM 右上角区域漂移**（2026-05-27）
- **症状**：机器人到达右上（blue/orange/purple 区）时手动 SLAM 出现漂移。
- **根因**：外墙窄道有长平行墙段/重复短墙角/LiDAR 特征贫乏，局部扫描匹配歧义。
- **修复**：加一条干净内 east 边界墙（`x=8.75, y=-6.0~6.5`）挡住外墙窄带 + 两段短水平锚墙（`x=8.2, y=2.55 / -1.0`）打断长墙制造可辨特征。（`已修复`，需人工复跑复核）

**A-6 SDF 墙体作为单一 model 的 links 无法在 GUI 单独编辑**（2026-05-27）
- **症状**：所有墙段作为一个 `<model>` 内的多个 `<link>`，Gazebo GUI 难以选中/复制/移动单段墙。
- **根因**：旧结构为紧凑 SDF 文本优化，不利手动 GUI 编辑。
- **修复**：一次性 XML 迁移——每段墙拆成独立 `<model name="wall_X">`（pose 从 link 提到 model），得 43~44 个独立 wall model；解析校验 `old grouped model present False`。（`已修复`。注意墙仍 `static=true`，GUI 若拒绝移动静态模型则直接改 SDF pose。）

**A-7 GUI 手动保存覆写 src `map.sdf` 但 ROS 仍加载旧图**（2026-05-27）
- **症状**：Gazebo GUI 直接开世界显示新图，但 `ros2 launch` 仍加载旧图。
- **根因**：`gz sim src/.../map.sdf` 读源文件；而 `ros2 launch` 经安装的包 share 读 `install/sim/share/sim/worlds/map.sdf`，两者未同步。
- **修复**：编辑 `src` world 后必须 `colcon build` 重装并 re-source 再经 ROS 启动（rebuild 后两文件 sha256 一致）。（`已修复`／流程规约）

**A-8 task_layer_v020 综合调试笔记**（2026-06-03）
- 初始导航失败实为**初始位姿朝向/坐标对齐**问题而非地图不可用，对齐后 Gazebo/RViz 匹配通过；
- 移除基于 **LiDAR 点数**推断障碍异常的 v0.2 尝试（结果不稳定、不直观）；
- 候选巡检位姿从固定 ±0.5m 偏移改为**按区域 bounds 展开**（`candidate_spread_ratio:=0.35`），避免 north_hall 等大区候选全挤中心；
- 移除临时 per-goal 超时/取消（会打断"慢但可达"的路线），改让 Nav2 结果自然完成；
- 返航从直接回坞（撞充电模型）改为**前方 0.6m 安全 standoff**（非精密对接）；
- `task_gui` 首启失败因 `use_sim_time` 重复声明，移除 `TaskGuiNode` 里的手动声明。（均 `已修复`）

---

## B. 多机 / TF / 命名空间

**B-1 gazebo diff_drive odom TF 漏发到根 /tf**（2026-06-12，P0-3）
- **症状**：`-robot_namespace` 下 diff_drive 的 odom TF 仍发到根 `/tf`，双机 TF 冲突。
- **根因**：gazebo diff_drive 插件不受 namespace 约束（B5 风险预警应验）。
- **修复**：sim 包内置 `turtlebot3_burger_cam_ns/model.sdf`，diff_drive 加 `/tf:=tf` 重映射。验证：根 `/tf` 零发布者，`/tb3/tf` 与 `/arm/tf` 各自有 odom TF。（`已修复`）

**B-2 嵌套 include 内部 TimerAction 惰性读全局 namespace 被覆写**（2026-06-13，P0-4）
- **症状**：双机时第二台覆写全局 namespace，两套 Nav2 挤进同一 ns。
- **根因**：嵌套 include 的内部 TimerAction 惰性（非原子）求值全局 namespace。
- **修复**：multi_nav 改为每机**字面量内联** nav2 include（原子求值）。（`已修复`）

**B-3 gazebo.launch.py 空串 params_file 污染全局作用域**（2026-06-13，P0-4）
- **根因**：`gazebo.launch.py` 先声明 `params_file` 默认空串，污染全局作用域。
- **修复**：multi_nav 必须显式传 map / params_file。（`已修复`）

**B-4 amcl_pose 订阅端 QoS 不对齐收不到**（2026-06-13，P0-4）
- **根因**：`amcl_pose` 为 `transient_local` 锁存，订阅端 QoS 未对齐。
- **修复**：订阅端 QoS 对齐 `transient_local`。（`已修复`）

---

## C. SLAM / 建图 / 定位

**C-1 SLAM 墙角画薄致刮蹭 + AMCL 漂移**（2026-06-13，P1-3）
- **症状**：贴角通过时物理刮蹭 + 轮滑致 AMCL 漂移 0.6m。
- **根因**：SLAM 地图把三处墙角画薄（与 Gazebo 真值差 5~15cm）。
- **修复**：按 `gz model -p` 真值把 `wall_corridor_west/east`、`wall_hall_south_right` 画实（两份 pgm 同改）；arm 床位南移到 `(-4.8,-2.95)` 离开北墙窄带。验证：arm 连续 4 个全周期 100% 通过。（`已修复`）

**C-2 east_hall 长轴超激光直径致 AMCL 沿轴失约束**（2026-06-12，P1-4）
- **症状**：east_hall 中心站位 AMCL 沿轴失约束（std 0.76+）发散。
- **根因**：east_hall 长 7.2m 超激光直径，单观测点无法约束。
- **修复**：大区域用双观测点（`east_hall_w/e` 4.0/7.5:0.85），后续 viewpoints 机制承接。（`已修复`）

**C-3 贴箱刮蹭致 AMCL 失锁 3.9m 全面瘫痪**（2026-06-12/13）
- **症状**：投箱放在路径上→贴箱刮蹭→轮滑→AMCL 失锁 3.9m→代价地图画鬼墙→全面瘫痪。
- **根因**：障碍放在行进路径上导致物理接触与定位失锁。
- **修复**：测试编排规约——投箱别放路径上；带箱门禁一律 1 倍速；救援=gz 传送+initialpose+清双 costmap。（`已缓解`／规约）

**C-4 AMCL 静止不更新协方差冻结**（2026-06-12，P1-4）
- **症状**：到点后位姿/对齐门不过（协方差冻结在走廊穿行的高位）。
- **根因**：AMCL 静止时不更新，协方差保持高位。
- **修复**：harness 到点若门不过先原地慢转一圈重试（storage 实测 std 0.388→0.238 救活）。（`已修复`）

**C-5 central_hall 地图配准系统性北偏 ~0.23m**（2026-06-12，P1-4）
- **症状**：central_hall 事件坐标带 ~0.23m 北向偏置（坞区仅 0.04m）。
- **根因**：该区地图配准系统性北偏（gz 真值三点验证）。
- **修复**：已知边界，记录留档（答辩 Q&A 素材），未强修。（`已知弱项`）

**C-6 拍照朝向瞬态偏移污染对齐**（2026-06-13，P1-5v）
- **症状**：AMCL yaw 自旋后瞬态偏 ~0.5rad，污染对齐+投影。
- **根因**：自旋后 AMCL yaw 未收敛即拍照。
- **修复**：`corrected_capture_yaw`——复用废弃激光检测器地图机件，用 alignment_ratio 对 360° 墙结构爬山把拍照朝向锚到真值。（`已修复`）

**C-7 冷启动后首个导航目标偶见无结果**（2026-06-13）
- **症状**：冷启动后首个导航目标偶见无结果（1 次，未复发）。
- **修复**：未复发，列为观察项。（`观察项`）

---

## D. 导航 / Nav2 / 避障

**D-1 tb3 东向路线 nav ABORTED（二元代价地形贴墙崩溃）**（2026-06-13，P1-3）
- **症状**：tb3 东向约 50% 概率 nav ABORTED；基线 6 轮东向往返仅 2/7 段成功；`MPPI Optimizer fail to compute path`×32 + `NavFn failed to create plan`×24。
- **根因**：二元代价地形——`inflation_radius 0.25` 之外代价为零，最短路径必然贴膨胀带边缘（贴墙 25cm），地图误差+AMCL 抖动吃光余量后在窄口崩溃。
- **修复**：代价地形改缓坡谷地——`inflation_radius 0.25→0.7`、`cost_scaling_factor 4.0→2.5`（两个 costmap）。对照：基线 2/7 段→梯度版 10/11 段成功。副作用：回坞精度降至 ±20cm（容差内）。（`已修复`）

**D-2 scan 话题解析进节点子命名空间死信地址（costmap 全聋）**（2026-06-13/14，PR #11）
- **症状**：双 costmap 自 namespace 化以来对 scan **全聋**，障碍从没进过任何代价图，机器人"无视障碍"直接撞穿。
- **根因**：观测话题 `scan` 解析进 costmap 节点自己的子命名空间 `/<ns>/global_costmap/scan` 死信地址。
- **修复**：`nav2_inspection.yaml` 两处改 `<robot_namespace>/scan`（nav2_bringup ReplaceString 原生替换）。三层验证：`/tb3/scan` 出现两 costmap 订阅者 + 箱处 global_costmap cost 99-100 + 带障碍导航 SUCCEEDED。（`已修复`，已 merge main）

**D-3 障碍物在区域中心（导航目标）致挂死/绕圈/碰撞**（2026-06-13/14，Cycle 1）
- **症状**：巡检挂死（allocator 超时被杀）、贴障碍蹭/碰撞、日志死循环 `follow_path Abort → spin/backup "Collision Ahead" → 重规划`。
- **根因**：runner 候选点先试区域中心(=障碍位置) + 发目标前无代价图预验证 + `send_goal_and_wait` 用无超时 `spin_until_future_complete`。
- **修复**：B 轴韧性——① 单目标墙钟 backstop 120s + 途中 ~2s 复查目标格；② `candidate_is_clear()` 发目标前查 costmap 跳致命候选、失败后用更新图重筛；③ 途中发现即取消（车停在传感器距离）；④ attempts 2→4。（`已修复`）

**D-4 candidate_is_clear 把静态墙体误判为动态障碍**（2026-06-14，Cycle 1 Codex blocker #1）
- **症状**：默认单机 `return_home:=true` 返航目标（墙边 dock）~2s 后被 `goal_blocked` 误取消。
- **根因**：`candidate_is_clear()` 读合成 global costmap，无法区分静态墙体与动态障碍；home 目标距南墙 ~0.225m，落入 0.7m inflation。
- **修复**：costmap 与 static map 做差——仅当某格 costmap 真致命（`lethal_cost 90→100`，排除 99/inscribed 与 inflation 渐变）且 static map 上为自由地面（`static_free_max=50`）才算动态障碍；新增订阅 `<ns>/map`。（`已修复`）

**D-5 取消后未确认旧目标终态致抢占竞态**（2026-06-14，Cycle 1 Codex major #2）
- **根因**：`_cancel()` 只等 cancel response 5s，不检查 future 完成/返回码、不等原 result_future 进终态；下一候选可能与旧目标抢占重叠。
- **修复**：`_cancel` 检查 `goals_canceling` + 有界等待原 result_future 终态（≤5s），记 `cancel_terminal`；引入 `safe_to_continue` 不变量——非终态确认则 latch `_nav_aborted` 停派发。（`已修复`）

**D-6 goal request 阶段无超时可永久挂死**（2026-06-14，Cycle 1 Codex major #3）
- **根因**：`send_goal_async()` 后仍用无超时 `spin_until_future_complete`；server 已发现但 goal request 无响应时永久挂死。
- **修复**：发送阶段加 `server_timeout_sec`，未完成返回 `send_timeout`。（`已修复`）

**D-7 send_timeout 迟到 goal 成无管理目标**（2026-06-14，Cycle 1 复审 major #2）
- **根因**：`send_timeout` 后 goal request 仍在 rclpy `_pending_goal_requests` pending；server 超时后返回 accepted 会创建无 handle 的服务端目标，产生"迟到旧目标+新目标"竞态。`send_future.cancel()` 只让客户端忽略，不撤销服务端请求。
- **修复**：`_handle_send_timeout`——给迟到响应有界第二次机会，迟到 accepted 立即取消并确认终态，无法确认则 `safe_to_continue=False` 触发 abort。（`已修复`）

**D-8 栅格换算 int 截断 + 越界乐观放行**（2026-06-14，Cycle 1 Codex minor #4）
- **根因**：世界→栅格用 `int()`，负非整数坐标向 0 截断；目标完全在地图外时不检查任何格返回 True。
- **修复**：改 `math.floor`；新增 `_grid_value` helper，越界返回 None（差分时保守不计动态障碍）。（`已修复`）

**D-9 stall-abort 看门狗在终点原地转向误判**（2026-06-13，已回退）
- **症状**：加无进展看门狗(25s/0.12m)后，空旷可达区做最后转向时平移≈0 被误判取消（restricted_gate/storage 误 nav_failed）。
- **根因**：仅基于位移的 stall 检测在终点调姿时误判。
- **修复**：加"接近目标 0.40m 豁免"重测，但仿真被高负载跑废（narrow_passage 373s，CPU 饥饿）结果不可信 → 用户叫停回退。（`已回退`）

**D-10 大障碍压区域中心时固定偏移改址不感知障碍范围**（2026-06-13）
- **症状**：1.22m pallet 的 0.7m 膨胀晕罩住所有环形候选点→改址擦边/全候选不可达 nav_failed/卡边缘很久。
- **根因**：改址环形候选点是按区域尺寸的固定偏移，不感知障碍实际范围。
- **修复**：正解=用检测层给出的障碍范围做 standoff（归 P1-6 detection-driven）；B 轴代价图预验证+途中取消+优雅放弃已缓解。（`已缓解`）

**D-11 定位未收敛期 bt_navigator rejected 所有目标**（2026-06-14，Cycle 1/2）
- **根因**：readiness 问题（非 bug）——AMCL 未收敛即发目标，bt_navigator 全 `rejected`。
- **修复**：发任务前先等 `/tb3/amcl_pose` 与 `/arm/amcl_pose` 都有输出（~30-40s）。（`已缓解`／规约）

**D-12 departure not confirmed 致 tb3 全区 nav_aborted（偶发调度不稳）**（2026-06-16，Cycle 5）
- **症状**：S1 round2 tb3 全部 5 区 `nav_aborted`，`aborted_unsafe_nav_state`，日志 `departure not confirmed after 90s`。
- **根因**：goal 握手/调度/启动状态/动作确认不稳定（非 photo-diff）。
- **修复**：判为已知偶发（暂不纠结），未修。（`已知弱项`）

**D-13 Nav2 半栈起来致 navigate_to_pose server_unavailable 被误计为 nav_failed**（2026-06-15，来源：项目记忆）
- **症状**：Ryan 首次 GUI 测试时 arm 的 Nav2 只起来一半（只有 `controller_server`，没有 bt_navigator/planner/amcl，arm 从未定位、无 `/arm/amcl_pose`），`/arm/navigate_to_pose` 无活动 server；每次 arm 导航都 `server_unavailable`，被 runner 误计为候选失败尝试 → 全区 `candidate_attempt_limit_reached/nav_failed`，看似"路被堵"，**掩盖了真正的半栈起因**。
- **根因**：bringup flake（Nav2 部分起栈）+ runner 未区分"server 不可用"与"候选被堵"。
- **修复**：runner latch `_nav_server_unavailable` + run_once 预检 + 诚实区分区域状态 `nav_server_unavailable`；allocator `preflight_nav()` 在派发前检查各机 NavigateToPose server 在 `nav_ready_timeout_sec`(20s) 内就绪，宕机机器人大声记录、不派发、mission 视情况 abort。（`已修复`，在 parked 分支 `fix/nav-readiness-guards`。运维规约：派发前确认 `ros2 action list` 中双机 `navigate_to_pose` + `/arm/amcl_pose` 均在，半死栈就 pkill 清场重启。）

---

## E. 仿真 / Gazebo / SDF 模型

**E-1 burger 碰撞体仅 LDS 圆柱致双车激光互不可见互撞**（2026-06-12）
- **症状**：双车会车互撞；对方在激光里仅 2-7 束回波，代价地图膨胀无从下手。
- **根因**：CPU ray 激光打碰撞体，burger 碰撞体里只有 5cm LDS 圆柱伸到 0.171m 扫描面（基座箱只到 0.14m）。
- **修复**：`model.sdf` 加足迹半径(0.09m)可见性碰撞圆柱（own 激光 `range_min 0.12` 看不到自己）+ allocator 出发错峰。验证：4 区域并发全程最小间距 0.522m。（`已修复`）

**E-2 PalletJack 资产穿地 z=-380**（2026-06-13，Cycle 2 预检）
- **症状**：`aws_robomaker_warehouse_PalletJackB_01` spawn 即穿地（z=-9.6→-380），GUI 里等于没放障碍。
- **根因**：非 static + 相对 mesh URI 未解析；mesh collision 未能承托模型。
- **修复**：从可选障碍列表排除；建议改简化 primitive collision（未修）。（`已缓解`）

**E-3 pallet_box_mobile 的 boxes 视觉需 GAZEBO_MODEL_PATH**（2026-06-13，Cycle 2）
- **根因**：`model://` URI 需 `GAZEBO_MODEL_PATH`（碰撞是 primitive，env 无关）。
- **修复**：设 `GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:.../src/task_layer/models`。（`已修复`）

**E-4 相机保真度错误（182° 鱼眼 320×240 不可用）**（2026-06-13，P1-5v）
- **症状**：sim 相机为 182° 鱼眼 320×240 @0.093m，地面交汇 2m 处 0.4m/px 不可用，不对应任何真实硬件。
- **根因**：`model.sdf` 相机参数为占位鱼眼配置。
- **修复**：改 90° 针孔 640×480 @0.25m（后改 1280×720），雷达上方支架=真机华硕 C3 规格。（`已修复`）

**E-5 arm 挂 D436 848×480 与共享基线 shape 不符静默零检出**（2026-06-16，Cycle 5 P2 blocker）
- **症状**：arm 分到的区域静默零检出；`/arm/camera/camera_info` 为 848×480@15Hz，tb3 为 640×480@30Hz。
- **根因**：arm 挂 `turtlebot3_burger_d436_ns`（搁置 A 轴遗留随 main 继承），`photo_diff_check.py:308` 对 shape 不同直接返回 `baseline_shape_mismatch` + 空 anomalies。
- **修复**：`multi_sim.launch.py:23` arm 模型 D436→`turtlebot3_burger_cam_ns`，双机都 C3 640×480@30Hz 与共享基线一致；D436 模型文件保留（搁置 a-axis 分支用）。（`已修复`）

**E-6 cone 圆锥剪影面积不足漏检**（2026-06-22，Cycle 6 S2）
- **症状**：`large_cone`@server_room 两轮均 0/2 漏检。
- **根因**：`photo_diff_check.py:211` `changed_regions(min_area_px=1500)` 按 0.45m 箱标定；圆锥三角剪影实心像素≈同包围盒圆柱一半，0.20m 下掉到 1500px 以下被滤（形状内禀）。
- **修复**：定案为已知弱项，靠调 cone 尺寸解决（加宽/加高越过面积下限），不动 `min_area_px`（护全局误报率）。（`已知弱项`）

**E-7 cone STL mesh 路径解析**（2026-06-22）
- **根因**：cone 用 STL mesh，`model://primitives/cone.stl` 需路径。
- **修复**：`prepare_sdf_file` spawn 流程改写为绝对路径，无需改 launch/GAZEBO_MODEL_PATH。（`已修复`）

---

## F. 感知 / 异常检测 / photo-diff

> 感知路线经历一次重大转向：**激光空区检测器（P1-4）→ 因与 AMCL 根本矛盾整器废弃 → 纯视觉 photo-diff（P1-5v）**。
> A 轴多模态（深度+视觉融合，Cycle 3）曾整轴推进后因过拟合放弃，回到更简单的 photo-diff。

### 激光空区检测器时期（后废弃）

**F-1 激光检测器空区误报（三轮校准）**（2026-06-12，P1-4）
- **症状**：①初版空场 24 站出 1 个 3 格鬼影；②min_cluster 提到 5 后重跑 27 站出 5 个更大假阳性（全在 2.75-3.2m 远处）。
- **根因**：yaw 小偏角×距离=位移，仅 xy 协方差门控不住；远处回波。
- **修复**：加证据半径上限 2.5m + 扫描-地图对齐率门控（≥0.80，阈下记 `skipped_misaligned`）。第三跑 24 站零误报（3 站诚实跳过）。（`已修复`，后随检测器废弃）

**F-2 alignment_ratio 把真异常自己门控掉**（2026-06-12，P1-4）
- **症状**：Gate2 stops=0 之谜——真异常被门控掉（0.82 基线被箱子拖到 0.77<0.80）。
- **根因**：alignment_ratio 分母含 ≤2.5m 证据半径内深自由区回波。
- **修复**：alignment_ratio 排除 ≤2.5m 证据半径内深自由区回波。（`已修复`）

**F-3 main_corridor 门洞 5 格鬼影**（2026-06-12，P1-4 arm Gate1）
- **根因**：沿走廊轴 AMCL 误差 0.3m，侧墙波束沿墙滑动不破对齐率、协方差不破门槛，门框端面波束穿门洞落深自由区。
- **修复**：clearance 0.3→0.4 + 异常限定区域 bounds 内缩 0.3m，重跑零误报。（`已修复`）

**F-4 激光检测器与 AMCL 根本矛盾（整器废弃）**（2026-06-13）
- **症状**：带障碍多区任务中 arm AMCL 瞬移，位姿错误期激光把鬼墙画满全局代价图 → storage_area 全部 8 候选 2 秒内误判被堵 → nav_failed。
- **根因**：检测器与 AMCL 根本矛盾（位姿错误期激光画满鬼墙）；贴墙/角落/远距物体端点贴占据区被 clearance 滤掉。
- **修复**：架构转向——废弃 `area_clear` 激光检测器（`detect_anomalies` 默认 false，留作答辩素材），保留事件管线+导航层修复，转向"激光导航+视觉找异常"。（`已回退`／架构转向）

### 纯视觉 photo-diff 时期（当前主线）

**F-5 位姿扰动地板/墙交界视差残条误报**（2026-06-13，P1-5v）
- **根因**：相位相关全局平移后残留视差。
- **修复**：形状滤波（min_height 15px + 长宽比 ≤6）杀地板/墙交界视差残条。（`已修复`）

**F-6 旋转后相位相关在无效边界锁伪峰**（2026-06-13，P1-5v 硬化根因 bug）
- **症状**：旋转后相位相关在大块无效边界上锁到伪峰（实测 dy=-120px），把墙-地边界拽成巨型假带。
- **根因**：基准与复访朝向差可达数十度，纯平移搜索吸收不了；旋转后无效边界干扰相位相关。
- **修复**：①旋转单应对齐 `H=K·R·K⁻¹` 先按 yaw 差旋转基准再相位相关补残余；②残余平移限幅 12px 越限即弃（信旋转）。（`已修复`，消灭纹理贫乏房间假阳性的关键）

**F-7 素墙上连续优化自刷假掩膜**（2026-06-13，P1-5v）
- **根因**：内点度量在素墙上分辨不到 0.05rad，连续优化反而自刷假掩膜。
- **修复**：朝向择优——在 {记录位姿差, 视觉估计, 0} 三候选取"变化面积最小"者。（`已修复`）

**F-8 门柱/门框近景视差投成高物假阳性**（2026-06-13，P1-5v）
- **修复**：`min_range`（门口 0.8m，余 0.3m）杀近景门框视差、`max_range 3.5m`、隐含物高 >1.8m 弃。（`已修复`）

**F-9 6 向拍照接缝漏检**（2026-06-13，P1-5v）
- **根因**：90° 间距拍照，旋转对齐裁掉一侧 ~20° 留接缝。
- **修复**：改 6 向拍照（60° 间距，原 4×90°）。（`已修复`）

**F-10 清图阈值 50 致 1m 走廊中线膨胀永久误判**（2026-06-13，P1-5v）
- **修复**：阈值 50→120；清图守门=≥N 候选全堵才清一次重试（先确认定位后清）。（`已修复`）

**F-11 面积下限 1500px 标定（离线普查）**（2026-06-13，P1-5v）
- **修复**：四轮全图普查——真 0.45m 箱投影从不低于 2700px，bounds 裁剪+合并后最大残余伪影 667px，取 1500px（4× 间隔）稳杀。（`已修复`／标定）

**F-12 A2 深度默认话题接线错误**（2026-06-15，Cycle 3 blocker #1，A 轴时期）
- **症状**：A2 与自动复核默认永远收不到点云，11 条 verdict 全 `depth.available=false`，全走 geometric_fallback。
- **根因**：默认订阅 `camera/depth/points`（不存在），实际话题 `/arm/camera/points`，frame 是 `camera_link` 非假设的 `camera_rgb_optical_frame`。
- **修复**：runner+verifier 默认话题改 `camera/points`；收不到点云时大声报错不静默降级。（`已修复`，后随 A 轴放弃）

**F-13 numpy.float32 标量致 YAML 序列化崩溃**（2026-06-15，Cycle 3 blocker #2）
- **症状**：接到真实点云后 runner 落盘崩溃 `RepresenterError: cannot represent an object 0.856`，details.yaml 空、无 report.yaml。
- **根因**：PointCloud2 读出的 numpy scalar 经 round() 后仍是 numpy.float32，`yaml.safe_dump` 无法序列化。
- **修复**：`depth_points_to_map` 输出 mx/my/height 全 `float()` 强转。（`已修复`）

**F-14 A 轴多模态整轴过拟合放弃（空场涌现误报）**（2026-06-14/15）
- **症状**：实测空场地涌出 13 个候选、2 个假确认、arm 复核完不归桩。
- **根因**：Codex 验收被设计成单图摆箱子过拟合方案，从没测"空场→0 确认"；A2 深度复核漏了静态地图排除把墙确认成异常。
- **修复**：整条 A 轴放弃（8 条分支 parked，main 一行未动），复活更简单的 photo-diff（p1-5v）+ 已修好的 B 轴导航。（`已回退`／架构决策。A 轴 Cycle 3 期间另修复了 depth 话题、YAML 序列化、地图投影用实际位姿、按 bounds 裁剪、allocator 多模态元数据等多个子问题，均随整轴放弃。）

**F-15 no_baseline 伪装成 checked / clean 区静默零检出**（2026-06-16，Cycle 5）
- **症状**：main_corridor 中心放箱 0 检出（`no_baseline`）；clean 区拍了图做了差分却聚合为 `no_baseline`；反之缺基线区伪装成干净 `checked`。
- **根因**：中心 viewpoint 被箱膨胀堵死降级到**没录基线**的备选点 → 每张 base 不存在 → 静默零检出；`process_photo_views()` 只读 `views`，detect 模式 clean 无 anomaly 时 views 为空落到 no_baseline。
- **修复**：world_model 加显式 viewpoints（偏离中心 ~1m 堵不到）+ 重录基线（narrow_passage 同修）；汇总层累计 `photos_checked`，`checked_total>0 or all_found→checked`，缺基线上浮 `checked_no_baseline` + warn + `completed_with_failures`。（`已修复`）

**F-16 restricted_gate 门条带 clip 全丢检出 / clip=false 门口幻影 FP**（2026-06-16，Cycle 5）
- **症状**：restricted_gate `checked` 但 anomalies 全空；改 clip=false 后 clean 场景投出门口幻影 FP。
- **根因**：restricted_gate 是 0.45m 深门条带，detect_bounds 按 margin 0.30 内缩后 y 带反转成空矩形→clip 恒假 100% 丢弃；直接关 clip 又让近门框视差在干净场景投门口幻影。
- **修复**：clip 仍开但判定边界从"门条带自身"改指被监视的 `restricted_zone`（新机制 `photo_detect_bounds_area`）；复跑 clean FP=0、中心箱 1/1。（`已修复`。诚实边界：单门口位只覆盖禁区中央锥，纵深/两侧/角落是设计盲区。）

**F-17 evidence_photo=None 取证字段不统一**（2026-06-16，Cycle 5 Codex major #2）
- **根因**：runner 只写 `detected_from.photo`，publish event 时读 `anomaly.get('evidence_photo')` 为空。
- **修复**：生成 anomaly 处加 `anomaly['evidence_photo'] = photo`（与 detected_from.photo 同源）。（`已修复`）

**F-18 单走廊三箱同跑中心箱漏检**（2026-06-16，Cycle 5 S5 caveat）
- **症状**：main_corridor 三箱同跑时中心箱漏（单独放则可检 err 0.366）。
- **根因**：疑似近箱遮挡/最小掩码对齐竞争/投影竞争。
- **修复**：caveat 收下不阻塞（真实 demo 稀疏布置），成因另行排查。（`观察项`）

**F-19 A0 几何检测因地图配准偏移在墙边产生假阳性**（2026-06-14，A 轴时期，来源：项目记忆）
- **症状**：storage_area 墙边 `(6.0,-2.343)` 处无障碍却生成一个候选。
- **根因**：A0 用 costmap 减 static map 找"lethal 格盖在 static-free 格上"，但 SLAM 地图与 gazebo 世界存在 ~0.23m 配准偏移（见 [C-5](#c-slam--建图--定位)），使一个真实墙格恰好落到 static-free 格上 → 误判为未建图障碍。高召回 A0 下属预期现象。
- **修复**：arm 验证器加 `wall_clearance=0.30` 判别器——候选质心须离最近 static 墙格 ≥0.30m，否则 `adjacent_to_static_wall` 拒；离线合成栅格验证：墙边 FP（wall_distance 0.103<0.30）被拒、独立障碍确认。（`已缓解`，随 A 轴整轴 parked。可迁移经验：任何"当前图 vs 基准图做差"的检测都要为配准偏移留 1~2 格墙边余量。）

---

## G. 任务分配 / allocator

**G-1 撞家 bug——两机都回 tb3 充电桩**（2026-06-13，P1-3）
- **症状**：arm 返航卡死，两机都回 tb3 充电桩（必现）。
- **根因**：runner 返航读全局 `robot_start`。
- **修复**：`robots.yaml` 增 `home_pose`，runner 增 home_x/y/yaw 参数（最高优先级）——各回各家。（`已修复`）

**G-2 双机同时返航 mother_base 门口会车卡死**（2026-06-12/13）
- **根因**：单门互挤（风险矩阵预言应验）。
- **修复**：初版全部结束后逐台串行发回家；后升级返航并行+门口互斥——`robots.yaml` 加 `home_gate`，正接近被占门口者 cancel 原地等待、门空重发。（`已修复`）

**G-3 已过门者被反向扣留 5s**（2026-06-12）
- **根因**：门口互斥判据未区分"正在接近"vs"已通过"。
- **修复**：加 `past_gate` 判据（比门更接近自家床位=已通过，不扣）。（`已修复`）

**G-4 贪心分配级联倾斜**（2026-06-13，P0-4）
- **根因**：无配额约束。
- **修复**：加 `ceil(N/robots)` 配额防级联倾斜。（`已修复`）

**G-5 分配结果与操作员选房顺序耦合**（2026-06-12）
- **根因**：`allocate()` 未用真实路径代价（直线距离不准）。
- **修复**：`allocate()` 调 `ComputePathToPose(use_start)` 求路径长度（规划器不可用回退直线告警），改全局最小 (robot,area) 对优先。（`已修复`）

**G-6 桩/门被堵返航无限磨蹭**（2026-06-14，Cycle 2 场景 D）
- **根因**：返航需兜底超时防无限驾驶。
- **修复**：每车 `return_timeout_sec=150s` 内放弃、不锁死门、不无限磨蹭，mission 仍出报告退出。验证：tb3/arm 均约 15.7s 后优雅 failed。（`已修复`）

**G-7 子进程 stdout 块缓冲致 GUI 运行中读不到分配日志**（2026-06-13）
- **根因**：子进程 stdout 默认块缓冲，运行中读不到 `Allocation:` 行。
- **修复**：Popen 加 `PYTHONUNBUFFERED=1` + `RCUTILS_LOGGING_BUFFERED_STREAM=0`。（`已修复`）

---

## H. 报告 / evidence

**H-1 报告未合并——多机分区结果散落**（2026-06-12）
- **修复**：每次派单建 `reports/mission_<ts>/` 单目录，顶层生成单份 report（合并+中文指南注释头+detail_report 链接）。（`已修复`）

**H-2 取证照 ppm 不便查看 + 无一屏速览**（2026-06-13）
- **修复**：新增 `mission_*/SUMMARY.txt` 一屏速览；异常取证照 PIL ppm→png 集中拷到 `anomaly_evidence/`（`NN_robot_area` 命名）。（`已修复`）

**H-3 RViz 异常标签白字看不见**（2026-06-13）
- **修复**：标签字体白→红（地图白底）。（`已修复`）

**H-4 report.yaml 冗余 + 报告无上限堆积**（2026-06-22）
- **修复**：每文件夹只留 `details.yaml`(内嵌 summary_report) + 中英双语 `report.md`（mission 级对应 `mission_details.yaml` + `mission_report.md`），不再写 report.yaml；各级目录仅留最近 10 份自动清理。（`已修复`）

**H-5 RViz 截图不实用剥离**（2026-06-22）
- **症状**：RViz 截图（xwininfo + xwd -id）实测不实用；xwd 报 X BadColor。
- **修复**：整块剥离删除（含相关 import）。（`已修复`）

---

## I. 构建 / 环境 / 工具链

**I-1 地图路径硬编码失效**（2026-06-11，P0-1）
- **根因**：地图实际在 `ros_ws/maps/` 而非计划假设的 `~/roboinspec_ws/maps/`；`task_gui_node.py` 有一份漏列的重复 `default_report_dir()` 硬编码。
- **修复**：修路径 + report_dir 去硬编码 ×2 + `set_initial_pose` 的 node.handle 修复；地图入包 `task_layer/maps/`。（`已修复`）

**I-2 重启栈残留 gzserver 致新栈起不来**（2026-06-14，Cycle 1/2/3）
- **症状**：重启栈时旧 gzserver 没死透、新栈起不来（空输出 exit1）；半死栈让 nav 起不来。
- **根因**：TaskStop 与 pkill 竞争致残留进程。
- **修复**：重启前彻底 `pkill -9 -f "gzserver|gzclient|rviz2|component_container_isolated|..."` 清零残留再启。（`已修复`／规约）

**I-3 multi_nav 起栈 load_node 超时（CPU 抢占）**（2026-06-12）
- **根因**：gzclient/RViz CPU 抢占。
- **修复**：`gui:=false` 无头跑根治。（`已修复`）

**I-4 台式机误以为需 tb3_ws overlay**（2026-06-14）
- **根因**：错误假设 burger_cam 是笔记本定制模型。
- **修复**：apt `turtlebot3-gazebo 2.3.8` 自带 burger_cam，无需 overlay；`.bashrc` 加 `TURTLEBOT3_MODEL=burger` + source install。（`已修复`）

**I-5 GUI 编辑吞行致 NameError 死循环（回归事故）**（2026-06-13）
- **症状**：GUI 分配方案不显示且红字异常反馈消失。
- **根因**：上一提交编辑把 `def extract_report_line` 行吞了，方法体并进 `update_allocation_display` 尾部 → 首次轮询 `NameError('output')` → tk `after()` 回调链死亡；py_compile 查不出（语法合法的运行时错）。
- **修复**：还原方法 + 轮询重排程移进 try/finally（单次异常只记日志不杀循环）；建立无头驱动测试法（`DISPLAY=:0` 实例化 TaskGui + 伪造 Popen/事件手动调 poll）。（`已修复`）

**I-6 Codex 沙箱禁 socket 跑不了 ROS**（2026-06-14，Cycle 1；亦见 2026-05-31 bwrap loopback）
- **症状**：Codex 环境 `getifaddrs: Operation not permitted` / `Error creating socket` / `bwrap: loopback: Failed RTM_NEWADDR`，ros2 action list 失败，不能独立复跑。
- **根因**：沙箱安全策略禁止创建 ROS2/FastDDS socket。
- **修复**：后续 cycle 改用非沙箱权限启动 Gazebo/Nav2/RViz；早期文件编辑用 approval 外的短 Python writer 绕过。（`已缓解`）

**I-7 min_area_px 未暴露为 ROS 参数**（2026-06-15，Cycle 3 minor #6）
- **修复**：`min_area_px` 升为 runner ROS 参数并传入 detector，连同 `baseline_pose_tolerance`/`depth_candidates` 记入报告 `execution_policy`。（`已修复`）

**I-8 ros2 daemon discovery 缓存波动致 echo 超时**（2026-06-16，Cycle 5/6）
- **症状**：首次两个 amcl echo 各超时 12s。
- **根因**：CLI discovery 缓存波动。
- **修复**：刷新 ros2 daemon 后立即成功（已知现象非 bug）。（`观察项`）

---

## J. 其他 / 方法论

**J-1 检测-处置物理矛盾**（2026-06-12）
- **症状**：激光可见的搬不动、So-Arm 夹得动的看不见。
- **根因**：检测模态与处置能力错配。
- **修复**：定稿异常分级——大障碍=激光检出+arm 取证标记（"非清除"）；小物体=视觉检出+So-Arm 真抓取（Final）；处置措辞从 handled 改 documented & flagged。（`已修复`／架构决策）

**J-2 两 burger 互见仅 2-7 格证据致互滤**（2026-06-12）
- **根因**：LDS 同高只见对方圆盘。
- **修复**：用队友互滤（`near_peer 0.9m`）+ 最小簇双保险。（`已修复`）

**J-3 git 远端历史含 AI 署名**（2026-06-12）
- **症状**：远端 main 顶端 commit 含 AI 署名。
- **修复**：重写去除 AI 署名（强推）；后续全程守身份 `ryanwonglala`、无 AI 署名。（`已修复`／规约）

**J-4 旧 BaselineLibrary 死代码占 104M**（2026-06-16）
- **根因**：photo-diff 复活后基线格式改为平铺 `baselines/<area>/<stop>/yawNN.ppm`，旧 `baselines/photo_diff/`（BaselineLibrary 格式）无代码引用。
- **修复**：2026-06-16 删除（释放 104M）；死代码属主 `photo_baseline.py` 暂留未清。（`已修复`）

---

## 手动中止运行记录（ABORTED runs）

以下运行经 GUI "Abort & Reset to Dock" **人工中止**，机器人被传送回坞，属**无效巡检结果**（留档以说明运行上下文，非缺陷）：

| 时间 | 运行 | 备注 |
|---|---|---|
| 2026-06-15 04:02 | `tb3/inspection_20260614T205930Z_entrance_lobby_central_hall_east_hall` | 人工中止 |
| 2026-06-15 21:05 | `inspection_20260615T133627Z_east_hall_server_room_restricted_gate` | 人工中止 |
| 2026-06-23 16:54 | `mission_20260623T095223Z/arm/inspection_20260623T095248Z_entrance_lobby_central_hall` | 人工中止 |

---

## 附录：贯穿性救援铁律 / 方法论

以下经验贯穿多次排障，作为通用规约：

1. **救援顺序铁律**：先 `initialpose` 修正定位**并验证**、后清 costmap。反了会把错位期鬼墙留图，全图瘫痪。
2. **真值对照法**：下结论前先 `gz model -p` 拿 Gazebo 真值对照 AMCL / 地图，再判断是定位问题还是地图问题（多次证明"看似地图坏"实为初始位姿对齐问题）。
3. **速度纪律**：0.22 m/s 是实机极限；仿真提速只用 `gz physics -u` 改步长，不改 world 默认，避免高负载跑废使结果不可信。
4. **障碍编排规约**：投箱别放在机器人行进路径上（会物理刮蹭致 AMCL 失锁）；带箱门禁一律 1 倍速跑。
5. **启动就绪规约**：发任务前先等双机 `amcl_pose` 都有输出（~30-40s）再派发，否则 bt_navigator 全 rejected。
6. **重启清场规约**：重启栈前 `pkill -9` 清零残留 `gzserver|gzclient|rviz2|component_container_isolated`，确认零残留再启。
7. **编辑-生效规约**：改 `src` world / 参数后必须 `colcon build` 重装 + re-source，ROS 从 `install/share` 读而非源文件。
8. **GUI 回归防护**：GUI 轮询回调放进 try/finally（单次异常只记日志不杀循环）；用无头驱动法（伪造 Popen/事件手动 poll）测试，`py_compile` 查不出语法合法的运行时错。
