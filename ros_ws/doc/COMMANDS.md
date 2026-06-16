# RoboInspect 常用命令速查

> 约定：每个新终端先执行 **环境加载**；命令均在 `~/roboinspec_ws/ros_ws` 仓库语境下。

## 0. 环境加载（每个终端必做）

```bash
source /opt/ros/humble/setup.bash
source ~/tb3_ws/install/setup.bash          # TB3 overlay（含 turtlebot3_gazebo）
source ~/roboinspec_ws/ros_ws/install/setup.bash
```

## 1. 构建

```bash
cd ~/roboinspec_ws/ros_ws
colcon build --base-paths src               # 必须带 --base-paths src
colcon build --base-paths src --packages-select task_layer   # 只编一个包
```

## 2. 系统启动

```bash
# 双机完整系统（Gazebo + 双 Nav2 + 双机 RViz 视图 + 双初始位姿），等约 35 s
ros2 launch task_layer multi_nav.launch.py

# 旧单机系统（行为同 v0.2）
ros2 launch task_layer bringup_nav.launch.py

# 单机但带命名空间（调试用）
ros2 launch task_layer bringup_nav.launch.py namespace:=tb3 use_namespace:=true

# 只起仿真不起导航（双机 spawn 调试）
ros2 launch sim multi_sim.launch.py
```

## 3. 双机系统健康检查（启动后判定）

```bash
ros2 topic echo /tb3/amcl_pose --once   # 位置 ≈ (-4.8, -3.825) 南墙充电桩
ros2 topic echo /arm/amcl_pose --once   # 位置 ≈ (-4.8, -2.78)  北墙镜像位
ros2 action list | grep navigate        # 应见 /tb3/ 与 /arm/ 各一
ros2 topic list | grep -c '^/tb3/'      # 应 > 30
ros2 topic info /tf --verbose           # 根 /tf 应无 diff_drive 发布者
```

## 4. 导航（手动发目标）

```bash
# 双机同时发（两个终端或加 &）
ros2 action send_goal /tb3/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 1.2}, orientation: {w: 1.0}}}}"
ros2 action send_goal /arm/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.575, y: -3.3}, orientation: {w: 1.0}}}}"
```

## 5. GUI

```bash
ros2 launch task_layer task_gui.launch.py                  # 双机（默认 ['tb3','arm']）
ros2 launch task_layer task_gui.launch.py robots:="['']"   # 旧单机模式
```

- 顶部下拉框 = 当前控制的机器人（Navigate 发目标 / Manual 巡检跟随它）
- Inspect tab → Dispatch Mode：**Auto allocate**（默认，系统拆路线给全部机器人）/ Manual
- 列表底部灰色 `(walled off)` 的区域（lab_room、prep_room）不可选

## 6. 任务分配器（答辩核心演示）

```bash
# 演示版 6 区域（约 5-8 min，含回桩）
ros2 run task_layer task_allocator.py --ros-args \
  -p route:='storage_area,utility_area,server_room,central_hall,north_hall,east_hall'

# 快速验证版（4 区域不回桩，约 3 min）
ros2 run task_layer task_allocator.py --ros-args \
  -p route:='storage_area,utility_area,server_room,north_hall' -p return_home:=false

# 查看产物
ls ~/roboinspec_ws/reports/tb3/ ~/roboinspec_ws/reports/arm/
grep -E "status|checked_count|failed_count" ~/roboinspec_ws/reports/*/inspection_*/report.yaml
```

## 6b. 录制视觉基线（换设备 / 换地图 / 改观测点后必做）

> **基线 = 干净场景参考照**，照片差分拿当前帧跟它比。下面三条任一不满足，就在
> **目标机器、干净场景**下重录一次（这就是换设备后"开箱不能直接用"的原因）：
> 1. **不进 git**（`baselines/` 已 gitignore）→ `git pull` **不会**带过来；
> 2. **与渲染强相关** → 换机器(GPU/驱动)、换地图/world、改相机 SDF 后，旧基线会误报/漏检；
> 3. **跟随 viewpoints** → 改了某区 `viewpoints` 后新观测点没基线 → 该区报 `checked_no_baseline`、不检出。

```bash
# 前提：① 场景干净(不放任何箱子) ② 已 colcon build ③ 起栈并等 AMCL 稳(~40s)
ros2 launch task_layer multi_nav.launch.py        # 终端1

# 录制：baseline_record:=true 把这一轮巡逻变成"录基线"，双机分工，
# 写入默认 ~/roboinspec_ws/baselines/<区>/<停靠点>/yawNN.ppm (+ 同名 .json 位姿)
ros2 run task_layer task_allocator.py --ros-args \
  -p baseline_record:=true -p return_home:=false \
  -p route:='entrance_lobby,main_corridor,central_hall,north_hall,east_hall,server_room,restricted_gate,storage_area,utility_area,narrow_passage'

# 验证：每个 stop 应有 yaw00..05.ppm(+ 同名 json)；restricted_gate 只 3 张(3 个 scan_yaw)
ls -R ~/roboinspec_ws/baselines | grep -E 'main_corridor|narrow_passage|east_hall|restricted_gate' -A2
```

**停靠点由 world_model 决定（机器人到哪个停就录哪个）**：
- viewpoint 区：`main_corridor` / `narrow_passage` / `east_hall` → `viewpoint_1` + `viewpoint_2`；`restricted_gate` → `viewpoint_1`。
- 环形区（central_hall 等 6 个）→ `center`。一次干净跑只到第一个可达点(center)；`east_wide` 是备份点，仅当异常正压在该区中心采样点时才用到，缺了只让该区报 `checked_no_baseline`（不会静默漏）。

**要点**：① 干净场景是硬前提（基线脏了全盘皆误）；② 双机共用同一套基线（两机相机同为 ASUS-C3 640×480，谁录哪区都一样）；③ 单区补录可用 §7 的 runner 加 `-p baseline_record:=true`；④ 录完布箱、GUI Auto-allocate 跑一轮确认检出。

## 7. 单独跑巡检 runner（无 GUI，调试用）

```bash
# 根命名空间（单机）
ros2 run task_layer inspection_runner.py --ros-args \
  -p use_sim_time:=true -p route:='mother_base,central_hall' -p return_home:=false

# 指定机器人（命名空间注入）
ros2 run task_layer inspection_runner.py --ros-args -r __ns:=/tb3 \
  -p use_sim_time:=true -p route:='server_room' -p return_home:=false
```

## 8. 建图（换场地 / 真机第一步）

```bash
ros2 launch sim map.launch.py use_slam:=true        # 仿真建图（参考）
# 存图（在地图满意时）：
ros2 run nav2_map_server map_saver_cli -f ~/roboinspec_ws/ros_ws/maps/<站点名>
```

## 9. 进程清理（仿真卡死/重启前）

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient
pkill -9 -f component_container; pkill -9 -f robot_state_publisher
pkill -9 -f rviz2
pgrep -af "gzserver|component_container" | grep -v pgrep || echo clean
```

## 10. Git 工作流（项目约定）

```bash
git checkout main && git pull                # 同步队友/已 merge 内容
git checkout -b feat/<任务代号>              # 新工作一律开分支
git add <具体路径> && git commit             # commit 信息无 AI 字样
git push -u origin feat/<任务代号>           # 推分支，GitHub 上人工 merge
# 推送前安检（确认无 AI 署名混入）：
git log --format=%B origin/main..HEAD | grep -ci "claude\|anthropic"   # 应为 0
```

> 计划文档（MASTER_PLAN / ADR / DEVLOG 等）已被 .gitignore 保护，`git add -A` 不会带上。

## 11. 常见故障速判

| 症状 | 第一反应 |
|---|---|
| 跨机/跨终端看不到话题 | `echo $ROS_DOMAIN_ID`（约定 = 2）|
| `ros2 topic echo` 没数据但 `topic info` 有发布者 | QoS 不匹配，加 `--qos-reliability best_effort`；amcl_pose 则需 transient_local |
| 双机话题/TF 混流 | 检查节点是否用了绝对话题名（带 `/` 开头）或忘了 `/tf` remap |
| Nav2 lifecycle 卡死/超时 | CPU 抢占——确认 multi_nav 的错峰启动在用；或清进程重启 |
| AMCL 定位发散 | 初始位姿没发上去（看 `[10/10] initialpose` 日志）；或两机 TF 串了；蹭墙轮滑也会污染（对照真值：`gz model -m <ns> -p`）|
| 机器人卡墙角不动、目标秒 ABORTED | 物理卡死或起点落入致命区。倒车解困后重发：`for i in 1 2 3; do ros2 topic pub -1 /<ns>/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.08}}'; done` |
| launch 传参没生效 | launch 配置是全局的：include 链上游可能已占用同名配置，显式传参覆盖 |
