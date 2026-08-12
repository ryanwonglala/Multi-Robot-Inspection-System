# Demo audio cues (feat/live-demo-tweaks)

Played by the sidecar node `task_layer/demo_audio_node.py`. Drop four audio
files here with these exact names (override via params if needed):

| File | Plays when |
|------|-----------|
| `01_ready.wav`         | 系统加载完毕、双机就绪（nav2 服务可用 + 双机 amcl_pose 到达）|
| `02_task_received.wav` | 点 Start Inspection 后机器人开始执行（首个机器人 cmd_vel 动起来）|
| `03_anomaly.wav`       | 发现异常（`/anomaly_events` 上来新事件，与 RViz/GUI 同源）|
| `04_complete.wav`      | 双机都完成且归位、报告存档（reports/ 下出现新的顶层 `mission_report.md`；单机各自的 `report.md` 不会触发）|

## Format
`paplay` 播放，支持 **.wav / .ogg / .flac**（**不支持 .mp3**）。
若想用 mp3：`sudo apt install mpg123`，启动时加 `-p player_cmd:='mpg123 -q'`。

## Run (与 sim/GUI 并行，独立进程)
```bash
source ~/roboinspec_ws/ros_ws/install/setup.bash
ros2 run task_layer demo_audio_node.py
# 常用可调参数：
#   -p robots:="['tb3','arm']"
#   -p task_requires_both:=true     # cue2 等两台都动起来才播（默认首台动即播）
#   -p player_cmd:='mpg123 -q'      # 改播放器/支持 mp3
#   -p anomaly_debounce_sec:=4.0    # 异常播报最小间隔
```
纯观察节点：只订阅现有话题 + 看 reports 目录，不发指令、不碰巡检/检测逻辑；
不启动它则一切照旧。
