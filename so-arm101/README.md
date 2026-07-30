# SO-ARM101 Vision Sorting Subsystem | SO-ARM101 视觉分拣子系统

**English** | [中文](#中文)

Robotic-arm sorting module of the Multi-Robot Inspection System: a fixed
SO-ARM101 detects abnormal-colored objects on a TurtleBot3 tray, grasps them,
and drops them into a recycling bin. **Fully independent from `ros_ws/`
(TB3)**: pure Python + lerobot, no ROS, runs on macOS.

## Approach
- Monocular wrist-mounted webcam + planar constraint (objects on a
  known-height plane) — no depth camera
- Observation-pose detection: reference-frame differencing / not-white
  segmentation (tray scenario) + solidity/Lab-rule classification
- Localization: pixel→joint mapping from a **human-taught grid**
  (6–16 points, plane fit, LOO ≈ 1.6°) refined by hover visual servoing
  (2×2 Jacobian)
- Grasping: vertical-posture constraint (shoulder + elbow + wrist pitch =
  const) + contact-stop gripper + load verification
- Key design decisions and negative results (YOLO-World zero detection,
  three failed self-learned-depth schemes) are documented in
  `docs/worklog-2026-07-30.md`

## Layout
- `src/soarm/` — reusable modules: arm (motion/gripping), vision
  (detection/classification), mapping (taught-grid fit), camera_client
- `scripts/` — numbered step scripts: 01–05 basics, 09 grasp executor
  (`--loop/--test/--step`), 10 servo calibration, 13 e-stop recovery,
  14 reference capture, 15 per-class teaching, 16 grip-width measurement,
  17 log analysis, 18 grid teaching, 19 manual cockpit
- `config/` — serial port / poses / work-zone ROI / class params / servo calib
- `calibration/` — taught sample libraries (v3 = current), stress-test log
  (`attempts_log.jsonl`), reference frame
- `docs/` — full worklog + new-site deployment guide
  (`deployment-new-site.md`)

## Environment
- Main venv `.venv` (Python 3.12): `lerobot[feetech]==0.6.0`,
  `opencv-python`, `scipy`, `matplotlib`
- Camera-only venv `.venv-cam` (python.org-signed Python 3.13 + opencv):
  required by macOS TCC — the camera server must use it and be launched
  from Terminal (see "camera architecture" in `CLAUDE.md`)

## Quick start
```bash
# Terminal 1: camera server (owns the camera, serves frames over HTTP)
.venv-cam/bin/python scripts/camera_server.py --index 0
# Terminal 2: sorting loop (auto grasp+drop per round; Enter for next round)
.venv/bin/python scripts/09_grasp.py --loop --step
```

Conventions and safety notes (power-on lunge, unit pitfalls, pose-recording
caveats) are centralized in `CLAUDE.md`.

---

## 中文

多机器人巡检系统的机械臂分拣模块：固定安装的 SO-ARM101 从 TurtleBot3 托盘上
识别并抓取异常颜色物体，投放至回收区。**与 `ros_ws/`（TB3）完全独立**：
纯 Python + lerobot，不依赖 ROS，运行平台 macOS。

### 技术路线
- 单目腕装 WebCam + 平面约束（物体在已知高度平面上），不用深度相机
- 观察位检测：参考帧差分 / 非白分割（托盘场景）+ 实心度/Lab 规则分类
- 定位：像素→关节映射（**人工示教网格** 6-16 点，平面拟合，LOO≈1.6°）
  + 悬停视觉伺服（2×2 雅可比）精修
- 抓取：垂直姿态约束（肩+肘+腕俯仰=常数）+ 接触即停合爪 + 负载验证
- 关键设计决策与失败实验（YOLO-World 零检出、三种自学深度方案的失败）
  见 `docs/worklog-2026-07-30.md`

### 目录
- `src/soarm/` 可复用模块：arm(运动/夹持) vision(检测/分类) mapping(示教映射) camera_client
- `scripts/` 按序号排列的步骤脚本：01-05 基础，09 抓取执行器(--loop/--test/--step)，
  10 伺服标定，13 急停恢复，14 参考照，15 类示教，16 夹宽实测，17 日志分析，
  18 示教网格，19 手动驾驶舱
- `config/` 串口/位姿/工作区 ROI/目标类参数/伺服标定
- `calibration/` 示教样本库(v3=当前)、压测日志(attempts_log.jsonl)、参考帧
- `docs/` 完整工作日志 + 新场地部署手册(deployment-new-site.md)

### 环境
- 主环境 `.venv`（Python 3.12）：`lerobot[feetech]==0.6.0` `opencv-python` `scipy` `matplotlib`
- 相机专用 `.venv-cam`（python.org 签名版 Python 3.13 + opencv）：macOS TCC 限制，
  相机服务必须用它并从终端启动（详见 CLAUDE.md"相机取流架构"）

### 快速开始
```bash
# 终端1: 相机服务(独占相机, 提供 HTTP 帧)
.venv-cam/bin/python scripts/camera_server.py --index 0
# 终端2: 分拣循环(每轮自动抓取+投放, Enter 进入下一轮)
.venv/bin/python scripts/09_grasp.py --loop --step
```

约定与安全事项（上电冲击、单位陷阱、位姿录制注意）统一见 `CLAUDE.md`。
