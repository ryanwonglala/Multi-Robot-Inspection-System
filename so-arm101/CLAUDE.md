# SO-ARM101 视觉抓取项目（毕业设计）

## 项目目标
机械臂抓取视野内出现的指定颜色的异常物品，放置到固定回收区域。

## 硬件与平台
- SO-ARM101 机械臂（6x Feetech STS3215 总线舵机，ID 1-6），串口: `/dev/cu.usbmodem5AE70447161`
- 臂上装普通 USB WebCam（icspring camera，eye-in-hand）
- 开发平台: **macOS**（本机）。Intel RealSense D436 深度相机暂不使用；若未来需要深度，再迁移到 Ubuntu 台式机
- 尺寸参数来自开源仓库 TheRobotStudio/SO-ARM100 的 URDF，不做实测

## 技术路线（已确定，勿重新讨论）
- 纯 Python + lerobot（0.6.0，`lerobot.robots.so_follower.SOFollower`）+ OpenCV，不用 ROS
- 单目平面约束方案: 物体在已知高度平面上，标定"像素 → 臂基座平面坐标"映射后反算 (x, y)；z 由支撑面高度 + 物体高度先验给出
- 标定平面取**物体顶面**高度（棋盘格垫高标定），避免斜视角视差偏移
- WebCam 在臂上，观察必须在固定的"观察位姿"进行: 臂回观察位拍照 → 解算 → 抓取
- 物体高度按颜色/类别做成配置参数，不依赖深度

## 环境
- venv: `.venv`（CPython 3.12，uv 创建）。运行: `.venv/bin/python ...`
- 依赖: `lerobot[feetech]`、`opencv-python`

## 部件命名约定（与用户口头沟通用）
关节: 底座=shoulder_pan(ID1) 肩=shoulder_lift(ID2) 肘=elbow_flex(ID3)
腕俯仰=wrist_flex(ID4) 腕旋转=wrist_roll(ID5) 夹爪=gripper(ID6)
方向约定(实测勿凭直觉): 肩增大=前倾伏低/减小=抬起(观察位-85,趴桌+0~50)；
肘增大=折叠/减小=伸展(home 96, 远伸-35)；腕俯仰增大=低头/减小=抬头；
夹爪 0=全闭 100=全开(机器人层)
连杆/附件: 大臂(肩→肘) 小臂(肘→腕) 相机支架(腕上悬臂) 指尖(夹爪手指末端)
夹爪是**单动指**结构(一指固定一指动)：合爪前球必须偏向定指侧，伺服基准(config/servo.json
的 ref)即按此标定；球居中放会被动指扫飞
位姿: home/停靠位(指尖撑桌断电姿势) 观察位(俯视工作区)

## 目录约定
- `scripts/` 按序号排列的可执行步骤脚本（01_test_arm、02_test_camera、...）
- `src/soarm/` 可复用模块（视觉、运动学、控制）
- `config/` 配置（串口、相机 index、HSV 阈值、高度参数、观察位姿）
- `calibration/` 标定产物与快照（棋盘格结果、单应矩阵等）

## 相机取流架构（重要，勿改回直连）
Claude Code 的子进程在 macOS TCC 下拿不到摄像头权限（已验证：静默拒绝、无法在设置中授权）。
uv 安装的 Python（linker-signed）同样被 TCC 静默拒绝，连弹框都不给——相机取流必须用
python.org 官方签名版 Python（`.venv-cam`，3.13 + opencv）。
因此相机由 `scripts/camera_server.py` 独占持有，**必须由用户从终端启动**（终端已授权摄像头）:
`.venv-cam/bin/python scripts/camera_server.py --index 0`（index 0 = 臂上 WebCam 1920x1080，
index 1 = Mac 内置相机；换 USB 口后 index 可能变，用 --list 重新确认）。
所有代码一律通过 `src/soarm/camera_client.py` 的 `get_frame()` 从 http://127.0.0.1:8765 取帧，
绝不要在 Claude Code 运行的脚本里直接 cv2.VideoCapture。
主环境 `.venv`（uv, 3.12）只跑臂控制与算法，不碰相机硬件。

## 检测与深度约定（2026-07-30 定稿）
- 观察位检测 = **参考帧差分**（target.json mode=refdiff）：净空工作面跑
  `scripts/14_capture_ref.py` 拍空场参考照；**灯光/白纸/观察位任一变动必须重拍参考照**
- 分类与抓取参数按类挂在 target.json classes（match 规则/ref/depth_delta/hold/
  servo_hsv/no_roll/grasp_joints）；伺服分割按类：笔袋绿通道，其余 seg="notwhite"
- **深度 = 人工示教网格**（18 号手教样本入 calibration/samples_v3.json，平面拟合），
  无偏置无试探无自学习；12 号自采器已封存勿再启用（三种自学深度方案全部实测失败）
- 垂直姿态约束：夹爪俯仰≈肩+肘+腕俯仰（K≈98.4，示教 grasp_joints 提供），
  执行时腕俯仰=K-肩-肘 实时补偿
- **04 号录位姿会覆盖 observe**（已两次误覆盖）！录完必查 poses.json；observe 恢复值:
  pan 1.0 / lift -16.5 / elbow -11.0 / wrist_flex 100.3 / wrist_roll 3.5 / gripper 1.1
- 调试：SOARM_TRACE=1 打开运动指令飞行记录；指令干净≠物理干净，肉眼/侧拍视频定案

## 注意事项
- 舵机操作默认只读；任何使能扭矩/运动的脚本要先小范围低速验证
- 舵机 RAM 会记住上次会话的目标位置：使能扭矩前必须把 Goal_Position 钉到当前位置
  （`arm.connect()` 已内置 `_pin_goals_to_present()`），否则上电瞬间臂会朝过期目标猛冲
- home 位必须是"上电可达 + 断电自稳"的姿势。当前 home 是实测停点（肩 -82.3/腕 45.4），
  勿手动改 poses.json 里的 home。深折叠区禁止抬腕分段——夹爪/相机支架会撞大臂（实测）；
  手摆记录的深折叠角度（肩 < -85）舵机走位置轨迹普遍到不了，以实测停点为准
- shoulder_lift 标定 range_min 已从 1075 放宽到 780（备份 main_arm.json.bak）
- 单位约定：机器人层(send_action/get_observation) 夹爪是 0-100 开合度、其余关节是度；
  底层 FeetechMotorsBus 按 norm_mode 解释——两层的夹爪数值不通用，混用会导致夹爪猛冲
