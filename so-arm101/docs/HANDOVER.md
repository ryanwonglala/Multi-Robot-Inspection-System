# 交接简报（给 Ubuntu/Jetson 侧 AI 助手的冷启动文档）

> 读者：明天在 Ubuntu + VS Code AI IDE 里协助部署的助手（以及未来的任何新会话）。
> 使命：把已在 macOS 上验证完毕的 SO-ARM101 分拣系统迁移到 Jetson 并完成新场地部署。
> **必读顺序**：本文件 → `../CLAUDE.md`（约定与安全铁律）→
> `deployment-new-site.md`（部署流程+Jetson迁移清单）→ 有疑问再查 `worklog-2026-07-30.md`（全部设计决策的来龙去脉）。

## 一、系统是什么（30 秒版）

固定安装的 SO-ARM101 机械臂 + 腕装 RGB 相机，检测工作区内异常颜色物体，
分类后抓取并投放回收罐。TurtleBot3 把物体驮到卸货区托盘上，臂负责分拣。
技术路线：**经典视觉 + 人工示教标定 + 闭环控制，无神经网络**（这是深思熟虑的
定稿，不是待优化项——三种自学习深度方案全部被实测否决，过程在 worklog）。

当前验证状态（2026-07-30 于 macOS 老场地）：
- 单类目标（乐高企鹅替身）全链路稳定：检测→分类→对位（伺服 1-8 轮收敛）
  →垂直下潜→合爪（开合度判定）→提起验证→运输→罐口投放；
- 示教网格 16+ 点，深度平面拟合留一误差 ≈1.6°；
- 工作区=画面绿框 ROI（托盘尺度 13×10cm），区内中上带稳定。

## 二、明天的任务清单（按序执行）

### T0 打印目标方块（到场第一件事，打印期间并行做 T1-T2）
规格（已定稿，勿改）：正方体、边长 2.5-3cm、**各色同高**、哑光、
红/蓝/绿/橙 3-4 色、避免白/浅灰。托盘面：白色哑光平整。

### T1 Jetson 环境搭建
按 `deployment-new-site.md`"迁移清单"§1-4 执行：
拉仓库 → venv → `pip install "lerobot[feetech]" opencv-python scipy matplotlib`
→ dialout 组 + `SOARM_PORT=/dev/ttyACM0` → 拷贝舵机标定文件（从 Mac：
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/main_arm.json`）
→ WebCam 用同款 `scripts/camera_server.py`（Linux 无 TCC 之苦，直接跑）。

### T2 链路冒烟测试（顺序：01→相机→05）
`scripts/01_test_arm.py`（臂连接，只读）→ camera_server + 浏览器预览
→ `05_verify_observe.py`。全绿才继续。

### T3 场地布置 + 标定链（deployment 阶段 1-2）
臂固定 → TB3 停靠限位 → 04 录 observe/drop（⚠ 04 会覆盖 observe，录完必查
poses.json！）→ ROI 画托盘内缩区 → target.json mode 切换（见 §四.3）
→ 18 号示教网格 6-10 点（垂直纪律）→ 15 号教类参数 → 雅可比重标。

### T4 方块接入
每色 `16_measure_grip.py --cls <色>` 量宽度 → classes 每色加 Lab 颜色
match 规则（参考现有"笔袋/积木"条目格式）→ 抓取参数教一色后全色共享。

### T5 验证阶梯
单色单轮 `09_grasp.py` → 混合 `09 --loop --step` → 压测 `09 --test 20 --step`
→ `17_analyze_log.py` 出成功率曲线（论文图）。

## 三、给 AI 助手的工作约定

1. **已定稿的决定不要重新发起讨论**（用户明确要求过）：技术路线（无 NN）、
   深度=人工示教（勿再实现任何自学习深度）、目标物=正方体、Jetson 中控架构、
   工作区=托盘内缩绿框、D436 只做监督站不参与抓取。
2. **交互式脚本（带键盘微调的 04/10/15/16/18/19）由用户在终端跑**，
   助手负责非交互脚本与代码修改。
3. 调试顺序：先 `SOARM_TRACE=1` 看运动指令时间线；但**指令干净≠物理干净**，
   最终以用户肉眼/侧拍视频定案（此教训当天验证过两次）。
4. 舵机安全铁律在 CLAUDE.md"注意事项"：上电前钉扎目标位（connect() 已内置）、
   低速验证、夹爪两层单位不可混用、单动指偏置。**任何使能扭矩的新代码先小
   范围低速测**。
5. 修改检测/抓取参数时逐类改 `config/target.json`，不要动共享几何
   （`config/servo.json` 由标定脚本写）。

## 四、高频陷阱速查（全部实测踩过）

1. **04 录位姿覆盖 observe**（两次）：录完必查 `config/poses.json`。
2. **挪臂/挪托盘/换灯 = 标定链作废**：重跑 18（网格）+ 14（参考照, 若用 refdiff）。
3. **检测模式**：老场地白纸用 mode=refdiff（需净空参考照）；托盘场景托盘会微移，
   refdiff 失效 → 需切"非白即物"。⚠ 代码现状：观察位 mode="notwhite" 与
   classify_blob 的掩码跟随**尚未实现**（在 deployment 文档"代码预改造"清单里，
   这是到场后第一个代码任务）。
4. **伺服振荡不收敛** = 雅可比过时（姿态/高度变了就要重标，worklog 有自动重标脚本段）。
5. **物体必须完整落在绿框内**——半个身子在框外会被 ROI 裁切，分类和对位全歪。
6. 方形物体不要开朝向对齐（`no_roll: true`）：正方形主轴是数学噪声。

## 五、关键文件地图

| 路径 | 作用 |
|---|---|
| `src/soarm/arm.py` | 运动/夹持全部原语（smooth_goto/grip_close/transport_to_drop/飞行记录） |
| `src/soarm/vision.py` | 检测(refdiff/notwhite/HSV)+分类(classify_blob)+ROI |
| `src/soarm/mapping.py` | 示教样本→关节 平面拟合（v3 库优先） |
| `config/target.json` | 类定义：match 规则+ref+depth_delta+hold+grasp_joints+servo_hsv |
| `config/poses.json` | observe/home/drop 三个关键位姿 |
| `config/roi.json` | 工作区绿框（预览页自动叠显） |
| `calibration/samples_v3.json` | 人工示教样本库（老场地的，新场地要重教） |
| `scripts/09_grasp.py` | 主执行器：`--loop --step --test N` |

## 六、老场地遗留状态（供参考，新场地会全部重建）
observe/drop/绿框/示教网格/雅可比均为老场地值——新场地一律按 T3 重建，
不要试图复用位姿数值。samples_v3.json 到场后先归档再重教。
唯一跨场地不变的：代码、类的 match 规则思路、hold 区间（同一批物体则有效）。
