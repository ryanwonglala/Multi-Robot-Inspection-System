# 新场地完整部署流程（SO-ARM + TB3 托盘分拣）

预计耗时：布置+标定半天；TB3 联调另计。
原则：**顺序不可乱**——每步的产物是下一步的输入。

---

## 阶段 0：出发前（老场地完成，避免现场抓瞎）

### 物料
- [ ] 3D 打印正方体：边长 2.5-3cm、**各色同高**、哑光、红/蓝/绿/橙（避白/浅灰）
- [ ] 托盘面：白色、哑光、平整无纹理
- [ ] TB3 停靠机械限位件（V 形挡块或导轨贴条，目标停车重复性 ±1cm）
- [ ] 硬件：臂+电源、WebCam+USB 线、Mac、回收罐、备用白纸+胶带

### 方块预接入（在老场地白纸上先跑通！）
- [ ] 每色 `16_measure_grip.py --cls <色名>` 实测宽度区间（正跨+斜跨都量）
- [ ] target.json classes 每色一条：match 用 Lab 颜色规则（a/b 轴区分红绿蓝橙），
      抓取参数（ref/depth_delta/grasp_joints/hold）教一个色后全色共享
- [ ] 白纸上 `09 --loop` 多色混合验证通过后再出发

### 代码预改造（Claude 完成）
- [ ] 观察位检测支持 mode="notwhite"（托盘场景 refdiff 不可用：托盘每次停靠
      位置微变，参考帧会失效；白托盘+彩色方块用"非白即物"最稳）
- [ ] classify_blob 的掩码来源跟随检测模式（现在写死 refdiff）
- [ ] 04_record_pose 修"覆盖 observe"陷阱（录制时强制指定位姿名）
- [ ] （可选彩蛋）白托盘自寻位动态 ROI

---

## 阶段 1：场地布置（顺序固定）

1. **臂基座刚性固定**。固定后不再挪动——挪臂=整条标定链重做（07-30 实测教训）
2. **TB3 停靠位 + 机械限位**：托盘中心落在臂前方舒适工作半径内。
   ⚠ 关键几何验证（断电手动）：把夹爪摆到托盘中心上方，确认
   ① 托盘面高度可达 ② 垂直姿态可行 ③ 托盘四角均可达
3. **回收罐**：摆在臂转体方向一侧，罐口位姿手动确认可达
4. **光照**：稳定光源；避免强侧光（曾导致空间感误判）；部署后不再改灯
5. 相机 USB 接好；换过 USB 口则 index 可能变

---

## 阶段 2：软件链路开通（顺序固定）

```
① 相机服务（用户终端）
   .venv-cam/bin/python scripts/camera_server.py --list   # 确认 index
   .venv-cam/bin/python scripts/camera_server.py --index N
② 录观察位（04）：俯视托盘停靠区完整入画
   ⚠ 录完必查 config/poses.json 名字对不对（04 有覆盖 observe 前科！）
③ 拍照验证构图（05）→ 更新 config/roi.json：绿框 = 托盘内缩 3-4cm 区域
   （内缩量 > 停车误差 ⇒ 黑框架/彩色电线永远在框外；预览页绿框自动跟随）
④ 录 drop（04）：罐口上方 3-5cm 悬停位（不探罐），自然姿态
⑤ 检测模式切换：target.json mode → "notwhite"（白托盘场景）
⑥ 示教网格（18）：旧 samples_v3 先归档；方块摆托盘上手教 6-10 点
   （四角靠内+中心+补边），全程垂直纪律。这一步同时重建深度面与 K 常数
⑦ 类参数（15）：任一色方块教一次 → 参数复制到其余色
⑧ 伺服雅可比自动重标（Claude 跑，悬停几何随新高度变了）
⑨ 验证阶梯：单色单轮（09）→ 多色混合（09 --loop --step）→ 压测（09 --test 20）
   → 17 号出成功率曲线
```

## 阶段 3：TB3 联调

- [ ] 停靠重复性实测：连停 5 次量托盘角点偏差，确认 < ROI 内缩量
- [ ] **空载 vs 满载托盘高度差**（悬挂下沉）：若 >5mm，示教网格用满载状态教
- [ ] 卸货触发方式：起步用人工节拍（--loop --step，TB3 停稳按 Enter）；
      后续可升级 TB3→Mac 的到位信号（TCP/文件旗标）
- [ ] 全流程演练 ×3 + 侧拍视频（验收标准：指令干净≠物理干净，录像定案）

---

## 故障速查

| 症状 | 处置 |
|---|---|
| observe 被 04 覆盖 | 恢复值在 CLAUDE.md（pan 1.0 / lift -16.5 / elbow -11.0 / wf 100.3 / wr 3.5 / grip 1.1 为老场地值；新场地录完自己备份 poses.json） |
| 对位普遍偏/深度不对 | 检查是否挪过臂/托盘高度变了 → 重跑 18 示教网格 |
| 伺服振荡不收敛 | 雅可比过时 → 自动重标 |
| 某色认不出 | 分类特征打印（worklog 有调试片段）→ 调该色 match 规则 |
| 急停后恢复 | `13_recover.py` |
| 疑难运动问题 | `SOARM_TRACE=1` 开飞行记录对时间线 |

## 不变量（换场地也不变的东西）
臂控制栈、合爪逻辑、运输编排、伺服算法、示教工具链、单位约定、
"干净深度来自人手"原则、"物体完整摆进绿框"纪律。

---

# 架构修订（07-30 定）：Jetson 中控方案

自制小车底座判死（不修），其 Jetson Nano Super 8G + 深度相机 + 雷达降级重组为
**卸货区固定监督站 + 全系统中控**。论文叙事: 异构协作 + 硬件故障下的降级运行设计。

## 最终拓扑
```
Jetson Nano Super (卸货区, Ubuntu + ROS2 Humble) —— 中控
├─ SO-ARM101 (USB串口, SOARM_PORT=/dev/ttyACM*)   ← 臂控制栈(本仓库代码原样跑)
├─ 腕装 WebCam (USB)                               ← camera_server 同款(无TCC之苦, 可systemd自启)
├─ 深度相机 D436 (侧视卸货区)                       ← 第三方验收: 托盘清空判定/掉件检测/录像存证
├─ 雷达                                            ← TB3到位触发 + 工作单元安全监护
└─ ROS2 ←→ TB3 车队 / 巡检报告系统
Mac: 退役为开发机 + 仪表盘(随时可回退为运行平台)
```

## 迁移清单（与新场地部署合并执行, 不重复劳动）
1. Jetson 拉仓库 `so-arm101/`; venv + `pip install "lerobot[feetech]" opencv-python scipy matplotlib`
   （aarch64 若 torch 装不动, 用 NVIDIA 官方 wheel; 本栈不跑神经网络, CPU torch 即可）
2. 串口: 用户加入 dialout 组; `ls /dev/ttyACM*` 确认; `export SOARM_PORT=/dev/ttyACM0`
3. 从 Mac 拷贝舵机标定: `~/.cache/huggingface/lerobot/calibration/robots/so_follower/main_arm.json`
4. WebCam 接 Jetson; `camera_server.py --list` 确认 index; 可做 systemd 服务
5. 链路验证: 01(臂连接) → 相机 → 05(观察位) → 09 单轮 —— 全通过后才进部署阶段2
6. 二期(链路稳定后): 雷达到位触发+安全监护、D436 验收节点, 以 ROS2 节点形式挂同机,
   与臂控制栈用本机话题/简单IPC对接

## 风险与回退
- lerobot aarch64 安装若受阻: Mac 方案完整保留, 插回 Mac 即可运行(仅 SOARM_PORT 不同)
- Jetson 算力富余(Nano Super 8G), 阶段二 SmolVLA 推理可直接落本机(训练仍在 5070Ti)
