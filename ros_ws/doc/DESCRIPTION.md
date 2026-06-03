# RoboInspect 软件端开发方向引导

## 1. 当前阶段的核心目标

接下来两周的软件目标不是立刻做完整系统，而是先在 ROS2 + Gazebo 中验证“机器人以上的 task layer”。

当前主线应当是：

> 从固定路线巡检，升级为任务驱动的多机器人协作系统。

也就是说，重点不只是让 robot 能移动，而是验证：

- 人输入一个 inspection task
- 系统能拆解任务
- 系统能选择合适 robot
- 系统能调用对应 skill
- 系统能根据执行结果调整计划

---

## 2. 不要把系统理解成“完全不预设”

我们的系统仍然会有 predefined skills。

例如：

- `NavigateTo(location)`
- `InspectArea(area)`
- `CaptureImage(target)`
- `ReturnHome()`
- `WaitForRobot(robot_id)`
- `ReportStatus()`

真正区别不是“没有预设”，而是：

> 预设的是 robot 的基础能力，不是整条巡检路线。

传统系统更像：

```text
Route 1 = A → B → C → Return

我们的系统应该更像：

Task Goal
↓
Task Graph
↓
Skill Selection
↓
Robot Assignment
↓
Execution Feedback
↓
Re-planning / Reassignment

所以核心表达应当是：

The robot skills are predefined, but the full inspection routine is generated and adjusted based on task goals, robot capabilities, and execution feedback.

3. 软件架构的核心概念

建议把系统分成 5 层：

Task Input Layer
↓
Task Planner Layer
↓
Skill Layer
↓
Robot Assignment Layer
↓
ROS2 Execution Layer
3.1 Task Input Layer

先不要急着做自然语言。

第一阶段建议使用 structured task input，例如：

task:
  type: inspect_area
  target: room_A
  require_image: true
  priority: normal

自然语言可以后续作为 optional interface。

3.2 Task Planner Layer

负责把任务拆成 task graph。

例如：

Inspect Room A
├── NavigateTo(Room A)
├── ScanArea(Room A)
├── CaptureImage(Room A)
└── ReturnHome()

后续可以加入条件：

If path blocked:
  Try alternative route
  If still failed:
    Assign another robot
3.3 Skill Layer

每个 skill 是一个可复用的机器人能力。

重点是让 skill 尽量 atomic、清晰、可组合。

例如：

navigate_to
inspect_area
capture_image
return_home
wait
report_status

不要一开始就做太复杂的 skill。

3.4 Robot Assignment Layer

不要写死：

Robot1 always does inspection
Robot2 always does manipulation

而是给 robot 定义 capability model：

robot_1:
  capabilities:
    - navigate
    - inspect_area
    - capture_image
  status: available

robot_2:
  capabilities:
    - navigate
    - interact
    - capture_image
  status: available

系统根据任务需要选择 robot。

3.5 ROS2 Execution Layer

底层通过 ROS2 执行具体动作。

建议优先考虑：

ROS2 Actions
Nav2 action interface
namespace 管理多机器人
Gazebo 中先完成 sim validation
4. 多机器人协作应该怎么定义

不要只说“多个机器人一起巡检”。

需要定义具体协作模式。

建议优先验证 3 种：

4.1 Parallel Inspection

两个 robot 分别检查不同区域。

Robot A → Room 1
Robot B → Room 2

价值：

提高效率
对比 single-robot baseline
4.2 Sequential Handoff

一个 robot 先完成导航/观察，另一个 robot 接着执行更具体的任务。

Mobile Inspector → Navigate and inspect
Mobile Manipulator → Local interaction

价值：

展示 heterogeneous robot collaboration
为后续机械臂硬件接入做准备
4.3 Backup / Reassignment

一个 robot 失败或不可用时，另一个 robot 接管任务。

Robot A failed
↓
Robot B reassigned

价值：

证明系统不是固定脚本
展示 adaptive task coordination
5. 母子机器人方向如何处理

Prof 对母子机器人感兴趣，可以保留为 exploration branch。

但不要把它作为主线核心承诺。

建议定位为：

Deployable scout robot concept for narrow-space inspection.

软件端可以先在 Gazebo 中抽象为：

Main robot → reach entrance
Scout robot → inspect narrow area
Main robot → collect/report result

现阶段不需要解决真实 docking、release、return-to-base 等复杂硬件问题。

先验证 task logic：

主机器人到达狭窄区域入口
scout robot 被分配进入局部区域
scout robot 返回 inspection result
主系统汇总状态
6. 两周内建议达成的最小闭环

优先做一个可以演示的 minimal system：

Structured Task Input
↓
Task Planner generates skill sequence / task graph
↓
Robot Assignment selects robot
↓
Robot executes Nav2 navigation in Gazebo
↓
System receives feedback
↓
Task completes or triggers fallback

最小 demo 可以是：

Demo A：单机器人任务拆解
Input: Inspect Room A
Output:
Robot1 → NavigateTo(Room A)
Robot1 → InspectArea(Room A)
Robot1 → ReturnHome()
Demo B：双机器人并行巡检
Input: Inspect Room A and Room B
Output:
Robot1 → Room A
Robot2 → Room B
Demo C：机器人不可用时重新分配
Robot2 unavailable
Input: Inspect Room A and Room B
Output:
Robot1 → Room A → Room B
Demo D：母子机器人概念模拟
Input: Inspect narrow area
Output:
Main Robot → Navigate to entrance
Scout Robot → Enter narrow area
Scout Robot → Report result
7. 当前不要优先做的事情

为了避免走歪，以下内容暂时不要作为前两周重点：

不要急着做 LLM / natural language
不要急着做复杂 object detection
不要急着做 RL
不要急着做真实机械臂控制
不要急着做完整 mother-child docking
不要陷入过度复杂的 map sharing / multi-robot SLAM

这些都可以作为后续 optional features。

当前重点是：

task layer 是否能跑通。

8. 项目主线一句话

后续开发和汇报都可以围绕这句话：

We aim to develop a task-driven multi-robot inspection framework where predefined robot skills are dynamically composed, assigned, and adjusted based on task goals, robot capabilities, and execution feedback.

中文理解：

我们的目标是构建一个任务驱动的多机器人巡检框架。机器人技能是预设的，但完整任务流程会根据任务目标、机器人能力和执行反馈动态组合、分配和调整。

9. 当前开发优先级
Priority 1

让一个 TurtleBot4 在 Gazebo 中接受 task command，并完成 basic inspection task sequence。

Priority 2

扩展到两个 TurtleBot4，实现 namespace、独立导航和简单任务分配。

Priority 3

建立 capability model，让系统根据 robot 状态和能力选择执行者。

Priority 4

加入 feedback / fallback，例如 robot unavailable、navigation failed、task reassignment。

Priority 5

抽象 mother-child / scout robot task flow，先在 simulation 里验证逻辑。

10. 判断是否走对方向的标准

每次开发新功能前，先问：

这个功能是否服务于 task-driven inspection？
它是否能证明系统不是固定路线脚本？
它是否提升了 robot coordination？
它是否能在 Gazebo 里快速验证？
它是否能成为 final demo 的一部分？

如果答案是否定的，就暂时不要做。