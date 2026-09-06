# Geomagic Touch → 机械臂实时控制（本markdown文档由程序自动生成）

> 本工程(主要实现在Htest1_Touch_PLUS_vision_solver.cpp)实现了 **Geomagic Touch → 机械臂（MoveIt）** 的实时映射控制。
>
> * Touch 提供 **相对位置** 与 **相对姿态（四元数差）** 的控制信号；
> * 机械臂端将 Touch 的相对信号映射为末端位姿，并通过 MoveIt 做 IK 下发关节角；
> * 姿态支持 **轴/角缩放**、**swing–twist 分解**（保留 x 轴高灵敏度、降低其它轴灵敏度），并支持可切换模态（软锁自由度）。

---

## 📖 目录

- [Geomagic Touch → 机械臂实时控制（本markdown文档由程序自动生成）](#geomagic-touch--机械臂实时控制本markdown文档由程序自动生成)
  - [📖 目录](#-目录)
  - [🔎 概览](#-概览)
  - [⚙️ 依赖与环境](#️-依赖与环境)
  - [🛠️ 编译与运行](#️-编译与运行)
  - [📡 话题说明](#-话题说明)
    - [Touch 端](#touch-端)
    - [机械臂端](#机械臂端)
  - [🔧 主要参数](#-主要参数)
  - [🎛️ 控制模式](#️-控制模式)
  - [⌨️ 常用命令](#️-常用命令)
  - [🚀 使用流程](#-使用流程)
  - [⚖️ 调参建议](#️-调参建议)
  - [❓ 常见问题](#-常见问题)
  - [🛡️ 安全与最佳实践](#️-安全与最佳实践)
  - [🔮 开发与扩展](#-开发与扩展)
  - [📜 作者与许可](#-作者与许可)

---

## 🔎 概览

* **位置**：由 Touch 端发布锚点 `Touch_Pose_Anchor` 与增量 `Touch_Pose_increment` 控制
* **姿态**：由 Touch 端发布锚点 `Touch_Orient_Anchor` 与相对四元数 `Touch_Orient_increment` 控制
* **姿态处理**：

  * `q_new = q_arm_anchor * q_delta_scaled`
  * 支持 **轴角缩放** 与 **swing–twist 分解**

---

## ⚙️ 依赖与环境

* **操作系统**：Ubuntu 18.04 / 20.04
* **ROS**：ROS Noetic
* **MoveIt**：与 Noetic 兼容
* **C++ 库**：

  * `tf2`、`tf2_geometry_msgs`、`tf2_eigen`
  * Eigen
* **设备**：Geomagic Touch
* **消息类型**：`geomagic_control` 自定义消息

---

## 🛠️ 编译与运行

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

运行顺序：

1. 启动机器人与 MoveIt
2. 启动 Touch 节点（`pubforce`）
3. 启动机械臂控制节点（`H_real_time_mixed_ctrl_solver`）

---

## 📡 话题说明

### Touch 端

* `/Geomagic/pose` — Touch 绝对位姿 (`PoseStamped`)
* `/Geomagic/button` — 按键事件 (`DeviceButtonEvent`)
* `Touch_Pose_Anchor` (`Float64MultiArray`) — 位置锚点 `[x,y,z]`
* `Touch_Pose_increment` (`Float64MultiArray`) — 位置相对量 `[dx,dy,dz]`
* `Touch_Orient_Anchor` (`Float64MultiArray`) — 姿态锚点 `[x,y,z,w]`
* `Touch_Orient_increment` (`Float64MultiArray`) — 相对姿态 `[x,y,z,w]`
* `Touch_Anchor_updated` (`Bool`) — 位置锚点更新
* `Touch_Ori_Anchor_updated` (`Bool`) — 姿态锚点更新

### 机械臂端

* **订阅**：

  * `/Touch_Pose_increment`
  * `/Touch_Orient_increment`
  * `/Touch_Anchor_updated`
  * `/Touch_Ori_Anchor_updated`
  * `/end_effector_pose`
  * `/Touch_Ori_Filter_Mode`
* **发布**：

  * `/ttttttttarget_pose` (`PoseStamped`)
  * `/goal_joint_positions` (`Float64MultiArray`)

---

## 🔧 主要参数

```cpp
// 位置与姿态缩放
K_touch_pos_FIXED = 0.001
K_touch_ori_CTRL  = 0.2

// swing–twist 模式缩放
K_twist  = 0.85  // x轴旋转灵敏度
K_swing  = 0.2   // 其他轴旋转灵敏度
K_global = 0.5   // 全局缩放模式

// 平滑参数
SMOOTH_FACTOR_POS = 0.02
SMOOTH_FACTOR_ORI = 0.01
```

---

## 🎛️ 控制模式

* **0** — passthrough（无缩放）
* **1** — keep-x-high（默认）：保留 x 高灵敏度，降低其他轴
* **2** — global-reduced：全局缩放
* **3** — custom：实验用（例如更强的 swing 抑制）

切换方式：

```bash
rostopic pub /Touch_Ori_Filter_Mode std_msgs/Int32 "data: 1"
```

---

## ⌨️ 常用命令

* **绕 Z 轴 10° 四元数增量**：

```bash
rostopic pub /Touch_Orient_increment std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.08716, 0.99619]" -r 10
```

* **触发锚点更新**：

```bash
rostopic pub /Touch_Anchor_updated std_msgs/Bool "data: true"
rostopic pub /Touch_Ori_Anchor_updated std_msgs/Bool "data: true"
```

* **查看目标位姿**：

```bash
rostopic echo /ttttttttarget_pose
```

---

## 🚀 使用流程

1. 启动 robot + MoveIt
2. 启动 Touch 节点
3. 启动机械臂 solver 节点
4. 按灰键设置位置锚点，按白键设置姿态锚点
5. 小幅旋转 Touch，验证机械臂响应
6. 根据需要调整 `K_twist`、`K_swing`、`K_touch_ori_CTRL`

---

## ⚖️ 调参建议

* 初始值：

  * `K_touch_pos_FIXED = 0.001`
  * `K_touch_ori_CTRL = 0.1 ~ 0.2`
  * `K_twist = 0.8 ~ 1.0`
  * `K_swing = 0.1 ~ 0.3`
* 最大角度限制：`max_angle_rad ≈ 60°`
* 逐步加大，先在仿真验证，再上真机

---

## ❓ 常见问题

* **`NaNs from NLOpt!!`**：

  * 四元数未归一化 → 检查 `isValidQuaternion`
  * 锚点未设置 → 确认 `/end_effector_pose` 有发布
  * IK 求解失败 → 检查目标是否超出机械臂工作空间

---

## 🛡️ 安全与最佳实践

* 真机前必须在仿真验证
* 始终确保急停可用
* 建议缩放系数暴露为 ROS 参数，便于动态调试

---

## 🔮 开发与扩展

* 将 `K_*` 系数改为 ROS 参数或 `dynamic_reconfigure`
* 改用语义更强的消息类型（如 `QuaternionStamped`）
* 增加日志等级控制与调试选项

---

## 📜 作者与许可

* 作者：张耀华
* 最后更新：2025-09-05
* 许可：MIT
