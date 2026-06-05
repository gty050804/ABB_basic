# ABB IRB 1300 运动学与动力学理论计算

本项目基于 ABB IRB 1300 六自由度机械臂 URDF，完成正运动学、逆运动学、逆动力学和正动力学四项任务。四项任务的主结果均由项目内理论计算得到；PyBullet 只作为独立对比工具使用，不参与主结果生成。

## 任务说明

| 任务 | 脚本 | 输入 | 输出 |
|------|------|------|------|
| Task 1 正运动学 | `scripts/01_forward_kinematics.py` | 关节轨迹 `q(t)` | 末端齐次变换、位置和姿态轨迹 |
| Task 2 逆运动学 | `scripts/02_inverse_kinematics.py` | 末端位姿轨迹、初始关节角 | 关节轨迹、求解成功标志、位置误差 |
| Task 3 逆动力学 | `scripts/03_inverse_dynamics.py` | `q, qd, qdd` 和末端外力 | 驱动力矩、约束力矩、总力矩 |
| Task 4 正动力学 | `scripts/04_forward_dynamics.py` | `q, qd`、驱动力矩和末端外力 | 关节加速度 `qdd` |

## 项目结构

```text
irb1300_kin_dyn/
├── irb1300_kin_dyn/           # 运动学/动力学 Python 核心库
│   ├── se3.py                 # SE(3)、伴随矩阵、指数/对数映射
│   ├── urdf_model.py          # URDF 解析、POE 参数提取
│   ├── poe_kinematics.py      # POE 正运动学与阻尼最小二乘逆运动学
│   ├── dynamics.py            # 递归牛顿-欧拉逆/正动力学理论计算
│   ├── robot.py               # IRB1300Robot 统一接口
│   ├── pybullet_backend.py    # PyBullet 对比后端
│   └── trajectories.py        # 示例轨迹生成
├── irb1300_urdf/              # IRB 1300 硬件资产：URDF、mesh、ROS 配置
├── scripts/
│   ├── 01_forward_kinematics.py
│   ├── 02_inverse_kinematics.py
│   ├── 03_inverse_dynamics.py
│   ├── 04_forward_dynamics.py
│   ├── 05_verify_dynamics.py
│   ├── 06_compare_pybullet.py
│   ├── plot_results.py
│   └── run_all.py
├── results/                   # 运行生成的 .npz 数据和图片
├── requirements.txt
└── README.md
```

项目默认读取的 URDF 路径为：

```text
irb1300_urdf/urdf/1300_urdf.urdf
```

## 环境安装

建议使用 Python 3.10 或更新版本。

```bash
cd /data/IRMV_IMITATION_SEMANTIC_origin/project
python3 -m pip install -r requirements.txt
```

依赖项：

```text
numpy
matplotlib
pybullet
```

其中 `pybullet` 只用于 `scripts/06_compare_pybullet.py` 的仿真对比。

## 运行方法

运行全部脚本：

```bash
python3 scripts/run_all.py
```

单独运行四项任务：

```bash
python3 scripts/01_forward_kinematics.py
python3 scripts/02_inverse_kinematics.py
python3 scripts/03_inverse_dynamics.py
python3 scripts/04_forward_dynamics.py
```

验证动力学内部一致性：

```bash
python3 scripts/05_verify_dynamics.py
```

与 PyBullet 仿真结果对比：

```bash
python3 scripts/06_compare_pybullet.py --no-tables
```

绘制结果图：

```bash
python3 scripts/plot_results.py
```

图片默认保存在：

```text
results/figures/
```

## 理论计算方法

### 正运动学

正运动学采用指数积公式 Product of Exponentials, POE：

```text
T(q) = exp([S1] q1) exp([S2] q2) ... exp([S6] q6) M
```

其中：

- `Si` 为第 `i` 个关节在空间坐标系下的螺旋轴；
- `qi` 为关节角；
- `M` 为机械臂零位时末端坐标系相对基坐标系的齐次变换。

螺旋轴和零位变换由 URDF 解析得到，实现在 `urdf_model.py` 和 `poe_kinematics.py`。

### 逆运动学

逆运动学采用 POE 雅可比迭代和阻尼最小二乘法：

```text
dq = J^T (J J^T + lambda^2 I)^(-1) V_error
```

其中 `V_error` 由当前末端位姿和目标末端位姿之间的 SE(3) 对数映射得到。轨迹求解时，当前时刻初值采用上一时刻的解，以提高连续性。

### 逆动力学

逆动力学采用项目内递归牛顿-欧拉算法 RNEA，求解问题为：

```text
已知 q, qd, qdd, f_ext，求 tau
```

等价动力学关系为：

```text
tau = M(q) qdd + C(q, qd) + G(q) + J(q)^T f_ext
```

代码中通过两次递推计算：

1. 从基座到末端递推各连杆空间速度和空间加速度；
2. 从末端到基座递推空间力，并投影到关节轴得到关节力矩。

Task 3 输出中：

```text
tau_total      = ID(q, qd, qdd, f_ext)
tau_drive      = ID(q, qd, qdd, 0)
tau_constraint = tau_total - tau_drive
```

因此有：

```text
tau_total = tau_drive + tau_constraint
```

### 正动力学

正动力学求解问题为：

```text
已知 q, qd, tau_drive, f_ext，求 qdd
```

项目通过同一套 RNEA 构造质量矩阵和偏置项：

```text
M(q) 的第 i 列 = ID(q, 0, ei, gravity=0, f_ext=0)
h(q, qd, f_ext) = ID(q, qd, 0, f_ext)
```

然后求解线性方程：

```text
M(q) qdd = tau_drive - h(q, qd, f_ext)
qdd = M(q)^(-1) [tau_drive - h(q, qd, f_ext)]
```

## 结果文件

运行脚本后会在 `results/` 下生成：

| 文件 | 内容 |
|------|------|
| `forward_kinematics.npz` | 关节轨迹、末端位姿轨迹、位置和姿态轨迹 |
| `inverse_kinematics.npz` | 目标位姿轨迹、参考关节轨迹、IK 求解关节轨迹、误差 |
| `inverse_dynamics.npz` | `q, qd, qdd`、末端外力、驱动力矩、约束力矩、总力矩 |
| `forward_dynamics.npz` | `q, qd`、驱动力矩、末端外力、关节加速度 |
| `dynamics_verification.npz` | Task 3 和 Task 4 的一致性验证指标 |

## PyBullet 对比说明

默认创建 `IRB1300Robot()` 时不会启用 PyBullet。主任务结果由 POE、阻尼最小二乘、RNEA 和线性方程求解得到。

只有运行以下脚本时才会显式加载 PyBullet：

```bash
python3 scripts/06_compare_pybullet.py --no-tables
```

该脚本将理论计算结果与 PyBullet 在相同 URDF 下的动力学结果进行比较。当前对比误差为数值舍入量级，可用于说明理论计算与仿真工具箱结果一致。

## Python API 示例

```python
import numpy as np
from irb1300_kin_dyn import IRB1300Robot

robot = IRB1300Robot()

q = np.zeros(6)
qd = np.zeros(6)
qdd = np.zeros(6)
f_ext = np.array([0.0, 0.0, -10.0, 0.0, 0.0, 0.0])

# 正运动学
T = robot.forward_kinematics(q)

# 逆运动学
q_sol, ok, iters = robot.inverse_kinematics(T, q)

# 逆动力学
id_result = robot.inverse_dynamics(q, qd, qdd, f_ext)
tau_drive = id_result["tau_drive"]
tau_constraint = id_result["tau_constraint"]
tau_total = id_result["tau_total"]

# 正动力学
tau = np.array([5.0, 3.0, 2.0, 1.0, 0.5, 0.2])
qdd_fd = robot.forward_dynamics(q, qd, tau, f_ext)
```

## 关节与末端定义

- 关节：`joint2` 到 `joint7`
- 末端连杆：`link7`
- 末端外力输入格式：`[Fx, Fy, Fz, Mx, My, Mz]`
