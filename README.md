# ABB IRB 1300 MATLAB Kinematics And Dynamics

This directory is a MATLAB implementation of the sibling Python `project`.
It keeps the same four theoretical tasks:

| Task | MATLAB script | Output |
|---|---|---|
| Forward kinematics | `scripts/task01_forward_kinematics.m` | `results/forward_kinematics.mat` |
| Inverse kinematics | `scripts/task02_inverse_kinematics.m` | `results/inverse_kinematics.mat` |
| Inverse dynamics | `scripts/task03_inverse_dynamics.m` | `results/inverse_dynamics.mat` |
| Forward dynamics | `scripts/task04_forward_dynamics.m` | `results/forward_dynamics.mat` |

The MATLAB package is under `+irb1300_kin_dyn/`.  It implements:

- SE(3) utilities, adjoint matrices, exponential/logarithm maps.
- URDF parsing with `xmlread`.
- Product-of-exponentials forward kinematics.
- Damped least-squares inverse kinematics.
- Recursive Newton-Euler inverse dynamics.
- Mass-matrix-based forward dynamics.

The original `irb1300_urdf/` assets are copied into this project and the default URDF path is:

```text
irb1300_urdf/urdf/1300_urdf.urdf
```

## Run

From MATLAB:

```matlab
cd /path/to/project_matlab
run('scripts/run_all.m')
```

Or run individual tasks:

```matlab
run('scripts/task01_forward_kinematics.m')
run('scripts/task02_inverse_kinematics.m')
run('scripts/task03_inverse_dynamics.m')
run('scripts/task04_forward_dynamics.m')
run('scripts/plot_results.m')
run('scripts/task08_export_task3_task4_summaries.m')
```

## Notes

MATLAB writes `.mat` files instead of Python `.npz` files.

## Function API

Add the project root to the MATLAB path before calling package functions:

```matlab
cd /path/to/project_matlab
addpath(pwd)
```

All public functions are called through the package name `irb1300_kin_dyn`.
The joint order is:

```matlab
{'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'}
```

Joint vectors are 6-by-1 column vectors. End-effector wrench vectors use:

```matlab
[Fx; Fy; Fz; Mx; My; Mz]
```

where force is in N and moment is in Nm.

### Create Robot Model

```matlab
robot = irb1300_kin_dyn.create_robot();
```

Or provide a custom URDF path:

```matlab
robot = irb1300_kin_dyn.create_robot('/path/to/1300_urdf.urdf');
```

The returned `robot` struct contains the parsed URDF model, screw axes, home transform, joint limits, inertial parameters, and gravity.

Useful fields:

```matlab
robot.n_joints
robot.joint_names
robot.q_lower
robot.q_upper
robot.gravity
```

### Forward Kinematics

Input:

- `q`: 6-by-1 joint angle vector in rad.

Output:

- `T`: 4-by-4 homogeneous transform of the end-effector frame.

Example:

```matlab
robot = irb1300_kin_dyn.create_robot();
q = [0; -0.4; 0.6; 0; 0.5; 0];

T = irb1300_kin_dyn.forward_kinematics(robot, q);
[xyz, rpy] = irb1300_kin_dyn.transform_to_rpy_xyz(T);
```

`xyz` is a 1-by-3 position vector in m. `rpy` is a 1-by-3 roll-pitch-yaw vector in rad.

### Forward Kinematics For A Trajectory

Input:

- `q_traj`: N-by-6 joint trajectory, each row is one sample.

Output:

- `pose_traj`: 4-by-4-by-N transform trajectory.

Example:

```matlab
robot = irb1300_kin_dyn.create_robot();
q_center = irb1300_kin_dyn.default_home_configuration(robot.n_joints);
[q_traj, qd_traj, qdd_traj] = irb1300_kin_dyn.joint_sinusoid_trajectory(50, 6, 0.05, ...
    'q_center', q_center);

pose_traj = irb1300_kin_dyn.forward_kinematics_trajectory(robot, q_traj);
```

### Inverse Kinematics

Input:

- `target_pose`: 4-by-4 desired end-effector transform.
- `q_init`: 6-by-1 initial joint guess in rad.

Output:

- `q_sol`: 6-by-1 solved joint vector in rad.
- `ok`: logical success flag.
- `iterations`: number of iterations used.

Example:

```matlab
robot = irb1300_kin_dyn.create_robot();
q_ref = [0.1; -0.3; 0.5; 0.2; 0.4; -0.1];
target_pose = irb1300_kin_dyn.forward_kinematics(robot, q_ref);
q_init = [0; -0.4; 0.6; 0; 0.5; 0];

[q_sol, ok, iterations] = irb1300_kin_dyn.inverse_kinematics( ...
    robot.screws, robot.m_home, target_pose, q_init, ...
    'q_lower', robot.q_lower, 'q_upper', robot.q_upper);
```

Optional name-value parameters:

```matlab
'max_iter'   % default 200
'pos_tol'    % default 1e-4 m
'rot_tol'    % default 1e-3 rad
'damping'    % default 0.05
'q_lower'    % joint lower limits
'q_upper'    % joint upper limits
```

For a pose trajectory:

```matlab
[q_traj_ik, success] = irb1300_kin_dyn.inverse_kinematics_trajectory(robot, pose_traj, q_init);
```

### Inverse Dynamics

Input:

- `q`: 6-by-1 joint position in rad.
- `qd`: 6-by-1 joint velocity in rad/s.
- `qdd`: 6-by-1 joint acceleration in rad/s^2.
- `f_ext_ee`: 6-by-1 end-effector wrench `[Fx; Fy; Fz; Mx; My; Mz]`.

Output:

- `result.tau_drive`: torque without external wrench.
- `result.tau_constraint`: torque contribution from external wrench.
- `result.tau_total`: total torque.

Example:

```matlab
robot = irb1300_kin_dyn.create_robot();
q = [0; -0.4; 0.6; 0; 0.5; 0];
qd = [0.1; -0.05; 0.08; 0; 0.06; -0.04];
qdd = [0.5; -0.3; 0.4; 0.2; -0.2; 0.3];
f_ext_ee = [0; 0; -10; 0; 0; 0];

result = irb1300_kin_dyn.inverse_dynamics(robot, q, qd, qdd, f_ext_ee);
tau_total = result.tau_total;
```

### Forward Dynamics

Input:

- `q`: 6-by-1 joint position in rad.
- `qd`: 6-by-1 joint velocity in rad/s.
- `tau_drive`: 6-by-1 input torque in Nm.
- `f_ext_ee`: 6-by-1 end-effector wrench `[Fx; Fy; Fz; Mx; My; Mz]`.

Output:

- `qdd`: 6-by-1 joint acceleration in rad/s^2.

Example:

```matlab
robot = irb1300_kin_dyn.create_robot();
q = [0; -0.4; 0.6; 0; 0.5; 0];
qd = [0.1; -0.05; 0.08; 0; 0.06; -0.04];
tau_drive = [10; -5; 3; 1; -1; 0.5];
f_ext_ee = [0; 0; -8; 0; 0; 0];

qdd = irb1300_kin_dyn.forward_dynamics(robot, q, qd, tau_drive, f_ext_ee);
```

### Mass Matrix And Bias Force

These helpers are useful when users need the equation of motion explicitly:

```matlab
M = irb1300_kin_dyn.mass_matrix(robot, q);
c = irb1300_kin_dyn.coriolis_gravity(robot, q, qd);
```

The forward dynamics calculation solves:

```matlab
qdd = M \ (tau_drive - bias_with_external_wrench);
```

### Minimal End-To-End Example

```matlab
cd /path/to/project_matlab
addpath(pwd)

robot = irb1300_kin_dyn.create_robot();

q = [0; -0.4; 0.6; 0; 0.5; 0];
qd = zeros(6, 1);
qdd_target = [0.2; -0.1; 0.15; 0; 0.1; -0.05];
f_ext_ee = [0; 0; -5; 0; 0; 0];

T = irb1300_kin_dyn.forward_kinematics(robot, q);
[xyz, rpy] = irb1300_kin_dyn.transform_to_rpy_xyz(T);

id = irb1300_kin_dyn.inverse_dynamics(robot, q, qd, qdd_target, f_ext_ee);
qdd = irb1300_kin_dyn.forward_dynamics(robot, q, qd, id.tau_total, f_ext_ee);
```
