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
