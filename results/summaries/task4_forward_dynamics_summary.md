# Task 4 Forward Dynamics Summary

Goal: input joint position, velocity, drive torque and end-effector wrench; output joint acceleration.

## Inputs And Outputs

- Joint order: `joint2, joint3, joint4, joint5, joint6, joint7`
- End-effector wrench `[Fx, Fy, Fz, Mx, My, Mz]`: `[0, 0, -8, 0, 0, 0]`

| Joint | q [rad] | qd [rad/s] | drive torque [Nm] | target qdd [rad/s^2] | calc qdd [rad/s^2] | sim qdd [rad/s^2] | calc - sim [rad/s^2] |
|---|---:|---:|---:|---:|---:|---:|---:|
| joint2 | 0 | 0.1 | 0.414823 | 0.5 | 0.5 | 0.5 | -6.24943e-11 |
| joint3 | -0.4 | -0.05 | 43.3533 | -0.3 | -0.3 | -0.3 | 4.92828e-13 |
| joint4 | 0.6 | 0.08 | -10.2956 | 0.4 | 0.4 | 0.4 | 7.28639e-13 |
| joint5 | 0 | 0 | 0.000699123 | 0.2 | 0.2 | 0.2 | -4.47765e-11 |
| joint6 | 0.5 | 0.06 | -0.0072763 | -0.2 | -0.2 | -0.2 | 1.29985e-12 |
| joint7 | 0 | -0.04 | 1.38953e-06 | 0.3 | 0.3 | 0.3 | 1.45537e-11 |

## Vector Form

- q [rad]: `[0, -0.4, 0.6, 0, 0.5, 0]`
- qd [rad/s]: `[0.1, -0.05, 0.08, 0, 0.06, -0.04]`
- drive torque [Nm]: `[0.414823, 43.3533, -10.2956, 0.000699123, -0.0072763, 1.38953e-06]`
- target qdd [rad/s^2]: `[0.5, -0.3, 0.4, 0.2, -0.2, 0.3]`
- calculated qdd [rad/s^2]: `[0.5, -0.3, 0.4, 0.2, -0.2, 0.3]`
- simulator qdd [rad/s^2]: `[0.5, -0.3, 0.4, 0.2, -0.2, 0.3]`

## Error Summary

| Quantity | Norm | Max absolute joint error |
|---|---:|---:|
| saved calc qdd - sim qdd | 7.82608e-11 | 6.24943e-11 |
| recomputed theoretical qdd - sim qdd | 7.82608e-11 | 6.24943e-11 |
| saved calc qdd - target qdd | 9.81261e-15 | 8.71525e-15 |
