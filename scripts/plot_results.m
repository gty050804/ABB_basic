% Plot trajectories from results/*.mat for the four theoretical tasks.
ROOT = fileparts(fileparts(mfilename('fullpath')));
addpath(ROOT);

results_dir = fullfile(ROOT, 'results');
out_dir = fullfile(results_dir, 'figures');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

plot_forward_kinematics(results_dir, out_dir);
plot_inverse_kinematics(results_dir, out_dir);
plot_inverse_dynamics(results_dir, out_dir);
plot_forward_dynamics(results_dir, out_dir);
fprintf('Figures saved to %s\n', out_dir);

function require_file(path)
if ~isfile(path)
    error('Missing %s. Run scripts/run_all.m or the corresponding task first.', path);
end
end

function plot_forward_kinematics(results_dir, out_dir)
path = fullfile(results_dir, 'forward_kinematics.mat');
require_file(path);
d = load(path);
steps = 0:size(d.q_traj, 1)-1;
figure('Visible', 'off', 'Position', [100, 100, 1200, 800]);
subplot(2, 2, 1);
plot(steps, d.q_traj, 'LineWidth', 1.2); grid on;
xlabel('Step'); ylabel('Joint angle [rad]'); title('Joint trajectory');
legend(d.joint_names, 'Location', 'bestoutside');
subplot(2, 2, 2);
plot(steps, d.xyz_traj, 'LineWidth', 1.2); grid on;
xlabel('Step'); ylabel('Position [m]'); title('End-effector position');
legend({'x', 'y', 'z'});
subplot(2, 2, 3);
plot(steps, rad2deg(unwrap(d.rpy_traj)), 'LineWidth', 1.2); grid on;
xlabel('Step'); ylabel('Angle [deg]'); title('End-effector RPY');
legend({'roll', 'pitch', 'yaw'});
subplot(2, 2, 4);
plot3(d.xyz_traj(:, 1), d.xyz_traj(:, 2), d.xyz_traj(:, 3), 'b-', 'LineWidth', 1.5); hold on;
scatter3(d.xyz_traj(1, 1), d.xyz_traj(1, 2), d.xyz_traj(1, 3), 60, 'g', 'filled');
scatter3(d.xyz_traj(end, 1), d.xyz_traj(end, 2), d.xyz_traj(end, 3), 60, 'r', 'filled');
grid on; axis equal; xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('End-effector 3D path');
saveas(gcf, fullfile(out_dir, '01_forward_kinematics.png'));
close(gcf);
end

function plot_inverse_kinematics(results_dir, out_dir)
path = fullfile(results_dir, 'inverse_kinematics.mat');
require_file(path);
d = load(path);
steps = 0:size(d.q_traj_ref, 1)-1;
figure('Visible', 'off', 'Position', [100, 100, 1200, 800]);
subplot(2, 2, 1);
plot(steps, d.q_traj_ref, '-', steps, d.q_traj_ik, '--', 'LineWidth', 1.1); grid on;
xlabel('Step'); ylabel('Joint angle [rad]'); title('Reference and IK joints');
subplot(2, 2, 2);
semilogy(steps, max(d.position_error, 1e-16), 'r-o', 'LineWidth', 1.1); grid on;
xlabel('Step'); ylabel('Position error [m]'); title('IK position error');
subplot(2, 2, 3);
bar(steps, double(d.ik_success)); ylim([0, 1.2]); grid on;
xlabel('Step'); ylabel('Success'); title('IK success');
subplot(2, 2, 4);
diff_q = d.q_traj_ik - d.q_traj_ref;
plot(steps, diff_q, 'LineWidth', 1.1); grid on;
xlabel('Step'); ylabel('IK - reference [rad]'); title('Joint difference');
saveas(gcf, fullfile(out_dir, '02_inverse_kinematics.png'));
close(gcf);
end

function plot_inverse_dynamics(results_dir, out_dir)
path = fullfile(results_dir, 'inverse_dynamics.mat');
require_file(path);
d = load(path);
steps = 0:size(d.q_traj, 1)-1;
figure('Visible', 'off', 'Position', [100, 100, 1200, 800]);
subplot(2, 2, 1);
plot(steps, d.q_traj, 'LineWidth', 1.1); grid on;
xlabel('Step'); ylabel('q [rad]'); title('Joint position input');
subplot(2, 2, 2);
plot(steps, d.tau_drive, 'LineWidth', 1.1); grid on;
xlabel('Step'); ylabel('Torque [Nm]'); title('Drive torque');
subplot(2, 2, 3);
plot(steps, d.tau_constraint, 'LineWidth', 1.1); grid on;
xlabel('Step'); ylabel('Torque [Nm]'); title('Constraint torque');
subplot(2, 2, 4);
plot(steps, d.tau_total, 'LineWidth', 1.1); grid on;
xlabel('Step'); ylabel('Torque [Nm]'); title('Total torque');
saveas(gcf, fullfile(out_dir, '03_inverse_dynamics.png'));
close(gcf);
end

function plot_forward_dynamics(results_dir, out_dir)
path = fullfile(results_dir, 'forward_dynamics.mat');
require_file(path);
d = load(path);
figure('Visible', 'off', 'Position', [100, 100, 1200, 650]);
subplot(1, 3, 1);
bar([d.q(:), d.qd(:)]); grid on;
xlabel('Joint'); title('q and qd'); legend({'q [rad]', 'qd [rad/s]'});
subplot(1, 3, 2);
bar(d.tau_drive(:)); grid on;
xlabel('Joint'); ylabel('Torque [Nm]'); title('Drive torque');
subplot(1, 3, 3);
bar([d.qdd_target(:), d.qdd(:)]); grid on;
xlabel('Joint'); ylabel('qdd [rad/s^2]'); title('Target and computed qdd');
legend({'target', 'computed'});
saveas(gcf, fullfile(out_dir, '04_forward_dynamics.png'));
close(gcf);
end
