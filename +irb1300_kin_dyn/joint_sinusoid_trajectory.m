function [q, qd, qdd] = joint_sinusoid_trajectory(n_steps, n_joints, dt, varargin)
%JOINT_SINUSOID_TRAJECTORY Sample sinusoidal joint position/velocity/acceleration.
% dt is accepted for API parity with the Python version.
unused_dt = dt; %#ok<NASGU>
p = inputParser;
addParameter(p, 'amplitude', 0.25);
addParameter(p, 'q_center', zeros(1, n_joints));
parse(p, varargin{:});
amplitude = p.Results.amplitude;
q_center = p.Results.q_center(:).';

t = linspace(0, 1.85 * pi, n_steps).';
omega = 2 * pi / (t(end) - t(1) + 1e-9);
q = zeros(n_steps, n_joints);
qd = zeros(n_steps, n_joints);
qdd = zeros(n_steps, n_joints);
for j = 1:n_joints
    phase = (j - 1) * pi / n_joints;
    q(:, j) = q_center(j) + amplitude * sin(t + phase);
    qd(:, j) = amplitude * cos(t + phase) * omega;
    qdd(:, j) = -amplitude * sin(t + phase) * omega^2;
end
end
