% PyBullet comparison is not available in the MATLAB port.
%
% The Python project uses pybullet as an independent simulator backend.
% This MATLAB implementation keeps the theoretical POE/RNEA computation path.
% Run scripts/05_verify_dynamics.m for internal dynamics consistency checks.
error(['PyBullet comparison is not implemented in MATLAB. ', ...
    'Use the Python project for simulator comparison, or run scripts/05_verify_dynamics.m for theoretical checks.']);
