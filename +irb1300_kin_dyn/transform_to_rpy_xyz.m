function [xyz, rpy] = transform_to_rpy_xyz(T)
%TRANSFORM_TO_RPY_XYZ Extract xyz and roll-pitch-yaw from a transform.
xyz = T(1:3, 4).';
R = T(1:3, 1:3);
pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
if abs(cos(pitch)) > 1e-8
    roll = atan2(R(3,2), R(3,3));
    yaw = atan2(R(2,1), R(1,1));
else
    roll = atan2(-R(2,3), R(2,2));
    yaw = 0;
end
rpy = [roll, pitch, yaw];
end
