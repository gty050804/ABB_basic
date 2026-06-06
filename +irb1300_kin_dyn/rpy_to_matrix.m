function R = rpy_to_matrix(roll, pitch, yaw)
%RPY_TO_MATRIX Convert roll-pitch-yaw angles to a rotation matrix.
cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);
R = [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr;
     sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr;
     -sp, cp*sr, cp*cr];
end
