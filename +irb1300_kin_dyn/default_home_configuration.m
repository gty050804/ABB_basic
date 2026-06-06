function q = default_home_configuration(n_joints)
%DEFAULT_HOME_CONFIGURATION Return the demo home joint vector.
if nargin < 1
    n_joints = 6;
end
base = [0, -0.4, 0.6, 0, 0.5, 0];
q = base(1:n_joints);
end
