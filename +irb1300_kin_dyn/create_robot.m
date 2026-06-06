function robot = create_robot(urdf_path)
%CREATE_ROBOT Build the high-level IRB1300 robot struct.
if nargin < 1 || isempty(urdf_path)
    urdf_path = irb1300_kin_dyn.default_urdf_path();
end
if ~isfile(urdf_path)
    error('IRB1300:URDFNotFound', 'URDF not found: %s', urdf_path);
end
[joints, links, base_link] = irb1300_kin_dyn.load_urdf_model(urdf_path);
[screws, m_home] = irb1300_kin_dyn.build_poe_parameters(joints, base_link, 'link7');
robot = struct();
robot.urdf_path = urdf_path;
robot.joints = joints;
robot.links = links;
robot.base_link = base_link;
robot.ee_link = 'link7';
robot.joint_names = {'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'};
robot.n_joints = numel(joints);
robot.screws = screws;
robot.m_home = m_home;
robot.q_lower = [joints.lower].';
robot.q_upper = [joints.upper].';
robot.gravity = [0; 0; -9.81];
robot.mlist = irb1300_kin_dyn.build_mlist(joints);
robot.glist = irb1300_kin_dyn.build_glist(joints, links);
robot.alist = irb1300_kin_dyn.body_screws_in_link_frames(robot);
end
