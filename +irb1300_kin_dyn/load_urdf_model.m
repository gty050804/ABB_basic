function [joints, links, base_link] = load_urdf_model(urdf_path)
%LOAD_URDF_MODEL Parse revolute joints and inertial properties from a URDF.
doc = xmlread(char(urdf_path));
root = doc.getDocumentElement();

link_nodes = root.getElementsByTagName('link');
links = struct();
for i = 0:link_nodes.getLength()-1
    node = link_nodes.item(i);
    name = char(node.getAttribute('name'));
    inertial = first_child(node, 'inertial');
    if isempty(inertial)
        continue;
    end
    [com, ~] = parse_xyz_rpy(first_child(inertial, 'origin'));
    mass_node = first_child(inertial, 'mass');
    inertia_node = first_child(inertial, 'inertia');
    inertia = [attr_double(inertia_node, 'ixx'), attr_double(inertia_node, 'ixy'), attr_double(inertia_node, 'ixz');
               attr_double(inertia_node, 'ixy'), attr_double(inertia_node, 'iyy'), attr_double(inertia_node, 'iyz');
               attr_double(inertia_node, 'ixz'), attr_double(inertia_node, 'iyz'), attr_double(inertia_node, 'izz')];
    safe_name = matlab.lang.makeValidName(name);
    links.(safe_name) = struct('name', name, 'mass', attr_double(mass_node, 'value'), ...
        'com', com(:), 'inertia', inertia);
end

raw = struct([]);
joint_nodes = root.getElementsByTagName('joint');
for i = 0:joint_nodes.getLength()-1
    node = joint_nodes.item(i);
    if ~strcmp(char(node.getAttribute('type')), 'revolute')
        continue;
    end
    [origin_xyz, origin_rpy] = parse_xyz_rpy(first_child(node, 'origin'));
    axis_node = first_child(node, 'axis');
    axis = parse_vec3(char(axis_node.getAttribute('xyz')));
    axis_norm = norm(axis);
    if axis_norm > 0
        axis = axis / axis_norm;
    end
    parent_node = first_child(node, 'parent');
    child_node = first_child(node, 'child');
    limit_node = first_child(node, 'limit');
    j = struct();
    j.name = char(node.getAttribute('name'));
    j.parent = char(parent_node.getAttribute('link'));
    j.child = char(child_node.getAttribute('link'));
    j.origin_xyz = origin_xyz(:);
    j.origin_rpy = origin_rpy(:);
    j.axis = axis(:);
    j.lower = attr_double(limit_node, 'lower');
    j.upper = attr_double(limit_node, 'upper');
    j.effort = attr_double(limit_node, 'effort');
    j.velocity = attr_double(limit_node, 'velocity');
    raw = [raw; j]; %#ok<AGROW>
end

children = string({raw.child});
parents = string({raw.parent});
base_candidates = setdiff(unique(parents), unique(children), 'stable');
if isempty(base_candidates)
    base_link = 'base_link';
else
    base_link = char(base_candidates(1));
end

joints = struct([]);
current = base_link;
while true
    idx = find(strcmp({raw.parent}, current), 1);
    if isempty(idx)
        break;
    end
    joints = [joints; raw(idx)]; %#ok<AGROW>
    current = raw(idx).child;
end
end

function child = first_child(node, tag)
list = node.getElementsByTagName(tag);
if list.getLength() == 0
    child = [];
else
    child = list.item(0);
end
end

function value = attr_double(node, name)
value = str2double(char(node.getAttribute(name)));
end

function [xyz, rpy] = parse_xyz_rpy(node)
xyz = zeros(3, 1);
rpy = zeros(3, 1);
if isempty(node)
    return;
end
if node.hasAttribute('xyz')
    xyz = parse_vec3(char(node.getAttribute('xyz')));
end
if node.hasAttribute('rpy')
    rpy = parse_vec3(char(node.getAttribute('rpy')));
end
end

function v = parse_vec3(text)
parts = sscanf(text, '%f');
v = parts(1:3);
end
