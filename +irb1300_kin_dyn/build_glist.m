function glist = build_glist(joints, links)
%BUILD_GLIST Spatial inertia matrices for each moving link.
n = numel(joints);
glist = zeros(6, 6, n);
for i = 1:n
    key = matlab.lang.makeValidName(joints(i).child);
    link = links.(key);
    glist(:, :, i) = irb1300_kin_dyn.spatial_inertia(link.mass, link.com, link.inertia);
end
end
