function G = spatial_inertia(mass, com, inertia)
%SPATIAL_INERTIA Spatial inertia from mass, COM, and inertia about COM.
cx = irb1300_kin_dyn.skew(com);
top_left = inertia - mass * (cx * cx);
top_right = mass * cx;
bottom_left = (mass * cx).';
bottom_right = mass * eye(3);
G = [top_left, top_right; bottom_left, bottom_right];
end
