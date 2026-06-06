function f = spatial_wrench(f_ext_ee)
%SPATIAL_WRENCH Convert [Fx,Fy,Fz,Mx,My,Mz] to [Mx,My,Mz,Fx,Fy,Fz].
f_ext_ee = f_ext_ee(:);
f = [f_ext_ee(4:6); f_ext_ee(1:3)];
end
