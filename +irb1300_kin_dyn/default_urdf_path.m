function path = default_urdf_path()
%DEFAULT_URDF_PATH Return the default IRB 1300 URDF path.
pkg_dir = fileparts(mfilename('fullpath'));
root = fileparts(pkg_dir);
path = fullfile(root, 'irb1300_urdf', 'urdf', '1300_urdf.urdf');
end
