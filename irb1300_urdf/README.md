# ABB IRB 1300 URDF Model

This repository provides a URDF (Unified Robot Description Format) model of the ABB IRB 1300 industrial robotic arm. It is structured for use with **ROS 2 Humble** and can be visualized in **RViz** for simulation and testing purposes.

---

## 📁 Directory Structure

```
irb1300_urdf/
├── urdf/
│   ├── irb1300.urdf.xacro
│   ├── irb1300.xacro
│   └── meshes/
│       ├── base_link.stl
│       ├── link1.stl
│       └── ...
├── launch/
│   └── display.launch.py
├── config/
└── package.xml
```

---

## 🚀 Installation & Usage

### 1. Clone the repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/seyit05/irb1300_urdf.git
cd ~/ros2_ws
colcon build
```

### 2. Launch in RViz

```bash
source install/setup.bash
ros2 launch irb1300_urdf display.launch.py
```

---

## 🔧 Features

- ✅ Modular URDF/Xacro structure  
- ✅ Mesh-based STL visualization  
- ✅ Optimized for RViz visualization  
- 🔜 Upcoming: Joint limits, dynamics, control plugin integration

---

## 🤝 Contributing

Feel free to contribute or suggest improvements by opening [issues](https://github.com/seyit05/irb1300_urdf/issues) or pull requests. All contributions are welcome!

---

## 📬 Contact

- Email: seyitahmetkaris@gmail.com  
- GitHub: [seyit05](https://github.com/seyit05)
