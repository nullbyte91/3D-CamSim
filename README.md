# 3D CAM SIMULATION



## Table of content

- [3D CAM SIMULATION - This project aims to simulate a 3D scanner](#3d-cam-simulation)
  - [Table of content](#table-of-content)
  - [🗃Project structure](#project-structure)
  - [🖖 Quick Start](#quick-start)
  - [💌 Acknowledgments](#-acknowledgments)

## 🗃 Project structure
```bash
├── CMakeLists.txt
├── data
│   └── kinect-pattern_3x3.png
├── include
│   ├── 3d_simulation.hpp
│   ├── camera.hpp            # Camera information
│   ├── geometry.hpp          # Vector, Matrix operations
│   ├── objectMeshModel.hpp   # Mesh file handling
├── model
│   └── cube.obj              #Input mesh
├── README.md
└── src
    ├── 3d_simulation.cpp     # Simulation core framework
    └── main.cpp              
```

## 🖖 Quick Start
```
git clone https://github.com/nullbyte91/3D-CamSim.git
cd 3D-CamSim && mkdir build && cmake ..
../bin/3d_camsim
```

## 💌 Acknowledgments
[The Pinhole camera model and projection ](https://www.scratchapixel.com/lessons/3d-basic-rendering/3d-viewing-pinhole-camera)<br>
[Depth Map Zbuffer pipeline](https://www.cs.utexas.edu/~fussell/courses/cs384g-spring2017/lectures/Lecture9-Zbuffer_pipeline.pdf)<br>
[OpenGL Depth Simulation](https://github.com/Jmeyer1292/gl_depth_sim)<br>
[Carla stereo camera projection matrix](https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/draw_skeleton.py)<br>

