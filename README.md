# 3D CAM SIMULATION



## Table of content

- [3D CAM SIMULATION - This project aims to simulate a 3D scanner](#3d-cam-simulation)
  - [Table of content](#table-of-content)
  - [πProject structure](#project-structure)
  - [π Quick Start](#quick-start)
  - [π Acknowledgments](#-acknowledgments)

## π Project structure
```bash
βββ CMakeLists.txt
βββ data
β   βββ kinect-pattern_3x3.png
βββ include
β   βββ 3d_simulation.hpp
β   βββ camera.hpp            # Camera information
β   βββ geometry.hpp          # Vector, Matrix operations
β   βββ objectMeshModel.hpp   # Mesh file handling
βββ model
β   βββ cube.obj              #Input mesh
βββ README.md
βββ src
    βββ 3d_simulation.cpp     # Simulation core framework
    βββ main.cpp              
```

## π Quick Start
```
git clone https://github.com/nullbyte91/3D-CamSim.git
cd 3D-CamSim && mkdir build && cmake ..
../bin/3d_camsim
```

## π Acknowledgments
[The Pinhole camera model and projection ](https://www.scratchapixel.com/lessons/3d-basic-rendering/3d-viewing-pinhole-camera)<br>
[Depth Map Zbuffer pipeline](https://www.cs.utexas.edu/~fussell/courses/cs384g-spring2017/lectures/Lecture9-Zbuffer_pipeline.pdf)<br>
[OpenGL Depth Simulation](https://github.com/Jmeyer1292/gl_depth_sim)<br>
[Carla stereo camera projection matrix](https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/draw_skeleton.py)<br>

