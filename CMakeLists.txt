cmake_minimum_required (VERSION 2.8)
cmake_policy(SET CMP0054 NEW)

project(3d_camsim)

# C++ Compiler Flags
set (CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -Wunused-variable")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV QUIET REQUIRED COMPONENTS core highgui)
if(OpenCV_FOUND)
  message("Found OpenCV")
  include_directories(${OpenCV_INCLUDE_DIRS})
  set(OpenCV_LIBS ${OpenCV_LIBS} opencv_imgcodecs)
  set(LIBS ${LIBS} ${OpenCV_LIBS})
endif(OpenCV_FOUND)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(3d_camsim src/main.cpp src/3d_simulation.cpp)
target_link_libraries(3d_camsim CGAL assimp ${OpenCV_LIBS})
