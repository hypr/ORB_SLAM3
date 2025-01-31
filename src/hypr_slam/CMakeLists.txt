cmake_minimum_required(VERSION 3.4)
project(hypr_slam)

# Find Python
# find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python 3.12 EXACT REQUIRED COMPONENTS Interpreter Development)

# Find pybind11
find_package(pybind11 REQUIRED)

# Add the Python module
pybind11_add_module(hypr_slam 
    slam_binding.cpp
)

# Link against ORB_SLAM3 library
target_link_libraries(hypr_slam PRIVATE 
    ${PROJECT_SOURCE_DIR}/../../lib/libORB_SLAM3.so
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)

# Include directories
target_include_directories(hypr_slam PRIVATE
    ${PROJECT_SOURCE_DIR}/../../
    ${PROJECT_SOURCE_DIR}/../../include
    ${PROJECT_SOURCE_DIR}/../../include/CameraModels
    ${PROJECT_SOURCE_DIR}/../../Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)