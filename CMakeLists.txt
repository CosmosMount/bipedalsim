cmake_minimum_required(VERSION 3.16)
project(bipedalsim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)

# Try to find MuJoCo. 
set(MUJOCO_FOLDER /opt/mujoco/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)
if (mujoco_FOUND)
    message(STATUS "Find mujoco:  ${mujoco_INCLUDE_DIRS}, ${mujoco_LIBRARIES}")
endif()

# Include directories
# 'Inc' contains your project headers
include_directories(Inc)

# Define the executable
add_executable(bipedalsim main.cpp)

# Link libraries
target_link_libraries(bipedalsim
    PRIVATE
    Eigen3::Eigen
    mujoco::mujoco
    glfw
    Threads::Threads
)
