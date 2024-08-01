cmake_minimum_required(VERSION 3.10)
project(mujocosim)

set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_SOURCE_DIR}/include ~/.mujoco/mujoco-3.2.0/include)
link_directories(~/.mujoco/mujoco-3.2.0/bin)

find_package(glfw3 REQUIRED)

add_executable(mujocosim main.cpp)
target_link_libraries(mujocosim glfw ~/.mujoco/mujoco-3.2.0/lib/libmujoco.so GL)