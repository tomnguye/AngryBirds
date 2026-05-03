cmake_minimum_required(VERSION 3.10)
project(AngryBirds)

# ==========================================
# Eigen3 Include Directories for various OS
# ==========================================
include_directories(/usr/include/eigen3)          # Standard Linux (Ubuntu, etc.)
include_directories(/usr/local/include/eigen3)    # Intel Mac
include_directories(/opt/homebrew/include/eigen3) # Apple Silicon Mac (M1/M2/M3)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Optimization flags
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native  -funroll-loops")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(glfw3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# include_directories(/usr/local/include ./include)
add_executable(main main.cpp CollisionDetection.hpp CollisionDetection.cpp CollisionEvent.hpp PhysicsEngine.hpp PhysicsEngine.cpp PhysicsObject.hpp)
target_link_libraries(main ${OpenCV_LIBRARIES} GL GLEW glfw)
