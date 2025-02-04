cmake_minimum_required(VERSION 3.10...3.30)
project(rovio)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# Configuration options
set(ROVIO_NMAXFEATURE 25 CACHE STRING "Number of features for ROVIO")
set(ROVIO_NCAM 1 CACHE STRING "Number of enabled cameras")
set(ROVIO_NLEVELS 4 CACHE STRING "Number of image levels for the features")
set(ROVIO_PATCHSIZE 6 CACHE STRING "Size of patch (edge length in pixel)")
set(ROVIO_NPOSE 0 CACHE STRING "Additional estimated poses for external pose measurements")

add_definitions(
    -DROVIO_NMAXFEATURE=${ROVIO_NMAXFEATURE}
    -DROVIO_NCAM=${ROVIO_NCAM}
    -DROVIO_NLEVELS=${ROVIO_NLEVELS}
    -DROVIO_PATCHSIZE=${ROVIO_PATCHSIZE}
    -DROVIO_NPOSE=${ROVIO_NPOSE}
)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d)

# Find yaml-cpp library
find_path(YAML_CPP_INCLUDE_DIR yaml-cpp/yaml.h
    PATHS /opt/homebrew/include
    NO_DEFAULT_PATH
)
find_library(YAML_CPP_LIBRARY
    NAMES yaml-cpp
    PATHS /opt/homebrew/lib
    NO_DEFAULT_PATH
)

if(NOT YAML_CPP_LIBRARY)
    message(FATAL_ERROR "yaml-cpp library not found")
endif()

# Handle kindr
set(KINDR_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/kindr/include" CACHE PATH "Path to kindr include directory")
if(NOT EXISTS "${KINDR_INCLUDE_DIR}/kindr/Core")
    message(FATAL_ERROR "Kindr include directory not found at ${KINDR_INCLUDE_DIR}")
endif()

# Add lightweight filtering subdirectory
add_subdirectory(lightweight_filtering)

# Optional OpenGL visualization
option(MAKE_SCENE "Build with OpenGL visualization" OFF)
if(MAKE_SCENE)
    find_package(GLEW REQUIRED)
    find_package(GLUT REQUIRED)
    find_package(OpenGL REQUIRED)
    add_definitions(-DMAKE_SCENE=1)
endif()

# Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${KINDR_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/lightweight_filtering/include
)

if(MAKE_SCENE)
    include_directories(
        ${GLEW_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
        ${OpenGL_INCLUDE_DIRS}
    )
endif()

# Main library
if(MAKE_SCENE)
    add_library(${PROJECT_NAME} 
        src/Camera.cpp 
        src/FeatureCoordinates.cpp 
        src/FeatureDistance.cpp 
        src/Scene.cpp
    )
else()
    add_library(${PROJECT_NAME} 
        src/Camera.cpp 
        src/FeatureCoordinates.cpp 
        src/FeatureDistance.cpp
    )
endif()

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARY}
    lightweight_filtering
)

if(MAKE_SCENE)
    target_link_libraries(${PROJECT_NAME}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
        ${GLEW_LIBRARY}
    )
endif()

# Add executables
add_executable(rovio_standalone src/rovio_node.cpp)
target_link_libraries(rovio_standalone 
    ${PROJECT_NAME}
    ${YAML_CPP_LIBRARY}
)

# Testing (if available)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gtest/")
    enable_testing()
    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})
    
    add_executable(test_patch src/test_patch.cpp src/Camera.cpp src/FeatureCoordinates.cpp src/FeatureDistance.cpp)
    target_link_libraries(test_patch ${GTEST_LIBRARIES} pthread ${PROJECT_NAME})
    add_test(test_patch test_patch)
    
    add_executable(test_mlp src/test_mlp.cpp src/Camera.cpp src/FeatureCoordinates.cpp src/FeatureDistance.cpp)
    target_link_libraries(test_mlp ${GTEST_LIBRARIES} pthread ${PROJECT_NAME})
    add_test(test_mlp test_mlp)
endif()
