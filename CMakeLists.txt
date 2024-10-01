cmake_minimum_required(VERSION 3.5)
project(adi_3dtof_image_stitching)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

if (ENABLE_GPU)
  find_package(CUDA REQUIRED)
endif()


if(CUDA_FOUND)
message(STATUS "CUDA FOUND")
add_compile_definitions(ENABLE_GPU_OPTIMIZATION)
#SET(CUDA_NVCC_FLAGS "-arch=compute_30" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
set(LIB_TYPE STATIC) 
endif()  

if(OPENMP_FOUND)
  add_compile_definitions(ENABLE_OPENMP_OPTIMIZATION)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

#add_compile_definitions(ENABLE_FUNCTION_PROFILING)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  /opt/ros/humble/include
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
if(CUDA_FOUND)
cuda_add_executable(${PROJECT_NAME}_node src/adi_3dtof_image_stitching.cpp src/module_profile.cpp src/stitch_frames.cpp src/callback_impl.cpp src/adi_3dtof_image_stitching_output_thread.cpp src/output_sensor_file.cpp src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp src/stitch_frames_core_gpu.cu)
else()
add_executable(${PROJECT_NAME}_node src/adi_3dtof_image_stitching.cpp src/module_profile.cpp src/stitch_frames.cpp src/callback_impl.cpp src/adi_3dtof_image_stitching_output_thread.cpp src/output_sensor_file.cpp src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp src/stitch_frames_core_cpu.cpp)
endif()

ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions cv_bridge image_geometry image_transport OpenCV)

if(OPENMP_FOUND)
  target_compile_options(${PROJECT_NAME}_node PRIVATE ${OpenMP_FLAGS})
endif()

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against
if(OPENMP_FOUND)
 target_link_libraries(${PROJECT_NAME}_node
	  ${Boost_SYSTEM_LIBRARY}
	  ${OpenCV_LIBRARIES}
    ${OpenMP_LIBS}
  )
else()
  target_link_libraries(${PROJECT_NAME}_node
    ${Boost_SYSTEM_LIBRARY}
    ${OpenCV_LIBRARIES}
  )
endif()


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node
   DESTINATION lib/${PROJECT_NAME}
 )
install(DIRECTORY rviz 
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

## Add folders to be run by python nosetests

ament_package()
