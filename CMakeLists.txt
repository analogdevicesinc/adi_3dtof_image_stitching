cmake_minimum_required(VERSION 3.0.2)
project(adi_3dtof_image_stitching)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_view
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  eigen_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)

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
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adi_3dtof_image_stitching
  CATKIN_DEPENDS roscpp std_msgs image_geometry sensor_msgs pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${compressed_depth_image_transport_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
if(CUDA_FOUND)
cuda_add_executable(${PROJECT_NAME}_node src/adi_3dtof_image_stitching.cpp src/stitch_frames.cpp src/callback_impl.cpp src/adi_3dtof_image_stitching_output_thread.cpp src/output_sensor_file.cpp src/stitch_frames_core_gpu.cu)
else()
add_executable(${PROJECT_NAME}_node src/adi_3dtof_image_stitching.cpp src/stitch_frames.cpp src/callback_impl.cpp src/adi_3dtof_image_stitching_output_thread.cpp src/output_sensor_file.cpp src/stitch_frames_core_cpu.cpp)
endif()

if(OPENMP_FOUND)
  target_compile_options(${PROJECT_NAME}_node PRIVATE ${OpenMP_FLAGS})
endif()

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against
if(OPENMP_FOUND)
 target_link_libraries(${PROJECT_NAME}_node
	  ${catkin_LIBRARIES}
	  ${OpenCV_LIBRARIES}
    ${OpenMP_LIBS}
  )
else()
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 install(TARGETS ${PROJECT_NAME}_node
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(DIRECTORY launch/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 )

 install(DIRECTORY rviz/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

## Add folders to be run by python nosetests

