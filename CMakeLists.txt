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
  pcl_ros
)

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
)

## Specify path based on host cpu architecture
if (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64|AARCH64")
  set(STITCH_CORE_CPU ${PROJECT_SOURCE_DIR}/libs/libstitch_frames_core.a)
  set(STITCH_CORE_GPU ${PROJECT_SOURCE_DIR}/libs/gpu/libstitch_frames_core.a)
elseif (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64|X86_64")
  set(STITCH_CORE_CPU ${PROJECT_SOURCE_DIR}/libs_linux_x86/libstitch_frames_core.a)
endif()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
if(CUDA_FOUND)
cuda_add_executable(${PROJECT_NAME}_node src/adi_3dtof_image_stitching.cpp src/stitch_frames.cpp src/callback_impl.cpp src/adi_3dtof_image_stitching_output_thread.cpp src/output_sensor_file.cpp)
else()
add_executable(${PROJECT_NAME}_node src/adi_3dtof_image_stitching.cpp src/stitch_frames.cpp src/callback_impl.cpp src/adi_3dtof_image_stitching_output_thread.cpp src/output_sensor_file.cpp)
endif()

if(OPENMP_FOUND)
  target_compile_options(${PROJECT_NAME}_node PRIVATE ${OpenMP_FLAGS})
endif()

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against
if(OPENMP_FOUND AND CUDA_FOUND)
 target_link_libraries(${PROJECT_NAME}_node
	  ${catkin_LIBRARIES}
	  ${OpenCV_LIBRARIES}
    ${OpenMP_LIBS}
    ${STITCH_CORE_GPU}
  )
elseif(OPENMP_FOUND)
 target_link_libraries(${PROJECT_NAME}_node
	  ${catkin_LIBRARIES}
	  ${OpenCV_LIBRARIES}
    ${OpenMP_LIBS}
    ${STITCH_CORE_CPU}
  )
else()
  target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${STITCH_CORE_CPU}
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries

## Add folders to be run by python nosetests

