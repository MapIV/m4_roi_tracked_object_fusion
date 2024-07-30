cmake_minimum_required(VERSION 3.14)
project(roi_tracked_object_fusion)
# add_compile_options(-Wno-unknown-pragmas)
add_compile_options(-Wno-unknown-pragmas -fopenmp)

find_package(autoware_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
autoware_package()

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Build non-CUDA dependent nodes
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/fusion_node.cpp
  src/debugger.cpp
  src/roi_tracked_object_fusion/node.cpp
  src/utils/utils.cpp
  src/utils/geometry.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "roi_tracked_object_fusion::RoiTrackedObjectFusionNode"
  EXECUTABLE roi_tracked_object_fusion_node
)

ament_auto_package(INSTALL_TO_SHARE
    launch
    config
)
