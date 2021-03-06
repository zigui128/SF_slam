cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
)

find_package(GTest REQUIRED)

find_package(Ceres REQUIRED)

add_service_files(
   FILES
   saveMap.srv
   optimizeMap.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include(cmake/g2o.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${GTEST_INCLUDE_DIRS})

include(cmake/global_defination.cmake)
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
file(GLOB_RECURSE GTEST_SRCS "*_test.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})
list(REMOVE_ITEM ALL_SRCS ${GTEST_SRCS})

add_executable(test_frame_node src/apps/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(data_pretreat_node src/apps/data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


add_executable(front_end_node src/apps/front_end_node.cpp ${ALL_SRCS})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(back_end_node src/apps/back_end_node.cpp ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS} optimizeMap_gencpp)
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(loop_closing_node src/apps/loop_closing_node.cpp ${ALL_SRCS})
add_dependencies(loop_closing_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(loop_closing_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(viewer_node src/apps/viewer_node.cpp ${ALL_SRCS})
add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(simple_test src/simple_test/simple_test.cpp)
target_include_directories(simple_test PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(simple_test ${GTEST_LIBRARIES} ${CERES_LIBRARIES} pthread)

add_executable(imu_preintegration_test src/models/imu_preintegration/Imupreintegration_test.cpp src/models/imu_preintegration/imu_simulation.cpp
    src/models/imu_preintegration/ImuPreintegration.cpp src/models/imu_preintegration/utility.cpp)
target_link_libraries(imu_preintegration_test ${GTEST_LIBRARIES} pthread)
