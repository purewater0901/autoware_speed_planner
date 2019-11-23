cmake_minimum_required(VERSION 2.8.3)
project(speed_planner)

add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -std=c++11 -g -D_GLIBCXX_USE_CXX11_ABI=0")
set(GUROBI_INCLUDE_DIRECTORY "/opt/gurobi811/linux64/include")
set(GUROBI_LIB_DIRECTORY "/opt/gurobi811/linux64/lib")

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  vector_map
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  tf2_ros
  #tf2
  autoware_msgs
  vector_map
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GUROBI_INCLUDE_DIRECTORY}
)

#link_directories(${GUROBI_LIB_DIRECTORY})

add_executable(speed_planner src/speed_planner_node.cpp src/convex_speed_optimizer.cpp src/trajectory_loader.cpp src/collision_checker.cpp src/utils.cpp src/obstacle.cpp)
#target_link_libraries (speed_planner gurobi_c++ gurobi81 ${catkin_LIBRARIES} /opt/gurobi811/linux64/lib/libgurobi_g++5.2.a)
target_link_libraries (speed_planner ${catkin_LIBRARIES} /opt/gurobi811/linux64/lib/libgurobi_g++5.2.a /opt/gurobi811/linux64/lib/libgurobi81.so)
add_dependencies(speed_planner ${catkin_EXPORTED_TARGETS})

install(TARGETS
        speed_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

