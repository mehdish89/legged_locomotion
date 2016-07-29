cmake_minimum_required(VERSION 2.8.3)
project(daedalus_ikfast_LEG1_plugin)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(include)

set(IKFAST_LIBRARY_NAME daedalus_LEG1_moveit_ikfast_plugin)

find_package(LAPACK REQUIRED)

add_library(${IKFAST_LIBRARY_NAME} src/daedalus_LEG1_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

# add_executable(daedalus_LEG1_ikfast_moveit src/daedalus_LEG1_ikfast_moveit_plugin.cpp)
# target_link_libraries(daedalus_LEG1_ikfast_moveit ${catkin_LIBRARIES})
# add_dependencies(daedalus_LEG1_ikfast_moveit daedalus_LEG1_ikfast_moveit_generate_messages_cpp)

install(TARGETS ${IKFAST_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  daedalus_LEG1_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
