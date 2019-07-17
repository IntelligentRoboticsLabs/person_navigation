cmake_minimum_required(VERSION 2.8.3)
project(person_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  bica_planning
  tf
  geometry_msgs
  nav_msgs
  visualization_msgs
  bica
  move_base_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  costmap_2d
  image_transport
  topological_navigation
  topological_navigation_msgs
  roslint
)


catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES vizbox
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
#    DEPENDS system_lib
)

include_directories(
# 	  include
    ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references")
roslint_cpp()


#add_executable(person_follower_node
#  src/person_follower_node.cpp
#  src/Scan.cpp
#)

#add_executable(rp_approach_person src/actions/RP_approach_person.cpp)
#add_executable(rp_follow_person src/actions/RP_follow_person.cpp)
add_executable(rp_guide_move src/actions/RP_guide_move.cpp)
add_executable(rp_guide_cross src/actions/RP_guide_cross.cpp)
add_executable(rp_guide_navigate src/actions/RP_guide_navigate.cpp)
add_executable(rp_guide_person src/actions/RP_guide_person.cpp)

#add_dependencies(person_follower_node ${catkin_EXPORTED_TARGETS} )
#add_dependencies(rp_approach_person ${catkin_EXPORTED_TARGETS} )
#add_dependencies(rp_follow_person ${catkin_EXPORTED_TARGETS} )
add_dependencies(rp_guide_move ${catkin_EXPORTED_TARGETS} )
add_dependencies(rp_guide_cross ${catkin_EXPORTED_TARGETS} )
add_dependencies(rp_guide_navigate ${catkin_EXPORTED_TARGETS} )
add_dependencies(rp_guide_person ${catkin_EXPORTED_TARGETS} )

#target_link_libraries(person_follower_node ${catkin_LIBRARIES} )
#target_link_libraries(rp_approach_person ${catkin_LIBRARIES} )
#target_link_libraries(rp_follow_person ${catkin_LIBRARIES} )
target_link_libraries(rp_guide_move ${catkin_LIBRARIES} )
target_link_libraries(rp_guide_cross ${catkin_LIBRARIES} )
target_link_libraries(rp_guide_navigate ${catkin_LIBRARIES} )
target_link_libraries(rp_guide_person ${catkin_LIBRARIES} )


## Mark executables and/or libraries for installation
install(TARGETS
    #person_follower_node
    #rp_approach_person
    #rp_follow_person
    rp_guide_move
    rp_guide_cross
    rp_guide_navigate
    rp_guide_person
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

install(DIRECTORY pddl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pddl/
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-test test/test_PersonNav.launch test/test_PersonNav.cpp)

  target_link_libraries(
    ${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
  )

endif()
