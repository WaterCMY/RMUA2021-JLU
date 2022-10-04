project(roborts_decision)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  roborts_common
  roborts_msgs
  roborts_costmap
  roborts_base
  actionlib_msgs
  zcx
  sg
  )

find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#generate proto files
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
  DecisionProtoSrc
  DecisionProtoHds
  ${ProtoFiles}
  )

catkin_package()



add_executable(test_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
action_node/GoGoal.h
 action_node/SearchAction.h
action_node/BackBootArea.h
action_node/DefendAction.h
action_node/SwingDefend.h
action_node/ChaseAction.h
action_node/Follow.h
action_node/FrozenAction.h
action_node/Escape.h
  test.cpp
  behavior_tree/behavior_node.h
  executor/chassis_executor.cpp
 executor/gimbal_executor.cpp
 # zcx_node.cpp
  )

target_link_libraries(test_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(test_node

  roborts_msgs_generate_messages)

add_executable(vd_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
action_node/GoGoal.h
 action_node/SearchAction.h
action_node/BackBootArea.h
action_node/DefendAction.h
action_node/SwingDefend.h
  vd.cpp
  behavior_tree/behavior_node.h
  executor/chassis_executor.cpp
 executor/gimbal_executor.cpp
  )

target_link_libraries(vd_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(vd_node

  roborts_msgs_generate_messages)






add_executable(decision_node
    ${DecisionProtoHds}
    ${DecisionProtoSrc}
    action_node/BackBootArea.h
    action_node/ChaseAction.h
    action_node/ChassisLimited.h
    action_node/DefendAction.h
    action_node/Escape.h
    action_node/Follow.h
    action_node/FrozenAction.h
    action_node/GainBullets.h    
    action_node/GainBlood.h
    action_node/TurnDefend.h
    action_node/GimbalLimited.h
    action_node/SearchAction.h
    action_node/ShootAction.h
    decision_node.cpp
    behavior_tree/behavior_node.h
    executor/chassis_executor.cpp
    executor/gimbal_executor.cpp
    interact/mutualboard.h
    interact/correspond.h
 )

target_link_libraries(decision_node
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${catkin_INCLUDE_DIRS}
   )
 add_dependencies(decision_node 
   roborts_msgs_generate_messages)


add_executable(comone
    ${DecisionProtoHds}
    ${DecisionProtoSrc}
    comone.cpp
    interact/correspond.h
    executor/chassis_executor.cpp
    executor/gimbal_executor.cpp
 )

target_link_libraries(comone
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${catkin_INCLUDE_DIRS}
   )

add_executable(sentry_node
    ${DecisionProtoHds}
    ${DecisionProtoSrc}
    sentry.cpp
    executor/chassis_executor.cpp
    executor/gimbal_executor.cpp
 )

target_link_libraries(sentry_node
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${catkin_INCLUDE_DIRS}
   )



add_executable(comtwo
    ${DecisionProtoHds}
    ${DecisionProtoSrc}
    comtwo.cpp
    interact/correspond.h
    executor/chassis_executor.cpp
    executor/gimbal_executor.cpp
 )

target_link_libraries(comtwo
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${catkin_INCLUDE_DIRS}
   )





 add_executable(goaltopic_node
   ${DecisionProtoHds}
   ${DecisionProtoSrc}
   goaltopic.cpp
   )
 
 target_link_libraries(goaltopic_node
   PRIVATE
   
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   )
 
 add_dependencies(goaltopic_node
   roborts_msgs_generate_messages)

 
 #add_executable(gimbalswing_server
#   ${DecisionProtoHds}
   #${DecisionProtoSrc}
  # GimbalSwingServer.cpp
 #  )
 
 #target_link_libraries(gimbalswing_server
  # PRIVATE
 #  roborts_costmap
  # ${catkin_LIBRARIES}
 #  ${PROTOBUF_LIBRARIES}
 #  )
 #add_dependencies(gimbalswing_server
  # roborts_msgs_generate_messages)

# add_executable(gimbalswing_client
  # ${DecisionProtoHds}
  # ${DecisionProtoSrc}
  # GimbalSwingClient.cpp
  # )
 
 #target_link_libraries(gimbalswing_client
  # PRIVATE
 #  roborts_costmap
  # ${catkin_LIBRARIES}
 #  ${PROTOBUF_LIBRARIES}
  # )
# add_dependencies(gimbalswing_client
  # roborts_msgs_generate_messages)




add_executable(along_attack_node
    ${DecisionProtoHds}
    ${DecisionProtoSrc}
    action_node/BackBootArea.h
    action_node/ChaseAction.h
    action_node/ChassisLimited.h
    action_node/DefendAction.h
    action_node/Escape.h
    action_node/Follow.h
    action_node/FrozenAction.h
    action_node/GainBullets.h    
    action_node/GainBlood.h
    action_node/TurnDefend.h
    action_node/GimbalLimited.h
    action_node/SearchAction.h
    action_node/ShootAction.h
    along_attack.cpp
    behavior_tree/behavior_node.h
    executor/chassis_executor.cpp
    executor/gimbal_executor.cpp
    interact/mutualboard.h
    interact/correspond.h
 )

target_link_libraries(along_attack_node
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${catkin_INCLUDE_DIRS}
   )
 add_dependencies(along_attack_node 
   roborts_msgs_generate_messages)




add_executable(robot_angel_node
    ${DecisionProtoHds}
    ${DecisionProtoSrc}
    robotangel.cpp
    executor/chassis_executor.cpp
    executor/gimbal_executor.cpp
 )

   target_link_libraries(robot_angel_node
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${catkin_INCLUDE_DIRS}
   )
 add_dependencies(robot_angel_node 
   roborts_msgs_generate_messages)


add_executable(aaa_node
  ${DecisionProtoHds}
  ${DecisionProtoSrc}
  aaa.cpp
  executor/chassis_executor.cpp
 executor/gimbal_executor.cpp
  )

target_link_libraries(aaa_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  )
add_dependencies(aaa_node

  roborts_msgs_generate_messages)



  add_executable(stop_node
    ${DecisionProtoHds}
    ${DecisionProtoSrc}
    action_node/BackBootArea.h
    action_node/ChaseAction.h
    action_node/ChassisLimited.h
    action_node/DefendAction.h
    action_node/Escape.h
    action_node/Follow.h
    action_node/FrozenAction.h
    action_node/GainBullets.h    
    action_node/GainBlood.h
    action_node/TurnDefend.h
    action_node/GimbalLimited.h
    action_node/SearchAction.h
    action_node/ShootAction.h
    action_node/GoStaticPose.h
    stop.cpp
    behavior_tree/behavior_node.h
    executor/chassis_executor.cpp
    executor/gimbal_executor.cpp
    interact/mutualboard.h
    interact/correspond.h
 )

target_link_libraries(stop_node
   PRIVATE
   roborts_costmap
   ${catkin_LIBRARIES}
   ${PROTOBUF_LIBRARIES}
   ${catkin_INCLUDE_DIRS}
   )
 add_dependencies(stop_node 
   roborts_msgs_generate_messages)


   add_executable(new_stop_node
   ${DecisionProtoHds}
   ${DecisionProtoSrc}
   action_node/BackBootArea.h
   action_node/ChaseAction.h
   action_node/ChassisLimited.h
   action_node/DefendAction.h
   action_node/Escape.h
   action_node/Follow.h
   action_node/FrozenAction.h
   action_node/GainBullets.h    
   action_node/GainBlood.h
   action_node/TurnDefend.h
   action_node/GimbalLimited.h
   action_node/SearchAction.h
   action_node/ShootAction.h
   action_node/GoStaticPose.h
   newstop.cpp
   behavior_tree/behavior_node.h
   executor/chassis_executor.cpp
   executor/gimbal_executor.cpp
   interact/correspond.h
)

target_link_libraries(new_stop_node
  PRIVATE
  roborts_costmap
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  )
add_dependencies(new_stop_node 
  roborts_msgs_generate_messages)
