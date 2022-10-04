cmake_minimum_required(VERSION 3.1)
project(buff_infomation)

find_package(catkin REQUIRED COMPONENTS 
roscpp
 rospy 
 std_msgs 
 actionlib
 genmsg
 roborts_decision
 roborts_msgs
 geometry_msgs
)
find_package(Boost REQUIRED)

generate_messages(DEPENDENCIES std_msgs )
catkin_package( 
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(disablemovement src/disablemovement.cpp)
target_link_libraries(disablemovement ${catkin_LIBRARIES})
add_dependencies(disablemovement buff_infomation_generate_messages_cpp)

add_executable(disableshooting src/disableshooting.cpp)
target_link_libraries(disableshooting ${catkin_LIBRARIES})
add_dependencies(disableshooting buff_infomation_generate_messages_cpp)


add_executable(hprecovery src/hprecovery.cpp)
target_link_libraries(hprecovery ${catkin_LIBRARIES})
add_dependencies(hprecovery buff_infomation_generate_messages_cpp)


add_executable(bulletsupply src/bulletsupply.cpp)
target_link_libraries(bulletsupply ${catkin_LIBRARIES})
add_dependencies(bulletsupply buff_infomation_generate_messages_cpp)


add_executable(set_initialpose src/set_initialpose.cpp)
target_link_libraries(set_initialpose ${catkin_LIBRARIES})
add_dependencies(set_initialpose buff_infomation_generate_messages_cpp)

add_executable(chassisctr src/chassisctr.cpp)
target_link_libraries(chassisctr ${catkin_LIBRARIES})
add_dependencies(chassisctr buff_infomation_generate_messages_cpp)

add_executable(gimbalctr src/gimbalctr.cpp)
target_link_libraries(gimbalctr ${catkin_LIBRARIES})
add_dependencies(gimbalctr buff_infomation_generate_messages_cpp)

