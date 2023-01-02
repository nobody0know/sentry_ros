cmake_minimum_required(VERSION 3.1)
project(simple_robot)

set(CMAKE_CXX_STANDARD 11)



#############################
#Dependencies
#############################
find_package(Threads)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    nav_msgs
    geometry_msgs
    std_msgs
    message_generation
    )
    
add_message_files(
FILES 
sc_rc_msg.msg
robot_ctrl.msg
vision.msg
)

add_service_files(
FILES 

)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(

INCLUDE_DIRS include
CATKIN_DEPENDS message_runtime
)



include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(simple_robot src/robot_node.cpp src/crc.cpp src/serial_device.cpp)
add_dependencies(simple_robot ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   ${PROJECT_NAME}_gencpp
                                   )
target_link_libraries(simple_robot
    ${CMAKE_THREAD_LIBS_INIT}
    ${catkin_LIBRARIES})
