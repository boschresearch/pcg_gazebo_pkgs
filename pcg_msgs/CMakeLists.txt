cmake_minimum_required(VERSION 2.8.3)
project(pcg_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    gazebo_msgs
    geometry_msgs
    std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    JointProperties.msg
    BulletPhysics.msg
    SimbodyPhysics.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    SetJointPosition.srv
    GetJointPosition.srv
    SetJointProperties.srv
    GetJointProperties.srv
    GetPhysicsEngineName.srv
    SetPhysicsProperties.srv
    GetPhysicsProperties.srv
    SetPhysicsProperty.srv
    GetPhysicsProperty.srv
    SetGlobalPhysicsProperties.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
      gazebo_msgs
      geometry_msgs
      std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        gazebo_msgs
        geometry_msgs
        std_msgs
)
