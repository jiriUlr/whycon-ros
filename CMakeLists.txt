cmake_minimum_required(VERSION 2.8.3)
project(whycon)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    geometry_msgs
    visualization_msgs
    message_generation
    dynamic_reconfigure
    tf2_ros
    )

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/whycon.cfg)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -ggdb")

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    Marker.msg
    MarkerArray.msg
    )

# Generate services in the 'srv' folder
add_service_files(
    FILES
    SelectMarker.srv
    SetCalibMethod.srv
    SetCalibPath.srv
    SetCoords.srv
    SetDrawing.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES charging
    #  CATKIN_DEPENDS image_transport roscpp rospy std_msgs
    CATKIN_DEPENDS roscpp image_transport message_runtime geometry_msgs visualization_msgs dynamic_reconfigure tf
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

add_executable(whycon
    src/whycon_ros/whycon_ros_node.cpp
    src/whycon/CWhycon.cpp
    src/whycon/CCircleDetect.cpp
    src/whycon/CRawImage.cpp
    src/whycon/CTimer.cpp
    src/whycon/CTransformation.cpp
    src/whycon/CNecklace.cpp
    )

add_dependencies(whycon ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(whycon ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

# install(TARGETS
#     whycon
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     )

# install(DIRECTORY include/${PROJECT_NAME}/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING PATTERN "*.h"
#     )

# install(DIRECTORY launch/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#     )

# install(DIRECTORY etc/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc
#     )

# install(DIRECTORY id/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/id
#     )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_charging.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

