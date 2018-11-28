cmake_minimum_required(VERSION 2.8.3)
project(prediction_communicator)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        actionlib
        std_msgs
        std_srvs
        diagnostic_msgs
        rosplan_dispatch_msgs
        rosplan_knowledge_msgs
        message_generation
        actionlib_msgs
        genmsg
        )



## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

###################################
## catkin specific configuration ##
###################################

add_service_files(
        FILES
        CSVService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        actionlib_msgs
        diagnostic_msgs
        std_msgs
)

## Declare things to be passed to dependent projects
catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        CATKIN_DEPENDS roscpp rospy actionlib std_msgs std_srvs diagnostic_msgs rosplan_dispatch_msgs rosplan_knowledge_msgs message_generation actionlib_msgs genmsg
        DEPENDS
)

###########
## Build ##
###########

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

find_package(FLEX REQUIRED)



# Disable deprecated declarations warning (about std::auto_ptr)
add_definitions(-Wno-deprecated-declarations)

## visualisation
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

## Declare cpp executables
add_executable(communicatorInterface src/CommunicatorInterface.cpp src/CSVStateGenerator.cpp)


## Add dependencies
add_dependencies(communicatorInterface ${catkin_EXPORTED_TARGETS})


## Specify libraries against which to link a library or executable target
target_link_libraries(communicatorInterface ${catkin_LIBRARIES})


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
