cmake_minimum_required(VERSION 2.8.3)
project(bwi_object_search)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


find_package(OpenCV)
# find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
#     add_definitions(-DHAVE_NEW_YAMLCPP)
# endif()

# add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        image_transport
        actionlib
        actionlib_msgs
        # bwi_kr_execution
        # message_generation
        move_base_msgs
        # pcl_ros
)


find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(artoolkit)

# add_message_files(
#   DIRECTORY msg
#   FILES
# 	ARMarker.msg
# 	ARMarkers.msg
# )


catkin_package(
    CATKIN_DEPENDS
        roscpp
        std_msgs
        actionlib_msgs
        move_base_msgs
        message_runtime
        artoolkit
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(object_search src/object_search.cpp)
target_link_libraries(object_search
    ${catkin_LIBRARIES}
#     ${PCL_LIBRARY_DIRS}
#     ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

add_executable(ar_object_transform src/ar_object_transform.cpp)
target_link_libraries(ar_object_transform
    ${catkin_LIBRARIES}
#     ${PCL_LIBRARY_DIRS}
#     ${OpenCV_LIBRARIES}
    # ${YAML_CPP_LIBRARIES}
)

#############
## Install ##
#############

# install(TARGETS scavenger
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
