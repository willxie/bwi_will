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
        tf
        # pcl_ros
)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(artoolkit)




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

add_library(map
  src/map/map.c
  src/map/map_cspace.cpp
  src/map/map_range.c
  src/map/map_store.c
  src/map/map_draw.c
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  include

)

add_executable(object_search src/object_search.cpp)
target_link_libraries(object_search
    ${catkin_LIBRARIES}
#     ${PCL_LIBRARY_DIRS}
#     ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    map
)


add_executable(object_search_find src/object_search_find.cpp)
target_link_libraries(object_search_find
    ${catkin_LIBRARIES}
#     ${PCL_LIBRARY_DIRS}
#     ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    map
)



add_executable(object_search_find_random src/object_search_find_random.cpp)
target_link_libraries(object_search_find_random
    ${catkin_LIBRARIES}
#     ${PCL_LIBRARY_DIRS}
#     ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    map
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
