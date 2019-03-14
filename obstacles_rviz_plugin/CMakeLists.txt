cmake_minimum_required(VERSION 2.8.3)
project(obstacles_rviz_plugin)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS rviz msgs_perception )
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
#include(${QT_USE_FILE})
#add_definitions(-DQT_NO_KEYWORDS)

#qt4_wrap_cpp(MOC_FILES
#  src/obstacle_display.h
#  src/obstacle_array_display.h
#  src/obstacles/shape_obstacle.h
#
#)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

set(SOURCE_FILES
  src/obstacle_display.cpp
  src/obstacle_array_display.cpp
  src/obstacles/mesh_resource_obstacle.cpp
  #src/obstacles/pedestrian_mesh_resource_obstacle.cpp
  src/obstacles/shape_obstacle.cpp
  src/obstacles/obstacle_selection_handler.cpp
  src/obstacles/obstacle_base.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # for generation of message dependencies
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
