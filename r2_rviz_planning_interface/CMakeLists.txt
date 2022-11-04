cmake_minimum_required(VERSION 2.8.3)
project(r2_rviz_planning_interface)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Qt Stuff

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

# this line is very important!!!!
set(QT_LIBRARIES Qt5::Core Qt5::Widgets Qt5::Gui)

#find_package(Qt5 REQUIRED)
# include(${QT_USE_FILE})
# add_definitions(-DQT_NO_KEYWORDS)

#设置UIC的查找路径，UIC默认会在引用ui头文件的源代码路径下查找ui文件，而我们的ui文件存放在/ui文件夹下
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_SOURCE_DIR}/ui)
 
#设置可执行文件的输出路径，默认在哪个路径编译则会放在哪个路径，而我们是在/build下编译，要存放在/bin路径
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

find_package(catkin COMPONENTS
  roscpp
  rviz
  r2_planning_interface
  #interactive_markers
  pluginlib
  REQUIRED)

# catkin_package defines the env vars used in install later
catkin_package(
   LIBRARIES r2_rviz_planning_interface
   CATKIN_DEPENDS roscpp rviz pluginlib r2_planning_interface #interactive_markers
   DEPENDS
   INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
)

include_directories(${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIR} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${catkin_LIBRARY_DIRS})


# QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
# QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

# QT MOC pre-processor for object headers
set(QT_MOC_HEADERS include/r2_rviz_planning_interface/RVizPlanningInterface.h)
QT5_WRAP_CPP(MOC_SOURCES ${QT_MOC_HEADERS})

add_library(r2_rviz_planning_interface src/RVizPlanningInterface.cpp ${MOC_SOURCES})
target_link_libraries(r2_rviz_planning_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES})

install (TARGETS ${PROJECT_NAME}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY icons/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES
  r2_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
