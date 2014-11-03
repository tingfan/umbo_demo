cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(umbo_demo)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
message("${CMAKE_MODULE_PATH}")
set(CMAKE_MODULE_PATH "/usr/local/share/pcl-1.8/")
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(openni2_viewer openni2_viewer.cpp)
target_link_libraries(openni2_viewer ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})


#add_executable(qtVtkVisualizer qtVtkVisualizer.cpp)
#target_link_libraries(qtVtkVisualizer  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${OpenCV_LIBS})

add_executable(tripVolume tripVolume.cpp)
target_link_libraries(tripVolume ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${OpenCV_LIBS})

SET(APPS "\${CMAKE_INSTALL_PREFIX}/bin/tripVolume")
INSTALL(TARGETS tripVolume
    BUNDLE DESTINATION . COMPONENT Runtime
    RUNTIME DESTINATION bin COMPONENT Runtime
    )
 
set(YOUR_PLUGINS "") #empty for now
set(YOUR_LIBRARY_DIRS "")
install(CODE "
  include(BundleUtilities)
  fixup_bundle(\"${APPS}\" \"\${YOUR_PLUGINS}\" \"${YOUR_LIBRARY_DIRS}\")
  " COMPONENT Runtime)

