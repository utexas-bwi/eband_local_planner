include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

##############################################################################
# Qt Environment
##############################################################################

# rosbuild_include does not work in rosbuild2 (no rospack assumption)
# but we do have the source directories scanned by toplevel.cmake
# rosbuild_include(qt_build qt-ros)
include(${qt_build_SOURCE_DIR}/qt-ros.cmake)

rosbuild_prepare_qt4(QtCore QtGui)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/segbot_controller/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

rosbuild_add_executable(segbot_controller ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(segbot_controller ${QT_LIBRARIES})
