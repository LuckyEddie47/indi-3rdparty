cmake_minimum_required(VERSION 3.13)
PROJECT(indi_ocs C CXX)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")
include(GNUInstallDirs)

set(INDI_OCS_VERSION_MAJOR 1)
set(INDI_OCS_VERSION_MINOR 2)

find_package(INDI REQUIRED)
find_package(Nova REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_ocs.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_ocs.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})

include(CMakeCommon)

########### OCS  ###########
set(indi_ocs_srcs
   ${CMAKE_CURRENT_SOURCE_DIR}/ocs.cpp
   )

add_executable(indi_ocs ${indi_ocs_srcs})

target_link_libraries(indi_ocs ${INDI_LIBRARIES} ${NOVA_LIBRARIES})

install(TARGETS indi_ocs RUNTIME DESTINATION bin )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_ocs.xml DESTINATION ${INDI_DATA_DIR})
