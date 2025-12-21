cmake_minimum_required(VERSION 3.13)
PROJECT(indi_onstep_aux C CXX)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")
include(GNUInstallDirs)

set(INDI_ONSTEP_AUX_VERSION_MAJOR 0)
set(INDI_ONSTEP_AUX_VERSION_MINOR 2)

find_package(INDI REQUIRED)
find_package(Nova REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_onstep_aux.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_onstep_aux.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})

include(CMakeCommon)

########### ONSTEP_AUX  ###########
set(indi_onstep_aux_srcs
   ${CMAKE_CURRENT_SOURCE_DIR}/onstep_aux.cpp
   )

add_executable(indi_onstep_aux ${indi_onstep_aux_srcs})

target_link_libraries(indi_onstep_aux ${INDI_LIBRARIES} ${NOVA_LIBRARIES})

install(TARGETS indi_onstep_aux RUNTIME DESTINATION bin )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_onstep_aux.xml DESTINATION ${INDI_DATA_DIR})
