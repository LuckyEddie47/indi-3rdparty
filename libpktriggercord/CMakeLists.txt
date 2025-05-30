cmake_minimum_required (VERSION 3.12)
project (libpktriggercord C)

set (PK_VERSION 0.85.03)
set (PK_SOVERSION 0)
set (PK_DATADIR /usr/share/pktriggercord)
set (PK_DEBUG true)
set (PK_WARNINGS true)

list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")
include (GNUInstallDirs)
find_package(INDI REQUIRED)

option(INDI_INSTALL_UDEV_RULES "Install UDEV rules" On)
set (UDEVRULES_INSTALL_DIR "/lib/udev/rules.d" CACHE STRING "Base directory for udev rules")

# Build library
add_definitions (-DPKTDATADIR="${PK_DATADIR}")
add_definitions (-DVERSION="${PK_VERSION}")

include_directories (${CMAKE_CURRENT_BINARY_DIR})
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/src/src/external/js0n)
include_directories (${INDI_INCLUDE_DIR})

# strip main function from pktriggercord
file (READ ${CMAKE_CURRENT_SOURCE_DIR}/src/pktriggercord-cli.c PK_CLI)
string (REGEX REPLACE "int main.*$" "" PK_CLI "${PK_CLI}")
file (WRITE ${CMAKE_CURRENT_BINARY_DIR}/libpktriggercord.c "${PK_CLI}")

set (libpktriggercord_SRCS
  ${CMAKE_CURRENT_BINARY_DIR}/libpktriggercord.c # stripped

  ${CMAKE_CURRENT_SOURCE_DIR}/src/pslr.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/pslr_model.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pslr_lens.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pslr_enum.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pslr_utils.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pslr_log.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/pslr_scsi.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/src/external/js0n/js0n.c
	${CMAKE_CURRENT_SOURCE_DIR}/src/pktriggercord-servermode.c
)

add_library (pktriggercord SHARED ${libpktriggercord_SRCS})
set_target_properties (pktriggercord PROPERTIES VERSION ${PK_VERSION} SOVERSION ${PK_SOVERSION})

# Build udev rules
add_custom_command (
  OUTPUT 95-pentax.rules
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/src/pentax.rules ${CMAKE_CURRENT_BINARY_DIR}/95-pentax.rules
)

add_custom_command (
  OUTPUT 95-samsung.rules
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/src/samsung.rules ${CMAKE_CURRENT_BINARY_DIR}/95-samsung.rules
)

add_custom_target (udev_rules ALL DEPENDS 95-pentax.rules 95-samsung.rules)

# Install udev rules
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux" AND INDI_INSTALL_UDEV_RULES)
  install (FILES ${CMAKE_CURRENT_BINARY_DIR}/95-pentax.rules ${CMAKE_CURRENT_BINARY_DIR}/95-samsung.rules DESTINATION ${UDEVRULES_INSTALL_DIR})
  install (FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/pentax_settings.json DESTINATION ${PK_DATADIR})
endif()

# Install header files
install (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/ DESTINATION include/libpktriggercord FILES_MATCHING PATTERN "*.h")
install (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION include/libpktriggercord)

# Install library
install (TARGETS pktriggercord DESTINATION ${CMAKE_INSTALL_LIBDIR})

