cmake_minimum_required (VERSION 3.6)


#Add sources to the project
set(SOURCES_PREFIX ${CMAKE_SOURCE_DIR}/src)
add_subdirectory(src)
list(APPEND SOS_LIB_SOURCELIST ${SOURCES})

if( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin" )
  set(SOS_TOOLCHAIN_CMAKE_PATH /Applications/StratifyLabs-SDK/Tools/gcc/arm-none-eabi/cmake)
endif()
if( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows" )
  set(SOS_TOOLCHAIN_CMAKE_PATH C:/StratifyLabs-SDK/Tools/gcc/arm-none-eabi/cmake)
endif()
set(CMAKE_TOOLCHAIN_FILE ${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib-toolchain.cmake)
set(SOS_LIB_NAME sos)
project(${SOS_LIB_NAME} C ASM)

install(DIRECTORY include/ DESTINATION include/mcu/arch/lpc)

set(SOS_LIB_INCLUDE_DIRECTORIES include)

set(SOS_LIB_OPTION mcu_lpc17xx)
set(SOS_LIB_DEFINITIONS __lpc17xx)
set(SOS_LIB_ARCH v7m)
set(SOS_LIB_TYPE release)
include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)
set(SOS_LIB_TYPE debug)
include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)

set(SOS_LIB_OPTION mcu_lpc407x_8x)
set(SOS_LIB_DEFINITIONS __lpc407x_8x)
set(SOS_LIB_ARCH v7em_f4sh)
set(SOS_LIB_TYPE release)
#include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)
set(SOS_LIB_TYPE debug)
#include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)

install(DIRECTORY ldscript/ DESTINATION lib/ldscripts)
