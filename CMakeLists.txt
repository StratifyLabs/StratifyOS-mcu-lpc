cmake_minimum_required (VERSION 3.6)

if( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin" )
  set(SOS_TOOLCHAIN_CMAKE_PATH /Applications/StratifyLabs-SDK/Tools/gcc/arm-none-eabi/cmake)
endif()
if( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows" )
  set(SOS_TOOLCHAIN_CMAKE_PATH C:/StratifyLabs-SDK/Tools/gcc/arm-none-eabi/cmake)
endif()
if( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux" )
	set(SOS_TOOLCHAIN_CMAKE_PATH /StratifyLabs-SDK/Tools/gcc/arm-none-eabi/cmake)
endif()

include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-variables.cmake)
set(CMAKE_TOOLCHAIN_FILE ${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib-toolchain.cmake)

set(SOS_NAME sos)
project(${SOS_NAME} C ASM)
#Add sources to the project
set(SOURCES_PREFIX ${CMAKE_SOURCE_DIR}/src)
add_subdirectory(src)
list(APPEND SOS_SOURCELIST ${SOURCES})
file(GLOB LDSCRIPTS ldscript/*)
list(APPEND SOS_SOURCELIST ${LDSCRIPTS})

install(DIRECTORY include/ DESTINATION include/mcu/arch/lpc)

set(SOS_INCLUDE_DIRECTORIES include)

option(BUILD_ALL "Build Lpc17xx device family" OFF)
option(BUILD_LPC17XX "Build Lpc17xx device family" OFF)
option(BUILD_LPC40XX "Build Lpc17xx device family" OFF)

if(BUILD_ALL OR BUILD_LPC17XX)
	set(SOS_OPTION mcu_lpc17xx)
	set(SOS_DEFINITIONS __lpc17xx)
	set(SOS_ARCH v7m)
	set(SOS_CONFIG release)
	include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)
	set(SOS_CONFIG debug)
	include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)
endif()

if(BUILD_ALL OR BUILD_LPC40XX)
	set(SOS_OPTION mcu_lpc407x_8x)
	set(SOS_DEFINITIONS __lpc407x_8x)
	set(SOS_ARCH v7em_f4sh)
	set(SOS_CONFIG release)
	include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)
	set(SOS_CONFIG debug)
	include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib.cmake)
endif()

install(DIRECTORY ldscript/ DESTINATION lib/ldscripts)
