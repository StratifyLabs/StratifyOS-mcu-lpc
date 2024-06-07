cmake_minimum_required (VERSION 3.6)

set(SOS_NAME sos)
project(StratifyOS-mcu-stm32 C ASM)

set(CMSDK_NAME StratifyOS-mcu-lpc)
project(${CMSDK_NAME} C ASM)

set(CMSDK_INCLUDE_DIRECTORIES include)

option(BUILD_LPC_ALL "Build all LPC device families" ON)
option(BUILD_LPC17XX "Build Lpc17xx device family" OFF)
option(BUILD_LPC40XX "Build Lpc40xx device family" OFF)

cmsdk_add_subdirectory(LPC_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src)

function(add_targets FAMILY MCU CHIP VARIANT VARIANT_DEFINE ARCH)

	file(GLOB VARIANT_HEADERS
		${CMAKE_CURRENT_SOURCE_DIR}/include/*${VARIANT}.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/cmsis/${CHIP}.h)
	file(GLOB LDSCRIPT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ldscript/${FAMILY}*")

	set(SOURCELIST ${LPC_SOURCES} ${VARIANT_HEADERS} ${LDSCRIPT_SOURCES} include/lpc_arch.h)
	set(DEFINITIONS __${VARIANT} ${VARIANT_DEFINE}=1 MCU_ARCH_LPC)

	cmsdk_library_target(RELEASE StratifyOS ${MCU} release ${ARCH})
	cmsdk_library_target(DEBUG StratifyOS ${MCU} debug ${ARCH})
	add_library(${RELEASE_TARGET} STATIC)
	target_compile_definitions(${RELEASE_TARGET} PUBLIC ${DEFINITIONS} )
	target_compile_options(${RELEASE_TARGET} PRIVATE -Wno-address-of-packed-member)
	target_sources(${RELEASE_TARGET} PRIVATE ${SOURCELIST})
	target_include_directories(${RELEASE_TARGET}
		PUBLIC
		$<INSTALL_INTERFACE:include/lpc>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
		)
	target_compile_options(${RELEASE_TARGET} PUBLIC -Os)

	# create debug as a copy of release
	add_library(${DEBUG_TARGET} STATIC)
	cmsdk_copy_target(${RELEASE_TARGET} ${DEBUG_TARGET})

	target_link_libraries(${RELEASE_TARGET}
		PUBLIC
		StratifyOS_mcu_release_${ARCH}
		)

	target_link_libraries(${DEBUG_TARGET}
		PUBLIC
		StratifyOS_mcu_debug_${ARCH}
		)


	# Add SOS SDK build settings to targets
	cmsdk_library("${RELEASE_OPTIONS}")
	cmsdk_library("${DEBUG_OPTIONS}")
endfunction()

if(BUILD_LPC_ALL OR BUILD_LPC17XX)
	add_targets(lpc17 lpc17xx LPC17xx mcu_lpc17xx __lpc17xx v7m)

endif()

if(BUILD_LPC_ALL OR BUILD_LPC40XX)
	add_targets(lpc40 lpc407x_8x LPC407x_8x_177x_8x mcu_lpc407x_8x __lpc407x_8x v7em_f4sh)

endif()

install(DIRECTORY ldscript/ DESTINATION lib/ldscripts)
install(DIRECTORY include/ DESTINATION include/lpc)
