project(mayfly)

set(FILES
	mayfly.cpp
)

set(LIBRARIES core pcap_rxtx fec camera arducam_tof imu)

if(PLATFORM_RPI)
	#include(FindPkgConfig) # pkg_check_modules
	#pkg_check_modules(LIBCAMERA REQUIRED libcamera)

	add_executable(${PROJECT_NAME} ${FILES})
	target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS} ${LIBCAMERA_LINK_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} ${LIBRARIES} uwb -lrt ${LIBPCAP_LINK_LIBRARIES})
else()
	add_executable(${PROJECT_NAME} ${FILES})
	if(MSVC)
		target_link_libraries(${PROJECT_NAME} ws2_32.lib iphlpapi.lib)
		target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS} ${LIBRARIES})
	endif()
	if(UNIX)
		if(APPLE)
			target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS} ${LIBRARIES})
		else()
			target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS} ${LIBRARIES} ${LIBPCAP_LINK_LIBRARIES})
		endif()
	endif()
endif()
