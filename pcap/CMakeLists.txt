project(pcap_rxtx)

include(FindPkgConfig)
pkg_check_modules(LIBPCAP libpcap)

if(LIBPCAP_FOUND)
	add_definitions(-DENABLE_LIBPCAP=1)
	include_directories(${CMAKE_SOURCE_DIR})
	include_directories(${CMAKE_SOURCE_DIR}/3rdparty/radiotap)

	message(STATUS "libpcap version ${LIBPCAP_VERSION}")
	message(STATUS "  inc: ${LIBPCAP_INCLUDEDIR}")
	message(STATUS "  lib: ${LIBPCAP_LIBRARIES}")

	set(src_files
		pcap_rxtx.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/radiotap/radiotap.c
	)
	add_library(${PROJECT_NAME} ${src_files})
	target_link_libraries(${PROJECT_NAME} PRIVATE shared ${LIBPCAP_LIBRARIES})
else()
	message(STATUS "pcap is not available")
	include_directories(${CMAKE_SOURCE_DIR})
	add_library(${PROJECT_NAME} pcap_rxtx.cpp)
endif()


set_source_files_properties(
  ${CMAKE_SOURCE_DIR}/3rdparty/radiotap/radiotap.c
  PROPERTIES COMPILE_FLAGS -Wno-address-of-packed-member
)
