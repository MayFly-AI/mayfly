project(bindings)

if(Python3_Development_FOUND)
	message(STATUS "Creating python bindings")
	include_directories(${CMAKE_SOURCE_DIR}/3rdparty)

	set(LIBRARIES core shared memory video_decode fec openh264 camera arducam_tof imu pcap_rxtx)
	if(MSVC)
		list(PREPEND LIBRARIES ws2_32.lib iphlpapi.lib)
	endif()

	# sensor module
	set(sensor_src
		mayfly/dicttopython.h
		mayfly/dicttopython.cpp
		mayfly/sensor_module.cpp
	)
	pybind11_add_module(sensor ${sensor_src})
	target_link_libraries(sensor PRIVATE ${LIBRARIES})

	# posesensor module
	if(MSVC)
		set(SERIAL_CONNECTION_SRC ${CMAKE_SOURCE_DIR}/posesensor/comserial.cpp)
	else()
		set(SERIAL_CONNECTION_SRC ${CMAKE_SOURCE_DIR}/posesensor/ttyserial.cpp)
	endif()
	set(posesensor_src
		mayfly/posesensor.cpp
		${SERIAL_CONNECTION_SRC}
		${CMAKE_SOURCE_DIR}/posesensor/usb.cpp
	)
	pybind11_add_module(posesensor ${posesensor_src})
	target_link_libraries(posesensor PRIVATE)

	# main
	set(native_test_src
		mayfly/dicttopython.h
		mayfly/dicttopython.cpp
		mayfly/main.cpp
	)
	add_executable(native_test ${native_test_src})
	target_link_libraries(native_test PRIVATE pybind11::embed pybind11::pybind11)
	target_link_libraries(native_test PRIVATE ${LIBRARIES})

else()
	message(STATUS "python3-dev not found. not building bindings")
endif()

