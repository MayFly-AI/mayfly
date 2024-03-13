project(posesensor_test_serial)

if(MSVC)
	set(SERIAL_CONNECTION_SRC ${CMAKE_SOURCE_DIR}/posesensor/comserial.cpp)
else()
	set(SERIAL_CONNECTION_SRC ${CMAKE_SOURCE_DIR}/posesensor/ttyserial.cpp)
endif()

set(src_files
	main.cpp
	${SERIAL_CONNECTION_SRC}
	${CMAKE_SOURCE_DIR}/posesensor/usb.cpp
)


add_executable(${PROJECT_NAME} ${src_files})
include_directories(${CMAKE_SOURCE_DIR})


