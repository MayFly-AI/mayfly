project(platform)

set(files
	gpio.cpp
	gpio.h
	spi.cpp
	spi.h
	i2c.cpp
	i2c.h
)

if(PLATFORM_RPI)
	source_group("files" FILES ${files})
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
	add_library(${PROJECT_NAME} ${files})
if(LIBGPIOD_FOUND)
	target_link_libraries(${PROJECT_NAME} ${LIBGPIOD_LINK_LIBRARIES})
endif()
	target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR})
	#set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
