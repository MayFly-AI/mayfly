project(magnetometer)

if(LIBGPIOD_FOUND)
	add_definitions(-DENABLE_LIBGPIOD=1)
	set(src_files
		magnetometer.cpp
		bmm350/bmm350_oor.c
		bmm350/bmm350.c
		bmm350/common.c
		bmm350/coines.cpp
	)
	add_library(${PROJECT_NAME} ${src_files})
	target_include_directories(${PROJECT_NAME} PUBLIC . .. bmm350)
	target_link_libraries(${PROJECT_NAME} shared ${LIBGPIOD_LINK_LIBRARIES} platform)
	set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
	add_subdirectory(example)
else()
	message(STATUS "magnetometer is not functional")
	set(src_files
		magnetometer.cpp
	)
	add_library(${PROJECT_NAME} ${src_files})
	target_include_directories(${PROJECT_NAME} PUBLIC . ..)
endif()
