project(datasource_ranging)

#ifdef PLATFORM_RPI
set(files
	datasourceranging.cpp
	ranging.h
	ranging.cpp
)
source_group("files" FILES ${files})
add_library(${PROJECT_NAME} ${files})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

if(PLATFORM_RPI)
	target_link_libraries(${PROJECT_NAME} shared uwb)
else()
	target_link_libraries(${PROJECT_NAME} shared)
endif()

