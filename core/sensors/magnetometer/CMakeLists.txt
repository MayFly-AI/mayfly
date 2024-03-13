project(datasource_mag)
set(files
	datasourcemag.cpp
)
source_group("files" FILES ${files})
add_library(${PROJECT_NAME} ${files})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME} shared magnetometer)

