project(datasource_status)

set(files
	datasourcestatus.cpp
)
if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_USE_MATH_DEFINES)
endif()
add_definitions(-DSTATIC_LINK_DATASOURCES)

source_group("files" FILES ${files})
add_library(${PROJECT_NAME} ${files})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME} shared)
