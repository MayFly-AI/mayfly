project(core)
add_subdirectory(sensors)

set(files
	service.h
	app.cpp
	app.h
	sensorserver.cpp
	sensorserver.h
	sensorclient.cpp
	sensorclient.h
	httpserver.cpp
	httpserver.h
	datasource.cpp
	datasource.h
	packet.cpp
	packet.h
	errorcorrection.cpp
	errorcorrection.h
	transfer.cpp
	transfer.h
	history.h
)
if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_USE_MATH_DEFINES)
endif()
add_definitions(-DSTATIC_LINK_DATASOURCES)

source_group("files" FILES ${files})
add_library(${PROJECT_NAME} ${files})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME} shared memory ${ALL_DATASOURCE_LIBS})
