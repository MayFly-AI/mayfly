project(fec C ASM)

set(files
	fecbuffer.h
	fecbuffer.cpp
	addmul.cpp
	fec.h
	fec.cpp
	tables.h
)
source_group("files" FILES ${files})

if(MSVC)
	#SET(CMAKE_CXX_FLAGS_DEBUG "/D_DEBUG /MTd /ZI /Ob2 /O2 /Gy")
	#target_compile_options(${PROJECT_NAME} PRIVATE "/O2")
	add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_USE_MATH_DEFINES)
endif()

add_library(${PROJECT_NAME} ${files})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
