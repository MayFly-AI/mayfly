project(shared)

set(files
	types.h
	math.cpp
	math.h
	std_ext.h
	std_ext.cpp
	stb_ext.h
	stb_ext.cpp
	misc.cpp
	misc.h
	file.cpp
	file.h
	stb.cpp
	net.h
	net.cpp
	thread.h
	thread.cpp
	queue.h
	sha1.cpp
	sha1.h
	crc32.cpp
	crc32.h
	dict.cpp
	dict.h
	dctquant16.cpp
	dctquant16.h
	medianfilter.cpp
	medianfilter.h
)

source_group("files" FILES ${files})
#source_group("files/stb" FILES ${files_stb})

add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_USE_MATH_DEFINES)

add_library(${PROJECT_NAME} ${files})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR})

if(MSVC)
#	target_compile_options(${PROJECT_NAME} PRIVATE "/Ob0 /Od /RTC1")
endif()
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

add_subdirectory(example)
