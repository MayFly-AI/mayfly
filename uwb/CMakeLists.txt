project(uwb)

if(PLATFORM_RPI)

	set(src_files
		dw_device.cpp
		platform_io.cpp
	)

	add_library(${PROJECT_NAME} ${src_files})

	target_compile_options(${PROJECT_NAME} PRIVATE -Wall -O2 -g)
	target_include_directories(${PROJECT_NAME} PUBLIC . ..)
	target_link_libraries(${PROJECT_NAME} shared platform)

	add_subdirectory(example)
endif()

