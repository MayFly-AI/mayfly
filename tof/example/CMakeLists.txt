project(arducam_tof_example)

find_package(OpenCV 4 QUIET)
if(OpenCV_FOUND)
	message(STATUS "OpenCV version ${OpenCV_VERSION}")
	message(STATUS "  inc: ${OpenCV_INCLUDE_DIRS}")
	message(STATUS "  lib: ${OpenCV_LIBS}")

	add_executable(${PROJECT_NAME} main.cpp)
	target_include_directories(${PROJECT_NAME} PRIVATE ..)
	target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} arducam_tof)
else()
	message(STATUS "${PROJECT_NAME} not functional - could not find OpenCV library!")
endif()
