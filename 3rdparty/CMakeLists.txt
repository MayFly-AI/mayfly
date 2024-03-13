add_subdirectory(openh264)
add_subdirectory(plainh264)
add_subdirectory(fec)

if(Python3_Development_FOUND)
	add_subdirectory(pybind11)
else()
	message(STATUS "python3-dev not found. not building pybind11")
endif()
