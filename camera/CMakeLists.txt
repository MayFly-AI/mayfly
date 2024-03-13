project(camera)
include(FindPkgConfig)
pkg_check_modules(LIBCAMERA libcamera QUIET)

if(LIBCAMERA_FOUND)

add_definitions(-DENABLE_LIBCAMERA=1)

message(STATUS "libcamera version ${LIBCAMERA_VERSION}")
message(STATUS "  inc: ${LIBCAMERA_INCLUDE_DIRS}")
message(STATUS "  lib: ${LIBCAMERA_LINK_LIBRARIES}")

add_compile_options(-Wall -Wextra -pedantic -Werror -Wfatal-errors -faligned-new)

set(src_files
	synccameraencoder.cpp
	h264_encoder.cpp
)

include_directories(${CMAKE_SOURCE_DIR} ${LIBCAMERA_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${src_files})
target_link_libraries(${PROJECT_NAME} PRIVATE shared ${LIBCAMERA_LINK_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

add_subdirectory(example)
else()

message(STATUS "camera is not functional")
add_library(${PROJECT_NAME} synccameraencoder.cpp)

endif()
