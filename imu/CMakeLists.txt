project(imu)

if(LIBGPIOD_FOUND)
add_definitions(-DENABLE_LIBGPIOD=1)

set(src_files
	imu.cpp
	bmi088.h
	bmi088.cpp
)

add_library(${PROJECT_NAME} ${src_files})
target_include_directories(${PROJECT_NAME} PUBLIC . ..)
target_link_libraries(${PROJECT_NAME} shared ${LIBGPIOD_LINK_LIBRARIES} platform)

add_subdirectory(example)

else()
message(STATUS "imu is not functional")

set(src_files
	imu.cpp
)
add_library(${PROJECT_NAME} ${src_files})
target_include_directories(${PROJECT_NAME} PUBLIC . ..)

endif()
