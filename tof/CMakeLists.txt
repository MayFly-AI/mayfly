project(arducam_tof)


set(src_files
	arducam_tof.cpp)

add_library(${PROJECT_NAME} ${src_files})
target_link_libraries(${PROJECT_NAME} PRIVATE shared)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

add_subdirectory(example)
