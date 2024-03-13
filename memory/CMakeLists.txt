project(memory)

set(files
	cuda_alloc.h
	cuda_alloc.cpp
	tensor.cpp
	tensor.h
)

source_group("files" FILES ${files})
if(CUDA_FOUND)
	cuda_add_library(${PROJECT_NAME} ${files})
	target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR} ${CUDA_INCLUDE_DIRS})
	target_link_libraries(${PROJECT_NAME} ${CUDA_CUDA_LIBRARY})
else()
	add_library(${PROJECT_NAME} ${files})
	target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR})
endif()
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
