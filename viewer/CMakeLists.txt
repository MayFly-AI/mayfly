project(viewer)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL)

include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rdparty ${CMAKE_SOURCE_DIR}/3rdparty/glew-2.1.0/include ${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking)
add_definitions(-DGLEW_STATIC -DGLEW_NO_GLU)

if(OpenGL_FOUND)
	#include_directories(${OPENGL_INCLUDE_DIRS})
	add_definitions(-DOPEN_GL)
	set(src_files_imgui
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui.h
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_demo.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_draw.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_tables.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_widgets.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/imgui_internal.h
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/backends/imgui_impl_glfw.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/backends/imgui_impl_glfw.h
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/backends/imgui_impl_opengl3.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/imgui-docking/backends/imgui_impl_opengl3.h
	)
	set(src_files_implot
		${CMAKE_SOURCE_DIR}/3rdparty/implot/implot.h
		${CMAKE_SOURCE_DIR}/3rdparty/implot/implot.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/implot/implot_internal.h
		${CMAKE_SOURCE_DIR}/3rdparty/implot/implot_items.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/implot/implot_demo.cpp
	)
endif()

set(FILES
	viewer.cpp
	${CMAKE_SOURCE_DIR}/3rdparty/glew-2.1.0/src/glew.c
)

set(LIBRARIES
	core fec pcap_rxtx camera arducam_tof imu
)

if(MSVC)
	include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rdparty/glfw/include)
	if(CUDA_FOUND)
		cuda_add_executable(${PROJECT_NAME} WIN32 ${FILES} ${src_files_imgui} ${FilesCuda})
		target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/3rdparty/nv_video_codec ${CMAKE_SOURCE_DIR}/3rdparty/nv_video_codec/NvCodec)
		target_link_libraries(${PROJECT_NAME} core OpenGL::GL ${NVENCODEAPI_LIB} ${CUVID_LIB} ${CUDA_CUDA_LIBRARY})
		set(FFMPEG_DIR ${CMAKE_SOURCE_DIR}/3rdparty/nv_video_codec/External/FFmpeg)
		Set(FFMPEG_DLL_DIR ${FFMPEG_DIR}/lib/x64/)
	else()
		add_executable(${PROJECT_NAME} WIN32 ${FILES} ${src_files_imgui})
	endif()
	target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/3rdparty/glfw/lib-vc2022/glfw3_mt.lib)
	#set_property(TARGET ${PROJECT_NAME} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
	target_link_libraries(${PROJECT_NAME} ws2_32.lib iphlpapi.lib ${LIBRARIES} OpenGL::GL)
	#target_compile_options(${PROJECT_NAME} PRIVATE "/Ob0 /Od /RTC1")
else()
	if(PLATFORM_RPI)
		find_package(glfw3 3.3 QUIET)
		if(glfw3_FOUND)
			#include(FindPkgConfig) # pkg_check_modules
			#pkg_check_modules(LIBCAMERA REQUIRED libcamera)
			add_executable( ${PROJECT_NAME} ${FILES} ${FILES_RPI} ${src_files_imgui})
			target_link_libraries(${PROJECT_NAME} glfw ${CMAKE_DL_LIBS} ${LIBCAMERA_LINK_LIBRARIES})
			target_link_libraries(${PROJECT_NAME} ${LIBRARIES} uwb OpenGL::GL -lrt ${LIBPCAP_LINK_LIBRARIES})
		else()
			message(STATUS "glfw3 not found. Not building viewer")
		endif()
	else()
		find_package(glfw3 3.3 REQUIRED)
		include_directories(${CMAKE_SOURCE_DIR})
		add_executable( ${PROJECT_NAME} ${FILES} ${src_files_imgui})
		target_link_libraries(${PROJECT_NAME} glfw ${CMAKE_DL_LIBS} ${LIBRARIES} OpenGL::GL)
		if(NOT APPLE)
			target_link_libraries(${PROJECT_NAME} ${LIBPCAP_LINK_LIBRARIES})
		endif()
		message(STATUS "build linux not pi or apple")
	endif()
endif()
