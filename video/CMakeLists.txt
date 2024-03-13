project(video_decode)

set(files
	framedecoder.h
	framedecoder.cpp
	decoder.h
	decoder.cpp
	huffdecoder.h
	huffdecoder.cpp
	internal/decoder_openh264.cpp
	internal/decoder_plainh264.cpp
	internal/decoder_cuda.cpp
	internal/decoder_avcodec.cpp
	internal/decoder_null.cpp
)

source_group("files" FILES ${files})
add_definitions(-DSTATIC_LINK_DECODERS)
if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_USE_MATH_DEFINES)
endif()

if(CUDA_FOUND)
	set(NV_DEC_DIR ${CMAKE_SOURCE_DIR}/3rdparty/nv_video_codec/NvCodec/NvDecoder)
	set(FilesCuda
		${NV_DEC_DIR}/NvDecoder.cpp
		${CMAKE_SOURCE_DIR}/3rdparty/nv_video_codec/Utils/ColorSpace.cu
	)
	cuda_add_library(${PROJECT_NAME} ${files} ${FilesCuda})
	#add_library(${PROJECT_NAME} OBJECT ${files})
	#cuda_wrap_srcs(${PROJECT_NAME} OBJ ${FilesCuda})
	target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR} ${CUDA_INCLUDE_DIRS} ${NV_DEC_DIR} ${CMAKE_SOURCE_DIR}/3rdparty/nv_video_codec ${CMAKE_SOURCE_DIR}/3rdparty/nv_video_codec/NvCodec ${AVLIBS})
	target_link_libraries(${PROJECT_NAME} ${NVENCODEAPI_LIB} ${CUVID_LIB} ${CUDA_CUDA_LIBRARY} ${AVLIBS})
else()
	add_library(${PROJECT_NAME} ${files})
	target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR} ${AVLIBS})
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
if(FFMPEG_PATH)
	include_directories(BEFORE SYSTEM ${FFMPEG_PATH})
endif()

target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS} shared memory openh264 plainh264 ${AVLIBS})
