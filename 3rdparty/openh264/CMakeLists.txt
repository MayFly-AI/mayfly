project(openh264 C ASM)

IF(USE_ASM)
  IF(X86 OR X64)
	#message(STATUS "X86 OR X64")
    add_definitions(-DX86_ASM=1)
  endif()
  IF(UNIX)
    SET(CMAKE_ASM_NASM_FLAGS "${CMAKE_ASM_NASM_FLAGS} -DUNIX64")
  ENDIF()
  IF(AVX2)
	#message(STATUS "AVX2")
    add_definitions(-DHAVE_AVX2=1)
    SET(CMAKE_ASM_NASM_FLAGS "${CMAKE_ASM_NASM_FLAGS} -DHAVE_AVX2=1")
  ENDIF()
ENDIF()

set(files
#	lort.cpp
)

SET(COMMON_SRCDIR codec/common)
set(DECODER_SRCDIR codec/decoder)
SET(DECODER_API_H
  codec/api/svc/codec_api.h
  codec/api/svc/codec_app_def.h
  codec/api/svc/codec_def.h
  codec/api/svc/codec_ver.h
  )
SET(DECODER_CPP_SRCS
  ${DECODER_SRCDIR}/core/src/au_parser.cpp
  ${DECODER_SRCDIR}/core/src/bit_stream.cpp
  ${DECODER_SRCDIR}/core/src/cabac_decoder.cpp
  ${DECODER_SRCDIR}/core/src/deblocking.cpp
  ${DECODER_SRCDIR}/core/src/decode_mb_aux.cpp
  ${DECODER_SRCDIR}/core/src/decode_slice.cpp
  ${DECODER_SRCDIR}/core/src/decoder.cpp
  ${DECODER_SRCDIR}/core/src/decoder_core.cpp
  ${DECODER_SRCDIR}/core/src/decoder_data_tables.cpp
  ${DECODER_SRCDIR}/core/src/error_concealment.cpp
  ${DECODER_SRCDIR}/core/src/fmo.cpp
  ${DECODER_SRCDIR}/core/src/get_intra_predictor.cpp
  ${DECODER_SRCDIR}/core/src/manage_dec_ref.cpp
  ${DECODER_SRCDIR}/core/src/memmgr_nal_unit.cpp
  ${DECODER_SRCDIR}/core/src/mv_pred.cpp
  ${DECODER_SRCDIR}/core/src/parse_mb_syn_cabac.cpp
  ${DECODER_SRCDIR}/core/src/parse_mb_syn_cavlc.cpp
  ${DECODER_SRCDIR}/core/src/pic_queue.cpp
  ${DECODER_SRCDIR}/core/src/rec_mb.cpp
  ${DECODER_SRCDIR}/plus/src/welsDecoderExt.cpp
  ${DECODER_SRCDIR}/core/src/wels_decoder_thread.cpp)

SET(COMMON_CPP_SRCS
  ${COMMON_SRCDIR}/src/common_tables.cpp
  ${COMMON_SRCDIR}/src/copy_mb.cpp
  ${COMMON_SRCDIR}/src/cpu.cpp
  ${COMMON_SRCDIR}/src/crt_util_safe_x.cpp
  ${COMMON_SRCDIR}/src/deblocking_common.cpp
  ${COMMON_SRCDIR}/src/expand_pic.cpp
  ${COMMON_SRCDIR}/src/intra_pred_common.cpp
  ${COMMON_SRCDIR}/src/mc.cpp
  ${COMMON_SRCDIR}/src/memory_align.cpp
  ${COMMON_SRCDIR}/src/sad_common.cpp
  ${COMMON_SRCDIR}/src/utils.cpp
  ${COMMON_SRCDIR}/src/welsCodecTrace.cpp
  ${COMMON_SRCDIR}/src/WelsTaskThread.cpp
  ${COMMON_SRCDIR}/src/WelsThread.cpp
  ${COMMON_SRCDIR}/src/WelsThreadLib.cpp
  ${COMMON_SRCDIR}/src/WelsThreadPool.cpp
  )

IF(USE_ASM)
  IF(X86 OR X64)
    SET(DECODER_ASM_SRCS
      ${DECODER_SRCDIR}/core/x86/dct.asm
      ${DECODER_SRCDIR}/core/x86/intra_pred.asm)

    SET(COMMON_ASM_SRCS
      ${COMMON_SRCDIR}/x86/asm_inc.asm
      ${COMMON_SRCDIR}/x86/cpuid.asm
      ${COMMON_SRCDIR}/x86/dct_com.asm
      ${COMMON_SRCDIR}/x86/deblock.asm
      ${COMMON_SRCDIR}/x86/expand_picture.asm
      ${COMMON_SRCDIR}/x86/intra_pred_com.asm
      ${COMMON_SRCDIR}/x86/mb_copy.asm
      ${COMMON_SRCDIR}/x86/mc_chroma.asm
      ${COMMON_SRCDIR}/x86/mc_luma.asm
      ${COMMON_SRCDIR}/x86/satd_sad.asm
      ${COMMON_SRCDIR}/x86/vaa.asm)
    include_directories(codec/common/x86/)
  ENDIF()

  IF(ARM)
    add_definitions(-DHAVE_NEON=1)
	message(STATUS "ARM NEON")
    SET(DECODER_ASM_SRCS
      ${DECODER_SRCDIR}/core/arm/block_add_neon.S
      ${DECODER_SRCDIR}/core/arm/intra_pred_neon.S)
    SET(COMMON_ASM_SRCS
      ${COMMON_SRCDIR}/arm/copy_mb_neon.S
      ${COMMON_SRCDIR}/arm/deblocking_neon.S
      ${COMMON_SRCDIR}/arm/expand_picture_neon.S
      ${COMMON_SRCDIR}/arm/intra_pred_common_neon.S
      ${COMMON_SRCDIR}/arm/mc_neon.S)
    include_directories(codec/common/arm/)
  ENDIF()

  IF(ARM64)
    add_definitions(-DHAVE_NEON_AARCH64=1)
	message(STATUS "ARM NEON 64")
    SET(DECODER_ASM_SRCS
      ${DECODER_SRCDIR}/core/arm64/block_add_aarch64_neon.S
      ${DECODER_SRCDIR}/core/arm64/intra_pred_aarch64_neon.S)
    SET(COMMON_ASM_SRCS
      ${COMMON_SRCDIR}/arm64/copy_mb_aarch64_neon.S
      ${COMMON_SRCDIR}/arm64/deblocking_aarch64_neon.S
      ${COMMON_SRCDIR}/arm64/expand_picture_aarch64_neon.S
      ${COMMON_SRCDIR}/arm64/intra_pred_common_aarch64_neon.S
      ${COMMON_SRCDIR}/arm64/mc_aarch64_neon.S)
    include_directories(codec/common/arm64/)
  ENDIF()
ENDIF()

source_group("files" FILES ${files})

source_group("codec" FILES ${CODEC})
source_group("codec\\decoder" FILES ${DECODER_CPP_SRCS})
source_group("codec\\asm" FILES ${DECODER_ASM_SRCS})
source_group("common" FILES ${COMMON_CPP_SRCS})
source_group("common\\asm" FILES ${COMMON_ASM_SRCS})
source_group("codec\\api" FILES ${DECODER_API_H})
add_definitions(-DCODEC_FOR_TESTBED)
include_directories(. codec/decoder/core/inc codec/common/inc codec/api/svc codec/decoder/plus/inc)

#add_library(${PROJECT_NAME} STATIC ${DECODER_API_H} ${CODEC} ${DECODER_CPP_SRCS} ${DECODER_ASM_SRCS} ${COMMON_CPP_SRCS} ${COMMON_ASM_SRCS})

if(MSVC)
	#SET(CMAKE_CXX_FLAGS_DEBUG "/D_DEBUG /MTd /ZI /Ob2 /O2 /Gy")
	#target_compile_options(${PROJECT_NAME} PRIVATE "/O2")
endif()

add_definitions(-D_USE_MATH_DEFINES)

add_library(${PROJECT_NAME} ${files} ${DECODER_API_H} ${CODEC} ${DECODER_CPP_SRCS} ${DECODER_ASM_SRCS} ${COMMON_CPP_SRCS} ${COMMON_ASM_SRCS})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

set(CONSOLE_SRC
	codec/console/common/inc/read_config.h
	codec/console/common/src/read_config.cpp
	codec/console/dec/src/h264dec.cpp
	codec/console/dec/src/d3d9_utils.cpp
	codec/console/dec/inc/d3d9_utils.h
)
set(CONSOLE_INC
	codec/console/common/inc
	codec/console/dec/inc
)
add_executable("${PROJECT_NAME}_console" ${DECODER_API_H} ${CODEC} ${DECODER_CPP_SRCS} ${DECODER_ASM_SRCS} ${COMMON_CPP_SRCS} ${COMMON_ASM_SRCS} ${CONSOLE_SRC})
target_include_directories("${PROJECT_NAME}_console" PRIVATE ${CONSOLE_INC})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR})

if(UNIX)
  set_source_files_properties(
	  codec/decoder/core/src/parse_mb_syn_cavlc.cpp
	  codec/decoder/core/src/mv_pred.cpp
	  codec/decoder/core/src/parse_mb_syn_cabac.cpp
	  codec/decoder/core/src/decode_slice.cpp
	  PROPERTIES COMPILE_FLAGS -Wno-strict-aliasing
  )

  set_source_files_properties(
	  codec/decoder/core/src/decoder_core.cpp
	  PROPERTIES COMPILE_FLAGS -Wno-type-limits
  )
endif()
