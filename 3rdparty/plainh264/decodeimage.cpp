/*!
 *@page License
 *
 * \copy
 *     Copyright (c)  2013, Cisco Systems
 *     All rights reserved.
 *
 *     Redistribution and use in source and binary forms, with or without
 *     modification, are permitted provided that the following conditions
 *     are met:
 *
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *
 *        * Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in
 *          the documentation and/or other materials provided with the
 *          distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *     COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *     POSSIBILITY OF SUCH DAMAGE.
 *
 *		ALL ORIGINAL CODE COPIED INTO THIS FILE, BY MAYFLY 2024
 */

#include "string.h"

#include "shared/misc.h"
#include "shared/math.h"
#include "decodeimage.h"

#define _PARSE_NALHRD_VCLHRD_PARAMS_

namespace NewDec{

#if defined(_MSC_VER)
#define ALIGNED_DECLARE( type, var, n ) __declspec(align(n)) type var
#elif defined(__GNUC__)
#define ALIGNED_DECLARE( type, var, n ) type var __attribute__((aligned(n)))
#endif//_MSC_VER

#define ALIGNBYTES (16)		// Worst case is requiring alignment to an 16 byte boundary

void WelsFree(void* pPointer){
#if defined(_MSC_VER)
	_aligned_free(pPointer);
#else
	free(pPointer);
#endif
}
inline void* WelsMemset(void* pPointer,int32_t iValue,uint32_t uiSize){
	return ::memset(pPointer,iValue,uiSize);
}
void* WelsMalloc(const uint32_t kuiSize,const uint32_t kiAlign){
#if defined(_MSC_VER)
	return _aligned_malloc(kuiSize,kiAlign);
#else
	// Assume kiAlign is power of two
	int size=(kuiSize+kiAlign-1)&(-(int)kiAlign);
	return aligned_alloc(kiAlign,size);
#endif
}
void* WelsMallocz(const uint32_t kuiSize){
	void* pPointer=WelsMalloc(kuiSize,16);
	::memset(pPointer,0,kuiSize);
	return pPointer;
}

// brief Feedback that whether or not have VCL NAL in current AU
typedef enum{
	FEEDBACK_NON_VCL_NAL=0,
	FEEDBACK_VCL_NAL,
	FEEDBACK_UNKNOWN_NAL
} FEEDBACK_VCL_NAL_IN_AU;

#define WELS_CHECK_SE_BOTH_WARNING(val,lower_bound,upper_bound,syntax_name) do {\
if ((val < lower_bound) || (val > upper_bound)) {\
 uprintf("invalid syntax " syntax_name " %d",val);\
}\
}while(0)

// define macros to check syntax elements
#define WELS_CHECK_SE_BOTH_ERROR(val,lower_bound,upper_bound,syntax_name,ret_code) do {\
if ((val < lower_bound) || (val > upper_bound)) {\
 FATAL("invalid syntax " syntax_name " %d",val);\
 return ret_code;\
}\
}while(0)

#define WELS_CHECK_SE_BOTH_ERROR_NOLOG(val,lower_bound,upper_bound,syntax_name,ret_code) do {\
if ((val < lower_bound) || (val > upper_bound)) {\
 return ret_code;\
}\
}while(0)

#define WELS_CHECK_SE_UPPER_ERROR_NOLOG(val,upper_bound,syntax_name,ret_code) do {\
if (val > upper_bound) {\
 return ret_code;\
}\
}while(0)

#define WELS_CHECK_SE_UPPER_ERROR(val,upper_bound,syntax_name,ret_code) do {\
if (val > upper_bound) {\
 FATAL("invalid syntax " syntax_name " %d",val);\
 return ret_code;\
}\
}while(0)

#define WELS_CHECK_SE_LOWER_WARNING(val,lower_bound,syntax_name) do {\
if (val < lower_bound) {\
 uprintf("invalid syntax " syntax_name " %d",val);\
}\
}while(0)

#define WELS_CHECK_SE_UPPER_WARNING(val,upper_bound,syntax_name) do {\
if (val > upper_bound) {\
 uprintf("invalid syntax " syntax_name " %d",val);\
}\
}while(0)

// below define syntax element offset
// for bit_depth_luma_minus8 and bit_depth_chroma_minus8
#define BIT_DEPTH_LUMA_OFFSET 8
#define BIT_DEPTH_CHROMA_OFFSET 8
// for log2_max_frame_num_minus4
#define LOG2_MAX_FRAME_NUM_OFFSET 4
// for log2_max_pic_order_cnt_lsb_minus4
#define LOG2_MAX_PIC_ORDER_CNT_LSB_OFFSET 4
// for pic_width_in_mbs_minus1
#define PIC_WIDTH_IN_MBS_OFFSET 1
// for pic_height_in_map_units_minus1
#define PIC_HEIGHT_IN_MAP_UNITS_OFFSET 1
// for bit_depth_aux_minus8
#define BIT_DEPTH_AUX_OFFSET 8
// for num_slice_groups_minus1
#define NUM_SLICE_GROUPS_OFFSET 1
// for run_length_minus1
#define RUN_LENGTH_OFFSET 1
// for slice_group_change_rate_minus1
#define SLICE_GROUP_CHANGE_RATE_OFFSET 1
// for pic_size_in_map_units_minus1
#define PIC_SIZE_IN_MAP_UNITS_OFFSET 1
// for num_ref_idx_l0_default_active_minus1 and num_ref_idx_l1_default_active_minus1
#define NUM_REF_IDX_L0_DEFAULT_ACTIVE_OFFSET 1
#define NUM_REF_IDX_L1_DEFAULT_ACTIVE_OFFSET 1
// for pic_init_qp_minus26 and pic_init_qs_minus26
#define PIC_INIT_QP_OFFSET 26
#define PIC_INIT_QS_OFFSET 26
// for num_ref_idx_l0_active_minus1 and num_ref_idx_l1_active_minus1
#define NUM_REF_IDX_L0_ACTIVE_OFFSET 1
#define NUM_REF_IDX_L1_ACTIVE_OFFSET 1

// From Level 5.2
#define MAX_MB_SIZE 36864
// for aspect_ratio_idc
#define EXTENDED_SAR 255

// Specified error format:
// ERR_NO=(ERR_LEVEL_FROM (HIGH WORD) << 16) | (ERR_INFO_FROM (LOW WORD))

#define GENERATE_ERROR_NO(iErrLevel,iErrInfo) ((iErrLevel << 16) | (iErrInfo & 0xFFFF))
#define ERR_INVALID_INTRA4X4_MODE -1

// ERR_LEVEL
// -----------------------------------------------------------------------------------------------------------
enum{
	ERR_LEVEL_ACCESS_UNIT=1,
	ERR_LEVEL_NAL_UNIT_HEADER,
	ERR_LEVEL_PREFIX_NAL,
	ERR_LEVEL_PARAM_SETS,
	ERR_LEVEL_SLICE_HEADER,
	ERR_LEVEL_SLICE_DATA,
	ERR_LEVEL_MB_DATA
};

// More detailed error information,maximal value is 65535
#define ERR_INFO_COMMON_BASE 1
#define ERR_INFO_SYNTAX_BASE 1001
#define ERR_INFO_LOGIC_BASE 10001
enum{
	// Error from common system level: 1-1000
	ERR_INFO_OUT_OF_MEMORY=ERR_INFO_COMMON_BASE,
	ERR_INFO_INVALID_ACCESS,
	ERR_INFO_INVALID_PTR,
	ERR_INFO_INVALID_PARAM,
	ERR_INFO_FILE_NO_FOUND,
	ERR_INFO_PATH_NO_FOUND,
	ERR_INFO_ACCESS_DENIED,
	ERR_INFO_NOT_READY,
	ERR_INFO_WRITE_FAULT,
	ERR_INFO_READ_FAULT,
	ERR_INFO_READ_OVERFLOW,
	ERR_INFO_READ_LEADING_ZERO,
	ERR_INFO_UNINIT,
	// Error from H.264 syntax elements parser: 1001-10000
	ERR_INFO_NO_PREFIX_CODE=ERR_INFO_SYNTAX_BASE,// No start prefix code indication
	ERR_INFO_NO_PARAM_SETS, 	// No SPS and/ PPS before sequence header
	ERR_INFO_PARAM_SETS_NOT_INTEGRATED, 	// Parameters sets (sps/pps) are not integrated at all before to decode VCL nal
	ERR_INFO_SPS_ID_OVERFLOW,
	ERR_INFO_PPS_ID_OVERFLOW,
	ERR_INFO_INVALID_PROFILE_IDC,
	ERR_INFO_UNMATCHED_LEVEL_IDC,
	ERR_INFO_INVALID_POC_TYPE,
	ERR_INFO_INVALID_MB_SIZE_INFO,
	ERR_INFO_REF_COUNT_OVERFLOW,
	ERR_INFO_CROPPING_NO_SUPPORTED,
	ERR_INFO_INVALID_CROPPING_DATA,
	ERR_INFO_UNSUPPORTED_VUI_HRD,
	ERR_INFO_INVALID_SLICEGROUP,
	ERR_INFO_INVALID_SLICEGROUP_MAP_TYPE,
	ERR_INFO_INVALID_FRAME_NUM,
	ERR_INFO_INVALID_IDR_PIC_ID,
	ERR_INFO_INVALID_REDUNDANT_PIC_CNT,
	ERR_INFO_INVALID_MAX_NUM_REF_FRAMES,
	ERR_INFO_INVALID_MAX_MB_SIZE,
	ERR_INFO_INVALID_FIRST_MB_IN_SLICE,
	ERR_INFO_INVALID_NUM_REF_IDX_L0_ACTIVE_MINUS1,
	ERR_INFO_INVALID_NUM_REF_IDX_L1_ACTIVE_MINUS1,
	ERR_INFO_INVALID_SLICE_ALPHA_C0_OFFSET_DIV2,
	ERR_INFO_INVALID_SLICE_BETA_OFFSET_DIV2,
	ERR_INFO_FMO_INIT_FAIL,
	ERR_INFO_SLICE_TYPE_OVERFLOW,
	ERR_INFO_INVALID_CABAC_INIT_IDC,
	ERR_INFO_INVALID_QP,
	ERR_INFO_INVALID_PIC_INIT_QS,
	ERR_INFO_INVALID_CHROMA_QP_INDEX_OFFSET,
	ERR_INFO_INVALID_PIC_INIT_QP,
	ERR_INFO_INVALID_LOG2_MAX_FRAME_NUM_MINUS4,
	ERR_INFO_INVALID_LOG2_MAX_PIC_ORDER_CNT_LSB_MINUS4,
	ERR_INFO_INVALID_NUM_REF_FRAME_IN_PIC_ORDER_CNT_CYCLE,
	ERR_INFO_INVALID_DBLOCKING_IDC,
	ERR_INFO_INVALID_MB_TYPE,
	ERR_INFO_INVALID_MB_SKIP_RUN,
	ERR_INFO_INVALID_SPS_ID,
	ERR_INFO_INVALID_PPS_ID,
	ERR_INFO_INVALID_SUB_MB_TYPE,
	ERR_INFO_UNAVAILABLE_TOP_BLOCK_FOR_INTRA,
	ERR_INFO_UNAVAILABLE_LEFT_BLOCK_FOR_INTRA,
	ERR_INFO_INVALID_REF_INDEX,
	ERR_INFO_INVALID_CBP,
	ERR_INFO_DQUANT_OUT_OF_RANGE,
	ERR_INFO_CAVLC_INVALID_PREFIX,
	ERR_INFO_CAVLC_INVALID_LEVEL,
	ERR_INFO_CAVLC_INVALID_TOTAL_COEFF_OR_TRAILING_ONES,
	ERR_INFO_CAVLC_INVALID_ZERO_LEFT,
	ERR_INFO_CAVLC_INVALID_RUN_BEFORE,
	ERR_INFO_MV_OUT_OF_RANGE,

	ERR_INFO_INVALID_I4x4_PRED_MODE,
	ERR_INFO_INVALID_I16x16_PRED_MODE,
	ERR_INFO_INVALID_I_CHROMA_PRED_MODE,

	ERR_INFO_INVALID_LUMA_LOG2_WEIGHT_DENOM,
	ERR_INFO_INVALID_CHROMA_LOG2_WEIGHT_DENOM,
	ERR_INFO_INVALID_LUMA_WEIGHT,
	ERR_INFO_INVALID_CHROMA_WEIGHT,
	ERR_INFO_INVALID_LUMA_OFFSET,
	ERR_INFO_INVALID_CHROMA_OFFSET,

	ERR_INFO_UNSUPPORTED_NON_BASELINE,
	ERR_INFO_UNSUPPORTED_FMOTYPE,
	ERR_INFO_UNSUPPORTED_MBAFF,
	ERR_INFO_UNSUPPORTED_ILP,
	ERR_INFO_UNSUPPORTED_CABAC_EL,
	ERR_INFO_UNSUPPORTED_SPSI,
	ERR_INFO_UNSUPPORTED_MGS,
	ERR_INFO_UNSUPPORTED_BIPRED,
	ERR_INFO_UNSUPPORTED_WP,
	ERR_INFO_UNSUPPORTED_SLICESKIP,

	ERR_INFO_FRAMES_LOST,
	ERR_INFO_DEPENDENCY_SPATIAL_LAYER_LOST,
	ERR_INFO_DEPENDENCY_QUALIT_LAYER_LOST,
	ERR_INFO_REFERENCE_PIC_LOST,
	ERR_INFO_INVALID_REORDERING,
	ERR_INFO_INVALID_MARKING,

	ERR_INFO_FMO_NOT_SUPPORTED_IN_BASE_LAYER,
	ERR_INFO_INVALID_ESS,
	ERR_INFO_INVALID_SLICE_TYPE,
	ERR_INFO_INVALID_REF_MARKING,
	ERR_INFO_INVALID_REF_REORDERING,

	// Error from corresponding logic,10001-65535
	ERR_INFO_NO_IDR_PIC=ERR_INFO_LOGIC_BASE,		// NO IDR picture available before sequence header
	ERR_INFO_EC_NO_NEIGHBOUR_MBS,
	ERR_INFO_EC_UNEXPECTED_MB_TYPE,
	ERR_INFO_EC_NO_ENOUGH_NEIGHBOUR_MBS,
	ERR_INFO_DUPLICATE_FRAME_NUM,
	// for LTR
	ERR_INFO_INVALID_MMCO_NUM,
	ERR_INFO_INVALID_MMCO_OPCODE_BASE,
	ERR_INFO_INVALID_MMCO_SHORT2UNUSED,
	EER_INFO_INVALID_MMCO_LONG2UNUSED,
	ERR_INFO_INVALID_MMCO_SHOART2LONG,
	ERR_INFO_INVALID_MMCO_REF_NUM_OVERFLOW,
	ERR_INFO_INVALID_MMCO_REF_NUM_NOT_ENOUGH,
	ERR_INFO_INVALID_MMCO_LONG_TERM_IDX_EXCEED_MAX,
	// for CABAC
	ERR_CABAC_NO_BS_TO_READ,
	ERR_CABAC_UNEXPECTED_VALUE,
	// for scaling list
	ERR_SCALING_LIST_DELTA_SCALE,
	// logic error related to multi-layer
	ERR_INFO_WIDTH_MISMATCH,
	// reconstruction error
	ERR_INFO_MB_RECON_FAIL,
	ERR_INFO_MB_NUM_EXCEED_FAIL,
	ERR_INFO_BS_INCOMPLETE,
	ERR_INFO_MB_NUM_INADEQUATE,
	// parse only error
	ERR_INFO_PARSEONLY_PENDING,
	ERR_INFO_PARSEONLY_ERROR,
};

#ifndef WELS_CEIL
#define WELS_CEIL(x) ceil(x)		// FIXME: low complexity instead of math library used
#endif// WELS_CEIL

#ifndef WELS_FLOOR
#define WELS_FLOOR(x) floor(x)		// FIXME: low complexity instead of math library used
#endif// WELS_FLOOR

#ifndef WELS_ROUND
#define WELS_ROUND(x) ((int32_t)(0.5+(x)))
#endif// WELS_ROUND

#ifndef WELS_ROUND64
#define WELS_ROUND64(x) ((int64_t)(0.5+(x)))
#endif// WELS_ROUND

#ifndef WELS_DIV_ROUND
#define WELS_DIV_ROUND(x,y) ((int32_t)((y)==0?((x)/((y)+1)):(((y)/2+(x))/(y))))
#endif// WELS_DIV_ROUND

#ifndef WELS_DIV_ROUND64
#define WELS_DIV_ROUND64(x,y) ((int64_t)((y)==0?((x)/((y)+1)):(((y)/2+(x))/(y))))
#endif// WELS_DIV_ROUND64

#ifndef WELS_ALIGN
#define WELS_ALIGN(x,n) (((x)+(n)-1)&~((n)-1))
#endif// WELS_ALIGN

#ifndef WELS_MAX
#define WELS_MAX(x,y) ((x) > (y) ? (x) : (y))
#endif// WELS_MAX

#ifndef WELS_MIN
#define WELS_MIN(x,y) ((x) < (y) ? (x) : (y))
#endif// WELS_MIN

#ifndef WELS_MIN_POSITIVE
#define WELS_MIN_POSITIVE(x,y) (x >=0 && y >=0) ? WELS_MIN(x,y) : WELS_MAX(x,y);
#endif// WELS_MIN_POSITIVE

#ifndef NEG_NUM
// #define NEG_NUM( num ) (-num)
#define NEG_NUM(iX) (1+(~(iX)))
#endif// NEG_NUM

#define WELS_NON_ZERO_COUNT_AVERAGE(nC,nA,nB) { \
 nC=nA+nB+1; \
 nC >>=(uint8_t)( nA !=-1 && nB !=-1); \
 nC+=(uint8_t)(nA==-1 && nB==-1); \
}

static inline uint8_t WelsClip1(int32_t iX){
	uint8_t uiTmp=(uint8_t)(((iX)&~255) ? (-(iX)>>31) : (iX));
	return uiTmp;
}

#ifndef WELS_SIGN
#define WELS_SIGN(iX) ((int32_t)(iX) >> 31)
#endif		// WELS_SIGN
#ifndef WELS_ABS
#if 1
#define WELS_ABS(iX) ((iX)>0 ? (iX) :-(iX))
#else
#define WELS_ABS(iX) ((WELS_SIGN(iX) ^ (int32_t)(iX))-WELS_SIGN(iX))
#endif
#endif		// WELS_ABS

// WELS_CLIP3
#ifndef WELS_CLIP3
#define WELS_CLIP3(iX,iY,iZ) ((iX) < (iY) ? (iY) : ((iX) > (iZ) ? (iZ) : (iX)))
#endif		// WELS_CLIP3


#define LD16(a) (*((uint16_t*)(a)))
#define LD32(a) (*((uint32_t*)(a)))
#define LD64(a) (*((uint64_t*)(a)))

#define ST16(a,b) *((uint16_t*)(a))=(b)
#define ST32(a,b) *((uint32_t*)(a))=(b)
#define ST64(a,b) *((uint64_t*)(a))=(b)
#define LD16A2 LD16
#define LD32A2 LD32
#define LD32A4 LD32
#define LD64A2 LD64
#define LD64A4 LD64
#define LD64A8 LD64
#define ST16A2 ST16
#define ST32A2 ST32
#define ST32A4 ST32
#define ST64A2 ST64
#define ST64A4 ST64
#define ST64A8 ST64

template<typename T> T WelsClip3(T iX,T iY,T iZ){
	if(iX<iY)
		return iY;
	if(iX>iZ)
		return iZ;
	return iX;
}
// ENFORCE_STACK_ALIGN_1D: force 1 dimension local data aligned in stack
// _tp: type
// _nm: var name
// _sz: size
// _al: align bytes
// auxiliary var: _nm ## _tEmP
#define ENFORCE_STACK_ALIGN_1D(_tp,_nm,_sz,_al) \
 _tp _nm ## _tEmP[(_sz)+(_al)-1]; \
 _tp *_nm=_nm ## _tEmP+((_al)-1)-(((uintptr_t)(_nm ## _tEmP+((_al)-1)) & ((_al)-1))/sizeof(_tp));


// Description: to check variable validation and return the specified result
// iResult: value to be checked
// iExpected: the expected value
#ifndef WELS_VERIFY_RETURN_IFNEQ
#define WELS_VERIFY_RETURN_IFNEQ(iResult,iExpected) \
 if (iResult !=iExpected) { \
 return iResult; \
 }
#endif// #if WELS_VERIFY_RETURN_IF

// Description: to check variable validation and return the specified result
// iResult: value to be return
// bCaseIf: negative condition to be verified
#ifndef WELS_VERIFY_RETURN_IF
#define WELS_VERIFY_RETURN_IF(iResult,bCaseIf) \
 if (bCaseIf) { \
 return iResult; \
 }
#endif// #if WELS_VERIFY_RETURN_IF

// Description: to check variable validation and return the specified result
// with correspoinding process advance.
// result: value to be return
// case_if: negative condition to be verified
// proc: process need perform
#ifndef WELS_VERIFY_RETURN_PROC_IF
#define WELS_VERIFY_RETURN_PROC_IF(iResult,bCaseIf,fProc) \
 if (bCaseIf) { \
 fProc; \
 return iResult; \
 }
#endif// #if WELS_VERIFY_RETURN_PROC_IF

#define MAX_TEMPORAL_LAYER_NUM 4
#define MAX_SPATIAL_LAYER_NUM 4
#define MAX_QUALITY_LAYER_NUM 4

#define MAX_LAYER_NUM_OF_FRAME 128
#define MAX_NAL_UNITS_IN_LAYER 128			// predetermined here,adjust it later if need

#define MAX_RTP_PAYLOAD_LEN 1000
#define AVERAGE_RTP_PAYLOAD_LEN 800

#define SAVED_NALUNIT_NUM_TMP ( (MAX_SPATIAL_LAYER_NUM*MAX_QUALITY_LAYER_NUM)+1+MAX_SPATIAL_LAYER_NUM )	// SPS/PPS+SEI/SSEI+PADDING_NAL
#define MAX_SLICES_NUM_TMP ( ( MAX_NAL_UNITS_IN_LAYER-SAVED_NALUNIT_NUM_TMP ) / 3 )

#define AUTO_REF_PIC_COUNT -1	// encoder selects the number of reference frame automatically
#define UNSPECIFIED_BIT_RATE 0	// to do: add detail comment

#define MAX_LOG_SIZE 1024
#define MAX_MBS_PER_FRAME 36864		// in accordance with max level support in Rec

typedef enum TagWelsErr{
	ERR_NONE=0,
	ERR_INVALID_PARAMETERS=1,
	ERR_MALLOC_FAILED=2,
	ERR_API_FAILED=3,
	ERR_BOUND=31
} EWelsErr;

struct SDataBuffer{
	uint8_t* pHead;
	uint8_t* pEnd;
	uint8_t* pStartPos;
	uint8_t* pCurPos;
};

// NAL Unit Type (5 Bits)
enum EWelsNalUnitType{
	NAL_UNIT_UNSPEC_0=0,
	NAL_UNIT_CODED_SLICE=1,
	NAL_UNIT_CODED_SLICE_DPA=2,
	NAL_UNIT_CODED_SLICE_DPB=3,
	NAL_UNIT_CODED_SLICE_DPC=4,
	NAL_UNIT_CODED_SLICE_IDR=5,
	NAL_UNIT_SEI=6,
	NAL_UNIT_SPS=7,
	NAL_UNIT_PPS=8,
	NAL_UNIT_AU_DELIMITER=9,
	NAL_UNIT_END_OF_SEQ=10,
	NAL_UNIT_END_OF_STR=11,
	NAL_UNIT_FILLER_DATA=12,
	NAL_UNIT_SPS_EXT=13,
	NAL_UNIT_PREFIX=14,
	NAL_UNIT_SUBSET_SPS=15,
	NAL_UNIT_DEPTH_PARAM=16,// NAL_UNIT_RESV_16
	NAL_UNIT_RESV_17=17,
	NAL_UNIT_RESV_18=18,
	NAL_UNIT_AUX_CODED_SLICE=19,
	NAL_UNIT_CODED_SLICE_EXT=20,
	NAL_UNIT_MVC_SLICE_EXT=21,// NAL_UNIT_RESV_21
	NAL_UNIT_RESV_22=22,
	NAL_UNIT_RESV_23=23,
	NAL_UNIT_UNSPEC_24=24,
	NAL_UNIT_UNSPEC_25=25,
	NAL_UNIT_UNSPEC_26=26,
	NAL_UNIT_UNSPEC_27=27,
	NAL_UNIT_UNSPEC_28=28,
	NAL_UNIT_UNSPEC_29=29,
	NAL_UNIT_UNSPEC_30=30,
	NAL_UNIT_UNSPEC_31=31
};

enum EWelsNalRefIdc{
	NRI_PRI_LOWEST=0,
	NRI_PRI_LOW=1,
	NRI_PRI_HIGH=2,
	NRI_PRI_HIGHEST=3
};

enum EVclType{
	NON_VCL=0,
	VCL=1,
	NOT_APP=2
};

const EVclType g_keTypeMap[32][2]={
	{NON_VCL,NON_VCL},		// 0: NAL_UNIT_UNSPEC_0
	{VCL,VCL,},				// 1: NAL_UNIT_CODED_SLICE
	{VCL,NOT_APP},			// 2: NAL_UNIT_CODED_SLICE_DPA
	{VCL,NOT_APP},			// 3: NAL_UNIT_CODED_SLICE_DPB
	{VCL,NOT_APP},			// 4: NAL_UNIT_CODED_SLICE_DPC
	{VCL,VCL},				// 5: NAL_UNIT_CODED_SLICE_IDR
	{NON_VCL,NON_VCL},		// 6: NAL_UNIT_SEI
	{NON_VCL,NON_VCL},		// 7: NAL_UNIT_SPS
	{NON_VCL,NON_VCL},		// 8: NAL_UNIT_PPS
	{NON_VCL,NON_VCL},		// 9: NAL_UNIT_AU_DELIMITER
	{NON_VCL,NON_VCL},		// 10: NAL_UNIT_END_OF_SEQ
	{NON_VCL,NON_VCL},		// 11: NAL_UNIT_END_OF_STR
	{NON_VCL,NON_VCL},		// 12: NAL_UNIT_FILLER_DATA
	{NON_VCL,NON_VCL},		// 13: NAL_UNIT_SPS_EXT
	{NON_VCL,NON_VCL},		// 14: NAL_UNIT_PREFIX,NEED associate succeeded NAL to make a VCL
	{NON_VCL,NON_VCL},		// 15: NAL_UNIT_SUBSET_SPS
	{NON_VCL,NON_VCL},		// 16: NAL_UNIT_DEPTH_PARAM
	{NON_VCL,NON_VCL},		// 17: NAL_UNIT_RESV_17
	{NON_VCL,NON_VCL},		// 18: NAL_UNIT_RESV_18
	{NON_VCL,NON_VCL},		// 19: NAL_UNIT_AUX_CODED_SLICE
	{NON_VCL,VCL},			// 20: NAL_UNIT_CODED_SLICE_EXT
	{NON_VCL,NON_VCL},		// 21: NAL_UNIT_MVC_SLICE_EXT
	{NON_VCL,NON_VCL},		// 22: NAL_UNIT_RESV_22
	{NON_VCL,NON_VCL},		// 23: NAL_UNIT_RESV_23
	{NON_VCL,NON_VCL},		// 24: NAL_UNIT_UNSPEC_24
	{NON_VCL,NON_VCL},		// 25: NAL_UNIT_UNSPEC_25
	{NON_VCL,NON_VCL},		// 26: NAL_UNIT_UNSPEC_26
	{NON_VCL,NON_VCL},		// 27: NAL_UNIT_UNSPEC_27
	{NON_VCL,NON_VCL},		// 28: NAL_UNIT_UNSPEC_28
	{NON_VCL,NON_VCL},		// 29: NAL_UNIT_UNSPEC_29
	{NON_VCL,NON_VCL},		// 30: NAL_UNIT_UNSPEC_30
	{NON_VCL,NON_VCL}		// 31: NAL_UNIT_UNSPEC_31
};

#define IS_VCL_NAL(t,ext_idx) (g_keTypeMap[t][ext_idx]==VCL)
#define IS_PARAM_SETS_NALS(t) ( (t)==NAL_UNIT_SPS || (t)==NAL_UNIT_PPS || (t)==NAL_UNIT_SUBSET_SPS )
#define IS_SPS_NAL(t) ( (t)==NAL_UNIT_SPS )
#define IS_SUBSET_SPS_NAL(t) ( (t)==NAL_UNIT_SUBSET_SPS )
#define IS_PPS_NAL(t) ( (t)==NAL_UNIT_PPS )
#define IS_SEI_NAL(t) ( (t)==NAL_UNIT_SEI )
#define IS_AU_DELIMITER_NAL(t) ( (t)==NAL_UNIT_AU_DELIMITER )
#define IS_PREFIX_NAL(t) ( (t)==NAL_UNIT_PREFIX )
#define IS_SUBSET_SPS_USED(t) ( (t)==NAL_UNIT_SUBSET_SPS || (t)==NAL_UNIT_CODED_SLICE_EXT )
#define IS_VCL_NAL_AVC_BASE(t) ( (t)==NAL_UNIT_CODED_SLICE || (t)==NAL_UNIT_CODED_SLICE_IDR )
#define IS_NEW_INTRODUCED_SVC_NAL(t) ( (t)==NAL_UNIT_PREFIX || (t)==NAL_UNIT_CODED_SLICE_EXT )

enum EWelsSliceType{
	P_SLICE=0,
	B_SLICE=1,
	I_SLICE=2,
	SP_SLICE=3,
	SI_SLICE=4,
	UNKNOWN_SLICE=5
};


// List Index
enum EListIndex{
	LIST_0=0,
	LIST_1=1,
	LIST_A=2
};

// Motion Vector components
enum EMvComp{
	MV_X=0,
	MV_Y=1,
	MV_A=2
};

// NAL Unix Header in AVC,refer to Page 56 in JVT X201wcm
struct SNalUnitHeader{
	uint8_t uiForbiddenZeroBit;
	uint8_t uiNalRefIdc;
	EWelsNalUnitType eNalUnitType;
	uint8_t uiReservedOneByte; 						// only padding usage
};

// brief Wels Flexible Macroblock Ordering (FMO)
struct SFmo{
	uint8_t* pMbAllocMap;
	int32_t iCountMbNum;
	int32_t iSliceGroupCount;
	int32_t iSliceGroupType;
	bool bActiveFlag;
	uint8_t uiReserved[3];							// reserved padding bytes
};

#define MB_SUB_PARTITION_SIZE 4						// Sub partition size in a 8x8 sub-block
#define NAL_UNIT_HEADER_EXT_SIZE 3					// Size of NAL unit header for extension in byte
#define MAX_PPS_COUNT 256							// Count number of PPS

#define MAX_REF_PIC_COUNT 16						// MAX Short+Long reference pictures
#define MIN_REF_PIC_COUNT 1							// minimal count number of reference pictures,1 short+2 key reference based?
#define MAX_SHORT_REF_COUNT 16						// maximal count number of short reference pictures
#define MAX_LONG_REF_COUNT 16						// maximal count number of long reference pictures
#define MAX_DPB_COUNT (MAX_REF_PIC_COUNT+1)			// 1 additional position for re-order and other process

#define MAX_MMCO_COUNT 66

#define MAX_SLICEGROUP_IDS 8						// Count number of Slice Groups

#define MAX_LAYER_NUM 8

#define LAYER_NUM_EXCHANGEABLE 1

#define MAX_NAL_UNIT_NUM_IN_AU 32					// predefined maximal number of NAL Units in an access unit
#define MIN_ACCESS_UNIT_CAPACITY 1048576			// Min AU capacity in bytes: (1<<20)=1024 KB predefined
#define MAX_BUFFERED_NUM 3							// mamixum stored number of AU|packet to prevent overwrite
#define MAX_ACCESS_UNIT_CAPACITY 7077888			// Maximum AU size in bytes for level 5.2 for single frame
#define MAX_ACCESS_UNIT_CAPACITY 7077888			// Maximum AU size in bytes for level 5.2 for single frame
#define MAX_MACROBLOCK_CAPACITY 5000				// Maximal legal MB capacity,15000 bits is enough

// Miscellaneous sizing infos
#ifndef MAX_FNAME_LEN
#define MAX_FNAME_LEN 256							// maximal length of file name in char size
#endif// MAX_FNAME_LEN

#ifndef WELS_LOG_BUF_SIZE
#define WELS_LOG_BUF_SIZE 4096
#endif// WELS_LOG_BUF_SIZE

// MB width in pixels for specified colorspace I420 usually used in codec
#define MB_WIDTH_LUMA 16
#define MB_WIDTH_CHROMA (MB_WIDTH_LUMA>>1)
// MB height in pixels for specified colorspace I420 usually used in codec
#define MB_HEIGHT_LUMA 16
#define MB_HEIGHT_CHROMA (MB_HEIGHT_LUMA>>1)
#define MB_COEFF_LIST_SIZE (256+((MB_WIDTH_CHROMA*MB_HEIGHT_CHROMA)<<1))
#define MB_PARTITION_SIZE 4							// Macroblock partition size in 8x8 sub-blocks
#define MB_BLOCK4x4_NUM 16
#define MB_BLOCK8x8_NUM 4
#define MAX_SPS_COUNT 32							// Count number of SPS
#define BASE_QUALITY_ID 0

struct SPicture{
	uint8_t* pBuffer[4];							// pointer to the first allocated byte,basical offset of buffer,dimension:
	uint8_t* pData[4];								// pointer to picture planes respectively
	int32_t iLinesize[4];							// linesize of picture planes respectively used currently
	int32_t iPlanes; 								// How many planes are introduced due to color space format?
	bool bIdrFlag;
	int32_t iWidthInPixel;							// picture width in pixel
	int32_t iHeightInPixel;							// picture height in pixel
	int32_t iFramePoc;								// frame POC

	bool bUsedAsRef;								// for ref pic management
	bool bIsLongRef;								// long term reference frame flag		// for ref pic management
	int8_t iRefCount;

	bool bIsComplete;								// indicate whether current picture is complete,not from EC
	uint8_t uiTemporalId;
	uint8_t uiSpatialId;
	uint8_t uiQualityId;

	int32_t iFrameNum;								// frame number, for ref pic management
	int32_t iFrameWrapNum;							// frame wrap number, for ref pic management
	int32_t iLongTermFrameIdx;						// id for long term ref pic
	uint32_t uiLongTermPicNum;						// long_term_pic_num

	int32_t iSpsId;									// against mosaic caused by cross-IDR interval reference.
	int32_t iPpsId;
	uint64_t uiTimeStamp;
	int32_t iPicBuffIdx;
	EWelsSliceType eSliceType;
	bool bIsUngroupedMultiSlice;					// multi-slice picture with each each slice group contains one slice.
	bool bNewSeqBegin;
	int32_t iMbEcedNum;
	int32_t iMbEcedPropNum;
	int32_t iMbNum;

	bool* pMbCorrectlyDecodedFlag;
	int8_t(*pNzc)[24];
	uint32_t* pMbType;								// mb type used for direct mode
	int16_t(*pMv[LIST_A])[MB_BLOCK4x4_NUM][MV_A];	// used for direct mode
	int8_t(*pRefIndex[LIST_A])[MB_BLOCK4x4_NUM];	// used for direct mode
	struct SPicture* pRefPic[LIST_A][17];			// ref pictures used for direct mode
};

struct SRefPic{
	SPicture* pRefList[LIST_A][MAX_DPB_COUNT];		// reference picture marking plus FIFO scheme
	SPicture* pShortRefList[LIST_A][MAX_DPB_COUNT];
	SPicture* pLongRefList[LIST_A][MAX_DPB_COUNT];
	uint8_t uiRefCount[LIST_A];
	uint8_t uiShortRefCount[LIST_A];
	uint8_t uiLongRefCount[LIST_A];					// dependend on ref pic module
	int32_t iMaxLongTermFrameIdx;
};

struct SVlcTable{
	const uint8_t(*kpCoeffTokenVlcTable[4][8])[2];
	const uint8_t(*kpChromaCoeffTokenVlcTable)[2];
	const uint8_t(*kpZeroTable[7])[2];
	const uint8_t(*kpTotalZerosTable[2][15])[2];
} ;

//#ifdef __LP64__
typedef int64_t intX_t;
//#else
//typedef int32_t intX_t;
//#endif

// Memory Management Control Operation (MMCO) code
enum EMmcoCode{
	MMCO_END=0,
	MMCO_SHORT2UNUSED=1,
	MMCO_LONG2UNUSED=2,
	MMCO_SHORT2LONG=3,
	MMCO_SET_MAX_LONG=4,
	MMCO_RESET=5,
	MMCO_LONG=6
};

enum EVuiVideoFormat{
	VUI_COMPONENT=0,
	VUI_PAL=1,
	VUI_NTSC=2,
	VUI_SECAM=3,
	VUI_MAC=4,
	VUI_UNSPECIFIED=5,
	VUI_RESERVED1=6,
	VUI_RESERVED2=7
};

// Bit-stream auxiliary reading / writing
struct SBitStringAux{
	uint8_t* pStartBuf;							// buffer to start position
	uint8_t* pEndBuf;							// buffer+length
	int32_t iBits;								// count bits of overall bitstreaming input
	intX_t iIndex;								// only for cavlc usage
	uint8_t* pCurBuf;							// current reading position
	uint32_t uiCurBits;
	int32_t iLeftBits;							// count number of available bits left ([1,8]),
	// need pointer to next byte start position in case 0 bit left then 8 instead
};

// Position Offset structure
struct SPosOffset{
	int32_t iLeftOffset;
	int32_t iTopOffset;
	int32_t iRightOffset;
	int32_t iBottomOffset;
};

typedef uint8_t ProfileIdc;

// VUI syntax in Sequence Parameter Set,refer to E.1 in Rec
struct SVui{
	bool bAspectRatioInfoPresentFlag;
	uint32_t uiAspectRatioIdc;
	uint32_t uiSarWidth;
	uint32_t uiSarHeight;
	bool bOverscanInfoPresentFlag;
	bool bOverscanAppropriateFlag;
	bool bVideoSignalTypePresentFlag;
	uint8_t uiVideoFormat;
	bool bVideoFullRangeFlag;
	bool bColourDescripPresentFlag;
	uint8_t uiColourPrimaries;
	uint8_t uiTransferCharacteristics;
	uint8_t uiMatrixCoeffs;
	bool bChromaLocInfoPresentFlag;
	uint32_t uiChromaSampleLocTypeTopField;
	uint32_t uiChromaSampleLocTypeBottomField;
	bool bTimingInfoPresentFlag;
	uint32_t uiNumUnitsInTick;
	uint32_t uiTimeScale;
	bool bFixedFrameRateFlag;
	bool bNalHrdParamPresentFlag;
	bool bVclHrdParamPresentFlag;
	bool bPicStructPresentFlag;
	bool bBitstreamRestrictionFlag;
	bool bMotionVectorsOverPicBoundariesFlag;
	uint32_t uiMaxBytesPerPicDenom;
	uint32_t uiMaxBitsPerMbDenom;
	uint32_t uiLog2MaxMvLengthHorizontal;
	uint32_t uiLog2MaxMvLengthVertical;
	uint32_t uiMaxNumReorderFrames;
	uint32_t uiMaxDecFrameBuffering;
};

// rief Enumerate the type of rate control mode
typedef enum{
	RC_QUALITY_MODE=0,							// quality mode
	RC_BITRATE_MODE=1,							// bitrate mode
	RC_BUFFERBASED_MODE=2,						// no bitrate control,only using buffer status,adjust the video quality
	RC_TIMESTAMP_MODE=3,						// rate control based timestamp
	RC_BITRATE_MODE_POST_SKIP=4,				// this is in-building RC MODE,WILL BE DELETED after algorithm tuning!
	RC_OFF_MODE=-1,								// rate control off mode
} RC_MODES;

// brief Enumerate the type of profile id
typedef enum{
	PRO_UNKNOWN=0,
	PRO_BASELINE=66,
	PRO_MAIN=77,
	PRO_EXTENDED=88,
	PRO_HIGH=100,
	PRO_HIGH10=110,
	PRO_HIGH422=122,
	PRO_HIGH444=144,
	PRO_CAVLC444=244,

	PRO_SCALABLE_BASELINE=83,
	PRO_SCALABLE_HIGH=86
} EProfileIdc;

// brief Enumerate the type of level id
typedef enum{
	LEVEL_UNKNOWN=0,
	LEVEL_1_0=10,
	LEVEL_1_B=9,
	LEVEL_1_1=11,
	LEVEL_1_2=12,
	LEVEL_1_3=13,
	LEVEL_2_0=20,
	LEVEL_2_1=21,
	LEVEL_2_2=22,
	LEVEL_3_0=30,
	LEVEL_3_1=31,
	LEVEL_3_2=32,
	LEVEL_4_0=40,
	LEVEL_4_1=41,
	LEVEL_4_2=42,
	LEVEL_5_0=50,
	LEVEL_5_1=51,
	LEVEL_5_2=52
} ELevelIdc;

#define CTX_NA 0
#define WELS_CONTEXT_COUNT 460
#define LEVEL_NUMBER 17

struct SLevelLimits{
	ELevelIdc uiLevelIdc;					// level idc
	uint32_t uiMaxMBPS;						// Max macroblock processing rate(MB/s)
	uint32_t uiMaxFS;						// Max frame sizea(MBs)
	uint32_t uiMaxDPBMbs;					// Max decoded picture buffer size(MBs)
	uint32_t uiMaxBR;						// Max video bit rate
	uint32_t uiMaxCPB;						// Max CPB size
	int16_t iMinVmv;						// Vertical MV component range upper bound
	int16_t iMaxVmv;						// Vertical MV component range lower bound
	uint16_t uiMinCR;						// Min compression ration
	int16_t iMaxMvsPer2Mb;					// Max number of motion vectors per two consecutive MBs
};

// Sequence Parameter Set,refer to Page 57 in JVT X201wcm
struct SSps{
	int32_t iSpsId;
	uint32_t iMbWidth;
	uint32_t iMbHeight;
	uint32_t uiTotalMbCount;					// used in decode_slice_data()

	uint32_t uiLog2MaxFrameNum;
	uint32_t uiPocType;
	// POC type 0
	int32_t iLog2MaxPocLsb;
	// POC type 1
	int32_t iOffsetForNonRefPic;

	int32_t iOffsetForTopToBottomField;
	int32_t iNumRefFramesInPocCycle;
	int8_t iOffsetForRefFrame[256];
	int32_t iNumRefFrames;

	SPosOffset sFrameCrop;

	ProfileIdc uiProfileIdc;
	uint8_t uiLevelIdc;
	uint8_t uiChromaFormatIdc;
	uint8_t uiChromaArrayType;

	uint8_t uiBitDepthLuma;
	uint8_t uiBitDepthChroma;

	bool bDeltaPicOrderAlwaysZeroFlag;
	bool bGapsInFrameNumValueAllowedFlag;

	bool bFrameMbsOnlyFlag;
	bool bMbaffFlag;							// MB Adapative Frame Field
	bool bDirect8x8InferenceFlag;
	bool bFrameCroppingFlag;

	bool bVuiParamPresentFlag;
	bool bConstraintSet0Flag;
	bool bConstraintSet1Flag;
	bool bConstraintSet2Flag;
	bool bConstraintSet3Flag;
	bool bSeparateColorPlaneFlag;
	bool bQpPrimeYZeroTransfBypassFlag;
	bool bSeqScalingMatrixPresentFlag;
	bool bSeqScalingListPresentFlag[12];
	// Add scaling list supporting
	uint8_t iScalingList4x4[6][16];
	uint8_t iScalingList8x8[6][64];
	SVui sVui;
	const SLevelLimits* pSLevelLimits;
};

// Sequence Parameter Set extension syntax,refer to Page 391 in JVT X201wcm
struct SSpsSvcExt{
	SPosOffset sSeqScaledRefLayer;
	uint8_t uiExtendedSpatialScalability;		// ESS
	uint8_t uiChromaPhaseXPlus1Flag;
	uint8_t uiChromaPhaseYPlus1;
	uint8_t uiSeqRefLayerChromaPhaseXPlus1Flag;
	uint8_t uiSeqRefLayerChromaPhaseYPlus1;
	bool bInterLayerDeblockingFilterCtrlPresentFlag;
	bool bSeqTCoeffLevelPredFlag;
	bool bAdaptiveTCoeffLevelPredFlag;
	bool bSliceHeaderRestrictionFlag;
};

// Subset sequence parameter set syntax,refer to Page 391 in JVT X201wcm
struct SSubsetSps{
	SSps sSps;
	SSpsSvcExt sSpsSvcExt;
	bool bSvcVuiParamPresentFlag;
	bool bAdditionalExtension2Flag;
	bool bAdditionalExtension2DataFlag;
};

// Picture parameter set syntax,refer to Page 59 in JVT X201wcm
struct SPps{
	int32_t iSpsId;
	int32_t iPpsId;

	uint32_t uiNumSliceGroups;
	uint32_t uiSliceGroupMapType;
	// slice_group_map_type=0
	uint32_t uiRunLength[MAX_SLICEGROUP_IDS];
	// slice_group_map_type=2
	uint32_t uiTopLeft[MAX_SLICEGROUP_IDS];
	uint32_t uiBottomRight[MAX_SLICEGROUP_IDS];
	// slice_group_map_type=3,4 or 5
	uint32_t uiSliceGroupChangeRate;
	// slice_group_map_type=6
	uint32_t uiPicSizeInMapUnits;
	uint32_t uiSliceGroupId[MAX_SLICEGROUP_IDS];

	uint32_t uiNumRefIdxL0Active;
	uint32_t uiNumRefIdxL1Active;

	int32_t iPicInitQp;
	int32_t iPicInitQs;
	int32_t iChromaQpIndexOffset[2];	// cb,cr

	bool bEntropyCodingModeFlag;
	bool bPicOrderPresentFlag;
	// slice_group_map_type=3,4 or 5
	bool bSliceGroupChangeDirectionFlag;
	bool bDeblockingFilterControlPresentFlag;

	bool bConstainedIntraPredFlag;
	bool bRedundantPicCntPresentFlag;
	bool bWeightedPredFlag;
	uint8_t uiWeightedBipredIdc;

	bool bTransform8x8ModeFlag;
	// Add for scalinglist support
	bool bPicScalingMatrixPresentFlag;
	bool bPicScalingListPresentFlag[12];
	uint8_t iScalingList4x4[6][16];
	uint8_t iScalingList8x8[6][64];

	int32_t iSecondChromaQPIndexOffset;		// second_chroma_qp_index_offset

};


// NAL Unit Header in scalable extension syntax,refer to Page 390 in JVT X201wcm
struct SNalUnitHeaderExt{
	SNalUnitHeader sNalUnitHeader;

	// uint8_t reserved_one_bit;
	bool bIdrFlag;
	uint8_t uiPriorityId;
	int8_t iNoInterLayerPredFlag;			// change as int8_t to support 3 values probably in encoder
	uint8_t uiDependencyId;

	uint8_t uiQualityId;
	uint8_t uiTemporalId;
	bool bUseRefBasePicFlag;
	bool bDiscardableFlag;

	bool bOutputFlag;
	uint8_t uiReservedThree2Bits;
	// Derived variable(s)
	uint8_t uiLayerDqId;
	bool bNalExtFlag;
};

// AVC MB types
#define MB_TYPE_INTRA4x4 0x00000001
#define MB_TYPE_INTRA16x16 0x00000002
#define MB_TYPE_INTRA8x8 0x00000004
#define MB_TYPE_16x16 0x00000008
#define MB_TYPE_16x8 0x00000010
#define MB_TYPE_8x16 0x00000020
#define MB_TYPE_8x8 0x00000040
#define MB_TYPE_8x8_REF0 0x00000080
#define MB_TYPE_SKIP 0x00000100
#define MB_TYPE_INTRA_PCM 0x00000200
#define MB_TYPE_INTRA_BL 0x00000400
#define MB_TYPE_DIRECT 0x00000800
#define MB_TYPE_P0L0 0x00001000
#define MB_TYPE_P1L0 0x00002000
#define MB_TYPE_P0L1 0x00004000
#define MB_TYPE_P1L1 0x00008000
#define MB_TYPE_L0 (MB_TYPE_P0L0 | MB_TYPE_P1L0)
#define MB_TYPE_L1 (MB_TYPE_P0L1 | MB_TYPE_P1L1)

#define SUB_MB_TYPE_8x8 0x00000001
#define SUB_MB_TYPE_8x4 0x00000002
#define SUB_MB_TYPE_4x8 0x00000004
#define SUB_MB_TYPE_4x4 0x00000008

#define MB_TYPE_INTRA (MB_TYPE_INTRA4x4 | MB_TYPE_INTRA16x16 | MB_TYPE_INTRA8x8 | MB_TYPE_INTRA_PCM)
#define MB_TYPE_INTER (MB_TYPE_16x16 | MB_TYPE_16x8 | MB_TYPE_8x16 | MB_TYPE_8x8 | MB_TYPE_8x8_REF0 | MB_TYPE_SKIP | MB_TYPE_DIRECT)
#define IS_INTRA4x4(type) ( MB_TYPE_INTRA4x4==(type) )
#define IS_INTRA8x8(type) ( MB_TYPE_INTRA8x8==(type) )
#define IS_INTRANxN(type) ( MB_TYPE_INTRA4x4==(type) || MB_TYPE_INTRA8x8==(type) )
#define IS_INTRA16x16(type) ( MB_TYPE_INTRA16x16==(type) )
#define IS_INTRA(type) ( (type)&MB_TYPE_INTRA )
#define IS_INTER(type) ( (type)&MB_TYPE_INTER )
#define IS_INTER_16x16(type) ( (type)&MB_TYPE_16x16 )
#define IS_INTER_16x8(type) ( (type)&MB_TYPE_16x8 )
#define IS_INTER_8x16(type) ( (type)&MB_TYPE_8x16 )
#define IS_TYPE_L0(type) ( (type)&MB_TYPE_L0 )
#define IS_TYPE_L1(type) ( (type)&MB_TYPE_L1 )
#define IS_DIR(a,part,list) ((a) & (MB_TYPE_P0L0<<((part)+2*(list))))


#define IS_SKIP(type) ( ((type)&MB_TYPE_SKIP) !=0 )
#define IS_DIRECT(type) ( ((type)&MB_TYPE_DIRECT) !=0 )
#define IS_SVC_INTER(type) IS_INTER(type)
#define IS_I_BL(type) ( (type)==MB_TYPE_INTRA_BL )
#define IS_SVC_INTRA(type) ( IS_I_BL(type) || IS_INTRA(type) )
#define IS_Inter_8x8(type) ( ((type)&MB_TYPE_8x8) !=0)
#define IS_SUB_8x8(sub_type) (((sub_type)&SUB_MB_TYPE_8x8) !=0)
#define IS_SUB_8x4(sub_type) (((sub_type)&SUB_MB_TYPE_8x4) !=0)
#define IS_SUB_4x8(sub_type) (((sub_type)&SUB_MB_TYPE_4x8) !=0)
#define IS_SUB_4x4(sub_type) (((sub_type)&SUB_MB_TYPE_4x4) !=0)

#define REF_NOT_AVAIL -2
#define REF_NOT_IN_LIST -1			// intra

// // // // /intra16x16 Luma
#define I16_PRED_INVALID -1
#define I16_PRED_V 0
#define I16_PRED_H 1
#define I16_PRED_DC 2
#define I16_PRED_P 3

#define I16_PRED_DC_L 4
#define I16_PRED_DC_T 5
#define I16_PRED_DC_128 6
#define I16_PRED_DC_A 7
// intra4x4 Luma
// Here,I8x8 also use these definitions
#define I4_PRED_INVALID 0
#define I4_PRED_V 0
#define I4_PRED_H 1
#define I4_PRED_DC 2
#define I4_PRED_DDL 3				// diagonal_down_left
#define I4_PRED_DDR 4				// diagonal_down_right
#define I4_PRED_VR 5				// vertical_right
#define I4_PRED_HD 6				// horizon_down
#define I4_PRED_VL 7				// vertical_left
#define I4_PRED_HU 8				// horizon_up

#define I4_PRED_DC_L 9
#define I4_PRED_DC_T 10
#define I4_PRED_DC_128 11

#define I4_PRED_DDL_TOP 12		// right-top replacing by padding rightmost pixel of top
#define I4_PRED_VL_TOP 13		// right-top replacing by padding rightmost pixel of top
#define I4_PRED_A 14

// intra Chroma
#define C_PRED_INVALID -1
#define C_PRED_DC 0
#define C_PRED_H 1
#define C_PRED_V 2
#define C_PRED_P 3

#define C_PRED_DC_L 4
#define C_PRED_DC_T 5
#define C_PRED_DC_128 6
#define C_PRED_A 7

// Reference picture list reordering syntax,refer to page 64 in JVT X201wcm
struct SRefPicListReorderSyn{
	struct{
		uint32_t uiAbsDiffPicNumMinus1;
		uint16_t uiLongTermPicNum;
		uint16_t uiReorderingOfPicNumsIdc;
	} sReorderingSyn[LIST_A][MAX_REF_PIC_COUNT];
	bool bRefPicListReorderingFlag[LIST_A];
};

// Decoded reference picture marking syntax,refer to Page 66 in JVT X201wcm
struct SRefPicMarking{
	struct{
		uint32_t uiMmcoType;
		int32_t iShortFrameNum;
		int32_t iDiffOfPicNum;
		uint32_t uiLongTermPicNum;
		int32_t iLongTermFrameIdx;
		int32_t iMaxLongTermFrameIdx;
	} sMmcoRef[MAX_MMCO_COUNT];
	bool bNoOutputOfPriorPicsFlag;
	bool bLongTermRefFlag;
	bool bAdaptiveRefPicMarkingModeFlag;
};

// Decode reference base picture marking syntax in Page 396 of JVT X201wcm
typedef struct TagRefBasePicMarkingSyn{
	struct{
		uint32_t uiMmcoType;
		int32_t iShortFrameNum;
		uint32_t uiDiffOfPicNums;
		uint32_t uiLongTermPicNum;		// should uint32_t,cover larger range of iFrameNum.
	} mmco_base[MAX_MMCO_COUNT];		// MAX_REF_PIC for reference picture based on frame
	bool bAdaptiveRefBasePicMarkingModeFlag;
} SRefBasePicMarking,* PRefBasePicMarking;


// Prefix NAL Unix syntax,refer to Page 392 in JVT X201wcm
typedef struct TagPrefixNalUnit{
	SRefBasePicMarking sRefPicBaseMarking;
	bool bStoreRefBasePicFlag;
	bool bPrefixNalUnitAdditionalExtFlag;
	bool bPrefixNalUnitExtFlag;
	bool bPrefixNalCorrectFlag;
} SPrefixNalUnit,* PPrefixNalUnit;

// Prediction weight table syntax,refer to page 65 in JVT X201wcm
typedef struct TagPredWeightTabSyntax{
	uint32_t uiLumaLog2WeightDenom;
	uint32_t uiChromaLog2WeightDenom;
	struct{
		int32_t iLumaWeight[MAX_REF_PIC_COUNT];
		int32_t iLumaOffset[MAX_REF_PIC_COUNT];
		int32_t iChromaWeight[MAX_REF_PIC_COUNT][2];
		int32_t iChromaOffset[MAX_REF_PIC_COUNT][2];
		bool bLumaWeightFlag;
		bool bChromaWeightFlag;
	} sPredList[LIST_A];
	int32_t iImplicitWeight[MAX_REF_PIC_COUNT][MAX_REF_PIC_COUNT];
} SPredWeightTabSyn,* PPredWeightTabSyn;

// Header of slice syntax elements,refer to Page 63 in JVT X201wcm
struct SSliceHeader{
	int32_t iFirstMbInSlice;
	int32_t iFrameNum;
	int32_t iPicOrderCntLsb;
	int32_t iDeltaPicOrderCntBottom;
	int32_t iDeltaPicOrderCnt[2];
	int32_t iRedundantPicCnt;
	int32_t iDirectSpatialMvPredFlag;		// !< Direct Mode type to be used (0: Temporal,1: Spatial)
	int32_t uiRefCount[LIST_A];
	int32_t iSliceQpDelta;					// no use for iSliceQp is used directly
	int32_t iSliceQp;
	int32_t iSliceQsDelta;					// For SP/SI slices
	uint32_t uiDisableDeblockingFilterIdc;
	int32_t iSliceAlphaC0Offset;
	int32_t iSliceBetaOffset;
	int32_t iSliceGroupChangeCycle;

	SSps* pSps;
	SPps* pPps;
	int32_t iSpsId;
	int32_t iPpsId;
	bool bIdrFlag;

	SRefPicListReorderSyn pRefPicListReordering;		// Reference picture list reordering syntaxs
	SPredWeightTabSyn sPredWeightTable;
	int32_t iCabacInitIdc;
	int32_t iMbWidth;		// from?
	int32_t iMbHeight;		// from?
	SRefPicMarking sRefMarking;		// Decoded reference picture marking syntaxs

	uint16_t uiIdrPicId;
	EWelsSliceType eSliceType;
	bool bNumRefIdxActiveOverrideFlag;
	bool bFieldPicFlag;		// not supported in base profile
	bool bBottomFiledFlag;	// not supported in base profile
	uint8_t uiPadding1Byte;
	bool bSpForSwitchFlag; 	// For SP/SI slices
	int16_t iPadding2Bytes;
};

// Slice header in scalable extension syntax,refer to Page 394 in JVT X201wcm
struct SSliceHeaderExt{
	SSliceHeader sSliceHeader;
	SSubsetSps* pSubsetSps;

	uint32_t uiDisableInterLayerDeblockingFilterIdc;
	int32_t iInterLayerSliceAlphaC0Offset;
	int32_t iInterLayerSliceBetaOffset;

	// SPosOffset sScaledRefLayer;
	int32_t iScaledRefLayerPicWidthInSampleLuma;
	int32_t iScaledRefLayerPicHeightInSampleLuma;

	SRefBasePicMarking sRefBasePicMarking;
	bool bBasePredWeightTableFlag;
	bool bStoreRefBasePicFlag;
	bool bConstrainedIntraResamplingFlag;
	bool bSliceSkipFlag;

	bool bAdaptiveBaseModeFlag;
	bool bDefaultBaseModeFlag;
	bool bAdaptiveMotionPredFlag;
	bool bDefaultMotionPredFlag;
	bool bAdaptiveResidualPredFlag;
	bool bDefaultResidualPredFlag;
	bool bTCoeffLevelPredFlag;
	uint8_t uiRefLayerChromaPhaseXPlus1Flag;

	uint8_t uiRefLayerChromaPhaseYPlus1;
	uint8_t uiRefLayerDqId;
	uint8_t uiScanIdxStart;
	uint8_t uiScanIdxEnd;
};

struct SSlice{
	SSliceHeaderExt sSliceHeaderExt;
	// for Macroblock coding within slice
	int32_t iLastMbQp;								// stored qp for last mb coded,maybe more efficient for mb skip detection etc.
	// slice_data_ext()
	int32_t iMbSkipRun;
	int32_t iTotalMbInCurSlice;						// record the total number of MB in current slice.
	// slice_data_ext() generate
	bool bSliceHeaderExtFlag;						// Indicate which slice header is used,avc or ext?
	// from lower layer: slice header
	uint8_t eSliceType;
	uint8_t uiPadding[2];
	int32_t iLastDeltaQp;
	int16_t iMvScale[LIST_A][MAX_DPB_COUNT];		// Moton vector scale For Temporal Direct Mode Type
};

// NAL Unit Structure
struct SNalUnit{
	SNalUnitHeaderExt sNalHeaderExt;
	union{
		struct SVclNal{
			SSliceHeaderExt sSliceHeaderExt;
			SBitStringAux sSliceBitsRead;
			uint8_t* pNalPos;						// save the address of slice nal for GPU function
			int32_t iNalLength;						// save the nal length for GPU function
			bool bSliceHeaderExtFlag;
		} sVclNal;
		SPrefixNalUnit sPrefixNal;
	} sNalData;
	uint64_t uiTimeStamp;
};

// Access Unit structure
struct SAccessUnit{
	SNalUnit** pNalUnitsList;						// list of NAL Units pointer in this AU
	uint32_t uiAvailUnitsNum;						// Number of NAL Units available in each AU list based current bitstream,
	uint32_t uiActualUnitsNum;						// actual number of NAL units belong to current au
	// While available number exceeds count size below,need realloc extra NAL Units for list space.
	uint32_t uiCountUnitsNum;						// Count size number of malloced NAL Units in each AU list
	uint32_t uiStartPos;
	uint32_t uiEndPos;
	bool bCompletedAuFlag;							// Indicate whether it is a completed AU
};

enum{
	OVERWRITE_NONE=0,
	OVERWRITE_PPS=1,
	OVERWRITE_SPS=1<<1,
	OVERWRITE_SUBSETSPS=1<<2
};

// Decoder SPS and PPS global CTX
struct SWelsDecoderSpsPpsCTX{
	SPosOffset sFrameCrop;

	SSps sSpsBuffer[MAX_SPS_COUNT+1];
	SPps sPpsBuffer[MAX_PPS_COUNT+1];

	SSubsetSps sSubsetSpsBuffer[MAX_SPS_COUNT+1];
	SNalUnit sPrefixNal;

	SSps* pActiveLayerSps[MAX_LAYER_NUM];
	bool bAvcBasedFlag;							// For decoding bitstream:

	// for EC parameter sets
	bool bSpsExistAheadFlag;					// whether does SPS NAL exist ahead of sequence?
	bool bSubspsExistAheadFlag;					// whether does Subset SPS NAL exist ahead of sequence?
	bool bPpsExistAheadFlag;					// whether does PPS NAL exist ahead of sequence?

	int32_t iSpsErrorIgnored;
	int32_t iSubSpsErrorIgnored;
	int32_t iPpsErrorIgnored;

	bool bSpsAvailFlags[MAX_SPS_COUNT];
	bool bSubspsAvailFlags[MAX_SPS_COUNT];
	bool bPpsAvailFlags[MAX_PPS_COUNT];
	int32_t iPPSLastInvalidId;
	int32_t iPPSInvalidNum;
	int32_t iSPSLastInvalidId;
	int32_t iSPSInvalidNum;
	int32_t iSubSPSLastInvalidId;
	int32_t iSubSPSInvalidNum;
	int32_t iSeqId;								// sequence id
	int iOverwriteFlags;
};

struct SPictInfo{
	SBufferInfo sBufferInfo;
	int32_t iPOC;
	int32_t iPicBuffIdx;
	bool bLastGOP;
};

struct SPictReoderingStatus{
	int32_t iPictInfoIndex;
	int32_t iMinPOC;
	int32_t iNumOfPicts;
	int32_t iLastGOPRemainPicts;
	int32_t iLastWrittenPOC;
	int32_t iLargestBufferedPicIndex;
};

#define PICTURE_RESOLUTION_ALIGNMENT 32

struct SPicBuff{
	SPicture** ppPic;
	int32_t iCapacity;							// capacity size of queue
	int32_t iCurrentIdx;
};

// MB Type & Sub-MB Type
typedef uint32_t MbType;
typedef uint32_t SubMbType;

#define I16_LUMA_DC 1
#define I16_LUMA_AC 2
#define LUMA_DC_AC 3
#define CHROMA_DC 4
#define CHROMA_AC 5
#define LUMA_DC_AC_8 6
#define CHROMA_DC_U 7
#define CHROMA_DC_V 8
#define CHROMA_AC_U 9
#define CHROMA_AC_V 10
#define LUMA_DC_AC_INTRA 11
#define LUMA_DC_AC_INTER 12
#define CHROMA_DC_U_INTER 13
#define CHROMA_DC_V_INTER 14
#define CHROMA_AC_U_INTER 15
#define CHROMA_AC_V_INTER 16
#define LUMA_DC_AC_INTRA_8 17
#define LUMA_DC_AC_INTER_8 18

#define SHIFT_BUFFER(pBitsCache) { pBitsCache->pBuf+=2; pBitsCache->uiRemainBits+=16; pBitsCache->uiCache32Bit |=(((pBitsCache->pBuf[2] << 8) | pBitsCache->pBuf[3]) << (32-pBitsCache->uiRemainBits)); }
#define POP_BUFFER(pBitsCache,iCount) { pBitsCache->uiCache32Bit <<=iCount; pBitsCache->uiRemainBits-=iCount; }

//DQ Layer level
typedef struct TagDqLayer SDqLayer;
typedef SDqLayer* PDqLayer;
struct SLayerInfo{
	SNalUnitHeaderExt sNalHeaderExt;
	SSlice sSliceInLayer;					// Here Slice identify to Frame on concept
	SSubsetSps* pSubsetSps;					// current pSubsetSps used,memory alloc in external
	SSps* pSps;								// current sps based avc used,memory alloc in external
	SPps* pPps;								// current pps used
};
// Layer Representation

struct TagDqLayer{
	SLayerInfo sLayerInfo;
	SBitStringAux* pBitStringAux;			// pointer to SBitStringAux
	SFmo* pFmo;								// Current fmo context pointer used
	uint32_t* pMbType;
	int32_t* pSliceIdc;						// using int32_t for slice_idc
	int16_t(*pMv[LIST_A])[MB_BLOCK4x4_NUM][MV_A];
	int16_t(*pMvd[LIST_A])[MB_BLOCK4x4_NUM][MV_A];
	int8_t(*pRefIndex[LIST_A])[MB_BLOCK4x4_NUM];
	int8_t(*pDirect)[MB_BLOCK4x4_NUM];
	bool* pNoSubMbPartSizeLessThan8x8Flag;
	bool* pTransformSize8x8Flag;
	int8_t* pLumaQp;
	int8_t(*pChromaQp)[2];
	int8_t* pCbp;
	uint16_t* pCbfDc;
	int8_t(*pNzc)[24];
	int8_t(*pNzcRs)[24];
	int8_t* pResidualPredFlag;
	int8_t* pInterPredictionDoneFlag;
	bool* pMbCorrectlyDecodedFlag;
	bool* pMbRefConcealedFlag;
	int16_t(*pScaledTCoeff)[MB_COEFF_LIST_SIZE];
	int8_t(*pIntraPredMode)[8];	// 0~3 top4x4 ; 4~6 left 4x4; 7 intra16x16
	int8_t(*pIntra4x4FinalMode)[MB_BLOCK4x4_NUM];
	uint8_t* pIntraNxNAvailFlag;
	int8_t* pChromaPredMode;
	uint32_t(*pSubMbType)[MB_SUB_PARTITION_SIZE];
	int32_t iLumaStride;
	int32_t iChromaStride;
	uint8_t* pPred[3];
	int32_t iMbX;
	int32_t iMbY;
	int32_t iMbXyIndex;
	int32_t iMbWidth;						// MB width of this picture,equal to sSps.iMbWidth
	int32_t iMbHeight;						// MB height of this picture,equal to sSps.iMbHeight;

	// Common syntax elements across all slices of a DQLayer
	int32_t iSliceIdcBackup;
	uint32_t uiSpsId;
	uint32_t uiPpsId;
	uint32_t uiDisableInterLayerDeblockingFilterIdc;
	int32_t iInterLayerSliceAlphaC0Offset;
	int32_t iInterLayerSliceBetaOffset;
	int32_t iSliceGroupChangeCycle;

	SRefPicListReorderSyn* pRefPicListReordering;
	PPredWeightTabSyn pPredWeightTable;
	SRefPicMarking* pRefPicMarking;			// Decoded reference picture marking syntaxs
	PRefBasePicMarking pRefPicBaseMarking;

	SPicture* pRef;							// reference picture pointer
	SPicture* pDec;							// reconstruction picture pointer for layer

	int16_t iColocMv[2][16][2];				// Colocated MV cache
	int8_t iColocRefIndex[2][16];			// Colocated RefIndex cache
	int8_t iColocIntra[16];					// Colocated Intra cache

	bool bUseWeightPredictionFlag;
	bool bUseWeightedBiPredIdc;
	bool bStoreRefBasePicFlag;				// iCurTid==0 && iCurQid=0 && bEncodeKeyPic=1
	bool bTCoeffLevelPredFlag;
	bool bConstrainedIntraResamplingFlag;
	uint8_t uiRefLayerDqId;
	uint8_t uiRefLayerChromaPhaseXPlus1Flag;
	uint8_t uiRefLayerChromaPhaseYPlus1;
	uint8_t uiLayerDqId;					// dq_id of current layer
	bool bUseRefBasePicFlag;				// whether reference pic or reference base pic is referred?
};



typedef void (*PGetIntraPredFunc) (uint8_t* pPred,const int32_t kiLumaStride);
typedef void (*PIdctResAddPredFunc) (uint8_t* pPred,const int32_t kiStride,int16_t* pRs);
typedef void (*PIdctFourResAddPredFunc) (uint8_t* pPred,int32_t iStride,int16_t* pRs,const int8_t* pNzc);
typedef void (*PGetIntraPred8x8Func) (uint8_t* pPred,const int32_t kiLumaStride,bool bTLAvail,bool bTRAvail);
typedef void (*PWelsMcFunc) (const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight);
typedef void (*PWelsLumaHalfpelMcFunc) (const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight);
typedef void (*PWelsSampleAveragingFunc) (uint8_t*,int32_t,const uint8_t*,int32_t,const uint8_t*,int32_t,int32_t,int32_t);

typedef struct TagMcFunc{
	PWelsLumaHalfpelMcFunc pfLumaHalfpelHor;
	PWelsLumaHalfpelMcFunc pfLumaHalfpelVer;
	PWelsLumaHalfpelMcFunc pfLumaHalfpelCen;
	PWelsMcFunc pMcChromaFunc;
	PWelsMcFunc pMcLumaFunc;
	PWelsSampleAveragingFunc pfSampleAveraging;
} SMcFunc;

typedef void (*PCopyFunc) (uint8_t* pDst,int32_t iStrideD,uint8_t* pSrc,int32_t iStrideS);
typedef struct TagCopyFunc{
	PCopyFunc pCopyLumaFunc;
	PCopyFunc pCopyChromaFunc;
} SCopyFunc;

struct SDeblockingFilter{
	uint8_t* pCsData[3];						// pointer to reconstructed picture data
	int32_t iCsStride[2];						// Cs stride
	EWelsSliceType eSliceType;
	int8_t iSliceAlphaC0Offset;
	int8_t iSliceBetaOffset;
	int8_t iChromaQP[2];
	int8_t iLumaQP;
	struct SDeblockingFunc* pLoopf;
	SPicture** pRefPics[LIST_A];
};

typedef void (*PDeblockingFilterMbFunc) (PDqLayer pCurDqLayer,SDeblockingFilter* filter,int32_t boundry_flag);
typedef void (*PLumaDeblockingLT4Func) (uint8_t* iSampleY,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* iTc);
typedef void (*PLumaDeblockingEQ4Func) (uint8_t* iSampleY,int32_t iStride,int32_t iAlpha,int32_t iBeta);
typedef void (*PChromaDeblockingLT4Func) (uint8_t* iSampleCb,uint8_t* iSampleCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* iTc);
typedef void (*PChromaDeblockingEQ4Func) (uint8_t* iSampleCb,uint8_t* iSampleCr,int32_t iStride,int32_t iAlpha,int32_t iBeta);
typedef void (*PChromaDeblockingLT4Func2) (uint8_t* iSampleCbr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* iTc);
typedef void (*PChromaDeblockingEQ4Func2) (uint8_t* iSampleCbr,int32_t iStride,int32_t iAlpha,int32_t iBeta);

struct SDeblockingFunc{
	PLumaDeblockingLT4Func pfLumaDeblockingLT4Ver;
	PLumaDeblockingEQ4Func pfLumaDeblockingEQ4Ver;
	PLumaDeblockingLT4Func pfLumaDeblockingLT4Hor;
	PLumaDeblockingEQ4Func pfLumaDeblockingEQ4Hor;

	PChromaDeblockingLT4Func pfChromaDeblockingLT4Ver;
	PChromaDeblockingEQ4Func pfChromaDeblockingEQ4Ver;
	PChromaDeblockingLT4Func pfChromaDeblockingLT4Hor;
	PChromaDeblockingEQ4Func pfChromaDeblockingEQ4Hor;

	PChromaDeblockingLT4Func2 pfChromaDeblockingLT4Ver2;
	PChromaDeblockingEQ4Func2 pfChromaDeblockingEQ4Ver2;
	PChromaDeblockingLT4Func2 pfChromaDeblockingLT4Hor2;
	PChromaDeblockingEQ4Func2 pfChromaDeblockingEQ4Hor2;
};

typedef void (*PExpandPictureFunc) (uint8_t* pDst,const int32_t kiStride,const int32_t kiPicW,const int32_t kiPicH);

struct SExpandPicFunc{
	PExpandPictureFunc pfExpandLumaPicture;
	PExpandPictureFunc pfExpandChromaPicture[2];
};

typedef void (*PWelsNonZeroCountFunc) (int8_t* pNonZeroCount);
typedef void (*PWelsBlockZeroFunc) (int16_t* block,int32_t stride);

struct SBlockFunc{
	PWelsNonZeroCountFunc pWelsSetNonZeroCountFunc;
	PWelsBlockZeroFunc pWelsBlockZero16x16Func;
	PWelsBlockZeroFunc pWelsBlockZero8x8Func;
};

const uint8_t g_kuiScan4[16]={		// for mb cache in sMb (only current element,without neighbor)
	// 4*4block scan mb cache order
	0,1,4,5,		// 0 1 | 4 5 0 1 | 2 3
	2,3,6,7,		// 2 3 | 6 7 4 5 | 6 7
	8,9,12,13,		// ----------------->-----------
	10,11,14,15		// 8 9 |12 13 8 9 |10 11
}; 	// 10 11 |14 15 12 13 |14 15

typedef struct TagNeighborAvail{
	int32_t iTopAvail;
	int32_t iLeftAvail;
	int32_t iRightTopAvail;
	int32_t iLeftTopAvail;		// used for check intra_pred_mode avail or not		// 1: avail; 0: unavail

	int32_t iLeftType;
	int32_t iTopType;
	int32_t iLeftTopType;
	int32_t iRightTopType;

	int8_t iTopCbp;
	int8_t iLeftCbp;
	int8_t iDummy[2];		// for align
} SWelsNeighAvail,* PWelsNeighAvail;

typedef void (*PWelsFillNeighborMbInfoIntra4x4Func) (PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCount,int8_t* pIntraPredMode,PDqLayer pCurDqLayer);
typedef void (*PWelsMapNeighToSample) (PWelsNeighAvail pNeighAvail,int32_t* pSampleAvail);
typedef void (*PWelsMap16NeighToSample) (PWelsNeighAvail pNeighAvail,uint8_t* pSampleAvail);

// Last Decoded Picture Info
struct SWelsLastDecPicInfo{
	// Save the last nal header info
	SNalUnitHeaderExt sLastNalHdrExt;
	SSliceHeader sLastSliceHeader;
	int32_t iPrevPicOrderCntMsb;
	int32_t iPrevPicOrderCntLsb;
	SPicture* pPreviousDecodedPictureInDpb;			// pointer to previously decoded picture in DPB for error concealment
	int32_t iPrevFrameNum;							// frame number of previous frame well decoded for non-truncated mode yet
	bool bLastHasMmco5;
};

#define MAX_PRED_MODE_ID_I16x16 3
#define MAX_PRED_MODE_ID_CHROMA 3
#define MAX_PRED_MODE_ID_I4x4 8
#define WELS_QP_MAX 51

#define IMinInt32 -0x7FFFFFFF
struct SWelsCabacCtx{
	uint8_t uiState;
	uint8_t uiMPS;
};

struct SWelsCabacDecEngine{
	uint64_t uiRange;
	uint64_t uiOffset;
	int32_t iBitsLeft;
	uint8_t* pBuffStart;
	uint8_t* pBuffCurr;
	uint8_t* pBuffEnd;
};

#define NEW_CTX_OFFSET_MB_TYPE_I 3
#define NEW_CTX_OFFSET_SKIP 11
#define NEW_CTX_OFFSET_SUBMB_TYPE 21
#define NEW_CTX_OFFSET_B_SUBMB_TYPE 36
#define NEW_CTX_OFFSET_MVD 40
#define NEW_CTX_OFFSET_REF_NO 54
#define NEW_CTX_OFFSET_DELTA_QP 60
#define NEW_CTX_OFFSET_IPR 68
#define NEW_CTX_OFFSET_CIPR 64
#define NEW_CTX_OFFSET_CBP 73
#define NEW_CTX_OFFSET_CBF 85
#define NEW_CTX_OFFSET_MAP 105
#define NEW_CTX_OFFSET_LAST 166
#define NEW_CTX_OFFSET_ONE 227
#define NEW_CTX_OFFSET_ABS 232
#define NEW_CTX_OFFSET_TS_8x8_FLAG 399
#define CTX_NUM_MVD 7
#define CTX_NUM_CBP 4
// Table 9-34 in Page 270
#define NEW_CTX_OFFSET_TRANSFORM_SIZE_8X8_FLAG 399
#define NEW_CTX_OFFSET_MAP_8x8 402
#define NEW_CTX_OFFSET_LAST_8x8 417
#define NEW_CTX_OFFSET_ONE_8x8 426
#define NEW_CTX_OFFSET_ABS_8x8 431		// Puzzle,where is the definition?

// SDecoderContext: to maintail all modules data over decoder@framework
struct SDecoderContext{
	// Input
	void* pArgDec;							// structured arguments for decoder,reserved here for extension in the future

	SDataBuffer sRawData;
	SDataBuffer sSavedData;					// for parse only purpose

	// Configuration
	SDecodingParam* pParam;
	uint32_t uiCpuFlag;						// CPU compatibility detected

	VIDEO_BITSTREAM_TYPE eVideoType;		// indicate the type of video to decide whether or not to do qp_delta error detection.
	bool bHaveGotMemory;					// global memory for decoder context related ever requested?

	int32_t iImgWidthInPixel;				// width of image in pixel reconstruction picture to be output
	int32_t iImgHeightInPixel;				// height of image in pixel reconstruction picture to be output
	int32_t iLastImgWidthInPixel;			// width of image in last successful pixel reconstruction picture to be output
	int32_t iLastImgHeightInPixel;			// height of image in last successful pixel reconstruction picture to be output
	bool bFreezeOutput;		// indicating current frame freezing. Default: true

	// Derived common elements
	SNalUnitHeader sCurNalHead;
	EWelsSliceType eSliceType;				// Slice type
	bool bUsedAsRef; 						// flag as ref
	int32_t iFrameNum;
	int32_t iErrorCode; 					// error code return while decoding in case packets lost
	SFmo sFmoList[MAX_PPS_COUNT];			// list for FMO storage
	SFmo* pFmo; 								// current fmo context after parsed slice_header
	int32_t iActiveFmoNum;					// active count number of fmo context in list

	// needed info by decode slice level and mb level
	int32_t	iDecBlockOffsetArray[24];		// address talbe for sub 4x4 block in intra4x4_mb,so no need to caculta the address every time.

	struct{
		uint32_t* pMbType[LAYER_NUM_EXCHANGEABLE];	// mb type
		int16_t(*pMv[LAYER_NUM_EXCHANGEABLE][LIST_A])[MB_BLOCK4x4_NUM][MV_A];		// [LAYER_NUM_EXCHANGEABLE MB_BLOCK4x4_NUM*]
		int8_t(*pRefIndex[LAYER_NUM_EXCHANGEABLE][LIST_A])[MB_BLOCK4x4_NUM];
		int8_t(*pDirect[LAYER_NUM_EXCHANGEABLE])[MB_BLOCK4x4_NUM];
		bool* pNoSubMbPartSizeLessThan8x8Flag[LAYER_NUM_EXCHANGEABLE];
		bool* pTransformSize8x8Flag[LAYER_NUM_EXCHANGEABLE];
		int8_t* pLumaQp[LAYER_NUM_EXCHANGEABLE];		// mb luma_qp
		int8_t(*pChromaQp[LAYER_NUM_EXCHANGEABLE])[2];		// mb chroma_qp
		int16_t(*pMvd[LAYER_NUM_EXCHANGEABLE][LIST_A])[MB_BLOCK4x4_NUM][MV_A];		// [LAYER_NUM_EXCHANGEABLE MB_BLOCK4x4_NUM*]
		uint16_t* pCbfDc[LAYER_NUM_EXCHANGEABLE];
		int8_t(*pNzc[LAYER_NUM_EXCHANGEABLE])[24];
		int8_t(*pNzcRs[LAYER_NUM_EXCHANGEABLE])[24];
		int16_t(*pScaledTCoeff[LAYER_NUM_EXCHANGEABLE])[MB_COEFF_LIST_SIZE];	// need be aligned
		int8_t(*pIntraPredMode[LAYER_NUM_EXCHANGEABLE])[8];		// 0~3 top4x4 ; 4~6 left 4x4; 7 intra16x16
		int8_t(*pIntra4x4FinalMode[LAYER_NUM_EXCHANGEABLE])[MB_BLOCK4x4_NUM];
		uint8_t* pIntraNxNAvailFlag[LAYER_NUM_EXCHANGEABLE];
		int8_t* pChromaPredMode[LAYER_NUM_EXCHANGEABLE];
		int8_t* pCbp[LAYER_NUM_EXCHANGEABLE];
		uint8_t(*pMotionPredFlag[LAYER_NUM_EXCHANGEABLE][LIST_A])[MB_PARTITION_SIZE];		// 8x8
		uint32_t(*pSubMbType[LAYER_NUM_EXCHANGEABLE])[MB_SUB_PARTITION_SIZE];
		int32_t* pSliceIdc[LAYER_NUM_EXCHANGEABLE];		// using int32_t for slice_idc
		int8_t* pResidualPredFlag[LAYER_NUM_EXCHANGEABLE];
		int8_t* pInterPredictionDoneFlag[LAYER_NUM_EXCHANGEABLE];
		bool* pMbCorrectlyDecodedFlag[LAYER_NUM_EXCHANGEABLE];
		bool* pMbRefConcealedFlag[LAYER_NUM_EXCHANGEABLE];
		uint32_t iMbWidth;
		uint32_t iMbHeight;
	} sMb;

	// reconstruction picture
	SPicture* pDec; 							// pointer to current picture being reconstructed
	SPicture* pTempDec;							// pointer to temp decoder picture to be used only for Bi Prediction.
	SRefPic sRefPic;
	SVlcTable* pVlcTable;						// vlc table

	SBitStringAux sBs;
	int32_t iMaxBsBufferSizeInByte;				// actual memory size for BS buffer

	// Global memory external
	SWelsDecoderSpsPpsCTX sSpsPpsCtx;

	SPosOffset sFrameCrop;
	SSliceHeader* pSliceHeader;

	SPicBuff* pPicBuff;							// Initially allocated memory for pictures which are used in decoding.
	int32_t iPicQueueNumber;

	SAccessUnit* pAccessUnitList;				// current access unit list to be performed
	SSps* pSps;									// used by current AU
	SPps* pPps;									// used by current AU
// Memory for pAccessUnitList is dynamically held till decoder destruction.
	PDqLayer pCurDqLayer;						// current DQ layer representation,also carry reference base layer if applicable
	PDqLayer pDqLayersList[LAYER_NUM_EXCHANGEABLE];		// DQ layers list with memory allocated
	SNalUnit* pNalCur;							// point to current NAL Nnit
	uint8_t uiNalRefIdc;						// NalRefIdc for easy access;
	int32_t iPicWidthReq;						// picture width have requested the memory
	int32_t iPicHeightReq;						// picture height have requested the memory

	uint8_t uiTargetDqId;						// maximal DQ ID in current access unit,meaning target layer ID
	bool bEndOfStreamFlag;						// Flag on end of stream requested by external application layer
	bool bInstantDecFlag;						// Flag for no-delay decoding
	bool bInitialDqLayersMem;					// dq layers related memory is available?

	bool bOnlyOneLayerInCurAuFlag;				// only one layer in current AU: 1

	bool bReferenceLostAtT0Flag;
	int32_t iTotalNumMbRec;						// record current number of decoded MB
	bool bParamSetsLostFlag;					// sps or pps do not exist or not correct

	bool bCurAuContainLtrMarkSeFlag;			// current AU has the LTR marking syntax element,mark the previous frame or self
	int32_t iFrameNumOfAuMarkedLtr;				// if bCurAuContainLtrMarkSeFlag==true,SHOULD set this variable

	uint16_t uiCurIdrPicId;
	bool bNewSeqBegin;
	bool bNextNewSeqBegin;

	PGetIntraPredFunc pGetI16x16LumaPredFunc[7];	// h264_predict_copy_16x16;
	PGetIntraPredFunc pGetI4x4LumaPredFunc[14];		// h264_predict_4x4_t
	PGetIntraPredFunc pGetIChromaPredFunc[7];		// h264_predict_8x8_t
	PIdctResAddPredFunc pIdctResAddPredFunc;
	PIdctFourResAddPredFunc pIdctFourResAddPredFunc;
	SMcFunc sMcFunc;
// Transform8x8
	PGetIntraPred8x8Func pGetI8x8LumaPredFunc[14];
	PIdctResAddPredFunc pIdctResAddPredFunc8x8;

// For error concealment
	SCopyFunc sCopyFunc;
// For Deblocking
	SDeblockingFunc sDeblockingFunc;
	SExpandPicFunc sExpandPicFunc;

// For Block
	SBlockFunc sBlockFunc;

	int32_t iCurSeqIntervalTargetDependId;
	int32_t iCurSeqIntervalMaxPicWidth;
	int32_t iCurSeqIntervalMaxPicHeight;

	PWelsFillNeighborMbInfoIntra4x4Func pFillInfoCacheIntraNxNFunc;
	PWelsMapNeighToSample pMapNxNNeighToSampleFunc;
	PWelsMap16NeighToSample pMap16x16NeighToSampleFunc;

// feedback whether or not have VCL in current AU,and the temporal ID
	int32_t iFeedbackVclNalInAu;
	int32_t iFeedbackTidInAu;
	int32_t iFeedbackNalRefIdc;

	bool bAuReadyFlag;							// true: one au is ready for decoding; false: default value

	bool bPrintFrameErrorTraceFlag;				// true: can print info for upper layer
	int32_t iIgnoredErrorInfoPacketCount;		// store the packet number with error decoding info

	SWelsLastDecPicInfo* pLastDecPicInfo;

	SWelsCabacCtx sWelsCabacContexts[4][WELS_QP_MAX+1][WELS_CONTEXT_COUNT];
	bool bCabacInited;
	SWelsCabacCtx pCabacCtx[WELS_CONTEXT_COUNT];
	SWelsCabacDecEngine* pCabacDecEngine;
	double dDecTime;
// SDecoderStatistics* pDecoderStatistics;		// For real time debugging
	int32_t iMbEcedNum;
	int32_t iMbEcedPropNum;
	int32_t iMbNum;
	bool bMbRefConcealed;
	bool bRPLRError;
	int32_t iECMVs[16][2];
	SPicture* pECRefPic[16];
	uint64_t uiTimeStamp;
// To support scaling list HP
	uint16_t pDequant_coeff_buffer4x4[6][52][16];
	uint16_t pDequant_coeff_buffer8x8[6][52][64];
	uint16_t(*pDequant_coeff4x4[6])[16];		// 4x4 sclaing list value pointer
	uint16_t(*pDequant_coeff8x8[6])[64];		// 64 residual coeff ,with 6 kinds of residual type,52 qp level
	int iDequantCoeffPpsid;						// When a new pps actived,reinitialised the scaling list value
	bool bDequantCoeff4x4Init;
	bool bUseScalingList;
	int16_t lastReadyHeightOffset[LIST_A][MAX_REF_PIC_COUNT];		// last ready reference MB offset
	SPictInfo* pPictInfoList;
	SPictReoderingStatus* pPictReoderingStatus;
};

const uint32_t g_kuiGolombUELength[256]={
	1,3,3,5,5,5,5,7,7,7,7,7,7,7,7,		// 14
	9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,// 30
	11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,// 46
	11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,// 62
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,// 
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
	17
};

#define WRITE_BE_32(ptr,val) do { \
 (ptr)[0]=(val) >> 24; \
 (ptr)[1]=(val) >> 16; \
 (ptr)[2]=(val) >> 8; \
 (ptr)[3]=(val) >> 0; \
 } while (0)

#define WELS_READ_VERIFY(uiRet) do{ \
 uint32_t uiRetTmp=(uint32_t)uiRet; \
 if( uiRetTmp !=ERR_NONE ) \
 return uiRetTmp; \
}while(0)
#define GET_WORD(iCurBits,pBufPtr,iLeftBits,iAllowedBytes,iReadBytes) { \
 if (iReadBytes > iAllowedBytes+1) { \
 return ERR_INFO_READ_OVERFLOW; \
 } \
 iCurBits |=((uint32_t)((pBufPtr[0] << 8) | pBufPtr[1])) << (iLeftBits); \
 iLeftBits-=16; \
 pBufPtr+=2; \
}
#define NEED_BITS(iCurBits,pBufPtr,iLeftBits,iAllowedBytes,iReadBytes) { \
 if (iLeftBits > 0) { \
 GET_WORD(iCurBits,pBufPtr,iLeftBits,iAllowedBytes,iReadBytes); \
 } \
}
#define UBITS(iCurBits,iNumBits) (iCurBits>>(32-(iNumBits)))
#define DUMP_BITS(iCurBits,pBufPtr,iLeftBits,iNumBits,iAllowedBytes,iReadBytes) { \
 iCurBits <<=(iNumBits); \
 iLeftBits+=(iNumBits); \
 NEED_BITS(iCurBits,pBufPtr,iLeftBits,iAllowedBytes,iReadBytes); \
}

static inline int32_t BsGetBits(SBitStringAux* pBs,int32_t iNumBits,uint32_t* pCode){
	intX_t iRc=UBITS(pBs->uiCurBits,iNumBits);
	intX_t iAllowedBytes=pBs->pEndBuf-pBs->pStartBuf;		// actual stream bytes
	intX_t iReadBytes=pBs->pCurBuf-pBs->pStartBuf;
	DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,iNumBits,iAllowedBytes,iReadBytes);
	*pCode=(uint32_t)iRc;
	return ERR_NONE;
}

// Read one bit from bit stream followed
static inline uint32_t BsGetOneBit(SBitStringAux* pBs,uint32_t* pCode){
	return (BsGetBits(pBs,1,pCode));
}

// brief initialize bitstream writing
// param pBs Bit string auxiliary pointer
// param pBuf bit-stream pBuffer
// param iSize iSize in bits for decoder; iSize in bytes for encoder
// return iSize of pBuffer pData in byte; failed in-1 return
static inline int32_t InitBits(SBitStringAux* pBs,const uint8_t* kpBuf,const int32_t kiSize){
	uint8_t* ptr=(uint8_t*)kpBuf;
	pBs->pStartBuf=ptr;
	pBs->pCurBuf=ptr;
	pBs->pEndBuf=ptr+kiSize;
	pBs->iLeftBits=32;
	pBs->uiCurBits=0;
	return kiSize;
}

static inline int32_t BsWriteBits(SBitStringAux* pBitString,int32_t iLen,const uint32_t kuiValue){
	if(iLen<pBitString->iLeftBits){
		pBitString->uiCurBits=(pBitString->uiCurBits<<iLen)|kuiValue;
		pBitString->iLeftBits-=iLen;
	}else{
		iLen-=pBitString->iLeftBits;
		pBitString->uiCurBits=(pBitString->uiCurBits<<pBitString->iLeftBits)|(kuiValue>>iLen);
		WRITE_BE_32(pBitString->pCurBuf,pBitString->uiCurBits);
		pBitString->pCurBuf+=4;
		pBitString->uiCurBits=kuiValue&((1<<iLen)-1);
		pBitString->iLeftBits=32-iLen;
	}
	return 0;
}
// Write 1 bit
static inline int32_t BsWriteOneBit(SBitStringAux* pBitString,const uint32_t kuiValue){
	BsWriteBits(pBitString,1,kuiValue);
	return 0;
}

static inline int32_t BsFlush(SBitStringAux* pBitString){
	WRITE_BE_32(pBitString->pCurBuf,pBitString->uiCurBits<<pBitString->iLeftBits);
	pBitString->pCurBuf+=4-pBitString->iLeftBits/8;
	pBitString->iLeftBits=32;
	pBitString->uiCurBits=0;
	return 0;
}

// Write unsigned exp golomb codes
static inline int32_t BsWriteUE(SBitStringAux* pBitString,const uint32_t kuiValue){
	uint32_t iTmpValue=kuiValue+1;
	if(256>kuiValue){
		BsWriteBits(pBitString,g_kuiGolombUELength[kuiValue],kuiValue+1);
	}else{
		uint32_t n=0;
		if(iTmpValue&0xffff0000){
			iTmpValue>>=16;
			n+=16;
		}
		if(iTmpValue&0xff00){
			iTmpValue>>=8;
			n+=8;
		}

		// n+=(g_kuiGolombUELength[iTmpValue] >> 1);

		n+=(g_kuiGolombUELength[iTmpValue-1]>>1);
		BsWriteBits(pBitString,(n<<1)+1,kuiValue+1);
	}
	return 0;
}

// Write signed exp golomb codes
static inline int32_t BsWriteSE(SBitStringAux* pBitString,const int32_t kiValue){
	uint32_t iTmpValue;
	if(0==kiValue){
		BsWriteOneBit(pBitString,1);
	}else
	if(0<kiValue){
		iTmpValue=(kiValue<<1)-1;
		BsWriteUE(pBitString,iTmpValue);
	}else{
		iTmpValue=((-kiValue)<<1);
		BsWriteUE(pBitString,iTmpValue);
	}
	return 0;
}

// Write RBSP trailing bits
static inline int32_t BsRbspTrailingBits(SBitStringAux* pBitString){
	BsWriteOneBit(pBitString,1);
	BsFlush(pBitString);
	return 0;
}

inline uint32_t GetValue4Bytes(uint8_t* pDstNal){
	uint32_t uiValue=0;
	uiValue=(pDstNal[0]<<24)|(pDstNal[1]<<16)|(pDstNal[2]<<8)|(pDstNal[3]);
	return uiValue;
}

int32_t InitReadBits(SBitStringAux* pBitString,intX_t iEndOffset){
	if(pBitString->pCurBuf>=(pBitString->pEndBuf-iEndOffset)){
		return ERR_INFO_INVALID_ACCESS;
	}
	pBitString->uiCurBits=GetValue4Bytes(pBitString->pCurBuf);
	pBitString->pCurBuf+=4;
	pBitString->iLeftBits=-16;
	return ERR_NONE;
}

// brief input bits for decoder or initialize bitstream writing in encoder
// param pBitString Bit string auxiliary pointer
// param kpBuf bit-stream buffer
// param kiSize size in bits for decoder; size in bytes for encoder
// return 0: success,other: fail
int32_t DecInitBits(SBitStringAux* pBitString,const uint8_t* kpBuf,const int32_t kiSize){
	const int32_t kiSizeBuf=(kiSize+7)>>3;
	uint8_t* pTmp=(uint8_t*)kpBuf;
	if(NULL==pTmp)
		return ERR_INFO_INVALID_ACCESS;
	pBitString->pStartBuf=pTmp;				// buffer to start position
	pBitString->pEndBuf=pTmp+kiSizeBuf;		// buffer+length
	pBitString->iBits=kiSize;				// count bits of overall bitstreaming inputindex;
	pBitString->pCurBuf=pBitString->pStartBuf;
	int32_t iErr=InitReadBits(pBitString,0);
	if(iErr){
		return iErr;
	}
	return ERR_NONE;
}

const uint8_t g_kuiLeadingZeroTable[256]={
	8,7,6,6,5,5,5,5,4,4,4,4,4,4,4,4,
	3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
	2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

static inline int32_t GetLeadingZeroBits(uint32_t iCurBits){		// <=32 bits
	uint32_t uiValue;
	uiValue=UBITS(iCurBits,8);		// ShowBits( bs,8 );
	if(uiValue){
		return g_kuiLeadingZeroTable[uiValue];
	}
	uiValue=UBITS(iCurBits,16);		// ShowBits( bs,16 );
	if(uiValue){
		return (g_kuiLeadingZeroTable[uiValue]+8);
	}
	uiValue=UBITS(iCurBits,24);		// ShowBits( bs,24 );
	if(uiValue){
		return (g_kuiLeadingZeroTable[uiValue]+16);
	}
	uiValue=iCurBits;		// ShowBits( bs,32 );
	if(uiValue){
		return (g_kuiLeadingZeroTable[uiValue]+24);
	}
	// ASSERT(false);		// should not go here
	return-1;
}

static inline uint32_t BsGetUe(SBitStringAux* pBs,uint32_t* pCode){
	uint32_t iValue=0;
	int32_t iLeadingZeroBits=GetLeadingZeroBits(pBs->uiCurBits);
	intX_t iAllowedBytes,iReadBytes;
	iAllowedBytes=pBs->pEndBuf-pBs->pStartBuf;		// actual stream bytes
	if(iLeadingZeroBits==-1){						// bistream error
		return ERR_INFO_READ_LEADING_ZERO;			// -1
	}else
	if(iLeadingZeroBits>16){						// rarely into this condition (even may be bitstream error),prevent from 16-bit reading overflow
		// using two-step reading instead of one time reading of >16 bits.
		iReadBytes=pBs->pCurBuf-pBs->pStartBuf;
		DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,16,iAllowedBytes,iReadBytes);
		iReadBytes=pBs->pCurBuf-pBs->pStartBuf;
		DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,iLeadingZeroBits+1-16,iAllowedBytes,iReadBytes);
	}else{
		iReadBytes=pBs->pCurBuf-pBs->pStartBuf;
		DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,iLeadingZeroBits+1,iAllowedBytes,iReadBytes);
	}
	if(iLeadingZeroBits){
		iValue=UBITS(pBs->uiCurBits,iLeadingZeroBits);
		iReadBytes=pBs->pCurBuf-pBs->pStartBuf;
		DUMP_BITS(pBs->uiCurBits,pBs->pCurBuf,pBs->iLeftBits,iLeadingZeroBits,iAllowedBytes,iReadBytes);
	}

	*pCode=((1u<<iLeadingZeroBits)-1+iValue);
	return ERR_NONE;
}

// Read signed exp golomb codes
static inline int32_t BsGetSe(SBitStringAux* pBs,int32_t* pCode){
	uint32_t uiCodeNum;

	WELS_READ_VERIFY(BsGetUe(pBs,&uiCodeNum));

	if(uiCodeNum&0x01){
		*pCode=(int32_t)((uiCodeNum+1)>>1);
	}else{
		*pCode=NEG_NUM((int32_t)(uiCodeNum>>1));
	}
	return ERR_NONE;
}

void RBSP2EBSP(uint8_t* pDstBuf,uint8_t* pSrcBuf,const int32_t kiSize){
	uint8_t* pSrcPointer=pSrcBuf;
	uint8_t* pDstPointer=pDstBuf;
	uint8_t* pSrcEnd=pSrcBuf+kiSize;
	int32_t iZeroCount=0;
	while(pSrcPointer<pSrcEnd){
		if(iZeroCount==2 && *pSrcPointer<=3){
			// add the code 0x03
			*pDstPointer++=3;
			iZeroCount=0;
		}
		if(*pSrcPointer==0){
			++iZeroCount;
		}else{
			iZeroCount=0;
		}
		*pDstPointer++=*pSrcPointer++;
	}
}

int32_t MemFreeNalList(SAccessUnit** ppAu){
	if(ppAu!=NULL){
		SAccessUnit* pAu=*ppAu;
		if(pAu!=NULL){
			WelsFree(pAu);
			*ppAu=NULL;
		}
	}
	return ERR_NONE;
}

int32_t MemInitNalList(SAccessUnit** ppAu,const uint32_t kuiSize){
	uint32_t uiIdx=0;
	uint8_t* pBase=NULL,* pPtr=NULL;
	const uint32_t kuiSizeAu=sizeof(SAccessUnit);
	const uint32_t kuiSizeNalUnitPtr=kuiSize*sizeof(SNalUnit*);
	const uint32_t kuiSizeNalUnit=sizeof(SNalUnit);
	const uint32_t kuiCountSize=(kuiSizeAu+kuiSizeNalUnitPtr+kuiSize*kuiSizeNalUnit)*sizeof(uint8_t);

	if(kuiSize==0)
		return ERR_INFO_INVALID_PARAM;

	if(*ppAu!=NULL){
		MemFreeNalList(ppAu);
	}

	pBase=(uint8_t*)WelsMallocz(kuiCountSize);
	if(pBase==NULL)
		return ERR_INFO_OUT_OF_MEMORY;
	pPtr=pBase;
	*ppAu=(SAccessUnit*)pPtr;
	pPtr+=kuiSizeAu;
	(*ppAu)->pNalUnitsList=(SNalUnit**)pPtr;
	pPtr+=kuiSizeNalUnitPtr;
	do{
		(*ppAu)->pNalUnitsList[uiIdx]=(SNalUnit*)pPtr;
		pPtr+=kuiSizeNalUnit;
		++uiIdx;
	} while(uiIdx<kuiSize);
	(*ppAu)->uiCountUnitsNum=kuiSize;
	(*ppAu)->uiAvailUnitsNum=0;
	(*ppAu)->uiActualUnitsNum=0;
	(*ppAu)->uiStartPos=0;
	(*ppAu)->uiEndPos=0;
	(*ppAu)->bCompletedAuFlag=false;
	return ERR_NONE;
}

int32_t InitBsBuffer(SDecoderContext* pCtx){
	if(pCtx==NULL)
		return ERR_INFO_INVALID_PTR;

	pCtx->iMaxBsBufferSizeInByte=MIN_ACCESS_UNIT_CAPACITY*MAX_BUFFERED_NUM;
	if((pCtx->sRawData.pHead=static_cast<uint8_t*> (WelsMallocz(pCtx->iMaxBsBufferSizeInByte)))==NULL){
		return ERR_INFO_OUT_OF_MEMORY;
	}
	pCtx->sRawData.pStartPos=pCtx->sRawData.pCurPos=pCtx->sRawData.pHead;
	pCtx->sRawData.pEnd=pCtx->sRawData.pHead+pCtx->iMaxBsBufferSizeInByte;
	return ERR_NONE;
}

int32_t ExpandBsBuffer(SDecoderContext* pCtx,const int kiSrcLen){
	if(pCtx==NULL)
		return ERR_INFO_INVALID_PTR;
	int32_t iExpandStepShift=1;
	int32_t iNewBuffLen=WELS_MAX((kiSrcLen*MAX_BUFFERED_NUM),(pCtx->iMaxBsBufferSizeInByte<<iExpandStepShift));
	// allocate new bs buffer

	// Realloc sRawData
	uint8_t* pNewBsBuff=static_cast<uint8_t*> (WelsMallocz(iNewBuffLen));
	if(pNewBsBuff==NULL){
		FATAL("ExpandBsBuffer() Failed for malloc pNewBsBuff (%d)",iNewBuffLen);
		pCtx->iErrorCode|=dsOutOfMemory;
		return ERR_INFO_OUT_OF_MEMORY;
	}

	// Calculate and set the bs start and end position
	for(uint32_t i=0; i<=pCtx->pAccessUnitList->uiActualUnitsNum; i++){
		SBitStringAux* pSliceBitsRead=&pCtx->pAccessUnitList->pNalUnitsList[i]->sNalData.sVclNal.sSliceBitsRead;
		pSliceBitsRead->pStartBuf=pSliceBitsRead->pStartBuf-pCtx->sRawData.pHead+pNewBsBuff;
		pSliceBitsRead->pEndBuf=pSliceBitsRead->pEndBuf-pCtx->sRawData.pHead+pNewBsBuff;
		pSliceBitsRead->pCurBuf=pSliceBitsRead->pCurBuf-pCtx->sRawData.pHead+pNewBsBuff;
	}

	// Copy current buffer status to new buffer
	memcpy(pNewBsBuff,pCtx->sRawData.pHead,pCtx->iMaxBsBufferSizeInByte);
	pCtx->sRawData.pStartPos=pNewBsBuff+(pCtx->sRawData.pStartPos-pCtx->sRawData.pHead);
	pCtx->sRawData.pCurPos=pNewBsBuff+(pCtx->sRawData.pCurPos-pCtx->sRawData.pHead);
	pCtx->sRawData.pEnd=pNewBsBuff+iNewBuffLen;
	WelsFree(pCtx->sRawData.pHead);
	pCtx->sRawData.pHead=pNewBsBuff;
	pCtx->iMaxBsBufferSizeInByte=iNewBuffLen;
	return ERR_NONE;
}

int32_t CheckBsBuffer(SDecoderContext* pCtx,const int32_t kiSrcLen){
	if(kiSrcLen>MAX_ACCESS_UNIT_CAPACITY){		// exceeds max allowed data
		uprintf("Max AU size exceeded. Allowed size=%d,current size=%d",MAX_ACCESS_UNIT_CAPACITY,kiSrcLen);
		pCtx->iErrorCode|=dsBitstreamError;
		return ERR_INFO_INVALID_ACCESS;
	}else
	if(kiSrcLen>pCtx->iMaxBsBufferSizeInByte/
			 MAX_BUFFERED_NUM){		// may lead to buffer overwrite,prevent it by expanding buffer
		if(ExpandBsBuffer(pCtx,kiSrcLen)){
			return ERR_INFO_OUT_OF_MEMORY;
		}
	}
	return ERR_NONE;
}


// WelsInitStaticMemory
// Memory request for new introduced data
// Especially for:
// rbsp_au_buffer,cur_dq_layer_ptr and ref_dq_layer_ptr in MB info cache.
// return:
// 0-success; otherwise returned error_no defined in error_no.h.
int32_t WelsInitStaticMemory(SDecoderContext* pCtx){
	if(pCtx==NULL){
		return ERR_INFO_INVALID_PTR;
	}

	if(MemInitNalList(&pCtx->pAccessUnitList,MAX_NAL_UNIT_NUM_IN_AU)!=0)
		return ERR_INFO_OUT_OF_MEMORY;

	if(InitBsBuffer(pCtx)!=0)
		return ERR_INFO_OUT_OF_MEMORY;

	pCtx->uiTargetDqId=(uint8_t)-1;
	pCtx->bEndOfStreamFlag=false;

	return ERR_NONE;
}

class PlainH264Decoder: public ISVCDecoderBase{
	public:
		PlainH264Decoder(void);
		virtual ~PlainH264Decoder();
		virtual long Initialize(const SDecodingParam* pParam);
		virtual long Uninitialize();
		virtual DECODING_STATE DecodeFrame(const unsigned char* kpSrc,const int kiSrcLen,unsigned char** ppDst,SBufferInfo* pDstInfo);
		virtual long SetOption(DECODER_OPTION eOptID,void* pOption);
		virtual long GetOption(DECODER_OPTION eOptID,void* pOption);
		//virtual void GetProfileDisplay(std::vector<DisplayTimer>* profilerDisplay) {
		//	*profilerDisplay=m_profilerDisplay;
		//}
	private:
		//std::vector<DisplayTimer> m_profilerDisplay;
		uint32_t m_uiDecodeTimeStamp;
		SPicBuff* m_pPicBuff;
		bool m_bParamSetsLostFlag;
		bool m_bFreezeOutput;
		int32_t m_DecCtxActiveCount;
		SDecoderContext* m_pCtx;
		int32_t m_iLastBufferedIdx;
		SPictInfo m_sPictInfoList[16];
		SPictReoderingStatus m_sReoderingStatus;
		SVlcTable m_sVlcTable;
		SWelsLastDecPicInfo m_sLastDecPicInfo;
		int32_t InitDecoder(const SDecodingParam* pParam);
		void UninitDecoder(void);
		int32_t InitDecoderCtx(const SDecodingParam* pParam);
		void UninitDecoderCtx(SDecoderContext* pCtx);
		//Profiler m_profiler;
};


long CreateDecoder(ISVCDecoderBase** ppDecoder){

	if(NULL==ppDecoder){
		return ERR_INVALID_PARAMETERS;
	}

	*ppDecoder=new PlainH264Decoder();

	if(NULL==*ppDecoder){
		return ERR_MALLOC_FAILED;
	}

	return ERR_NONE;
}

#define countof(Array) ((int)(sizeof(Array)/sizeof(*Array)))

static inline int32_t GetThreadCount(SDecoderContext* pCtx){
	return 0;
}

// true-the AU to be construct is the start of new sequence; false-not
static bool CheckNewSeqBeginAndUpdateActiveLayerSps(SDecoderContext* pCtx){
	bool bNewSeq=false;
	SAccessUnit* pCurAu=pCtx->pAccessUnitList;
	SSps* pTmpLayerSps[MAX_LAYER_NUM];
	for(int i=0; i<MAX_LAYER_NUM; i++){
		pTmpLayerSps[i]=NULL;
	}
	// track the layer sps for the current au
	for(unsigned int i=pCurAu->uiStartPos; i<=pCurAu->uiEndPos; i++){
		uint32_t uiDid=pCurAu->pNalUnitsList[i]->sNalHeaderExt.uiDependencyId;
		pTmpLayerSps[uiDid]=pCurAu->pNalUnitsList[i]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.pSps;
		if((pCurAu->pNalUnitsList[i]->sNalHeaderExt.sNalUnitHeader.eNalUnitType==NAL_UNIT_CODED_SLICE_IDR) || (pCurAu->pNalUnitsList[i]->sNalHeaderExt.bIdrFlag))
			bNewSeq=true;
	}
	int iMaxActiveLayer=0,iMaxCurrentLayer=0;
	for(int i=MAX_LAYER_NUM-1; i>=0; i--){
		if(pCtx->sSpsPpsCtx.pActiveLayerSps[i]!=NULL){
			iMaxActiveLayer=i;
			break;
		}
	}
	for(int i=MAX_LAYER_NUM-1; i>=0; i--){
		if(pTmpLayerSps[i]!=NULL){
			iMaxCurrentLayer=i;
			break;
		}
	}
	if((iMaxCurrentLayer!=iMaxActiveLayer) || (pTmpLayerSps[iMaxCurrentLayer]!=pCtx->sSpsPpsCtx.pActiveLayerSps[iMaxActiveLayer])){
		bNewSeq=true;
	}
	// fill active sps if the current sps is not null while active layer is null
	if(!bNewSeq){
		for(int i=0; i<MAX_LAYER_NUM; i++){
			if(pCtx->sSpsPpsCtx.pActiveLayerSps[i]==NULL && pTmpLayerSps[i]!=NULL){
				pCtx->sSpsPpsCtx.pActiveLayerSps[i]=pTmpLayerSps[i];
			}
		}
	}else{
		// UpdateActiveLayerSps if new sequence start
		memcpy(&pCtx->sSpsPpsCtx.pActiveLayerSps[0],&pTmpLayerSps[0],MAX_LAYER_NUM*sizeof(SSps*));
	}
	return bNewSeq;
}

int32_t UpdateAccessUnit(SDecoderContext* pCtx){
	SAccessUnit* pCurAu=pCtx->pAccessUnitList;
	int32_t iIdx=pCurAu->uiEndPos;

	// Conversed iterator
	pCtx->uiTargetDqId=pCurAu->pNalUnitsList[iIdx]->sNalHeaderExt.uiLayerDqId;
	pCurAu->uiActualUnitsNum=iIdx+1;
	pCurAu->bCompletedAuFlag=true;

	// Added for mosaic avoidance,11/19/2009
	if(pCtx->bParamSetsLostFlag || pCtx->bNewSeqBegin){
		uint32_t uiActualIdx=0;
		while(uiActualIdx<pCurAu->uiActualUnitsNum){
			SNalUnit* nal=pCurAu->pNalUnitsList[uiActualIdx];
			if(nal->sNalHeaderExt.sNalUnitHeader.eNalUnitType==NAL_UNIT_CODED_SLICE_IDR || nal->sNalHeaderExt.bIdrFlag){
				break;
			}
			++uiActualIdx;
		}
		if(uiActualIdx==pCurAu->uiActualUnitsNum){		// no found IDR nal within incoming AU,need exit to avoid mosaic issue,11/19/2009
			if(!pCtx->bParamSetsLostFlag)
				uprintf("UpdateAccessUnit():::::Key frame lost.....CAN NOT find IDR from current AU.");
			pCtx->iErrorCode|=dsRefLost;
			if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
				pCtx->iErrorCode|=dsNoParamSets;
				return dsNoParamSets;
			}
		}
	}
	return ERR_NONE;
}

// main purpose: to support multi-slice and to include all slice which have the same uiDependencyId,uiQualityId and frame_num
// for single slice,pIdxNoInterLayerPred SHOULD NOT be modified
void RefineIdxNoInterLayerPred(SAccessUnit* pCurAu,int32_t* pIdxNoInterLayerPred){
	int32_t iLastNalDependId=pCurAu->pNalUnitsList[*pIdxNoInterLayerPred]->sNalHeaderExt.uiDependencyId;
	int32_t iLastNalQualityId=pCurAu->pNalUnitsList[*pIdxNoInterLayerPred]->sNalHeaderExt.uiQualityId;
	uint8_t uiLastNalTId=pCurAu->pNalUnitsList[*pIdxNoInterLayerPred]->sNalHeaderExt.uiTemporalId;
	int32_t iLastNalFrameNum=pCurAu->pNalUnitsList[*pIdxNoInterLayerPred]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iFrameNum;
	int32_t iLastNalPoc=pCurAu->pNalUnitsList[*pIdxNoInterLayerPred]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iPicOrderCntLsb;
	int32_t iLastNalFirstMb=pCurAu->pNalUnitsList[*pIdxNoInterLayerPred]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iFirstMbInSlice;
	int32_t iCurNalDependId,iCurNalQualityId,iCurNalTId,iCurNalFrameNum,iCurNalPoc,iCurNalFirstMb,iCurIdx,iFinalIdxNoInterLayerPred;
	bool bMultiSliceFind=false;
	iFinalIdxNoInterLayerPred=0;
	iCurIdx=*pIdxNoInterLayerPred-1;
	while(iCurIdx>=0){
		if(pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.iNoInterLayerPredFlag){
			iCurNalDependId=pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.uiDependencyId;
			iCurNalQualityId=pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.uiQualityId;
			iCurNalTId=pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.uiTemporalId;
			iCurNalFrameNum=pCurAu->pNalUnitsList[iCurIdx]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iFrameNum;
			iCurNalPoc=pCurAu->pNalUnitsList[iCurIdx]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iPicOrderCntLsb;
			iCurNalFirstMb=pCurAu->pNalUnitsList[iCurIdx]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iFirstMbInSlice;
			if(iCurNalDependId==iLastNalDependId && iCurNalQualityId==iLastNalQualityId && iCurNalTId==uiLastNalTId && iCurNalFrameNum==iLastNalFrameNum && iCurNalPoc==iLastNalPoc && iCurNalFirstMb!=iLastNalFirstMb){
				bMultiSliceFind=true;
				iFinalIdxNoInterLayerPred=iCurIdx;
				--iCurIdx;
				continue;
			}else{
				break;
			}
		}
		--iCurIdx;
	}
	if(bMultiSliceFind && *pIdxNoInterLayerPred!=iFinalIdxNoInterLayerPred){
		*pIdxNoInterLayerPred=iFinalIdxNoInterLayerPred;
	}
}


void CheckAvailNalUnitsListContinuity(SDecoderContext* pCtx,int32_t iStartIdx,int32_t iEndIdx){
	SAccessUnit* pCurAu=pCtx->pAccessUnitList;
	uint8_t uiLastNuDependencyId,uiLastNuLayerDqId;
	uint8_t uiCurNuDependencyId,uiCurNuQualityId,uiCurNuLayerDqId,uiCurNuRefLayerDqId;
	int32_t iCurNalUnitIdx=0;

	// check the continuity of pNalUnitsList forwards (from pIdxNoInterLayerPred to end_postion)
	uiLastNuDependencyId=pCurAu->pNalUnitsList[iStartIdx]->sNalHeaderExt.uiDependencyId;	// starting nal unit
	uiLastNuLayerDqId=pCurAu->pNalUnitsList[iStartIdx]->sNalHeaderExt.uiLayerDqId;	// starting nal unit
	iCurNalUnitIdx=iStartIdx+1;	// current nal unit
	while(iCurNalUnitIdx<=iEndIdx){
		uiCurNuDependencyId=pCurAu->pNalUnitsList[iCurNalUnitIdx]->sNalHeaderExt.uiDependencyId;
		uiCurNuQualityId=pCurAu->pNalUnitsList[iCurNalUnitIdx]->sNalHeaderExt.uiQualityId;
		uiCurNuLayerDqId=pCurAu->pNalUnitsList[iCurNalUnitIdx]->sNalHeaderExt.uiLayerDqId;
		uiCurNuRefLayerDqId=pCurAu->pNalUnitsList[iCurNalUnitIdx]->sNalData.sVclNal.sSliceHeaderExt.uiRefLayerDqId;
		if(uiCurNuDependencyId==uiLastNuDependencyId){
			uiLastNuLayerDqId=uiCurNuLayerDqId;
			++iCurNalUnitIdx;
		}else{		// uiCurNuDependencyId !=uiLastNuDependencyId,new dependency arrive
			if(uiCurNuQualityId==0){
				uiLastNuDependencyId=uiCurNuDependencyId;
				if(uiCurNuRefLayerDqId==uiLastNuLayerDqId){
					uiLastNuLayerDqId=uiCurNuLayerDqId;
					++iCurNalUnitIdx;
				}else{		// cur_nu_layer_id !=next_nu_ref_layer_dq_id,the chain is broken at this point
					break;
				}
			}else{		// new dependency arrive,but no base quality layer,so we must stop in this point
				break;
			}
		}
	}
	--iCurNalUnitIdx;
	pCurAu->uiEndPos=iCurNalUnitIdx;
	pCtx->uiTargetDqId=pCurAu->pNalUnitsList[iCurNalUnitIdx]->sNalHeaderExt.uiLayerDqId;
}

bool CheckPocOfCurValidNalUnits(SAccessUnit* pCurAu,int32_t pIdxNoInterLayerPred){
	int32_t iEndIdx=pCurAu->uiEndPos;
	int32_t iCurAuPoc=pCurAu->pNalUnitsList[pIdxNoInterLayerPred]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iPicOrderCntLsb;
	int32_t iTmpPoc,i;
	for(i=pIdxNoInterLayerPred+1; i<iEndIdx; i++){
		iTmpPoc=pCurAu->pNalUnitsList[i]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iPicOrderCntLsb;
		if(iTmpPoc!=iCurAuPoc){
			return false;
		}
	}

	return true;
}

bool CheckIntegrityNalUnitsList(SDecoderContext* pCtx){
	SAccessUnit* pCurAu=pCtx->pAccessUnitList;
	const int32_t kiEndPos=pCurAu->uiEndPos;
	int32_t iIdxNoInterLayerPred=0;
	if(!pCurAu->bCompletedAuFlag)
		return false;
	if(pCtx->bNewSeqBegin){
		pCurAu->uiStartPos=0;
		// step1: search the pNalUnit whose iNoInterLayerPredFlag equal to 1 backwards (from uiEndPos to 0)
		iIdxNoInterLayerPred=kiEndPos;
		while(iIdxNoInterLayerPred>=0){
			if(pCurAu->pNalUnitsList[iIdxNoInterLayerPred]->sNalHeaderExt.iNoInterLayerPredFlag){
				break;
			}
			--iIdxNoInterLayerPred;
		}
		if(iIdxNoInterLayerPred<0){
			// can not find the Nal Unit whose no_inter_pred_falg equal to 1,MUST STOP decode
			return false;
		}
		// step2: support multi-slice,to include all base layer slice
		RefineIdxNoInterLayerPred(pCurAu,&iIdxNoInterLayerPred);
		pCurAu->uiStartPos=iIdxNoInterLayerPred;
		CheckAvailNalUnitsListContinuity(pCtx,iIdxNoInterLayerPred,kiEndPos);
		if(!CheckPocOfCurValidNalUnits(pCurAu,iIdxNoInterLayerPred)){
			return false;
		}

		pCtx->iCurSeqIntervalTargetDependId=pCurAu->pNalUnitsList[pCurAu->uiEndPos]->sNalHeaderExt.uiDependencyId;
		pCtx->iCurSeqIntervalMaxPicWidth=pCurAu->pNalUnitsList[pCurAu->uiEndPos]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iMbWidth<<4;
		pCtx->iCurSeqIntervalMaxPicHeight=pCurAu->pNalUnitsList[pCurAu->uiEndPos]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.iMbHeight<<4;
	}else{		// P_SLICE
		// step 1: search uiDependencyId equal to pCtx->cur_seq_interval_target_dependency_id
		bool bGetDependId=false;
		int32_t iIdxDependId=0;
		iIdxDependId=kiEndPos;
		while(iIdxDependId>=0){
			if(pCtx->iCurSeqIntervalTargetDependId==pCurAu->pNalUnitsList[iIdxDependId]->sNalHeaderExt.uiDependencyId){
				bGetDependId=true;
				break;
			}else{
				--iIdxDependId;
			}
		}
		// step 2: switch according to whether or not find the index of pNalUnit whose uiDependencyId equal to iCurSeqIntervalTargetDependId
		if(bGetDependId){		// get the index of pNalUnit whose uiDependencyId equal to iCurSeqIntervalTargetDependId
			bool bGetNoInterPredFront=false;
			// step 2a: search iNoInterLayerPredFlag [0....iIdxDependId]
			iIdxNoInterLayerPred=iIdxDependId;
			while(iIdxNoInterLayerPred>=0){
				if(pCurAu->pNalUnitsList[iIdxNoInterLayerPred]->sNalHeaderExt.iNoInterLayerPredFlag){
					bGetNoInterPredFront=true;
					break;
				}
				--iIdxNoInterLayerPred;
			}
			// step 2b: switch,whether or not find the NAL unit whose no_inter_pred_flag equal to 1 among [0....iIdxDependId]
			if(bGetNoInterPredFront){		// YES
				RefineIdxNoInterLayerPred(pCurAu,&iIdxNoInterLayerPred);
				pCurAu->uiStartPos=iIdxNoInterLayerPred;
				CheckAvailNalUnitsListContinuity(pCtx,iIdxNoInterLayerPred,iIdxDependId);
				if(!CheckPocOfCurValidNalUnits(pCurAu,iIdxNoInterLayerPred)){
					return false;
				}
			}else{		// NO,should find the NAL unit whose no_inter_pred_flag equal to 1 among [iIdxDependId....uiEndPos]
				iIdxNoInterLayerPred=iIdxDependId;
				while(iIdxNoInterLayerPred<=kiEndPos){
					if(pCurAu->pNalUnitsList[iIdxNoInterLayerPred]->sNalHeaderExt.iNoInterLayerPredFlag){
						break;
					}
					++iIdxNoInterLayerPred;
				}
				if(iIdxNoInterLayerPred>kiEndPos){
					return false;		// cann't find the index of pNalUnit whose no_inter_pred_flag=1
				}
				RefineIdxNoInterLayerPred(pCurAu,&iIdxNoInterLayerPred);
				pCurAu->uiStartPos=iIdxNoInterLayerPred;
				CheckAvailNalUnitsListContinuity(pCtx,iIdxNoInterLayerPred,kiEndPos);
				if(!CheckPocOfCurValidNalUnits(pCurAu,iIdxNoInterLayerPred)){
					return false;
				}
			}
		}else{		// without the index of pNalUnit,should process this AU as common case
			iIdxNoInterLayerPred=kiEndPos;
			while(iIdxNoInterLayerPred>=0){
				if(pCurAu->pNalUnitsList[iIdxNoInterLayerPred]->sNalHeaderExt.iNoInterLayerPredFlag){
					break;
				}
				--iIdxNoInterLayerPred;
			}
			if(iIdxNoInterLayerPred<0){
				return false;		// cann't find the index of pNalUnit whose iNoInterLayerPredFlag=1
			}
			RefineIdxNoInterLayerPred(pCurAu,&iIdxNoInterLayerPred);
			pCurAu->uiStartPos=iIdxNoInterLayerPred;
			CheckAvailNalUnitsListContinuity(pCtx,iIdxNoInterLayerPred,kiEndPos);
			if(!CheckPocOfCurValidNalUnits(pCurAu,iIdxNoInterLayerPred)){
				return false;
			}
		}
	}
	return true;
}

void CheckOnlyOneLayerInAu(SDecoderContext* pCtx){
	SAccessUnit* pCurAu=pCtx->pAccessUnitList;
	int32_t iEndIdx=pCurAu->uiEndPos;
	int32_t iCurIdx=pCurAu->uiStartPos;
	uint8_t uiDId=pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.uiDependencyId;
	uint8_t uiQId=pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.uiQualityId;
	uint8_t uiTId=pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.uiTemporalId;
	uint8_t uiCurDId,uiCurQId,uiCurTId;
	pCtx->bOnlyOneLayerInCurAuFlag=true;
	if(iEndIdx==iCurIdx){		// only one NAL in pNalUnitsList
		return;
	}
	++iCurIdx;
	while(iCurIdx<=iEndIdx){
		uiCurDId=pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.uiDependencyId;
		uiCurQId=pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.uiQualityId;
		uiCurTId=pCurAu->pNalUnitsList[iCurIdx]->sNalHeaderExt.uiTemporalId;
		if(uiDId!=uiCurDId || uiQId!=uiCurQId || uiTId!=uiCurTId){
			pCtx->bOnlyOneLayerInCurAuFlag=false;
			return;
		}
		++iCurIdx;
	}
}


int32_t WelsDecodeAccessUnitStart(SDecoderContext* pCtx){
	// Roll back NAL units not being belong to current access unit list for proceeded access unit
	int32_t iRet=UpdateAccessUnit(pCtx);
	if(iRet!=ERR_NONE)
		return iRet;
	pCtx->pAccessUnitList->uiStartPos=0;
	if(!pCtx->sSpsPpsCtx.bAvcBasedFlag && !CheckIntegrityNalUnitsList(pCtx)){
		pCtx->iErrorCode|=dsBitstreamError;
		return dsBitstreamError;
	}
	// check current AU has only one layer or not
	// If YES,can use deblocking based on AVC
	if(!pCtx->sSpsPpsCtx.bAvcBasedFlag){
		CheckOnlyOneLayerInAu(pCtx);
	}
	return ERR_NONE;
}

void GetVclNalTemporalId(SDecoderContext* pCtx){
	SAccessUnit* pAccessUnit=pCtx->pAccessUnitList;
	int32_t idx=pAccessUnit->uiStartPos;
	pCtx->iFeedbackVclNalInAu=FEEDBACK_VCL_NAL;
	pCtx->iFeedbackTidInAu=pAccessUnit->pNalUnitsList[idx]->sNalHeaderExt.uiTemporalId;
	pCtx->iFeedbackNalRefIdc=pAccessUnit->pNalUnitsList[idx]->sNalHeaderExt.sNalUnitHeader.uiNalRefIdc;
}

// brief Force reset current Acess Unit Nal list in case error parsing/decoding in current AU
// author
// history 11/16/2009
void ForceResetCurrentAccessUnit(SAccessUnit* pAu){
	uint32_t uiSucAuIdx=pAu->uiEndPos+1;
	uint32_t uiCurAuIdx=0;
	// swap the succeeding AU's nal units to the front
	while(uiSucAuIdx<pAu->uiAvailUnitsNum){
		SNalUnit* t=pAu->pNalUnitsList[uiSucAuIdx];
		pAu->pNalUnitsList[uiSucAuIdx]=pAu->pNalUnitsList[uiCurAuIdx];
		pAu->pNalUnitsList[uiCurAuIdx]=t;
		++uiSucAuIdx;
		++uiCurAuIdx;
	}
	// Update avail/actual units num accordingly for next AU parsing
	if(pAu->uiAvailUnitsNum>pAu->uiEndPos)
		pAu->uiAvailUnitsNum-=(pAu->uiEndPos+1);
	else
		pAu->uiAvailUnitsNum=0;
	pAu->uiActualUnitsNum=0;
	pAu->uiStartPos=0;
	pAu->uiEndPos=0;
	pAu->bCompletedAuFlag=false;
}

static inline void ResetActiveSPSForEachLayer(SDecoderContext* pCtx){
	if(pCtx->iTotalNumMbRec==0){
		for(int i=0; i<MAX_LAYER_NUM; i++){
			pCtx->sSpsPpsCtx.pActiveLayerSps[i]=NULL;
		}
	}
}

// WelsDecodeInitAccessUnitStart
// check and (re)allocate picture buffers on new sequence begin
// bit_len: size in bit length of data
// buf_len: size in byte length of data
// coded_au: mark an Access Unit decoding finished
// return:
// 0-success; otherwise returned error_no defined in error_no.h
int32_t WelsDecodeInitAccessUnitStart(SDecoderContext* pCtx,SBufferInfo* pDstInfo){
	int32_t iErr=ERR_NONE;
	SAccessUnit* pCurAu=pCtx->pAccessUnitList;
	pCtx->bAuReadyFlag=false;
	pCtx->pLastDecPicInfo->bLastHasMmco5=false;
	bool bTmpNewSeqBegin=CheckNewSeqBeginAndUpdateActiveLayerSps(pCtx);
	pCtx->bNewSeqBegin=pCtx->bNewSeqBegin || bTmpNewSeqBegin;
	iErr=WelsDecodeAccessUnitStart(pCtx);
	GetVclNalTemporalId(pCtx);
	if(ERR_NONE!=iErr){
		ForceResetCurrentAccessUnit(pCtx->pAccessUnitList);
		pDstInfo->iBufferStatus=0;
		pCtx->bNewSeqBegin=pCtx->bNewSeqBegin || pCtx->bNextNewSeqBegin;
		pCtx->bNextNewSeqBegin=false;		// reset it
		if(pCtx->bNewSeqBegin)
			ResetActiveSPSForEachLayer(pCtx);
		return iErr;
	}
	pCtx->pSps=pCurAu->pNalUnitsList[pCurAu->uiStartPos]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.pSps;
	pCtx->pPps=pCurAu->pNalUnitsList[pCurAu->uiStartPos]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.pPps;
	return iErr;
}

static void SetUnRef(SPicture* pRef){
	if(NULL!=pRef){
		pRef->bUsedAsRef=false;
		pRef->bIsLongRef=false;
		pRef->iFrameNum=-1;
		pRef->iFrameWrapNum=-1;
		pRef->iLongTermFrameIdx=-1;
		pRef->uiLongTermPicNum=0;
		pRef->uiQualityId=-1;
		pRef->uiTemporalId=-1;
		pRef->uiSpatialId=-1;
		pRef->iSpsId=-1;
		pRef->bIsComplete=false;
		pRef->iRefCount=0;
		if(pRef->eSliceType==I_SLICE){
			return;
		}
		int32_t lists=pRef->eSliceType==P_SLICE ? 1 : 2;
		for(int32_t i=0; i<MAX_DPB_COUNT;++i){
			for(int32_t list=0; list<lists;++list){
				if(pRef->pRefPic[list][i]!=NULL){
					pRef->pRefPic[list][i]->iRefCount=0;
					pRef->pRefPic[list][i]=NULL;
				}
			}
		}
	}
}

// reset pRefList when
// 1.sps arrived that is new sequence starting
// 2.IDR NAL i.e. 1st layer in IDR AU

void WelsResetRefPic(SDecoderContext* pCtx){
	int32_t i=0;
	SRefPic* pRefPic=&pCtx->sRefPic;
	pCtx->sRefPic.uiLongRefCount[LIST_0]=pCtx->sRefPic.uiShortRefCount[LIST_0]=0;
	pRefPic->uiRefCount[LIST_0]=0;
	pRefPic->uiRefCount[LIST_1]=0;
	for(i=0; i<MAX_DPB_COUNT; i++){
		if(pRefPic->pShortRefList[LIST_0][i]!=NULL){
			SetUnRef(pRefPic->pShortRefList[LIST_0][i]);
			pRefPic->pShortRefList[LIST_0][i]=NULL;
		}
	}
	pRefPic->uiShortRefCount[LIST_0]=0;

	for(i=0; i<MAX_DPB_COUNT; i++){
		if(pRefPic->pLongRefList[LIST_0][i]!=NULL){
			SetUnRef(pRefPic->pLongRefList[LIST_0][i]);
			pRefPic->pLongRefList[LIST_0][i]=NULL;
		}
	}
	pRefPic->uiLongRefCount[LIST_0]=0;
}

#define PADDING_LENGTH 32		// reference extension

void FreePicture(SPicture* pPic){
	if(NULL!=pPic){
		if(pPic->pBuffer[0]){
			WelsFree(pPic->pBuffer[0]);
			pPic->pBuffer[0]=NULL;
		}
		if(pPic->pMbCorrectlyDecodedFlag){
			WelsFree(pPic->pMbCorrectlyDecodedFlag);
			pPic->pMbCorrectlyDecodedFlag=NULL;
		}
		if(pPic->pNzc){
			WelsFree(pPic->pNzc);
			pPic->pNzc=NULL;
		}
		if(pPic->pMbType){
			WelsFree(pPic->pMbType);
			pPic->pMbType=NULL;
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(pPic->pMv[listIdx]){
				WelsFree(pPic->pMv[listIdx]);
				pPic->pMv[listIdx]=NULL;
			}
			if(pPic->pRefIndex[listIdx]){
				WelsFree(pPic->pRefIndex[listIdx]);
				pPic->pRefIndex[listIdx]=NULL;
			}
		}
		WelsFree(pPic);
		pPic=NULL;
	}
}


SPicture* AllocPicture(SDecoderContext* pCtx,const int32_t kiPicWidth,const int32_t kiPicHeight){
	SPicture* pPic=NULL;
	int32_t iPicWidth=0;
	int32_t iPicHeight=0;

	int32_t iPicChromaWidth=0;
	int32_t iPicChromaHeight=0;
	int32_t iLumaSize=0;
	int32_t iChromaSize=0;

	pPic=(SPicture*)WelsMallocz(sizeof(SPicture));
	WELS_VERIFY_RETURN_IF(NULL,NULL==pPic);

	memset(pPic,0,sizeof(SPicture));

	iPicWidth=WELS_ALIGN(kiPicWidth+(PADDING_LENGTH<<1),PICTURE_RESOLUTION_ALIGNMENT);
	iPicHeight=WELS_ALIGN(kiPicHeight+(PADDING_LENGTH<<1),PICTURE_RESOLUTION_ALIGNMENT);
	iPicChromaWidth=iPicWidth>>1;
	iPicChromaHeight=iPicHeight>>1;

	iLumaSize=iPicWidth*iPicHeight;
	iChromaSize=iPicChromaWidth*iPicChromaHeight;

	pPic->pBuffer[0]=static_cast<uint8_t*> (WelsMallocz(iLumaSize+(iChromaSize<<1)));
	WELS_VERIFY_RETURN_PROC_IF(NULL,NULL==pPic->pBuffer[0],FreePicture(pPic));

	memset(pPic->pBuffer[0],128,(iLumaSize+(iChromaSize<<1)));
	pPic->iLinesize[0]=iPicWidth;
	pPic->iLinesize[1]=pPic->iLinesize[2]=iPicChromaWidth;
	pPic->pBuffer[1]=pPic->pBuffer[0]+iLumaSize;
	pPic->pBuffer[2]=pPic->pBuffer[1]+iChromaSize;
	pPic->pData[0]=pPic->pBuffer[0]+(1+pPic->iLinesize[0])*PADDING_LENGTH;
	pPic->pData[1]=pPic->pBuffer[1]+(((1+pPic->iLinesize[1])*PADDING_LENGTH)>>1);
	pPic->pData[2]=pPic->pBuffer[2]+(((1+pPic->iLinesize[2])*PADDING_LENGTH)>>1);

	pPic->iPlanes=3;		// yv12 in default
	pPic->iWidthInPixel=kiPicWidth;
	pPic->iHeightInPixel=kiPicHeight;
	pPic->iFrameNum=-1;
	pPic->iRefCount=0;

	uint32_t uiMbWidth=(kiPicWidth+15)>>4;
	uint32_t uiMbHeight=(kiPicHeight+15)>>4;
	uint32_t uiMbCount=uiMbWidth*uiMbHeight;

	pPic->pMbCorrectlyDecodedFlag=(bool*)WelsMallocz(uiMbCount*sizeof(bool));
	pPic->pNzc=0;	// GetThreadCount (pCtx) > 1 ? (int8_t (*)[24])WelsMallocz (uiMbCount * 24,"pPic->pNzc") : NULL;
	pPic->pMbType=(uint32_t*)WelsMallocz(uiMbCount*sizeof(uint32_t));
	pPic->pMv[LIST_0]=(int16_t(*)[16][2])WelsMallocz(uiMbCount*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
	pPic->pMv[LIST_1]=(int16_t(*)[16][2])WelsMallocz(uiMbCount*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
	pPic->pRefIndex[LIST_0]=(int8_t(*)[16])WelsMallocz(uiMbCount*sizeof(int8_t)*MB_BLOCK4x4_NUM);
	pPic->pRefIndex[LIST_1]=(int8_t(*)[16])WelsMallocz(uiMbCount*sizeof(int8_t)*MB_BLOCK4x4_NUM);

	return pPic;
}

// get size of reference picture list in target layer incoming,=(iNumRefFrames
static inline int32_t GetTargetRefListSize(SDecoderContext* pCtx){
	int32_t iNumRefFrames=0;
	// +2 for EC MV Copy buffer exchange
	if((pCtx==NULL) || (pCtx->pSps==NULL)){
		iNumRefFrames=MAX_REF_PIC_COUNT+2;
	}else{
		iNumRefFrames=pCtx->pSps->iNumRefFrames+2;
	}

	// pic_queue size minimum set 2
	if(iNumRefFrames<2){
		iNumRefFrames=2;
	}

	return iNumRefFrames;
}

// reset picture reodering buffer list
void ResetReorderingPictureBuffers(SPictReoderingStatus* pPictReoderingStatus,SPictInfo* pPictInfo,const bool& fullReset){
	if(pPictReoderingStatus!=NULL && pPictInfo!=NULL){
		int32_t pictInfoListCount=fullReset ? 16 : (pPictReoderingStatus->iLargestBufferedPicIndex+1);
		pPictReoderingStatus->iPictInfoIndex=0;
		pPictReoderingStatus->iMinPOC=IMinInt32;
		pPictReoderingStatus->iNumOfPicts=0;
		pPictReoderingStatus->iLastGOPRemainPicts=0;
		pPictReoderingStatus->iLastWrittenPOC=IMinInt32;
		pPictReoderingStatus->iLargestBufferedPicIndex=0;
		for(int32_t i=0; i<pictInfoListCount;++i){
			pPictInfo[i].bLastGOP=false;
			pPictInfo[i].iPOC=IMinInt32;
		}
	}
}

void DestroyPicBuff(SDecoderContext* pCtx,SPicBuff** ppPicBuf){
	SPicBuff* pPicBuf=NULL;
	ResetReorderingPictureBuffers(pCtx->pPictReoderingStatus,pCtx->pPictInfoList,false);
	if(NULL==ppPicBuf || NULL==*ppPicBuf)
		return;
	pPicBuf=*ppPicBuf;
	while(pPicBuf->ppPic!=NULL){
		int32_t iPicIdx=0;
		while(iPicIdx<pPicBuf->iCapacity){
			SPicture* pPic=pPicBuf->ppPic[iPicIdx];
			if(pPic!=NULL){
				FreePicture(pPic);
			}
			pPic=NULL;
			++iPicIdx;
		}
		WelsFree(pPicBuf->ppPic);
		pPicBuf->ppPic=NULL;
	}
	pPicBuf->iCapacity=0;
	pPicBuf->iCurrentIdx=0;
	WelsFree(pPicBuf);
	pPicBuf=NULL;
	*ppPicBuf=NULL;
}

static int32_t CreatePicBuff(SDecoderContext* pCtx,SPicBuff** ppPicBuf,const int32_t kiSize,const int32_t kiPicWidth,const int32_t kiPicHeight){
	SPicBuff* pPicBuf=NULL;
	int32_t iPicIdx=0;
	if(kiSize<=0 || kiPicWidth<=0 || kiPicHeight<=0){
		return ERR_INFO_INVALID_PARAM;
	}
	pPicBuf=(SPicBuff*)WelsMallocz(sizeof(SPicBuff));
	if(NULL==pPicBuf){
		return ERR_INFO_OUT_OF_MEMORY;
	}
	pPicBuf->ppPic=(SPicture**)WelsMallocz(kiSize*sizeof(SPicture*));
	if(NULL==pPicBuf->ppPic){
		pPicBuf->iCapacity=0;
		DestroyPicBuff(pCtx,&pPicBuf);
		return ERR_INFO_OUT_OF_MEMORY;
	}
	for(iPicIdx=0; iPicIdx<kiSize;++iPicIdx){
		SPicture* pPic=AllocPicture(pCtx,kiPicWidth,kiPicHeight);
		if(NULL==pPic){
			// init capacity first for free memory
			pPicBuf->iCapacity=iPicIdx;
			DestroyPicBuff(pCtx,&pPicBuf);
			return ERR_INFO_OUT_OF_MEMORY;
		}
		pPicBuf->ppPic[iPicIdx]=pPic;
	}
	// initialize context in queue
	pPicBuf->iCapacity=kiSize;
	pPicBuf->iCurrentIdx=0;
	*ppPicBuf=pPicBuf;
	return ERR_NONE;
}

static int32_t IncreasePicBuff(SDecoderContext* pCtx,SPicBuff** ppPicBuf,const int32_t kiOldSize,const int32_t kiPicWidth,const int32_t kiPicHeight,const int32_t kiNewSize){
	SPicBuff* pPicOldBuf=*ppPicBuf;
	SPicBuff* pPicNewBuf=NULL;
	int32_t iPicIdx=0;
	if(kiOldSize<=0 || kiNewSize<=0 || kiPicWidth<=0 || kiPicHeight<=0){
		return ERR_INFO_INVALID_PARAM;
	}
	pPicNewBuf=(SPicBuff*)WelsMallocz(sizeof(SPicBuff));
	if(NULL==pPicNewBuf){
		return ERR_INFO_OUT_OF_MEMORY;
	}
	pPicNewBuf->ppPic=(SPicture**)WelsMallocz(kiNewSize*sizeof(SPicture*));
	if(NULL==pPicNewBuf->ppPic){
		pPicNewBuf->iCapacity=0;
		DestroyPicBuff(pCtx,&pPicNewBuf);
		return ERR_INFO_OUT_OF_MEMORY;
	}
	// increase new PicBuf
	for(iPicIdx=kiOldSize; iPicIdx<kiNewSize;++iPicIdx){
		SPicture* pPic=AllocPicture(pCtx,kiPicWidth,kiPicHeight);
		if(NULL==pPic){
			// Set maximum capacity as the new malloc memory at the tail
			pPicNewBuf->iCapacity=iPicIdx;
			DestroyPicBuff(pCtx,&pPicNewBuf);
			return ERR_INFO_OUT_OF_MEMORY;
		}
		pPicNewBuf->ppPic[iPicIdx]=pPic;
	}

	// copy old PicBuf to new PicBuf
	memcpy(pPicNewBuf->ppPic,pPicOldBuf->ppPic,kiOldSize*sizeof(SPicture*));

	// initialize context in queue
	pPicNewBuf->iCapacity=kiNewSize;
	pPicNewBuf->iCurrentIdx=pPicOldBuf->iCurrentIdx;
	*ppPicBuf=pPicNewBuf;

	for(int32_t i=0; i<pPicNewBuf->iCapacity; i++){
		pPicNewBuf->ppPic[i]->bUsedAsRef=false;
		pPicNewBuf->ppPic[i]->bIsLongRef=false;
		pPicNewBuf->ppPic[i]->iRefCount=0;
		pPicNewBuf->ppPic[i]->bIsComplete=false;
	}
	// remove old PicBuf
	if(pPicOldBuf->ppPic!=NULL){
		WelsFree(pPicOldBuf->ppPic);
		pPicOldBuf->ppPic=NULL;
	}
	pPicOldBuf->iCapacity=0;
	pPicOldBuf->iCurrentIdx=0;
	WelsFree(pPicOldBuf);
	pPicOldBuf=NULL;
	return ERR_NONE;
}

static int32_t DecreasePicBuff(SDecoderContext* pCtx,SPicBuff** ppPicBuf,const int32_t kiOldSize,const int32_t kiPicWidth,const int32_t kiPicHeight,const int32_t kiNewSize){
	SPicBuff* pPicOldBuf=*ppPicBuf;
	SPicBuff* pPicNewBuf=NULL;
	int32_t iPicIdx=0;
	if(kiOldSize<=0 || kiNewSize<=0 || kiPicWidth<=0 || kiPicHeight<=0){
		return ERR_INFO_INVALID_PARAM;
	}
	pPicNewBuf=(SPicBuff*)WelsMallocz(sizeof(SPicBuff));
	if(NULL==pPicNewBuf){
		return ERR_INFO_OUT_OF_MEMORY;
	}
	pPicNewBuf->ppPic=(SPicture**)WelsMallocz(kiNewSize*sizeof(SPicture*));
	if(NULL==pPicNewBuf->ppPic){
		pPicNewBuf->iCapacity=0;
		DestroyPicBuff(pCtx,&pPicNewBuf);
		return ERR_INFO_OUT_OF_MEMORY;
	}
	ResetReorderingPictureBuffers(pCtx->pPictReoderingStatus,pCtx->pPictInfoList,false);
	int32_t iPrevPicIdx=-1;
	for(iPrevPicIdx=0; iPrevPicIdx<kiOldSize;++iPrevPicIdx){
		if(pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb==pPicOldBuf->ppPic[iPrevPicIdx]){
			break;
		}
	}
	int32_t iDelIdx;
	if(iPrevPicIdx<kiOldSize && iPrevPicIdx>=kiNewSize){
		// found pPreviousDecodedPictureInDpb,
		pPicNewBuf->ppPic[0]=pPicOldBuf->ppPic[iPrevPicIdx];
		pPicNewBuf->iCurrentIdx=0;
		memcpy(pPicNewBuf->ppPic+1,pPicOldBuf->ppPic,(kiNewSize-1)*sizeof(SPicture*));
		iDelIdx=kiNewSize-1;
	}else{
		memcpy(pPicNewBuf->ppPic,pPicOldBuf->ppPic,kiNewSize*sizeof(SPicture*));
		pPicNewBuf->iCurrentIdx=iPrevPicIdx<kiNewSize ? iPrevPicIdx : 0;
		iDelIdx=kiNewSize;
	}
	// update references due to allocation changes
	// all references' references have to be reset oss-buzz 14423
	for(int32_t i=0; i<kiNewSize; i++){
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			int32_t j=-1;
			while(++j<MAX_DPB_COUNT && pPicNewBuf->ppPic[i]->pRefPic[listIdx][j]!=NULL){
				pPicNewBuf->ppPic[i]->pRefPic[listIdx][j]=NULL;
			}
		}
	}
	for(iPicIdx=iDelIdx; iPicIdx<kiOldSize; iPicIdx++){
		if(iPrevPicIdx!=iPicIdx){
			if(pPicOldBuf->ppPic[iPicIdx]!=NULL){
				FreePicture(pPicOldBuf->ppPic[iPicIdx]);
				pPicOldBuf->ppPic[iPicIdx]=NULL;
			}
		}
	}
	// initialize context in queue
	pPicNewBuf->iCapacity=kiNewSize;
	*ppPicBuf=pPicNewBuf;
	for(int32_t i=0; i<pPicNewBuf->iCapacity; i++){
		pPicNewBuf->ppPic[i]->bUsedAsRef=false;
		pPicNewBuf->ppPic[i]->bIsLongRef=false;
		pPicNewBuf->ppPic[i]->iRefCount=0;
		pPicNewBuf->ppPic[i]->bIsComplete=false;
	}
	// remove old PicBuf
	if(pPicOldBuf->ppPic!=NULL){
		WelsFree(pPicOldBuf->ppPic);
		pPicOldBuf->ppPic=NULL;
	}
	pPicOldBuf->iCapacity=0;
	pPicOldBuf->iCurrentIdx=0;
	WelsFree(pPicOldBuf);
	pPicOldBuf=NULL;
	return ERR_NONE;
}

// request memory blocks for decoder avc part
int32_t WelsRequestMem(SDecoderContext* pCtx,const int32_t kiMbWidth,const int32_t kiMbHeight,bool& bReallocFlag){
	const int32_t kiPicWidth=kiMbWidth<<4;
	const int32_t kiPicHeight=kiMbHeight<<4;
	int32_t iErr=ERR_NONE;

	int32_t iPicQueueSize=0;		// adaptive size of picture queue,=(pSps->iNumRefFrames x 2)
	bReallocFlag=false;
	bool bNeedChangePicQueue=true;
	WELS_VERIFY_RETURN_IF(ERR_INFO_INVALID_PARAM,(NULL==pCtx || kiPicWidth<=0 || kiPicHeight<=0))
		// Fixed the issue about different gop size over last,5/17/2010
		// get picture queue size currently
		iPicQueueSize=GetTargetRefListSize(pCtx);		// adaptive size of picture queue,=(pSps->iNumRefFrames x 2)
	pCtx->iPicQueueNumber=iPicQueueSize;
	if(pCtx->pPicBuff!=NULL && pCtx->pPicBuff->iCapacity==iPicQueueSize)		// comparing current picture queue size requested and previous allocation picture queue
		bNeedChangePicQueue=false;
	// HD based pic buffer need consider memory size consumed when switch from 720p to other lower size
	WELS_VERIFY_RETURN_IF(ERR_NONE,pCtx->bHaveGotMemory && (kiPicWidth==pCtx->iImgWidthInPixel && kiPicHeight==pCtx->iImgHeightInPixel) && (!bNeedChangePicQueue))		// have same scaled buffer
		// sync update pRefList
		WelsResetRefPic(pCtx);		// added to sync update ref list due to pictures are free

	if(pCtx->bHaveGotMemory && (kiPicWidth==pCtx->iImgWidthInPixel && kiPicHeight==pCtx->iImgHeightInPixel) && pCtx->pPicBuff!=NULL && pCtx->pPicBuff->iCapacity!=iPicQueueSize){
		// currently only active for LIST_0 due to have no B frames
		// Actually just need one memory allocation for the PicBuff. While it needs two pointer list (LIST_0 and LIST_1).
		uprintf("WelsRequestMem(): memory re-alloc for no resolution change (size=%d * %d),ref list size change from %d to %d",kiPicWidth,kiPicHeight,pCtx->pPicBuff->iCapacity,iPicQueueSize);
		if(pCtx->pPicBuff->iCapacity<iPicQueueSize){
			iErr=IncreasePicBuff(pCtx,&pCtx->pPicBuff,pCtx->pPicBuff->iCapacity,kiPicWidth,kiPicHeight,iPicQueueSize);
		}else{
			iErr=DecreasePicBuff(pCtx,&pCtx->pPicBuff,pCtx->pPicBuff->iCapacity,kiPicWidth,kiPicHeight,iPicQueueSize);
		}
	}else{
		if(pCtx->bHaveGotMemory)
			uprintf("WelsRequestMem(): memory re-alloc for resolution change,size change from %d * %d to %d * %d,ref list size change from %d to %d",pCtx->iImgWidthInPixel,pCtx->iImgHeightInPixel,kiPicWidth,kiPicHeight,pCtx->pPicBuff->iCapacity,iPicQueueSize);
		else
			uprintf("WelsRequestMem(): memory alloc size=%d * %d,ref list size=%d",kiPicWidth,kiPicHeight,iPicQueueSize);
		// for Recycled_Pic_Queue
		SPicBuff** ppPic=&pCtx->pPicBuff;
		if(NULL!=ppPic && NULL!=*ppPic){
			DestroyPicBuff(pCtx,ppPic);
		}
		pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb=NULL;
		// currently only active for LIST_0 due to have no B frames
		iErr=CreatePicBuff(pCtx,&pCtx->pPicBuff,iPicQueueSize,kiPicWidth,kiPicHeight);
	}
	if(iErr!=ERR_NONE)
		return iErr;

	pCtx->iImgWidthInPixel=kiPicWidth;					// target width of image to be reconstruted while decoding
	pCtx->iImgHeightInPixel=kiPicHeight;				// target height of image to be reconstruted while decoding

	pCtx->bHaveGotMemory=true;							// global memory for decoder context related is requested
	pCtx->pDec=NULL;									// need prefetch a new pic due to spatial size changed

	if(pCtx->pCabacDecEngine==NULL)
		pCtx->pCabacDecEngine=(SWelsCabacDecEngine*)WelsMallocz(sizeof(SWelsCabacDecEngine));
	WELS_VERIFY_RETURN_IF(ERR_INFO_OUT_OF_MEMORY,(NULL==pCtx->pCabacDecEngine))
	bReallocFlag=true;									// memory re-allocation successfully finished
	return ERR_NONE;
}

void UninitialDqLayersContext(SDecoderContext* pCtx){
	int32_t i=0;
	do{
		PDqLayer pDq=pCtx->pDqLayersList[i];
		if(pDq==NULL){
			++i;
			continue;
		}
		if(pCtx->sMb.pMbType[i]){
			WelsFree(pCtx->sMb.pMbType[i]);
			pCtx->sMb.pMbType[i]=NULL;
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(pCtx->sMb.pMv[i][listIdx]){
				WelsFree(pCtx->sMb.pMv[i][listIdx]);
				pCtx->sMb.pMv[i][listIdx]=NULL;
			}
			if(pCtx->sMb.pRefIndex[i][listIdx]){
				WelsFree(pCtx->sMb.pRefIndex[i][listIdx]);
				pCtx->sMb.pRefIndex[i][listIdx]=NULL;
			}
			if(pCtx->sMb.pDirect[i]){
				WelsFree(pCtx->sMb.pDirect[i]);
				pCtx->sMb.pDirect[i]=NULL;
			}
			if(pCtx->sMb.pMvd[i][listIdx]){
				WelsFree(pCtx->sMb.pMvd[i][listIdx]);
				pCtx->sMb.pMvd[i][listIdx]=NULL;
			}
		}
		if(pCtx->sMb.pNoSubMbPartSizeLessThan8x8Flag[i]){
			WelsFree(pCtx->sMb.pNoSubMbPartSizeLessThan8x8Flag[i]);
			pCtx->sMb.pNoSubMbPartSizeLessThan8x8Flag[i]=NULL;
		}
		if(pCtx->sMb.pTransformSize8x8Flag[i]){
			WelsFree(pCtx->sMb.pTransformSize8x8Flag[i]);
			pCtx->sMb.pTransformSize8x8Flag[i]=NULL;
		}
		if(pCtx->sMb.pLumaQp[i]){
			WelsFree(pCtx->sMb.pLumaQp[i]);
			pCtx->sMb.pLumaQp[i]=NULL;
		}
		if(pCtx->sMb.pChromaQp[i]){
			WelsFree(pCtx->sMb.pChromaQp[i]);
			pCtx->sMb.pChromaQp[i]=NULL;
		}
		if(pCtx->sMb.pCbfDc[i]){
			WelsFree(pCtx->sMb.pCbfDc[i]);
			pCtx->sMb.pCbfDc[i]=NULL;
		}
		if(pCtx->sMb.pNzc[i]){
			WelsFree(pCtx->sMb.pNzc[i]);
			pCtx->sMb.pNzc[i]=NULL;
		}
		if(pCtx->sMb.pNzcRs[i]){
			WelsFree(pCtx->sMb.pNzcRs[i]);
			pCtx->sMb.pNzcRs[i]=NULL;
		}
		if(pCtx->sMb.pScaledTCoeff[i]){
			WelsFree(pCtx->sMb.pScaledTCoeff[i]);
			pCtx->sMb.pScaledTCoeff[i]=NULL;
		}
		if(pCtx->sMb.pIntraPredMode[i]){
			WelsFree(pCtx->sMb.pIntraPredMode[i]);
			pCtx->sMb.pIntraPredMode[i]=NULL;
		}
		if(pCtx->sMb.pIntra4x4FinalMode[i]){
			WelsFree(pCtx->sMb.pIntra4x4FinalMode[i]);
			pCtx->sMb.pIntra4x4FinalMode[i]=NULL;
		}
		if(pCtx->sMb.pIntraNxNAvailFlag[i]){
			WelsFree(pCtx->sMb.pIntraNxNAvailFlag[i]);
			pCtx->sMb.pIntraNxNAvailFlag[i]=NULL;
		}
		if(pCtx->sMb.pChromaPredMode[i]){
			WelsFree(pCtx->sMb.pChromaPredMode[i]);
			pCtx->sMb.pChromaPredMode[i]=NULL;
		}
		if(pCtx->sMb.pCbp[i]){
			WelsFree(pCtx->sMb.pCbp[i]);
			pCtx->sMb.pCbp[i]=NULL;
		}
		if(pCtx->sMb.pSubMbType[i]){
			WelsFree(pCtx->sMb.pSubMbType[i]);
			pCtx->sMb.pSubMbType[i]=NULL;
		}
		if(pCtx->sMb.pSliceIdc[i]){
			WelsFree(pCtx->sMb.pSliceIdc[i]);
			pCtx->sMb.pSliceIdc[i]=NULL;
		}
		if(pCtx->sMb.pResidualPredFlag[i]){
			WelsFree(pCtx->sMb.pResidualPredFlag[i]);
			pCtx->sMb.pResidualPredFlag[i]=NULL;
		}
		if(pCtx->sMb.pInterPredictionDoneFlag[i]){
			WelsFree(pCtx->sMb.pInterPredictionDoneFlag[i]);
			pCtx->sMb.pInterPredictionDoneFlag[i]=NULL;
		}
		if(pCtx->sMb.pMbCorrectlyDecodedFlag[i]){
			WelsFree(pCtx->sMb.pMbCorrectlyDecodedFlag[i]);
			pCtx->sMb.pMbCorrectlyDecodedFlag[i]=NULL;
		}
		if(pCtx->sMb.pMbRefConcealedFlag[i]){
			WelsFree(pCtx->sMb.pMbRefConcealedFlag[i]);
			pCtx->sMb.pMbRefConcealedFlag[i]=NULL;
		}
		WelsFree(pDq);
		pDq=NULL;
		pCtx->pDqLayersList[i]=NULL;
		++i;
	} while(i<LAYER_NUM_EXCHANGEABLE);
	pCtx->iPicWidthReq=0;
	pCtx->iPicHeightReq=0;
	pCtx->bInitialDqLayersMem=false;
}

int32_t InitialDqLayersContext(SDecoderContext* pCtx,const int32_t kiMaxWidth,const int32_t kiMaxHeight){
	int32_t i=0;
	WELS_VERIFY_RETURN_IF(ERR_INFO_INVALID_PARAM,(NULL==pCtx || kiMaxWidth<=0 || kiMaxHeight<=0))
		pCtx->sMb.iMbWidth=(kiMaxWidth+15)>>4;
	pCtx->sMb.iMbHeight=(kiMaxHeight+15)>>4;

	if(pCtx->bInitialDqLayersMem && kiMaxWidth<=pCtx->iPicWidthReq
		 && kiMaxHeight<=pCtx->iPicHeightReq)		// have same dimension memory,skipped
		return ERR_NONE;
	UninitialDqLayersContext(pCtx);
	do{
		PDqLayer pDq=(PDqLayer)WelsMallocz(sizeof(SDqLayer));
		if(pDq==NULL)
			return ERR_INFO_OUT_OF_MEMORY;

		pCtx->pDqLayersList[i]=pDq;		// to keep consistence with in UninitialDqLayersContext()
		memset(pDq,0,sizeof(SDqLayer));
		pCtx->sMb.pMbType[i]=(uint32_t*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(uint32_t));
		pCtx->sMb.pMv[i][LIST_0]=(int16_t(*)[16][2])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
		pCtx->sMb.pMv[i][LIST_1]=(int16_t(*)[16][2])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
		pCtx->sMb.pRefIndex[i][LIST_0]=(int8_t(*)[MB_BLOCK4x4_NUM])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t)*MB_BLOCK4x4_NUM);
		pCtx->sMb.pRefIndex[i][LIST_1]=(int8_t(*)[MB_BLOCK4x4_NUM])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t)*MB_BLOCK4x4_NUM);
		pCtx->sMb.pDirect[i]=(int8_t(*)[MB_BLOCK4x4_NUM])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t)*MB_BLOCK4x4_NUM);
		pCtx->sMb.pLumaQp[i]=(int8_t*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t));
		pCtx->sMb.pNoSubMbPartSizeLessThan8x8Flag[i]=(bool*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(bool));
		pCtx->sMb.pTransformSize8x8Flag[i]=(bool*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(bool));
		pCtx->sMb.pChromaQp[i]=(int8_t(*)[2])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t)*2);
		pCtx->sMb.pMvd[i][LIST_0]=(int16_t(*)[16][2])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
		pCtx->sMb.pMvd[i][LIST_1]=(int16_t(*)[16][2])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int16_t)*MV_A*MB_BLOCK4x4_NUM);
		pCtx->sMb.pCbfDc[i]=(uint16_t*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(uint16_t));
		pCtx->sMb.pNzc[i]=(int8_t(*)[24])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t)*24);
		pCtx->sMb.pNzcRs[i]=(int8_t(*)[24])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t)*24);
		pCtx->sMb.pScaledTCoeff[i]=(int16_t(*)[MB_COEFF_LIST_SIZE])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int16_t)*MB_COEFF_LIST_SIZE);
		pCtx->sMb.pIntraPredMode[i]=(int8_t(*)[8])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t)*8);
		pCtx->sMb.pIntra4x4FinalMode[i]=(int8_t(*)[MB_BLOCK4x4_NUM])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t)*MB_BLOCK4x4_NUM);
		pCtx->sMb.pIntraNxNAvailFlag[i]=(uint8_t(*))WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t));
		pCtx->sMb.pChromaPredMode[i]=(int8_t*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t));
		pCtx->sMb.pCbp[i]=(int8_t*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t));
		pCtx->sMb.pSubMbType[i]=(uint32_t(*)[MB_PARTITION_SIZE])WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(uint32_t)*MB_PARTITION_SIZE);
		pCtx->sMb.pSliceIdc[i]=(int32_t*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int32_t));		// using int32_t for slice_idc,4/21/2010
		pCtx->sMb.pResidualPredFlag[i]=(int8_t*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t));
		pCtx->sMb.pInterPredictionDoneFlag[i]=(int8_t*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int8_t));
		pCtx->sMb.pMbCorrectlyDecodedFlag[i]=(bool*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(bool));
		pCtx->sMb.pMbRefConcealedFlag[i]=(bool*)WelsMallocz(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(bool));
		// check memory block valid due above allocated..
		WELS_VERIFY_RETURN_IF(ERR_INFO_OUT_OF_MEMORY,
							 ((NULL==pCtx->sMb.pMbType[i]) || 
								 (NULL==pCtx->sMb.pMv[i][LIST_0]) || 
								 (NULL==pCtx->sMb.pMv[i][LIST_1]) || 
								 (NULL==pCtx->sMb.pRefIndex[i][LIST_0]) || 
								 (NULL==pCtx->sMb.pRefIndex[i][LIST_1]) || 
								 (NULL==pCtx->sMb.pDirect[i]) || 
								 (NULL==pCtx->sMb.pLumaQp[i]) || 
								 (NULL==pCtx->sMb.pNoSubMbPartSizeLessThan8x8Flag[i]) || 
								 (NULL==pCtx->sMb.pTransformSize8x8Flag[i]) || 
								 (NULL==pCtx->sMb.pChromaQp[i]) || 
								 (NULL==pCtx->sMb.pMvd[i][LIST_0]) || 
								 (NULL==pCtx->sMb.pMvd[i][LIST_1]) || 
								 (NULL==pCtx->sMb.pCbfDc[i]) || 
								 (NULL==pCtx->sMb.pNzc[i]) || 
								 (NULL==pCtx->sMb.pNzcRs[i]) || 
								 (NULL==pCtx->sMb.pScaledTCoeff[i]) || 
								 (NULL==pCtx->sMb.pIntraPredMode[i]) || 
								 (NULL==pCtx->sMb.pIntra4x4FinalMode[i]) || 
								 (NULL==pCtx->sMb.pIntraNxNAvailFlag[i]) || 
								 (NULL==pCtx->sMb.pChromaPredMode[i]) || 
								 (NULL==pCtx->sMb.pCbp[i]) || 
								 (NULL==pCtx->sMb.pSubMbType[i]) || 
								 (NULL==pCtx->sMb.pSliceIdc[i]) || 
								 (NULL==pCtx->sMb.pResidualPredFlag[i]) || 
								 (NULL==pCtx->sMb.pInterPredictionDoneFlag[i]) || 
								 (NULL==pCtx->sMb.pMbRefConcealedFlag[i]) || 
								 (NULL==pCtx->sMb.pMbCorrectlyDecodedFlag[i])
								 )
		)
		memset(pCtx->sMb.pSliceIdc[i],0xff,(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int32_t)));
		++i;
	} while(i<LAYER_NUM_EXCHANGEABLE);
	pCtx->bInitialDqLayersMem=true;
	pCtx->iPicWidthReq=kiMaxWidth;
	pCtx->iPicHeightReq=kiMaxHeight;
	return ERR_NONE;
}

// brief make sure synchonozization picture resolution (get from slice header) among different parts (i.e,memory related and so on)
// over decoder internal
// MB coordinate and parts of data within decoder context structure )
// param pCtx Wels decoder context
// param iMbWidth MB width
// pram iMbHeight MB height
// return 0-successful; none 0-something wrong
int32_t SyncPictureResolutionExt(SDecoderContext* pCtx,const int32_t kiMbWidth,const int32_t kiMbHeight){
	int32_t iErr=ERR_NONE;
	const int32_t kiPicWidth=kiMbWidth<<4;
	const int32_t kiPicHeight=kiMbHeight<<4;
	// fix Bugzilla Bug1479656 reallocate temp dec picture
	if(pCtx->pTempDec!=NULL && (pCtx->pTempDec->iWidthInPixel!=kiPicWidth || pCtx->pTempDec->iHeightInPixel!=kiPicHeight)){
		FreePicture(pCtx->pTempDec);
		pCtx->pTempDec=AllocPicture(pCtx,pCtx->pSps->iMbWidth<<4,pCtx->pSps->iMbHeight<<4);
	}
	bool bReallocFlag=false;
	iErr=WelsRequestMem(pCtx,kiMbWidth,kiMbHeight,bReallocFlag);		// common memory used
	if(ERR_NONE!=iErr){
		FATAL("SyncPictureResolutionExt()::WelsRequestMem--buffer allocated failure.");
		pCtx->iErrorCode|=dsOutOfMemory;
		return iErr;
	}

	iErr=InitialDqLayersContext(pCtx,kiPicWidth,kiPicHeight);
	if(ERR_NONE!=iErr){
		FATAL("SyncPictureResolutionExt()::InitialDqLayersContext--buffer allocated failure.");
		pCtx->iErrorCode|=dsOutOfMemory;
	}
	return iErr;
}

// AllocPicBuffOnNewSeqBegin
// check and (re)allocate picture buffers on new sequence begin
// return:
// 0-success; otherwise returned error_no defined in error_no.h
int32_t AllocPicBuffOnNewSeqBegin(SDecoderContext* pCtx){
	// try to allocate or relocate DPB memory only when new sequence is coming.
	WelsResetRefPic(pCtx);		// clear ref pPic when IDR NAL
	int32_t iErr=SyncPictureResolutionExt(pCtx,pCtx->pSps->iMbWidth,pCtx->pSps->iMbHeight);
	if(ERR_NONE!=iErr){
		uprintf("sync picture resolution ext failed, the error is %d",iErr);
		return iErr;
	}

	return iErr;
}

int32_t DecodeCurrentAccessUnit(SDecoderContext* pCtx,uint8_t** ppDst,SBufferInfo* pDstInfo);
void WelsDecodeAccessUnitEnd(SDecoderContext* pCtx);


// InitConstructAccessUnit
// Init before constructing an access unit for given input bitstream,maybe partial NAL Unit,one or more Units are involved to
// joint a collective access unit.
// SBufferInfo: Buffer info
// return:
// 0-success; otherwise returned error_no defined in error_no.h
int32_t InitConstructAccessUnit(SDecoderContext* pCtx,SBufferInfo* pDstInfo){
	int32_t iErr=ERR_NONE;
	iErr=WelsDecodeInitAccessUnitStart(pCtx,pDstInfo);
	if(ERR_NONE!=iErr){
		return iErr;
	}
	if(pCtx->bNewSeqBegin){
		iErr=AllocPicBuffOnNewSeqBegin(pCtx);
		if(ERR_NONE!=iErr){
			return iErr;
		}
	}
	return iErr;
}
// ConstructAccessUnit
// construct an access unit for given input bitstream,maybe partial NAL Unit,one or more Units are involved to
// joint a collective access unit.
// buf: bitstream data buffer
// bit_len: size in bit length of data
// buf_len: size in byte length of data
// coded_au: mark an Access Unit decoding finished
// return:
// 0-success; otherwise returned error_no defined in error_no.h
int32_t ConstructAccessUnit(SDecoderContext* pCtx,uint8_t** ppDst,SBufferInfo* pDstInfo){
	int32_t iErr=ERR_NONE;
	iErr=InitConstructAccessUnit(pCtx,pDstInfo);
	if(ERR_NONE!=iErr){
		return iErr;
	}
	if(pCtx->pCabacDecEngine==NULL){
		pCtx->pCabacDecEngine=(SWelsCabacDecEngine*)WelsMallocz(sizeof(SWelsCabacDecEngine));
		WELS_VERIFY_RETURN_IF(ERR_INFO_OUT_OF_MEMORY,(NULL==pCtx->pCabacDecEngine))
	}
	iErr=DecodeCurrentAccessUnit(pCtx,ppDst,pDstInfo);
	WelsDecodeAccessUnitEnd(pCtx);
	if(ERR_NONE!=iErr){
		uprintf("returned error from decoding:[0x%x]",iErr);
		return iErr;
	}
	return ERR_NONE;
}


inline uint8_t GetTargetDqId(uint8_t uiTargetDqId,SDecodingParam* psParam){
	uint8_t uiRequiredDqId=psParam ? psParam->uiTargetDqLayer : (uint8_t)255;
	return WELS_MIN(uiTargetDqId,uiRequiredDqId);
}

void InitCurDqLayerData(SDecoderContext* pCtx,PDqLayer pCurDq){
	if(NULL!=pCtx && NULL!=pCurDq){
		pCurDq->pMbType=pCtx->sMb.pMbType[0];
		pCurDq->pSliceIdc=pCtx->sMb.pSliceIdc[0];
		pCurDq->pMv[LIST_0]=pCtx->sMb.pMv[0][LIST_0];
		pCurDq->pMv[LIST_1]=pCtx->sMb.pMv[0][LIST_1];
		pCurDq->pRefIndex[LIST_0]=pCtx->sMb.pRefIndex[0][LIST_0];
		pCurDq->pRefIndex[LIST_1]=pCtx->sMb.pRefIndex[0][LIST_1];
		pCurDq->pDirect=pCtx->sMb.pDirect[0];
		pCurDq->pNoSubMbPartSizeLessThan8x8Flag=pCtx->sMb.pNoSubMbPartSizeLessThan8x8Flag[0];
		pCurDq->pTransformSize8x8Flag=pCtx->sMb.pTransformSize8x8Flag[0];
		pCurDq->pLumaQp=pCtx->sMb.pLumaQp[0];
		pCurDq->pChromaQp=pCtx->sMb.pChromaQp[0];
		pCurDq->pMvd[LIST_0]=pCtx->sMb.pMvd[0][LIST_0];
		pCurDq->pMvd[LIST_1]=pCtx->sMb.pMvd[0][LIST_1];
		pCurDq->pCbfDc=pCtx->sMb.pCbfDc[0];
		pCurDq->pNzc=pCtx->sMb.pNzc[0];
		pCurDq->pNzcRs=pCtx->sMb.pNzcRs[0];
		pCurDq->pScaledTCoeff=pCtx->sMb.pScaledTCoeff[0];
		pCurDq->pIntraPredMode=pCtx->sMb.pIntraPredMode[0];
		pCurDq->pIntra4x4FinalMode=pCtx->sMb.pIntra4x4FinalMode[0];
		pCurDq->pIntraNxNAvailFlag=pCtx->sMb.pIntraNxNAvailFlag[0];
		pCurDq->pChromaPredMode=pCtx->sMb.pChromaPredMode[0];
		pCurDq->pCbp=pCtx->sMb.pCbp[0];
		pCurDq->pSubMbType=pCtx->sMb.pSubMbType[0];
		pCurDq->pInterPredictionDoneFlag=pCtx->sMb.pInterPredictionDoneFlag[0];
		pCurDq->pResidualPredFlag=pCtx->sMb.pResidualPredFlag[0];
		pCurDq->pMbCorrectlyDecodedFlag=pCtx->sMb.pMbCorrectlyDecodedFlag[0];
		pCurDq->pMbRefConcealedFlag=pCtx->sMb.pMbRefConcealedFlag[0];
	}
}

SPicture* PrefetchPic(SPicBuff* pPicBuf){
	int32_t iPicIdx=0;
	SPicture* pPic=NULL;
	if(pPicBuf->iCapacity==0){
		return NULL;
	}
	for(iPicIdx=pPicBuf->iCurrentIdx+1; iPicIdx<pPicBuf->iCapacity;++iPicIdx){
		if(pPicBuf->ppPic[iPicIdx]!=NULL && !pPicBuf->ppPic[iPicIdx]->bUsedAsRef
			 && pPicBuf->ppPic[iPicIdx]->iRefCount<=0){
			pPic=pPicBuf->ppPic[iPicIdx];
			break;
		}
	}
	if(pPic!=NULL){
		pPicBuf->iCurrentIdx=iPicIdx;
		pPic->iPicBuffIdx=iPicIdx;
		return pPic;
	}
	for(iPicIdx=0; iPicIdx<=pPicBuf->iCurrentIdx;++iPicIdx){
		if(pPicBuf->ppPic[iPicIdx]!=NULL && !pPicBuf->ppPic[iPicIdx]->bUsedAsRef
			 && pPicBuf->ppPic[iPicIdx]->iRefCount<=0){
			pPic=pPicBuf->ppPic[iPicIdx];
			break;
		}
	}

	pPicBuf->iCurrentIdx=iPicIdx;
	if(pPic!=NULL){
		pPic->iPicBuffIdx=iPicIdx;
	}
	return pPic;
}

const uint8_t g_kuiScan8[24]={		// [16+2*4]
	9,10,17,18,		// 1+1*8,2+1*8,1+2*8,2+2*8,
	11,12,19,20,		// 3+1*8,4+1*8,3+2*8,4+2*8,
	25,26,33,34,		// 1+3*8,2+3*8,1+4*8,2+4*8,
	27,28,35,36,		// 3+3*8,4+3*8,3+4*8,4+4*8,
	14,15,		// 6+1*8,7+1*8,
	22,23,		// 6+2*8,7+2*8,
	38,39,		// 6+4*8,7+4*8,
	46,47,		// 6+5*8,7+5*8,
};


void GetI4LumaIChromaAddrTable(int32_t* pBlockOffset,const int32_t kiYStride,const int32_t kiUVStride){
	int32_t* pOffset=pBlockOffset;
	int32_t i;
	const uint8_t kuiScan0=g_kuiScan8[0];
	for(i=0; i<16; i++){
		const uint32_t kuiA=g_kuiScan8[i]-kuiScan0;
		const uint32_t kuiX=kuiA&0x07;
		const uint32_t kuiY=kuiA>>3;
		pOffset[i]=(kuiX+kiYStride*kuiY)<<2;
	}
	for(i=0; i<4; i++){
		const uint32_t kuiA=g_kuiScan8[i]-kuiScan0;
		pOffset[16+i]=pOffset[20+i]=((kuiA&0x07)+(kiUVStride)*(kuiA>>3))<<2;
	}
}

inline bool CheckSliceNeedReconstruct(uint8_t uiLayerDqId,uint8_t uiTargetDqId){
	return (uiLayerDqId==uiTargetDqId);		// target layer
}

// \brief detect parameter sets are changed or not
// 
// \param pFmo fmo context
// \param kiCountNumMb (iMbWidth * iMbHeight) in Sps
// \param iSliceGroupType slice group type if fmo is exactly enabled
// \param iSliceGroupCount slice group count if fmo is exactly enabled
// 
// \return true-changed or not initialized yet; false-not change at all
bool FmoParamSetsChanged(SFmo* pFmo,const int32_t kiCountNumMb,const int32_t kiSliceGroupType,const int32_t kiSliceGroupCount){
	WELS_VERIFY_RETURN_IF(false,(NULL==pFmo))
		return ((!pFmo->bActiveFlag) || (kiCountNumMb!=pFmo->iCountMbNum) || (kiSliceGroupType!=pFmo->iSliceGroupType) || (kiSliceGroupCount!=pFmo->iSliceGroupCount));
}

// \brief Generate MB allocated map for interleaved slice group (TYPE 0)
// \param pFmo fmo context
// \param pPps pps context
// \return 0-successful; none 0-failed
static inline int32_t FmoGenerateMbAllocMapType0(SFmo* pFmo,const SPps* pPps){
	uint32_t uiNumSliceGroups=0;
	int32_t iMbNum=0;
	int32_t i=0;

	WELS_VERIFY_RETURN_IF(ERR_INFO_INVALID_PARAM,(NULL==pFmo || NULL==pPps))
		uiNumSliceGroups=pPps->uiNumSliceGroups;
	iMbNum=pFmo->iCountMbNum;
	WELS_VERIFY_RETURN_IF(ERR_INFO_INVALID_PARAM,(NULL==pFmo->pMbAllocMap || iMbNum<=0 || uiNumSliceGroups>MAX_SLICEGROUP_IDS))
		do{
			uint8_t uiGroup=0;
			do{
				const int32_t kiRunIdx=pPps->uiRunLength[uiGroup];
				int32_t j=0;
				do{
					pFmo->pMbAllocMap[i+j]=uiGroup;
					++j;
				} while(j<kiRunIdx && i+j<iMbNum);
				i+=kiRunIdx;
				++uiGroup;
			} while(uiGroup<uiNumSliceGroups && i<iMbNum);
		} while(i<iMbNum);

		return ERR_NONE;		// well here
}

// brief Generate MB allocated map for dispersed slice group (TYPE 1)
// param pFmo fmo context
// param pPps pps context
// param iMbWidth MB width
// return 0-successful; none 0-failed
static inline int32_t FmoGenerateMbAllocMapType1(SFmo* pFmo,const SPps* pPps,const int32_t kiMbWidth){
	uint32_t uiNumSliceGroups=0;
	int32_t iMbNum=0;
	int32_t i=0;
	WELS_VERIFY_RETURN_IF(ERR_INFO_INVALID_PARAM,(NULL==pFmo || NULL==pPps))
		uiNumSliceGroups=pPps->uiNumSliceGroups;
	iMbNum=pFmo->iCountMbNum;
	WELS_VERIFY_RETURN_IF(ERR_INFO_INVALID_PARAM,(NULL==pFmo->pMbAllocMap || iMbNum<=0 || kiMbWidth==0
		 || uiNumSliceGroups>MAX_SLICEGROUP_IDS))

		do{
			pFmo->pMbAllocMap[i]=(uint8_t)(((i%kiMbWidth)+(((i/kiMbWidth)*uiNumSliceGroups)>>1))%uiNumSliceGroups);
			++i;
		} while(i<iMbNum);

		return ERR_NONE;		// well here
}

// brief Generate MB allocated map for various type of slice group cases (TYPE 0,..,6)
// param pFmo fmo context
// param pPps pps context
// param kiMbWidth MB width
// param kiMbHeight MB height
// return 0-successful; none 0-failed
static inline int32_t FmoGenerateSliceGroup(SFmo* pFmo,const SPps* kpPps,const int32_t kiMbWidth,const int32_t kiMbHeight){
	int32_t iNumMb=0;
	int32_t iErr=0;
	bool bResolutionChanged=false;

	// the cases we would not like
	WELS_VERIFY_RETURN_IF(ERR_INFO_INVALID_PARAM,(NULL==pFmo || NULL==kpPps))
		iNumMb=kiMbWidth*kiMbHeight;

	if(0==iNumMb)
		return ERR_INFO_INVALID_PARAM;

	WelsFree(pFmo->pMbAllocMap);
	pFmo->pMbAllocMap=(uint8_t*)WelsMallocz(iNumMb*sizeof(uint8_t));
	WELS_VERIFY_RETURN_IF(ERR_INFO_OUT_OF_MEMORY,(NULL==pFmo->pMbAllocMap))		// out of memory

		pFmo->iCountMbNum=iNumMb;

	if(kpPps->uiNumSliceGroups<2 && iNumMb > 0){		// only one slice group,exactly it is single slice based
		memset(pFmo->pMbAllocMap,0,iNumMb*sizeof(int8_t));		// for safe

		pFmo->iSliceGroupCount=1;

		return ERR_NONE;
	}

	if(bResolutionChanged || ((int32_t)kpPps->uiSliceGroupMapType!=pFmo->iSliceGroupType)
		 || ((int32_t)kpPps->uiNumSliceGroups!=pFmo->iSliceGroupCount)){
		switch(kpPps->uiSliceGroupMapType){
			case 0:
				iErr=FmoGenerateMbAllocMapType0(pFmo,kpPps);
				break;
			case 1:
				iErr=FmoGenerateMbAllocMapType1(pFmo,kpPps,kiMbWidth);
				break;
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
				// Reserve for others slice group type
				iErr=1;
				break;
			default:
				return ERR_INFO_UNSUPPORTED_FMOTYPE;
		}
	}

	if(0==iErr){		// well now
		pFmo->iSliceGroupCount=kpPps->uiNumSliceGroups;
		pFmo->iSliceGroupType=kpPps->uiSliceGroupMapType;
	}

	return iErr;
}

// brief Initialize Wels Flexible Macroblock Ordering (FMO)
// param pFmo Wels fmo to be initialized
// param pPps pps argument
// param kiMbWidth mb width
// param kiMbHeight mb height
// return 0-successful; none 0-failed;
int32_t InitFmo(SFmo* pFmo,SPps* pPps,const int32_t kiMbWidth,const int32_t kiMbHeight){
	return FmoGenerateSliceGroup(pFmo,pPps,kiMbWidth,kiMbHeight);
}

// brief update/insert FMO parameter unit
// param _fmo FMO context
// param _sps SSps*
// param _pps SPps*
// param pActiveFmoNum int32_t* [in/out]
// return true-update/insert successfully; false-failed;
int32_t FmoParamUpdate(SFmo* pFmo,SSps* pSps,SPps* pPps,int32_t* pActiveFmoNum){
	const uint32_t kuiMbWidth=pSps->iMbWidth;
	const uint32_t kuiMbHeight=pSps->iMbHeight;
	int32_t iRet=ERR_NONE;
	if(FmoParamSetsChanged(pFmo,kuiMbWidth*kuiMbHeight,pPps->uiSliceGroupMapType,pPps->uiNumSliceGroups)){
		iRet=InitFmo(pFmo,pPps,kuiMbWidth,kuiMbHeight);
		WELS_VERIFY_RETURN_IF(iRet,iRet);

		if(!pFmo->bActiveFlag && *pActiveFmoNum<MAX_PPS_COUNT){
			++(*pActiveFmoNum);
			pFmo->bActiveFlag=true;
		}
	}
	return iRet;
}

void WelsDqLayerDecodeStart(SDecoderContext* pCtx,SNalUnit* pCurNal,SSps* pSps,SPps* pPps){
	SSliceHeader* pSh=&pCurNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader;

	pCtx->eSliceType=pSh->eSliceType;
	pCtx->pSliceHeader=pSh;
	pCtx->bUsedAsRef=false;

	pCtx->iFrameNum=pSh->iFrameNum;
	// UpdateDecoderStatisticsForActiveParaset (pCtx->pDecoderStatistics,pSps,pPps);
}

static inline void InitDqLayerInfo(PDqLayer pDqLayer,SLayerInfo* pLayerInfo,SNalUnit* pNalUnit,SPicture* pPicDec){
	SNalUnitHeaderExt* pNalHdrExt=&pNalUnit->sNalHeaderExt;
	SSliceHeaderExt* pShExt=&pNalUnit->sNalData.sVclNal.sSliceHeaderExt;
	SSliceHeader* pSh=&pShExt->sSliceHeader;
	const uint8_t kuiQualityId=pNalHdrExt->uiQualityId;

	memcpy(&pDqLayer->sLayerInfo,pLayerInfo,sizeof(SLayerInfo));

	pDqLayer->pDec=pPicDec;
	pDqLayer->iMbWidth=pSh->iMbWidth;				// MB width of this picture
	pDqLayer->iMbHeight=pSh->iMbHeight;				// MB height of this picture

	pDqLayer->iSliceIdcBackup=(pSh->iFirstMbInSlice<<7)|(pNalHdrExt->uiDependencyId<<4)|(pNalHdrExt->uiQualityId);

	// Common syntax elements across all slices of a DQLayer
	pDqLayer->uiPpsId=pLayerInfo->pPps->iPpsId;
	pDqLayer->uiDisableInterLayerDeblockingFilterIdc=pShExt->uiDisableInterLayerDeblockingFilterIdc;
	pDqLayer->iInterLayerSliceAlphaC0Offset=pShExt->iInterLayerSliceAlphaC0Offset;
	pDqLayer->iInterLayerSliceBetaOffset=pShExt->iInterLayerSliceBetaOffset;
	pDqLayer->iSliceGroupChangeCycle=pSh->iSliceGroupChangeCycle;
	pDqLayer->bStoreRefBasePicFlag=pShExt->bStoreRefBasePicFlag;
	pDqLayer->bTCoeffLevelPredFlag=pShExt->bTCoeffLevelPredFlag;
	pDqLayer->bConstrainedIntraResamplingFlag=pShExt->bConstrainedIntraResamplingFlag;
	pDqLayer->uiRefLayerDqId=pShExt->uiRefLayerDqId;
	pDqLayer->uiRefLayerChromaPhaseXPlus1Flag=pShExt->uiRefLayerChromaPhaseXPlus1Flag;
	pDqLayer->uiRefLayerChromaPhaseYPlus1=pShExt->uiRefLayerChromaPhaseYPlus1;
	pDqLayer->bUseWeightPredictionFlag=false;
	pDqLayer->bUseWeightedBiPredIdc=false;

	if(kuiQualityId==BASE_QUALITY_ID){
		pDqLayer->pRefPicListReordering=&pSh->pRefPicListReordering;
		pDqLayer->pRefPicMarking=&pSh->sRefMarking;

		pDqLayer->bUseWeightPredictionFlag=pSh->pPps->bWeightedPredFlag;
		pDqLayer->bUseWeightedBiPredIdc=pSh->pPps->uiWeightedBipredIdc!=0;
		if(pSh->pPps->bWeightedPredFlag || pSh->pPps->uiWeightedBipredIdc){
			pDqLayer->pPredWeightTable=&pSh->sPredWeightTable;
		}
		pDqLayer->pRefPicBaseMarking=&pShExt->sRefBasePicMarking;
	}

	pDqLayer->uiLayerDqId=pNalHdrExt->uiLayerDqId;		// dq_id of current layer
	pDqLayer->bUseRefBasePicFlag=pNalHdrExt->bUseRefBasePicFlag;
}

static inline void ExpandPictureChroma_c(uint8_t* pDst,const int32_t kiStride,const int32_t kiPicW,const int32_t kiPicH){
	uint8_t* pTmp=pDst;
	uint8_t* pDstLastLine=pTmp+(kiPicH-1)*kiStride;
	const int32_t kiPaddingLen=(PADDING_LENGTH>>1);
	const uint8_t kuiTL=pTmp[0];
	const uint8_t kuiTR=pTmp[kiPicW-1];
	const uint8_t kuiBL=pDstLastLine[0];
	const uint8_t kuiBR=pDstLastLine[kiPicW-1];
	int32_t i=0;

	do{
		const int32_t kiStrides=(1+i)*kiStride;
		uint8_t* pTop=pTmp-kiStrides;
		uint8_t* pBottom=pDstLastLine+kiStrides;

		// pad pTop and pBottom
		memcpy(pTop,pTmp,kiPicW); 	// confirmed_safe_unsafe_usage
		memcpy(pBottom,pDstLastLine,kiPicW);

		// pad corners
		memset(pTop-kiPaddingLen,kuiTL,kiPaddingLen);		// pTop left
		memset(pTop+kiPicW,kuiTR,kiPaddingLen);				// pTop right
		memset(pBottom-kiPaddingLen,kuiBL,kiPaddingLen);	// pBottom left
		memset(pBottom+kiPicW,kuiBR,kiPaddingLen);			// pBottom right

		++i;
	} while(i<kiPaddingLen);

	// pad left and right
	i=0;
	do{
		memset(pTmp-kiPaddingLen,pTmp[0],kiPaddingLen);
		memset(pTmp+kiPicW,pTmp[kiPicW-1],kiPaddingLen);

		pTmp+=kiStride;
		++i;
	} while(i<kiPicH);
}

// void ExpandReferencingPicture (SPicture* pPic,PExpandPictureFunc pExpLuma,PExpandPictureFunc pExpChrom[2]) {
void ExpandReferencingPicture(uint8_t* pData[3],int32_t iWidth,int32_t iHeight,int32_t iStride[3],PExpandPictureFunc pExpLuma,PExpandPictureFunc pExpChrom[2]){
	// local variable
	uint8_t* pPicY=pData[0];
	uint8_t* pPicCb=pData[1];
	uint8_t* pPicCr=pData[2];
	const int32_t kiWidthY=iWidth;
	const int32_t kiHeightY=iHeight;
	const int32_t kiWidthUV=kiWidthY>>1;
	const int32_t kiHeightUV=kiHeightY>>1;
	pExpLuma(pPicY,iStride[0],kiWidthY,kiHeightY);
	if(kiWidthUV>=16){
		// fix coding picture size as 16x16
		const bool kbChrAligned=((kiWidthUV&0x0F)==0);		// chroma planes: (16+iWidthUV) & 15
		pExpChrom[kbChrAligned](pPicCb,iStride[1],kiWidthUV,kiHeightUV);
		pExpChrom[kbChrAligned](pPicCr,iStride[2],kiWidthUV,kiHeightUV);
	}else{
		// fix coding picture size as 16x16
		ExpandPictureChroma_c(pPicCb,iStride[1],kiWidthUV,kiHeightUV);
		ExpandPictureChroma_c(pPicCr,iStride[2],kiWidthUV,kiHeightUV);
	}
}


static int32_t AddShortTermToList(SRefPic* pRefPic,SPicture* pPic){
	pPic->bUsedAsRef=true;
	pPic->bIsLongRef=false;
	pPic->iLongTermFrameIdx=-1;
	if(pRefPic->uiShortRefCount[LIST_0]>0){
		// Check the duplicate frame_num in short ref list
		for(int32_t iPos=0; iPos<pRefPic->uiShortRefCount[LIST_0]; iPos++){
			if(!pRefPic->pShortRefList[LIST_0][iPos]){
				return ERR_INFO_INVALID_PTR;
			}
			if(pPic->iFrameNum==pRefPic->pShortRefList[LIST_0][iPos]->iFrameNum){
				// Replace the previous ref pic with the new one with the same frame_num
				pRefPic->pShortRefList[LIST_0][iPos]=pPic;
				return ERR_INFO_DUPLICATE_FRAME_NUM;
			}
		}

		memmove(&pRefPic->pShortRefList[LIST_0][1],&pRefPic->pShortRefList[LIST_0][0],pRefPic->uiShortRefCount[LIST_0]*sizeof(SPicture*));	// confirmed_safe_unsafe_usage
	}
	pRefPic->pShortRefList[LIST_0][0]=pPic;
	pRefPic->uiShortRefCount[LIST_0]++;
	return ERR_NONE;
}

static int32_t WelsCheckAndRecoverForFutureDecoding(SDecoderContext* pCtx){
	if((pCtx->sRefPic.uiShortRefCount[LIST_0]+pCtx->sRefPic.uiLongRefCount[LIST_0]<=0)
		 && (pCtx->eSliceType!=I_SLICE
			 && pCtx->eSliceType!=SI_SLICE)){
		if(pCtx->pParam->eEcActiveIdc!=
			ERROR_CON_DISABLE){		// IDR lost!,recover it for future decoding with data all set to 0
			SPicture* pRef=PrefetchPic(pCtx->pPicBuff);
			if(pRef!=NULL){
				// IDR lost,set new
				pRef->bIsComplete=false;		// Set complete flag to false for lost IDR ref picture
				pRef->iSpsId=pCtx->pSps->iSpsId;
				pRef->iPpsId=pCtx->pPps->iPpsId;
				if(pCtx->eSliceType==B_SLICE){
					// reset reference's references when IDR is lost
					for(int32_t list=LIST_0; list<LIST_A;++list){
						for(int32_t i=0; i<MAX_DPB_COUNT;++i){
							pRef->pRefPic[list][i]=NULL;
						}
					}
				}
				pCtx->iErrorCode|=dsDataErrorConcealed;
				bool bCopyPrevious=((ERROR_CON_FRAME_COPY_CROSS_IDR==pCtx->pParam->eEcActiveIdc)
									  || (ERROR_CON_SLICE_COPY_CROSS_IDR==pCtx->pParam->eEcActiveIdc)
									  || (ERROR_CON_SLICE_COPY_CROSS_IDR_FREEZE_RES_CHANGE==pCtx->pParam->eEcActiveIdc)
									  || (ERROR_CON_SLICE_MV_COPY_CROSS_IDR==pCtx->pParam->eEcActiveIdc)
									  || (ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE==pCtx->pParam->eEcActiveIdc))
					 && (NULL!=pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb);
				bCopyPrevious=bCopyPrevious
					 && (pRef->iWidthInPixel==pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb->iWidthInPixel)
					 && (pRef->iHeightInPixel==pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb->iHeightInPixel);

				if(!bCopyPrevious){
					memset(pRef->pData[0],128,pRef->iLinesize[0]*pRef->iHeightInPixel);
					memset(pRef->pData[1],128,pRef->iLinesize[1]*pRef->iHeightInPixel/2);
					memset(pRef->pData[2],128,pRef->iLinesize[2]*pRef->iHeightInPixel/2);
				}else
				if(pRef==pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb){
					uprintf("WelsInitRefList()::EC memcpy overlap.");
				}else{
					memcpy(pRef->pData[0],pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb->pData[0],pRef->iLinesize[0]*pRef->iHeightInPixel);
					memcpy(pRef->pData[1],pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb->pData[1],pRef->iLinesize[1]*pRef->iHeightInPixel/2);
					memcpy(pRef->pData[2],pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb->pData[2],pRef->iLinesize[2]*pRef->iHeightInPixel/2);
				}
				pRef->iFrameNum=0;
				pRef->iFramePoc=0;
				pRef->uiTemporalId=pRef->uiQualityId=0;
				pRef->eSliceType=pCtx->eSliceType;
				ExpandReferencingPicture(pRef->pData,pRef->iWidthInPixel,pRef->iHeightInPixel,pRef->iLinesize,pCtx->sExpandPicFunc.pfExpandLumaPicture,pCtx->sExpandPicFunc.pfExpandChromaPicture);
				AddShortTermToList(&pCtx->sRefPic,pRef);
			}else{
				FATAL("WelsInitRefList()::PrefetchPic for EC errors.");
				pCtx->iErrorCode|=dsOutOfMemory;
				return ERR_INFO_REF_COUNT_OVERFLOW;
			}
		}
	}
	return ERR_NONE;
}

static void WrapShortRefPicNum(SDecoderContext* pCtx){
	int32_t i;
	SSliceHeader* pSliceHeader=&pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader;
	int32_t iMaxPicNum=1<<pSliceHeader->pSps->uiLog2MaxFrameNum;
	SPicture** ppShoreRefList=pCtx->sRefPic.pShortRefList[LIST_0];
	int32_t iShortRefCount=pCtx->sRefPic.uiShortRefCount[LIST_0];
	// wrap pic num
	for(i=0; i<iShortRefCount; i++){
		if(ppShoreRefList[i]){
			if(ppShoreRefList[i]->iFrameNum>pSliceHeader->iFrameNum)
				ppShoreRefList[i]->iFrameWrapNum=ppShoreRefList[i]->iFrameNum-iMaxPicNum;
			else
				ppShoreRefList[i]->iFrameWrapNum=ppShoreRefList[i]->iFrameNum;
		}
	}
}


// fills the pRefPic.pRefList LIST_0 and LIST_0 for B-Slice.
int32_t WelsInitBSliceRefList(SDecoderContext* pCtx,int32_t iPoc){

	int32_t err=WelsCheckAndRecoverForFutureDecoding(pCtx);
	if(err!=ERR_NONE) return err;

	WrapShortRefPicNum(pCtx);

	SPicture** ppShoreRefList=pCtx->sRefPic.pShortRefList[LIST_0];
	SPicture** ppLongRefList=pCtx->sRefPic.pLongRefList[LIST_0];
	memset(pCtx->sRefPic.pRefList[LIST_0],0,MAX_DPB_COUNT*sizeof(SPicture*));
	memset(pCtx->sRefPic.pRefList[LIST_1],0,MAX_DPB_COUNT*sizeof(SPicture*));
	int32_t iLSCurrPocCount=0;
	int32_t iLTCurrPocCount=0;
	SPicture* pLSCurrPocList0[MAX_DPB_COUNT];
	SPicture* pLTCurrPocList0[MAX_DPB_COUNT];
	for(int32_t i=0; i<pCtx->sRefPic.uiShortRefCount[LIST_0];++i){
		if(ppShoreRefList[i]->iFramePoc<iPoc){
			pLSCurrPocList0[iLSCurrPocCount++]=ppShoreRefList[i];
		}
	}
	for(int32_t i=pCtx->sRefPic.uiShortRefCount[LIST_0]-1; i>=0;--i){
		if(ppShoreRefList[i]->iFramePoc>iPoc){
			pLTCurrPocList0[iLTCurrPocCount++]=ppShoreRefList[i];
		}
	}
	if(pCtx->sRefPic.uiLongRefCount[LIST_0]>1){
		// long sorts in increasing order
		SPicture* pTemp;
		for(int32_t i=0; i<pCtx->sRefPic.uiLongRefCount[LIST_0];++i){
			for(int32_t j=i+1; j<pCtx->sRefPic.uiLongRefCount[LIST_0];++j){
				if(ppLongRefList[j]->iFramePoc<ppLongRefList[i]->iFramePoc){
					pTemp=ppLongRefList[i];
					ppLongRefList[i]=ppLongRefList[j];
					ppLongRefList[j]=pTemp;
				}
			}
		}
	}
	int32_t iCurrPocCount=iLSCurrPocCount+iLTCurrPocCount;
	int32_t iCount=0;
	// LIST_0
	// short
	// It may need to sort LIST_0 and LIST_1 so that they will have the right default orders.
	for(int32_t i=0; i<iLSCurrPocCount;++i){
		pCtx->sRefPic.pRefList[LIST_0][iCount++]=pLSCurrPocList0[i];
	}
	if(iLSCurrPocCount>1){
		// LIST_0 short sorts in decreasing order
		SPicture* pTemp;
		for(int32_t i=0; i<iLSCurrPocCount;++i){
			for(int32_t j=i+1; j<iLSCurrPocCount;++j){
				if(pCtx->sRefPic.pRefList[LIST_0][j]->iFramePoc>pCtx->sRefPic.pRefList[LIST_0][i]->iFramePoc){
					pTemp=pCtx->sRefPic.pRefList[LIST_0][i];
					pCtx->sRefPic.pRefList[LIST_0][i]=pCtx->sRefPic.pRefList[LIST_0][j];
					pCtx->sRefPic.pRefList[LIST_0][j]=pTemp;
				}
			}
		}
	}
	for(int32_t i=0; i<iLTCurrPocCount;++i){
		pCtx->sRefPic.pRefList[LIST_0][iCount++]=pLTCurrPocList0[i];
	}
	if(iLTCurrPocCount>1){
		// LIST_0 short sorts in increasing order
		SPicture* pTemp;
		for(int32_t i=iLSCurrPocCount; i<iCurrPocCount;++i){
			for(int32_t j=i+1; j<iCurrPocCount;++j){
				if(pCtx->sRefPic.pRefList[LIST_0][j]->iFramePoc<pCtx->sRefPic.pRefList[LIST_0][i]->iFramePoc){
					pTemp=pCtx->sRefPic.pRefList[LIST_0][i];
					pCtx->sRefPic.pRefList[LIST_0][i]=pCtx->sRefPic.pRefList[LIST_0][j];
					pCtx->sRefPic.pRefList[LIST_0][j]=pTemp;
				}
			}
		}
	}
	// long
	for(int32_t i=0; i<pCtx->sRefPic.uiLongRefCount[LIST_0];++i){
		pCtx->sRefPic.pRefList[LIST_0][iCount++]=ppLongRefList[i];
	}
	pCtx->sRefPic.uiRefCount[LIST_0]=iCount;

	iCount=0;
	// LIST_1
	// short
	for(int32_t i=0; i<iLTCurrPocCount;++i){
		pCtx->sRefPic.pRefList[LIST_1][iCount++]=pLTCurrPocList0[i];
	}
	if(iLTCurrPocCount>1){
		// LIST_1 short sorts in increasing order
		SPicture* pTemp;
		for(int32_t i=0; i<iLTCurrPocCount;++i){
			for(int32_t j=i+1; j<iLTCurrPocCount;++j){
				if(pCtx->sRefPic.pRefList[LIST_1][j]->iFramePoc<pCtx->sRefPic.pRefList[LIST_1][i]->iFramePoc){
					pTemp=pCtx->sRefPic.pRefList[LIST_1][i];
					pCtx->sRefPic.pRefList[LIST_1][i]=pCtx->sRefPic.pRefList[LIST_1][j];
					pCtx->sRefPic.pRefList[LIST_1][j]=pTemp;
				}
			}
		}
	}
	for(int32_t i=0; i<iLSCurrPocCount;++i){
		pCtx->sRefPic.pRefList[LIST_1][iCount++]=pLSCurrPocList0[i];
	}
	if(iLSCurrPocCount>1){
		// LIST_1 short sorts in decreasing order
		SPicture* pTemp;
		for(int32_t i=iLTCurrPocCount; i<iCurrPocCount;++i){
			for(int32_t j=i+1; j<iCurrPocCount;++j){
				if(pCtx->sRefPic.pRefList[LIST_1][j]->iFramePoc>pCtx->sRefPic.pRefList[LIST_1][i]->iFramePoc){
					pTemp=pCtx->sRefPic.pRefList[LIST_1][i];
					pCtx->sRefPic.pRefList[LIST_1][i]=pCtx->sRefPic.pRefList[LIST_1][j];
					pCtx->sRefPic.pRefList[LIST_1][j]=pTemp;
				}
			}
		}
	}
	// long
	for(int32_t i=0; i<pCtx->sRefPic.uiLongRefCount[LIST_0];++i){
		pCtx->sRefPic.pRefList[LIST_1][iCount++]=ppLongRefList[i];
	}
	pCtx->sRefPic.uiRefCount[LIST_1]=iCount;
	return ERR_NONE;
}

void CreateImplicitWeightTable(SDecoderContext* pCtx){

	SSlice* pSlice=&pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	if(pCurDqLayer->bUseWeightedBiPredIdc && pSliceHeader->pPps->uiWeightedBipredIdc==2){
		int32_t iPoc=pSliceHeader->iPicOrderCntLsb;

		// fix Bugzilla 1485229 check if pointers are NULL
		if(pCtx->sRefPic.pRefList[LIST_0][0] && pCtx->sRefPic.pRefList[LIST_1][0]){
			if(pSliceHeader->uiRefCount[0]==1 && pSliceHeader->uiRefCount[1]==1
				 && pCtx->sRefPic.pRefList[LIST_0][0]->iFramePoc+pCtx->sRefPic.pRefList[LIST_1][0]->iFramePoc==2*iPoc){
				pCurDqLayer->bUseWeightedBiPredIdc=false;
				return;
			}
		}

		pCurDqLayer->pPredWeightTable->uiLumaLog2WeightDenom=5;
		pCurDqLayer->pPredWeightTable->uiChromaLog2WeightDenom=5;
		for(int32_t iRef0=0; iRef0<pSliceHeader->uiRefCount[0]; iRef0++){
			if(pCtx->sRefPic.pRefList[LIST_0][iRef0]){
				const int32_t iPoc0=pCtx->sRefPic.pRefList[LIST_0][iRef0]->iFramePoc;
				bool bIsLongRef0=pCtx->sRefPic.pRefList[LIST_0][iRef0]->bIsLongRef;
				for(int32_t iRef1=0; iRef1<pSliceHeader->uiRefCount[1]; iRef1++){
					if(pCtx->sRefPic.pRefList[LIST_1][iRef1]){
						const int32_t iPoc1=pCtx->sRefPic.pRefList[LIST_1][iRef1]->iFramePoc;
						bool bIsLongRef1=pCtx->sRefPic.pRefList[LIST_1][iRef1]->bIsLongRef;
						pCurDqLayer->pPredWeightTable->iImplicitWeight[iRef0][iRef1]=32;
						if(!bIsLongRef0 && !bIsLongRef1){
							const int32_t iTd=WELS_CLIP3(iPoc1-iPoc0,-128,127);
							if(iTd){
								int32_t iTb=WELS_CLIP3(iPoc-iPoc0,-128,127);
								int32_t iTx=(16384+(WELS_ABS(iTd)>>1))/iTd;
								int32_t iDistScaleFactor=(iTb*iTx+32)>>8;
								if(iDistScaleFactor>=-64 && iDistScaleFactor<=128){
									pCurDqLayer->pPredWeightTable->iImplicitWeight[iRef0][iRef1]=64-iDistScaleFactor;
								}
							}
						}
					}
				}
			}
		}
	}
	return;
}


// fills the pRefPic.pRefList.
int32_t WelsInitRefList(SDecoderContext* pCtx,int32_t iPoc){

	int32_t err=WelsCheckAndRecoverForFutureDecoding(pCtx);
	if(err!=ERR_NONE) return err;

	WrapShortRefPicNum(pCtx);

	SPicture** ppShoreRefList=pCtx->sRefPic.pShortRefList[LIST_0];
	SPicture** ppLongRefList=pCtx->sRefPic.pLongRefList[LIST_0];
	memset(pCtx->sRefPic.pRefList[LIST_0],0,MAX_DPB_COUNT*sizeof(SPicture*));

	int32_t i,iCount=0;
	// short
	for(i=0; i<pCtx->sRefPic.uiShortRefCount[LIST_0];++i){
		pCtx->sRefPic.pRefList[LIST_0][iCount++]=ppShoreRefList[i];
	}

	// long
	for(i=0; i<pCtx->sRefPic.uiLongRefCount[LIST_0];++i){
		pCtx->sRefPic.pRefList[LIST_0][iCount++]=ppLongRefList[i];
	}
	pCtx->sRefPic.uiRefCount[LIST_0]=iCount;

	return ERR_NONE;
}

int32_t WelsReorderRefList(SDecoderContext* pCtx){

	if(pCtx->eSliceType==I_SLICE || pCtx->eSliceType==SI_SLICE){
		return ERR_NONE;
	}

	SRefPicListReorderSyn* pRefPicListReorderSyn=pCtx->pCurDqLayer->pRefPicListReordering;
	SNalUnitHeaderExt* pNalHeaderExt=&pCtx->pCurDqLayer->sLayerInfo.sNalHeaderExt;
	SSliceHeader* pSliceHeader=&pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader;
	int32_t ListCount=1;
	if(pCtx->eSliceType==B_SLICE) ListCount=2;
	for(int32_t listIdx=0; listIdx<ListCount;++listIdx){
		SPicture* pPic=NULL;
		SPicture** ppRefList=pCtx->sRefPic.pRefList[listIdx];
		int32_t iMaxRefIdx=pCtx->iPicQueueNumber;
		if(iMaxRefIdx>=MAX_REF_PIC_COUNT){
			iMaxRefIdx=MAX_REF_PIC_COUNT-1;
		}
		int32_t iRefCount=pSliceHeader->uiRefCount[listIdx];
		int32_t iPredFrameNum=pSliceHeader->iFrameNum;
		int32_t iMaxPicNum=1<<pSliceHeader->pSps->uiLog2MaxFrameNum;
		int32_t iAbsDiffPicNum=-1;
		int32_t iReorderingIndex=0;
		int32_t i=0;

		if(iRefCount<=0){
			pCtx->iErrorCode=dsNoParamSets;		// No any reference for decoding,SHOULD request IDR
			return ERR_INFO_REFERENCE_PIC_LOST;
		}

		if(pRefPicListReorderSyn->bRefPicListReorderingFlag[listIdx]){
			while((iReorderingIndex<iMaxRefIdx)
				  && (pRefPicListReorderSyn->sReorderingSyn[listIdx][iReorderingIndex].uiReorderingOfPicNumsIdc!=3)){
				uint16_t uiReorderingOfPicNumsIdc=
					pRefPicListReorderSyn->sReorderingSyn[listIdx][iReorderingIndex].uiReorderingOfPicNumsIdc;
				if(uiReorderingOfPicNumsIdc<2){
					iAbsDiffPicNum=pRefPicListReorderSyn->sReorderingSyn[listIdx][iReorderingIndex].uiAbsDiffPicNumMinus1+1;

					if(uiReorderingOfPicNumsIdc==0){
						iPredFrameNum-=iAbsDiffPicNum;
					}else{
						iPredFrameNum+=iAbsDiffPicNum;
					}
					iPredFrameNum&=iMaxPicNum-1;

					for(i=iMaxRefIdx-1; i>=0; i--){
						if(ppRefList[i]!=NULL && ppRefList[i]->iFrameNum==iPredFrameNum && !ppRefList[i]->bIsLongRef){
							if((pNalHeaderExt->uiQualityId==ppRefList[i]->uiQualityId)
								 && (pSliceHeader->iSpsId!=ppRefList[i]->iSpsId)){		// check;
								uprintf("WelsReorderRefList()::::BASE LAYER::::iSpsId:%d,ref_sps_id:%d",pSliceHeader->iSpsId,ppRefList[i]->iSpsId);
								pCtx->iErrorCode=dsNoParamSets;		// cross-IDR reference frame selection,SHOULD request IDR.--
								return ERR_INFO_REFERENCE_PIC_LOST;
							}else{
								break;
							}
						}
					}

				}else
				if(uiReorderingOfPicNumsIdc==2){
					for(i=iMaxRefIdx-1; i>=0; i--){
						if(ppRefList[i]!=NULL && ppRefList[i]->bIsLongRef
							 && ppRefList[i]->iLongTermFrameIdx==
							pRefPicListReorderSyn->sReorderingSyn[listIdx][iReorderingIndex].uiLongTermPicNum){
							if((pNalHeaderExt->uiQualityId==ppRefList[i]->uiQualityId)
								 && (pSliceHeader->iSpsId!=ppRefList[i]->iSpsId)){		// check;
								uprintf("WelsReorderRefList()::::BASE LAYER::::iSpsId:%d,ref_sps_id:%d",pSliceHeader->iSpsId,ppRefList[i]->iSpsId);
								pCtx->iErrorCode=dsNoParamSets;		// cross-IDR reference frame selection,SHOULD request IDR.--
								return ERR_INFO_REFERENCE_PIC_LOST;
							}else{
								break;
							}
						}
					}
				}
				if(i<0){
					return ERR_INFO_REFERENCE_PIC_LOST;
				}
				pPic=ppRefList[i];
				if(i>iReorderingIndex){
					memmove(&ppRefList[1+iReorderingIndex],&ppRefList[iReorderingIndex],(i-iReorderingIndex)*sizeof(SPicture*));
				}else
				if(i<iReorderingIndex){
					memmove(&ppRefList[1+iReorderingIndex],&ppRefList[iReorderingIndex],(iMaxRefIdx-iReorderingIndex)*sizeof(SPicture*));
				}
				ppRefList[iReorderingIndex]=pPic;
				iReorderingIndex++;
			}
		}
	}
	return ERR_NONE;
}


int32_t InitRefPicList(SDecoderContext* pCtx,const uint8_t kuiNRi,int32_t iPoc){
	int32_t iRet=ERR_NONE;
	if(pCtx->eSliceType==B_SLICE){
		iRet=WelsInitBSliceRefList(pCtx,iPoc);
		CreateImplicitWeightTable(pCtx);
	}else
		iRet=WelsInitRefList(pCtx,iPoc);
	if((pCtx->eSliceType!=I_SLICE && pCtx->eSliceType!=SI_SLICE)){
#if 0
		if(pCtx->pSps->uiProfileIdc!=66 && pCtx->pPps->bEntropyCodingModeFlag)
			iRet=WelsReorderRefList2(pCtx);
		else
#endif
			iRet=WelsReorderRefList(pCtx);
	}

	return iRet;
}

inline void HandleReferenceLost(SDecoderContext* pCtx,SNalUnit* pCurNal){
	if((0==pCurNal->sNalHeaderExt.uiTemporalId) || (1==pCurNal->sNalHeaderExt.uiTemporalId)){
		pCtx->bReferenceLostAtT0Flag=true;
	}
	pCtx->iErrorCode|=dsRefLost;
}

// Compute the temporal-direct scaling factor that's common
// to all direct MBs in this slice,as per clause 8.4.1.2.3
// of T-REC H.264 201704
bool ComputeColocatedTemporalScaling(SDecoderContext* pCtx){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SSlice* pCurSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pCurSlice->sSliceHeaderExt.sSliceHeader;
	if(!pSliceHeader->iDirectSpatialMvPredFlag){
		uint32_t uiRefCount=pSliceHeader->uiRefCount[LIST_0];
		if(pCtx->sRefPic.pRefList[LIST_1][0]!=NULL){
			for(uint32_t i=0; i<uiRefCount;++i){
				if(pCtx->sRefPic.pRefList[LIST_0][i]!=NULL){
					const int32_t poc0=pCtx->sRefPic.pRefList[LIST_0][i]->iFramePoc;
					const int32_t poc1=pCtx->sRefPic.pRefList[LIST_1][0]->iFramePoc;
					const int32_t poc=pSliceHeader->iPicOrderCntLsb;
					const int32_t td=WELS_CLIP3(poc1-poc0,-128,127);
					if(td==0){
						pCurSlice->iMvScale[LIST_0][i]=1<<8;
					}else{
						int32_t tb=WELS_CLIP3(poc-poc0,-128,127);
						int32_t tx=(16384+(abs(td)>>1))/td;
						pCurSlice->iMvScale[LIST_0][i]=WELS_CLIP3((tb*tx+32)>>6,-1024,1023);
					}
				}
			}
		}
	}
	return true;
}

#define MAX_LEVEL_PREFIX 15

typedef struct TagReadBitsCache{
	uint32_t uiCache32Bit;
	uint8_t uiRemainBits;
	uint8_t* pBuf;
} SReadBitsCache;

void GetNeighborAvailMbType(PWelsNeighAvail pNeighAvail,PDqLayer pCurDqLayer){
	int32_t iCurSliceIdc,iTopSliceIdc,iLeftTopSliceIdc,iRightTopSliceIdc,iLeftSliceIdc;
	int32_t iCurXy,iTopXy=0,iLeftXy=0,iLeftTopXy=0,iRightTopXy=0;
	int32_t iCurX,iCurY;

	iCurXy=pCurDqLayer->iMbXyIndex;
	iCurX=pCurDqLayer->iMbX;
	iCurY=pCurDqLayer->iMbY;
	iCurSliceIdc=pCurDqLayer->pSliceIdc[iCurXy];
	if(iCurX!=0){
		iLeftXy=iCurXy-1;
		iLeftSliceIdc=pCurDqLayer->pSliceIdc[iLeftXy];
		pNeighAvail->iLeftAvail=(iLeftSliceIdc==iCurSliceIdc);
		pNeighAvail->iLeftCbp=pNeighAvail->iLeftAvail ? pCurDqLayer->pCbp[iLeftXy] : 0;
	}else{
		pNeighAvail->iLeftAvail=0;
		pNeighAvail->iLeftTopAvail=0;
		pNeighAvail->iLeftCbp=0;
	}

	if(iCurY!=0){
		iTopXy=iCurXy-pCurDqLayer->iMbWidth;
		iTopSliceIdc=pCurDqLayer->pSliceIdc[iTopXy];
		pNeighAvail->iTopAvail=(iTopSliceIdc==iCurSliceIdc);
		pNeighAvail->iTopCbp=pNeighAvail->iTopAvail ? pCurDqLayer->pCbp[iTopXy] : 0;
		if(iCurX!=0){
			iLeftTopXy=iTopXy-1;
			iLeftTopSliceIdc=pCurDqLayer->pSliceIdc[iLeftTopXy];
			pNeighAvail->iLeftTopAvail=(iLeftTopSliceIdc==iCurSliceIdc);
		}else{
			pNeighAvail->iLeftTopAvail=0;
		}
		if(iCurX!=(pCurDqLayer->iMbWidth-1)){
			iRightTopXy=iTopXy+1;
			iRightTopSliceIdc=pCurDqLayer->pSliceIdc[iRightTopXy];
			pNeighAvail->iRightTopAvail=(iRightTopSliceIdc==iCurSliceIdc);
		}else{
			pNeighAvail->iRightTopAvail=0;
		}
	}else{
		pNeighAvail->iTopAvail=0;
		pNeighAvail->iLeftTopAvail=0;
		pNeighAvail->iRightTopAvail=0;
		pNeighAvail->iTopCbp=0;
	}

	pNeighAvail->iLeftType=(pNeighAvail->iLeftAvail ? pCurDqLayer->pDec->pMbType[iLeftXy] : 0);
	pNeighAvail->iTopType=(pNeighAvail->iTopAvail ? pCurDqLayer->pDec->pMbType[iTopXy] : 0);
	pNeighAvail->iLeftTopType=(pNeighAvail->iLeftTopAvail ? pCurDqLayer->pDec->pMbType[iLeftTopXy] : 0);
	pNeighAvail->iRightTopType=(pNeighAvail->iRightTopAvail ? pCurDqLayer->pDec->pMbType[iRightTopXy] : 0);
}

const uint8_t g_kuiCabacRangeLps[64][4]={
	{128,176,208,240},{128,167,197,227},{128,158,187,216},{123,150,178,205},{116,142,169,195},{111,135,160,185},{105,128,152,175},{100,122,144,166},
 {95,116,137,158},{90,110,130,150},{85,104,123,142},{81,99,117,135},{77,94,111,128},{73,89,105,122},{69,85,100,116},{66,80,95,110},
 {62,76,90,104},{59,72,86,99},{56,69,81,94},{53,65,77,89},{51,62,73,85},{48,59,69,80},{46,56,66,76},{43,53,63,72},
 {41,50,59,69},{39,48,56,65},{37,45,54,62},{35,43,51,59},{33,41,48,56},{32,39,46,53},{30,37,43,50},{29,35,41,48},
 {27,33,39,45},{26,31,37,43},{24,30,35,41},{23,28,33,39},{22,27,32,37},{21,26,30,35},{20,24,29,33},{19,23,27,31},
 {18,22,26,30},{17,21,25,28},{16,20,23,27},{15,19,22,25},{14,18,21,24},{14,17,20,23},{13,16,19,22},{12,15,18,21},
 {12,14,17,20},{11,14,16,19},{11,13,15,18},{10,12,15,17},{10,12,14,16},{9,11,13,15},{9,11,12,14},{8,10,12,14},
 {8,9,11,13},{7,9,11,12},{7,9,10,12},{7,8,10,11},{6,8,9,11},{6,7,9,10},{6,7,8,9},{2,2,2,2}
};

// Table 9-45 – State transition table

const uint8_t g_kuiStateTransTable[64][2]={

	{0,1},{0,2},{1,3},{2,4},{2,5},{4,6},{4,7},{5,8},{6,9},{7,10},

 {8,11},{9,12},{9,13},{11,14},{11,15},{12,16},{13,17},{13,18},{15,19},{15,20},

 {16,21},{16,22},{18,23},{18,24},{19,25},{19,26},{21,27},{21,28},{22,29},{22,30},

 {23,31},{24,32},{24,33},{25,34},{26,35},{26,36},{27,37},{27,38},{28,39},{29,40},

 {29,41},{30,42},{30,43},{30,44},{31,45},{32,46},{32,47},{33,48},{33,49},{33,50},

 {34,51},{34,52},{35,53},{35,54},{35,55},{36,56},{36,57},{36,58},{37,59},{37,60},

 {37,61},{38,62},{38,62},{63,63}

};
static const uint8_t g_kRenormTable256[256]={
	6,6,6,6,6,6,6,6,
	5,5,5,5,5,5,5,5,
	4,4,4,4,4,4,4,4,
	4,4,4,4,4,4,4,4,
	3,3,3,3,3,3,3,3,
	3,3,3,3,3,3,3,3,
	3,3,3,3,3,3,3,3,
	3,3,3,3,3,3,3,3,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1
};

#define WELS_CABAC_HALF 0x01FE
#define WELS_CABAC_QUARTER 0x0100
#define WELS_CABAC_FALSE_RETURN(iErrorInfo) \
if(iErrorInfo) { \
 return iErrorInfo; \
}

// -------------------3. actual decoding
int32_t Read32BitsCabac(SWelsCabacDecEngine* pDecEngine,uint32_t& uiValue,int32_t& iNumBitsRead){
	intX_t iLeftBytes=pDecEngine->pBuffEnd-pDecEngine->pBuffCurr;
	iNumBitsRead=0;
	uiValue=0;
	if(iLeftBytes<=0){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_CABAC_NO_BS_TO_READ);
	}
	switch(iLeftBytes){
		case 3:
			uiValue=((pDecEngine->pBuffCurr[0])<<16|(pDecEngine->pBuffCurr[1])<<8|(pDecEngine->pBuffCurr[2]));
			pDecEngine->pBuffCurr+=3;
			iNumBitsRead=24;
			break;
		case 2:
			uiValue=((pDecEngine->pBuffCurr[0])<<8|(pDecEngine->pBuffCurr[1]));
			pDecEngine->pBuffCurr+=2;
			iNumBitsRead=16;
			break;
		case 1:
			uiValue=pDecEngine->pBuffCurr[0];
			pDecEngine->pBuffCurr+=1;
			iNumBitsRead=8;
			break;
		default:
			uiValue=((pDecEngine->pBuffCurr[0]<<24)|(pDecEngine->pBuffCurr[1])<<16|(pDecEngine->pBuffCurr[2])<<8|
					 (pDecEngine->pBuffCurr[3]));
			pDecEngine->pBuffCurr+=4;
			iNumBitsRead=32;
			break;
	}
	return ERR_NONE;
}

int32_t DecodeBinCabac(SWelsCabacDecEngine* pDecEngine,SWelsCabacCtx* pBinCtx,uint32_t& uiBinVal){
	int32_t iErrorInfo=ERR_NONE;
	uint32_t uiState=pBinCtx->uiState;
	uiBinVal=pBinCtx->uiMPS;
	uint64_t uiOffset=pDecEngine->uiOffset;
	uint64_t uiRange=pDecEngine->uiRange;

	int32_t iRenorm=1;
	uint32_t uiRangeLPS=g_kuiCabacRangeLps[uiState][(uiRange>>6)&0x03];
	uiRange-=uiRangeLPS;
	if(uiOffset>=(uiRange<<pDecEngine->iBitsLeft)){		// LPS
		uiOffset-=(uiRange<<pDecEngine->iBitsLeft);
		uiBinVal^=0x0001;
		if(!uiState)
			pBinCtx->uiMPS^=0x01;
		pBinCtx->uiState=g_kuiStateTransTable[uiState][0];
		iRenorm=g_kRenormTable256[uiRangeLPS];
		uiRange=(uiRangeLPS<<iRenorm);
	}else{		// MPS
		pBinCtx->uiState=g_kuiStateTransTable[uiState][1];
		if(uiRange>=WELS_CABAC_QUARTER){
			pDecEngine->uiRange=uiRange;
			return ERR_NONE;
		}else{
			uiRange<<=1;
		}
	}
	// Renorm
	pDecEngine->uiRange=uiRange;
	pDecEngine->iBitsLeft-=iRenorm;
	if(pDecEngine->iBitsLeft>0){
		pDecEngine->uiOffset=uiOffset;
		return ERR_NONE;
	}
	uint32_t uiVal=0;
	int32_t iNumBitsRead=0;
	iErrorInfo=Read32BitsCabac(pDecEngine,uiVal,iNumBitsRead);
	pDecEngine->uiOffset=(uiOffset<<iNumBitsRead)|uiVal;
	pDecEngine->iBitsLeft+=iNumBitsRead;
	if(iErrorInfo && pDecEngine->iBitsLeft<0){
		return iErrorInfo;
	}
	return ERR_NONE;
}


int32_t ParseSkipFlagCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint32_t& uiSkip){
	uiSkip=0;
	int32_t iCtxInc=NEW_CTX_OFFSET_SKIP;
	iCtxInc+=(pNeighAvail->iLeftAvail && !IS_SKIP(pNeighAvail->iLeftType))+(pNeighAvail->iTopAvail && !IS_SKIP(pNeighAvail->iTopType));
	if(B_SLICE==pCtx->eSliceType)
		iCtxInc+=13;
	SWelsCabacCtx* pBinCtx=(pCtx->pCabacCtx+iCtxInc);
	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pBinCtx,uiSkip));
	return ERR_NONE;
}

inline uint32_t* GetMbType(PDqLayer& pCurDqLayer){
	if(pCurDqLayer->pDec!=NULL){
		return pCurDqLayer->pDec->pMbType;
	}else{
		return pCurDqLayer->pMbType;
	}
}

// the second path will degrades the performance
#if 1
static inline int32_t WelsMedian(int32_t iX,int32_t iY,int32_t iZ){
	int32_t iMin=iX,iMax=iX;

	if(iY<iMin)
		iMin=iY;
	else
		iMax=iY;

	if(iZ<iMin)
		iMin=iZ;
	else if(iZ>iMax)
		iMax=iZ;

	return (iX+iY+iZ)-(iMin+iMax);
}
#else
static inline int32_t WelsMedian(int32_t iX,int32_t iY,int32_t iZ){
	int32_t iTmp=(iX-iY)&((iX-iY)>>31);
	iX-=iTmp;
	iY+=iTmp;
	iY-=(iY-iZ)&((iY-iZ)>>31);
	iY+=(iX-iY)&((iX-iY)>>31);
	return iY;
}

#endif

void PredPSkipMvFromNeighbor(PDqLayer pCurDqLayer,int16_t iMvp[2]){
	bool bTopAvail,bLeftTopAvail,bRightTopAvail,bLeftAvail;

	int32_t iCurSliceIdc,iTopSliceIdc,iLeftTopSliceIdc,iRightTopSliceIdc,iLeftSliceIdc;
	int32_t iLeftTopType,iRightTopType,iTopType,iLeftType;
	int32_t iCurX,iCurY,iCurXy,iLeftXy,iTopXy=0,iLeftTopXy=0,iRightTopXy=0;

	int8_t iLeftRef;
	int8_t iTopRef;
	int8_t iRightTopRef;
	int8_t iLeftTopRef;
	int8_t iDiagonalRef;
	int8_t iMatchRef;
	int16_t iMvA[2],iMvB[2],iMvC[2],iMvD[2];

	iCurXy=pCurDqLayer->iMbXyIndex;
	iCurX=pCurDqLayer->iMbX;
	iCurY=pCurDqLayer->iMbY;
	iCurSliceIdc=pCurDqLayer->pSliceIdc[iCurXy];

	if(iCurX!=0){
		iLeftXy=iCurXy-1;
		iLeftSliceIdc=pCurDqLayer->pSliceIdc[iLeftXy];
		bLeftAvail=(iLeftSliceIdc==iCurSliceIdc);
	}else{
		bLeftAvail=0;
		bLeftTopAvail=0;
	}

	if(iCurY!=0){
		iTopXy=iCurXy-pCurDqLayer->iMbWidth;
		iTopSliceIdc=pCurDqLayer->pSliceIdc[iTopXy];
		bTopAvail=(iTopSliceIdc==iCurSliceIdc);
		if(iCurX!=0){
			iLeftTopXy=iTopXy-1;
			iLeftTopSliceIdc=pCurDqLayer->pSliceIdc[iLeftTopXy];
			bLeftTopAvail=(iLeftTopSliceIdc==iCurSliceIdc);
		}else{
			bLeftTopAvail=0;
		}
		if(iCurX!=(pCurDqLayer->iMbWidth-1)){
			iRightTopXy=iTopXy+1;
			iRightTopSliceIdc=pCurDqLayer->pSliceIdc[iRightTopXy];
			bRightTopAvail=(iRightTopSliceIdc==iCurSliceIdc);
		}else{
			bRightTopAvail=0;
		}
	}else{
		bTopAvail=0;
		bLeftTopAvail=0;
		bRightTopAvail=0;
	}

	iLeftType=((iCurX!=0 && bLeftAvail) ? GetMbType(pCurDqLayer)[iLeftXy] : 0);
	iTopType=((iCurY!=0 && bTopAvail) ? GetMbType(pCurDqLayer)[iTopXy] : 0);
	iLeftTopType=((iCurX!=0 && iCurY!=0 && bLeftTopAvail)
					? GetMbType(pCurDqLayer)[iLeftTopXy] : 0);
	iRightTopType=((iCurX!=pCurDqLayer->iMbWidth-1 && iCurY!=0 && bRightTopAvail)
					 ? GetMbType(pCurDqLayer)[iRightTopXy] : 0);

	// get neb mv&iRefIdxArray
	// left
	if(bLeftAvail && IS_INTER(iLeftType)){
		ST32(iMvA,LD32(pCurDqLayer->pDec ? pCurDqLayer->pDec->pMv[0][iLeftXy][3] : pCurDqLayer->pMv[0][iLeftXy][3]));
		iLeftRef=pCurDqLayer->pDec ? pCurDqLayer->pDec->pRefIndex[0][iLeftXy][3] : pCurDqLayer->pRefIndex[0][iLeftXy][3];
	}else{
		ST32(iMvA,0);
		if(0==bLeftAvail){		// not available
			iLeftRef=REF_NOT_AVAIL;
		}else{		// available but is intra mb type
			iLeftRef=REF_NOT_IN_LIST;
		}
	}
	if(REF_NOT_AVAIL==iLeftRef || 
		(0==iLeftRef && 0==*(int32_t*)iMvA)){
		ST32(iMvp,0);
		return;
	}

	// top
	if(bTopAvail && IS_INTER(iTopType)){
		ST32(iMvB,LD32(pCurDqLayer->pDec ? pCurDqLayer->pDec->pMv[0][iTopXy][12] : pCurDqLayer->pMv[0][iTopXy][12]));
		iTopRef=pCurDqLayer->pDec ? pCurDqLayer->pDec->pRefIndex[0][iTopXy][12] : pCurDqLayer->pRefIndex[0][iTopXy][12];
	}else{
		ST32(iMvB,0);
		if(0==bTopAvail){		// not available
			iTopRef=REF_NOT_AVAIL;
		}else{		// available but is intra mb type
			iTopRef=REF_NOT_IN_LIST;
		}
	}
	if(REF_NOT_AVAIL==iTopRef || 
		(0==iTopRef && 0==*(int32_t*)iMvB)){
		ST32(iMvp,0);
		return;
	}

	// right_top
	if(bRightTopAvail && IS_INTER(iRightTopType)){
		ST32(iMvC,LD32(pCurDqLayer->pDec ? pCurDqLayer->pDec->pMv[0][iRightTopXy][12] :
			pCurDqLayer->pMv[0][iRightTopXy][12]));
		iRightTopRef=pCurDqLayer->pDec ? pCurDqLayer->pDec->pRefIndex[0][iRightTopXy][12] :
			pCurDqLayer->pRefIndex[0][iRightTopXy][12];
	}else{
		ST32(iMvC,0);
		if(0==bRightTopAvail){		// not available
			iRightTopRef=REF_NOT_AVAIL;
		}else{		// available but is intra mb type
			iRightTopRef=REF_NOT_IN_LIST;
		}
	}

	// left_top
	if(bLeftTopAvail && IS_INTER(iLeftTopType)){
		ST32(iMvD,LD32(pCurDqLayer->pDec ? pCurDqLayer->pDec->pMv[0][iLeftTopXy][15] : pCurDqLayer->pMv[0][iLeftTopXy][15]));
		iLeftTopRef=pCurDqLayer->pDec ? pCurDqLayer->pDec->pRefIndex[0][iLeftTopXy][15] :
			pCurDqLayer->pRefIndex[0][iLeftTopXy][15];
	}else{
		ST32(iMvD,0);
		if(0==bLeftTopAvail){		// not available
			iLeftTopRef=REF_NOT_AVAIL;
		}else{		// available but is intra mb type
			iLeftTopRef=REF_NOT_IN_LIST;
		}
	}

	iDiagonalRef=iRightTopRef;
	if(REF_NOT_AVAIL==iDiagonalRef){
		iDiagonalRef=iLeftTopRef;
		*(int32_t*)iMvC=*(int32_t*)iMvD;
	}

	if(REF_NOT_AVAIL==iTopRef && REF_NOT_AVAIL==iDiagonalRef && iLeftRef>=REF_NOT_IN_LIST){
		ST32(iMvp,LD32(iMvA));
		return;
	}

	iMatchRef=(0==iLeftRef)+(0==iTopRef)+(0==iDiagonalRef);
	if(1==iMatchRef){
		if(0==iLeftRef){
			ST32(iMvp,LD32(iMvA));
		}else
		if(0==iTopRef){
			ST32(iMvp,LD32(iMvB));
		}else{
			ST32(iMvp,LD32(iMvC));
		}
	}else{
		iMvp[0]=WelsMedian(iMvA[0],iMvB[0],iMvC[0]);
		iMvp[1]=WelsMedian(iMvA[1],iMvB[1],iMvC[1]);
	}
}
// extern at wels_common_defs.h
const uint8_t g_kuiChromaQpTable[52]={
	0,1,2,3,4,5,6,7,8,9,10,11,
	12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,
	28,29,29,30,31,32,32,33,34,34,35,35,36,36,37,37,
	37,38,38,38,39,39,39,39
};

int32_t DecodeTerminateCabac(SWelsCabacDecEngine* pDecEngine,uint32_t& uiBinVal){
	int32_t iErrorInfo=ERR_NONE;
	uint64_t uiRange=pDecEngine->uiRange-2;
	uint64_t uiOffset=pDecEngine->uiOffset;

	if(uiOffset>=(uiRange<<pDecEngine->iBitsLeft)){
		uiBinVal=1;
	}else{
		uiBinVal=0;
		// Renorm
		if(uiRange<WELS_CABAC_QUARTER){
			int32_t iRenorm=g_kRenormTable256[uiRange];
			pDecEngine->uiRange=(uiRange<<iRenorm);
			pDecEngine->iBitsLeft-=iRenorm;
			if(pDecEngine->iBitsLeft<0){
				uint32_t uiVal=0;
				int32_t iNumBitsRead=0;
				iErrorInfo=Read32BitsCabac(pDecEngine,uiVal,iNumBitsRead);
				pDecEngine->uiOffset=(pDecEngine->uiOffset<<iNumBitsRead)|uiVal;
				pDecEngine->iBitsLeft+=iNumBitsRead;
			}
			if(iErrorInfo && pDecEngine->iBitsLeft<0){
				return iErrorInfo;
			}
			return ERR_NONE;
		}else{
			pDecEngine->uiRange=uiRange;
			return ERR_NONE;
		}
	}
	return ERR_NONE;
}

int32_t ParseEndOfSliceCabac(SDecoderContext* pCtx,uint32_t& uiBinVal){
	uiBinVal=0;
	WELS_READ_VERIFY(DecodeTerminateCabac(pCtx->pCabacDecEngine,uiBinVal));
	return ERR_NONE;
}

int32_t ParseMBTypePSliceCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint32_t& uiMbType){
	uint32_t uiCode;
	uiMbType=0;
	SWelsCabacDecEngine* pCabacDecEngine=pCtx->pCabacDecEngine;

	SWelsCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_SKIP;
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
	if(uiCode){
		// Intra MB
		WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+6,uiCode));
		if(uiCode){		// Intra 16x16
			WELS_READ_VERIFY(DecodeTerminateCabac(pCabacDecEngine,uiCode));
			if(uiCode){
				uiMbType=30;
				return ERR_NONE;	// MB_TYPE_INTRA_PCM;
			}

			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+7,uiCode));
			uiMbType=6+uiCode*12;

			// uiCbp: 0,1,2
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+8,uiCode));
			if(uiCode){
				uiMbType+=4;
				WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+8,uiCode));
				if(uiCode)
					uiMbType+=4;
			}

			// IPredMode: 0,1,2,3
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+9,uiCode));
			uiMbType+=(uiCode<<1);
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+9,uiCode));
			uiMbType+=uiCode;
		}else
			// Intra 4x4
			uiMbType=5;
	}else{		// P MB
		WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+4,uiCode));
		if(uiCode){		// second bit
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+6,uiCode));
			if(uiCode)
				uiMbType=1;
			else
				uiMbType=2;
		}else{
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+5,uiCode));
			if(uiCode)
				uiMbType=3;
			else
				uiMbType=0;
		}
	}
	return ERR_NONE;
}

typedef struct TagPartMbInfo{
	MbType iType;
	int8_t iPartCount;		// P_16*16,P_16*8,P_8*16,P_8*8 based on 8*8 block; P_8*4,P_4*8,P_4*4 based on 4*4 block
	int8_t iPartWidth;		// based on 4*4 block
} SPartMbInfo;


// Table 7.13. Macroblock type values 0 to 4 for P slices.
static const SPartMbInfo g_ksInterPMbTypeInfo[5]={
	{MB_TYPE_16x16,1,4},
 {MB_TYPE_16x8,2,4},
 {MB_TYPE_8x16,2,2},
 {MB_TYPE_8x8,4,4},
 {MB_TYPE_8x8_REF0,4,4},// ref0--ref_idx not present in bit-stream and default as 0
};

void WelsFillCacheNonZeroCount(PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCount,PDqLayer pCurDqLayer){		// no matter slice type,intra_pred_constrained_flag
	int32_t iCurXy=pCurDqLayer->iMbXyIndex;
	int32_t iTopXy=0;
	int32_t iLeftXy=0;
	if(pNeighAvail->iTopAvail){
		iTopXy=iCurXy-pCurDqLayer->iMbWidth;
	}
	if(pNeighAvail->iLeftAvail){
		iLeftXy=iCurXy-1;
	}

	// stuff non_zero_coeff_count from pNeighAvail(left and top)
	if(pNeighAvail->iTopAvail){
		ST32(&pNonZeroCount[1],LD32(&pCurDqLayer->pNzc[iTopXy][12]));
		pNonZeroCount[0]=pNonZeroCount[5]=pNonZeroCount[29]=0;
		ST16(&pNonZeroCount[6],LD16(&pCurDqLayer->pNzc[iTopXy][20]));
		ST16(&pNonZeroCount[30],LD16(&pCurDqLayer->pNzc[iTopXy][22]));
	}else{
		ST32(&pNonZeroCount[1],0xFFFFFFFFU);
		pNonZeroCount[0]=pNonZeroCount[5]=pNonZeroCount[29]=0xFF;
		ST16(&pNonZeroCount[6],0xFFFF);
		ST16(&pNonZeroCount[30],0xFFFF);
	}

	if(pNeighAvail->iLeftAvail){
		pNonZeroCount[8*1]=pCurDqLayer->pNzc[iLeftXy][3];
		pNonZeroCount[8*2]=pCurDqLayer->pNzc[iLeftXy][7];
		pNonZeroCount[8*3]=pCurDqLayer->pNzc[iLeftXy][11];
		pNonZeroCount[8*4]=pCurDqLayer->pNzc[iLeftXy][15];

		pNonZeroCount[5+8*1]=pCurDqLayer->pNzc[iLeftXy][17];
		pNonZeroCount[5+8*2]=pCurDqLayer->pNzc[iLeftXy][21];
		pNonZeroCount[5+8*4]=pCurDqLayer->pNzc[iLeftXy][19];
		pNonZeroCount[5+8*5]=pCurDqLayer->pNzc[iLeftXy][23];
	}else{
		pNonZeroCount[8*1]=
			pNonZeroCount[8*2]=
			pNonZeroCount[8*3]=
			pNonZeroCount[8*4]=-1;	// unavailable

		pNonZeroCount[5+8*1]=
			pNonZeroCount[5+8*2]=-1;	// unavailable

		pNonZeroCount[5+8*4]=
			pNonZeroCount[5+8*5]=-1;	// unavailable
	}
}

void WelsFillCacheInterCabac(PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCount,int16_t iMvArray[LIST_A][30][MV_A],int16_t iMvdCache[LIST_A][30][MV_A],int8_t iRefIdxArray[LIST_A][30],PDqLayer pCurDqLayer){
	int32_t iCurXy=pCurDqLayer->iMbXyIndex;
	int32_t iTopXy=0;
	int32_t iLeftXy=0;
	int32_t iLeftTopXy=0;
	int32_t iRightTopXy=0;

	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	int32_t listCount=1;
	if(pSliceHeader->eSliceType==B_SLICE){
		listCount=2;
	}
	// stuff non_zero_coeff_count from pNeighAvail(left and top)
	WelsFillCacheNonZeroCount(pNeighAvail,pNonZeroCount,pCurDqLayer);

	if(pNeighAvail->iTopAvail){
		iTopXy=iCurXy-pCurDqLayer->iMbWidth;
	}
	if(pNeighAvail->iLeftAvail){
		iLeftXy=iCurXy-1;
	}
	if(pNeighAvail->iLeftTopAvail){
		iLeftTopXy=iCurXy-1-pCurDqLayer->iMbWidth;
	}
	if(pNeighAvail->iRightTopAvail){
		iRightTopXy=iCurXy+1-pCurDqLayer->iMbWidth;
	}

	for(int32_t listIdx=0; listIdx<listCount;++listIdx){
		// stuff mv_cache and iRefIdxArray from left and top (inter)
		if(pNeighAvail->iLeftAvail && IS_INTER(pNeighAvail->iLeftType)){
			ST32(iMvArray[listIdx][6],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftXy][3]));
			ST32(iMvArray[listIdx][12],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftXy][7]));
			ST32(iMvArray[listIdx][18],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftXy][11]));
			ST32(iMvArray[listIdx][24],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftXy][15]));

			ST32(iMvdCache[listIdx][6],LD32(pCurDqLayer->pMvd[listIdx][iLeftXy][3]));
			ST32(iMvdCache[listIdx][12],LD32(pCurDqLayer->pMvd[listIdx][iLeftXy][7]));
			ST32(iMvdCache[listIdx][18],LD32(pCurDqLayer->pMvd[listIdx][iLeftXy][11]));
			ST32(iMvdCache[listIdx][24],LD32(pCurDqLayer->pMvd[listIdx][iLeftXy][15]));

			iRefIdxArray[listIdx][6]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftXy][3];
			iRefIdxArray[listIdx][12]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftXy][7];
			iRefIdxArray[listIdx][18]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftXy][11];
			iRefIdxArray[listIdx][24]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftXy][15];
		}else{
			ST32(iMvArray[listIdx][6],0);
			ST32(iMvArray[listIdx][12],0);
			ST32(iMvArray[listIdx][18],0);
			ST32(iMvArray[listIdx][24],0);

			ST32(iMvdCache[listIdx][6],0);
			ST32(iMvdCache[listIdx][12],0);
			ST32(iMvdCache[listIdx][18],0);
			ST32(iMvdCache[listIdx][24],0);


			if(0==pNeighAvail->iLeftAvail){		// not available
				iRefIdxArray[listIdx][6]=
					iRefIdxArray[listIdx][12]=
					iRefIdxArray[listIdx][18]=
					iRefIdxArray[listIdx][24]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][6]=
					iRefIdxArray[listIdx][12]=
					iRefIdxArray[listIdx][18]=
					iRefIdxArray[listIdx][24]=REF_NOT_IN_LIST;
			}
		}
		if(pNeighAvail->iLeftTopAvail && IS_INTER(pNeighAvail->iLeftTopType)){
			ST32(iMvArray[listIdx][0],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftTopXy][15]));
			ST32(iMvdCache[listIdx][0],LD32(pCurDqLayer->pMvd[listIdx][iLeftTopXy][15]));
			iRefIdxArray[listIdx][0]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftTopXy][15];
		}else{
			ST32(iMvArray[listIdx][0],0);
			ST32(iMvdCache[listIdx][0],0);
			if(0==pNeighAvail->iLeftTopAvail){		// not available
				iRefIdxArray[listIdx][0]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][0]=REF_NOT_IN_LIST;
			}
		}

		if(pNeighAvail->iTopAvail && IS_INTER(pNeighAvail->iTopType)){
			ST64(iMvArray[listIdx][1],LD64(pCurDqLayer->pDec->pMv[listIdx][iTopXy][12]));
			ST64(iMvArray[listIdx][3],LD64(pCurDqLayer->pDec->pMv[listIdx][iTopXy][14]));
			ST64(iMvdCache[listIdx][1],LD64(pCurDqLayer->pMvd[listIdx][iTopXy][12]));
			ST64(iMvdCache[listIdx][3],LD64(pCurDqLayer->pMvd[listIdx][iTopXy][14]));
			ST32(&iRefIdxArray[listIdx][1],LD32(&pCurDqLayer->pDec->pRefIndex[listIdx][iTopXy][12]));
		}else{
			ST64(iMvArray[listIdx][1],0);
			ST64(iMvArray[listIdx][3],0);
			ST64(iMvdCache[listIdx][1],0);
			ST64(iMvdCache[listIdx][3],0);
			if(0==pNeighAvail->iTopAvail){		// not available
				iRefIdxArray[listIdx][1]=
					iRefIdxArray[listIdx][2]=
					iRefIdxArray[listIdx][3]=
					iRefIdxArray[listIdx][4]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][1]=
					iRefIdxArray[listIdx][2]=
					iRefIdxArray[listIdx][3]=
					iRefIdxArray[listIdx][4]=REF_NOT_IN_LIST;
			}
		}

		if(pNeighAvail->iRightTopAvail && IS_INTER(pNeighAvail->iRightTopType)){
			ST32(iMvArray[listIdx][5],LD32(pCurDqLayer->pDec->pMv[listIdx][iRightTopXy][12]));
			ST32(iMvdCache[listIdx][5],LD32(pCurDqLayer->pMvd[listIdx][iRightTopXy][12]));
			iRefIdxArray[listIdx][5]=pCurDqLayer->pDec->pRefIndex[listIdx][iRightTopXy][12];
		}else{
			ST32(iMvArray[listIdx][5],0);
			if(0==pNeighAvail->iRightTopAvail){		// not available
				iRefIdxArray[listIdx][5]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][5]=REF_NOT_IN_LIST;
			}
		}

		// right-top 4*4 block unavailable
		ST32(iMvArray[listIdx][9],0);
		ST32(iMvArray[listIdx][21],0);
		ST32(iMvArray[listIdx][11],0);
		ST32(iMvArray[listIdx][17],0);
		ST32(iMvArray[listIdx][23],0);
		ST32(iMvdCache[listIdx][9],0);
		ST32(iMvdCache[listIdx][21],0);
		ST32(iMvdCache[listIdx][11],0);
		ST32(iMvdCache[listIdx][17],0);
		ST32(iMvdCache[listIdx][23],0);
		iRefIdxArray[listIdx][9]=
			iRefIdxArray[listIdx][21]=
			iRefIdxArray[listIdx][11]=
			iRefIdxArray[listIdx][17]=
			iRefIdxArray[listIdx][23]=REF_NOT_AVAIL;
	}
}

// cache element equal to 30
const uint8_t g_kuiCache30ScanIdx[16]={		// mv or uiRefIndex cache scan index,4*4 block as basic unit
	7,8,13,14,
	9,10,15,16,
	19,20,25,26,
	21,22,27,28
};

int32_t DecodeUnaryBinCabac(SWelsCabacDecEngine* pDecEngine,SWelsCabacCtx* pBinCtx,int32_t iCtxOffset,uint32_t& uiSymVal){
	uiSymVal=0;
	WELS_READ_VERIFY(DecodeBinCabac(pDecEngine,pBinCtx,uiSymVal));
	if(uiSymVal==0){
		return ERR_NONE;
	}else{
		uint32_t uiCode;
		pBinCtx+=iCtxOffset;
		uiSymVal=0;
		do{
			WELS_READ_VERIFY(DecodeBinCabac(pDecEngine,pBinCtx,uiCode));
			++uiSymVal;
		} while(uiCode!=0);
		return ERR_NONE;
	}
}


int32_t ParseRefIdxCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint8_t* nzc,int8_t ref_idx[LIST_A][30],int8_t direct[30],int32_t iListIdx,int32_t iZOrderIdx,int32_t iActiveRefNum,int32_t b8mode,int8_t& iRefIdxVal){
	if(iActiveRefNum==1){
		iRefIdxVal=0;
		return ERR_NONE;
	}
	uint32_t uiCode;
	int32_t iIdxA=0,iIdxB=0;
	int32_t iCtxInc=0;
	int8_t* pRefIdxInMB=pCtx->pCurDqLayer->pDec->pRefIndex[iListIdx][pCtx->pCurDqLayer->iMbXyIndex];
	int8_t* pDirect=pCtx->pCurDqLayer->pDirect[pCtx->pCurDqLayer->iMbXyIndex];
	if(iZOrderIdx==0){
		iIdxB=(pNeighAvail->iTopAvail && pNeighAvail->iTopType!=MB_TYPE_INTRA_PCM && ref_idx[iListIdx][g_kuiCache30ScanIdx[iZOrderIdx]-6]>0);
		iIdxA=(pNeighAvail->iLeftAvail && pNeighAvail->iLeftType!=MB_TYPE_INTRA_PCM && ref_idx[iListIdx][g_kuiCache30ScanIdx[iZOrderIdx]-1]>0);
		if(pCtx->eSliceType==B_SLICE){
			if(iIdxB>0 && direct[g_kuiCache30ScanIdx[iZOrderIdx]-6]==0){
				iCtxInc+=2;
			}
			if(iIdxA>0 && direct[g_kuiCache30ScanIdx[iZOrderIdx]-1]==0){
				iCtxInc++;
			}
		}
	}else
	if(iZOrderIdx==4){
		iIdxB=(pNeighAvail->iTopAvail && pNeighAvail->iTopType!=MB_TYPE_INTRA_PCM && ref_idx[iListIdx][g_kuiCache30ScanIdx[iZOrderIdx]-6]>0);
		iIdxA=pRefIdxInMB[g_kuiScan4[iZOrderIdx]-1]>0;
		if(pCtx->eSliceType==B_SLICE){
			if(iIdxB>0 && direct[g_kuiCache30ScanIdx[iZOrderIdx]-6]==0){
				iCtxInc+=2;
			}
			if(iIdxA>0 && pDirect[g_kuiScan4[iZOrderIdx]-1]==0){
				iCtxInc++;
			}
		}
	}else
	if(iZOrderIdx==8){

		iIdxB=pRefIdxInMB[g_kuiScan4[iZOrderIdx]-4]>0;
		iIdxA=(pNeighAvail->iLeftAvail && pNeighAvail->iLeftType!=MB_TYPE_INTRA_PCM && ref_idx[iListIdx][g_kuiCache30ScanIdx[iZOrderIdx]-1]>0);
		if(pCtx->eSliceType==B_SLICE){
			if(iIdxB>0 && pDirect[g_kuiScan4[iZOrderIdx]-4]==0){
				iCtxInc+=2;
			}
			if(iIdxA>0 && direct[g_kuiCache30ScanIdx[iZOrderIdx]-1]==0){
				iCtxInc++;
			}
		}
	}else{
		iIdxB=pRefIdxInMB[g_kuiScan4[iZOrderIdx]-4]>0;
		iIdxA=pRefIdxInMB[g_kuiScan4[iZOrderIdx]-1]>0;
		if(pCtx->eSliceType==B_SLICE){
			if(iIdxB>0 && pDirect[g_kuiScan4[iZOrderIdx]-4]==0){
				iCtxInc+=2;
			}
			if(iIdxA>0 && pDirect[g_kuiScan4[iZOrderIdx]-1]==0){
				iCtxInc++;
			}
		}
	}
	if(pCtx->eSliceType!=B_SLICE){
		iCtxInc=iIdxA+(iIdxB<<1);
	}

	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_REF_NO+iCtxInc,uiCode));
	if(uiCode){
		WELS_READ_VERIFY(DecodeUnaryBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_REF_NO+4,1,uiCode));
		++uiCode;
	}
	iRefIdxVal=(int8_t)uiCode;
	return ERR_NONE;
}

// basic iMVs prediction unit for iMVs partition width (4,2,1)
void PredMv(int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int32_t iPartWidth,int8_t iRef,int16_t iMVP[2]){
	const uint8_t kuiLeftIdx=g_kuiCache30ScanIdx[iPartIdx]-1;
	const uint8_t kuiTopIdx=g_kuiCache30ScanIdx[iPartIdx]-6;
	const uint8_t kuiRightTopIdx=kuiTopIdx+iPartWidth;
	const uint8_t kuiLeftTopIdx=kuiTopIdx-1;

	const int8_t kiLeftRef=iRefIndex[listIdx][kuiLeftIdx];
	const int8_t kiTopRef=iRefIndex[listIdx][kuiTopIdx];
	const int8_t kiRightTopRef=iRefIndex[listIdx][kuiRightTopIdx];
	const int8_t kiLeftTopRef=iRefIndex[listIdx][kuiLeftTopIdx];
	int8_t iDiagonalRef=kiRightTopRef;

	int8_t iMatchRef=0;


	int16_t iAMV[2],iBMV[2],iCMV[2];

	ST32(iAMV,LD32(iMotionVector[listIdx][kuiLeftIdx]));
	ST32(iBMV,LD32(iMotionVector[listIdx][kuiTopIdx]));
	ST32(iCMV,LD32(iMotionVector[listIdx][kuiRightTopIdx]));

	if(REF_NOT_AVAIL==iDiagonalRef){
		iDiagonalRef=kiLeftTopRef;
		ST32(iCMV,LD32(iMotionVector[listIdx][kuiLeftTopIdx]));
	}

	iMatchRef=(iRef==kiLeftRef)+(iRef==kiTopRef)+(iRef==iDiagonalRef);

	if(REF_NOT_AVAIL==kiTopRef && REF_NOT_AVAIL==iDiagonalRef && kiLeftRef>=REF_NOT_IN_LIST){
		ST32(iMVP,LD32(iAMV));
		return;
	}

	if(1==iMatchRef){
		if(iRef==kiLeftRef){
			ST32(iMVP,LD32(iAMV));
		}else
		if(iRef==kiTopRef){
			ST32(iMVP,LD32(iBMV));
		}else{
			ST32(iMVP,LD32(iCMV));
		}
	}else{
		iMVP[0]=WelsMedian(iAMV[0],iBMV[0],iCMV[0]);
		iMVP[1]=WelsMedian(iAMV[1],iBMV[1],iCMV[1]);
	}
}

static const int16_t g_kMvdBinPos2Ctx[8]={0,1,2,3,3,3,3,3};

int32_t DecodeBypassCabac(SWelsCabacDecEngine* pDecEngine,uint32_t& uiBinVal){
	int32_t iErrorInfo=ERR_NONE;
	int32_t iBitsLeft=pDecEngine->iBitsLeft;
	uint64_t uiOffset=pDecEngine->uiOffset;
	uint64_t uiRangeValue;


	if(iBitsLeft<=0){
		uint32_t uiVal=0;
		int32_t iNumBitsRead=0;
		iErrorInfo=Read32BitsCabac(pDecEngine,uiVal,iNumBitsRead);
		uiOffset=(uiOffset<<iNumBitsRead)|uiVal;
		iBitsLeft=iNumBitsRead;
		if(iErrorInfo && iBitsLeft==0){
			return iErrorInfo;
		}
	}
	iBitsLeft--;
	uiRangeValue=(pDecEngine->uiRange<<iBitsLeft);
	if(uiOffset>=uiRangeValue){
		pDecEngine->iBitsLeft=iBitsLeft;
		pDecEngine->uiOffset=uiOffset-uiRangeValue;
		uiBinVal=1;
		return ERR_NONE;
	}
	pDecEngine->iBitsLeft=iBitsLeft;
	pDecEngine->uiOffset=uiOffset;
	uiBinVal=0;
	return ERR_NONE;
}

int32_t DecodeExpBypassCabac(SWelsCabacDecEngine* pDecEngine,int32_t iCount,uint32_t& uiSymVal){
	uint32_t uiCode;
	int32_t iSymTmp=0;
	int32_t iSymTmp2=0;
	uiSymVal=0;
	do{
		WELS_READ_VERIFY(DecodeBypassCabac(pDecEngine,uiCode));
		if(uiCode==1){
			iSymTmp+=(1<<iCount);
			++iCount;
		}
	} while(uiCode!=0 && iCount!=16);
	if(iCount==16){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_CABAC_UNEXPECTED_VALUE);
	}

	while(iCount--){
		WELS_READ_VERIFY(DecodeBypassCabac(pDecEngine,uiCode));
		if(uiCode==1){
			iSymTmp2|=(1<<iCount);
		}
	}
	uiSymVal=(uint32_t)(iSymTmp+iSymTmp2);
	return ERR_NONE;
}

int32_t DecodeUEGMvCabac(SWelsCabacDecEngine* pDecEngine,SWelsCabacCtx* pBinCtx,uint32_t iMaxBin,uint32_t& uiCode){
	WELS_READ_VERIFY(DecodeBinCabac(pDecEngine,pBinCtx+g_kMvdBinPos2Ctx[0],uiCode));
	if(uiCode==0)
		return ERR_NONE;
	else{
		uint32_t uiTmp,uiCount=1;
		uiCode=0;
		do{
			WELS_READ_VERIFY(DecodeBinCabac(pDecEngine,pBinCtx+g_kMvdBinPos2Ctx[uiCount++],uiTmp));
			uiCode++;
		} while(uiTmp!=0 && uiCount!=8);

		if(uiTmp!=0){
			WELS_READ_VERIFY(DecodeExpBypassCabac(pDecEngine,3,uiTmp));
			uiCode+=(uiTmp+1);
		}
		return ERR_NONE;
	}
}

int32_t ParseMvdInfoCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,int8_t pRefIndex[LIST_A][30],int16_t pMvdCache[LIST_A][30][2],int32_t index,int8_t iListIdx,int8_t iMvComp,int16_t& iMvdVal){
	uint32_t uiCode;
	int32_t iIdxA=0;
	// int32_t sym;
	SWelsCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_MVD+iMvComp*CTX_NUM_MVD;
	iMvdVal=0;

	if(pRefIndex[iListIdx][g_kuiCache30ScanIdx[index]-6]>=0)
		iIdxA=WELS_ABS(pMvdCache[iListIdx][g_kuiCache30ScanIdx[index]-6][iMvComp]);
	if(pRefIndex[iListIdx][g_kuiCache30ScanIdx[index]-1]>=0)
		iIdxA+=WELS_ABS(pMvdCache[iListIdx][g_kuiCache30ScanIdx[index]-1][iMvComp]);

	int32_t iCtxInc=0;
	if(iIdxA>=3)
		iCtxInc=1+(iIdxA>32);

	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pBinCtx+iCtxInc,uiCode));
	if(uiCode){
		WELS_READ_VERIFY(DecodeUEGMvCabac(pCtx->pCabacDecEngine,pBinCtx+3,3,uiCode));
		iMvdVal=(int16_t)(uiCode+1);
		WELS_READ_VERIFY(DecodeBypassCabac(pCtx->pCabacDecEngine,uiCode));
		if(uiCode){
			iMvdVal=-iMvdVal;
		}
	}else{
		iMvdVal=0;
	}
	return ERR_NONE;
}


// update iMVs and iRefIndex cache for current MB,only for P_16*16 (SKIP inclusive)
// can be further optimized
void UpdateP16x16MotionInfo(PDqLayer pCurDqLayer,int32_t listIdx,int8_t iRef,int16_t iMVs[2]){
	const int16_t kiRef2=((uint8_t)iRef<<8)|(uint8_t)iRef;
	const int32_t kiMV32=LD32(iMVs);
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;

	for(i=0; i<16; i+=4){
		// mb
		const uint8_t kuiScan4Idx=g_kuiScan4[i];
		const uint8_t kuiScan4IdxPlus4=4+kuiScan4Idx;
		if(pCurDqLayer->pDec!=NULL){
			ST16(&pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);

			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}else{
			ST16(&pCurDqLayer->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&pCurDqLayer->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);

			ST32(pCurDqLayer->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}
	}
}

void UpdateP16x16MvdCabac(SDqLayer* pCurDqLayer,int16_t pMvd[2],const int8_t iListIdx){
	int32_t pMvd32[2];
	ST32(&pMvd32[0],LD32(pMvd));
	ST32(&pMvd32[1],LD32(pMvd));
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	for(i=0; i<16; i+=2){
		ST64(pCurDqLayer->pMvd[iListIdx][iMbXy][i],LD64(pMvd32));
	}
}

void UpdateP16x8RefIdxCabac(PDqLayer pCurDqLayer,int8_t pRefIndex[LIST_A][30],int32_t iPartIdx,const int8_t iRef,const int8_t iListIdx){
	uint32_t iRef32Bit=(uint32_t)iRef;
	const int32_t iRef4Bytes=(iRef32Bit<<24)|(iRef32Bit<<16)|(iRef32Bit<<8)|iRef32Bit;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
	const uint8_t iScan4Idx4=4+iScan4Idx;
	const uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
	const uint8_t iCacheIdx6=6+iCacheIdx;
	// mb
	ST32(&pCurDqLayer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx],iRef4Bytes);
	ST32(&pCurDqLayer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx4],iRef4Bytes);
	// cache
	ST32(&pRefIndex[iListIdx][iCacheIdx],iRef4Bytes);
	ST32(&pRefIndex[iListIdx][iCacheIdx6],iRef4Bytes);
}

void PredInter16x8Mv(int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVP[2]){
	if(0==iPartIdx){
		const int8_t kiTopRef=iRefIndex[listIdx][1];
		if(iRef==kiTopRef){
			ST32(iMVP,LD32(&iMotionVector[listIdx][1][0]));
			return;
		}
	}else{		// 8==iPartIdx
		const int8_t kiLeftRef=iRefIndex[listIdx][18];
		if(iRef==kiLeftRef){
			ST32(iMVP,LD32(&iMotionVector[listIdx][18][0]));
			return;
		}
	}

	PredMv(iMotionVector,iRefIndex,listIdx,iPartIdx,4,iRef,iMVP);
}

// update iRefIndex and iMVs of Mb,only for P16x8
// need further optimization,mb_cache not work
void UpdateP16x8MotionInfo(PDqLayer pCurDqLayer,int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVs[2]){
	const int16_t kiRef2=((uint8_t)iRef<<8)|(uint8_t)iRef;
	const int32_t kiMV32=LD32(iMVs);
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	for(i=0; i<2; i++,iPartIdx+=4){
		const uint8_t kuiScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t kuiScan4IdxPlus4=4+kuiScan4Idx;
		const uint8_t kuiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t kuiCacheIdxPlus6=6+kuiCacheIdx;

		// mb
		if(pCurDqLayer->pDec!=NULL){
			ST16(&pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}else{
			ST16(&pCurDqLayer->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&pCurDqLayer->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}
		// cache
		ST16(&iRefIndex[listIdx][kuiCacheIdx],kiRef2);
		ST16(&iRefIndex[listIdx][kuiCacheIdxPlus6],kiRef2);
		ST32(iMotionVector[listIdx][kuiCacheIdx],kiMV32);
		ST32(iMotionVector[listIdx][1+kuiCacheIdx],kiMV32);
		ST32(iMotionVector[listIdx][kuiCacheIdxPlus6],kiMV32);
		ST32(iMotionVector[listIdx][1+kuiCacheIdxPlus6],kiMV32);
	}
}

void UpdateP16x8MvdCabac(SDqLayer* pCurDqLayer,int16_t pMvdCache[LIST_A][30][MV_A],int32_t iPartIdx,int16_t pMvd[2],const int8_t iListIdx){
	int32_t pMvd32[2];
	ST32(&pMvd32[0],LD32(pMvd));
	ST32(&pMvd32[1],LD32(pMvd));
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	for(i=0; i<2; i++,iPartIdx+=4){
		const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t iScan4Idx4=4+iScan4Idx;
		const uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t iCacheIdx6=6+iCacheIdx;
		// mb
		ST64(pCurDqLayer->pMvd[iListIdx][iMbXy][iScan4Idx],LD64(pMvd32));
		ST64(pCurDqLayer->pMvd[iListIdx][iMbXy][iScan4Idx4],LD64(pMvd32));
		// cache
		ST64(pMvdCache[iListIdx][iCacheIdx],LD64(pMvd32));
		ST64(pMvdCache[iListIdx][iCacheIdx6],LD64(pMvd32));
	}
}

void UpdateP8x16RefIdxCabac(PDqLayer pCurDqLayer,int8_t pRefIndex[LIST_A][30],int32_t iPartIdx,const int8_t iRef,const int8_t iListIdx){
	uint16_t iRef16Bit=(uint16_t)iRef;
	const int16_t iRef2Bytes=(iRef16Bit<<8)|iRef16Bit;
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	for(i=0; i<2; i++,iPartIdx+=8){
		const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t iScan4Idx4=4+iScan4Idx;
		const uint8_t iCacheIdx6=6+iCacheIdx;
		// mb
		ST16(&pCurDqLayer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx],iRef2Bytes);
		ST16(&pCurDqLayer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx4],iRef2Bytes);
		// cache
		ST16(&pRefIndex[iListIdx][iCacheIdx],iRef2Bytes);
		ST16(&pRefIndex[iListIdx][iCacheIdx6],iRef2Bytes);
	}
}

void PredInter8x16Mv(int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVP[2]){
	if(0==iPartIdx){
		const int8_t kiLeftRef=iRefIndex[listIdx][6];
		if(iRef==kiLeftRef){
			ST32(iMVP,LD32(&iMotionVector[listIdx][6][0]));
			return;
		}
	}else{		// 1==iPartIdx
		int8_t iDiagonalRef=iRefIndex[listIdx][5];		// top-right
		int8_t index=5;
		if(REF_NOT_AVAIL==iDiagonalRef){
			iDiagonalRef=iRefIndex[listIdx][2];		// top-left for 8*8 block(index 1)
			index=2;
		}
		if(iRef==iDiagonalRef){
			ST32(iMVP,LD32(&iMotionVector[listIdx][index][0]));
			return;
		}
	}

	PredMv(iMotionVector,iRefIndex,listIdx,iPartIdx,2,iRef,iMVP);
}

// update iRefIndex and iMVs of both Mb and Mb_cache,only for P8x16
void UpdateP8x16MotionInfo(PDqLayer pCurDqLayer,int16_t iMotionVector[LIST_A][30][MV_A],int8_t iRefIndex[LIST_A][30],int32_t listIdx,int32_t iPartIdx,int8_t iRef,int16_t iMVs[2]){
	const int16_t kiRef2=((uint8_t)iRef<<8)|(uint8_t)iRef;
	const int32_t kiMV32=LD32(iMVs);
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;

	for(i=0; i<2; i++,iPartIdx+=8){
		const uint8_t kuiScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t kuiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t kuiScan4IdxPlus4=4+kuiScan4Idx;
		const uint8_t kuiCacheIdxPlus6=6+kuiCacheIdx;

		// mb
		if(pCurDqLayer->pDec!=NULL){
			ST16(&pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}else{
			ST16(&pCurDqLayer->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
			ST16(&pCurDqLayer->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}
		// cache
		ST16(&iRefIndex[listIdx][kuiCacheIdx],kiRef2);
		ST16(&iRefIndex[listIdx][kuiCacheIdxPlus6],kiRef2);
		ST32(iMotionVector[listIdx][kuiCacheIdx],kiMV32);
		ST32(iMotionVector[listIdx][1+kuiCacheIdx],kiMV32);
		ST32(iMotionVector[listIdx][kuiCacheIdxPlus6],kiMV32);
		ST32(iMotionVector[listIdx][1+kuiCacheIdxPlus6],kiMV32);
	}
}

void UpdateP8x16MvdCabac(SDqLayer* pCurDqLayer,int16_t pMvdCache[LIST_A][30][MV_A],int32_t iPartIdx,int16_t pMvd[2],const int8_t iListIdx){
	int32_t pMvd32[2];
	ST32(&pMvd32[0],LD32(pMvd));
	ST32(&pMvd32[1],LD32(pMvd));
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;

	for(i=0; i<2; i++,iPartIdx+=8){
		const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
		const uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
		const uint8_t iScan4Idx4=4+iScan4Idx;
		const uint8_t iCacheIdx6=6+iCacheIdx;
		// mb
		ST64(pCurDqLayer->pMvd[iListIdx][iMbXy][iScan4Idx],LD64(pMvd32));
		ST64(pCurDqLayer->pMvd[iListIdx][iMbXy][iScan4Idx4],LD64(pMvd32));
		// cache
		ST64(pMvdCache[iListIdx][iCacheIdx],LD64(pMvd32));
		ST64(pMvdCache[iListIdx][iCacheIdx6],LD64(pMvd32));
	}
}

int32_t ParseSubMBTypeCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint32_t& uiSubMbType){
	uint32_t uiCode;
	SWelsCabacDecEngine* pCabacDecEngine=pCtx->pCabacDecEngine;
	SWelsCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_SUBMB_TYPE;
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx,uiCode));
	if(uiCode)
		uiSubMbType=0;
	else{
		WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+1,uiCode));
		if(uiCode){
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+2,uiCode));
			uiSubMbType=3-uiCode;
		}else{
			uiSubMbType=1;
		}
	}
	return ERR_NONE;
}

// Table 7.17 Sub-macroblock types in B macroblocks.
static const SPartMbInfo g_ksInterPSubMbTypeInfo[4]={
	{SUB_MB_TYPE_8x8,1,2},
 {SUB_MB_TYPE_8x4,2,2},
 {SUB_MB_TYPE_4x8,2,1},
 {SUB_MB_TYPE_4x4,4,1},
};

void UpdateP8x8RefIdxCabac(PDqLayer pCurDqLayer,int8_t pRefIndex[LIST_A][30],int32_t iPartIdx,const int8_t iRef,const int8_t iListIdx){
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
	pCurDqLayer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx]=pCurDqLayer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx+1]=pCurDqLayer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx+4]=pCurDqLayer->pDec->pRefIndex[iListIdx][iMbXy][iScan4Idx+5]=iRef;
}

int32_t ParseInterPMotionInfoCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCount,int16_t pMotionVector[LIST_A][30][MV_A],int16_t pMvdCache[LIST_A][30][MV_A],int8_t pRefIndex[LIST_A][30]){
	SSlice* pSlice=&pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SPicture** ppRefPic=pCtx->sRefPic.pRefList[LIST_0];
	int32_t pRefCount[2];
	int32_t i,j;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int16_t pMv[4]={0};
	int16_t pMvd[4]={0};
	int8_t iRef[2]={0};
	int32_t iPartIdx;
	int16_t iMinVmv=pSliceHeader->pSps->pSLevelLimits->iMinVmv;
	int16_t iMaxVmv=pSliceHeader->pSps->pSLevelLimits->iMaxVmv;
	pRefCount[0]=pSliceHeader->uiRefCount[0];
	pRefCount[1]=pSliceHeader->uiRefCount[1];

	bool bIsPending=false;	// GetThreadCount (pCtx) > 1;

	switch(pCurDqLayer->pDec->pMbType[iMbXy]){
		case MB_TYPE_16x16:
		{
			iPartIdx=0;
			WELS_READ_VERIFY(ParseRefIdxCabac(pCtx,pNeighAvail,pNonZeroCount,pRefIndex,0,LIST_0,iPartIdx,pRefCount[0],0,iRef[0]));
			if((iRef[0]<0) || (iRef[0]>=pRefCount[0]) || (ppRefPic[iRef[0]]==NULL)){		// error ref_idx
				pCtx->bMbRefConcealed=true;
				if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
					iRef[0]=0;
					pCtx->iErrorCode|=dsBitstreamError;
				}else{
					return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
				}
			}
			pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[iRef[0]] && (ppRefPic[iRef[0]]->bIsComplete || bIsPending));
			PredMv(pMotionVector,pRefIndex,LIST_0,0,4,iRef[0],pMv);
			WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,0,pMvd[0]));
			WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,1,pMvd[1]));
			pMv[0]+=pMvd[0];
			pMv[1]+=pMvd[1];
			WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
			UpdateP16x16MotionInfo(pCurDqLayer,LIST_0,iRef[0],pMv);
			UpdateP16x16MvdCabac(pCurDqLayer,pMvd,LIST_0);
		}
		break;
		case MB_TYPE_16x8:
			for(i=0; i<2; i++){
				iPartIdx=i<<3;
				WELS_READ_VERIFY(ParseRefIdxCabac(pCtx,pNeighAvail,pNonZeroCount,pRefIndex,0,LIST_0,iPartIdx,pRefCount[0],0,iRef[i]));
				if((iRef[i]<0) || (iRef[i]>=pRefCount[0]) || (ppRefPic[iRef[i]]==NULL)){		// error ref_idx
					pCtx->bMbRefConcealed=true;
					if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
						iRef[i]=0;
						pCtx->iErrorCode|=dsBitstreamError;
					}else{
						return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
					}
				}
				pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[iRef[i]] && (ppRefPic[iRef[i]]->bIsComplete || bIsPending));
				UpdateP16x8RefIdxCabac(pCurDqLayer,pRefIndex,iPartIdx,iRef[i],LIST_0);
			}
			for(i=0; i<2; i++){
				iPartIdx=i<<3;
				PredInter16x8Mv(pMotionVector,pRefIndex,LIST_0,iPartIdx,iRef[i],pMv);
				WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,0,pMvd[0]));
				WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,1,pMvd[1]));
				pMv[0]+=pMvd[0];
				pMv[1]+=pMvd[1];
				WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
				UpdateP16x8MotionInfo(pCurDqLayer,pMotionVector,pRefIndex,LIST_0,iPartIdx,iRef[i],pMv);
				UpdateP16x8MvdCabac(pCurDqLayer,pMvdCache,iPartIdx,pMvd,LIST_0);
			}
			break;
		case MB_TYPE_8x16:
			for(i=0; i<2; i++){
				iPartIdx=i<<2;
				WELS_READ_VERIFY(ParseRefIdxCabac(pCtx,pNeighAvail,pNonZeroCount,pRefIndex,0,LIST_0,iPartIdx,pRefCount[0],0,iRef[i]));
				if((iRef[i]<0) || (iRef[i]>=pRefCount[0]) || (ppRefPic[iRef[i]]==NULL)){		// error ref_idx
					pCtx->bMbRefConcealed=true;
					if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
						iRef[i]=0;
						pCtx->iErrorCode|=dsBitstreamError;
					}else{
						return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
					}
				}
				pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[iRef[i]] && (ppRefPic[iRef[i]]->bIsComplete || bIsPending));
				UpdateP8x16RefIdxCabac(pCurDqLayer,pRefIndex,iPartIdx,iRef[i],LIST_0);
			}
			for(i=0; i<2; i++){
				iPartIdx=i<<2;
				PredInter8x16Mv(pMotionVector,pRefIndex,LIST_0,i<<2,iRef[i],pMv);

				WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,0,pMvd[0]));
				WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,1,pMvd[1]));
				pMv[0]+=pMvd[0];
				pMv[1]+=pMvd[1];
				WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
				UpdateP8x16MotionInfo(pCurDqLayer,pMotionVector,pRefIndex,LIST_0,iPartIdx,iRef[i],pMv);
				UpdateP8x16MvdCabac(pCurDqLayer,pMvdCache,iPartIdx,pMvd,LIST_0);
			}
			break;
		case MB_TYPE_8x8:
		case MB_TYPE_8x8_REF0:
		{
			int8_t pRefIdx[4]={0},pSubPartCount[4],pPartW[4];
			uint32_t uiSubMbType;
			// sub_mb_type,partition
			for(i=0; i<4; i++){
				WELS_READ_VERIFY(ParseSubMBTypeCabac(pCtx,pNeighAvail,uiSubMbType));
				if(uiSubMbType>=4){		// invalid sub_mb_type
					return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_SUB_MB_TYPE);
				}
				pCurDqLayer->pSubMbType[iMbXy][i]=g_ksInterPSubMbTypeInfo[uiSubMbType].iType;
				pSubPartCount[i]=g_ksInterPSubMbTypeInfo[uiSubMbType].iPartCount;
				pPartW[i]=g_ksInterPSubMbTypeInfo[uiSubMbType].iPartWidth;

				// Need modification when B picture add in,reference to 7.3.5
				pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]&=(uiSubMbType==0);
			}

			for(i=0; i<4; i++){
				int16_t iIdx8=i<<2;
				WELS_READ_VERIFY(ParseRefIdxCabac(pCtx,pNeighAvail,pNonZeroCount,pRefIndex,0,LIST_0,iIdx8,pRefCount[0],1,pRefIdx[i]));
				if((pRefIdx[i]<0) || (pRefIdx[i]>=pRefCount[0]) || (ppRefPic[pRefIdx[i]]==NULL)){		// error ref_idx
					pCtx->bMbRefConcealed=true;
					if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
						pRefIdx[i]=0;
						pCtx->iErrorCode|=dsBitstreamError;
					}else{
						return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
					}
				}
				pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[pRefIdx[i]] && (ppRefPic[pRefIdx[i]]->bIsComplete || bIsPending));
				UpdateP8x8RefIdxCabac(pCurDqLayer,pRefIndex,iIdx8,pRefIdx[i],LIST_0);
			}
			// mv
			for(i=0; i<4; i++){
				int8_t iPartCount=pSubPartCount[i];
				uiSubMbType=pCurDqLayer->pSubMbType[iMbXy][i];
				int16_t iPartIdx,iBlockW=pPartW[i];
				uint8_t iScan4Idx,iCacheIdx;
				iCacheIdx=g_kuiCache30ScanIdx[i<<2];
				pRefIndex[0][iCacheIdx]=pRefIndex[0][iCacheIdx+1]
					=pRefIndex[0][iCacheIdx+6]=pRefIndex[0][iCacheIdx+7]=pRefIdx[i];

				for(j=0; j<iPartCount; j++){
					iPartIdx=(i<<2)+j*iBlockW;
					iScan4Idx=g_kuiScan4[iPartIdx];
					iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
					PredMv(pMotionVector,pRefIndex,LIST_0,iPartIdx,iBlockW,pRefIdx[i],pMv);
					WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,0,pMvd[0]));
					WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,LIST_0,1,pMvd[1]));
					pMv[0]+=pMvd[0];
					pMv[1]+=pMvd[1];
					WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
					if(SUB_MB_TYPE_8x8==uiSubMbType){
						ST32((pMv+2),LD32(pMv));
						ST32((pMvd+2),LD32(pMvd));
						ST64(pCurDqLayer->pDec->pMv[0][iMbXy][iScan4Idx],LD64(pMv));
						ST64(pCurDqLayer->pDec->pMv[0][iMbXy][iScan4Idx+4],LD64(pMv));
						ST64(pCurDqLayer->pMvd[0][iMbXy][iScan4Idx],LD64(pMvd));
						ST64(pCurDqLayer->pMvd[0][iMbXy][iScan4Idx+4],LD64(pMvd));
						ST64(pMotionVector[0][iCacheIdx],LD64(pMv));
						ST64(pMotionVector[0][iCacheIdx+6],LD64(pMv));
						ST64(pMvdCache[0][iCacheIdx],LD64(pMvd));
						ST64(pMvdCache[0][iCacheIdx+6],LD64(pMvd));
					}else
					if(SUB_MB_TYPE_8x4==uiSubMbType){
						ST32((pMv+2),LD32(pMv));
						ST32((pMvd+2),LD32(pMvd));
						ST64(pCurDqLayer->pDec->pMv[0][iMbXy][iScan4Idx],LD64(pMv));
						ST64(pCurDqLayer->pMvd[0][iMbXy][iScan4Idx],LD64(pMvd));
						ST64(pMotionVector[0][iCacheIdx],LD64(pMv));
						ST64(pMvdCache[0][iCacheIdx],LD64(pMvd));
					}else
					if(SUB_MB_TYPE_4x8==uiSubMbType){
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][iScan4Idx],LD32(pMv));
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][iScan4Idx+4],LD32(pMv));
						ST32(pCurDqLayer->pMvd[0][iMbXy][iScan4Idx],LD32(pMvd));
						ST32(pCurDqLayer->pMvd[0][iMbXy][iScan4Idx+4],LD32(pMvd));
						ST32(pMotionVector[0][iCacheIdx],LD32(pMv));
						ST32(pMotionVector[0][iCacheIdx+6],LD32(pMv));
						ST32(pMvdCache[0][iCacheIdx],LD32(pMvd));
						ST32(pMvdCache[0][iCacheIdx+6],LD32(pMvd));
					}else{		// SUB_MB_TYPE_4x4
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][iScan4Idx],LD32(pMv));
						ST32(pCurDqLayer->pMvd[0][iMbXy][iScan4Idx],LD32(pMvd));
						ST32(pMotionVector[0][iCacheIdx],LD32(pMv));
						ST32(pMvdCache[0][iCacheIdx],LD32(pMvd));
					}
				}
			}
		}
		break;
		default:
			break;
	}
	return ERR_NONE;
}

void RestoreCabacDecEngineToBS(SWelsCabacDecEngine* pDecEngine,SBitStringAux* pBsAux){
	// CABAC decoding finished,changing to SBitStringAux
	pDecEngine->pBuffCurr-=(pDecEngine->iBitsLeft>>3);
	pDecEngine->iBitsLeft=0;		// pcm_alignment_zero_bit in CABAC
	pBsAux->iLeftBits=0;
	pBsAux->pStartBuf=pDecEngine->pBuffStart;
	pBsAux->pCurBuf=pDecEngine->pBuffCurr;
	pBsAux->uiCurBits=0;
	pBsAux->iIndex=0;
}

// Get unsigned truncated exp golomb code.
static inline int32_t BsGetTe0(SBitStringAux* pBs,int32_t iRange,uint32_t* pCode){
	if(iRange==1){
		*pCode=0;
	}else
	if(iRange==2){
		WELS_READ_VERIFY(BsGetOneBit(pBs,pCode));
		*pCode^=1;
	}else{
		WELS_READ_VERIFY(BsGetUe(pBs,pCode));
	}
	return ERR_NONE;
}


// Get number of trailing bits
static inline int32_t BsGetTrailingBits(uint8_t* pBuf){
	// TODO
	uint32_t uiValue=*pBuf;
	int32_t iRetNum=0;

	do{
		if(uiValue&1)
			return iRetNum;
		uiValue>>=1;
		++iRetNum;
	} while(iRetNum<9);

	return 0;
}

// -------------------2. decoding Engine initialization
int32_t InitCabacDecEngineFromBS(SWelsCabacDecEngine* pDecEngine,SBitStringAux* pBsAux){
	int32_t iRemainingBits=-pBsAux->iLeftBits;		// pBsAux->iLeftBits < 0
	int32_t iRemainingBytes=(iRemainingBits>>3)+2;		// +2: indicating the pre-read 2 bytes
	uint8_t* pCurr;

	pCurr=pBsAux->pCurBuf-iRemainingBytes;
	if(pCurr>=(pBsAux->pEndBuf-1)){
		return ERR_INFO_INVALID_ACCESS;
	}
	pDecEngine->uiOffset=((pCurr[0]<<16)|(pCurr[1]<<8)|pCurr[2]);
	pDecEngine->uiOffset<<=16;
	pDecEngine->uiOffset|=(pCurr[3]<<8)|pCurr[4];
	pDecEngine->iBitsLeft=31;
	pDecEngine->pBuffCurr=pCurr+5;

	pDecEngine->uiRange=WELS_CABAC_HALF;
	pDecEngine->pBuffStart=pBsAux->pStartBuf;
	pDecEngine->pBuffEnd=pBsAux->pEndBuf;
	pBsAux->iLeftBits=0;
	return ERR_NONE;
}

int32_t ParseIPCMInfoCabac(SDecoderContext* pCtx){
	int32_t i;
	SWelsCabacDecEngine* pCabacDecEngine=pCtx->pCabacDecEngine;
	SBitStringAux* pBsAux=pCtx->pCurDqLayer->pBitStringAux;
	SDqLayer* pCurDqLayer=pCtx->pCurDqLayer;
	int32_t iDstStrideLuma=pCurDqLayer->pDec->iLinesize[0];
	int32_t iDstStrideChroma=pCurDqLayer->pDec->iLinesize[1];
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;

	int32_t iMbOffsetLuma=(iMbX+iMbY*iDstStrideLuma)<<4;
	int32_t iMbOffsetChroma=(iMbX+iMbY*iDstStrideChroma)<<3;

	uint8_t* pMbDstY=pCtx->pDec->pData[0]+iMbOffsetLuma;
	uint8_t* pMbDstU=pCtx->pDec->pData[1]+iMbOffsetChroma;
	uint8_t* pMbDstV=pCtx->pDec->pData[2]+iMbOffsetChroma;

	uint8_t* pPtrSrc;

	pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA_PCM;
	RestoreCabacDecEngineToBS(pCabacDecEngine,pBsAux);
	intX_t iBytesLeft=pBsAux->pEndBuf-pBsAux->pCurBuf;
	if(iBytesLeft<384){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_CABAC_NO_BS_TO_READ);
	}
	pPtrSrc=pBsAux->pCurBuf;
	for(i=0; i<16; i++){		// luma
		memcpy(pMbDstY,pPtrSrc,16);
		pMbDstY+=iDstStrideLuma;
		pPtrSrc+=16;
	}
	for(i=0; i<8; i++){		// cb
		memcpy(pMbDstU,pPtrSrc,8);
		pMbDstU+=iDstStrideChroma;
		pPtrSrc+=8;
	}
	for(i=0; i<8; i++){		// cr
		memcpy(pMbDstV,pPtrSrc,8);
		pMbDstV+=iDstStrideChroma;
		pPtrSrc+=8;
	}

	pBsAux->pCurBuf+=384;

	pCurDqLayer->pLumaQp[iMbXy]=0;
	pCurDqLayer->pChromaQp[iMbXy][0]=pCurDqLayer->pChromaQp[iMbXy][1]=0;
	memset(pCurDqLayer->pNzc[iMbXy],16,sizeof(pCurDqLayer->pNzc[iMbXy]));

	// step 4: cabac engine init
	WELS_READ_VERIFY(InitReadBits(pBsAux,1));
	WELS_READ_VERIFY(InitCabacDecEngineFromBS(pCabacDecEngine,pBsAux));
	return ERR_NONE;
}

int32_t ParseTransformSize8x8FlagCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,
										bool& bTransformSize8x8Flag){
	uint32_t uiCode;
	int32_t iIdxA,iIdxB;
	int32_t iCtxInc;
	SWelsCabacDecEngine* pCabacDecEngine=pCtx->pCabacDecEngine;
	SWelsCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_TS_8x8_FLAG;
	iIdxA=(pNeighAvail->iLeftAvail) && (pCtx->pCurDqLayer->pTransformSize8x8Flag[pCtx->pCurDqLayer->iMbXyIndex-1]);
	iIdxB=(pNeighAvail->iTopAvail) && (pCtx->pCurDqLayer->pTransformSize8x8Flag[pCtx->pCurDqLayer->iMbXyIndex-pCtx->pCurDqLayer->iMbWidth]);
	iCtxInc=iIdxA+iIdxB;
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+iCtxInc,uiCode));
	bTransformSize8x8Flag=!!uiCode;

	return ERR_NONE;
}

int32_t ParseIntraPredModeLumaCabac(SDecoderContext* pCtx,int32_t& iBinVal){
	uint32_t uiCode;
	iBinVal=0;
	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_IPR,uiCode));
	if(uiCode==1)
		iBinVal=-1;
	else{
		WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_IPR+1,uiCode));
		iBinVal|=uiCode;
		WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_IPR+1,uiCode));
		iBinVal|=(uiCode<<1);
		WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_IPR+1,uiCode));
		iBinVal|=(uiCode<<2);
	}
	return ERR_NONE;
}

int32_t PredIntra4x4Mode(int8_t* pIntraPredMode,int32_t iIdx4){
	int8_t iTopMode=pIntraPredMode[g_kuiScan8[iIdx4]-8];
	int8_t iLeftMode=pIntraPredMode[g_kuiScan8[iIdx4]-1];
	int8_t iBestMode;

	if(-1==iLeftMode || -1==iTopMode){
		iBestMode=2;
	}else{
		iBestMode=WELS_MIN(iLeftMode,iTopMode);
	}
	return iBestMode;
}


typedef struct TagI16PredInfo{
	int8_t iPredMode;
	int8_t iLeftAvail;
	int8_t iTopAvail;
	int8_t iLeftTopAvail;
} SI16PredInfo;
static const SI16PredInfo g_ksI16PredInfo[4]={
	{I16_PRED_V,0,1,0},
 {I16_PRED_H,1,0,0},
 {0,0,0,0},
 {I16_PRED_P,1,1,1},
};

static const SI16PredInfo g_ksChromaPredInfo[4]={
	{0,0,0,0},
 {C_PRED_H,1,0,0},
 {C_PRED_V,0,1,0},
 {C_PRED_P,1,1,1},
};


typedef struct TagI4PredInfo{
	int8_t iPredMode;
	int8_t iLeftAvail;
	int8_t iTopAvail;
	int8_t iLeftTopAvail;
	// int8_t right_top_avail;		// when right_top unavailable but top avail,we can pad the right-top with the rightmost pixel of top
} SI4PredInfo;
static const SI4PredInfo g_ksI4PredInfo[9]={
	{I4_PRED_V,0,1,0},
 {I4_PRED_H,1,0,0},
 {0,0,0,0},
 {I4_PRED_DDL,0,1,0},
 {I4_PRED_DDR,1,1,1},
 {I4_PRED_VR,1,1,1},
 {I4_PRED_HD,1,1,1},
 {I4_PRED_VL,0,1,0},
 {I4_PRED_HU,1,0,0},
};

#define CHECK_I16_MODE(a,b,c,d) \
 ((a==g_ksI16PredInfo[a].iPredMode) && \
 (b >=g_ksI16PredInfo[a].iLeftAvail) && \
 (c >=g_ksI16PredInfo[a].iTopAvail) && \
 (d >=g_ksI16PredInfo[a].iLeftTopAvail));
#define CHECK_CHROMA_MODE(a,b,c,d) \
 ((a==g_ksChromaPredInfo[a].iPredMode) && \
 (b >=g_ksChromaPredInfo[a].iLeftAvail) && \
 (c >=g_ksChromaPredInfo[a].iTopAvail) && \
 (d >=g_ksChromaPredInfo[a].iLeftTopAvail));
#define CHECK_I4_MODE(a,b,c,d) \
 ((a==g_ksI4PredInfo[a].iPredMode) && \
 (b >=g_ksI4PredInfo[a].iLeftAvail) && \
 (c >=g_ksI4PredInfo[a].iTopAvail) && \
 (d >=g_ksI4PredInfo[a].iLeftTopAvail));


int32_t CheckIntraNxNPredMode(int32_t* pSampleAvail,int8_t* pMode,int32_t iIndex,bool b8x8){
	int8_t iIdx=g_kuiCache30ScanIdx[iIndex];

	int32_t iLeftAvail=pSampleAvail[iIdx-1];
	int32_t iTopAvail=pSampleAvail[iIdx-6];
	int32_t bLeftTopAvail=pSampleAvail[iIdx-7];
	int32_t bRightTopAvail=pSampleAvail[iIdx-(b8x8 ? 4 : 5)];		// Diff with 4x4 Pred

	int8_t iFinalMode;

	if((*pMode<0) || (*pMode>MAX_PRED_MODE_ID_I4x4)){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INVALID_INTRA4X4_MODE);
	}

	if(I4_PRED_DC==*pMode){
		if(iLeftAvail && iTopAvail){
			return *pMode;
		}else
		if(iLeftAvail){
			iFinalMode=I4_PRED_DC_L;
		}else
		if(iTopAvail){
			iFinalMode=I4_PRED_DC_T;
		}else{
			iFinalMode=I4_PRED_DC_128;
		}
	}else{
		bool bModeAvail=CHECK_I4_MODE(*pMode,iLeftAvail,iTopAvail,bLeftTopAvail);
		if(0==bModeAvail){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INVALID_INTRA4X4_MODE);
		}

		iFinalMode=*pMode;

		// if right-top unavailable,modify mode DDL and VL (padding rightmost pixel of top)
		if(I4_PRED_DDL==iFinalMode && 0==bRightTopAvail){
			iFinalMode=I4_PRED_DDL_TOP;
		}else
		if(I4_PRED_VL==iFinalMode && 0==bRightTopAvail){
			iFinalMode=I4_PRED_VL_TOP;
		}
	}
	return iFinalMode;
}


int32_t ParseIntraPredModeChromaCabac(SDecoderContext* pCtx,uint8_t uiNeighAvail,int32_t& iBinVal){
	uint32_t uiCode;
	int32_t iIdxA,iIdxB,iCtxInc;
	int8_t* pChromaPredMode=pCtx->pCurDqLayer->pChromaPredMode;
	uint32_t* pMbType=pCtx->pCurDqLayer->pDec->pMbType;
	int32_t iLeftAvail=uiNeighAvail&0x04;
	int32_t iTopAvail=uiNeighAvail&0x01;

	int32_t iMbXy=pCtx->pCurDqLayer->iMbXyIndex;
	int32_t iMbXyTop=iMbXy-pCtx->pCurDqLayer->iMbWidth;
	int32_t iMbXyLeft=iMbXy-1;

	iBinVal=0;

	iIdxB=iTopAvail && (pChromaPredMode[iMbXyTop]>0 && pChromaPredMode[iMbXyTop]<=3)
		 && pMbType[iMbXyTop]!=MB_TYPE_INTRA_PCM;
	iIdxA=iLeftAvail && (pChromaPredMode[iMbXyLeft]>0 && pChromaPredMode[iMbXyLeft]<=3)
		 && pMbType[iMbXyLeft]!=MB_TYPE_INTRA_PCM;
	iCtxInc=iIdxA+iIdxB;
	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CIPR+iCtxInc,uiCode));
	iBinVal=uiCode;
	if(iBinVal!=0){
		uint32_t iSym;
		WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CIPR+3,iSym));
		if(iSym==0){
			iBinVal=(iSym+1);
			return ERR_NONE;
		}
		iSym=0;
		do{
			WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CIPR+3,uiCode));
			++iSym;
		} while((uiCode!=0) && (iSym<1));

		if((uiCode!=0) && (iSym==1))
			++iSym;
		iBinVal=(iSym+1);
		return ERR_NONE;
	}
	return ERR_NONE;
}

int32_t CheckIntraChromaPredMode(uint8_t uiSampleAvail,int8_t* pMode){
	int32_t iLeftAvail=uiSampleAvail&0x04;
	int32_t bLeftTopAvail=uiSampleAvail&0x02;
	int32_t iTopAvail=uiSampleAvail&0x01;

	if(C_PRED_DC==*pMode){
		if(iLeftAvail && iTopAvail){
			return ERR_NONE;
		}else
		if(iLeftAvail){
			*pMode=C_PRED_DC_L;
		}else
		if(iTopAvail){
			*pMode=C_PRED_DC_T;
		}else{
			*pMode=C_PRED_DC_128;
		}
	}else{
		bool bModeAvail=CHECK_CHROMA_MODE(*pMode,iLeftAvail,iTopAvail,bLeftTopAvail);
		if(0==bModeAvail){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
		}
	}
	return ERR_NONE;
}

int32_t ParseIntra8x8Mode(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,int8_t* pIntraPredMode,SBitStringAux* pBs,PDqLayer pCurDqLayer){
	// Similar with Intra_4x4,can put them together when needed
	int32_t iSampleAvail[5*6]={0};		// initialize as 0
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int32_t iFinalMode,i;

	uint8_t uiNeighAvail=0;
	uint32_t uiCode;
	int32_t iCode;
	pCtx->pMapNxNNeighToSampleFunc(pNeighAvail,iSampleAvail);
	// Top-Right : Left : Top-Left : Top
	uiNeighAvail=(iSampleAvail[5]<<3)|(iSampleAvail[6]<<2)|(iSampleAvail[0]<<1)|(iSampleAvail[1]);

	pCurDqLayer->pIntraNxNAvailFlag[iMbXy]=uiNeighAvail;

	for(i=0; i<4; i++){
		int32_t iPrevIntra4x4PredMode=0;
		if(pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag){
			WELS_READ_VERIFY(ParseIntraPredModeLumaCabac(pCtx,iCode));
			iPrevIntra4x4PredMode=iCode;
		}else{
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));
			iPrevIntra4x4PredMode=uiCode;
		}
		const int32_t kiPredMode=PredIntra4x4Mode(pIntraPredMode,i<<2);

		int8_t iBestMode;
		if(pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag){
			if(iPrevIntra4x4PredMode==-1)
				iBestMode=kiPredMode;
			else
				iBestMode=iPrevIntra4x4PredMode+(iPrevIntra4x4PredMode>=kiPredMode);
		}else{
			if(iPrevIntra4x4PredMode){
				iBestMode=kiPredMode;
			}else{
				WELS_READ_VERIFY(BsGetBits(pBs,3,&uiCode));
				iBestMode=uiCode+((int32_t)uiCode>=kiPredMode);
			}
		}

		iFinalMode=CheckIntraNxNPredMode(&iSampleAvail[0],&iBestMode,i<<2,true);

		if(iFinalMode==GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INVALID_INTRA4X4_MODE)){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I4x4_PRED_MODE);
		}

		for(int j=0; j<4; j++){
			pCurDqLayer->pIntra4x4FinalMode[iMbXy][g_kuiScan4[(i<<2)+j]]=iFinalMode;
			pIntraPredMode[g_kuiScan8[(i<<2)+j]]=iBestMode;
			iSampleAvail[g_kuiCache30ScanIdx[(i<<2)+j]]=1;
		}
	}
	ST32(&pCurDqLayer->pIntraPredMode[iMbXy][0],LD32(&pIntraPredMode[1+8*4]));
	pCurDqLayer->pIntraPredMode[iMbXy][4]=pIntraPredMode[4+8*1];
	pCurDqLayer->pIntraPredMode[iMbXy][5]=pIntraPredMode[4+8*2];
	pCurDqLayer->pIntraPredMode[iMbXy][6]=pIntraPredMode[4+8*3];

	if(pCtx->pSps->uiChromaFormatIdc==0)
		return ERR_NONE;

	if(pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag){
		WELS_READ_VERIFY(ParseIntraPredModeChromaCabac(pCtx,uiNeighAvail,iCode));
		if(iCode>MAX_PRED_MODE_ID_CHROMA){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
		}
		pCurDqLayer->pChromaPredMode[iMbXy]=iCode;
	}else{
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// intra_chroma_pred_mode
		if(uiCode>MAX_PRED_MODE_ID_CHROMA){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
		}
		pCurDqLayer->pChromaPredMode[iMbXy]=uiCode;
	}

	if(-1==pCurDqLayer->pChromaPredMode[iMbXy]
		 || CheckIntraChromaPredMode(uiNeighAvail,&pCurDqLayer->pChromaPredMode[iMbXy])){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
	}

	return ERR_NONE;
}

int32_t ParseIntra4x4Mode(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,int8_t* pIntraPredMode,SBitStringAux* pBs,PDqLayer pCurDqLayer){
	int32_t iSampleAvail[5*6]={0};		// initialize as 0
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int32_t iFinalMode,i;

	uint8_t uiNeighAvail=0;
	uint32_t uiCode;
	int32_t iCode;
	pCtx->pMapNxNNeighToSampleFunc(pNeighAvail,iSampleAvail);
	uiNeighAvail=(iSampleAvail[6]<<2)|(iSampleAvail[0]<<1)|(iSampleAvail[1]);
	for(i=0; i<16; i++){
		int32_t iPrevIntra4x4PredMode=0;
		if(pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag){
			WELS_READ_VERIFY(ParseIntraPredModeLumaCabac(pCtx,iCode));
			iPrevIntra4x4PredMode=iCode;
		}else{
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));
			iPrevIntra4x4PredMode=uiCode;
		}
		const int32_t kiPredMode=PredIntra4x4Mode(pIntraPredMode,i);

		int8_t iBestMode;
		if(pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag){
			if(iPrevIntra4x4PredMode==-1)
				iBestMode=kiPredMode;
			else
				iBestMode=iPrevIntra4x4PredMode+(iPrevIntra4x4PredMode>=kiPredMode);
		}else{
			if(iPrevIntra4x4PredMode){
				iBestMode=kiPredMode;
			}else{
				WELS_READ_VERIFY(BsGetBits(pBs,3,&uiCode));
				iBestMode=uiCode+((int32_t)uiCode>=kiPredMode);
			}
		}

		iFinalMode=CheckIntraNxNPredMode(&iSampleAvail[0],&iBestMode,i,false);
		if(iFinalMode==GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INVALID_INTRA4X4_MODE)){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I4x4_PRED_MODE);
		}

		pCurDqLayer->pIntra4x4FinalMode[iMbXy][g_kuiScan4[i]]=iFinalMode;

		pIntraPredMode[g_kuiScan8[i]]=iBestMode;

		iSampleAvail[g_kuiCache30ScanIdx[i]]=1;
	}
	ST32(&pCurDqLayer->pIntraPredMode[iMbXy][0],LD32(&pIntraPredMode[1+8*4]));
	pCurDqLayer->pIntraPredMode[iMbXy][4]=pIntraPredMode[4+8*1];
	pCurDqLayer->pIntraPredMode[iMbXy][5]=pIntraPredMode[4+8*2];
	pCurDqLayer->pIntraPredMode[iMbXy][6]=pIntraPredMode[4+8*3];

	if(pCtx->pSps->uiChromaFormatIdc==0)// no need parse chroma
		return ERR_NONE;

	if(pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag){
		WELS_READ_VERIFY(ParseIntraPredModeChromaCabac(pCtx,uiNeighAvail,iCode));
		if(iCode>MAX_PRED_MODE_ID_CHROMA){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
		}
		pCurDqLayer->pChromaPredMode[iMbXy]=iCode;
	}else{
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// intra_chroma_pred_mode
		if(uiCode>MAX_PRED_MODE_ID_CHROMA){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
		}
		pCurDqLayer->pChromaPredMode[iMbXy]=uiCode;
	}

	if(-1==pCurDqLayer->pChromaPredMode[iMbXy]
		 || CheckIntraChromaPredMode(uiNeighAvail,&pCurDqLayer->pChromaPredMode[iMbXy])){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
	}
	return ERR_NONE;
}

int32_t CheckIntra16x16PredMode(uint8_t uiSampleAvail,int8_t* pMode){
	int32_t iLeftAvail=uiSampleAvail&0x04;
	int32_t bLeftTopAvail=uiSampleAvail&0x02;
	int32_t iTopAvail=uiSampleAvail&0x01;

	if((*pMode<0) || (*pMode>MAX_PRED_MODE_ID_I16x16)){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I16x16_PRED_MODE);
	}

	if(I16_PRED_DC==*pMode){
		if(iLeftAvail && iTopAvail){
			return ERR_NONE;
		}else
		if(iLeftAvail){
			*pMode=I16_PRED_DC_L;
		}else
		if(iTopAvail){
			*pMode=I16_PRED_DC_T;
		}else{
			*pMode=I16_PRED_DC_128;
		}
	}else{
		bool bModeAvail=CHECK_I16_MODE(*pMode,iLeftAvail,iTopAvail,bLeftTopAvail);
		if(0==bModeAvail){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I16x16_PRED_MODE);
		}
	}
	return ERR_NONE;
}

int32_t ParseIntra16x16Mode(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,SBitStringAux* pBs,PDqLayer pCurDqLayer){
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	uint8_t uiNeighAvail=0;		// 0x07=0 1 1 1,means left,top-left,top avail or not. (1: avail,0: unavail)
	uint32_t uiCode;
	int32_t iCode;
	pCtx->pMap16x16NeighToSampleFunc(pNeighAvail,&uiNeighAvail);

	if(CheckIntra16x16PredMode(uiNeighAvail,&pCurDqLayer->pIntraPredMode[iMbXy][7])){		// invalid iPredMode,must stop decoding
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I16x16_PRED_MODE);
	}
	if(pCtx->pSps->uiChromaFormatIdc==0)
		return ERR_NONE;

	if(pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag){
		WELS_READ_VERIFY(ParseIntraPredModeChromaCabac(pCtx,uiNeighAvail,iCode));
		if(iCode>MAX_PRED_MODE_ID_CHROMA){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
		}
		pCurDqLayer->pChromaPredMode[iMbXy]=iCode;
	}else{
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// intra_chroma_pred_mode
		if(uiCode>MAX_PRED_MODE_ID_CHROMA){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
		}
		pCurDqLayer->pChromaPredMode[iMbXy]=uiCode;
	}
	if(-1==pCurDqLayer->pChromaPredMode[iMbXy]
		 || CheckIntraChromaPredMode(uiNeighAvail,&pCurDqLayer->pChromaPredMode[iMbXy])){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_I_CHROMA_PRED_MODE);
	}

	return ERR_NONE;
}

static const uint8_t g_kuiI16CbpTable[6]={0,16,32,15,31,47};

int32_t ParseCbpInfoCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint32_t& uiCbp){
	int32_t iIdxA=0,iIdxB=0,pALeftMb[2],pBTopMb[2];
	uiCbp=0;
	uint32_t pCbpBit[6];
	int32_t iCtxInc;

	// Luma: bit by bit for 4 8x8 blocks in z-order
	pBTopMb[0]=pNeighAvail->iTopAvail && pNeighAvail->iTopType!=MB_TYPE_INTRA_PCM
		 && ((pNeighAvail->iTopCbp&(1<<2))==0);
	pBTopMb[1]=pNeighAvail->iTopAvail && pNeighAvail->iTopType!=MB_TYPE_INTRA_PCM
		 && ((pNeighAvail->iTopCbp&(1<<3))==0);
	pALeftMb[0]=pNeighAvail->iLeftAvail && pNeighAvail->iLeftType!=MB_TYPE_INTRA_PCM
		 && ((pNeighAvail->iLeftCbp&(1<<1))==0);
	pALeftMb[1]=pNeighAvail->iLeftAvail && pNeighAvail->iLeftType!=MB_TYPE_INTRA_PCM
		 && ((pNeighAvail->iLeftCbp&(1<<3))==0);

	// left_top 8x8 block
	iCtxInc=pALeftMb[0]+(pBTopMb[0]<<1);
	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CBP+iCtxInc,pCbpBit[0]));
	if(pCbpBit[0])
		uiCbp+=0x01;

	// right_top 8x8 block
	iIdxA=!pCbpBit[0];
	iCtxInc=iIdxA+(pBTopMb[1]<<1);
	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CBP+iCtxInc,pCbpBit[1]));
	if(pCbpBit[1])
		uiCbp+=0x02;

	// left_bottom 8x8 block
	iIdxB=!pCbpBit[0];
	iCtxInc=pALeftMb[1]+(iIdxB<<1);
	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CBP+iCtxInc,pCbpBit[2]));
	if(pCbpBit[2])
		uiCbp+=0x04;

	// right_bottom 8x8 block
	iIdxB=!pCbpBit[1];
	iIdxA=!pCbpBit[2];
	iCtxInc=iIdxA+(iIdxB<<1);
	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CBP+iCtxInc,pCbpBit[3]));
	if(pCbpBit[3])
		uiCbp+=0x08;

	if(pCtx->pSps->uiChromaFormatIdc==0)// monochroma
		return ERR_NONE;


	// Chroma: bit by bit
	iIdxB=pNeighAvail->iTopAvail && (pNeighAvail->iTopType==MB_TYPE_INTRA_PCM || (pNeighAvail->iTopCbp>>4));
	iIdxA=pNeighAvail->iLeftAvail && (pNeighAvail->iLeftType==MB_TYPE_INTRA_PCM || (pNeighAvail->iLeftCbp>>4));

	// BitIdx=0
	iCtxInc=iIdxA+(iIdxB<<1);
	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CBP+CTX_NUM_CBP+iCtxInc,pCbpBit[4]));

	// BitIdx=1
	if(pCbpBit[4]){
		iIdxB=pNeighAvail->iTopAvail && (pNeighAvail->iTopType==MB_TYPE_INTRA_PCM || (pNeighAvail->iTopCbp>>4)==2);
		iIdxA=pNeighAvail->iLeftAvail && (pNeighAvail->iLeftType==MB_TYPE_INTRA_PCM || (pNeighAvail->iLeftCbp>>4)==2);
		iCtxInc=iIdxA+(iIdxB<<1);
		WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CBP+2*CTX_NUM_CBP+iCtxInc,pCbpBit[5]));
		uiCbp+=1<<(4+pCbpBit[5]);

	}

	return ERR_NONE;
}

int32_t ParseDeltaQpCabac(SDecoderContext* pCtx,int32_t& iQpDelta){
	uint32_t uiCode;
	SSlice* pCurrSlice=&(pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer);
	iQpDelta=0;
	SWelsCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_DELTA_QP;
	int32_t iCtxInc=(pCurrSlice->iLastDeltaQp!=0);
	WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pBinCtx+iCtxInc,uiCode));
	if(uiCode!=0){
		WELS_READ_VERIFY(DecodeUnaryBinCabac(pCtx->pCabacDecEngine,pBinCtx+2,1,uiCode));
		uiCode++;
		iQpDelta=(uiCode+1)>>1;
		if((uiCode&1)==0)
			iQpDelta=-iQpDelta;
	}
	pCurrSlice->iLastDeltaQp=iQpDelta;
	return ERR_NONE;
}

const uint8_t g_kuiLumaDcZigzagScan[16]={
	0,16,32,128,		// 0*16+0*64,1*16+0*64,2*16+0*64,0*16+2*64,
	48,64,80,96,		// 3*16+0*64,0*16+1*64,1*16+1*64,2*16+1*64,
	144,160,176,192,		// 1*16+2*64,2*16+2*64,3*16+2*64,0*16+3*64,
	112,208,224,240		// 3*16+1*64,1*16+3*64,2*16+3*64,3*16+3*64,
};

static inline void GetMbResProperty(int32_t* pMBproperty,int32_t* pResidualProperty,bool bCavlc){
	switch(*pResidualProperty){
		case CHROMA_AC_U:
			*pMBproperty=1;
			*pResidualProperty=bCavlc ? CHROMA_AC : CHROMA_AC_U;
			break;
		case CHROMA_AC_V:
			*pMBproperty=2;
			*pResidualProperty=bCavlc ? CHROMA_AC : CHROMA_AC_V;
			break;
		case LUMA_DC_AC_INTRA:
			*pMBproperty=0;
			*pResidualProperty=LUMA_DC_AC;
			break;
		case CHROMA_DC_U:
			*pMBproperty=1;
			*pResidualProperty=bCavlc ? CHROMA_DC : CHROMA_DC_U;
			break;
		case CHROMA_DC_V:
			*pMBproperty=2;
			*pResidualProperty=bCavlc ? CHROMA_DC : CHROMA_DC_V;
			break;
		case I16_LUMA_AC:
			*pMBproperty=0;
			break;
		case I16_LUMA_DC:
			*pMBproperty=0;
			break;
		case LUMA_DC_AC_INTER:
			*pMBproperty=3;
			*pResidualProperty=LUMA_DC_AC;
			break;
		case CHROMA_DC_U_INTER:
			*pMBproperty=4;
			*pResidualProperty=bCavlc ? CHROMA_DC : CHROMA_DC_U;
			break;
		case CHROMA_DC_V_INTER:
			*pMBproperty=5;
			*pResidualProperty=bCavlc ? CHROMA_DC : CHROMA_DC_V;
			break;
		case CHROMA_AC_U_INTER:
			*pMBproperty=4;
			*pResidualProperty=bCavlc ? CHROMA_AC : CHROMA_AC_U;
			break;
		case CHROMA_AC_V_INTER:
			*pMBproperty=5;
			*pResidualProperty=bCavlc ? CHROMA_AC : CHROMA_AC_V;
			break;
			// Reference to Table 7-2
		case LUMA_DC_AC_INTRA_8:
			*pMBproperty=6;
			*pResidualProperty=LUMA_DC_AC_8;
			break;
		case LUMA_DC_AC_INTER_8:
			*pMBproperty=7;
			*pResidualProperty=LUMA_DC_AC_8;
			break;
	}
}

ALIGNED_DECLARE(const uint16_t,g_kuiDequantCoeff[52][8],16)={
	/* 0*/{10,13,10,13,13,16,13,16},/* 1*/{11,14,11,14,14,18,14,18},
	/* 2*/{13,16,13,16,16,20,16,20},/* 3*/{14,18,14,18,18,23,18,23},
	/* 4*/{16,20,16,20,20,25,20,25},/* 5*/{18,23,18,23,23,29,23,29},
	/* 6*/{20,26,20,26,26,32,26,32},/* 7*/{22,28,22,28,28,36,28,36},
	/* 8*/{26,32,26,32,32,40,32,40},/* 9*/{28,36,28,36,36,46,36,46},
	/*10*/{32,40,32,40,40,50,40,50},/*11*/{36,46,36,46,46,58,46,58},
	/*12*/{40,52,40,52,52,64,52,64},/*13*/{44,56,44,56,56,72,56,72},
	/*14*/{52,64,52,64,64,80,64,80},/*15*/{56,72,56,72,72,92,72,92},
	/*16*/{64,80,64,80,80,100,80,100},/*17*/{72,92,72,92,92,116,92,116},
	/*18*/{80,104,80,104,104,128,104,128},/*19*/{88,112,88,112,112,144,112,144},
	/*20*/{104,128,104,128,128,160,128,160},/*21*/{112,144,112,144,144,184,144,184},
	/*22*/{128,160,128,160,160,200,160,200},/*23*/{144,184,144,184,184,232,184,232},
	/*24*/{160,208,160,208,208,256,208,256},/*25*/{176,224,176,224,224,288,224,288},
	/*26*/{208,256,208,256,256,320,256,320},/*27*/{224,288,224,288,288,368,288,368},
	/*28*/{256,320,256,320,320,400,320,400},/*29*/{288,368,288,368,368,464,368,464},
	/*30*/{320,416,320,416,416,512,416,512},/*31*/{352,448,352,448,448,576,448,576},
	/*32*/{416,512,416,512,512,640,512,640},/*33*/{448,576,448,576,576,736,576,736},
	/*34*/{512,640,512,640,640,800,640,800},/*35*/{576,736,576,736,736,928,736,928},
	/*36*/{640,832,640,832,832,1024,832,1024},/*37*/{704,896,704,896,896,1152,896,1152},
	/*38*/{832,1024,832,1024,1024,1280,1024,1280},/*39*/{896,1152,896,1152,1152,1472,1152,1472},
	/*40*/{1024,1280,1024,1280,1280,1600,1280,1600},/*41*/{1152,1472,1152,1472,1472,1856,1472,1856},
	/*42*/{1280,1664,1280,1664,1664,2048,1664,2048},/*43*/{1408,1792,1408,1792,1792,2304,1792,2304},
	/*44*/{1664,2048,1664,2048,2048,2560,2048,2560},/*45*/{1792,2304,1792,2304,2304,2944,2304,2944},
	/*46*/{2048,2560,2048,2560,2560,3200,2560,3200},/*47*/{2304,2944,2304,2944,2944,3712,2944,3712},
	/*48*/{2560,3328,2560,3328,3328,4096,3328,4096},/*49*/{2816,3584,2816,3584,3584,4608,3584,4608},
	/*50*/{3328,4096,3328,4096,4096,5120,4096,5120},/*51*/{3584,4608,3584,4608,4608,5888,4608,5888},
};

ALIGNED_DECLARE(const uint16_t,g_kuiDequantCoeff8x8[52][64],16)={
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==0
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==1
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==2
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==3
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==4
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==5
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==6
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==7
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==8
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==9
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==10
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==11
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==12
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==13
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==14
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==15
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==16
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==17
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==18
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==19
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==20
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==21
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==22
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==23
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==24
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==25
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==26
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==27
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==28
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==29
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==30
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==31
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==32
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==33
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==34
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==35
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==36
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==37
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==38
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==39
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==40
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==41
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==42
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==43
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==44
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==45
	{512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448,512,480,640,480,512,480,640,480,480,448,608,448,480,448,608,448,640,608,816,608,640,608,816,608,480,448,608,448,480,448,608,448},		// QP==46
	{576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512,576,544,736,544,576,544,736,544,544,512,688,512,544,512,688,512,736,688,928,688,736,688,928,688,544,512,688,512,544,512,688,512},		// QP==47
	{320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288,320,304,400,304,320,304,400,304,304,288,384,288,304,288,384,288,400,384,512,384,400,384,512,384,304,288,384,288,304,288,384,288},		// QP==48
	{352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304,352,336,448,336,352,336,448,336,336,304,416,304,336,304,416,304,448,416,560,416,448,416,560,416,336,304,416,304,336,304,416,304},		// QP==49
	{416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368,416,384,528,384,416,384,528,384,384,368,496,368,384,368,496,368,528,496,672,496,528,496,672,496,384,368,496,368,384,368,496,368},		// QP==50
	{448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400,448,416,560,416,448,416,560,416,416,400,528,400,416,400,528,400,560,528,720,528,560,528,720,528,416,400,528,400,416,400,528,400},		// QP==51
};

#define IDX_UNUSED -1

static const int16_t g_kMaxPos[]={IDX_UNUSED,15,14,15,3,14,63,3,3,14,14};
static const int16_t g_kMaxC2[]={IDX_UNUSED,4,4,4,3,4,4,3,3,4,4};
static const int16_t g_kBlockCat2CtxOffsetCBF[]={IDX_UNUSED,0,4,8,12,16,0,12,12,16,16};
static const int16_t g_kBlockCat2CtxOffsetMap[]={IDX_UNUSED,0,15,29,44,47,0,44,44,47,47};
static const int16_t g_kBlockCat2CtxOffsetLast[]={IDX_UNUSED,0,15,29,44,47,0,44,44,47,47};
static const int16_t g_kBlockCat2CtxOffsetOne[]={IDX_UNUSED,0,10,20,30,39,0,30,30,39,39};
static const int16_t g_kBlockCat2CtxOffsetAbs[]={IDX_UNUSED,0,10,20,30,39,0,30,30,39,39};

const uint8_t g_kTopBlkInsideMb[24]={		// for index with z-order 0~23
	// 0 1 | 4 5 luma 8*8 block pNonZeroCount[16+8]
	0,0,1,1,		// 2 3 | 6 7 0 | 1 0 1 2 3
	0,0,1,1,		// --------------- --------- 4 5 6 7
	1,1,1,1,		// 8 9 | 12 13 2 | 3 8 9 10 11
	1,1,1,1,		// 10 11 | 14 15-----------------------------> 12 13 14 15
	0,0,1,1,		// ---------------- chroma 8*8 block 16 17 18 19
	0,0,1,1		// 16 17 | 20 21 0 1 20 21 22 23
	// 18 19 | 22 23
};

const uint8_t g_kLeftBlkInsideMb[24]={		// for index with z-order 0~23
	// 0 1 | 4 5 luma 8*8 block pNonZeroCount[16+8]
	0,1,0,1,		// 2 3 | 6 7 0 | 1 0 1 2 3
	1,1,1,1,		// --------------- --------- 4 5 6 7
	0,1,0,1,		// 8 9 | 12 13 2 | 3 8 9 10 11
	1,1,1,1,		// 10 11 | 14 15-----------------------------> 12 13 14 15
	0,1,0,1,		// ---------------- chroma 8*8 block 16 17 18 19
	0,1,0,1		// 16 17 | 20 21 0 1 20 21 22 23
	// 18 19 | 22 23
};

const uint8_t g_kCacheNzcScanIdx[4*4+4+4+3]={
	// Luma
	9,10,17,18,		// 1+1*8,2+1*8,1+2*8,2+2*8,
	11,12,19,20,		// 3+1*8,4+1*8,3+2*8,4+2*8,
	25,26,33,34,		// 1+3*8,2+3*8,1+4*8,2+4*8,
	27,28,35,36,		// 3+3*8,4+3*8,3+4*8,4+4*8,
	// Cb
	14,15,		// 6+1*8,7+1*8,
	22,23,		// 6+2*8,7+2*8,

	// Cr
	38,39,		// 6+4*8,7+4*8,
	46,47,		// 6+5*8,7+5*8,
	// Luma DC
	41,		// 1+5*8
	// Chroma DC 
	42,43		// 2+5*8,3+5*8,
};


static const uint8_t g_kuiZigzagScan[16]={		// 4*4block residual zig-zag scan order
	0,1,4,8,
	5,2,3,6,
	9,12,13,10,
	7,11,14,15,
};

static const uint8_t g_kuiZigzagScan8x8[64]={		// 8x8 block residual zig-zag scan order
	0,1,8,16,9,2,3,10,
	17,24,32,25,18,11,4,5,
	12,19,26,33,40,48,41,34,
	27,20,13,6,7,14,21,28,
	35,42,49,56,57,50,43,36,
	29,22,15,23,30,37,44,51,
	58,59,52,45,38,31,39,46,
	53,60,61,54,47,55,62,63,
};

static const uint8_t g_kuiIdx2CtxSignificantCoeffFlag8x8[64]={		// Table 9-43,Page 289
	0,1,2,3,4,5,5,4,
	4,3,3,4,4,4,5,5,
	4,4,4,4,3,3,6,7,
	7,7,8,9,10,9,8,7,
	7,6,11,12,13,11,6,7,
	8,9,14,10,9,8,6,11,
	12,13,11,6,9,14,10,9,
	11,12,13,11,14,10,12,14,
};

static const uint8_t g_kuiIdx2CtxLastSignificantCoeffFlag8x8[64]={		// Table 9-43,Page 289
	0,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,
	2,2,2,2,2,2,2,2,
	2,2,2,2,2,2,2,2,
	3,3,3,3,3,3,3,3,
	4,4,4,4,4,4,4,4,
	5,5,5,5,6,6,6,6,
	7,7,7,7,8,8,8,8,
};


int32_t ParseCbfInfoCabac(PWelsNeighAvail pNeighAvail,uint8_t* pNzcCache,int32_t iZIndex,int32_t iResProperty,SDecoderContext* pCtx,uint32_t& uiCbfBit){
	int8_t nA,nB;
	int32_t iCurrBlkXy=pCtx->pCurDqLayer->iMbXyIndex;
	int32_t iTopBlkXy=iCurrBlkXy-pCtx->pCurDqLayer->iMbWidth;		// default value: MB neighboring
	int32_t iLeftBlkXy=iCurrBlkXy-1;		// default value: MB neighboring
	uint16_t* pCbfDc=pCtx->pCurDqLayer->pCbfDc;
	uint32_t* pMbType=pCtx->pCurDqLayer->pDec->pMbType;
	int32_t iCtxInc;
	uiCbfBit=0;
	nA=nB=(int8_t)!!IS_INTRA(pMbType[iCurrBlkXy]);

	if(iResProperty==I16_LUMA_DC || iResProperty==CHROMA_DC_U || iResProperty==CHROMA_DC_V){		// DC
		if(pNeighAvail->iTopAvail)
			nB=(pMbType[iTopBlkXy]==MB_TYPE_INTRA_PCM) || ((pCbfDc[iTopBlkXy]>>iResProperty)&1);
		if(pNeighAvail->iLeftAvail)
			nA=(pMbType[iLeftBlkXy]==MB_TYPE_INTRA_PCM) || ((pCbfDc[iLeftBlkXy]>>iResProperty)&1);
		iCtxInc=nA+(nB<<1);
		WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CBF+g_kBlockCat2CtxOffsetCBF[iResProperty]+iCtxInc,uiCbfBit));
		if(uiCbfBit)
			pCbfDc[iCurrBlkXy]|=(1<<iResProperty);
	}else{		// AC
		// for 4x4 blk,make sure blk-idx is correct
		if(pNzcCache[g_kCacheNzcScanIdx[iZIndex]-8]!=0xff){		// top blk available
			if(g_kTopBlkInsideMb[iZIndex])
				iTopBlkXy=iCurrBlkXy;
			nB=pNzcCache[g_kCacheNzcScanIdx[iZIndex]-8] || pMbType[iTopBlkXy]==MB_TYPE_INTRA_PCM;
		}
		if(pNzcCache[g_kCacheNzcScanIdx[iZIndex]-1]!=0xff){		// left blk available
			if(g_kLeftBlkInsideMb[iZIndex])
				iLeftBlkXy=iCurrBlkXy;
			nA=pNzcCache[g_kCacheNzcScanIdx[iZIndex]-1] || pMbType[iLeftBlkXy]==MB_TYPE_INTRA_PCM;
		}

		iCtxInc=nA+(nB<<1);
		WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pCtx->pCabacCtx+NEW_CTX_OFFSET_CBF+g_kBlockCat2CtxOffsetCBF[iResProperty]+iCtxInc,uiCbfBit));
	}
	return ERR_NONE;
}

int32_t ParseSignificantMapCabac(int32_t* pSignificantMap,int32_t iResProperty,SDecoderContext* pCtx,uint32_t& uiCoeffNum){
	uint32_t uiCode;
	SWelsCabacCtx* pMapCtx=pCtx->pCabacCtx+(iResProperty==LUMA_DC_AC_8 ? NEW_CTX_OFFSET_MAP_8x8 : NEW_CTX_OFFSET_MAP)+g_kBlockCat2CtxOffsetMap[iResProperty];
	SWelsCabacCtx* pLastCtx=pCtx->pCabacCtx+(iResProperty==LUMA_DC_AC_8 ? NEW_CTX_OFFSET_LAST_8x8 : NEW_CTX_OFFSET_LAST)+g_kBlockCat2CtxOffsetLast[iResProperty];
	int32_t i;
	uiCoeffNum=0;
	int32_t i0=0;
	int32_t i1=g_kMaxPos[iResProperty];
	int32_t iCtx;
	for(i=i0; i<i1;++i){
		iCtx=(iResProperty==LUMA_DC_AC_8 ? g_kuiIdx2CtxSignificantCoeffFlag8x8[i] : i);
		// read significant
		WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pMapCtx+iCtx,uiCode));
		if(uiCode){
			*(pSignificantMap++)=1;
			++uiCoeffNum;
			// read last significant
			iCtx=(iResProperty==LUMA_DC_AC_8 ? g_kuiIdx2CtxLastSignificantCoeffFlag8x8[i] : i);
			WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pLastCtx+iCtx,uiCode));
			if(uiCode){
				memset(pSignificantMap,0,(i1-i)*sizeof(int32_t));
				return ERR_NONE;
			}
		}else
			*(pSignificantMap++)=0;
	}

	// deal with last pSignificantMap if no data
	// if(i < i1+1)
	{
		*pSignificantMap=1;
		++uiCoeffNum;
	}

	return ERR_NONE;
}

uint32_t DecodeUEGLevelCabac(SWelsCabacDecEngine* pDecEngine,SWelsCabacCtx* pBinCtx,uint32_t& uiCode){
	uiCode=0;
	WELS_READ_VERIFY(DecodeBinCabac(pDecEngine,pBinCtx,uiCode));
	if(uiCode==0)
		return ERR_NONE;
	else{
		uint32_t uiTmp,uiCount=1;
		uiCode=0;
		do{
			WELS_READ_VERIFY(DecodeBinCabac(pDecEngine,pBinCtx,uiTmp));
			++uiCode;
			++uiCount;
		} while(uiTmp!=0 && uiCount!=13);

		if(uiTmp!=0){
			WELS_READ_VERIFY(DecodeExpBypassCabac(pDecEngine,0,uiTmp));
			uiCode+=uiTmp+1;
		}
		return ERR_NONE;
	}
	return ERR_NONE;
}


int32_t ParseSignificantCoeffCabac(int32_t* pSignificant,int32_t iResProperty,SDecoderContext* pCtx){
	uint32_t uiCode;
	SWelsCabacCtx* pOneCtx=pCtx->pCabacCtx+(iResProperty==LUMA_DC_AC_8 ? NEW_CTX_OFFSET_ONE_8x8 : NEW_CTX_OFFSET_ONE)+g_kBlockCat2CtxOffsetOne[iResProperty];
	SWelsCabacCtx* pAbsCtx=pCtx->pCabacCtx+(iResProperty==LUMA_DC_AC_8 ? NEW_CTX_OFFSET_ABS_8x8 : NEW_CTX_OFFSET_ABS)+g_kBlockCat2CtxOffsetAbs[iResProperty];
	const int16_t iMaxType=g_kMaxC2[iResProperty];
	int32_t i=g_kMaxPos[iResProperty];
	int32_t* pCoff=pSignificant+i;
	int32_t c1=1;
	int32_t c2=0;
	for(; i>=0;--i){
		if(*pCoff!=0){
			WELS_READ_VERIFY(DecodeBinCabac(pCtx->pCabacDecEngine,pOneCtx+c1,uiCode));
			*pCoff+=uiCode;
			if(*pCoff==2){
				WELS_READ_VERIFY(DecodeUEGLevelCabac(pCtx->pCabacDecEngine,pAbsCtx+c2,uiCode));
				*pCoff+=uiCode;
				++c2;
				c2=WELS_MIN(c2,iMaxType);
				c1=0;
			}else
			if(c1){
				++c1;
				c1=WELS_MIN(c1,4);
			}
			WELS_READ_VERIFY(DecodeBypassCabac(pCtx->pCabacDecEngine,uiCode));
			if(uiCode)
				*pCoff=-*pCoff;
		}
		pCoff--;
	}
	return ERR_NONE;
}

void WelsLumaDcDequantIdct(int16_t* pBlock,int32_t iQp,SDecoderContext* pCtx){
	const int32_t kiQMul=pCtx->bUseScalingList ? pCtx->pDequant_coeff4x4[0][iQp][0] : (g_kuiDequantCoeff[iQp][0]<<4);
#define STRIDE 16
	int32_t i;
	int32_t iTemp[16];		// FIXME check if this is a good idea
	int16_t* pBlk=pBlock;
	static const int32_t kiXOffset[4]={0,STRIDE,STRIDE<<2,5*STRIDE};
	static const int32_t kiYOffset[4]={0,STRIDE<<1,STRIDE<<3,10*STRIDE};

	for(i=0; i<4; i++){
		const int32_t kiOffset=kiYOffset[i];
		const int32_t kiX1=kiOffset+kiXOffset[2];
		const int32_t kiX2=STRIDE+kiOffset;
		const int32_t kiX3=kiOffset+kiXOffset[3];
		const int32_t kiI4=i<<2;		// 4*i
		const int32_t kiZ0=pBlk[kiOffset]+pBlk[kiX1];
		const int32_t kiZ1=pBlk[kiOffset]-pBlk[kiX1];
		const int32_t kiZ2=pBlk[kiX2]-pBlk[kiX3];
		const int32_t kiZ3=pBlk[kiX2]+pBlk[kiX3];

		iTemp[kiI4]=kiZ0+kiZ3;
		iTemp[1+kiI4]=kiZ1+kiZ2;
		iTemp[2+kiI4]=kiZ1-kiZ2;
		iTemp[3+kiI4]=kiZ0-kiZ3;
	}

	for(i=0; i<4; i++){
		const int32_t kiOffset=kiXOffset[i];
		const int32_t kiI4=4+i;
		const int32_t kiZ0=iTemp[i]+iTemp[4+kiI4];
		const int32_t kiZ1=iTemp[i]-iTemp[4+kiI4];
		const int32_t kiZ2=iTemp[kiI4]-iTemp[8+kiI4];
		const int32_t kiZ3=iTemp[kiI4]+iTemp[8+kiI4];

		pBlk[kiOffset]=((kiZ0+kiZ3)*kiQMul+(1<<5))>>6;		// FIXME think about merging this into decode_resdual
		pBlk[kiYOffset[1]+kiOffset]=((kiZ1+kiZ2)*kiQMul+(1<<5))>>6;
		pBlk[kiYOffset[2]+kiOffset]=((kiZ1-kiZ2)*kiQMul+(1<<5))>>6;
		pBlk[kiYOffset[3]+kiOffset]=((kiZ0-kiZ3)*kiQMul+(1<<5))>>6;
	}
#undef STRIDE
}

void WelsChromaDcIdct(int16_t* pBlock){
	int32_t iStride=32;
	int32_t iXStride=16;
	int32_t iStride1=iXStride+iStride;
	int16_t* pBlk=pBlock;
	int32_t iA,iB,iC,iD,iE;

	iA=pBlk[0];
	iB=pBlk[iXStride];
	iC=pBlk[iStride];
	iD=pBlk[iStride1];

	iE=iA-iB;
	iA+=iB;
	iB=iC-iD;
	iC+=iD;

	pBlk[0]=(iA+iC);
	pBlk[iXStride]=(iE+iB);
	pBlk[iStride]=(iA-iC);
	pBlk[iStride1]=(iE-iB);
}

int32_t ParseResidualBlockCabac(PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCountCache,SBitStringAux* pBsAux,int32_t iIndex,int32_t iMaxNumCoeff,const uint8_t* pScanTable,int32_t iResProperty,short* sTCoeff,uint8_t uiQp,SDecoderContext* pCtx){
	int32_t iCurNzCacheIdx;
	uint32_t uiTotalCoeffNum=0;
	uint32_t uiCbpBit;
	int32_t pSignificantMap[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	int32_t iMbResProperty=0;
	GetMbResProperty(&iMbResProperty,&iResProperty,false);
	const uint16_t* pDeQuantMul=(pCtx->bUseScalingList) ? pCtx->pDequant_coeff4x4[iMbResProperty][uiQp] :
		g_kuiDequantCoeff[uiQp];

	WELS_READ_VERIFY(ParseCbfInfoCabac(pNeighAvail,pNonZeroCountCache,iIndex,iResProperty,pCtx,uiCbpBit));
	if(uiCbpBit){		// has coeff
		WELS_READ_VERIFY(ParseSignificantMapCabac(pSignificantMap,iResProperty,pCtx,uiTotalCoeffNum));
		WELS_READ_VERIFY(ParseSignificantCoeffCabac(pSignificantMap,iResProperty,pCtx));
	}

	iCurNzCacheIdx=g_kCacheNzcScanIdx[iIndex];
	pNonZeroCountCache[iCurNzCacheIdx]=(uint8_t)uiTotalCoeffNum;
	if(uiTotalCoeffNum==0){
		return ERR_NONE;
	}
	int32_t j=0;
	if(iResProperty==I16_LUMA_DC){
		do{
			sTCoeff[pScanTable[j]]=pSignificantMap[j];
			++j;
		} while(j<16);
		WelsLumaDcDequantIdct(sTCoeff,uiQp,pCtx);
	}else
	if(iResProperty==CHROMA_DC_U || iResProperty==CHROMA_DC_V){
		do{
			sTCoeff[pScanTable[j]]=pSignificantMap[j];
			++j;
		} while(j<4);
		// iHadamard2x2
		WelsChromaDcIdct(sTCoeff);
		// scaling
		if(!pCtx->bUseScalingList){
			for(j=0; j<4;++j){
				sTCoeff[pScanTable[j]]=(int16_t)((int64_t)sTCoeff[pScanTable[j]]*(int64_t)pDeQuantMul[0]>>1);
			}
		}else{		// with scaling list
			for(j=0; j<4;++j){
				sTCoeff[pScanTable[j]]=(int16_t)((int64_t)sTCoeff[pScanTable[j]]*(int64_t)pDeQuantMul[0]>>5);
			}
		}
	}else{		// luma ac,chroma ac
		do{
			if(pSignificantMap[j]!=0){
				if(!pCtx->bUseScalingList){
					sTCoeff[pScanTable[j]]=pSignificantMap[j]*pDeQuantMul[pScanTable[j]&0x07];
				}else{
					sTCoeff[pScanTable[j]]=(int16_t)(((int64_t)pSignificantMap[j]*(int64_t)pDeQuantMul[pScanTable[j]]+8)>>4);
				}
			}
			++j;
		} while(j<16);
	}
	return ERR_NONE;
}

int32_t ParseResidualBlockCabac8x8(PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCountCache,SBitStringAux* pBsAux,int32_t iIndex,int32_t iMaxNumCoeff,const uint8_t* pScanTable,int32_t iResProperty,short* sTCoeff,uint8_t uiQp,SDecoderContext* pCtx){
	uint32_t uiTotalCoeffNum=0;
	uint32_t uiCbpBit;
	int32_t pSignificantMap[64]={0};

	int32_t iMbResProperty=0;
	GetMbResProperty(&iMbResProperty,&iResProperty,false);
	const uint16_t* pDeQuantMul=(pCtx->bUseScalingList) ? pCtx->pDequant_coeff8x8[iMbResProperty-6][uiQp] :
		g_kuiDequantCoeff8x8[uiQp];

	uiCbpBit=1;		// for 8x8,MaxNumCoeff==64 && uiCbpBit==1
	if(uiCbpBit){		// has coeff
		WELS_READ_VERIFY(ParseSignificantMapCabac(pSignificantMap,iResProperty,pCtx,uiTotalCoeffNum));
		WELS_READ_VERIFY(ParseSignificantCoeffCabac(pSignificantMap,iResProperty,pCtx));
	}

	pNonZeroCountCache[g_kCacheNzcScanIdx[iIndex]]=
		pNonZeroCountCache[g_kCacheNzcScanIdx[iIndex+1]]=
		pNonZeroCountCache[g_kCacheNzcScanIdx[iIndex+2]]=
		pNonZeroCountCache[g_kCacheNzcScanIdx[iIndex+3]]=(uint8_t)uiTotalCoeffNum;
	if(uiTotalCoeffNum==0){
		return ERR_NONE;
	}
	int32_t j=0,i;
	if(iResProperty==LUMA_DC_AC_8){
		do{
			if(pSignificantMap[j]!=0){
				i=pScanTable[j];
				sTCoeff[i]=uiQp>=36 ? ((pSignificantMap[j]*pDeQuantMul[i])*(1<<(uiQp/6-6))) : ((
					pSignificantMap[j]*pDeQuantMul[i]+(1<<(5-uiQp/6)))>>(6-uiQp/6));
			}
			++j;
		} while(j<64);
	}

	return ERR_NONE;
}

const uint8_t g_kuiChromaDcScan[4]={
	0,16,32,48
};

int32_t WelsDecodeMbCabacPSliceBaseMode0(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint32_t& uiEosFlag){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SBitStringAux* pBsAux=pCurDqLayer->pBitStringAux;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;

	int32_t iScanIdxStart=pSlice->sSliceHeaderExt.uiScanIdxStart;
	int32_t iScanIdxEnd=pSlice->sSliceHeaderExt.uiScanIdxEnd;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int32_t iMbResProperty;
	int32_t i;
	uint32_t uiMbType=0,uiCbp=0,uiCbpLuma=0,uiCbpChroma=0;

	ENFORCE_STACK_ALIGN_1D(uint8_t,pNonZeroCount,48,16);

	pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;

	WELS_READ_VERIFY(ParseMBTypePSliceCabac(pCtx,pNeighAvail,uiMbType));
	// uiMbType=4 is not allowded.
	if(uiMbType<4){		// Inter mode
		int16_t pMotionVector[LIST_A][30][MV_A];
		int16_t pMvdCache[LIST_A][30][MV_A];
		int8_t pRefIndex[LIST_A][30];
		pCurDqLayer->pDec->pMbType[iMbXy]=g_ksInterPMbTypeInfo[uiMbType].iType;
		WelsFillCacheInterCabac(pNeighAvail,pNonZeroCount,pMotionVector,pMvdCache,pRefIndex,pCurDqLayer);
		WELS_READ_VERIFY(ParseInterPMotionInfoCabac(pCtx,pNeighAvail,pNonZeroCount,pMotionVector,pMvdCache,pRefIndex));
		pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
	}else{		// Intra mode
		uiMbType-=5;
		if(uiMbType>25)
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);
		if(!pCtx->pSps->uiChromaFormatIdc && ((uiMbType>=5 && uiMbType<=12) || (uiMbType>=17 && uiMbType<=24)))
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);

		if(25==uiMbType){		// I_PCM
			uprintf("I_PCM mode exists in P slice!");
			WELS_READ_VERIFY(ParseIPCMInfoCabac(pCtx));
			pSlice->iLastDeltaQp=0;
			WELS_READ_VERIFY(ParseEndOfSliceCabac(pCtx,uiEosFlag));
			if(uiEosFlag){
				RestoreCabacDecEngineToBS(pCtx->pCabacDecEngine,pCtx->pCurDqLayer->pBitStringAux);
			}
			return ERR_NONE;
		}else{		// normal Intra mode
			if(0==uiMbType){		// Intra4x4
				ENFORCE_STACK_ALIGN_1D(int8_t,pIntraPredMode,48,16);
				pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA4x4;
				if(pCtx->pPps->bTransform8x8ModeFlag){
					WELS_READ_VERIFY(ParseTransformSize8x8FlagCabac(pCtx,pNeighAvail,pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]));
				}
				if(pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
					uiMbType=pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA8x8;
					pCtx->pFillInfoCacheIntraNxNFunc(pNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
					WELS_READ_VERIFY(ParseIntra8x8Mode(pCtx,pNeighAvail,pIntraPredMode,pBsAux,pCurDqLayer));
				}else{
					pCtx->pFillInfoCacheIntraNxNFunc(pNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
					WELS_READ_VERIFY(ParseIntra4x4Mode(pCtx,pNeighAvail,pIntraPredMode,pBsAux,pCurDqLayer));
				}
			}else{		// Intra16x16
				pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA16x16;
				pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;
				pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
				pCurDqLayer->pIntraPredMode[iMbXy][7]=(uiMbType-1)&3;
				pCurDqLayer->pCbp[iMbXy]=g_kuiI16CbpTable[(uiMbType-1)>>2];
				uiCbpChroma=pCtx->pSps->uiChromaFormatIdc ? pCurDqLayer->pCbp[iMbXy]>>4 : 0;
				uiCbpLuma=pCurDqLayer->pCbp[iMbXy]&15;
				WelsFillCacheNonZeroCount(pNeighAvail,pNonZeroCount,pCurDqLayer);
				WELS_READ_VERIFY(ParseIntra16x16Mode(pCtx,pNeighAvail,pBsAux,pCurDqLayer));
			}
		}
	}

	ST32(&pCurDqLayer->pNzc[iMbXy][0],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][4],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][8],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][12],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][16],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][20],0);

	if(MB_TYPE_INTRA16x16!=pCurDqLayer->pDec->pMbType[iMbXy]){
		WELS_READ_VERIFY(ParseCbpInfoCabac(pCtx,pNeighAvail,uiCbp));

		pCurDqLayer->pCbp[iMbXy]=uiCbp;
		pSlice->iLastDeltaQp=uiCbp==0 ? 0 : pSlice->iLastDeltaQp;
		uiCbpChroma=pCtx->pSps->uiChromaFormatIdc ? pCurDqLayer->pCbp[iMbXy]>>4 : 0;
		uiCbpLuma=pCurDqLayer->pCbp[iMbXy]&15;
	}

	if(pCurDqLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){

		if(MB_TYPE_INTRA16x16!=pCurDqLayer->pDec->pMbType[iMbXy]){
			// Need modification when B picutre add in
			bool bNeedParseTransformSize8x8Flag=
				(((pCurDqLayer->pDec->pMbType[iMbXy]>=MB_TYPE_16x16 && pCurDqLayer->pDec->pMbType[iMbXy]<=MB_TYPE_8x16) || pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy])
				  && (pCurDqLayer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA8x8)
				  && (pCurDqLayer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA4x4)
				  && ((pCurDqLayer->pCbp[iMbXy]&0x0F)>0)
				  && (pCtx->pPps->bTransform8x8ModeFlag));

			if(bNeedParseTransformSize8x8Flag){
				WELS_READ_VERIFY(ParseTransformSize8x8FlagCabac(pCtx,pNeighAvail,pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]));		// transform_size_8x8_flag
			}
		}

		memset(pCurDqLayer->pScaledTCoeff[iMbXy],0,384*sizeof(pCurDqLayer->pScaledTCoeff[iMbXy][0]));

		int32_t iQpDelta,iId8x8,iId4x4;

		WELS_READ_VERIFY(ParseDeltaQpCabac(pCtx,iQpDelta));
		if(iQpDelta>25 || iQpDelta<-26){		// out of iQpDelta range
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_QP);
		}
		pCurDqLayer->pLumaQp[iMbXy]=(pSlice->iLastMbQp+iQpDelta+52)%52;		// update last_mb_qp
		pSlice->iLastMbQp=pCurDqLayer->pLumaQp[iMbXy];
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pSlice->iLastMbQp+
											 pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}

		if(MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){
			// step1: Luma DC
			WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,0,16,g_kuiLumaDcZigzagScan,I16_LUMA_DC,pCurDqLayer->pScaledTCoeff[iMbXy],pCurDqLayer->pLumaQp[iMbXy],pCtx));
			// step2: Luma AC
			if(uiCbpLuma){
				for(i=0; i<16; i++){
					WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,i,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),I16_LUMA_AC,pCurDqLayer->pScaledTCoeff[iMbXy]+(i<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx));
				}
				ST32(&pCurDqLayer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&pCurDqLayer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&pCurDqLayer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&pCurDqLayer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{
				ST32(&pCurDqLayer->pNzc[iMbXy][0],0);
				ST32(&pCurDqLayer->pNzc[iMbXy][4],0);
				ST32(&pCurDqLayer->pNzc[iMbXy][8],0);
				ST32(&pCurDqLayer->pNzc[iMbXy][12],0);
			}
		}else{		// non-MB_TYPE_INTRA16x16
			if(pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
				// Transform 8x8 support for CABAC
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						WELS_READ_VERIFY(ParseResidualBlockCabac8x8(pNeighAvail,pNonZeroCount,pBsAux,(iId8x8<<2),iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan8x8+iScanIdxStart,IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]) ? LUMA_DC_AC_INTRA_8 : LUMA_DC_AC_INTER_8,pCurDqLayer->pScaledTCoeff[iMbXy]+(iId8x8<<6),pCurDqLayer->pLumaQp[iMbXy],pCtx));
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&pCurDqLayer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&pCurDqLayer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&pCurDqLayer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&pCurDqLayer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{
				iMbResProperty=(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						int32_t iIdx=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							// Luma (DC and AC decoding together)
							WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,iIdx,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan+iScanIdxStart,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIdx<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx));
							iIdx++;
						}
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[iId8x8<<2]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&pCurDqLayer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&pCurDqLayer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&pCurDqLayer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&pCurDqLayer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}
		}

		// chroma
		// step1: DC
		if(1==uiCbpChroma || 2==uiCbpChroma){
			for(i=0; i<2; i++){
				if(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_DC_V : CHROMA_DC_U;
				else
					iMbResProperty=i ? CHROMA_DC_V_INTER : CHROMA_DC_U_INTER;

				WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,16+(i<<2),4,g_kuiChromaDcScan,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+256+(i<<6),pCurDqLayer->pChromaQp[iMbXy][i],pCtx));
			}
		}
		// step2: AC
		if(2==uiCbpChroma){
			for(i=0; i<2; i++){
				if(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_AC_V : CHROMA_AC_U;
				else
					iMbResProperty=i ? CHROMA_AC_V_INTER : CHROMA_AC_U_INTER;
				int32_t index=16+(i<<2);
				for(iId4x4=0; iId4x4<4; iId4x4++){
					WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,index,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(index<<4),pCurDqLayer->pChromaQp[iMbXy][i],pCtx));
					index++;
				}
			}
			ST16(&pCurDqLayer->pNzc[iMbXy][16],LD16(&pNonZeroCount[6+8*1]));
			ST16(&pCurDqLayer->pNzc[iMbXy][20],LD16(&pNonZeroCount[6+8*2]));
			ST16(&pCurDqLayer->pNzc[iMbXy][18],LD16(&pNonZeroCount[6+8*4]));
			ST16(&pCurDqLayer->pNzc[iMbXy][22],LD16(&pNonZeroCount[6+8*5]));
		}else{
			ST32(&pCurDqLayer->pNzc[iMbXy][16],0);
			ST32(&pCurDqLayer->pNzc[iMbXy][20],0);
		}
	}else{
		pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pCurDqLayer->pLumaQp[iMbXy]+pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}
	}

	WELS_READ_VERIFY(ParseEndOfSliceCabac(pCtx,uiEosFlag));
	if(uiEosFlag){
		RestoreCabacDecEngineToBS(pCtx->pCabacDecEngine,pCtx->pCurDqLayer->pBitStringAux);
	}

	return ERR_NONE;
}

int32_t WelsDecodeMbCabacPSlice(SDecoderContext* pCtx,SNalUnit* pNalCur,uint32_t& uiEosFlag){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	SPicture** ppRefPic=pCtx->sRefPic.pRefList[LIST_0];
	uint32_t uiCode;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int32_t i;
	SWelsNeighAvail uiNeighAvail;
	pCurDqLayer->pCbp[iMbXy]=0;
	pCurDqLayer->pCbfDc[iMbXy]=0;
	pCurDqLayer->pChromaPredMode[iMbXy]=C_PRED_DC;

	pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
	pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;

	GetNeighborAvailMbType(&uiNeighAvail,pCurDqLayer);
	WELS_READ_VERIFY(ParseSkipFlagCabac(pCtx,&uiNeighAvail,uiCode));

	if(uiCode){
		int16_t pMv[2]={0};
		pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_SKIP;
		ST32(&pCurDqLayer->pNzc[iMbXy][0],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][4],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][8],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][12],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][16],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][20],0);

		pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
		memset(pCurDqLayer->pDec->pRefIndex[0][iMbXy],0,sizeof(int8_t)*16);
		bool bIsPending=false;	// GetThreadCount (pCtx) > 1;
		pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[0] && (ppRefPic[0]->bIsComplete
			 || bIsPending));
		// predict mv
		PredPSkipMvFromNeighbor(pCurDqLayer,pMv);
		for(i=0; i<16; i++){
			ST32(pCurDqLayer->pDec->pMv[0][iMbXy][i],*(uint32_t*)pMv);
			ST32(pCurDqLayer->pMvd[0][iMbXy][i],0);
		}

		// if (!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag) {
		// memset (pCurDqLayer->pScaledTCoeff[iMbXy],0,384 * sizeof (int16_t));
		// }

		// reset rS
		pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;		// ??????????????? dqaunt of previous mb
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pCurDqLayer->pLumaQp[iMbXy]+
											 pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}

		// for neighboring CABAC usage
		pSlice->iLastDeltaQp=0;

		WELS_READ_VERIFY(ParseEndOfSliceCabac(pCtx,uiEosFlag));

		return ERR_NONE;
	}

	WELS_READ_VERIFY(WelsDecodeMbCabacPSliceBaseMode0(pCtx,&uiNeighAvail,uiEosFlag));
	return ERR_NONE;
}

static inline void SetRectBlock(void* vp,int32_t w,const int32_t h,int32_t stride,const uint32_t val,const int32_t size){
	uint8_t* p=(uint8_t*)vp;
	w*=size;
	if(w==1 && h==4){
		*(uint8_t*)(p+0*stride)=
			*(uint8_t*)(p+1*stride)=
			*(uint8_t*)(p+2*stride)=
			*(uint8_t*)(p+3*stride)=(uint8_t)val;
	}else
	if(w==2 && h==2){
		*(uint16_t*)(p+0*stride)=
			*(uint16_t*)(p+1*stride)=size==4 ? (uint16_t)val : (uint16_t)(val*0x0101U);
	}else
	if(w==2 && h==4){
		*(uint16_t*)(p+0*stride)=
			*(uint16_t*)(p+1*stride)=
			*(uint16_t*)(p+2*stride)=
			*(uint16_t*)(p+3*stride)=size==4 ? (uint16_t)val : (uint16_t)(val*0x0101U);
	}else
	if(w==4 && h==2){
		*(uint32_t*)(p+0*stride)=
			*(uint32_t*)(p+1*stride)=size==4 ? val : (uint32_t)(val*0x01010101UL);
	}else
	if(w==4 && h==4){
		*(uint32_t*)(p+0*stride)=
			*(uint32_t*)(p+1*stride)=
			*(uint32_t*)(p+2*stride)=
			*(uint32_t*)(p+3*stride)=size==4 ? val : (uint32_t)(val*0x01010101UL);
	}else
	if(w==8 && h==1){
		*(uint32_t*)(p+0*stride)=
			*(uint32_t*)(p+0*stride+4)=size==4 ? val : (uint32_t)(val*0x01010101UL);
	}else
	if(w==8 && h==2){
		*(uint32_t*)(p+0*stride)=
			*(uint32_t*)(p+0*stride+4)=
			*(uint32_t*)(p+1*stride)=
			*(uint32_t*)(p+1*stride+4)=size==4 ? val : (uint32_t)(val*0x01010101UL);
	}else
	if(w==8 && h==4){
		*(uint32_t*)(p+0*stride)=
			*(uint32_t*)(p+0*stride+4)=
			*(uint32_t*)(p+1*stride)=
			*(uint32_t*)(p+1*stride+4)=
			*(uint32_t*)(p+2*stride)=
			*(uint32_t*)(p+2*stride+4)=
			*(uint32_t*)(p+3*stride)=
			*(uint32_t*)(p+3*stride+4)=size==4 ? val : (uint32_t)(val*0x01010101UL);
	}else
	if(w==16 && h==2){
		*(uint32_t*)(p+0*stride+0)=
			*(uint32_t*)(p+0*stride+4)=
			*(uint32_t*)(p+0*stride+8)=
			*(uint32_t*)(p+0*stride+12)=
			*(uint32_t*)(p+1*stride+0)=
			*(uint32_t*)(p+1*stride+4)=
			*(uint32_t*)(p+1*stride+8)=
			*(uint32_t*)(p+1*stride+12)=size==4 ? val : (uint32_t)(val*0x01010101UL);
	}else
	if(w==16 && h==3){
		*(uint32_t*)(p+0*stride+0)=
			*(uint32_t*)(p+0*stride+4)=
			*(uint32_t*)(p+0*stride+8)=
			*(uint32_t*)(p+0*stride+12)=
			*(uint32_t*)(p+1*stride+0)=
			*(uint32_t*)(p+1*stride+4)=
			*(uint32_t*)(p+1*stride+8)=
			*(uint32_t*)(p+1*stride+12)=
			*(uint32_t*)(p+2*stride+0)=
			*(uint32_t*)(p+2*stride+4)=
			*(uint32_t*)(p+2*stride+8)=
			*(uint32_t*)(p+2*stride+12)=size==4 ? val : (uint32_t)(val*0x01010101UL);
	}else
	if(w==16 && h==4){
		*(uint32_t*)(p+0*stride+0)=
			*(uint32_t*)(p+0*stride+4)=
			*(uint32_t*)(p+0*stride+8)=
			*(uint32_t*)(p+0*stride+12)=
			*(uint32_t*)(p+1*stride+0)=
			*(uint32_t*)(p+1*stride+4)=
			*(uint32_t*)(p+1*stride+8)=
			*(uint32_t*)(p+1*stride+12)=
			*(uint32_t*)(p+2*stride+0)=
			*(uint32_t*)(p+2*stride+4)=
			*(uint32_t*)(p+2*stride+8)=
			*(uint32_t*)(p+2*stride+12)=
			*(uint32_t*)(p+3*stride+0)=
			*(uint32_t*)(p+3*stride+4)=
			*(uint32_t*)(p+3*stride+8)=
			*(uint32_t*)(p+3*stride+12)=size==4 ? val : (uint32_t)(val*0x01010101UL);
	}
}

void CopyRectBlock4Cols(void* vdst,void* vsrc,const int32_t stride_dst,const int32_t stride_src,int32_t w,const int32_t size){
	uint8_t* dst=(uint8_t*)vdst;
	uint8_t* src=(uint8_t*)vsrc;
	w*=size;
	if(w==1){
		dst[stride_dst*0]=src[stride_src*0];
		dst[stride_dst*1]=src[stride_src*1];
		dst[stride_dst*2]=src[stride_src*2];
		dst[stride_dst*3]=src[stride_src*3];
	}else
	if(w==2){
		*(uint16_t*)(&dst[stride_dst*0])=*(uint16_t*)(&src[stride_src*0]);
		*(uint16_t*)(&dst[stride_dst*1])=*(uint16_t*)(&src[stride_src*1]);
		*(uint16_t*)(&dst[stride_dst*2])=*(uint16_t*)(&src[stride_src*2]);
		*(uint16_t*)(&dst[stride_dst*3])=*(uint16_t*)(&src[stride_src*3]);
	}else
	if(w==4){
		*(uint32_t*)(&dst[stride_dst*0])=*(uint32_t*)(&src[stride_src*0]);
		*(uint32_t*)(&dst[stride_dst*1])=*(uint32_t*)(&src[stride_src*1]);
		*(uint32_t*)(&dst[stride_dst*2])=*(uint32_t*)(&src[stride_src*2]);
		*(uint32_t*)(&dst[stride_dst*3])=*(uint32_t*)(&src[stride_src*3]);
	}else
	if(w==16){
		memcpy(&dst[stride_dst*0],&src[stride_src*0],16);
		memcpy(&dst[stride_dst*1],&src[stride_src*1],16);
		memcpy(&dst[stride_dst*2],&src[stride_src*2],16);
		memcpy(&dst[stride_dst*3],&src[stride_src*3],16);
	}
}

int32_t GetColocatedMb(SDecoderContext* pCtx,MbType& mbType,SubMbType& subMbType){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;

	uint32_t is8x8=IS_Inter_8x8(GetMbType(pCurDqLayer)[iMbXy]);
	mbType=GetMbType(pCurDqLayer)[iMbXy];

	SPicture* colocPic=pCtx->sRefPic.pRefList[LIST_1][0];

	if(colocPic==NULL){
		uprintf("Colocated Ref Picture for B-Slice is lost,B-Slice decoding cannot be continued!");
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_DATA,ERR_INFO_REFERENCE_PIC_LOST);
	}

	MbType coloc_mbType=colocPic->pMbType[iMbXy];
	if(coloc_mbType==MB_TYPE_SKIP){
		// This indicates the colocated MB is P SKIP MB
		coloc_mbType|=MB_TYPE_16x16|MB_TYPE_P0L0|MB_TYPE_P1L0;
	}
	if(IS_Inter_8x8(coloc_mbType) && !pCtx->pSps->bDirect8x8InferenceFlag){
		subMbType=SUB_MB_TYPE_4x4|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_DIRECT;
		mbType|=MB_TYPE_8x8|MB_TYPE_L0|MB_TYPE_L1;
	}else
	if(!is8x8 && (IS_INTER_16x16(coloc_mbType) || IS_INTRA(coloc_mbType))){
		subMbType=SUB_MB_TYPE_8x8|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_DIRECT;
		mbType|=MB_TYPE_16x16|MB_TYPE_L0|MB_TYPE_L1;
	}else{
		subMbType=SUB_MB_TYPE_8x8|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_DIRECT;
		mbType|=MB_TYPE_8x8|MB_TYPE_L0|MB_TYPE_L1;
	}

	if(IS_INTRA(coloc_mbType)){
		SetRectBlock(pCurDqLayer->iColocIntra,4,4,4*sizeof(int8_t),1,sizeof(int8_t));
		return ERR_NONE;
	}
	SetRectBlock(pCurDqLayer->iColocIntra,4,4,4*sizeof(int8_t),0,sizeof(int8_t));

	if(IS_INTER_16x16(mbType)){
		int16_t iMVZero[2]={0};
		int16_t* pMv=IS_TYPE_L1(coloc_mbType) ? colocPic->pMv[LIST_1][iMbXy][0] : iMVZero;
		ST32(pCurDqLayer->iColocMv[LIST_0][0],LD32(colocPic->pMv[LIST_0][iMbXy][0]));
		ST32(pCurDqLayer->iColocMv[LIST_1][0],LD32(pMv));
		pCurDqLayer->iColocRefIndex[LIST_0][0]=colocPic->pRefIndex[LIST_0][iMbXy][0];
		pCurDqLayer->iColocRefIndex[LIST_1][0]=IS_TYPE_L1(coloc_mbType) ? colocPic->pRefIndex[LIST_1][iMbXy][0] :
			REF_NOT_IN_LIST;
	}else{
		if(!pCtx->pSps->bDirect8x8InferenceFlag){
			CopyRectBlock4Cols(pCurDqLayer->iColocMv[LIST_0],colocPic->pMv[LIST_0][iMbXy],16,16,4,4);
			CopyRectBlock4Cols(pCurDqLayer->iColocRefIndex[LIST_0],colocPic->pRefIndex[LIST_0][iMbXy],4,4,4,1);
			if(IS_TYPE_L1(coloc_mbType)){
				CopyRectBlock4Cols(pCurDqLayer->iColocMv[LIST_1],colocPic->pMv[LIST_1][iMbXy],16,16,4,4);
				CopyRectBlock4Cols(pCurDqLayer->iColocRefIndex[LIST_1],colocPic->pRefIndex[LIST_1][iMbXy],4,4,4,1);
			}else{		// only forward prediction
				SetRectBlock(pCurDqLayer->iColocRefIndex[LIST_1],4,4,4,(uint8_t)REF_NOT_IN_LIST,1);
			}
		}else{
			for(int32_t listIdx=0; listIdx<1+!!(coloc_mbType&MB_TYPE_L1); listIdx++){
				SetRectBlock(pCurDqLayer->iColocMv[listIdx][0],2,2,16,LD32(colocPic->pMv[listIdx][iMbXy][0]),4);
				SetRectBlock(pCurDqLayer->iColocMv[listIdx][2],2,2,16,LD32(colocPic->pMv[listIdx][iMbXy][3]),4);
				SetRectBlock(pCurDqLayer->iColocMv[listIdx][8],2,2,16,LD32(colocPic->pMv[listIdx][iMbXy][12]),4);
				SetRectBlock(pCurDqLayer->iColocMv[listIdx][10],2,2,16,LD32(colocPic->pMv[listIdx][iMbXy][15]),4);

				SetRectBlock(&pCurDqLayer->iColocRefIndex[listIdx][0],2,2,4,colocPic->pRefIndex[listIdx][iMbXy][0],1);
				SetRectBlock(&pCurDqLayer->iColocRefIndex[listIdx][2],2,2,4,colocPic->pRefIndex[listIdx][iMbXy][3],1);
				SetRectBlock(&pCurDqLayer->iColocRefIndex[listIdx][8],2,2,4,colocPic->pRefIndex[listIdx][iMbXy][12],1);
				SetRectBlock(&pCurDqLayer->iColocRefIndex[listIdx][10],2,2,4,colocPic->pRefIndex[listIdx][iMbXy][15],1);
			}
			if(!(coloc_mbType&MB_TYPE_L1))		// only forward prediction
				SetRectBlock(&pCurDqLayer->iColocRefIndex[1][0],4,4,4,(uint8_t)REF_NOT_IN_LIST,1);
		}
	}
	return ERR_NONE;
}

void UpdateP16x16DirectCabac(PDqLayer pCurDqLayer){
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	const int16_t direct=(1<<8)|1;
	for(i=0; i<16; i+=4){
		const uint8_t kuiScan4Idx=g_kuiScan4[i];
		const uint8_t kuiScan4IdxPlus4=4+kuiScan4Idx;
		ST16(&pCurDqLayer->pDirect[iMbXy][kuiScan4Idx],direct);
		ST16(&pCurDqLayer->pDirect[iMbXy][kuiScan4IdxPlus4],direct);
	}
}

// Table 7.18 Sub-macroblock types in B macroblocks.
static const SPartMbInfo g_ksInterBSubMbTypeInfo[]={
	{MB_TYPE_DIRECT,1,2},// B_Direct_8x8
 {SUB_MB_TYPE_8x8|MB_TYPE_P0L0,1,2},// B_L0_8x8
 {SUB_MB_TYPE_8x8|MB_TYPE_P0L1,1,2},// B_L1_8x8
 {SUB_MB_TYPE_8x8|MB_TYPE_P0L0|MB_TYPE_P0L1,1,2},// B_Bi_8x8
 {SUB_MB_TYPE_8x4|MB_TYPE_P0L0,2,2},// B_L0_8x4
 {SUB_MB_TYPE_4x8|MB_TYPE_P0L0,2,1},// B_L0_4x8
 {SUB_MB_TYPE_8x4|MB_TYPE_P0L1,2,2},// B_L1_8x4
 {SUB_MB_TYPE_4x8|MB_TYPE_P0L1,2,1},// B_L1_4x8
 {SUB_MB_TYPE_8x4|MB_TYPE_P0L0|MB_TYPE_P0L1,2,2},// B_Bi_8x4
 {SUB_MB_TYPE_4x8|MB_TYPE_P0L0|MB_TYPE_P0L1,2,1},// B_Bi_4x8
 {SUB_MB_TYPE_4x4|MB_TYPE_P0L0,4,1},// B_L0_4x4
 {SUB_MB_TYPE_4x4|MB_TYPE_P0L1,4,1},// B_L1_4x4
 {SUB_MB_TYPE_4x4|MB_TYPE_P0L0|MB_TYPE_P0L1,4,1}		// B_Bi_4x4
};

void UpdateP8x8DirectCabac(PDqLayer pCurDqLayer,int32_t iPartIdx){
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
	pCurDqLayer->pDirect[iMbXy][iScan4Idx]=pCurDqLayer->pDirect[iMbXy][iScan4Idx+1]=pCurDqLayer->pDirect[iMbXy][iScan4Idx+4]=pCurDqLayer->pDirect[iMbXy][iScan4Idx+5]=1;
}

void FillSpatialDirect8x8Mv(PDqLayer pCurDqLayer,const int16_t& iIdx8,const int8_t& iPartCount,const int8_t& iPartW,const SubMbType& subMbType,const bool& bIsLongRef,int16_t pMvDirect[LIST_A][2],int8_t iRef[LIST_A],int16_t pMotionVector[LIST_A][30][MV_A],int16_t pMvdCache[LIST_A][30][MV_A]){
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	for(int32_t j=0; j<iPartCount; j++){
		int8_t iPartIdx=iIdx8+j*iPartW;
		uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
		uint8_t iColocIdx=g_kuiScan4[iPartIdx];
		uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];

		int16_t pMV[4]={0};
		if(IS_SUB_8x8(subMbType)){
			*(uint32_t*)pMV=*(uint32_t*)pMvDirect[LIST_0];
			ST32((pMV+2),LD32(pMV));
			ST64(pCurDqLayer->pDec->pMv[LIST_0][iMbXy][iScan4Idx],LD64(pMV));
			ST64(pCurDqLayer->pDec->pMv[LIST_0][iMbXy][iScan4Idx+4],LD64(pMV));
			ST64(pCurDqLayer->pMvd[LIST_0][iMbXy][iScan4Idx],0);
			ST64(pCurDqLayer->pMvd[LIST_0][iMbXy][iScan4Idx+4],0);
			if(pMotionVector!=NULL){
				ST64(pMotionVector[LIST_0][iCacheIdx],LD64(pMV));
				ST64(pMotionVector[LIST_0][iCacheIdx+6],LD64(pMV));
			}
			if(pMvdCache!=NULL){
				ST64(pMvdCache[LIST_0][iCacheIdx],0);
				ST64(pMvdCache[LIST_0][iCacheIdx+6],0);
			}
			*(uint32_t*)pMV=*(uint32_t*)pMvDirect[LIST_1];
			ST32((pMV+2),LD32(pMV));
			ST64(pCurDqLayer->pDec->pMv[LIST_1][iMbXy][iScan4Idx],LD64(pMV));
			ST64(pCurDqLayer->pDec->pMv[LIST_1][iMbXy][iScan4Idx+4],LD64(pMV));
			ST64(pCurDqLayer->pMvd[LIST_1][iMbXy][iScan4Idx],0);
			ST64(pCurDqLayer->pMvd[LIST_1][iMbXy][iScan4Idx+4],0);
			if(pMotionVector!=NULL){
				ST64(pMotionVector[LIST_1][iCacheIdx],LD64(pMV));
				ST64(pMotionVector[LIST_1][iCacheIdx+6],LD64(pMV));
			}
			if(pMvdCache!=NULL){
				ST64(pMvdCache[LIST_1][iCacheIdx],0);
				ST64(pMvdCache[LIST_1][iCacheIdx+6],0);
			}
		}else{		// SUB_4x4
			*(uint32_t*)pMV=*(uint32_t*)pMvDirect[LIST_0];
			ST32(pCurDqLayer->pDec->pMv[LIST_0][iMbXy][iScan4Idx],LD32(pMV));
			ST32(pCurDqLayer->pMvd[LIST_0][iMbXy][iScan4Idx],0);
			if(pMotionVector!=NULL){
				ST32(pMotionVector[LIST_0][iCacheIdx],LD32(pMV));
			}
			if(pMvdCache!=NULL){
				ST32(pMvdCache[LIST_0][iCacheIdx],0);
			}
			*(uint32_t*)pMV=*(uint32_t*)pMvDirect[LIST_1];
			ST32(pCurDqLayer->pDec->pMv[LIST_1][iMbXy][iScan4Idx],LD32(pMV));
			ST32(pCurDqLayer->pMvd[LIST_1][iMbXy][iScan4Idx],0);
			if(pMotionVector!=NULL){
				ST32(pMotionVector[LIST_1][iCacheIdx],LD32(pMV));
			}
			if(pMvdCache!=NULL){
				ST32(pMvdCache[LIST_1][iCacheIdx],0);
			}
		}
		if((*(int32_t*)pMvDirect[LIST_0]|*(int32_t*)pMvDirect[LIST_1])){
			uint32_t uiColZeroFlag=(0==pCurDqLayer->iColocIntra[iColocIdx]) && !bIsLongRef && 
				(pCurDqLayer->iColocRefIndex[LIST_0][iColocIdx]==0 || (pCurDqLayer->iColocRefIndex[LIST_0][iColocIdx]<0
					 && pCurDqLayer->iColocRefIndex[LIST_1][iColocIdx]==0));
			const int16_t(*mvColoc)[2]=0==pCurDqLayer->iColocRefIndex[LIST_0][iColocIdx] ? pCurDqLayer->iColocMv[LIST_0] :
				pCurDqLayer->iColocMv[LIST_1];
			const int16_t* mv=mvColoc[iColocIdx];
			if(IS_SUB_8x8(subMbType)){
				if(uiColZeroFlag && ((unsigned)(mv[0]+1)<=2 && (unsigned)(mv[1]+1)<=2)){
					if(iRef[LIST_0]==0){
						ST64(pCurDqLayer->pDec->pMv[LIST_0][iMbXy][iScan4Idx],0);
						ST64(pCurDqLayer->pDec->pMv[LIST_0][iMbXy][iScan4Idx+4],0);
						ST64(pCurDqLayer->pMvd[LIST_0][iMbXy][iScan4Idx],0);
						ST64(pCurDqLayer->pMvd[LIST_0][iMbXy][iScan4Idx+4],0);
						if(pMotionVector!=NULL){
							ST64(pMotionVector[LIST_0][iCacheIdx],0);
							ST64(pMotionVector[LIST_0][iCacheIdx+6],0);
						}
						if(pMvdCache!=NULL){
							ST64(pMvdCache[LIST_0][iCacheIdx],0);
							ST64(pMvdCache[LIST_0][iCacheIdx+6],0);
						}
					}

					if(iRef[LIST_1]==0){
						ST64(pCurDqLayer->pDec->pMv[LIST_1][iMbXy][iScan4Idx],0);
						ST64(pCurDqLayer->pDec->pMv[LIST_1][iMbXy][iScan4Idx+4],0);
						ST64(pCurDqLayer->pMvd[LIST_1][iMbXy][iScan4Idx],0);
						ST64(pCurDqLayer->pMvd[LIST_1][iMbXy][iScan4Idx+4],0);
						if(pMotionVector!=NULL){
							ST64(pMotionVector[LIST_1][iCacheIdx],0);
							ST64(pMotionVector[LIST_1][iCacheIdx+6],0);
						}
						if(pMvdCache!=NULL){
							ST64(pMvdCache[LIST_1][iCacheIdx],0);
							ST64(pMvdCache[LIST_1][iCacheIdx+6],0);
						}
					}
				}
			}else{
				if(uiColZeroFlag && ((unsigned)(mv[0]+1)<=2 && (unsigned)(mv[1]+1)<=2)){
					if(iRef[LIST_0]==0){
						ST32(pCurDqLayer->pDec->pMv[LIST_0][iMbXy][iScan4Idx],0);
						ST32(pCurDqLayer->pMvd[LIST_0][iMbXy][iScan4Idx],0);
						if(pMotionVector!=NULL){
							ST32(pMotionVector[LIST_0][iCacheIdx],0);
						}
						if(pMvdCache!=NULL){
							ST32(pMvdCache[LIST_0][iCacheIdx],0);
						}
					}
					if(iRef[LIST_1]==0){
						ST32(pCurDqLayer->pDec->pMv[LIST_1][iMbXy][iScan4Idx],0);
						ST32(pCurDqLayer->pMvd[LIST_1][iMbXy][iScan4Idx],0);
						if(pMotionVector!=NULL){
							ST32(pMotionVector[LIST_1][iCacheIdx],0);
						}
						if(pMvdCache!=NULL){
							ST32(pMvdCache[LIST_1][iCacheIdx],0);
						}
					}
				}
			}
		}
	}
}

int32_t PredMvBDirectSpatial(SDecoderContext* pCtx,int16_t iMvp[LIST_A][2],int8_t ref[LIST_A],SubMbType& subMbType){

	int32_t ret=ERR_NONE;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	bool bSkipOrDirect=(IS_SKIP(GetMbType(pCurDqLayer)[iMbXy])|IS_DIRECT(GetMbType(pCurDqLayer)[iMbXy]))>0;

	MbType mbType;
	ret=GetColocatedMb(pCtx,mbType,subMbType);
	if(ret!=ERR_NONE){
		return ret;
	}

	bool bTopAvail,bLeftTopAvail,bRightTopAvail,bLeftAvail;
	int32_t iLeftTopType,iRightTopType,iTopType,iLeftType;
	int32_t iCurSliceIdc,iTopSliceIdc,iLeftTopSliceIdc,iRightTopSliceIdc,iLeftSliceIdc;
	int32_t iCurX,iCurY,iCurXy,iLeftXy=0,iTopXy=0,iLeftTopXy=0,iRightTopXy=0;

	int8_t iLeftRef[LIST_A];
	int8_t iTopRef[LIST_A];
	int8_t iRightTopRef[LIST_A];
	int8_t iLeftTopRef[LIST_A];
	int8_t iDiagonalRef[LIST_A];
	int16_t iMvA[LIST_A][2],iMvB[LIST_A][2],iMvC[LIST_A][2],iMvD[LIST_A][2];

	iCurXy=pCurDqLayer->iMbXyIndex;

	iCurX=pCurDqLayer->iMbX;
	iCurY=pCurDqLayer->iMbY;
	iCurSliceIdc=pCurDqLayer->pSliceIdc[iCurXy];

	if(iCurX!=0){
		iLeftXy=iCurXy-1;
		iLeftSliceIdc=pCurDqLayer->pSliceIdc[iLeftXy];
		bLeftAvail=(iLeftSliceIdc==iCurSliceIdc);
	}else{
		bLeftAvail=0;
		bLeftTopAvail=0;
	}

	if(iCurY!=0){
		iTopXy=iCurXy-pCurDqLayer->iMbWidth;
		iTopSliceIdc=pCurDqLayer->pSliceIdc[iTopXy];
		bTopAvail=(iTopSliceIdc==iCurSliceIdc);
		if(iCurX!=0){
			iLeftTopXy=iTopXy-1;
			iLeftTopSliceIdc=pCurDqLayer->pSliceIdc[iLeftTopXy];
			bLeftTopAvail=(iLeftTopSliceIdc==iCurSliceIdc);
		}else{
			bLeftTopAvail=0;
		}
		if(iCurX!=(pCurDqLayer->iMbWidth-1)){
			iRightTopXy=iTopXy+1;
			iRightTopSliceIdc=pCurDqLayer->pSliceIdc[iRightTopXy];
			bRightTopAvail=(iRightTopSliceIdc==iCurSliceIdc);
		}else{
			bRightTopAvail=0;
		}
	}else{
		bTopAvail=0;
		bLeftTopAvail=0;
		bRightTopAvail=0;
	}

	iLeftType=((iCurX!=0 && bLeftAvail) ? GetMbType(pCurDqLayer)[iLeftXy] : 0);
	iTopType=((iCurY!=0 && bTopAvail) ? GetMbType(pCurDqLayer)[iTopXy] : 0);
	iLeftTopType=((iCurX!=0 && iCurY!=0 && bLeftTopAvail)
					? GetMbType(pCurDqLayer)[iLeftTopXy] : 0);
	iRightTopType=((iCurX!=pCurDqLayer->iMbWidth-1 && iCurY!=0 && bRightTopAvail)
					 ? GetMbType(pCurDqLayer)[iRightTopXy] : 0);

	// get neb mv&iRefIdxArray
	for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){

		// left
		if(bLeftAvail && IS_INTER(iLeftType)){
			ST32(iMvA[listIdx],LD32(pCurDqLayer->pDec ? pCurDqLayer->pDec->pMv[listIdx][iLeftXy][3] :
				pCurDqLayer->pMv[listIdx][iLeftXy][3]));
			iLeftRef[listIdx]=pCurDqLayer->pDec ? pCurDqLayer->pDec->pRefIndex[listIdx][iLeftXy][3] :
				pCurDqLayer->pRefIndex[listIdx][iLeftXy][3];
		}else{
			ST32(iMvA[listIdx],0);
			if(0==bLeftAvail){		// not available
				iLeftRef[listIdx]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iLeftRef[listIdx]=REF_NOT_IN_LIST;
			}
		}

		// top
		if(bTopAvail && IS_INTER(iTopType)){
			ST32(iMvB[listIdx],LD32(pCurDqLayer->pDec ? pCurDqLayer->pDec->pMv[listIdx][iTopXy][12] :
				pCurDqLayer->pMv[listIdx][iTopXy][12]));
			iTopRef[listIdx]=pCurDqLayer->pDec ? pCurDqLayer->pDec->pRefIndex[listIdx][iTopXy][12] :
				pCurDqLayer->pRefIndex[listIdx][iTopXy][12];
		}else{
			ST32(iMvB[listIdx],0);
			if(0==bTopAvail){		// not available
				iTopRef[listIdx]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iTopRef[listIdx]=REF_NOT_IN_LIST;
			}
		}

		// right_top
		if(bRightTopAvail && IS_INTER(iRightTopType)){
			ST32(iMvC[listIdx],LD32(pCurDqLayer->pDec ? pCurDqLayer->pDec->pMv[listIdx][iRightTopXy][12] : pCurDqLayer->pMv[listIdx][iRightTopXy][12]));
			iRightTopRef[listIdx]=pCurDqLayer->pDec ? pCurDqLayer->pDec->pRefIndex[listIdx][iRightTopXy][12] : pCurDqLayer->pRefIndex[listIdx][iRightTopXy][12];
		}else{
			ST32(iMvC[listIdx],0);
			if(0==bRightTopAvail){		// not available
				iRightTopRef[listIdx]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRightTopRef[listIdx]=REF_NOT_IN_LIST;
			}
		}
		// left_top
		if(bLeftTopAvail && IS_INTER(iLeftTopType)){
			ST32(iMvD[listIdx],LD32(pCurDqLayer->pDec ? pCurDqLayer->pDec->pMv[listIdx][iLeftTopXy][15] : pCurDqLayer->pMv[listIdx][iLeftTopXy][15]));
			iLeftTopRef[listIdx]=pCurDqLayer->pDec ? pCurDqLayer->pDec->pRefIndex[listIdx][iLeftTopXy][15] : pCurDqLayer->pRefIndex[listIdx][iLeftTopXy][15];
		}else{
			ST32(iMvD[listIdx],0);
			if(0==bLeftTopAvail){		// not available
				iLeftTopRef[listIdx]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iLeftTopRef[listIdx]=REF_NOT_IN_LIST;
			}
		}

		iDiagonalRef[listIdx]=iRightTopRef[listIdx];
		if(REF_NOT_AVAIL==iDiagonalRef[listIdx]){
			iDiagonalRef[listIdx]=iLeftTopRef[listIdx];
			ST32(iMvC[listIdx],LD32(iMvD[listIdx]));
		}

		int8_t ref_temp=WELS_MIN_POSITIVE(iTopRef[listIdx],iDiagonalRef[listIdx]);
		ref[listIdx]=WELS_MIN_POSITIVE(iLeftRef[listIdx],ref_temp);
		if(ref[listIdx]>=0){

			uint32_t match_count=(iLeftRef[listIdx]==ref[listIdx])+(iTopRef[listIdx]==ref[listIdx])+(iDiagonalRef[listIdx]==ref[listIdx]);
			if(match_count==1){
				if(iLeftRef[listIdx]==ref[listIdx]){
					ST32(iMvp[listIdx],LD32(iMvA[listIdx]));
				}else
				if(iTopRef[listIdx]==ref[listIdx]){
					ST32(iMvp[listIdx],LD32(iMvB[listIdx]));
				}else{
					ST32(iMvp[listIdx],LD32(iMvC[listIdx]));
				}
			}else{
				iMvp[listIdx][0]=WelsMedian(iMvA[listIdx][0],iMvB[listIdx][0],iMvC[listIdx][0]);
				iMvp[listIdx][1]=WelsMedian(iMvA[listIdx][1],iMvB[listIdx][1],iMvC[listIdx][1]);
			}
		}else{
			iMvp[listIdx][0]=0;
			iMvp[listIdx][1]=0;
			ref[listIdx]=REF_NOT_IN_LIST;
		}
	}
	if(ref[LIST_0]<=REF_NOT_IN_LIST && ref[LIST_1]<=REF_NOT_IN_LIST){
		ref[LIST_0]=ref[LIST_1]=0;
	}else
	if(ref[LIST_1]<0){
		mbType&=~MB_TYPE_L1;
		subMbType&=~MB_TYPE_L1;
	}else
	if(ref[LIST_0]<0){
		mbType&=~MB_TYPE_L0;
		subMbType&=~MB_TYPE_L0;
	}
	GetMbType(pCurDqLayer)[iMbXy]=mbType;

	int16_t pMvd[4]={0};

	bool bIsLongRef=pCtx->sRefPic.pRefList[LIST_1][0]->bIsLongRef;

	if(IS_INTER_16x16(mbType)){
		if((*(int32_t*)iMvp[LIST_0]|*(int32_t*)iMvp[LIST_1])){
			if(0==pCurDqLayer->iColocIntra[0] && !bIsLongRef
				 && ((pCurDqLayer->iColocRefIndex[LIST_0][0]==0 && (unsigned)(pCurDqLayer->iColocMv[LIST_0][0][0]+1)<=2
					 && (unsigned)(pCurDqLayer->iColocMv[LIST_0][0][1]+1)<=2)
					 || (pCurDqLayer->iColocRefIndex[LIST_0][0]<0 && pCurDqLayer->iColocRefIndex[LIST_1][0]==0
						 && (unsigned)(pCurDqLayer->iColocMv[LIST_1][0][0]+1)<=2
						 && (unsigned)(pCurDqLayer->iColocMv[LIST_1][0][1]+1)<=2))){
				if(0>=ref[0]) *(uint32_t*)iMvp[LIST_0]=0;
				if(0>=ref[1]) *(uint32_t*)iMvp[LIST_1]=0;
			}
		}
		UpdateP16x16DirectCabac(pCurDqLayer);
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			UpdateP16x16MotionInfo(pCurDqLayer,listIdx,ref[listIdx],iMvp[listIdx]);
			UpdateP16x16MvdCabac(pCurDqLayer,pMvd,listIdx);
		}
	}else{
		if(bSkipOrDirect){
			int8_t pSubPartCount[4],pPartW[4];
			for(int32_t i=0; i<4; i++){		// Direct 8x8 Ref and mv
				int16_t iIdx8=i<<2;
				pCurDqLayer->pSubMbType[iMbXy][i]=subMbType;
				int8_t pRefIndex[LIST_A][30];
				UpdateP8x8RefIdxCabac(pCurDqLayer,pRefIndex,iIdx8,ref[LIST_0],LIST_0);
				UpdateP8x8RefIdxCabac(pCurDqLayer,pRefIndex,iIdx8,ref[LIST_1],LIST_1);
				UpdateP8x8DirectCabac(pCurDqLayer,iIdx8);

				pSubPartCount[i]=g_ksInterBSubMbTypeInfo[0].iPartCount;
				pPartW[i]=g_ksInterBSubMbTypeInfo[0].iPartWidth;

				if(IS_SUB_4x4(subMbType)){
					pSubPartCount[i]=4;
					pPartW[i]=1;
				}
				FillSpatialDirect8x8Mv(pCurDqLayer,iIdx8,pSubPartCount[i],pPartW[i],subMbType,bIsLongRef,iMvp,ref,NULL,NULL);
			}
		}
	}
	return ret;
}

// update iRefIndex cache for current MB,only for P_16*16 (SKIP inclusive)
// can be further optimized
void UpdateP16x16RefIdx(PDqLayer pCurDqLayer,int32_t listIdx,int8_t iRef){
	const int16_t kiRef2=((uint8_t)iRef<<8)|(uint8_t)iRef;
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;

	for(i=0; i<16; i+=4){
		// mb
		const uint8_t kuiScan4Idx=g_kuiScan4[i];
		const uint8_t kuiScan4IdxPlus4=4+kuiScan4Idx;

		ST16(&pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4Idx],kiRef2);
		ST16(&pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][kuiScan4IdxPlus4],kiRef2);
	}
}

// update iMVs only cache for current MB,only for P_16*16 (SKIP inclusive)
// can be further optimized
void UpdateP16x16MotionOnly(PDqLayer pCurDqLayer,int32_t listIdx,int16_t iMVs[2]){
	const int32_t kiMV32=LD32(iMVs);
	int32_t i;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;

	for(i=0; i<16; i+=4){
		// mb
		const uint8_t kuiScan4Idx=g_kuiScan4[i];
		const uint8_t kuiScan4IdxPlus4=4+kuiScan4Idx;
		if(pCurDqLayer->pDec!=NULL){
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}else{
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][1+kuiScan4Idx],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][kuiScan4IdxPlus4],kiMV32);
			ST32(pCurDqLayer->pMv[listIdx][iMbXy][1+kuiScan4IdxPlus4],kiMV32);
		}
	}
}

int8_t MapColToList0(SDecoderContext* pCtx,const int8_t& colocRefIndexL0,const int32_t& ref0Count){		// ISO/IEC 14496-10:2009(E) (8-193)
	// When reference is lost,this function must be skipped.
	if((pCtx->iErrorCode&dsRefLost)==dsRefLost){
		return 0;
	}
	SPicture* pic1=pCtx->sRefPic.pRefList[LIST_1][0];
	if(pic1 && pic1->pRefPic[LIST_0][colocRefIndexL0]){
		const int32_t iFramePoc=pic1->pRefPic[LIST_0][colocRefIndexL0]->iFramePoc;
		for(int32_t i=0; i<ref0Count; i++){
			if(pCtx->sRefPic.pRefList[LIST_0][i]->iFramePoc==iFramePoc){
				return i;
			}
		}
	}
	return 0;
}

void FillTemporalDirect8x8Mv(PDqLayer pCurDqLayer,const int16_t& iIdx8,const int8_t& iPartCount,const int8_t& iPartW,const SubMbType& subMbType,int8_t iRef[LIST_A],int16_t(*mvColoc)[2],int16_t pMotionVector[LIST_A][30][MV_A],int16_t pMvdCache[LIST_A][30][MV_A]){
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int16_t pMvDirect[LIST_A][2]={{0,0},{0,0}};
	for(int32_t j=0; j<iPartCount; j++){
		int8_t iPartIdx=iIdx8+j*iPartW;
		uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
		uint8_t iColocIdx=g_kuiScan4[iPartIdx];
		uint8_t iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];

		int16_t* mv=mvColoc[iColocIdx];

		int16_t pMV[4]={0};
		if(IS_SUB_8x8(subMbType)){
			if(!pCurDqLayer->iColocIntra[iColocIdx]){
				pMvDirect[LIST_0][0]=(pSlice->iMvScale[LIST_0][iRef[LIST_0]]*mv[0]+128)>>8;
				pMvDirect[LIST_0][1]=(pSlice->iMvScale[LIST_0][iRef[LIST_0]]*mv[1]+128)>>8;
			}
			ST32(pMV,LD32(pMvDirect[LIST_0]));
			ST32((pMV+2),LD32(pMvDirect[LIST_0]));
			ST64(pCurDqLayer->pDec->pMv[LIST_0][iMbXy][iScan4Idx],LD64(pMV));
			ST64(pCurDqLayer->pDec->pMv[LIST_0][iMbXy][iScan4Idx+4],LD64(pMV));
			ST64(pCurDqLayer->pMvd[LIST_0][iMbXy][iScan4Idx],0);
			ST64(pCurDqLayer->pMvd[LIST_0][iMbXy][iScan4Idx+4],0);
			if(pMotionVector!=NULL){
				ST64(pMotionVector[LIST_0][iCacheIdx],LD64(pMV));
				ST64(pMotionVector[LIST_0][iCacheIdx+6],LD64(pMV));
			}
			if(pMvdCache!=NULL){
				ST64(pMvdCache[LIST_0][iCacheIdx],0);
				ST64(pMvdCache[LIST_0][iCacheIdx+6],0);
			}
			if(!pCurDqLayer->iColocIntra[g_kuiScan4[iIdx8]]){
				pMvDirect[LIST_1][0]=pMvDirect[LIST_0][0]-mv[0];
				pMvDirect[LIST_1][1]=pMvDirect[LIST_0][1]-mv[1];
			}
			ST32(pMV,LD32(pMvDirect[LIST_1]));
			ST32((pMV+2),LD32(pMvDirect[LIST_1]));
			ST64(pCurDqLayer->pDec->pMv[LIST_1][iMbXy][iScan4Idx],LD64(pMV));
			ST64(pCurDqLayer->pDec->pMv[LIST_1][iMbXy][iScan4Idx+4],LD64(pMV));
			ST64(pCurDqLayer->pMvd[LIST_1][iMbXy][iScan4Idx],0);
			ST64(pCurDqLayer->pMvd[LIST_1][iMbXy][iScan4Idx+4],0);
			if(pMotionVector!=NULL){
				ST64(pMotionVector[LIST_1][iCacheIdx],LD64(pMV));
				ST64(pMotionVector[LIST_1][iCacheIdx+6],LD64(pMV));
			}
			if(pMvdCache!=NULL){
				ST64(pMvdCache[LIST_1][iCacheIdx],0);
				ST64(pMvdCache[LIST_1][iCacheIdx+6],0);
			}
		}else{		// SUB_4x4
			if(!pCurDqLayer->iColocIntra[iColocIdx]){
				pMvDirect[LIST_0][0]=(pSlice->iMvScale[LIST_0][iRef[LIST_0]]*mv[0]+128)>>8;
				pMvDirect[LIST_0][1]=(pSlice->iMvScale[LIST_0][iRef[LIST_0]]*mv[1]+128)>>8;
			}
			ST32(pCurDqLayer->pDec->pMv[LIST_0][iMbXy][iScan4Idx],LD32(pMvDirect[LIST_0]));
			ST32(pCurDqLayer->pMvd[LIST_0][iMbXy][iScan4Idx],0);
			if(pMotionVector!=NULL){
				ST32(pMotionVector[LIST_0][iCacheIdx],LD32(pMvDirect[LIST_0]));
			}
			if(pMvdCache!=NULL){
				ST32(pMvdCache[LIST_0][iCacheIdx],0);
			}
			if(!pCurDqLayer->iColocIntra[iColocIdx]){
				pMvDirect[LIST_1][0]=pMvDirect[LIST_0][0]-mv[0];
				pMvDirect[LIST_1][1]=pMvDirect[LIST_0][1]-mv[1];
			}
			ST32(pCurDqLayer->pDec->pMv[LIST_1][iMbXy][iScan4Idx],LD32(pMvDirect[LIST_1]));
			ST32(pCurDqLayer->pMvd[LIST_1][iMbXy][iScan4Idx],0);
			if(pMotionVector!=NULL){
				ST32(pMotionVector[LIST_1][iCacheIdx],LD32(pMvDirect[LIST_1]));
			}
			if(pMvdCache!=NULL){
				ST32(pMvdCache[LIST_1][iCacheIdx],0);
			}
		}
	}
}

int32_t PredBDirectTemporal(SDecoderContext* pCtx,int16_t iMvp[LIST_A][2],int8_t ref[LIST_A],SubMbType& subMbType){
	int32_t ret=ERR_NONE;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	bool bSkipOrDirect=(IS_SKIP(GetMbType(pCurDqLayer)[iMbXy])|IS_DIRECT(GetMbType(pCurDqLayer)[iMbXy]))>0;

	MbType mbType;
	ret=GetColocatedMb(pCtx,mbType,subMbType);
	if(ret!=ERR_NONE){
		return ret;
	}

	GetMbType(pCurDqLayer)[iMbXy]=mbType;

	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	int16_t pMvd[4]={0};
	const int32_t ref0Count=WELS_MIN(pSliceHeader->uiRefCount[LIST_0],pCtx->sRefPic.uiRefCount[LIST_0]);
	if(IS_INTER_16x16(mbType)){
		ref[LIST_0]=0;
		ref[LIST_1]=0;
		UpdateP16x16DirectCabac(pCurDqLayer);
		UpdateP16x16RefIdx(pCurDqLayer,LIST_1,ref[LIST_1]);
		ST64(iMvp,0);
		if(pCurDqLayer->iColocIntra[0]){
			UpdateP16x16MotionOnly(pCurDqLayer,LIST_0,iMvp[LIST_0]);
			UpdateP16x16MotionOnly(pCurDqLayer,LIST_1,iMvp[LIST_1]);
			UpdateP16x16RefIdx(pCurDqLayer,LIST_0,ref[LIST_0]);
		}else{
			ref[LIST_0]=0;
			int16_t* mv=pCurDqLayer->iColocMv[LIST_0][0];
			int8_t colocRefIndexL0=pCurDqLayer->iColocRefIndex[LIST_0][0];
			if(colocRefIndexL0>=0){
				ref[LIST_0]=MapColToList0(pCtx,colocRefIndexL0,ref0Count);
			}else{
				mv=pCurDqLayer->iColocMv[LIST_1][0];
			}
			UpdateP16x16RefIdx(pCurDqLayer,LIST_0,ref[LIST_0]);

			iMvp[LIST_0][0]=(pSlice->iMvScale[LIST_0][ref[LIST_0]]*mv[0]+128)>>8;
			iMvp[LIST_0][1]=(pSlice->iMvScale[LIST_0][ref[LIST_0]]*mv[1]+128)>>8;
			UpdateP16x16MotionOnly(pCurDqLayer,LIST_0,iMvp[LIST_0]);
			iMvp[LIST_1][0]=iMvp[LIST_0][0]-mv[0];
			iMvp[LIST_1][1]=iMvp[LIST_0][1]-mv[1];
			UpdateP16x16MotionOnly(pCurDqLayer,LIST_1,iMvp[LIST_1]);
		}
		UpdateP16x16MvdCabac(pCurDqLayer,pMvd,LIST_0);
		UpdateP16x16MvdCabac(pCurDqLayer,pMvd,LIST_1);
	}else{
		if(bSkipOrDirect){
			int8_t pSubPartCount[4],pPartW[4];
			int8_t pRefIndex[LIST_A][30];
			for(int32_t i=0; i<4; i++){
				int16_t iIdx8=i<<2;
				const uint8_t iScan4Idx=g_kuiScan4[iIdx8];
				pCurDqLayer->pSubMbType[iMbXy][i]=subMbType;

				int16_t(*mvColoc)[2]=pCurDqLayer->iColocMv[LIST_0];

				ref[LIST_1]=0;
				UpdateP8x8RefIdxCabac(pCurDqLayer,pRefIndex,iIdx8,ref[LIST_1],LIST_1);
				if(pCurDqLayer->iColocIntra[iScan4Idx]){
					ref[LIST_0]=0;
					UpdateP8x8RefIdxCabac(pCurDqLayer,pRefIndex,iIdx8,ref[LIST_0],LIST_0);
					ST64(iMvp,0);
				}else{
					ref[LIST_0]=0;
					int8_t colocRefIndexL0=pCurDqLayer->iColocRefIndex[LIST_0][iScan4Idx];
					if(colocRefIndexL0>=0){
						ref[LIST_0]=MapColToList0(pCtx,colocRefIndexL0,ref0Count);
					}else{
						mvColoc=pCurDqLayer->iColocMv[LIST_1];
					}
					UpdateP8x8RefIdxCabac(pCurDqLayer,pRefIndex,iIdx8,ref[LIST_0],LIST_0);
				}
				UpdateP8x8DirectCabac(pCurDqLayer,iIdx8);

				pSubPartCount[i]=g_ksInterBSubMbTypeInfo[0].iPartCount;
				pPartW[i]=g_ksInterBSubMbTypeInfo[0].iPartWidth;

				if(IS_SUB_4x4(subMbType)){
					pSubPartCount[i]=4;
					pPartW[i]=1;
				}
				FillTemporalDirect8x8Mv(pCurDqLayer,iIdx8,pSubPartCount[i],pPartW[i],subMbType,ref,mvColoc,NULL,NULL);
			}
		}
	}
	return ret;
}

static uint32_t DecodeCabacIntraMbType(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,int ctx_base){
	uint32_t uiCode;
	uint32_t uiMbType=0;

	SWelsCabacDecEngine* pCabacDecEngine=pCtx->pCabacDecEngine;
	SWelsCabacCtx* pBinCtx=pCtx->pCabacCtx+ctx_base;

	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx,uiCode));
	if(!uiCode){
		return 0;		// I4x4
	}

	WELS_READ_VERIFY(DecodeTerminateCabac(pCabacDecEngine,uiCode));
	if(uiCode){
		return 25;		// PCM
	}
	uiMbType=1;		// I16x16
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+1,uiCode));		// cbp_luma !=0
	uiMbType+=12*uiCode;

	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+2,uiCode));
	if(uiCode){
		WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+2,uiCode));
		uiMbType+=4+4*uiCode;
	}
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
	uiMbType+=2*uiCode;
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
	uiMbType+=1*uiCode;
	return uiMbType;
}

int32_t ParseMBTypeBSliceCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint32_t& uiMbType){
	uint32_t uiCode;
	uiMbType=0;
	int32_t iIdxA=0,iIdxB=0;
	int32_t iCtxInc;

	SWelsCabacDecEngine* pCabacDecEngine=pCtx->pCabacDecEngine;
	SWelsCabacCtx* pBinCtx=pCtx->pCabacCtx+27;		// B slice

	iIdxA=(pNeighAvail->iLeftAvail) && !IS_DIRECT(pNeighAvail->iLeftType);
	iIdxB=(pNeighAvail->iTopAvail) && !IS_DIRECT(pNeighAvail->iTopType);

	iCtxInc=iIdxA+iIdxB;
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+iCtxInc,uiCode));
	if(!uiCode)
		uiMbType=0;		// Bi_Direct
	else{
		WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
		if(!uiCode){
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+5,uiCode));
			uiMbType=1+uiCode;		// 16x16 L0L1
		}else{
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+4,uiCode));
			uiMbType=uiCode<<3;
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+5,uiCode));
			uiMbType|=uiCode<<2;
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+5,uiCode));
			uiMbType|=uiCode<<1;
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+5,uiCode));
			uiMbType|=uiCode;
			if(uiMbType<8){
				uiMbType+=3;
				return ERR_NONE;
			}else
			if(uiMbType==13){
				uiMbType=DecodeCabacIntraMbType(pCtx,pNeighAvail,32)+23;
				return ERR_NONE;
			}else
			if(uiMbType==14){
				uiMbType=11;		// Bi8x16
				return ERR_NONE;
			}else
			if(uiMbType==15){
				uiMbType=22;		// 8x8
				return ERR_NONE;
			}
			uiMbType<<=1;
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+5,uiCode));
			uiMbType|=uiCode;
			uiMbType-=4;
		}
	}
	return ERR_NONE;
}

// Table 7.14. Macroblock type values 0 to 22 for B slices.
static const SPartMbInfo g_ksInterBMbTypeInfo[]={
	// Part 0 Part 1
	{MB_TYPE_DIRECT,1,4},// B_Direct_16x16
 {MB_TYPE_16x16|MB_TYPE_P0L0,1,4},// B_L0_16x16
 {MB_TYPE_16x16|MB_TYPE_P0L1,1,4},// B_L1_16x16
 {MB_TYPE_16x16|MB_TYPE_P0L0|MB_TYPE_P0L1,1,4},		// B_Bi_16x16
 {MB_TYPE_16x8|MB_TYPE_P0L0|MB_TYPE_P1L0,2,4},		// B_L0_L0_16x8
 {MB_TYPE_8x16|MB_TYPE_P0L0|MB_TYPE_P1L0,2,2},		// B_L0_L0_8x16
 {MB_TYPE_16x8|MB_TYPE_P0L1|MB_TYPE_P1L1,2,4},		// B_L1_L1_16x8
 {MB_TYPE_8x16|MB_TYPE_P0L1|MB_TYPE_P1L1,2,2},		// B_L1_L1_8x16
 {MB_TYPE_16x8|MB_TYPE_P0L0|MB_TYPE_P1L1,2,4},		// B_L0_L1_16x8
 {MB_TYPE_8x16|MB_TYPE_P0L0|MB_TYPE_P1L1,2,2},		// B_L0_L1_8x16
 {MB_TYPE_16x8|MB_TYPE_P0L1|MB_TYPE_P1L0,2,4},		// B_L1_L0_16x8
 {MB_TYPE_8x16|MB_TYPE_P0L1|MB_TYPE_P1L0,2,2},		// B_L1_L0_8x16
 {MB_TYPE_16x8|MB_TYPE_P0L0|MB_TYPE_P1L0|MB_TYPE_P1L1,2,4},		// B_L0_Bi_16x8
 {MB_TYPE_8x16|MB_TYPE_P0L0|MB_TYPE_P1L0|MB_TYPE_P1L1,2,2},		// B_L0_Bi_8x16
 {MB_TYPE_16x8|MB_TYPE_P0L1|MB_TYPE_P1L0|MB_TYPE_P1L1,2,4},		// B_L1_Bi_16x8
 {MB_TYPE_8x16|MB_TYPE_P0L1|MB_TYPE_P1L0|MB_TYPE_P1L1,2,2},		// B_L1_Bi_8x16
 {MB_TYPE_16x8|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_P1L0,2,4},		// B_Bi_L0_16x8
 {MB_TYPE_8x16|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_P1L0,2,2},		// B_Bi_L0_8x16
 {MB_TYPE_16x8|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_P1L1,2,4},		// B_Bi_L1_16x8
 {MB_TYPE_8x16|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_P1L1,2,2},		// B_Bi_L1_8x16
 {MB_TYPE_16x8|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_P1L0|MB_TYPE_P1L1,2,4},		// B_Bi_Bi_16x8
 {MB_TYPE_8x16|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_P1L0|MB_TYPE_P1L1,2,2},		// B_Bi_Bi_8x16
 {MB_TYPE_8x8|MB_TYPE_P0L0|MB_TYPE_P0L1|MB_TYPE_P1L0|MB_TYPE_P1L1,4,4}		// B_8x8
};

void WelsFillDirectCacheCabac(PWelsNeighAvail pNeighAvail,int8_t iDirect[30],PDqLayer pCurDqLayer){

	int32_t iCurXy=pCurDqLayer->iMbXyIndex;
	int32_t iTopXy=0;
	int32_t iLeftXy=0;
	int32_t iLeftTopXy=0;
	int32_t iRightTopXy=0;

	if(pNeighAvail->iTopAvail){
		iTopXy=iCurXy-pCurDqLayer->iMbWidth;
	}
	if(pNeighAvail->iLeftAvail){
		iLeftXy=iCurXy-1;
	}
	if(pNeighAvail->iLeftTopAvail){
		iLeftTopXy=iCurXy-1-pCurDqLayer->iMbWidth;
	}
	if(pNeighAvail->iRightTopAvail){
		iRightTopXy=iCurXy+1-pCurDqLayer->iMbWidth;
	}
	memset(iDirect,0,30);
	if(pNeighAvail->iLeftAvail && IS_INTER(pNeighAvail->iLeftType)){
		iDirect[6]=pCurDqLayer->pDirect[iLeftXy][3];
		iDirect[12]=pCurDqLayer->pDirect[iLeftXy][7];
		iDirect[18]=pCurDqLayer->pDirect[iLeftXy][11];
		iDirect[24]=pCurDqLayer->pDirect[iLeftXy][15];
	}
	if(pNeighAvail->iLeftTopAvail && IS_INTER(pNeighAvail->iLeftTopType)){
		iDirect[0]=pCurDqLayer->pDirect[iLeftTopXy][15];
	}

	if(pNeighAvail->iTopAvail && IS_INTER(pNeighAvail->iTopType)){
		ST32(&iDirect[1],LD32(&pCurDqLayer->pDirect[iTopXy][12]));
	}

	if(pNeighAvail->iRightTopAvail && IS_INTER(pNeighAvail->iRightTopType)){
		iDirect[5]=pCurDqLayer->pDirect[iRightTopXy][12];
	}
	// right-top 4*4 block unavailable
}

int32_t ParseBSubMBTypeCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint32_t& uiSubMbType){
	uint32_t uiCode;
	SWelsCabacDecEngine* pCabacDecEngine=pCtx->pCabacDecEngine;
	SWelsCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_B_SUBMB_TYPE;
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx,uiCode));
	if(!uiCode){
		uiSubMbType=0;		// B_Direct_8x8
		return ERR_NONE;
	}
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+1,uiCode));
	if(!uiCode){
		WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
		uiSubMbType=1+uiCode;		// B_L0_8x8,B_L1_8x8
		return ERR_NONE;
	}
	uiSubMbType=3;
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+2,uiCode));
	if(uiCode){
		WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
		if(uiCode){
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
			uiSubMbType=11+uiCode;		// B_L1_4x4,B_Bi_4x4
			return ERR_NONE;
		}
		uiSubMbType+=4;
	}
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
	uiSubMbType+=2*uiCode;
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
	uiSubMbType+=uiCode;

	return ERR_NONE;
}

void Update8x8RefIdx(PDqLayer& pCurDqLayer,const int16_t& iPartIdx,const int32_t& listIdx,const int8_t& iRef){
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	const uint8_t iScan4Idx=g_kuiScan4[iPartIdx];
	pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][iScan4Idx]=pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][iScan4Idx+1]=
		pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][iScan4Idx+4]=pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][iScan4Idx+
		5]=iRef;

}

void UpdateP8x8RefCacheIdxCabac(int8_t pRefIndex[LIST_A][30],const int16_t& iPartIdx,const int32_t& listIdx,const int8_t& iRef){
	const uint8_t uiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
	pRefIndex[listIdx][uiCacheIdx]=pRefIndex[listIdx][uiCacheIdx+1]=pRefIndex[listIdx][uiCacheIdx+6]=pRefIndex[listIdx][uiCacheIdx+7]=iRef;
}

int32_t ParseInterBMotionInfoCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCount,int16_t pMotionVector[LIST_A][30][MV_A],int16_t pMvdCache[LIST_A][30][MV_A],int8_t pRefIndex[LIST_A][30],int8_t pDirect[30]){
	SSlice* pSlice=&pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	int32_t pRefCount[LIST_A];
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int16_t pMv[4]={0};
	int16_t pMvd[4]={0};
	int8_t iRef[LIST_A]={0};
	int32_t iPartIdx;
	int16_t iMinVmv=pSliceHeader->pSps->pSLevelLimits->iMinVmv;
	int16_t iMaxVmv=pSliceHeader->pSps->pSLevelLimits->iMaxVmv;
	pRefCount[0]=pSliceHeader->uiRefCount[0];
	pRefCount[1]=pSliceHeader->uiRefCount[1];

	MbType mbType=pCurDqLayer->pDec->pMbType[iMbXy];

	bool bIsPending=false;	// GetThreadCount (pCtx) > 1;

	if(IS_DIRECT(mbType)){

		int16_t pMvDirect[LIST_A][2]={{0,0},{0,0}};
		SubMbType subMbType;
		if(pSliceHeader->iDirectSpatialMvPredFlag){
			// predict direct spatial mv
			int32_t ret=PredMvBDirectSpatial(pCtx,pMvDirect,iRef,subMbType);
			if(ret!=ERR_NONE){
				return ret;
			}
		}else{
			// temporal direct 16x16 mode
			int32_t ret=PredBDirectTemporal(pCtx,pMvDirect,iRef,subMbType);
			if(ret!=ERR_NONE){
				return ret;
			}
		}
	}else
	if(IS_INTER_16x16(mbType)){
		iPartIdx=0;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			iRef[listIdx]=REF_NOT_IN_LIST;
			if(IS_DIR(mbType,0,listIdx)){
				WELS_READ_VERIFY(ParseRefIdxCabac(pCtx,pNeighAvail,pNonZeroCount,pRefIndex,pDirect,listIdx,iPartIdx,pRefCount[listIdx],0,iRef[listIdx]));
				if((iRef[listIdx]<0) || (iRef[listIdx]>=pRefCount[listIdx])
					 || (pCtx->sRefPic.pRefList[listIdx][iRef[listIdx]]==NULL)){		// error ref_idx
					pCtx->bMbRefConcealed=true;
					if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
						iRef[listIdx]=0;
						pCtx->iErrorCode|=dsBitstreamError;
					}else{
						return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
					}
				}
				pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(pCtx->sRefPic.pRefList[listIdx][iRef[listIdx]] && (pCtx->sRefPic.pRefList[listIdx][iRef[listIdx]]->bIsComplete || bIsPending));
			}
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(IS_DIR(mbType,0,listIdx)){
				PredMv(pMotionVector,pRefIndex,listIdx,0,4,iRef[listIdx],pMv);
				WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,listIdx,0,pMvd[0]));
				WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,listIdx,1,pMvd[1]));
				pMv[0]+=pMvd[0];
				pMv[1]+=pMvd[1];
				WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
			}else{
				*(uint32_t*)pMv=*(uint32_t*)pMvd=0;
			}
			UpdateP16x16MotionInfo(pCurDqLayer,listIdx,iRef[listIdx],pMv);
			UpdateP16x16MvdCabac(pCurDqLayer,pMvd,listIdx);
		}
	}else
	if(IS_INTER_16x8(mbType)){
		int8_t ref_idx_list[LIST_A][2]={{REF_NOT_IN_LIST,REF_NOT_IN_LIST},{REF_NOT_IN_LIST,REF_NOT_IN_LIST}};
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<2;++i){
				iPartIdx=i<<3;
				int8_t ref_idx=REF_NOT_IN_LIST;
				if(IS_DIR(mbType,i,listIdx)){
					WELS_READ_VERIFY(ParseRefIdxCabac(pCtx,pNeighAvail,pNonZeroCount,pRefIndex,pDirect,listIdx,iPartIdx,pRefCount[listIdx],0,ref_idx));
					if((ref_idx<0) || (ref_idx>=pRefCount[listIdx])
						 || (pCtx->sRefPic.pRefList[listIdx][ref_idx]==NULL)){		// error ref_idx
						pCtx->bMbRefConcealed=true;
						if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
							ref_idx=0;
							pCtx->iErrorCode|=dsBitstreamError;
						}else{
							return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
						}
					}
					pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(pCtx->sRefPic.pRefList[listIdx][ref_idx] && (pCtx->sRefPic.pRefList[listIdx][ref_idx]->bIsComplete || bIsPending));
				}
				UpdateP16x8RefIdxCabac(pCurDqLayer,pRefIndex,iPartIdx,ref_idx,listIdx);
				ref_idx_list[listIdx][i]=ref_idx;
			}
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<2;++i){
				iPartIdx=i<<3;
				int8_t ref_idx=ref_idx_list[listIdx][i];
				if(IS_DIR(mbType,i,listIdx)){
					PredInter16x8Mv(pMotionVector,pRefIndex,listIdx,iPartIdx,ref_idx,pMv);
					WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,listIdx,0,pMvd[0]));
					WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,listIdx,1,pMvd[1]));
					pMv[0]+=pMvd[0];
					pMv[1]+=pMvd[1];
					WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
				}else{
					*(uint32_t*)pMv=*(uint32_t*)pMvd=0;
				}
				UpdateP16x8MotionInfo(pCurDqLayer,pMotionVector,pRefIndex,listIdx,iPartIdx,ref_idx,pMv);
				UpdateP16x8MvdCabac(pCurDqLayer,pMvdCache,iPartIdx,pMvd,listIdx);
			}
		}
	}else
	if(IS_INTER_8x16(mbType)){
		int8_t ref_idx_list[LIST_A][2]={{REF_NOT_IN_LIST,REF_NOT_IN_LIST},{REF_NOT_IN_LIST,REF_NOT_IN_LIST}};
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<2;++i){
				iPartIdx=i<<2;
				int8_t ref_idx=REF_NOT_IN_LIST;
				if(IS_DIR(mbType,i,listIdx)){
					WELS_READ_VERIFY(ParseRefIdxCabac(pCtx,pNeighAvail,pNonZeroCount,pRefIndex,pDirect,listIdx,iPartIdx,pRefCount[listIdx],0,ref_idx));
					if((ref_idx<0) || (ref_idx>=pRefCount[listIdx])
						 || (pCtx->sRefPic.pRefList[listIdx][ref_idx]==NULL)){		// error ref_idx
						pCtx->bMbRefConcealed=true;
						if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
							ref_idx=0;
							pCtx->iErrorCode|=dsBitstreamError;
						}else{
							return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
						}
					}
					pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(pCtx->sRefPic.pRefList[listIdx][ref_idx] && (pCtx->sRefPic.pRefList[listIdx][ref_idx]->bIsComplete || bIsPending));
				}
				UpdateP8x16RefIdxCabac(pCurDqLayer,pRefIndex,iPartIdx,ref_idx,listIdx);
				ref_idx_list[listIdx][i]=ref_idx;
			}
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<2;++i){
				iPartIdx=i<<2;
				int8_t ref_idx=ref_idx_list[listIdx][i];
				if(IS_DIR(mbType,i,listIdx)){
					PredInter8x16Mv(pMotionVector,pRefIndex,listIdx,iPartIdx,ref_idx,pMv);
					WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,listIdx,0,pMvd[0]));
					WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,listIdx,1,pMvd[1]));
					pMv[0]+=pMvd[0];
					pMv[1]+=pMvd[1];
					WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
				}else{
					*(uint32_t*)pMv=*(uint32_t*)pMvd=0;
				}
				UpdateP8x16MotionInfo(pCurDqLayer,pMotionVector,pRefIndex,listIdx,iPartIdx,ref_idx,pMv);
				UpdateP8x16MvdCabac(pCurDqLayer,pMvdCache,iPartIdx,pMvd,listIdx);
			}
		}
	}else
	if(IS_Inter_8x8(mbType)){
		int8_t pSubPartCount[4],pPartW[4];
		uint32_t uiSubMbType;
		// sub_mb_type,partition
		int16_t pMvDirect[LIST_A][2]={{0,0},{0,0}};
		if(pCtx->sRefPic.pRefList[LIST_1][0]==NULL){
			uprintf("Colocated Ref Picture for B-Slice is lost,B-Slice decoding cannot be continued!");
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_DATA,ERR_INFO_REFERENCE_PIC_LOST);
		}
		bool bIsLongRef=pCtx->sRefPic.pRefList[LIST_1][0]->bIsLongRef;
		const int32_t ref0Count=WELS_MIN(pSliceHeader->uiRefCount[LIST_0],pCtx->sRefPic.uiRefCount[LIST_0]);
		bool has_direct_called=false;
		SubMbType directSubMbType=0;
		for(int32_t i=0; i<4; i++){
			WELS_READ_VERIFY(ParseBSubMBTypeCabac(pCtx,pNeighAvail,uiSubMbType));
			if(uiSubMbType>=13){		// invalid sub_mb_type
				return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_SUB_MB_TYPE);
			}
			// pCurDqLayer->pSubMbType[iMbXy][i]=g_ksInterBSubMbTypeInfo[uiSubMbType].iType;
			pSubPartCount[i]=g_ksInterBSubMbTypeInfo[uiSubMbType].iPartCount;
			pPartW[i]=g_ksInterBSubMbTypeInfo[uiSubMbType].iPartWidth;

			// Need modification when B picture add in,reference to 7.3.5
			if(pSubPartCount[i]>1)
				pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=false;

			if(IS_DIRECT(g_ksInterBSubMbTypeInfo[uiSubMbType].iType)){
				if(!has_direct_called){
					if(pSliceHeader->iDirectSpatialMvPredFlag){
						int32_t ret=PredMvBDirectSpatial(pCtx,pMvDirect,iRef,directSubMbType);
						if(ret!=ERR_NONE){
							return ret;
						}

					}else{
						// temporal direct mode
						int32_t ret=PredBDirectTemporal(pCtx,pMvDirect,iRef,directSubMbType);
						if(ret!=ERR_NONE){
							return ret;
						}
					}
					has_direct_called=true;
				}
				pCurDqLayer->pSubMbType[iMbXy][i]=directSubMbType;
				if(IS_SUB_4x4(pCurDqLayer->pSubMbType[iMbXy][i])){
					pSubPartCount[i]=4;
					pPartW[i]=1;
				}
			}else{
				pCurDqLayer->pSubMbType[iMbXy][i]=g_ksInterBSubMbTypeInfo[uiSubMbType].iType;
			}
		}
		for(int32_t i=0; i<4; i++){		// Direct 8x8 Ref and mv
			int16_t iIdx8=i<<2;
			if(IS_DIRECT(pCurDqLayer->pSubMbType[iMbXy][i])){
				if(pSliceHeader->iDirectSpatialMvPredFlag){
					FillSpatialDirect8x8Mv(pCurDqLayer,iIdx8,pSubPartCount[i],pPartW[i],directSubMbType,bIsLongRef,pMvDirect,iRef,pMotionVector,pMvdCache);
				}else{
					int16_t(*mvColoc)[2]=pCurDqLayer->iColocMv[LIST_0];
					iRef[LIST_1]=0;
					iRef[LIST_0]=0;
					const uint8_t uiColoc4Idx=g_kuiScan4[iIdx8];
					if(!pCurDqLayer->iColocIntra[uiColoc4Idx]){
						iRef[LIST_0]=0;
						int8_t colocRefIndexL0=pCurDqLayer->iColocRefIndex[LIST_0][uiColoc4Idx];
						if(colocRefIndexL0>=0){
							iRef[LIST_0]=MapColToList0(pCtx,colocRefIndexL0,ref0Count);
						}else{
							mvColoc=pCurDqLayer->iColocMv[LIST_1];
						}
					}
					Update8x8RefIdx(pCurDqLayer,iIdx8,LIST_0,iRef[LIST_0]);
					Update8x8RefIdx(pCurDqLayer,iIdx8,LIST_1,iRef[LIST_1]);
					UpdateP8x8RefCacheIdxCabac(pRefIndex,iIdx8,LIST_0,iRef[LIST_0]);
					UpdateP8x8RefCacheIdxCabac(pRefIndex,iIdx8,LIST_1,iRef[LIST_1]);
					FillTemporalDirect8x8Mv(pCurDqLayer,iIdx8,pSubPartCount[i],pPartW[i],directSubMbType,iRef,mvColoc,pMotionVector,pMvdCache);
				}
			}
		}
		// ref no-direct
		int8_t ref_idx_list[LIST_A][4]={{REF_NOT_IN_LIST,REF_NOT_IN_LIST},{REF_NOT_IN_LIST,REF_NOT_IN_LIST}};
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<4; i++){
				int16_t iIdx8=i<<2;
				int32_t subMbType=pCurDqLayer->pSubMbType[iMbXy][i];
				int8_t iref=REF_NOT_IN_LIST;
				if(IS_DIRECT(subMbType)){
					if(pSliceHeader->iDirectSpatialMvPredFlag){
						Update8x8RefIdx(pCurDqLayer,iIdx8,listIdx,iRef[listIdx]);
						ref_idx_list[listIdx][i]=iRef[listIdx];
					}
					UpdateP8x8DirectCabac(pCurDqLayer,iIdx8);
				}else{
					if(IS_DIR(subMbType,0,listIdx)){
						WELS_READ_VERIFY(ParseRefIdxCabac(pCtx,pNeighAvail,pNonZeroCount,pRefIndex,pDirect,listIdx,iIdx8,pRefCount[listIdx],1,iref));
						if((iref<0) || (iref>=pRefCount[listIdx]) || (pCtx->sRefPic.pRefList[listIdx][iref]==NULL)){		// error ref_idx
							pCtx->bMbRefConcealed=true;
							if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
								iref=0;
								pCtx->iErrorCode|=dsBitstreamError;
							}else{
								return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
							}
						}
						pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(pCtx->sRefPic.pRefList[listIdx][iref] && (pCtx->sRefPic.pRefList[listIdx][iref]->bIsComplete || bIsPending));
					}
					Update8x8RefIdx(pCurDqLayer,iIdx8,listIdx,iref);
					ref_idx_list[listIdx][i]=iref;
				}
			}
		}
		// mv
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<4; i++){
				int16_t iIdx8=i<<2;

				uint32_t subMbType=pCurDqLayer->pSubMbType[iMbXy][i];
				if(IS_DIRECT(subMbType) && !pSliceHeader->iDirectSpatialMvPredFlag)
					continue;

				int8_t iref=ref_idx_list[listIdx][i];
				UpdateP8x8RefCacheIdxCabac(pRefIndex,iIdx8,listIdx,iref);

				if(IS_DIRECT(subMbType))
					continue;

				bool is_dir=IS_DIR(subMbType,0,listIdx)>0;
				int8_t iPartCount=pSubPartCount[i];
				int16_t iBlockW=pPartW[i];
				uint8_t iScan4Idx,iCacheIdx;
				for(int32_t j=0; j<iPartCount; j++){
					iPartIdx=(i<<2)+j*iBlockW;
					iScan4Idx=g_kuiScan4[iPartIdx];
					iCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
					if(is_dir){
						PredMv(pMotionVector,pRefIndex,listIdx,iPartIdx,iBlockW,iref,pMv);
						WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,listIdx,0,pMvd[0]));
						WELS_READ_VERIFY(ParseMvdInfoCabac(pCtx,pNeighAvail,pRefIndex,pMvdCache,iPartIdx,listIdx,1,pMvd[1]));
						pMv[0]+=pMvd[0];
						pMv[1]+=pMvd[1];
						WELS_CHECK_SE_BOTH_WARNING(pMv[1],iMinVmv,iMaxVmv,"vertical mv");
					}else{
						*(uint32_t*)pMv=*(uint32_t*)pMvd=0;
					}
					if(IS_SUB_8x8(subMbType)){		// MB_TYPE_8x8
						ST32((pMv+2),LD32(pMv));
						ST32((pMvd+2),LD32(pMvd));
						ST64(pCurDqLayer->pDec->pMv[listIdx][iMbXy][iScan4Idx],LD64(pMv));
						ST64(pCurDqLayer->pDec->pMv[listIdx][iMbXy][iScan4Idx+4],LD64(pMv));
						ST64(pCurDqLayer->pMvd[listIdx][iMbXy][iScan4Idx],LD64(pMvd));
						ST64(pCurDqLayer->pMvd[listIdx][iMbXy][iScan4Idx+4],LD64(pMvd));
						ST64(pMotionVector[listIdx][iCacheIdx],LD64(pMv));
						ST64(pMotionVector[listIdx][iCacheIdx+6],LD64(pMv));
						ST64(pMvdCache[listIdx][iCacheIdx],LD64(pMvd));
						ST64(pMvdCache[listIdx][iCacheIdx+6],LD64(pMvd));
					}else
					if(IS_SUB_4x4(subMbType)){		// MB_TYPE_4x4
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][iScan4Idx],LD32(pMv));
						ST32(pCurDqLayer->pMvd[listIdx][iMbXy][iScan4Idx],LD32(pMvd));
						ST32(pMotionVector[listIdx][iCacheIdx],LD32(pMv));
						ST32(pMvdCache[listIdx][iCacheIdx],LD32(pMvd));
					}else
					if(IS_SUB_4x8(subMbType)){		// MB_TYPE_4x8 5,7,9
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][iScan4Idx],LD32(pMv));
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][iScan4Idx+4],LD32(pMv));
						ST32(pCurDqLayer->pMvd[listIdx][iMbXy][iScan4Idx],LD32(pMvd));
						ST32(pCurDqLayer->pMvd[listIdx][iMbXy][iScan4Idx+4],LD32(pMvd));
						ST32(pMotionVector[listIdx][iCacheIdx],LD32(pMv));
						ST32(pMotionVector[listIdx][iCacheIdx+6],LD32(pMv));
						ST32(pMvdCache[listIdx][iCacheIdx],LD32(pMvd));
						ST32(pMvdCache[listIdx][iCacheIdx+6],LD32(pMvd));
					}else{		// MB_TYPE_8x4 4,6,8
						ST32((pMv+2),LD32(pMv));
						ST32((pMvd+2),LD32(pMvd));
						ST64(pCurDqLayer->pDec->pMv[listIdx][iMbXy][iScan4Idx],LD64(pMv));
						ST64(pCurDqLayer->pMvd[listIdx][iMbXy][iScan4Idx],LD64(pMvd));
						ST64(pMotionVector[listIdx][iCacheIdx],LD64(pMv));
						ST64(pMvdCache[listIdx][iCacheIdx],LD64(pMvd));
					}
				}
			}
		}
	}
	return ERR_NONE;
}

int32_t WelsDecodeMbCabacBSliceBaseMode0(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint32_t& uiEosFlag){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SBitStringAux* pBsAux=pCurDqLayer->pBitStringAux;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;

	int32_t iScanIdxStart=pSlice->sSliceHeaderExt.uiScanIdxStart;
	int32_t iScanIdxEnd=pSlice->sSliceHeaderExt.uiScanIdxEnd;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int32_t iMbResProperty;
	int32_t i;
	uint32_t uiMbType=0,uiCbp=0,uiCbpLuma=0,uiCbpChroma=0;

	ENFORCE_STACK_ALIGN_1D(uint8_t,pNonZeroCount,48,16);

	pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;

	WELS_READ_VERIFY(ParseMBTypeBSliceCabac(pCtx,pNeighAvail,uiMbType));

	if(uiMbType<23){		// Inter B mode
		int16_t pMotionVector[LIST_A][30][MV_A];
		int16_t pMvdCache[LIST_A][30][MV_A];
		int8_t pRefIndex[LIST_A][30];
		int8_t pDirect[30];
		pCurDqLayer->pDec->pMbType[iMbXy]=g_ksInterBMbTypeInfo[uiMbType].iType;
		WelsFillCacheInterCabac(pNeighAvail,pNonZeroCount,pMotionVector,pMvdCache,pRefIndex,pCurDqLayer);
		WelsFillDirectCacheCabac(pNeighAvail,pDirect,pCurDqLayer);
		WELS_READ_VERIFY(ParseInterBMotionInfoCabac(pCtx,pNeighAvail,pNonZeroCount,pMotionVector,pMvdCache,pRefIndex,pDirect));
		pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
	}else{		// Intra mode
		uiMbType-=23;
		if(uiMbType>25)
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);
		if(!pCtx->pSps->uiChromaFormatIdc && ((uiMbType>=5 && uiMbType<=12) || (uiMbType>=17 && uiMbType<=24)))
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);

		if(25==uiMbType){		// I_PCM
			uprintf("I_PCM mode exists in B slice!");
			WELS_READ_VERIFY(ParseIPCMInfoCabac(pCtx));
			pSlice->iLastDeltaQp=0;
			WELS_READ_VERIFY(ParseEndOfSliceCabac(pCtx,uiEosFlag));
			if(uiEosFlag){
				RestoreCabacDecEngineToBS(pCtx->pCabacDecEngine,pCtx->pCurDqLayer->pBitStringAux);
			}
			return ERR_NONE;
		}else{		// normal Intra mode
			if(0==uiMbType){		// Intra4x4
				ENFORCE_STACK_ALIGN_1D(int8_t,pIntraPredMode,48,16);
				pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA4x4;
				if(pCtx->pPps->bTransform8x8ModeFlag){
					WELS_READ_VERIFY(ParseTransformSize8x8FlagCabac(pCtx,pNeighAvail,pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]));
				}
				if(pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
					uiMbType=pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA8x8;
					pCtx->pFillInfoCacheIntraNxNFunc(pNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
					WELS_READ_VERIFY(ParseIntra8x8Mode(pCtx,pNeighAvail,pIntraPredMode,pBsAux,pCurDqLayer));
				}else{
					pCtx->pFillInfoCacheIntraNxNFunc(pNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
					WELS_READ_VERIFY(ParseIntra4x4Mode(pCtx,pNeighAvail,pIntraPredMode,pBsAux,pCurDqLayer));
				}
			}else{		// Intra16x16
				pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA16x16;
				pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;
				pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
				pCurDqLayer->pIntraPredMode[iMbXy][7]=(uiMbType-1)&3;
				pCurDqLayer->pCbp[iMbXy]=g_kuiI16CbpTable[(uiMbType-1)>>2];
				uiCbpChroma=pCtx->pSps->uiChromaFormatIdc ? pCurDqLayer->pCbp[iMbXy]>>4 : 0;
				uiCbpLuma=pCurDqLayer->pCbp[iMbXy]&15;
				WelsFillCacheNonZeroCount(pNeighAvail,pNonZeroCount,pCurDqLayer);
				WELS_READ_VERIFY(ParseIntra16x16Mode(pCtx,pNeighAvail,pBsAux,pCurDqLayer));
			}
		}
	}

	ST32(&pCurDqLayer->pNzc[iMbXy][0],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][4],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][8],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][12],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][16],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][20],0);

	if(MB_TYPE_INTRA16x16!=pCurDqLayer->pDec->pMbType[iMbXy]){
		WELS_READ_VERIFY(ParseCbpInfoCabac(pCtx,pNeighAvail,uiCbp));

		pCurDqLayer->pCbp[iMbXy]=uiCbp;
		pSlice->iLastDeltaQp=uiCbp==0 ? 0 : pSlice->iLastDeltaQp;
		uiCbpChroma=pCtx->pSps->uiChromaFormatIdc ? pCurDqLayer->pCbp[iMbXy]>>4 : 0;
		uiCbpLuma=pCurDqLayer->pCbp[iMbXy]&15;
	}

	if(pCurDqLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){

		if(MB_TYPE_INTRA16x16!=pCurDqLayer->pDec->pMbType[iMbXy]){
			// Need modification when B picutre add in
			bool bNeedParseTransformSize8x8Flag=
				(((IS_INTER_16x16(pCurDqLayer->pDec->pMbType[iMbXy]) || IS_DIRECT(pCurDqLayer->pDec->pMbType[iMbXy])
					 || IS_INTER_16x8(pCurDqLayer->pDec->pMbType[iMbXy]) || IS_INTER_8x16(pCurDqLayer->pDec->pMbType[iMbXy]))
					 || pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy])
				  && (pCurDqLayer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA8x8)
				  && (pCurDqLayer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA4x4)
				  && ((pCurDqLayer->pCbp[iMbXy]&0x0F)>0)
				  && (pCtx->pPps->bTransform8x8ModeFlag));

			if(bNeedParseTransformSize8x8Flag){
				WELS_READ_VERIFY(ParseTransformSize8x8FlagCabac(pCtx,pNeighAvail,pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]));		// transform_size_8x8_flag
			}
		}

		memset(pCurDqLayer->pScaledTCoeff[iMbXy],0,384*sizeof(pCurDqLayer->pScaledTCoeff[iMbXy][0]));

		int32_t iQpDelta,iId8x8,iId4x4;

		WELS_READ_VERIFY(ParseDeltaQpCabac(pCtx,iQpDelta));
		if(iQpDelta>25 || iQpDelta<-26){		// out of iQpDelta range
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_QP);
		}
		pCurDqLayer->pLumaQp[iMbXy]=(pSlice->iLastMbQp+iQpDelta+52)%52;		// update last_mb_qp
		pSlice->iLastMbQp=pCurDqLayer->pLumaQp[iMbXy];
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pSlice->iLastMbQp+
											 pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}

		if(MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){
			// step1: Luma DC
			WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,0,16,g_kuiLumaDcZigzagScan,I16_LUMA_DC,pCurDqLayer->pScaledTCoeff[iMbXy],pCurDqLayer->pLumaQp[iMbXy],pCtx));
			// step2: Luma AC
			if(uiCbpLuma){
				for(i=0; i<16; i++){
					WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,i,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),I16_LUMA_AC,pCurDqLayer->pScaledTCoeff[iMbXy]+(i<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx));
				}
				ST32(&pCurDqLayer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&pCurDqLayer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&pCurDqLayer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&pCurDqLayer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{
				ST32(&pCurDqLayer->pNzc[iMbXy][0],0);
				ST32(&pCurDqLayer->pNzc[iMbXy][4],0);
				ST32(&pCurDqLayer->pNzc[iMbXy][8],0);
				ST32(&pCurDqLayer->pNzc[iMbXy][12],0);
			}
		}else{		// non-MB_TYPE_INTRA16x16
			if(pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
				// Transform 8x8 support for CABAC
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						WELS_READ_VERIFY(ParseResidualBlockCabac8x8(pNeighAvail,pNonZeroCount,pBsAux,(iId8x8<<2),iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan8x8+iScanIdxStart,IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]) ? LUMA_DC_AC_INTRA_8 : LUMA_DC_AC_INTER_8,pCurDqLayer->pScaledTCoeff[iMbXy]+(iId8x8<<6),pCurDqLayer->pLumaQp[iMbXy],pCtx));
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&pCurDqLayer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&pCurDqLayer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&pCurDqLayer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&pCurDqLayer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{
				iMbResProperty=(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						int32_t iIdx=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							// Luma (DC and AC decoding together)
							WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,iIdx,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan+iScanIdxStart,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIdx<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx));
							iIdx++;
						}
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[iId8x8<<2]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&pCurDqLayer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&pCurDqLayer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&pCurDqLayer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&pCurDqLayer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}
		}

		// chroma
		// step1: DC
		if(1==uiCbpChroma || 2==uiCbpChroma){
			for(i=0; i<2; i++){
				if(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_DC_V : CHROMA_DC_U;
				else
					iMbResProperty=i ? CHROMA_DC_V_INTER : CHROMA_DC_U_INTER;

				WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,16+(i<<2),4,g_kuiChromaDcScan,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+256+(i<<6),pCurDqLayer->pChromaQp[iMbXy][i],pCtx));
			}
		}
		// step2: AC
		if(2==uiCbpChroma){
			for(i=0; i<2; i++){
				if(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_AC_V : CHROMA_AC_U;
				else
					iMbResProperty=i ? CHROMA_AC_V_INTER : CHROMA_AC_U_INTER;
				int32_t index=16+(i<<2);
				for(iId4x4=0; iId4x4<4; iId4x4++){
					WELS_READ_VERIFY(ParseResidualBlockCabac(pNeighAvail,pNonZeroCount,pBsAux,index,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(index<<4),pCurDqLayer->pChromaQp[iMbXy][i],pCtx));
					index++;
				}
			}
			ST16(&pCurDqLayer->pNzc[iMbXy][16],LD16(&pNonZeroCount[6+8*1]));
			ST16(&pCurDqLayer->pNzc[iMbXy][20],LD16(&pNonZeroCount[6+8*2]));
			ST16(&pCurDqLayer->pNzc[iMbXy][18],LD16(&pNonZeroCount[6+8*4]));
			ST16(&pCurDqLayer->pNzc[iMbXy][22],LD16(&pNonZeroCount[6+8*5]));
		}else{
			ST32(&pCurDqLayer->pNzc[iMbXy][16],0);
			ST32(&pCurDqLayer->pNzc[iMbXy][20],0);
		}
	}else{
		pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pCurDqLayer->pLumaQp[iMbXy]+
											 pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}
	}

	WELS_READ_VERIFY(ParseEndOfSliceCabac(pCtx,uiEosFlag));
	if(uiEosFlag){
		RestoreCabacDecEngineToBS(pCtx->pCabacDecEngine,pCtx->pCurDqLayer->pBitStringAux);
	}

	return ERR_NONE;
}

int32_t WelsDecodeMbCabacBSlice(SDecoderContext* pCtx,SNalUnit* pNalCur,uint32_t& uiEosFlag){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	SPicture** ppRefPicL0=pCtx->sRefPic.pRefList[LIST_0];
	SPicture** ppRefPicL1=pCtx->sRefPic.pRefList[LIST_1];
	uint32_t uiCode;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int32_t i;
	SWelsNeighAvail uiNeighAvail;
	pCurDqLayer->pCbp[iMbXy]=0;
	pCurDqLayer->pCbfDc[iMbXy]=0;
	pCurDqLayer->pChromaPredMode[iMbXy]=C_PRED_DC;

	pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
	pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;

	GetNeighborAvailMbType(&uiNeighAvail,pCurDqLayer);
	WELS_READ_VERIFY(ParseSkipFlagCabac(pCtx,&uiNeighAvail,uiCode));

	memset(pCurDqLayer->pDirect[iMbXy],0,sizeof(int8_t)*16);

	bool bIsPending=false;	// GetThreadCount (pCtx) > 1;

	if(uiCode){
		int16_t pMv[LIST_A][2]={{0,0},{0,0}};
		int8_t ref[LIST_A]={0};
		pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_SKIP|MB_TYPE_DIRECT;
		ST32(&pCurDqLayer->pNzc[iMbXy][0],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][4],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][8],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][12],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][16],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][20],0);

		pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
		memset(pCurDqLayer->pDec->pRefIndex[LIST_0][iMbXy],0,sizeof(int8_t)*16);
		memset(pCurDqLayer->pDec->pRefIndex[LIST_1][iMbXy],0,sizeof(int8_t)*16);
		pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPicL0[0] && (ppRefPicL0[0]->bIsComplete
			 || bIsPending)) || !(ppRefPicL1[0] && (ppRefPicL1[0]->bIsComplete || bIsPending));

		if(pCtx->bMbRefConcealed){
			uprintf("Ref Picture for B-Slice is lost,B-Slice decoding cannot be continued!");
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_DATA,ERR_INFO_REFERENCE_PIC_LOST);
		}

		SubMbType subMbType;
		if(pSliceHeader->iDirectSpatialMvPredFlag){

			// predict direct spatial mv
			int32_t ret=PredMvBDirectSpatial(pCtx,pMv,ref,subMbType);
			if(ret!=ERR_NONE){
				return ret;
			}
		}else{
			// temporal direct mode
			int32_t ret=PredBDirectTemporal(pCtx,pMv,ref,subMbType);
			if(ret!=ERR_NONE){
				return ret;
			}
		}


		// reset rS
		pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;		// ??????????????? dqaunt of previous mb
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pCurDqLayer->pLumaQp[iMbXy]+
											 pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}

		// for neighboring CABAC usage
		pSlice->iLastDeltaQp=0;

		WELS_READ_VERIFY(ParseEndOfSliceCabac(pCtx,uiEosFlag));

		return ERR_NONE;
	}

	WELS_READ_VERIFY(WelsDecodeMbCabacBSliceBaseMode0(pCtx,&uiNeighAvail,uiEosFlag));
	return ERR_NONE;
}

typedef int32_t(*PWelsDecMbFunc) (SDecoderContext* pCtx,SNalUnit* pNalCur,uint32_t& uiEosFlag);

int32_t ParseMBTypeISliceCabac(SDecoderContext* pCtx,PWelsNeighAvail pNeighAvail,uint32_t& uiBinVal){
	uint32_t uiCode;
	int32_t iIdxA=0,iIdxB=0;
	int32_t iCtxInc;
	uiBinVal=0;
	SWelsCabacDecEngine* pCabacDecEngine=pCtx->pCabacDecEngine;
	SWelsCabacCtx* pBinCtx=pCtx->pCabacCtx+NEW_CTX_OFFSET_MB_TYPE_I;		// I mode in I slice
	iIdxA=(pNeighAvail->iLeftAvail) && (pNeighAvail->iLeftType!=MB_TYPE_INTRA4x4
										  && pNeighAvail->iLeftType!=MB_TYPE_INTRA8x8);
	iIdxB=(pNeighAvail->iTopAvail) && (pNeighAvail->iTopType!=MB_TYPE_INTRA4x4
										  && pNeighAvail->iTopType!=MB_TYPE_INTRA8x8);
	iCtxInc=iIdxA+iIdxB;
	WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+iCtxInc,uiCode));
	uiBinVal=uiCode;
	if(uiBinVal!=0){		// I16x16
		WELS_READ_VERIFY(DecodeTerminateCabac(pCabacDecEngine,uiCode));
		if(uiCode==1)
			uiBinVal=25;		// I_PCM
		else{
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+3,uiCode));
			uiBinVal=1+uiCode*12;
			// decoding of uiCbp:0,1,2
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+4,uiCode));
			if(uiCode!=0){
				WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+5,uiCode));
				uiBinVal+=4;
				if(uiCode!=0)
					uiBinVal+=4;
			}
			// decoding of I pred-mode: 0,1,2,3
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+6,uiCode));
			uiBinVal+=(uiCode<<1);
			WELS_READ_VERIFY(DecodeBinCabac(pCabacDecEngine,pBinCtx+7,uiCode));
			uiBinVal+=uiCode;
		}
	}
	// I4x4
	return ERR_NONE;
}

int32_t WelsDecodeMbCabacISliceBaseMode0(SDecoderContext* pCtx,uint32_t& uiEosFlag){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SBitStringAux* pBsAux=pCurDqLayer->pBitStringAux;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	SWelsNeighAvail sNeighAvail;
	int32_t iScanIdxStart=pSlice->sSliceHeaderExt.uiScanIdxStart;
	int32_t iScanIdxEnd=pSlice->sSliceHeaderExt.uiScanIdxEnd;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int32_t i;
	uint32_t uiMbType=0,uiCbp=0,uiCbpLuma=0,uiCbpChroma=0;

	ENFORCE_STACK_ALIGN_1D(uint8_t,pNonZeroCount,48,16);

	pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
	pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;

	pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
	pCurDqLayer->pResidualPredFlag[iMbXy]=pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;
	GetNeighborAvailMbType(&sNeighAvail,pCurDqLayer);
	WELS_READ_VERIFY(ParseMBTypeISliceCabac(pCtx,&sNeighAvail,uiMbType));
	if(uiMbType>25){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);
	}else
	if(!pCtx->pSps->uiChromaFormatIdc && ((uiMbType>=5 && uiMbType<=12) || (uiMbType>=17
		 && uiMbType<=24))){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);
	}else
	if(25==uiMbType){		// I_PCM
		uprintf("I_PCM mode exists in I slice!");
		WELS_READ_VERIFY(ParseIPCMInfoCabac(pCtx));
		pSlice->iLastDeltaQp=0;
		WELS_READ_VERIFY(ParseEndOfSliceCabac(pCtx,uiEosFlag));
		if(uiEosFlag){
			RestoreCabacDecEngineToBS(pCtx->pCabacDecEngine,pCtx->pCurDqLayer->pBitStringAux);
		}
		return ERR_NONE;
	}else
	if(0==uiMbType){		// I4x4
		ENFORCE_STACK_ALIGN_1D(int8_t,pIntraPredMode,48,16);
		pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA4x4;
		if(pCtx->pPps->bTransform8x8ModeFlag){
			// Transform 8x8 cabac will be added soon
			WELS_READ_VERIFY(ParseTransformSize8x8FlagCabac(pCtx,&sNeighAvail,pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]));
		}
		if(pCtx->pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
			uiMbType=pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA8x8;
			pCtx->pFillInfoCacheIntraNxNFunc(&sNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
			WELS_READ_VERIFY(ParseIntra8x8Mode(pCtx,&sNeighAvail,pIntraPredMode,pBsAux,pCurDqLayer));
		}else{
			pCtx->pFillInfoCacheIntraNxNFunc(&sNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
			WELS_READ_VERIFY(ParseIntra4x4Mode(pCtx,&sNeighAvail,pIntraPredMode,pBsAux,pCurDqLayer));
		}
		// get uiCbp for I4x4
		WELS_READ_VERIFY(ParseCbpInfoCabac(pCtx,&sNeighAvail,uiCbp));
		pCurDqLayer->pCbp[iMbXy]=uiCbp;
		pSlice->iLastDeltaQp=uiCbp==0 ? 0 : pSlice->iLastDeltaQp;
		uiCbpChroma=pCtx->pSps->uiChromaFormatIdc ? uiCbp>>4 : 0;
		uiCbpLuma=uiCbp&15;
	}else{		// I16x16;
		pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA16x16;
		pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;
		pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
		pCurDqLayer->pIntraPredMode[iMbXy][7]=(uiMbType-1)&3;
		pCurDqLayer->pCbp[iMbXy]=g_kuiI16CbpTable[(uiMbType-1)>>2];
		uiCbpChroma=pCtx->pSps->uiChromaFormatIdc ? pCurDqLayer->pCbp[iMbXy]>>4 : 0;
		uiCbpLuma=pCurDqLayer->pCbp[iMbXy]&15;
		WelsFillCacheNonZeroCount(&sNeighAvail,pNonZeroCount,pCurDqLayer);
		WELS_READ_VERIFY(ParseIntra16x16Mode(pCtx,&sNeighAvail,pBsAux,pCurDqLayer));
	}

	ST32(&pCurDqLayer->pNzc[iMbXy][0],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][4],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][8],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][12],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][16],0);
	ST32(&pCurDqLayer->pNzc[iMbXy][20],0);
	pCurDqLayer->pCbfDc[iMbXy]=0;

	if(pCurDqLayer->pCbp[iMbXy]==0 && IS_INTRANxN(pCurDqLayer->pDec->pMbType[iMbXy])){
		pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3((pCurDqLayer->pLumaQp[iMbXy]+
				pSliceHeader->pPps->iChromaQpIndexOffset[i]),0,51)];
		}
	}

	if(pCurDqLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){
		memset(pCurDqLayer->pScaledTCoeff[iMbXy],0,384*sizeof(pCurDqLayer->pScaledTCoeff[iMbXy][0]));
		int32_t iQpDelta,iId8x8,iId4x4;
		WELS_READ_VERIFY(ParseDeltaQpCabac(pCtx,iQpDelta));
		if(iQpDelta>25 || iQpDelta<-26){// out of iQpDelta range
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_QP);
		}
		pCurDqLayer->pLumaQp[iMbXy]=(pSlice->iLastMbQp+iQpDelta+52)%52;		// update last_mb_qp
		pSlice->iLastMbQp=pCurDqLayer->pLumaQp[iMbXy];
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3((pSlice->iLastMbQp+
				pSliceHeader->pPps->iChromaQpIndexOffset[i]),0,51)];
		}
		if(MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){
			// step1: Luma DC
			WELS_READ_VERIFY(ParseResidualBlockCabac(&sNeighAvail,pNonZeroCount,pBsAux,0,16,g_kuiLumaDcZigzagScan,I16_LUMA_DC,pCurDqLayer->pScaledTCoeff[iMbXy],pCurDqLayer->pLumaQp[iMbXy],pCtx));
			// step2: Luma AC
			if(uiCbpLuma){
				for(i=0; i<16; i++){
					WELS_READ_VERIFY(ParseResidualBlockCabac(&sNeighAvail,pNonZeroCount,pBsAux,i,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),I16_LUMA_AC,pCurDqLayer->pScaledTCoeff[iMbXy]+(i<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx));
				}
				ST32(&pCurDqLayer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&pCurDqLayer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&pCurDqLayer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&pCurDqLayer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{		// pNonZeroCount=0
				ST32(&pCurDqLayer->pNzc[iMbXy][0],0);
				ST32(&pCurDqLayer->pNzc[iMbXy][4],0);
				ST32(&pCurDqLayer->pNzc[iMbXy][8],0);
				ST32(&pCurDqLayer->pNzc[iMbXy][12],0);
			}
		}else{		// non-MB_TYPE_INTRA16x16
			if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
				// Transform 8x8 support for CABAC
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						WELS_READ_VERIFY(ParseResidualBlockCabac8x8(&sNeighAvail,pNonZeroCount,pBsAux,(iId8x8<<2),iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan8x8+iScanIdxStart,LUMA_DC_AC_INTRA_8,pCurDqLayer->pScaledTCoeff[iMbXy]+(iId8x8<<6),pCurDqLayer->pLumaQp[iMbXy],pCtx));
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&pCurDqLayer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&pCurDqLayer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&pCurDqLayer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&pCurDqLayer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}else{
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpLuma&(1<<iId8x8)){
						int32_t iIdx=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							// Luma (DC and AC decoding together)
							WELS_READ_VERIFY(ParseResidualBlockCabac(&sNeighAvail,pNonZeroCount,pBsAux,iIdx,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan+iScanIdxStart,LUMA_DC_AC_INTRA,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIdx<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx));
							iIdx++;
						}
					}else{
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)]],0);
						ST16(&pNonZeroCount[g_kCacheNzcScanIdx[(iId8x8<<2)+2]],0);
					}
				}
				ST32(&pCurDqLayer->pNzc[iMbXy][0],LD32(&pNonZeroCount[1+8*1]));
				ST32(&pCurDqLayer->pNzc[iMbXy][4],LD32(&pNonZeroCount[1+8*2]));
				ST32(&pCurDqLayer->pNzc[iMbXy][8],LD32(&pNonZeroCount[1+8*3]));
				ST32(&pCurDqLayer->pNzc[iMbXy][12],LD32(&pNonZeroCount[1+8*4]));
			}
		}
		int32_t iMbResProperty;
		// chroma
		// step1: DC
		if(1==uiCbpChroma || 2==uiCbpChroma){
			// Cb Cr
			for(i=0; i<2; i++){
				iMbResProperty=i ? CHROMA_DC_V : CHROMA_DC_U;
				WELS_READ_VERIFY(ParseResidualBlockCabac(&sNeighAvail,pNonZeroCount,pBsAux,16+(i<<2),4,g_kuiChromaDcScan,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+256+(i<<6),pCurDqLayer->pChromaQp[iMbXy][i],pCtx));
			}
		}

		// step2: AC
		if(2==uiCbpChroma){
			for(i=0; i<2; i++){		// Cb Cr
				iMbResProperty=i ? CHROMA_AC_V : CHROMA_AC_U;
				int32_t iIdx=16+(i<<2);
				for(iId4x4=0; iId4x4<4; iId4x4++){
					WELS_READ_VERIFY(ParseResidualBlockCabac(&sNeighAvail,pNonZeroCount,pBsAux,iIdx,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIdx<<4),pCurDqLayer->pChromaQp[iMbXy][i],pCtx));
					iIdx++;
				}
			}
			ST16(&pCurDqLayer->pNzc[iMbXy][16],LD16(&pNonZeroCount[6+8*1]));
			ST16(&pCurDqLayer->pNzc[iMbXy][20],LD16(&pNonZeroCount[6+8*2]));
			ST16(&pCurDqLayer->pNzc[iMbXy][18],LD16(&pNonZeroCount[6+8*4]));
			ST16(&pCurDqLayer->pNzc[iMbXy][22],LD16(&pNonZeroCount[6+8*5]));
		}else{
			ST16(&pCurDqLayer->pNzc[iMbXy][16],0);
			ST16(&pCurDqLayer->pNzc[iMbXy][20],0);
			ST16(&pCurDqLayer->pNzc[iMbXy][18],0);
			ST16(&pCurDqLayer->pNzc[iMbXy][22],0);
		}
	}else{
		ST32(&pCurDqLayer->pNzc[iMbXy][0],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][4],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][8],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][12],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][16],0);
		ST32(&pCurDqLayer->pNzc[iMbXy][20],0);
	}

	WELS_READ_VERIFY(ParseEndOfSliceCabac(pCtx,uiEosFlag));
	if(uiEosFlag){
		RestoreCabacDecEngineToBS(pCtx->pCabacDecEngine,pCtx->pCurDqLayer->pBitStringAux);
	}
	return ERR_NONE;
}

int32_t WelsDecodeMbCabacISlice(SDecoderContext* pCtx,SNalUnit* pNalCur,uint32_t& uiEosFlag){
	WELS_READ_VERIFY(WelsDecodeMbCabacISliceBaseMode0(pCtx,uiEosFlag));
	return ERR_NONE;
}

void WelsFillCacheInter(PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCount,int16_t iMvArray[LIST_A][30][MV_A],int8_t iRefIdxArray[LIST_A][30],PDqLayer pCurDqLayer){
	int32_t iCurXy=pCurDqLayer->iMbXyIndex;
	int32_t iTopXy=0;
	int32_t iLeftXy=0;
	int32_t iLeftTopXy=0;
	int32_t iRightTopXy=0;

	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	int32_t listCount=1;
	if(pSliceHeader->eSliceType==B_SLICE){
		listCount=2;
	}

	// stuff non_zero_coeff_count from pNeighAvail(left and top)
	WelsFillCacheNonZeroCount(pNeighAvail,pNonZeroCount,pCurDqLayer);

	if(pNeighAvail->iTopAvail){
		iTopXy=iCurXy-pCurDqLayer->iMbWidth;
	}
	if(pNeighAvail->iLeftAvail){
		iLeftXy=iCurXy-1;
	}
	if(pNeighAvail->iLeftTopAvail){
		iLeftTopXy=iCurXy-1-pCurDqLayer->iMbWidth;
	}
	if(pNeighAvail->iRightTopAvail){
		iRightTopXy=iCurXy+1-pCurDqLayer->iMbWidth;
	}

	for(int32_t listIdx=0; listIdx<listCount;++listIdx){
		// stuff mv_cache and iRefIdxArray from left and top (inter)
		if(pNeighAvail->iLeftAvail && IS_INTER(pNeighAvail->iLeftType)){
			ST32(iMvArray[listIdx][6],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftXy][3]));
			ST32(iMvArray[listIdx][12],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftXy][7]));
			ST32(iMvArray[listIdx][18],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftXy][11]));
			ST32(iMvArray[listIdx][24],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftXy][15]));
			iRefIdxArray[listIdx][6]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftXy][3];
			iRefIdxArray[listIdx][12]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftXy][7];
			iRefIdxArray[listIdx][18]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftXy][11];
			iRefIdxArray[listIdx][24]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftXy][15];
		}else{
			ST32(iMvArray[listIdx][6],0);
			ST32(iMvArray[listIdx][12],0);
			ST32(iMvArray[listIdx][18],0);
			ST32(iMvArray[listIdx][24],0);

			if(0==pNeighAvail->iLeftAvail){		// not available
				iRefIdxArray[listIdx][6]=
					iRefIdxArray[listIdx][12]=
					iRefIdxArray[listIdx][18]=
					iRefIdxArray[listIdx][24]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][6]=
					iRefIdxArray[listIdx][12]=
					iRefIdxArray[listIdx][18]=
					iRefIdxArray[listIdx][24]=REF_NOT_IN_LIST;
			}
		}
		if(pNeighAvail->iLeftTopAvail && IS_INTER(pNeighAvail->iLeftTopType)){
			ST32(iMvArray[listIdx][0],LD32(pCurDqLayer->pDec->pMv[listIdx][iLeftTopXy][15]));
			iRefIdxArray[listIdx][0]=pCurDqLayer->pDec->pRefIndex[listIdx][iLeftTopXy][15];
		}else{
			ST32(iMvArray[listIdx][0],0);
			if(0==pNeighAvail->iLeftTopAvail){		// not available
				iRefIdxArray[listIdx][0]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][0]=REF_NOT_IN_LIST;
			}
		}
		if(pNeighAvail->iTopAvail && IS_INTER(pNeighAvail->iTopType)){
			ST64(iMvArray[listIdx][1],LD64(pCurDqLayer->pDec->pMv[listIdx][iTopXy][12]));
			ST64(iMvArray[listIdx][3],LD64(pCurDqLayer->pDec->pMv[listIdx][iTopXy][14]));
			ST32(&iRefIdxArray[listIdx][1],LD32(&pCurDqLayer->pDec->pRefIndex[listIdx][iTopXy][12]));
		}else{
			ST64(iMvArray[listIdx][1],0);
			ST64(iMvArray[listIdx][3],0);
			if(0==pNeighAvail->iTopAvail){		// not available
				iRefIdxArray[listIdx][1]=
					iRefIdxArray[listIdx][2]=
					iRefIdxArray[listIdx][3]=
					iRefIdxArray[listIdx][4]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][1]=
					iRefIdxArray[listIdx][2]=
					iRefIdxArray[listIdx][3]=
					iRefIdxArray[listIdx][4]=REF_NOT_IN_LIST;
			}
		}
		if(pNeighAvail->iRightTopAvail && IS_INTER(pNeighAvail->iRightTopType)){
			ST32(iMvArray[listIdx][5],LD32(pCurDqLayer->pDec->pMv[listIdx][iRightTopXy][12]));
			iRefIdxArray[listIdx][5]=pCurDqLayer->pDec->pRefIndex[listIdx][iRightTopXy][12];
		}else{
			ST32(iMvArray[listIdx][5],0);
			if(0==pNeighAvail->iRightTopAvail){		// not available
				iRefIdxArray[listIdx][5]=REF_NOT_AVAIL;
			}else{		// available but is intra mb type
				iRefIdxArray[listIdx][5]=REF_NOT_IN_LIST;
			}
		}
		// right-top 4*4 block unavailable
		ST32(iMvArray[listIdx][9],0);
		ST32(iMvArray[listIdx][21],0);
		ST32(iMvArray[listIdx][11],0);
		ST32(iMvArray[listIdx][17],0);
		ST32(iMvArray[listIdx][23],0);
		iRefIdxArray[listIdx][9]=
			iRefIdxArray[listIdx][21]=
			iRefIdxArray[listIdx][11]=
			iRefIdxArray[listIdx][17]=
			iRefIdxArray[listIdx][23]=REF_NOT_AVAIL;
	}
}

int32_t ParseInterInfo(SDecoderContext* pCtx,int16_t iMvArray[LIST_A][30][MV_A],int8_t iRefIdxArray[LIST_A][30],SBitStringAux* pBs){
	SSlice* pSlice=&pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	SPicture** ppRefPic=pCtx->sRefPic.pRefList[LIST_0];
	int32_t iRefCount[2];
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	int32_t i,j;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int32_t iMotionPredFlag[4];
	int16_t iMv[2];
	uint32_t uiCode;
	int32_t iCode;
	int16_t iMinVmv=pSliceHeader->pSps->pSLevelLimits->iMinVmv;
	int16_t iMaxVmv=pSliceHeader->pSps->pSLevelLimits->iMaxVmv;
	iMotionPredFlag[0]=iMotionPredFlag[1]=iMotionPredFlag[2]=iMotionPredFlag[3]=
		pSlice->sSliceHeaderExt.bDefaultMotionPredFlag;
	iRefCount[0]=pSliceHeader->uiRefCount[0];
	iRefCount[1]=pSliceHeader->uiRefCount[1];

	bool bIsPending=false;	// GetThreadCount (pCtx) > 1;

	switch(pCurDqLayer->pDec->pMbType[iMbXy]){
		case MB_TYPE_16x16:
		{
			int32_t iRefIdx=0;
			if(pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag){
				WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// motion_prediction_flag_l0[ mbPartIdx ]
				iMotionPredFlag[0]=uiCode;
			}
			if(iMotionPredFlag[0]==0){
				WELS_READ_VERIFY(BsGetTe0(pBs,iRefCount[0],&uiCode));		// motion_prediction_flag_l1[ mbPartIdx ]
				iRefIdx=uiCode;
				// Security check: iRefIdx should be in range 0 to num_ref_idx_l0_active_minus1,includsive
				// ref to standard section 7.4.5.1. iRefCount[0] is 1+num_ref_idx_l0_active_minus1.
				if((iRefIdx<0) || (iRefIdx>=iRefCount[0]) || (ppRefPic[iRefIdx]==NULL)){		// error ref_idx
					pCtx->bMbRefConcealed=true;
					if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
						iRefIdx=0;
						pCtx->iErrorCode|=dsBitstreamError;
					}else{
						return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
					}
				}
				pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[iRefIdx]
										 && (ppRefPic[iRefIdx]->bIsComplete || bIsPending));
			}else{
				uprintf("inter parse: iMotionPredFlag=1 not supported. ");
				return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
			}
			PredMv(iMvArray,iRefIdxArray,LIST_0,0,4,iRefIdx,iMv);

			WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l0[ mbPartIdx ][ 0 ][ compIdx ]
			iMv[0]+=iCode;
			WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l1[ mbPartIdx ][ 0 ][ compIdx ]
			iMv[1]+=iCode;
			WELS_CHECK_SE_BOTH_WARNING(iMv[1],iMinVmv,iMaxVmv,"vertical mv");
			UpdateP16x16MotionInfo(pCurDqLayer,LIST_0,iRefIdx,iMv);
		}
		break;
		case MB_TYPE_16x8:
		{
			int32_t iRefIdx[2];
			for(i=0; i<2; i++){
				if(pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag){
					WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// motion_prediction_flag_l0[ mbPartIdx ]
					iMotionPredFlag[i]=uiCode;
				}
			}

			for(i=0; i<2; i++){
				if(iMotionPredFlag[i]){
					uprintf("inter parse: iMotionPredFlag=1 not supported. ");
					return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
				}
				WELS_READ_VERIFY(BsGetTe0(pBs,iRefCount[0],&uiCode));		// ref_idx_l0[ mbPartIdx ]
				iRefIdx[i]=uiCode;
				if((iRefIdx[i]<0) || (iRefIdx[i]>=iRefCount[0]) || (ppRefPic[iRefIdx[i]]==NULL)){		// error ref_idx
					pCtx->bMbRefConcealed=true;
					if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
						iRefIdx[i]=0;
						pCtx->iErrorCode|=dsBitstreamError;
					}else{
						return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
					}
				}
				pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[iRefIdx[i]]
										 && (ppRefPic[iRefIdx[i]]->bIsComplete || bIsPending));
			}
			for(i=0; i<2; i++){
				PredInter16x8Mv(iMvArray,iRefIdxArray,LIST_0,i<<3,iRefIdx[i],iMv);

				WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l0[ mbPartIdx ][ 0 ][ compIdx ]
				iMv[0]+=iCode;
				WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l1[ mbPartIdx ][ 0 ][ compIdx ]
				iMv[1]+=iCode;
				WELS_CHECK_SE_BOTH_WARNING(iMv[1],iMinVmv,iMaxVmv,"vertical mv");
				UpdateP16x8MotionInfo(pCurDqLayer,iMvArray,iRefIdxArray,LIST_0,i<<3,iRefIdx[i],iMv);
			}
		}
		break;
		case MB_TYPE_8x16:
		{
			int32_t iRefIdx[2];
			for(i=0; i<2; i++){
				if(pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag){
					WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// motion_prediction_flag_l0[ mbPartIdx ]
					iMotionPredFlag[i]=uiCode;
				}
			}

			for(i=0; i<2; i++){
				if(iMotionPredFlag[i]==0){
					WELS_READ_VERIFY(BsGetTe0(pBs,iRefCount[0],&uiCode));		// ref_idx_l0[ mbPartIdx ]
					iRefIdx[i]=uiCode;
					if((iRefIdx[i]<0) || (iRefIdx[i]>=iRefCount[0]) || (ppRefPic[iRefIdx[i]]==NULL)){		// error ref_idx
						pCtx->bMbRefConcealed=true;
						if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
							iRefIdx[i]=0;
							pCtx->iErrorCode|=dsBitstreamError;
						}else{
							return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
						}
					}
					pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[iRefIdx[i]]
											 && (ppRefPic[iRefIdx[i]]->bIsComplete || bIsPending));
				}else{
					uprintf("inter parse: iMotionPredFlag=1 not supported. ");
					return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
				}

			}
			for(i=0; i<2; i++){
				PredInter8x16Mv(iMvArray,iRefIdxArray,LIST_0,i<<2,iRefIdx[i],iMv);

				WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l0[ mbPartIdx ][ 0 ][ compIdx ]
				iMv[0]+=iCode;
				WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l1[ mbPartIdx ][ 0 ][ compIdx ]
				iMv[1]+=iCode;
				WELS_CHECK_SE_BOTH_WARNING(iMv[1],iMinVmv,iMaxVmv,"vertical mv");
				UpdateP8x16MotionInfo(pCurDqLayer,iMvArray,iRefIdxArray,LIST_0,i<<2,iRefIdx[i],iMv);
			}
		}
		break;
		case MB_TYPE_8x8:
		case MB_TYPE_8x8_REF0:
		{
			int32_t iRefIdx[4]={0},iSubPartCount[4],iPartWidth[4];
			uint32_t uiSubMbType;

			if(MB_TYPE_8x8_REF0==pCurDqLayer->pDec->pMbType[iMbXy]){
				iRefCount[0]=
					iRefCount[1]=1;
			}

			// uiSubMbType,partition
			for(i=0; i<4; i++){
				WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// sub_mb_type[ mbPartIdx ]
				uiSubMbType=uiCode;
				if(uiSubMbType>=4){		// invalid uiSubMbType
					return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_SUB_MB_TYPE);
				}
				pCurDqLayer->pSubMbType[iMbXy][i]=g_ksInterPSubMbTypeInfo[uiSubMbType].iType;
				iSubPartCount[i]=g_ksInterPSubMbTypeInfo[uiSubMbType].iPartCount;
				iPartWidth[i]=g_ksInterPSubMbTypeInfo[uiSubMbType].iPartWidth;

				// Need modification when B picture add in,reference to 7.3.5
				pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]&=(uiSubMbType==0);
			}

			if(pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag){
				for(i=0; i<4; i++){
					WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// motion_prediction_flag_l0[ mbPartIdx ]
					iMotionPredFlag[i]=uiCode;
				}
			}

			// iRefIdxArray
			if(MB_TYPE_8x8_REF0==pCurDqLayer->pDec->pMbType[iMbXy]){
				memset(pCurDqLayer->pDec->pRefIndex[0][iMbXy],0,16);
			}else{
				for(i=0; i<4; i++){
					int16_t iIndex8=i<<2;
					uint8_t uiScan4Idx=g_kuiScan4[iIndex8];

					if(iMotionPredFlag[i]==0){
						WELS_READ_VERIFY(BsGetTe0(pBs,iRefCount[0],&uiCode));		// ref_idx_l0[ mbPartIdx ]
						iRefIdx[i]=uiCode;
						if((iRefIdx[i]<0) || (iRefIdx[i]>=iRefCount[0]) || (ppRefPic[iRefIdx[i]]==NULL)){		// error ref_idx
							pCtx->bMbRefConcealed=true;
							if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
								iRefIdx[i]=0;
								pCtx->iErrorCode|=dsBitstreamError;
							}else{
								return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
							}
						}
						pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[iRefIdx[i]]
												 && (ppRefPic[iRefIdx[i]]->bIsComplete || bIsPending));

						pCurDqLayer->pDec->pRefIndex[0][iMbXy][uiScan4Idx]=pCurDqLayer->pDec->pRefIndex[0][iMbXy][uiScan4Idx+1]=
							pCurDqLayer->pDec->pRefIndex[0][iMbXy][uiScan4Idx+4]=pCurDqLayer->pDec->pRefIndex[0][iMbXy][uiScan4Idx+5]=
							iRefIdx[i];
					}else{
						uprintf("inter parse: iMotionPredFlag=1 not supported. ");
						return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
					}
				}
			}

			// gain mv and update mv cache
			for(i=0; i<4; i++){
				int8_t iPartCount=iSubPartCount[i];
				uint32_t uiSubMbType=pCurDqLayer->pSubMbType[iMbXy][i];
				int16_t iMv[2],iPartIdx,iBlockWidth=iPartWidth[i],iIdx=i<<2;
				uint8_t uiScan4Idx,uiCacheIdx;

				uint8_t uiIdx4Cache=g_kuiCache30ScanIdx[iIdx];

				iRefIdxArray[0][uiIdx4Cache]=iRefIdxArray[0][uiIdx4Cache+1]=
					iRefIdxArray[0][uiIdx4Cache+6]=iRefIdxArray[0][uiIdx4Cache+7]=iRefIdx[i];

				for(j=0; j<iPartCount; j++){
					iPartIdx=iIdx+j*iBlockWidth;
					uiScan4Idx=g_kuiScan4[iPartIdx];
					uiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
					PredMv(iMvArray,iRefIdxArray,LIST_0,iPartIdx,iBlockWidth,iRefIdx[i],iMv);

					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l0[ mbPartIdx ][ subMbPartIdx ][ compIdx ]
					iMv[0]+=iCode;
					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l1[ mbPartIdx ][ subMbPartIdx ][ compIdx ]
					iMv[1]+=iCode;
					WELS_CHECK_SE_BOTH_WARNING(iMv[1],iMinVmv,iMaxVmv,"vertical mv");
					if(SUB_MB_TYPE_8x8==uiSubMbType){
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][uiScan4Idx],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][uiScan4Idx+1],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][uiScan4Idx+4],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][uiScan4Idx+5],LD32(iMv));
						ST32(iMvArray[0][uiCacheIdx],LD32(iMv));
						ST32(iMvArray[0][uiCacheIdx+1],LD32(iMv));
						ST32(iMvArray[0][uiCacheIdx+6],LD32(iMv));
						ST32(iMvArray[0][uiCacheIdx+7],LD32(iMv));
					}else
					if(SUB_MB_TYPE_8x4==uiSubMbType){
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][uiScan4Idx],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][uiScan4Idx+1],LD32(iMv));
						ST32(iMvArray[0][uiCacheIdx],LD32(iMv));
						ST32(iMvArray[0][uiCacheIdx+1],LD32(iMv));
					}else
					if(SUB_MB_TYPE_4x8==uiSubMbType){
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][uiScan4Idx],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][uiScan4Idx+4],LD32(iMv));
						ST32(iMvArray[0][uiCacheIdx],LD32(iMv));
						ST32(iMvArray[0][uiCacheIdx+6],LD32(iMv));
					}else{		// SUB_MB_TYPE_4x4==uiSubMbType
						ST32(pCurDqLayer->pDec->pMv[0][iMbXy][uiScan4Idx],LD32(iMv));
						ST32(iMvArray[0][uiCacheIdx],LD32(iMv));
					}
				}
			}
		}
		break;
		default:
			break;
	}

	return ERR_NONE;
}

const uint8_t g_kuiIntra4x4CbpTable[48]={
	47,31,15,0,23,27,29,30,7,11,13,14,39,43,45,46,// 15
	16,3,5,10,12,19,21,26,28,35,37,42,44,1,2,4,// 31
	8,17,18,20,24,6,9,22,25,32,33,34,36,40,38,41		// 47
};
const uint8_t g_kuiIntra4x4CbpTable400[16]={
	15,0,7,11,13,14,3,5,10,12,1,2,4,8,6,9
};
const uint8_t g_kuiInterCbpTable[48]={
	0,16,1,2,4,8,32,3,5,10,12,15,47,7,11,13,// 15
	14,6,9,31,35,37,42,44,33,34,36,40,39,43,45,46,// 31
	17,18,20,24,19,21,26,28,23,27,29,30,22,25,38,41		// 47
};

const uint8_t g_kuiInterCbpTable400[16]={
	0,1,2,4,8,3,5,10,12,15,7,11,13,14,6,9
};

void BsStartCavlc(SBitStringAux* pBs){
	pBs->iIndex=((pBs->pCurBuf-pBs->pStartBuf)<<3)-(16-pBs->iLeftBits);
}
void BsEndCavlc(SBitStringAux* pBs){
	pBs->pCurBuf=pBs->pStartBuf+(pBs->iIndex>>3);
	uint32_t uiCache32Bit=(uint32_t)((((pBs->pCurBuf[0]<<8)|pBs->pCurBuf[1])<<16)|
										(pBs->pCurBuf[2]<<8)|pBs->pCurBuf[3]);
	pBs->uiCurBits=uiCache32Bit<<(pBs->iIndex&0x07);
	pBs->pCurBuf+=4;
	pBs->iLeftBits=-16+(pBs->iIndex&0x07);
}

const uint8_t g_kuiCache48CountScan4Idx[24]={
	// Luma
	9,10,17,18,		// 1+1*8,2+1*8,1+2*8,2+2*8,
	11,12,19,20,		// 3+1*8,4+1*8,3+2*8,4+2*8,
	25,26,33,34,		// 1+3*8,2+3*8,1+4*8,2+4*8,
	27,28,35,36,		// 3+3*8,4+3*8,3+4*8,4+4*8,
	// Cb
	14,15,		// 6+1*8,7+1*8,
	22,23,		// 6+2*8,7+2*8,
	// Cr
	38,39,		// 6+4*8,7+4*8,
	46,47,		// 6+5*8,7+5*8,
};

const uint8_t g_kuiVlcTableNeedMoreBitsThread[3]={
	4,4,8
};

const uint8_t g_kuiVlcTableMoreBitsCount0[4]={
	8,2,1,1
};

const uint8_t g_kuiVlcTableMoreBitsCount1[4]={
	6,3,1,1
};

const uint8_t g_kuiVlcTableMoreBitsCount2[8]={
	2,2,2,2,1,1,1,1
};
const uint8_t g_kuiNcMapTable[17]={
	0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,3
};

const uint8_t g_kuiVlcTrailingOneTotalCoeffTable[62][2]={
	{0,0},
 {0,1},{1,1},
 {0,2},{1,2},{2,2},
 {0,3},{1,3},{2,3},{3,3},
 {0,4},{1,4},{2,4},{3,4},
 {0,5},{1,5},{2,5},{3,5},
 {0,6},{1,6},{2,6},{3,6},
 {0,7},{1,7},{2,7},{3,7},
 {0,8},{1,8},{2,8},{3,8},
 {0,9},{1,9},{2,9},{3,9},
 {0,10},{1,10},{2,10},{3,10},
 {0,11},{1,11},{2,11},{3,11},
 {0,12},{1,12},{2,12},{3,12},
 {0,13},{1,13},{2,13},{3,13},
 {0,14},{1,14},{2,14},{3,14},
 {0,15},{1,15},{2,15},{3,15},
 {0,16},{1,16},{2,16},{3,16}
};

// return: used bits
static int32_t CavlcGetTrailingOnesAndTotalCoeff(uint8_t& uiTotalCoeff,uint8_t& uiTrailingOnes,SReadBitsCache* pBitsCache,SVlcTable* pVlcTable,bool bChromaDc,int8_t nC){
	const uint8_t* kpVlcTableMoreBitsCountList[3]={g_kuiVlcTableMoreBitsCount0,g_kuiVlcTableMoreBitsCount1,g_kuiVlcTableMoreBitsCount2};
	int32_t iUsedBits=0;
	int32_t iIndexVlc,iIndexValue,iNcMapIdx;
	uint32_t uiCount;
	uint32_t uiValue;

	if(bChromaDc){
		uiValue=pBitsCache->uiCache32Bit>>24;
		iIndexVlc=pVlcTable->kpChromaCoeffTokenVlcTable[uiValue][0];
		uiCount=pVlcTable->kpChromaCoeffTokenVlcTable[uiValue][1];
		POP_BUFFER(pBitsCache,uiCount);
		iUsedBits+=uiCount;
		uiTrailingOnes=g_kuiVlcTrailingOneTotalCoeffTable[iIndexVlc][0];
		uiTotalCoeff=g_kuiVlcTrailingOneTotalCoeffTable[iIndexVlc][1];
	}else{		// luma
		iNcMapIdx=g_kuiNcMapTable[nC];
		if(iNcMapIdx<=2){
			uiValue=pBitsCache->uiCache32Bit>>24;
			if(uiValue<g_kuiVlcTableNeedMoreBitsThread[iNcMapIdx]){
				POP_BUFFER(pBitsCache,8);
				iUsedBits+=8;
				iIndexValue=pBitsCache->uiCache32Bit>>(32-kpVlcTableMoreBitsCountList[iNcMapIdx][uiValue]);
				iIndexVlc=pVlcTable->kpCoeffTokenVlcTable[iNcMapIdx+1][uiValue][iIndexValue][0];
				uiCount=pVlcTable->kpCoeffTokenVlcTable[iNcMapIdx+1][uiValue][iIndexValue][1];
				POP_BUFFER(pBitsCache,uiCount);
				iUsedBits+=uiCount;
			}else{
				iIndexVlc=pVlcTable->kpCoeffTokenVlcTable[0][iNcMapIdx][uiValue][0];
				uiCount=pVlcTable->kpCoeffTokenVlcTable[0][iNcMapIdx][uiValue][1];
				uiValue=pBitsCache->uiCache32Bit>>(32-uiCount);
				POP_BUFFER(pBitsCache,uiCount);
				iUsedBits+=uiCount;
			}
		}else{
			uiValue=pBitsCache->uiCache32Bit>>(32-6);
			POP_BUFFER(pBitsCache,6);
			iUsedBits+=6;
			iIndexVlc=pVlcTable->kpCoeffTokenVlcTable[0][3][uiValue][0];		// differ
		}
		uiTrailingOnes=g_kuiVlcTrailingOneTotalCoeffTable[iIndexVlc][0];
		uiTotalCoeff=g_kuiVlcTrailingOneTotalCoeffTable[iIndexVlc][1];
	}

	return iUsedBits;
}

static const uint32_t g_kuiPrefix8BitsTable[16]={
	0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3
};

static inline uint32_t GetPrefixBits(uint32_t uiValue){
	uint32_t iNumBit=0;

	if(uiValue&0xffff0000){
		uiValue>>=16;
		iNumBit+=16;
	}
	if(uiValue&0xff00){
		uiValue>>=8;
		iNumBit+=8;
	}

	if(uiValue&0xf0){
		uiValue>>=4;
		iNumBit+=4;
	}
	iNumBit+=g_kuiPrefix8BitsTable[uiValue];

	return (32-iNumBit);
}

#define WELS_GET_PREFIX_BITS(inval,outval) outval=GetPrefixBits(inval)

static int32_t CavlcGetLevelVal(int32_t iLevel[16],SReadBitsCache* pBitsCache,uint8_t uiTotalCoeff,uint8_t uiTrailingOnes){
	int32_t i,iUsedBits=0;
	int32_t iSuffixLength,iSuffixLengthSize,iLevelPrefix,iPrefixBits,iLevelCode,iThreshold;
	for(i=0; i<uiTrailingOnes; i++){
		iLevel[i]=1-((pBitsCache->uiCache32Bit>>(30-i))&0x02);
	}
	POP_BUFFER(pBitsCache,uiTrailingOnes);
	iUsedBits+=uiTrailingOnes;

	iSuffixLength=(uiTotalCoeff>10 && uiTrailingOnes<3);

	for(; i<uiTotalCoeff; i++){
		if(pBitsCache->uiRemainBits<=16) SHIFT_BUFFER(pBitsCache);
		WELS_GET_PREFIX_BITS(pBitsCache->uiCache32Bit,iPrefixBits);
		if(iPrefixBits>MAX_LEVEL_PREFIX+1)		// iPrefixBits includes leading "0"s and first "1",should+1
			return-1;
		POP_BUFFER(pBitsCache,iPrefixBits);
		iUsedBits+=iPrefixBits;
		iLevelPrefix=iPrefixBits-1;

		iLevelCode=iLevelPrefix<<iSuffixLength;		// differ
		iSuffixLengthSize=iSuffixLength;

		if(iLevelPrefix>=14){
			if(14==iLevelPrefix && 0==iSuffixLength)
				iSuffixLengthSize=4;
			else if(15==iLevelPrefix){
				iSuffixLengthSize=12;
				if(iSuffixLength==0)
					iLevelCode+=15;
			}
		}

		if(iSuffixLengthSize>0){
			if(pBitsCache->uiRemainBits<=iSuffixLengthSize) SHIFT_BUFFER(pBitsCache);
			iLevelCode+=(pBitsCache->uiCache32Bit>>(32-iSuffixLengthSize));
			POP_BUFFER(pBitsCache,iSuffixLengthSize);
			iUsedBits+=iSuffixLengthSize;
		}

		iLevelCode+=((i==uiTrailingOnes) && (uiTrailingOnes<3))<<1;
		iLevel[i]=((iLevelCode+2)>>1);
		iLevel[i]-=(iLevel[i]<<1)&(-(iLevelCode&0x01));

		iSuffixLength+=!iSuffixLength;
		iThreshold=3<<(iSuffixLength-1);
		iSuffixLength+=((iLevel[i]>iThreshold) || (iLevel[i]<-iThreshold)) && (iSuffixLength<6);
	}

	return iUsedBits;
}

const uint8_t g_kuiTotalZerosBitNumChromaMap[3]={
	3,2,1
};
const uint8_t g_kuiTotalZerosBitNumMap[15]={
	9,6,6,5,5,6,6,6,6,5,4,4,3,2,1
};

static int32_t CavlcGetTotalZeros(int32_t& iZerosLeft,SReadBitsCache* pBitsCache,uint8_t uiTotalCoeff,SVlcTable* pVlcTable,bool bChromaDc){
	int32_t iCount,iUsedBits=0;
	const uint8_t* kpBitNumMap;
	uint32_t uiValue;

	int32_t iTotalZeroVlcIdx;
	uint8_t uiTableType;
	// chroma_dc (0 < uiTotalCoeff < 4); others (chroma_ac or luma: 0 < uiTotalCoeff < 16)

	if(bChromaDc){
		iTotalZeroVlcIdx=uiTotalCoeff;
		kpBitNumMap=g_kuiTotalZerosBitNumChromaMap;
		uiTableType=bChromaDc;
	}else{
		iTotalZeroVlcIdx=uiTotalCoeff;
		kpBitNumMap=g_kuiTotalZerosBitNumMap;
		uiTableType=0;
	}

	iCount=kpBitNumMap[iTotalZeroVlcIdx-1];
	if(pBitsCache->uiRemainBits<iCount) SHIFT_BUFFER(
		pBitsCache);		// if uiRemainBits+16 still smaller than iCount?? potential bug
	uiValue=pBitsCache->uiCache32Bit>>(32-iCount);
	iCount=pVlcTable->kpTotalZerosTable[uiTableType][iTotalZeroVlcIdx-1][uiValue][1];
	POP_BUFFER(pBitsCache,iCount);
	iUsedBits+=iCount;
	iZerosLeft=pVlcTable->kpTotalZerosTable[uiTableType][iTotalZeroVlcIdx-1][uiValue][0];

	return iUsedBits;
}

const uint8_t g_kuiZeroLeftBitNumMap[16]={
	0,1,2,2,3,3,3,3,3,3,3,3,3,3,3,3
};

static int32_t CavlcGetRunBefore(int32_t iRun[16],SReadBitsCache* pBitsCache,uint8_t uiTotalCoeff,SVlcTable* pVlcTable,int32_t iZerosLeft){
	int32_t i,iUsedBits=0;
	uint32_t uiCount,uiValue,iPrefixBits;

	for(i=0; i<uiTotalCoeff-1; i++){
		if(iZerosLeft>0){
			uiCount=g_kuiZeroLeftBitNumMap[iZerosLeft];
			if(pBitsCache->uiRemainBits<uiCount) SHIFT_BUFFER(pBitsCache);
			uiValue=pBitsCache->uiCache32Bit>>(32-uiCount);
			if(iZerosLeft<7){
				uiCount=pVlcTable->kpZeroTable[iZerosLeft-1][uiValue][1];
				POP_BUFFER(pBitsCache,uiCount);
				iUsedBits+=uiCount;
				iRun[i]=pVlcTable->kpZeroTable[iZerosLeft-1][uiValue][0];
			}else{
				POP_BUFFER(pBitsCache,uiCount);
				iUsedBits+=uiCount;
				if(pVlcTable->kpZeroTable[6][uiValue][0]<7){
					iRun[i]=pVlcTable->kpZeroTable[6][uiValue][0];
				}else{
					if(pBitsCache->uiRemainBits<16) SHIFT_BUFFER(pBitsCache);
					WELS_GET_PREFIX_BITS(pBitsCache->uiCache32Bit,iPrefixBits);
					iRun[i]=iPrefixBits+6;
					if(iRun[i]>iZerosLeft)
						return-1;
					POP_BUFFER(pBitsCache,iPrefixBits);
					iUsedBits+=iPrefixBits;
				}
			}
		}else{
			for(int j=i; j<uiTotalCoeff; j++){
				iRun[j]=0;
			}
			return iUsedBits;
		}

		iZerosLeft-=iRun[i];
	}

	iRun[uiTotalCoeff-1]=iZerosLeft;

	return iUsedBits;
}

int32_t WelsResidualBlockCavlc(SVlcTable* pVlcTable,uint8_t* pNonZeroCountCache,SBitStringAux* pBs,int32_t iIndex,int32_t iMaxNumCoeff,const uint8_t* kpZigzagTable,int32_t iResidualProperty,int16_t* pTCoeff,uint8_t uiQp,SDecoderContext* pCtx){
	int32_t iLevel[16],iZerosLeft,iCoeffNum;
	int32_t iRun[16];
	int32_t iCurNonZeroCacheIdx,i;


	int32_t iMbResProperty=0;
	GetMbResProperty(&iMbResProperty,&iResidualProperty,1);
	const uint16_t* kpDequantCoeff=pCtx->bUseScalingList ? pCtx->pDequant_coeff4x4[iMbResProperty][uiQp] :
		g_kuiDequantCoeff[uiQp];

	int8_t nA,nB,nC;
	uint8_t uiTotalCoeff,uiTrailingOnes;
	int32_t iUsedBits=0;
	intX_t iCurIdx=pBs->iIndex;
	uint8_t* pBuf=((uint8_t*)pBs->pStartBuf)+(iCurIdx>>3);
	bool bChromaDc=(CHROMA_DC==iResidualProperty);
	uint8_t bChroma=(bChromaDc || CHROMA_AC==iResidualProperty);
	SReadBitsCache sReadBitsCache;

	uint32_t uiCache32Bit=(uint32_t)((((pBuf[0]<<8)|pBuf[1])<<16)|(pBuf[2]<<8)|pBuf[3]);
	sReadBitsCache.uiCache32Bit=uiCache32Bit<<(iCurIdx&0x07);
	sReadBitsCache.uiRemainBits=32-(iCurIdx&0x07);
	sReadBitsCache.pBuf=pBuf;
	if(bChroma){
		iCurNonZeroCacheIdx=g_kuiCache48CountScan4Idx[iIndex];
		nA=pNonZeroCountCache[iCurNonZeroCacheIdx-1];
		nB=pNonZeroCountCache[iCurNonZeroCacheIdx-8];
	}else{		// luma
		iCurNonZeroCacheIdx=g_kuiCache48CountScan4Idx[iIndex];
		nA=pNonZeroCountCache[iCurNonZeroCacheIdx-1];
		nB=pNonZeroCountCache[iCurNonZeroCacheIdx-8];
	}

	WELS_NON_ZERO_COUNT_AVERAGE(nC,nA,nB);

	iUsedBits+=CavlcGetTrailingOnesAndTotalCoeff(uiTotalCoeff,uiTrailingOnes,&sReadBitsCache,pVlcTable,bChromaDc,nC);

	if(iResidualProperty!=CHROMA_DC && iResidualProperty!=I16_LUMA_DC){
		pNonZeroCountCache[iCurNonZeroCacheIdx]=uiTotalCoeff;
	}
	if(0==uiTotalCoeff){
		pBs->iIndex+=iUsedBits;
		return ERR_NONE;
	}
	if((uiTrailingOnes>3) || (uiTotalCoeff>16)){		// // // // // // // // /check uiTrailingOnes and uiTotalCoeff
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_CAVLC_INVALID_TOTAL_COEFF_OR_TRAILING_ONES);
	}
	if((i=CavlcGetLevelVal(iLevel,&sReadBitsCache,uiTotalCoeff,uiTrailingOnes))==-1){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_CAVLC_INVALID_LEVEL);
	}
	iUsedBits+=i;
	if(uiTotalCoeff<iMaxNumCoeff){
		iUsedBits+=CavlcGetTotalZeros(iZerosLeft,&sReadBitsCache,uiTotalCoeff,pVlcTable,bChromaDc);
	}else{
		iZerosLeft=0;
	}

	if((iZerosLeft<0) || ((iZerosLeft+uiTotalCoeff)>iMaxNumCoeff)){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_CAVLC_INVALID_ZERO_LEFT);
	}
	if((i=CavlcGetRunBefore(iRun,&sReadBitsCache,uiTotalCoeff,pVlcTable,iZerosLeft))==-1){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_CAVLC_INVALID_RUN_BEFORE);
	}
	iUsedBits+=i;
	pBs->iIndex+=iUsedBits;
	iCoeffNum=-1;

	if(iResidualProperty==CHROMA_DC){
		// chroma dc scaling process,is kpDequantCoeff[0]? LevelScale(qPdc%6,0,0))<<(qPdc/6-6),the transform is done at construction.
		for(i=uiTotalCoeff-1; i>=0;--i){
			// FIXME merge into rundecode?
			int32_t j;
			iCoeffNum+=iRun[i]+1;		// FIXME add 1 earlier ?
			j=kpZigzagTable[iCoeffNum];
			pTCoeff[j]=iLevel[i];
		}
		WelsChromaDcIdct(pTCoeff);
		// scaling
		if(!pCtx->bUseScalingList){
			for(int j=0; j<4;++j){
				pTCoeff[kpZigzagTable[j]]=(pTCoeff[kpZigzagTable[j]]*kpDequantCoeff[0])>>1;
			}
		}else{
			for(int j=0; j<4;++j){
				pTCoeff[kpZigzagTable[j]]=((int64_t)pTCoeff[kpZigzagTable[j]]*(int64_t)kpDequantCoeff[0])>>5;
			}
		}
	}else
	if(iResidualProperty==I16_LUMA_DC){		// DC coefficent,only call in Intra_16x16,base_mode_flag=0
		for(i=uiTotalCoeff-1; i>=0;--i){		// FIXME merge into rundecode?
			int32_t j;
			iCoeffNum+=iRun[i]+1;		// FIXME add 1 earlier ?
			j=kpZigzagTable[iCoeffNum];
			pTCoeff[j]=iLevel[i];
		}
		WelsLumaDcDequantIdct(pTCoeff,uiQp,pCtx);
	}else{
		for(i=uiTotalCoeff-1; i>=0;--i){		// FIXME merge into rundecode?
			int32_t j;
			iCoeffNum+=iRun[i]+1;		// FIXME add 1 earlier ?
			j=kpZigzagTable[iCoeffNum];
			if(!pCtx->bUseScalingList){
				pTCoeff[j]=(iLevel[i]*kpDequantCoeff[j&0x07]);
			}else{
				pTCoeff[j]=(iLevel[i]*kpDequantCoeff[j]+8)>>4;
			}
		}
	}

	return ERR_NONE;
}


int32_t WelsResidualBlockCavlc8x8(SVlcTable* pVlcTable,uint8_t* pNonZeroCountCache,SBitStringAux* pBs,int32_t iIndex,int32_t iMaxNumCoeff,const uint8_t* kpZigzagTable,int32_t iResidualProperty,int16_t* pTCoeff,int32_t iIdx4x4,uint8_t uiQp,SDecoderContext* pCtx){
	int32_t iLevel[16],iZerosLeft,iCoeffNum;
	int32_t iRun[16];
	int32_t iCurNonZeroCacheIdx,i;

	int32_t iMbResProperty=0;
	GetMbResProperty(&iMbResProperty,&iResidualProperty,1);

	const uint16_t* kpDequantCoeff=pCtx->bUseScalingList ? pCtx->pDequant_coeff8x8[iMbResProperty-6][uiQp] :
		g_kuiDequantCoeff8x8[uiQp];

	int8_t nA,nB,nC;
	uint8_t uiTotalCoeff,uiTrailingOnes;
	int32_t iUsedBits=0;
	intX_t iCurIdx=pBs->iIndex;
	uint8_t* pBuf=((uint8_t*)pBs->pStartBuf)+(iCurIdx>>3);
	bool bChromaDc=(CHROMA_DC==iResidualProperty);
	uint8_t bChroma=(bChromaDc || CHROMA_AC==iResidualProperty);
	SReadBitsCache sReadBitsCache;

	uint32_t uiCache32Bit=(uint32_t)((((pBuf[0]<<8)|pBuf[1])<<16)|(pBuf[2]<<8)|pBuf[3]);
	sReadBitsCache.uiCache32Bit=uiCache32Bit<<(iCurIdx&0x07);
	sReadBitsCache.uiRemainBits=32-(iCurIdx&0x07);
	sReadBitsCache.pBuf=pBuf;
	if(bChroma){
		iCurNonZeroCacheIdx=g_kuiCache48CountScan4Idx[iIndex];
		nA=pNonZeroCountCache[iCurNonZeroCacheIdx-1];
		nB=pNonZeroCountCache[iCurNonZeroCacheIdx-8];
	}else{		// luma
		iCurNonZeroCacheIdx=g_kuiCache48CountScan4Idx[iIndex];
		nA=pNonZeroCountCache[iCurNonZeroCacheIdx-1];
		nB=pNonZeroCountCache[iCurNonZeroCacheIdx-8];
	}

	WELS_NON_ZERO_COUNT_AVERAGE(nC,nA,nB);

	iUsedBits+=CavlcGetTrailingOnesAndTotalCoeff(uiTotalCoeff,uiTrailingOnes,&sReadBitsCache,pVlcTable,bChromaDc,nC);

	if(iResidualProperty!=CHROMA_DC && iResidualProperty!=I16_LUMA_DC){
		pNonZeroCountCache[iCurNonZeroCacheIdx]=uiTotalCoeff;
	}
	if(0==uiTotalCoeff){
		pBs->iIndex+=iUsedBits;
		return ERR_NONE;
	}
	if((uiTrailingOnes>3) || (uiTotalCoeff>16)){		// check uiTrailingOnes and uiTotalCoeff
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_CAVLC_INVALID_TOTAL_COEFF_OR_TRAILING_ONES);
	}
	if((i=CavlcGetLevelVal(iLevel,&sReadBitsCache,uiTotalCoeff,uiTrailingOnes))==-1){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_CAVLC_INVALID_LEVEL);
	}
	iUsedBits+=i;
	if(uiTotalCoeff<iMaxNumCoeff){
		iUsedBits+=CavlcGetTotalZeros(iZerosLeft,&sReadBitsCache,uiTotalCoeff,pVlcTable,bChromaDc);
	}else{
		iZerosLeft=0;
	}

	if((iZerosLeft<0) || ((iZerosLeft+uiTotalCoeff)>iMaxNumCoeff)){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_CAVLC_INVALID_ZERO_LEFT);
	}
	if((i=CavlcGetRunBefore(iRun,&sReadBitsCache,uiTotalCoeff,pVlcTable,iZerosLeft))==-1){
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_CAVLC_INVALID_RUN_BEFORE);
	}
	iUsedBits+=i;
	pBs->iIndex+=iUsedBits;
	iCoeffNum=-1;

	for(i=uiTotalCoeff-1; i>=0;--i){		// FIXME merge into rundecode?
		int32_t j;
		iCoeffNum+=iRun[i]+1;				// FIXME add 1 earlier ?
		j=(iCoeffNum<<2)+iIdx4x4;
		j=kpZigzagTable[j];
		pTCoeff[j]=uiQp>=36 ? ((iLevel[i]*kpDequantCoeff[j])*(1<<(uiQp/6-6)))
			: ((iLevel[i]*kpDequantCoeff[j]+(1<<(5-uiQp/6)))>>(6-uiQp/6));
	}

	return ERR_NONE;
}

int32_t WelsActualDecodeMbCavlcPSlice(SDecoderContext* pCtx){
	SVlcTable* pVlcTable=pCtx->pVlcTable;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SBitStringAux* pBs=pCurDqLayer->pBitStringAux;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;

	int32_t iScanIdxStart=pSlice->sSliceHeaderExt.uiScanIdxStart;
	int32_t iScanIdxEnd=pSlice->sSliceHeaderExt.uiScanIdxEnd;

	SWelsNeighAvail sNeighAvail;
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;
	const int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int8_t* pNzc=pCurDqLayer->pNzc[iMbXy];
	int32_t i;
	int32_t iRet=ERR_NONE;
	uint32_t uiMbType=0,uiCbp=0,uiCbpL=0,uiCbpC=0;
	uint32_t uiCode;
	int32_t iCode;
	int32_t iMbResProperty;

	GetNeighborAvailMbType(&sNeighAvail,pCurDqLayer);
	ENFORCE_STACK_ALIGN_1D(uint8_t,pNonZeroCount,48,16);
	pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;	// 2009.10.23
	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// uiMbType
	uiMbType=uiCode;
	if(uiMbType<5){		// inter MB type
		int16_t iMotionVector[LIST_A][30][MV_A];
		int8_t iRefIndex[LIST_A][30];
		pCurDqLayer->pDec->pMbType[iMbXy]=g_ksInterPMbTypeInfo[uiMbType].iType;
		WelsFillCacheInter(&sNeighAvail,pNonZeroCount,iMotionVector,iRefIndex,pCurDqLayer);

		if((iRet=ParseInterInfo(pCtx,iMotionVector,iRefIndex,pBs))!=ERR_NONE){
			return iRet;	// abnormal
		}

		if(pSlice->sSliceHeaderExt.bAdaptiveResidualPredFlag==1){
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// residual_prediction_flag
			pCurDqLayer->pResidualPredFlag[iMbXy]=uiCode;
		}else{
			pCurDqLayer->pResidualPredFlag[iMbXy]=pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;
		}

		if(pCurDqLayer->pResidualPredFlag[iMbXy]==0){
			pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
		}else{
			uprintf("residual_pred_flag=1 not supported.");
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
		}
	}else{		// intra MB type
		uiMbType-=5;
		if(uiMbType>25)
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);
		if(!pCtx->pSps->uiChromaFormatIdc && ((uiMbType>=5 && uiMbType<=12) || (uiMbType>=17 && uiMbType<=24)))
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);

		if(25==uiMbType){
			uprintf("I_PCM mode exists in P slice!");
			int32_t iDecStrideL=pCurDqLayer->pDec->iLinesize[0];
			int32_t iDecStrideC=pCurDqLayer->pDec->iLinesize[1];

			int32_t iOffsetL=(iMbX+iMbY*iDecStrideL)<<4;
			int32_t iOffsetC=(iMbX+iMbY*iDecStrideC)<<3;

			uint8_t* pDecY=pCurDqLayer->pDec->pData[0]+iOffsetL;
			uint8_t* pDecU=pCurDqLayer->pDec->pData[1]+iOffsetC;
			uint8_t* pDecV=pCurDqLayer->pDec->pData[2]+iOffsetC;

			uint8_t* pTmpBsBuf;

			int32_t i;
			int32_t iCopySizeY=(sizeof(uint8_t)<<4);
			int32_t iCopySizeUV=(sizeof(uint8_t)<<3);

			int32_t iIndex=((-pBs->iLeftBits)>>3)+2;

			pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA_PCM;

			// step 1: locating bit-stream pointer [must align into integer byte]
			pBs->pCurBuf-=iIndex;

			// step 2: copy pixel from bit-stream into fdec [reconstruction]
			pTmpBsBuf=pBs->pCurBuf;
			for(i=0; i<16; i++){		// luma
				memcpy(pDecY,pTmpBsBuf,iCopySizeY);
				pDecY+=iDecStrideL;
				pTmpBsBuf+=16;
			}

			for(i=0; i<8; i++){		// cb
				memcpy(pDecU,pTmpBsBuf,iCopySizeUV);
				pDecU+=iDecStrideC;
				pTmpBsBuf+=8;
			}
			for(i=0; i<8; i++){		// cr
				memcpy(pDecV,pTmpBsBuf,iCopySizeUV);
				pDecV+=iDecStrideC;
				pTmpBsBuf+=8;
			}

			pBs->pCurBuf+=384;

			// step 3: update QP and pNonZeroCount
			pCurDqLayer->pLumaQp[iMbXy]=0;
			pCurDqLayer->pChromaQp[iMbXy][0]=pCurDqLayer->pChromaQp[iMbXy][1]=0;
			// Rec. 9.2.1 for PCM,nzc=16
			ST32A4(&pNzc[0],0x10101010);
			ST32A4(&pNzc[4],0x10101010);
			ST32A4(&pNzc[8],0x10101010);
			ST32A4(&pNzc[12],0x10101010);
			ST32A4(&pNzc[16],0x10101010);
			ST32A4(&pNzc[20],0x10101010);
			WELS_READ_VERIFY(InitReadBits(pBs,0));
			return ERR_NONE;
		}else{
			if(0==uiMbType){
				ENFORCE_STACK_ALIGN_1D(int8_t,pIntraPredMode,48,16);
				pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA4x4;
				if(pCtx->pPps->bTransform8x8ModeFlag){
					WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// transform_size_8x8_flag
					pCurDqLayer->pTransformSize8x8Flag[iMbXy]=!!uiCode;
					if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
						uiMbType=pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA8x8;
					}
				}
				if(!pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
					pCtx->pFillInfoCacheIntraNxNFunc(&sNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
					WELS_READ_VERIFY(ParseIntra4x4Mode(pCtx,&sNeighAvail,pIntraPredMode,pBs,pCurDqLayer));
				}else{
					pCtx->pFillInfoCacheIntraNxNFunc(&sNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
					WELS_READ_VERIFY(ParseIntra8x8Mode(pCtx,&sNeighAvail,pIntraPredMode,pBs,pCurDqLayer));
				}
			}else{		// I_PCM exclude,we can ignore it
				pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA16x16;
				pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;
				pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
				pCurDqLayer->pIntraPredMode[iMbXy][7]=(uiMbType-1)&3;
				pCurDqLayer->pCbp[iMbXy]=g_kuiI16CbpTable[(uiMbType-1)>>2];
				uiCbpC=pCtx->pSps->uiChromaFormatIdc ? pCurDqLayer->pCbp[iMbXy]>>4 : 0;
				uiCbpL=pCurDqLayer->pCbp[iMbXy]&15;
				WelsFillCacheNonZeroCount(&sNeighAvail,pNonZeroCount,pCurDqLayer);
				if((iRet=ParseIntra16x16Mode(pCtx,&sNeighAvail,pBs,pCurDqLayer))!=ERR_NONE){
					return iRet;
				}
			}
		}
	}

	if(MB_TYPE_INTRA16x16!=pCurDqLayer->pDec->pMbType[iMbXy]){
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// coded_block_pattern
		uiCbp=uiCode;
		{
			if(pCtx->pSps->uiChromaFormatIdc && (uiCbp>47))
				return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_CBP);
			if(!pCtx->pSps->uiChromaFormatIdc && (uiCbp>15))
				return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_CBP);
			if(MB_TYPE_INTRA4x4==pCurDqLayer->pDec->pMbType[iMbXy] || MB_TYPE_INTRA8x8==pCurDqLayer->pDec->pMbType[iMbXy]){

				uiCbp=pCtx->pSps->uiChromaFormatIdc ? g_kuiIntra4x4CbpTable[uiCbp] : g_kuiIntra4x4CbpTable400[uiCbp];
			}else		// inter
				uiCbp=pCtx->pSps->uiChromaFormatIdc ? g_kuiInterCbpTable[uiCbp] : g_kuiInterCbpTable400[uiCbp];
		}

		pCurDqLayer->pCbp[iMbXy]=uiCbp;
		uiCbpC=pCurDqLayer->pCbp[iMbXy]>>4;
		uiCbpL=pCurDqLayer->pCbp[iMbXy]&15;

		// Need modification when B picutre add in
		bool bNeedParseTransformSize8x8Flag=
			(((pCurDqLayer->pDec->pMbType[iMbXy]>=MB_TYPE_16x16 && pCurDqLayer->pDec->pMbType[iMbXy]<=MB_TYPE_8x16)
				 || pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy])
			  && (pCurDqLayer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA8x8)
			  && (pCurDqLayer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA4x4)
			  && (uiCbpL>0)
			  && (pCtx->pPps->bTransform8x8ModeFlag));

		if(bNeedParseTransformSize8x8Flag){
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// transform_size_8x8_flag
			pCurDqLayer->pTransformSize8x8Flag[iMbXy]=!!uiCode;
		}
	}

	ST32A4(&pNzc[0],0);
	ST32A4(&pNzc[4],0);
	ST32A4(&pNzc[8],0);
	ST32A4(&pNzc[12],0);
	ST32A4(&pNzc[16],0);
	ST32A4(&pNzc[20],0);
	if(pCurDqLayer->pCbp[iMbXy]==0 && !IS_INTRA16x16(pCurDqLayer->pDec->pMbType[iMbXy])
		 && !IS_I_BL(pCurDqLayer->pDec->pMbType[iMbXy])){
		pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pCurDqLayer->pLumaQp[iMbXy]+pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}
	}

	if(pCurDqLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){
		int32_t iQpDelta,iId8x8,iId4x4;
		memset(pCurDqLayer->pScaledTCoeff[iMbXy],0,MB_COEFF_LIST_SIZE*sizeof(int16_t));
		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mb_qp_delta
		iQpDelta=iCode;

		if(iQpDelta>25 || iQpDelta<-26){		// out of iQpDelta range
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_QP);
		}

		pCurDqLayer->pLumaQp[iMbXy]=(pSlice->iLastMbQp+iQpDelta+52)%52;		// update last_mb_qp
		pSlice->iLastMbQp=pCurDqLayer->pLumaQp[iMbXy];
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pSlice->iLastMbQp+pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}

		BsStartCavlc(pBs);

		if(MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){
			// step1: Luma DC
			if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,0,16,g_kuiLumaDcZigzagScan,I16_LUMA_DC,pCurDqLayer->pScaledTCoeff[iMbXy],pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
				return iRet;	// abnormal
			}
			// step2: Luma AC
			if(uiCbpL){
				for(i=0; i<16; i++){
					if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,i,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),I16_LUMA_AC,pCurDqLayer->pScaledTCoeff[iMbXy]+(i<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
						return iRet;	// abnormal
					}
				}
				ST32A4(&pNzc[0],LD32(&pNonZeroCount[1+8*1]));
				ST32A4(&pNzc[4],LD32(&pNonZeroCount[1+8*2]));
				ST32A4(&pNzc[8],LD32(&pNonZeroCount[1+8*3]));
				ST32A4(&pNzc[12],LD32(&pNonZeroCount[1+8*4]));
			}
		}else{		// non-MB_TYPE_INTRA16x16
			if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
				for(iId8x8=0; iId8x8<4; iId8x8++){
					iMbResProperty=(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA_8 : LUMA_DC_AC_INTER_8;
					if(uiCbpL&(1<<iId8x8)){
						int32_t iIndex=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							if((iRet=WelsResidualBlockCavlc8x8(pVlcTable,pNonZeroCount,pBs,iIndex,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan8x8+iScanIdxStart,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iId8x8<<6),iId4x4,pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
								return iRet;
							}
							iIndex++;
						}
					}else{
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8<<2]],0);
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[(iId8x8<<2)+2]],0);
					}
				}
				ST32A4(&pNzc[0],LD32(&pNonZeroCount[1+8*1]));
				ST32A4(&pNzc[4],LD32(&pNonZeroCount[1+8*2]));
				ST32A4(&pNzc[8],LD32(&pNonZeroCount[1+8*3]));
				ST32A4(&pNzc[12],LD32(&pNonZeroCount[1+8*4]));
			}else{		// Normal T4x4
				for(iId8x8=0; iId8x8<4; iId8x8++){
					iMbResProperty=(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
					if(uiCbpL&(1<<iId8x8)){
						int32_t iIndex=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							// Luma (DC and AC decoding together)
							if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,iIndex,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan+iScanIdxStart,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIndex<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
								return iRet;	// abnormal
							}
							iIndex++;
						}
					}else{
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8<<2]],0);
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[(iId8x8<<2)+2]],0);
					}
				}
				ST32A4(&pNzc[0],LD32(&pNonZeroCount[1+8*1]));
				ST32A4(&pNzc[4],LD32(&pNonZeroCount[1+8*2]));
				ST32A4(&pNzc[8],LD32(&pNonZeroCount[1+8*3]));
				ST32A4(&pNzc[12],LD32(&pNonZeroCount[1+8*4]));
			}
		}


		// chroma
		// step1: DC
		if(1==uiCbpC || 2==uiCbpC){
			for(i=0; i<2; i++){		// Cb Cr
				if(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_DC_V : CHROMA_DC_U;
				else
					iMbResProperty=i ? CHROMA_DC_V_INTER : CHROMA_DC_U_INTER;

				if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,16+(i<<2),4,g_kuiChromaDcScan,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+256+(i<<6),pCurDqLayer->pChromaQp[iMbXy][i],pCtx))!=ERR_NONE){
					return iRet;	// abnormal
				}
			}
		}else{
		}
		// step2: AC
		if(2==uiCbpC){
			for(i=0; i<2; i++){		// Cb Cr
				if(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_AC_V : CHROMA_AC_U;
				else
					iMbResProperty=i ? CHROMA_AC_V_INTER : CHROMA_AC_U_INTER;

				int32_t iIndex=16+(i<<2);
				for(iId4x4=0; iId4x4<4; iId4x4++){
					if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,iIndex,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIndex<<4),pCurDqLayer->pChromaQp[iMbXy][i],pCtx))!=ERR_NONE){
						return iRet;	// abnormal
					}
					iIndex++;
				}
			}
			ST16A2(&pNzc[16],LD16A2(&pNonZeroCount[6+8*1]));
			ST16A2(&pNzc[20],LD16A2(&pNonZeroCount[6+8*2]));
			ST16A2(&pNzc[18],LD16A2(&pNonZeroCount[6+8*4]));
			ST16A2(&pNzc[22],LD16A2(&pNonZeroCount[6+8*5]));
		}
		BsEndCavlc(pBs);
	}

	return ERR_NONE;
}


int32_t WelsDecodeMbCavlcPSlice(SDecoderContext* pCtx,SNalUnit* pNalCur,uint32_t& uiEosFlag){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SBitStringAux* pBs=pCurDqLayer->pBitStringAux;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	SPicture** ppRefPic=pCtx->sRefPic.pRefList[LIST_0];
	intX_t iUsedBits;
	const int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int8_t* pNzc=pCurDqLayer->pNzc[iMbXy];
	int32_t iBaseModeFlag,i;
	int32_t iRet=0;		// should have the return value to indicate decoding error or not,It's NECESSARY--2010.4.15
	uint32_t uiCode;

	pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
	pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;

	if(-1==pSlice->iMbSkipRun){
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// mb_skip_run
		pSlice->iMbSkipRun=uiCode;
		if(-1==pSlice->iMbSkipRun){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_SKIP_RUN);
		}
	}
	if(pSlice->iMbSkipRun--){
		int16_t iMv[2];

		pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_SKIP;
		ST32A4(&pNzc[0],0);
		ST32A4(&pNzc[4],0);
		ST32A4(&pNzc[8],0);
		ST32A4(&pNzc[12],0);
		ST32A4(&pNzc[16],0);
		ST32A4(&pNzc[20],0);

		pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
		memset(pCurDqLayer->pDec->pRefIndex[0][iMbXy],0,sizeof(int8_t)*16);
		bool bIsPending=false;	// GetThreadCount (pCtx) > 1;
		pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[0] && (ppRefPic[0]->bIsComplete
			 || bIsPending));
		// predict iMv
		PredPSkipMvFromNeighbor(pCurDqLayer,iMv);
		for(i=0; i<16; i++){
			ST32A2(pCurDqLayer->pDec->pMv[0][iMbXy][i],*(uint32_t*)iMv);
		}

		// if (!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag) {
		// memset (pCurDqLayer->pScaledTCoeff[iMbXy],0,384 * sizeof (int16_t));
		// }

		// reset rS
		if(!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag || 
			(pNalCur->sNalHeaderExt.uiQualityId==0 && pNalCur->sNalHeaderExt.uiDependencyId==0)){
			pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;
			for(i=0; i<2; i++){
				pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pCurDqLayer->pLumaQp[iMbXy]+
												 pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
			}
		}

		pCurDqLayer->pCbp[iMbXy]=0;
	}else{
		if(pSlice->sSliceHeaderExt.bAdaptiveBaseModeFlag==1){
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// base_mode_flag
			iBaseModeFlag=uiCode;
		}else{
			iBaseModeFlag=pSlice->sSliceHeaderExt.bDefaultBaseModeFlag;
		}
		if(!iBaseModeFlag){
			iRet=WelsActualDecodeMbCavlcPSlice(pCtx);
		}else{
			uprintf("iBaseModeFlag (%d) !=0,inter-layer prediction not supported.", iBaseModeFlag);
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_ILP);
		}
		if(iRet){		// occur error when parsing,MUST STOP decoding
			return iRet;
		}
	}
	// check whether there is left bits to read next time in case multiple slices
	iUsedBits=((pBs->pCurBuf-pBs->pStartBuf)<<3)-(16-pBs->iLeftBits);
	// sub 1,for stop bit
	if((iUsedBits==(pBs->iBits-1)) && (0>=pCurDqLayer->sLayerInfo.sSliceInLayer.iMbSkipRun)){		// slice boundary
		uiEosFlag=1;
	}
	if(iUsedBits>(pBs->iBits-
		1)){		// When BS incomplete,as long as find it,SHOULD stop decoding to avoid mosaic or crash.
		uprintf("WelsDecodeMbCavlcISlice()::::pBs incomplete,iUsedBits:%llu %d,MUST stop decoding.",(int64_t)iUsedBits,pBs->iBits);
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_BS_INCOMPLETE);
	}
	return ERR_NONE;
}

int32_t ParseInterBInfo(SDecoderContext* pCtx,int16_t iMvArray[LIST_A][30][MV_A],int8_t iRefIdxArray[LIST_A][30],SBitStringAux* pBs){
	SSlice* pSlice=&pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	SPicture** ppRefPic[2];
	ppRefPic[LIST_0]=pCtx->sRefPic.pRefList[LIST_0];
	ppRefPic[LIST_1]=pCtx->sRefPic.pRefList[LIST_1];
	int8_t ref_idx_list[LIST_A][4];
	int8_t iRef[2]={0,0};
	int32_t iRefCount[2];
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	uint8_t iMotionPredFlag[LIST_A][4];
	int16_t iMv[2];
	uint32_t uiCode;
	int32_t iCode;
	int16_t iMinVmv=pSliceHeader->pSps->pSLevelLimits->iMinVmv;
	int16_t iMaxVmv=pSliceHeader->pSps->pSLevelLimits->iMaxVmv;
	memset(ref_idx_list,-1,LIST_A*4);
	memset(iMotionPredFlag,(pSlice->sSliceHeaderExt.bDefaultMotionPredFlag ? 1 : 0),LIST_A*4);
	iRefCount[0]=pSliceHeader->uiRefCount[0];
	iRefCount[1]=pSliceHeader->uiRefCount[1];

	bool bIsPending=false;	// GetThreadCount (pCtx) > 1;

	MbType mbType=pCurDqLayer->pDec->pMbType[iMbXy];
	if(IS_DIRECT(mbType)){

		int16_t pMvDirect[LIST_A][2]={{0,0},{0,0}};
		SubMbType subMbType;
		if(pSliceHeader->iDirectSpatialMvPredFlag){
			// predict direct spatial mv
			int32_t ret=PredMvBDirectSpatial(pCtx,pMvDirect,iRef,subMbType);
			if(ret!=ERR_NONE){
				return ret;
			}
		}else{
			// temporal direct 16x16 mode
			int32_t ret=PredBDirectTemporal(pCtx,pMvDirect,iRef,subMbType);
			if(ret!=ERR_NONE){
				return ret;
			}
		}
	}else
	if(IS_INTER_16x16(mbType)){
		if(pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag){
			for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
				if(IS_DIR(mbType,0,listIdx)){
					WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// motion_prediction_flag_l0/l1[ mbPartIdx ]
					iMotionPredFlag[listIdx][0]=uiCode;
				}
			}
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(IS_DIR(mbType,0,listIdx)){
				if(iMotionPredFlag[listIdx][0]==0){
					WELS_READ_VERIFY(BsGetTe0(pBs,iRefCount[listIdx],&uiCode));		// motion_prediction_flag_l1[ mbPartIdx ]
					ref_idx_list[listIdx][0]=uiCode;
					// Security check: iRefIdx should be in range 0 to num_ref_idx_l0_active_minus1,includsive
					// ref to standard section 7.4.5.1. iRefCount[0] is 1+num_ref_idx_l0_active_minus1.
					if((ref_idx_list[listIdx][0]<0) || (ref_idx_list[listIdx][0]>=iRefCount[listIdx])
						 || (ppRefPic[listIdx][ref_idx_list[listIdx][0]]==NULL)){		// error ref_idx
						pCtx->bMbRefConcealed=true;
						if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
							ref_idx_list[listIdx][0]=0;
							pCtx->iErrorCode|=dsBitstreamError;
						}else{
							return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
						}
					}
					pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[listIdx][ref_idx_list[listIdx][0]] && (ppRefPic[listIdx][ref_idx_list[listIdx][0]]->bIsComplete || bIsPending));
				}else{
					uprintf("inter parse: iMotionPredFlag=1 not supported. ");
					return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
				}
			}
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(IS_DIR(mbType,0,listIdx)){
				PredMv(iMvArray,iRefIdxArray,listIdx,0,4,ref_idx_list[listIdx][0],iMv);
				WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l0[ mbPartIdx ][ 0 ][ compIdx ]
				iMv[0]+=iCode;
				WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l1[ mbPartIdx ][ 0 ][ compIdx ]
				iMv[1]+=iCode;
				WELS_CHECK_SE_BOTH_WARNING(iMv[1],iMinVmv,iMaxVmv,"vertical mv");
			}else{
				*(uint32_t*)iMv=0;
			}
			UpdateP16x16MotionInfo(pCurDqLayer,listIdx,ref_idx_list[listIdx][0],iMv);
		}
	}else
	if(IS_INTER_16x8(mbType)){
		if(pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag){
			for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
				for(int32_t i=0; i<2;++i){
					if(IS_DIR(mbType,i,listIdx)){
						WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// motion_prediction_flag_l0/l1[ mbPartIdx ]
						iMotionPredFlag[listIdx][i]=uiCode;
					}
				}
			}
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<2;++i){
				if(IS_DIR(mbType,i,listIdx)){
					if(iMotionPredFlag[listIdx][i]==0){
						WELS_READ_VERIFY(BsGetTe0(pBs,iRefCount[listIdx],&uiCode));		// motion_prediction_flag_l1[ mbPartIdx ]
						int32_t iRefIdx=uiCode;
						// Security check: iRefIdx should be in range 0 to num_ref_idx_l0_active_minus1,includsive
						// ref to standard section 7.4.5.1. iRefCount[0] is 1+num_ref_idx_l0_active_minus1.
						if((iRefIdx<0) || (iRefIdx>=iRefCount[listIdx]) || (ppRefPic[listIdx][iRefIdx]==NULL)){		// error ref_idx
							pCtx->bMbRefConcealed=true;
							if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
								iRefIdx=0;
								pCtx->iErrorCode|=dsBitstreamError;
							}else{
								return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
							}
						}
						ref_idx_list[listIdx][i]=iRefIdx;
						pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[listIdx][iRefIdx]
												 && (ppRefPic[listIdx][iRefIdx]->bIsComplete || bIsPending));
					}else{
						uprintf("inter parse: iMotionPredFlag=1 not supported. ");
						return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
					}
				}
			}
		}
		// Read mvd_L0 then mvd_L1
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			// Partitions
			for(int32_t i=0; i<2; i++){
				int iPartIdx=i<<3;
				int32_t iRefIdx=ref_idx_list[listIdx][i];
				if(IS_DIR(mbType,i,listIdx)){
					PredInter16x8Mv(iMvArray,iRefIdxArray,listIdx,iPartIdx,iRefIdx,iMv);

					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l{0,1}[ mbPartIdx ][ listIdx ][x]
					iMv[0]+=iCode;
					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l{0,1}[ mbPartIdx ][ listIdx ][y]
					iMv[1]+=iCode;

					WELS_CHECK_SE_BOTH_WARNING(iMv[1],iMinVmv,iMaxVmv,"vertical mv");
				}else{
					*(uint32_t*)iMv=0;
				}
				UpdateP16x8MotionInfo(pCurDqLayer,iMvArray,iRefIdxArray,listIdx,iPartIdx,iRefIdx,iMv);
			}
		}
	}else
	if(IS_INTER_8x16(mbType)){
		if(pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag){
			for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
				for(int32_t i=0; i<2;++i){
					if(IS_DIR(mbType,i,listIdx)){
						WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// motion_prediction_flag_l0/l1[ mbPartIdx ]
						iMotionPredFlag[listIdx][i]=uiCode;
					}
				}
			}
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<2;++i){
				if(IS_DIR(mbType,i,listIdx)){
					if(iMotionPredFlag[listIdx][i]==0){
						WELS_READ_VERIFY(BsGetTe0(pBs,iRefCount[listIdx],&uiCode));		// motion_prediction_flag_l1[ mbPartIdx ]
						int32_t iRefIdx=uiCode;
						// Security check: iRefIdx should be in range 0 to num_ref_idx_l0_active_minus1,includsive
						// ref to standard section 7.4.5.1. iRefCount[0] is 1+num_ref_idx_l0_active_minus1.
						if((iRefIdx<0) || (iRefIdx>=iRefCount[listIdx]) || (ppRefPic[listIdx][iRefIdx]==NULL)){		// error ref_idx
							pCtx->bMbRefConcealed=true;
							if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
								iRefIdx=0;
								pCtx->iErrorCode|=dsBitstreamError;
							}else{
								return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
							}
						}
						ref_idx_list[listIdx][i]=iRefIdx;
						pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[listIdx][iRefIdx]
												 && (ppRefPic[listIdx][iRefIdx]->bIsComplete || bIsPending));
					}else{
						uprintf("inter parse: iMotionPredFlag=1 not supported. ");
						return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
					}
				}
			}
		}
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<2; i++){
				int iPartIdx=i<<2;
				int32_t iRefIdx=ref_idx_list[listIdx][i];
				if(IS_DIR(mbType,i,listIdx)){
					PredInter8x16Mv(iMvArray,iRefIdxArray,listIdx,iPartIdx,iRefIdx,iMv);

					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l0[ mbPartIdx ][ 0 ][ compIdx ]
					iMv[0]+=iCode;
					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l1[ mbPartIdx ][ 0 ][ compIdx ]
					iMv[1]+=iCode;
					WELS_CHECK_SE_BOTH_WARNING(iMv[1],iMinVmv,iMaxVmv,"vertical mv");
				}else{
					*(uint32_t*)iMv=0;
				}
				UpdateP8x16MotionInfo(pCurDqLayer,iMvArray,iRefIdxArray,listIdx,iPartIdx,iRefIdx,iMv);
			}
		}
	}else
	if(IS_Inter_8x8(mbType)){
		int8_t pSubPartCount[4],pPartW[4];
		uint32_t uiSubMbType;
		// sub_mb_type,partition
		int16_t pMvDirect[LIST_A][2]={{0,0},{0,0}};
		if(pCtx->sRefPic.pRefList[LIST_1][0]==NULL){
			uprintf("Colocated Ref Picture for B-Slice is lost,B-Slice decoding cannot be continued!");
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_DATA,ERR_INFO_REFERENCE_PIC_LOST);
		}
		bool bIsLongRef=pCtx->sRefPic.pRefList[LIST_1][0]->bIsLongRef;
		const int32_t ref0Count=WELS_MIN(pSliceHeader->uiRefCount[LIST_0],pCtx->sRefPic.uiRefCount[LIST_0]);
		bool has_direct_called=false;
		SubMbType directSubMbType=0;

		// uiSubMbType,partition
		for(int32_t i=0; i<4; i++){
			WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// sub_mb_type[ mbPartIdx ]
			uiSubMbType=uiCode;
			if(uiSubMbType>=13){		// invalid uiSubMbType
				return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_SUB_MB_TYPE);
			}
			pSubPartCount[i]=g_ksInterBSubMbTypeInfo[uiSubMbType].iPartCount;
			pPartW[i]=g_ksInterBSubMbTypeInfo[uiSubMbType].iPartWidth;

			// Need modification when B picture add in,reference to 7.3.5
			if(pSubPartCount[i]>1)
				pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=false;

			if(IS_DIRECT(g_ksInterBSubMbTypeInfo[uiSubMbType].iType)){
				if(!has_direct_called){
					if(pSliceHeader->iDirectSpatialMvPredFlag){
						int32_t ret=PredMvBDirectSpatial(pCtx,pMvDirect,iRef,directSubMbType);
						if(ret!=ERR_NONE){
							return ret;
						}

					}else{
						// temporal direct mode
						int32_t ret=PredBDirectTemporal(pCtx,pMvDirect,iRef,directSubMbType);
						if(ret!=ERR_NONE){
							return ret;
						}
					}
					has_direct_called=true;
				}
				pCurDqLayer->pSubMbType[iMbXy][i]=directSubMbType;
				if(IS_SUB_4x4(pCurDqLayer->pSubMbType[iMbXy][i])){
					pSubPartCount[i]=4;
					pPartW[i]=1;
				}
			}else{
				pCurDqLayer->pSubMbType[iMbXy][i]=g_ksInterBSubMbTypeInfo[uiSubMbType].iType;
			}
		}
		if(pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag){
			for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
				for(int32_t i=0; i<4; i++){
					bool is_dir=IS_DIR(pCurDqLayer->pSubMbType[iMbXy][i],0,listIdx)>0;
					if(is_dir){
						WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// motion_prediction_flag_l0[ mbPartIdx ]
						iMotionPredFlag[listIdx][i]=uiCode;
					}
				}
			}
		}
		for(int32_t i=0; i<4; i++){		// Direct 8x8 Ref and mv
			int16_t iIdx8=i<<2;
			if(IS_DIRECT(pCurDqLayer->pSubMbType[iMbXy][i])){
				if(pSliceHeader->iDirectSpatialMvPredFlag){
					FillSpatialDirect8x8Mv(pCurDqLayer,iIdx8,pSubPartCount[i],pPartW[i],directSubMbType,bIsLongRef,pMvDirect,iRef,iMvArray,NULL);
				}else{
					int16_t(*mvColoc)[2]=pCurDqLayer->iColocMv[LIST_0];
					iRef[LIST_1]=0;
					iRef[LIST_0]=0;
					const uint8_t uiColoc4Idx=g_kuiScan4[iIdx8];
					if(!pCurDqLayer->iColocIntra[uiColoc4Idx]){
						iRef[LIST_0]=0;
						int8_t colocRefIndexL0=pCurDqLayer->iColocRefIndex[LIST_0][uiColoc4Idx];
						if(colocRefIndexL0>=0){
							iRef[LIST_0]=MapColToList0(pCtx,colocRefIndexL0,ref0Count);
						}else{
							mvColoc=pCurDqLayer->iColocMv[LIST_1];
						}
					}
					Update8x8RefIdx(pCurDqLayer,iIdx8,LIST_0,iRef[LIST_0]);
					Update8x8RefIdx(pCurDqLayer,iIdx8,LIST_1,iRef[LIST_1]);
					FillTemporalDirect8x8Mv(pCurDqLayer,iIdx8,pSubPartCount[i],pPartW[i],directSubMbType,iRef,mvColoc,iMvArray,NULL);
				}
			}
		}
		// ref no-direct
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<4; i++){
				int16_t iIdx8=i<<2;
				int32_t subMbType=pCurDqLayer->pSubMbType[iMbXy][i];
				int8_t iref=REF_NOT_IN_LIST;
				if(IS_DIRECT(subMbType)){
					if(pSliceHeader->iDirectSpatialMvPredFlag){
						Update8x8RefIdx(pCurDqLayer,iIdx8,listIdx,iRef[listIdx]);
						ref_idx_list[listIdx][i]=iRef[listIdx];
					}
				}else{
					if(IS_DIR(subMbType,0,listIdx)){
						if(iMotionPredFlag[listIdx][i]==0){
							WELS_READ_VERIFY(BsGetTe0(pBs,iRefCount[listIdx],&uiCode));		// ref_idx_l0[ mbPartIdx ]
							iref=uiCode;
							if((iref<0) || (iref>=iRefCount[listIdx]) || (ppRefPic[listIdx][iref]==NULL)){		// error ref_idx
								pCtx->bMbRefConcealed=true;
								if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
									iref=0;
									pCtx->iErrorCode|=dsBitstreamError;
								}else{
									return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_REF_INDEX);
								}
							}
							pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPic[listIdx][iref]
													 && (ppRefPic[listIdx][iref]->bIsComplete || bIsPending));
						}else{
							uprintf("inter parse: iMotionPredFlag=1 not supported. ");
							return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
						}
					}
					Update8x8RefIdx(pCurDqLayer,iIdx8,listIdx,iref);
					ref_idx_list[listIdx][i]=iref;
				}
			}
		}
		// mv
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			for(int32_t i=0; i<4; i++){
				int8_t iPartCount=pSubPartCount[i];
				int16_t iPartIdx,iBlockW=pPartW[i];
				uint8_t uiScan4Idx,uiCacheIdx;

				uiCacheIdx=g_kuiCache30ScanIdx[i<<2];

				int8_t iref=ref_idx_list[listIdx][i];
				iRefIdxArray[listIdx][uiCacheIdx]=iRefIdxArray[listIdx][uiCacheIdx+1]=
					iRefIdxArray[listIdx][uiCacheIdx+6]=iRefIdxArray[listIdx][uiCacheIdx+7]=iref;

				uint32_t subMbType=pCurDqLayer->pSubMbType[iMbXy][i];
				if(IS_DIRECT(subMbType)){
					continue;
				}
				bool is_dir=IS_DIR(subMbType,0,listIdx)>0;
				for(int32_t j=0; j<iPartCount; j++){
					iPartIdx=(i<<2)+j*iBlockW;
					uiScan4Idx=g_kuiScan4[iPartIdx];
					uiCacheIdx=g_kuiCache30ScanIdx[iPartIdx];
					if(is_dir){
						PredMv(iMvArray,iRefIdxArray,listIdx,iPartIdx,iBlockW,iref,iMv);

						WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l0[ mbPartIdx ][ subMbPartIdx ][ compIdx ]
						iMv[0]+=iCode;
						WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mvd_l1[ mbPartIdx ][ subMbPartIdx ][ compIdx ]
						iMv[1]+=iCode;
						WELS_CHECK_SE_BOTH_WARNING(iMv[1],iMinVmv,iMaxVmv,"vertical mv");
					}else{
						*(uint32_t*)iMv=0;
					}
					if(IS_SUB_8x8(subMbType)){		// MB_TYPE_8x8
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][uiScan4Idx],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][uiScan4Idx+1],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][uiScan4Idx+4],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][uiScan4Idx+5],LD32(iMv));
						ST32(iMvArray[listIdx][uiCacheIdx],LD32(iMv));
						ST32(iMvArray[listIdx][uiCacheIdx+1],LD32(iMv));
						ST32(iMvArray[listIdx][uiCacheIdx+6],LD32(iMv));
						ST32(iMvArray[listIdx][uiCacheIdx+7],LD32(iMv));
					}else
					if(IS_SUB_8x4(subMbType)){
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][uiScan4Idx],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][uiScan4Idx+1],LD32(iMv));
						ST32(iMvArray[listIdx][uiCacheIdx],LD32(iMv));
						ST32(iMvArray[listIdx][uiCacheIdx+1],LD32(iMv));
					}else
					if(IS_SUB_4x8(subMbType)){
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][uiScan4Idx],LD32(iMv));
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][uiScan4Idx+4],LD32(iMv));
						ST32(iMvArray[listIdx][uiCacheIdx],LD32(iMv));
						ST32(iMvArray[listIdx][uiCacheIdx+6],LD32(iMv));
					}else{		// SUB_MB_TYPE_4x4==uiSubMbType
						ST32(pCurDqLayer->pDec->pMv[listIdx][iMbXy][uiScan4Idx],LD32(iMv));
						ST32(iMvArray[listIdx][uiCacheIdx],LD32(iMv));
					}
				}
			}
		}
	}
	return ERR_NONE;
}

int32_t WelsActualDecodeMbCavlcBSlice(SDecoderContext* pCtx){
	SVlcTable* pVlcTable=pCtx->pVlcTable;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SBitStringAux* pBs=pCurDqLayer->pBitStringAux;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;

	int32_t iScanIdxStart=pSlice->sSliceHeaderExt.uiScanIdxStart;
	int32_t iScanIdxEnd=pSlice->sSliceHeaderExt.uiScanIdxEnd;

	SWelsNeighAvail sNeighAvail;
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;
	const int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int8_t* pNzc=pCurDqLayer->pNzc[iMbXy];
	int32_t i;
	int32_t iRet=ERR_NONE;
	uint32_t uiMbType=0,uiCbp=0,uiCbpL=0,uiCbpC=0;
	uint32_t uiCode;
	int32_t iCode;
	int32_t iMbResProperty;

	GetNeighborAvailMbType(&sNeighAvail,pCurDqLayer);
	ENFORCE_STACK_ALIGN_1D(uint8_t,pNonZeroCount,48,16);
	pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;	// 2009.10.23
	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// uiMbType
	uiMbType=uiCode;
	if(uiMbType<23){		// inter MB type
		int16_t iMotionVector[LIST_A][30][MV_A];
		int8_t iRefIndex[LIST_A][30];
		pCurDqLayer->pDec->pMbType[iMbXy]=g_ksInterBMbTypeInfo[uiMbType].iType;
		WelsFillCacheInter(&sNeighAvail,pNonZeroCount,iMotionVector,iRefIndex,pCurDqLayer);

		if((iRet=ParseInterBInfo(pCtx,iMotionVector,iRefIndex,pBs))!=ERR_NONE){
			return iRet;	// abnormal
		}

		if(pSlice->sSliceHeaderExt.bAdaptiveResidualPredFlag==1){
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// residual_prediction_flag
			pCurDqLayer->pResidualPredFlag[iMbXy]=uiCode;
		}else{
			pCurDqLayer->pResidualPredFlag[iMbXy]=pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;
		}

		if(pCurDqLayer->pResidualPredFlag[iMbXy]==0){
			pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
		}else{
			uprintf("residual_pred_flag=1 not supported.");
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_UNSUPPORTED_ILP);
		}
	}else{		// intra MB type
		uiMbType-=23;
		if(uiMbType>25)
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);
		if(!pCtx->pSps->uiChromaFormatIdc && ((uiMbType>=5 && uiMbType<=12) || (uiMbType>=17 && uiMbType<=24)))
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);

		if(25==uiMbType){
			uprintf("I_PCM mode exists in B slice!");
			int32_t iDecStrideL=pCurDqLayer->pDec->iLinesize[0];
			int32_t iDecStrideC=pCurDqLayer->pDec->iLinesize[1];

			int32_t iOffsetL=(iMbX+iMbY*iDecStrideL)<<4;
			int32_t iOffsetC=(iMbX+iMbY*iDecStrideC)<<3;

			uint8_t* pDecY=pCurDqLayer->pDec->pData[0]+iOffsetL;
			uint8_t* pDecU=pCurDqLayer->pDec->pData[1]+iOffsetC;
			uint8_t* pDecV=pCurDqLayer->pDec->pData[2]+iOffsetC;

			uint8_t* pTmpBsBuf;

			int32_t i;
			int32_t iCopySizeY=(sizeof(uint8_t)<<4);
			int32_t iCopySizeUV=(sizeof(uint8_t)<<3);

			int32_t iIndex=((-pBs->iLeftBits)>>3)+2;

			pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA_PCM;

			// step 1: locating bit-stream pointer [must align into integer byte]
			pBs->pCurBuf-=iIndex;

			// step 2: copy pixel from bit-stream into fdec [reconstruction]
			pTmpBsBuf=pBs->pCurBuf;
			for(i=0; i<16; i++){		// luma
				memcpy(pDecY,pTmpBsBuf,iCopySizeY);
				pDecY+=iDecStrideL;
				pTmpBsBuf+=16;
			}

			for(i=0; i<8; i++){		// cb
				memcpy(pDecU,pTmpBsBuf,iCopySizeUV);
				pDecU+=iDecStrideC;
				pTmpBsBuf+=8;
			}
			for(i=0; i<8; i++){		// cr
				memcpy(pDecV,pTmpBsBuf,iCopySizeUV);
				pDecV+=iDecStrideC;
				pTmpBsBuf+=8;
			}

			pBs->pCurBuf+=384;

			// step 3: update QP and pNonZeroCount
			pCurDqLayer->pLumaQp[iMbXy]=0;
			pCurDqLayer->pChromaQp[iMbXy][0]=pCurDqLayer->pChromaQp[iMbXy][1]=0;
			// Rec. 9.2.1 for PCM,nzc=16
			ST32A4(&pNzc[0],0x10101010);
			ST32A4(&pNzc[4],0x10101010);
			ST32A4(&pNzc[8],0x10101010);
			ST32A4(&pNzc[12],0x10101010);
			ST32A4(&pNzc[16],0x10101010);
			ST32A4(&pNzc[20],0x10101010);
			WELS_READ_VERIFY(InitReadBits(pBs,0));
			return ERR_NONE;
		}else{
			if(0==uiMbType){
				ENFORCE_STACK_ALIGN_1D(int8_t,pIntraPredMode,48,16);
				pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA4x4;
				if(pCtx->pPps->bTransform8x8ModeFlag){
					WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// transform_size_8x8_flag
					pCurDqLayer->pTransformSize8x8Flag[iMbXy]=!!uiCode;
					if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
						uiMbType=pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA8x8;
					}
				}
				if(!pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
					pCtx->pFillInfoCacheIntraNxNFunc(&sNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
					WELS_READ_VERIFY(ParseIntra4x4Mode(pCtx,&sNeighAvail,pIntraPredMode,pBs,pCurDqLayer));
				}else{
					pCtx->pFillInfoCacheIntraNxNFunc(&sNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
					WELS_READ_VERIFY(ParseIntra8x8Mode(pCtx,&sNeighAvail,pIntraPredMode,pBs,pCurDqLayer));
				}
			}else{		// I_PCM exclude,we can ignore it
				pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA16x16;
				pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;
				pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
				pCurDqLayer->pIntraPredMode[iMbXy][7]=(uiMbType-1)&3;
				pCurDqLayer->pCbp[iMbXy]=g_kuiI16CbpTable[(uiMbType-1)>>2];
				uiCbpC=pCtx->pSps->uiChromaFormatIdc ? pCurDqLayer->pCbp[iMbXy]>>4 : 0;
				uiCbpL=pCurDqLayer->pCbp[iMbXy]&15;
				WelsFillCacheNonZeroCount(&sNeighAvail,pNonZeroCount,pCurDqLayer);
				if((iRet=ParseIntra16x16Mode(pCtx,&sNeighAvail,pBs,pCurDqLayer))!=ERR_NONE){
					return iRet;
				}
			}
		}
	}

	if(MB_TYPE_INTRA16x16!=pCurDqLayer->pDec->pMbType[iMbXy]){
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// coded_block_pattern
		uiCbp=uiCode;
		{
			if(pCtx->pSps->uiChromaFormatIdc && (uiCbp>47))
				return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_CBP);
			if(!pCtx->pSps->uiChromaFormatIdc && (uiCbp>15))
				return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_CBP);
			if(MB_TYPE_INTRA4x4==pCurDqLayer->pDec->pMbType[iMbXy] || MB_TYPE_INTRA8x8==pCurDqLayer->pDec->pMbType[iMbXy]){

				uiCbp=pCtx->pSps->uiChromaFormatIdc ? g_kuiIntra4x4CbpTable[uiCbp] : g_kuiIntra4x4CbpTable400[uiCbp];
			}else		// inter
				uiCbp=pCtx->pSps->uiChromaFormatIdc ? g_kuiInterCbpTable[uiCbp] : g_kuiInterCbpTable400[uiCbp];
		}

		pCurDqLayer->pCbp[iMbXy]=uiCbp;
		uiCbpC=pCurDqLayer->pCbp[iMbXy]>>4;
		uiCbpL=pCurDqLayer->pCbp[iMbXy]&15;

		// Need modification when B picutre add in
		bool bNeedParseTransformSize8x8Flag=
			(((pCurDqLayer->pDec->pMbType[iMbXy]>=MB_TYPE_16x16 && pCurDqLayer->pDec->pMbType[iMbXy]<=MB_TYPE_8x16)
				 || pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy])
			  && (pCurDqLayer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA8x8)
			  && (pCurDqLayer->pDec->pMbType[iMbXy]!=MB_TYPE_INTRA4x4)
			  && (uiCbpL>0)
			  && (pCtx->pPps->bTransform8x8ModeFlag));

		if(bNeedParseTransformSize8x8Flag){
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// transform_size_8x8_flag
			pCurDqLayer->pTransformSize8x8Flag[iMbXy]=!!uiCode;
		}
	}

	ST32A4(&pNzc[0],0);
	ST32A4(&pNzc[4],0);
	ST32A4(&pNzc[8],0);
	ST32A4(&pNzc[12],0);
	ST32A4(&pNzc[16],0);
	ST32A4(&pNzc[20],0);
	if(pCurDqLayer->pCbp[iMbXy]==0 && !IS_INTRA16x16(pCurDqLayer->pDec->pMbType[iMbXy])
		 && !IS_I_BL(pCurDqLayer->pDec->pMbType[iMbXy])){
		pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pCurDqLayer->pLumaQp[iMbXy]+
											 pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}
	}

	if(pCurDqLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){
		int32_t iQpDelta,iId8x8,iId4x4;
		memset(pCurDqLayer->pScaledTCoeff[iMbXy],0,MB_COEFF_LIST_SIZE*sizeof(int16_t));
		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mb_qp_delta
		iQpDelta=iCode;

		if(iQpDelta>25 || iQpDelta<-26){		// out of iQpDelta range
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_QP);
		}

		pCurDqLayer->pLumaQp[iMbXy]=(pSlice->iLastMbQp+iQpDelta+52)%52;		// update last_mb_qp
		pSlice->iLastMbQp=pCurDqLayer->pLumaQp[iMbXy];
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pSlice->iLastMbQp+pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}

		BsStartCavlc(pBs);

		if(MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){
			// step1: Luma DC
			if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,0,16,g_kuiLumaDcZigzagScan,I16_LUMA_DC,pCurDqLayer->pScaledTCoeff[iMbXy],pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
				return iRet;	// abnormal
			}
			// step2: Luma AC
			if(uiCbpL){
				for(i=0; i<16; i++){
					if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,i,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),I16_LUMA_AC,pCurDqLayer->pScaledTCoeff[iMbXy]+(i<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
						return iRet;	// abnormal
					}
				}
				ST32A4(&pNzc[0],LD32(&pNonZeroCount[1+8*1]));
				ST32A4(&pNzc[4],LD32(&pNonZeroCount[1+8*2]));
				ST32A4(&pNzc[8],LD32(&pNonZeroCount[1+8*3]));
				ST32A4(&pNzc[12],LD32(&pNonZeroCount[1+8*4]));
			}
		}else{		// non-MB_TYPE_INTRA16x16
			if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
				for(iId8x8=0; iId8x8<4; iId8x8++){
					iMbResProperty=(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA_8 : LUMA_DC_AC_INTER_8;
					if(uiCbpL&(1<<iId8x8)){
						int32_t iIndex=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							if((iRet=WelsResidualBlockCavlc8x8(pVlcTable,pNonZeroCount,pBs,iIndex,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan8x8+iScanIdxStart,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iId8x8<<6),iId4x4,pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
								return iRet;
							}
							iIndex++;
						}
					}else{
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8<<2]],0);
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[(iId8x8<<2)+2]],0);
					}
				}
				ST32A4(&pNzc[0],LD32(&pNonZeroCount[1+8*1]));
				ST32A4(&pNzc[4],LD32(&pNonZeroCount[1+8*2]));
				ST32A4(&pNzc[8],LD32(&pNonZeroCount[1+8*3]));
				ST32A4(&pNzc[12],LD32(&pNonZeroCount[1+8*4]));
			}else{		// Normal T4x4
				for(iId8x8=0; iId8x8<4; iId8x8++){
					iMbResProperty=(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA : LUMA_DC_AC_INTER;
					if(uiCbpL&(1<<iId8x8)){
						int32_t iIndex=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							// Luma (DC and AC decoding together)
							if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,iIndex,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan+iScanIdxStart,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIndex<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
								return iRet;	// abnormal
							}
							iIndex++;
						}
					}else{
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8<<2]],0);
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[(iId8x8<<2)+2]],0);
					}
				}
				ST32A4(&pNzc[0],LD32(&pNonZeroCount[1+8*1]));
				ST32A4(&pNzc[4],LD32(&pNonZeroCount[1+8*2]));
				ST32A4(&pNzc[8],LD32(&pNonZeroCount[1+8*3]));
				ST32A4(&pNzc[12],LD32(&pNonZeroCount[1+8*4]));
			}
		}


		// chroma
		// step1: DC
		if(1==uiCbpC || 2==uiCbpC){
			for(i=0; i<2; i++){		// Cb Cr
				if(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_DC_V : CHROMA_DC_U;
				else
					iMbResProperty=i ? CHROMA_DC_V_INTER : CHROMA_DC_U_INTER;

				if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,16+(i<<2),4,g_kuiChromaDcScan,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+256+(i<<6),pCurDqLayer->pChromaQp[iMbXy][i],pCtx))!=ERR_NONE){
					return iRet;	// abnormal
				}
			}
		}else{
		}
		// step2: AC
		if(2==uiCbpC){
			for(i=0; i<2; i++){		// Cb Cr
				if(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy]))
					iMbResProperty=i ? CHROMA_AC_V : CHROMA_AC_U;
				else
					iMbResProperty=i ? CHROMA_AC_V_INTER : CHROMA_AC_U_INTER;

				int32_t iIndex=16+(i<<2);
				for(iId4x4=0; iId4x4<4; iId4x4++){
					if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,iIndex,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIndex<<4),pCurDqLayer->pChromaQp[iMbXy][i],pCtx))!=ERR_NONE){
						return iRet;	// abnormal
					}
					iIndex++;
				}
			}
			ST16A2(&pNzc[16],LD16A2(&pNonZeroCount[6+8*1]));
			ST16A2(&pNzc[20],LD16A2(&pNonZeroCount[6+8*2]));
			ST16A2(&pNzc[18],LD16A2(&pNonZeroCount[6+8*4]));
			ST16A2(&pNzc[22],LD16A2(&pNonZeroCount[6+8*5]));
		}
		BsEndCavlc(pBs);
	}

	return ERR_NONE;
}

int32_t WelsActualDecodeMbCavlcISlice(SDecoderContext* pCtx){
	SVlcTable* pVlcTable=pCtx->pVlcTable;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SBitStringAux* pBs=pCurDqLayer->pBitStringAux;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;

	SWelsNeighAvail sNeighAvail;
	int32_t iMbResProperty;

	int32_t iScanIdxStart=pSlice->sSliceHeaderExt.uiScanIdxStart;
	int32_t iScanIdxEnd=pSlice->sSliceHeaderExt.uiScanIdxEnd;

	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;
	const int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int8_t* pNzc=pCurDqLayer->pNzc[iMbXy];
	int32_t i;
	int32_t iRet=ERR_NONE;
	uint32_t uiMbType=0,uiCbp=0,uiCbpL=0,uiCbpC=0;
	uint32_t uiCode;
	int32_t iCode;

	ENFORCE_STACK_ALIGN_1D(uint8_t,pNonZeroCount,48,16);
	GetNeighborAvailMbType(&sNeighAvail,pCurDqLayer);
	pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
	pCurDqLayer->pResidualPredFlag[iMbXy]=pSlice->sSliceHeaderExt.bDefaultResidualPredFlag;

	pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
	pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;

	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// uiMbType
	uiMbType=uiCode;
	if(uiMbType>25)
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);
	if(!pCtx->pSps->uiChromaFormatIdc && ((uiMbType>=5 && uiMbType<=12) || (uiMbType>=17 && uiMbType<=24)))
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_TYPE);

	if(25==uiMbType){
		uprintf("I_PCM mode exists in I slice!");
		int32_t iDecStrideL=pCurDqLayer->pDec->iLinesize[0];
		int32_t iDecStrideC=pCurDqLayer->pDec->iLinesize[1];

		int32_t iOffsetL=(iMbX+iMbY*iDecStrideL)<<4;
		int32_t iOffsetC=(iMbX+iMbY*iDecStrideC)<<3;

		uint8_t* pDecY=pCurDqLayer->pDec->pData[0]+iOffsetL;
		uint8_t* pDecU=pCurDqLayer->pDec->pData[1]+iOffsetC;
		uint8_t* pDecV=pCurDqLayer->pDec->pData[2]+iOffsetC;

		uint8_t* pTmpBsBuf;


		int32_t i;
		int32_t iCopySizeY=(sizeof(uint8_t)<<4);
		int32_t iCopySizeUV=(sizeof(uint8_t)<<3);

		int32_t iIndex=((-pBs->iLeftBits)>>3)+2;

		pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA_PCM;

		// step 1: locating bit-stream pointer [must align into integer byte]
		pBs->pCurBuf-=iIndex;

		// step 2: copy pixel from bit-stream into fdec [reconstruction]
		pTmpBsBuf=pBs->pCurBuf;
		for(i=0; i<16; i++){		// luma
			memcpy(pDecY,pTmpBsBuf,iCopySizeY);
			pDecY+=iDecStrideL;
			pTmpBsBuf+=16;
		}
		for(i=0; i<8; i++){		// cb
			memcpy(pDecU,pTmpBsBuf,iCopySizeUV);
			pDecU+=iDecStrideC;
			pTmpBsBuf+=8;
		}
		for(i=0; i<8; i++){		// cr
			memcpy(pDecV,pTmpBsBuf,iCopySizeUV);
			pDecV+=iDecStrideC;
			pTmpBsBuf+=8;
		}

		pBs->pCurBuf+=384;

		// step 3: update QP and pNonZeroCount
		pCurDqLayer->pLumaQp[iMbXy]=0;
		memset(pCurDqLayer->pChromaQp[iMbXy],0,sizeof(pCurDqLayer->pChromaQp[iMbXy]));
		memset(pNzc,16,sizeof(pCurDqLayer->pNzc[iMbXy]));		// Rec. 9.2.1 for PCM,nzc=16
		WELS_READ_VERIFY(InitReadBits(pBs,0));
		return ERR_NONE;
	}else
	if(0==uiMbType){		// reference to JM
		ENFORCE_STACK_ALIGN_1D(int8_t,pIntraPredMode,48,16);
		pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA4x4;
		if(pCtx->pPps->bTransform8x8ModeFlag){
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// transform_size_8x8_flag
			pCurDqLayer->pTransformSize8x8Flag[iMbXy]=!!uiCode;
			if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
				uiMbType=pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA8x8;
			}
		}
		if(!pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
			pCtx->pFillInfoCacheIntraNxNFunc(&sNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
			WELS_READ_VERIFY(ParseIntra4x4Mode(pCtx,&sNeighAvail,pIntraPredMode,pBs,pCurDqLayer));
		}else{
			pCtx->pFillInfoCacheIntraNxNFunc(&sNeighAvail,pNonZeroCount,pIntraPredMode,pCurDqLayer);
			WELS_READ_VERIFY(ParseIntra8x8Mode(pCtx,&sNeighAvail,pIntraPredMode,pBs,pCurDqLayer));
		}

		// uiCbp
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// coded_block_pattern
		uiCbp=uiCode;
		// G.9.1 Alternative parsing process for coded pBlock pattern
		if(pCtx->pSps->uiChromaFormatIdc && (uiCbp>47))
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_CBP);
		if(!pCtx->pSps->uiChromaFormatIdc && (uiCbp>15))
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_CBP);

		if(pCtx->pSps->uiChromaFormatIdc)
			uiCbp=g_kuiIntra4x4CbpTable[uiCbp];
		else
			uiCbp=g_kuiIntra4x4CbpTable400[uiCbp];
		pCurDqLayer->pCbp[iMbXy]=uiCbp;
		uiCbpC=uiCbp>>4;
		uiCbpL=uiCbp&15;
	}else{		// I_PCM exclude,we can ignore it
		pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_INTRA16x16;
		pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;
		pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
		pCurDqLayer->pIntraPredMode[iMbXy][7]=(uiMbType-1)&3;
		pCurDqLayer->pCbp[iMbXy]=g_kuiI16CbpTable[(uiMbType-1)>>2];
		uiCbpC=pCtx->pSps->uiChromaFormatIdc ? pCurDqLayer->pCbp[iMbXy]>>4 : 0;
		uiCbpL=pCurDqLayer->pCbp[iMbXy]&15;
		WelsFillCacheNonZeroCount(&sNeighAvail,pNonZeroCount,pCurDqLayer);
		WELS_READ_VERIFY(ParseIntra16x16Mode(pCtx,&sNeighAvail,pBs,pCurDqLayer));
	}

	ST32A4(&pNzc[0],0);
	ST32A4(&pNzc[4],0);
	ST32A4(&pNzc[8],0);
	ST32A4(&pNzc[12],0);
	ST32A4(&pNzc[16],0);
	ST32A4(&pNzc[20],0);

	if(pCurDqLayer->pCbp[iMbXy]==0 && IS_INTRANxN(pCurDqLayer->pDec->pMbType[iMbXy])){
		pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pCurDqLayer->pLumaQp[iMbXy]+pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}

	}

	if(pCurDqLayer->pCbp[iMbXy] || MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){

		memset(pCurDqLayer->pScaledTCoeff[iMbXy],0,384*sizeof(pCurDqLayer->pScaledTCoeff[iMbXy][0]));
		int32_t iQpDelta,iId8x8,iId4x4;

		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// mb_qp_delta
		iQpDelta=iCode;

		if(iQpDelta>25 || iQpDelta<-26){		// out of iQpDelta range
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_QP);
		}

		pCurDqLayer->pLumaQp[iMbXy]=(pSlice->iLastMbQp+iQpDelta+52)%52;		// update last_mb_qp
		pSlice->iLastMbQp=pCurDqLayer->pLumaQp[iMbXy];
		for(i=0; i<2; i++){
			pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pSlice->iLastMbQp+pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
		}
		BsStartCavlc(pBs);

		if(MB_TYPE_INTRA16x16==pCurDqLayer->pDec->pMbType[iMbXy]){
			// step1: Luma DC
			if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,0,16,g_kuiLumaDcZigzagScan,I16_LUMA_DC,pCurDqLayer->pScaledTCoeff[iMbXy],pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
				return iRet;	// abnormal
			}
			// step2: Luma AC
			if(uiCbpL){
				for(i=0; i<16; i++){
					if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,i,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),I16_LUMA_AC,pCurDqLayer->pScaledTCoeff[iMbXy]+(i<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
						return iRet;	// abnormal
					}
				}
				ST32A4(&pNzc[0],LD32(&pNonZeroCount[1+8*1]));
				ST32A4(&pNzc[4],LD32(&pNonZeroCount[1+8*2]));
				ST32A4(&pNzc[8],LD32(&pNonZeroCount[1+8*3]));
				ST32A4(&pNzc[12],LD32(&pNonZeroCount[1+8*4]));
			}
		}else{		// non-MB_TYPE_INTRA16x16
			if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
				for(iId8x8=0; iId8x8<4; iId8x8++){
					iMbResProperty=(IS_INTRA(pCurDqLayer->pDec->pMbType[iMbXy])) ? LUMA_DC_AC_INTRA_8 : LUMA_DC_AC_INTER_8;
					if(uiCbpL&(1<<iId8x8)){
						int32_t iIndex=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							if((iRet=WelsResidualBlockCavlc8x8(pVlcTable,pNonZeroCount,pBs,iIndex,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan8x8+iScanIdxStart,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iId8x8<<6),iId4x4,pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
								return iRet;
							}
							iIndex++;
						}
					}else{
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[iId8x8<<2]],0);
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[(iId8x8<<2)+2]],0);
					}
				}
				ST32A4(&pNzc[0],LD32(&pNonZeroCount[1+8*1]));
				ST32A4(&pNzc[4],LD32(&pNonZeroCount[1+8*2]));
				ST32A4(&pNzc[8],LD32(&pNonZeroCount[1+8*3]));
				ST32A4(&pNzc[12],LD32(&pNonZeroCount[1+8*4]));
			}else{
				for(iId8x8=0; iId8x8<4; iId8x8++){
					if(uiCbpL&(1<<iId8x8)){
						int32_t iIndex=(iId8x8<<2);
						for(iId4x4=0; iId4x4<4; iId4x4++){
							// Luma (DC and AC decoding together)
							if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,iIndex,iScanIdxEnd-iScanIdxStart+1,g_kuiZigzagScan+iScanIdxStart,LUMA_DC_AC_INTRA,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIndex<<4),pCurDqLayer->pLumaQp[iMbXy],pCtx))!=ERR_NONE){
								return iRet;	// abnormal
							}
							iIndex++;
						}
					}else{
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[(iId8x8<<2)]],0);
						ST16(&pNonZeroCount[g_kuiCache48CountScan4Idx[(iId8x8<<2)+2]],0);
					}
				}
				ST32A4(&pNzc[0],LD32(&pNonZeroCount[1+8*1]));
				ST32A4(&pNzc[4],LD32(&pNonZeroCount[1+8*2]));
				ST32A4(&pNzc[8],LD32(&pNonZeroCount[1+8*3]));
				ST32A4(&pNzc[12],LD32(&pNonZeroCount[1+8*4]));
			}
		}

		// chroma
		// step1: DC
		if(1==uiCbpC || 2==uiCbpC){
			for(i=0; i<2; i++){		// Cb Cr
				iMbResProperty=i ? CHROMA_DC_V : CHROMA_DC_U;
				if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,16+(i<<2),4,g_kuiChromaDcScan,iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+256+(i<<6),pCurDqLayer->pChromaQp[iMbXy][i],pCtx))!=ERR_NONE){
					return iRet;	// abnormal
				}
			}
		}
		// step2: AC
		if(2==uiCbpC){
			for(i=0; i<2; i++){		// Cb Cr
				iMbResProperty=i ? CHROMA_AC_V : CHROMA_AC_U;
				int32_t iIndex=16+(i<<2);
				for(iId4x4=0; iId4x4<4; iId4x4++){
					if((iRet=WelsResidualBlockCavlc(pVlcTable,pNonZeroCount,pBs,iIndex,iScanIdxEnd-WELS_MAX(iScanIdxStart,1)+1,g_kuiZigzagScan+WELS_MAX(iScanIdxStart,1),iMbResProperty,pCurDqLayer->pScaledTCoeff[iMbXy]+(iIndex<<4),pCurDqLayer->pChromaQp[iMbXy][i],pCtx))!=ERR_NONE){
						return iRet;	// abnormal
					}
					iIndex++;
				}
			}
			ST16A2(&pNzc[16],LD16A2(&pNonZeroCount[6+8*1]));
			ST16A2(&pNzc[20],LD16A2(&pNonZeroCount[6+8*2]));
			ST16A2(&pNzc[18],LD16A2(&pNonZeroCount[6+8*4]));
			ST16A2(&pNzc[22],LD16A2(&pNonZeroCount[6+8*5]));
		}
		BsEndCavlc(pBs);
	}

	return ERR_NONE;
}

int32_t WelsDecodeMbCavlcBSlice(SDecoderContext* pCtx,SNalUnit* pNalCur,uint32_t& uiEosFlag){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SBitStringAux* pBs=pCurDqLayer->pBitStringAux;
	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	SPicture** ppRefPicL0=pCtx->sRefPic.pRefList[LIST_0];
	SPicture** ppRefPicL1=pCtx->sRefPic.pRefList[LIST_1];
	intX_t iUsedBits;
	const int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int8_t* pNzc=pCurDqLayer->pNzc[iMbXy];
	int32_t iBaseModeFlag,i;
	int32_t iRet=0;		// should have the return value to indicate decoding error or not,It's NECESSARY--2010.4.15
	uint32_t uiCode;

	pCurDqLayer->pNoSubMbPartSizeLessThan8x8Flag[iMbXy]=true;
	pCurDqLayer->pTransformSize8x8Flag[iMbXy]=false;

	if(-1==pSlice->iMbSkipRun){
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// mb_skip_run
		pSlice->iMbSkipRun=uiCode;
		if(-1==pSlice->iMbSkipRun){
			return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_INVALID_MB_SKIP_RUN);
		}
	}
	if(pSlice->iMbSkipRun--){
		int16_t iMv[LIST_A][2]={{0,0},{0,0}};
		int8_t ref[LIST_A]={0};

		pCurDqLayer->pDec->pMbType[iMbXy]=MB_TYPE_SKIP|MB_TYPE_DIRECT;
		ST32A4(&pNzc[0],0);
		ST32A4(&pNzc[4],0);
		ST32A4(&pNzc[8],0);
		ST32A4(&pNzc[12],0);
		ST32A4(&pNzc[16],0);
		ST32A4(&pNzc[20],0);

		pCurDqLayer->pInterPredictionDoneFlag[iMbXy]=0;
		memset(pCurDqLayer->pDec->pRefIndex[LIST_0][iMbXy],0,sizeof(int8_t)*16);
		memset(pCurDqLayer->pDec->pRefIndex[LIST_1][iMbXy],0,sizeof(int8_t)*16);
		bool bIsPending=false;	// GetThreadCount (pCtx) > 1;
		pCtx->bMbRefConcealed=pCtx->bRPLRError || pCtx->bMbRefConcealed || !(ppRefPicL0[0] && (ppRefPicL0[0]->bIsComplete || bIsPending)) || !(ppRefPicL1[0] && (ppRefPicL1[0]->bIsComplete || bIsPending));

		// predict iMv
		SubMbType subMbType;
		if(pSliceHeader->iDirectSpatialMvPredFlag){

			// predict direct spatial mv
			int32_t ret=PredMvBDirectSpatial(pCtx,iMv,ref,subMbType);
			if(ret!=ERR_NONE){
				return ret;
			}
		}else{
			// temporal direct mode
			int32_t ret=PredBDirectTemporal(pCtx,iMv,ref,subMbType);
			if(ret!=ERR_NONE){
				return ret;
			}
		}

		// if (!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag) {
		// memset (pCurDqLayer->pScaledTCoeff[iMbXy],0,384 * sizeof (int16_t));
		// }

		// reset rS
		if(!pSlice->sSliceHeaderExt.bDefaultResidualPredFlag || 
			(pNalCur->sNalHeaderExt.uiQualityId==0 && pNalCur->sNalHeaderExt.uiDependencyId==0)){
			pCurDqLayer->pLumaQp[iMbXy]=pSlice->iLastMbQp;
			for(i=0; i<2; i++){
				pCurDqLayer->pChromaQp[iMbXy][i]=g_kuiChromaQpTable[WELS_CLIP3(pCurDqLayer->pLumaQp[iMbXy]+pSliceHeader->pPps->iChromaQpIndexOffset[i],0,51)];
			}
		}

		pCurDqLayer->pCbp[iMbXy]=0;
	}else{
		if(pSlice->sSliceHeaderExt.bAdaptiveBaseModeFlag==1){
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// base_mode_flag
			iBaseModeFlag=uiCode;
		}else{
			iBaseModeFlag=pSlice->sSliceHeaderExt.bDefaultBaseModeFlag;
		}
		if(!iBaseModeFlag){
			iRet=WelsActualDecodeMbCavlcBSlice(pCtx);
		}else{
			uprintf("iBaseModeFlag (%d) !=0,inter-layer prediction not supported.",iBaseModeFlag);
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_ILP);
		}
		if(iRet){		// occur error when parsing,MUST STOP decoding
			return iRet;
		}
	}
	// check whether there is left bits to read next time in case multiple slices
	iUsedBits=((pBs->pCurBuf-pBs->pStartBuf)<<3)-(16-pBs->iLeftBits);
	// sub 1,for stop bit
	if((iUsedBits==(pBs->iBits-1)) && (0>=pCurDqLayer->sLayerInfo.sSliceInLayer.iMbSkipRun)){		// slice boundary
		uiEosFlag=1;
	}
	if(iUsedBits>(pBs->iBits-1)){		// When BS incomplete,as long as find it,SHOULD stop decoding to avoid mosaic or crash.
		uprintf("WelsDecodeMbCavlcBSlice()::::pBs incomplete,iUsedBits:%llu %d,MUST stop decoding.",(int64_t)iUsedBits,pBs->iBits);
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_BS_INCOMPLETE);
	}
	return ERR_NONE;
}

int32_t WelsDecodeMbCavlcISlice(SDecoderContext* pCtx,SNalUnit* pNalCur,uint32_t& uiEosFlag){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SBitStringAux* pBs=pCurDqLayer->pBitStringAux;
	SSliceHeaderExt* pSliceHeaderExt=&pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt;
	int32_t iBaseModeFlag;
	int32_t iRet=0;		// should have the return value to indicate decoding error or not,It's NECESSARY--2010.4.15
	uint32_t uiCode;
	intX_t iUsedBits;
	if(pSliceHeaderExt->bAdaptiveBaseModeFlag==1){
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// base_mode_flag
		iBaseModeFlag=uiCode;
	}else{
		iBaseModeFlag=pSliceHeaderExt->bDefaultBaseModeFlag;
	}
	if(!iBaseModeFlag){
		iRet=WelsActualDecodeMbCavlcISlice(pCtx);
	}else{
		uprintf("iBaseModeFlag (%d) !=0,inter-layer prediction not supported.",iBaseModeFlag);
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_ILP);
	}
	if(iRet){		// occur error when parsing,MUST STOP decoding
		return iRet;
	}

	// check whether there is left bits to read next time in case multiple slices
	iUsedBits=((pBs->pCurBuf-pBs->pStartBuf)<<3)-(16-pBs->iLeftBits);
	// sub 1,for stop bit
	if((iUsedBits==(pBs->iBits-1)) && (0>=pCurDqLayer->sLayerInfo.sSliceInLayer.iMbSkipRun)){		// slice boundary
		uiEosFlag=1;
	}
	if(iUsedBits>(pBs->iBits-
		1)){		// When BS incomplete,as long as find it,SHOULD stop decoding to avoid mosaic or crash.
		uprintf("WelsDecodeMbCavlcISlice()::::pBs incomplete,iUsedBits:%llu %d,MUST stop decoding.",(int64_t)iUsedBits,pBs->iBits);
		return GENERATE_ERROR_NO(ERR_LEVEL_MB_DATA,ERR_INFO_BS_INCOMPLETE);
	}
	return ERR_NONE;
}

void WelsFillCacheConstrain1IntraNxN(PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCount,int8_t* pIntraPredMode,PDqLayer pCurDqLayer){		// no matter slice type
	int32_t iCurXy=pCurDqLayer->iMbXyIndex;
	int32_t iTopXy=0;
	int32_t iLeftXy=0;

	// stuff non_zero_coeff_count from pNeighAvail(left and top)
	WelsFillCacheNonZeroCount(pNeighAvail,pNonZeroCount,pCurDqLayer);

	if(pNeighAvail->iTopAvail){
		iTopXy=iCurXy-pCurDqLayer->iMbWidth;
	}
	if(pNeighAvail->iLeftAvail){
		iLeftXy=iCurXy-1;
	}

	// intraNxN_pred_mode
	if(pNeighAvail->iTopAvail && IS_INTRANxN(pNeighAvail->iTopType)){		// top
		ST32(pIntraPredMode+1,LD32(&pCurDqLayer->pIntraPredMode[iTopXy][0]));
	}else{
		int32_t iPred;
		if(IS_INTRA16x16(pNeighAvail->iTopType) || (MB_TYPE_INTRA_PCM==pNeighAvail->iTopType))
			iPred=0x02020202;
		else
			iPred=0xffffffff;
		ST32(pIntraPredMode+1,iPred);
	}

	if(pNeighAvail->iLeftAvail && IS_INTRANxN(pNeighAvail->iLeftType)){		// left
		pIntraPredMode[0+8]=pCurDqLayer->pIntraPredMode[iLeftXy][4];
		pIntraPredMode[0+8*2]=pCurDqLayer->pIntraPredMode[iLeftXy][5];
		pIntraPredMode[0+8*3]=pCurDqLayer->pIntraPredMode[iLeftXy][6];
		pIntraPredMode[0+8*4]=pCurDqLayer->pIntraPredMode[iLeftXy][3];
	}else{
		int8_t iPred;
		if(IS_INTRA16x16(pNeighAvail->iLeftType) || (MB_TYPE_INTRA_PCM==pNeighAvail->iLeftType))
			iPred=2;
		else
			iPred=-1;
		pIntraPredMode[0+8]=
			pIntraPredMode[0+8*2]=
			pIntraPredMode[0+8*3]=
			pIntraPredMode[0+8*4]=iPred;
	}
}

void WelsMapNxNNeighToSampleConstrain1(PWelsNeighAvail pNeighAvail,int32_t* pSampleAvail){
	if(pNeighAvail->iLeftAvail && IS_INTRA(pNeighAvail->iLeftType)){		// left
		pSampleAvail[6]=
			pSampleAvail[12]=
			pSampleAvail[18]=
			pSampleAvail[24]=1;
	}
	if(pNeighAvail->iLeftTopAvail && IS_INTRA(pNeighAvail->iLeftTopType)){		// top_left
		pSampleAvail[0]=1;
	}
	if(pNeighAvail->iTopAvail && IS_INTRA(pNeighAvail->iTopType)){		// top
		pSampleAvail[1]=
			pSampleAvail[2]=
			pSampleAvail[3]=
			pSampleAvail[4]=1;
	}
	if(pNeighAvail->iRightTopAvail && IS_INTRA(pNeighAvail->iRightTopType)){		// top_right
		pSampleAvail[5]=1;
	}
}
void WelsMap16x16NeighToSampleConstrain1(PWelsNeighAvail pNeighAvail,uint8_t* pSampleAvail){
	if(pNeighAvail->iLeftAvail && IS_INTRA(pNeighAvail->iLeftType)){
		*pSampleAvail=(1<<2);
	}
	if(pNeighAvail->iLeftTopAvail && IS_INTRA(pNeighAvail->iLeftTopType)){
		*pSampleAvail|=(1<<1);
	}
	if(pNeighAvail->iTopAvail && IS_INTRA(pNeighAvail->iTopType)){
		*pSampleAvail|=1;
	}
}

void WelsFillCacheConstrain0IntraNxN(PWelsNeighAvail pNeighAvail,uint8_t* pNonZeroCount,int8_t* pIntraPredMode,PDqLayer pCurDqLayer){		// no matter slice type
	int32_t iCurXy=pCurDqLayer->iMbXyIndex;
	int32_t iTopXy=0;
	int32_t iLeftXy=0;

	// stuff non_zero_coeff_count from pNeighAvail(left and top)
	WelsFillCacheNonZeroCount(pNeighAvail,pNonZeroCount,pCurDqLayer);

	if(pNeighAvail->iTopAvail){
		iTopXy=iCurXy-pCurDqLayer->iMbWidth;
	}
	if(pNeighAvail->iLeftAvail){
		iLeftXy=iCurXy-1;
	}

	// intra4x4_pred_mode
	if(pNeighAvail->iTopAvail && IS_INTRANxN(pNeighAvail->iTopType)){		// top
		ST32(pIntraPredMode+1,LD32(&pCurDqLayer->pIntraPredMode[iTopXy][0]));
	}else{
		int32_t iPred;
		if(pNeighAvail->iTopAvail)
			iPred=0x02020202;
		else
			iPred=0xffffffff;
		ST32(pIntraPredMode+1,iPred);
	}

	if(pNeighAvail->iLeftAvail && IS_INTRANxN(pNeighAvail->iLeftType)){		// left
		pIntraPredMode[0+8*1]=pCurDqLayer->pIntraPredMode[iLeftXy][4];
		pIntraPredMode[0+8*2]=pCurDqLayer->pIntraPredMode[iLeftXy][5];
		pIntraPredMode[0+8*3]=pCurDqLayer->pIntraPredMode[iLeftXy][6];
		pIntraPredMode[0+8*4]=pCurDqLayer->pIntraPredMode[iLeftXy][3];
	}else{
		int8_t iPred;
		if(pNeighAvail->iLeftAvail)
			iPred=2;
		else
			iPred=-1;
		pIntraPredMode[0+8*1]=
			pIntraPredMode[0+8*2]=
			pIntraPredMode[0+8*3]=
			pIntraPredMode[0+8*4]=iPred;
	}
}

void WelsMapNxNNeighToSampleNormal(PWelsNeighAvail pNeighAvail,int32_t* pSampleAvail){
	if(pNeighAvail->iLeftAvail){		// left
		pSampleAvail[6]=
			pSampleAvail[12]=
			pSampleAvail[18]=
			pSampleAvail[24]=1;
	}
	if(pNeighAvail->iLeftTopAvail){		// top_left
		pSampleAvail[0]=1;
	}
	if(pNeighAvail->iTopAvail){		// top
		pSampleAvail[1]=
			pSampleAvail[2]=
			pSampleAvail[3]=
			pSampleAvail[4]=1;
	}
	if(pNeighAvail->iRightTopAvail){		// top_right
		pSampleAvail[5]=1;
	}
}
void WelsMap16x16NeighToSampleNormal(PWelsNeighAvail pNeighAvail,uint8_t* pSampleAvail){
	if(pNeighAvail->iLeftAvail){
		*pSampleAvail=(1<<2);
	}
	if(pNeighAvail->iLeftTopAvail){
		*pSampleAvail|=(1<<1);
	}
	if(pNeighAvail->iTopAvail){
		*pSampleAvail|=1;
	}
}

const int8_t g_kiCabacGlobalContextIdx[WELS_CONTEXT_COUNT][4][2]={
	// 0-10 Table 9-12
	{{20,-15},{20,-15},{20,-15},{20,-15}},
 {{2,54},{2,54},{2,54},{2,54}},
 {{3,74},{3,74},{3,74},{3,74}},
 {{20,-15},{20,-15},{20,-15},{20,-15}},
 {{2,54},{2,54},{2,54},{2,54}},
 {{3,74},{3,74},{3,74},{3,74}},
 {{-28,127},{-28,127},{-28,127},{-28,127}},
 {{-23,104},{-23,104},{-23,104},{-23,104}},
 {{-6,53},{-6,53},{-6,53},{-6,53}},
 {{-1,54},{-1,54},{-1,54},{-1,54}},
 {{7,51},{7,51},{7,51},{7,51}},
		// 11-23 Table 9-13
 {{CTX_NA,CTX_NA},{23,33},{22,25},{29,16}},
 {{CTX_NA,CTX_NA},{23,2},{34,0},{25,0}},
 {{CTX_NA,CTX_NA},{21,0},{16,0},{14,0}},
 {{CTX_NA,CTX_NA},{1,9},{-2,9},{-10,51}},
 {{CTX_NA,CTX_NA},{0,49},{4,41},{-3,62}},
 {{CTX_NA,CTX_NA},{-37,118},{-29,118},{-27,99}},
 {{CTX_NA,CTX_NA},{5,57},{2,65},{26,16}},
 {{CTX_NA,CTX_NA},{-13,78},{-6,71},{-4,85}},
 {{CTX_NA,CTX_NA},{-11,65},{-13,79},{-24,102}},
 {{CTX_NA,CTX_NA},{1,62},{5,52},{5,57}},
 {{CTX_NA,CTX_NA},{12,49},{9,50},{6,57}},
 {{CTX_NA,CTX_NA},{-4,73},{-3,70},{-17,73}},
 {{CTX_NA,CTX_NA},{17,50},{10,54},{14,57}},
		// 24-39 Table9-14
 {{CTX_NA,CTX_NA},{18,64},{26,34},{20,40}},
 {{CTX_NA,CTX_NA},{9,43},{19,22},{20,10}},
 {{CTX_NA,CTX_NA},{29,0},{40,0},{29,0}},
 {{CTX_NA,CTX_NA},{26,67},{57,2},{54,0}},
 {{CTX_NA,CTX_NA},{16,90},{41,36},{37,42}},
 {{CTX_NA,CTX_NA},{9,104},{26,69},{12,97}},
 {{CTX_NA,CTX_NA},{-46,127},{-45,127},{-32,127}},
 {{CTX_NA,CTX_NA},{-20,104},{-15,101},{-22,117}},
 {{CTX_NA,CTX_NA},{1,67},{-4,76},{-2,74}},
 {{CTX_NA,CTX_NA},{-13,78},{-6,71},{-4,85}},
 {{CTX_NA,CTX_NA},{-11,65},{-13,79},{-24,102}},
 {{CTX_NA,CTX_NA},{1,62},{5,52},{5,57}},
 {{CTX_NA,CTX_NA},{-6,86},{6,69},{-6,93}},
 {{CTX_NA,CTX_NA},{-17,95},{-13,90},{-14,88}},
 {{CTX_NA,CTX_NA},{-6,61},{0,52},{-6,44}},
 {{CTX_NA,CTX_NA},{9,45},{8,43},{4,55}},
		// 40-53 Table 9-15
 {{CTX_NA,CTX_NA},{-3,69},{-2,69},{-11,89}},
 {{CTX_NA,CTX_NA},{-6,81},{-5,82},{-15,103}},
 {{CTX_NA,CTX_NA},{-11,96},{-10,96},{-21,116}},
 {{CTX_NA,CTX_NA},{6,55},{2,59},{19,57}},
 {{CTX_NA,CTX_NA},{7,67},{2,75},{20,58}},
 {{CTX_NA,CTX_NA},{-5,86},{-3,87},{4,84}},
 {{CTX_NA,CTX_NA},{2,88},{-3,100},{6,96}},
 {{CTX_NA,CTX_NA},{0,58},{1,56},{1,63}},
 {{CTX_NA,CTX_NA},{-3,76},{-3,74},{-5,85}},
 {{CTX_NA,CTX_NA},{-10,94},{-6,85},{-13,106}},
 {{CTX_NA,CTX_NA},{5,54},{0,59},{5,63}},
 {{CTX_NA,CTX_NA},{4,69},{-3,81},{6,75}},
 {{CTX_NA,CTX_NA},{-3,81},{-7,86},{-3,90}},
 {{CTX_NA,CTX_NA},{0,88},{-5,95},{-1,101}},
		// 54-59 Table 9-16
 {{CTX_NA,CTX_NA},{-7,67},{-1,66},{3,55}},
 {{CTX_NA,CTX_NA},{-5,74},{-1,77},{-4,79}},
 {{CTX_NA,CTX_NA},{-4,74},{1,70},{-2,75}},
 {{CTX_NA,CTX_NA},{-5,80},{-2,86},{-12,97}},
 {{CTX_NA,CTX_NA},{-7,72},{-5,72},{-7,50}},
 {{CTX_NA,CTX_NA},{1,58},{0,61},{1,60}},
		// 60-69 Table 9-17
 {{0,41},{0,41},{0,41},{0,41}},
 {{0,63},{0,63},{0,63},{0,63}},
 {{0,63},{0,63},{0,63},{0,63}},
 {{0,63},{0,63},{0,63},{0,63}},
 {{-9,83},{-9,83},{-9,83},{-9,83}},
 {{4,86},{4,86},{4,86},{4,86}},
 {{0,97},{0,97},{0,97},{0,97}},
 {{-7,72},{-7,72},{-7,72},{-7,72}},
 {{13,41},{13,41},{13,41},{13,41}},
 {{3,62},{3,62},{3,62},{3,62}},
		// 70-104 Table 9-18
 {{0,11},{0,45},{13,15},{7,34}},
 {{1,55},{-4,78},{7,51},{-9,88}},
 {{0,69},{-3,96},{2,80},{-20,127}},
 {{-17,127},{-27,126},{-39,127},{-36,127}},
 {{-13,102},{-28,98},{-18,91},{-17,91}},
 {{0,82},{-25,101},{-17,96},{-14,95}},
 {{-7,74},{-23,67},{-26,81},{-25,84}},
 {{-21,107},{-28,82},{-35,98},{-25,86}},
 {{-27,127},{-20,94},{-24,102},{-12,89}},
 {{-31,127},{-16,83},{-23,97},{-17,91}},
 {{-24,127},{-22,110},{-27,119},{-31,127}},
 {{-18,95},{-21,91},{-24,99},{-14,76}},
 {{-27,127},{-18,102},{-21,110},{-18,103}},
 {{-21,114},{-13,93},{-18,102},{-13,90}},
 {{-30,127},{-29,127},{-36,127},{-37,127}},
 {{-17,123},{-7,92},{0,80},{11,80}},
 {{-12,115},{-5,89},{-5,89},{5,76}},
 {{-16,122},{-7,96},{-7,94},{2,84}},
 {{-11,115},{-13,108},{-4,92},{5,78}},
 {{-12,63},{-3,46},{0,39},{-6,55}},
 {{-2,68},{-1,65},{0,65},{4,61}},
 {{-15,84},{-1,57},{-15,84},{-14,83}},
 {{-13,104},{-9,93},{-35,127},{-37,127}},
 {{-3,70},{-3,74},{-2,73},{-5,79}},
 {{-8,93},{-9,92},{-12,104},{-11,104}},
 {{-10,90},{-8,87},{-9,91},{-11,91}},
 {{-30,127},{-23,126},{-31,127},{-30,127}},
 {{-1,74},{5,54},{3,55},{0,65}},
 {{-6,97},{6,60},{7,56},{-2,79}},
 {{-7,91},{6,59},{7,55},{0,72}},
 {{-20,127},{6,69},{8,61},{-4,92}},
 {{-4,56},{-1,48},{-3,53},{-6,56}},
 {{-5,82},{0,68},{0,68},{3,68}},
 {{-7,76},{-4,69},{-7,74},{-8,71}},
 {{-22,125},{-8,88},{-9,88},{-13,98}},
		// 105-165 Table 9-19
 {{-7,93},{-2,85},{-13,103},{-4,86}},
 {{-11,87},{-6,78},{-13,91},{-12,88}},
 {{-3,77},{-1,75},{-9,89},{-5,82}},
 {{-5,71},{-7,77},{-14,92},{-3,72}},
 {{-4,63},{2,54},{-8,76},{-4,67}},
 {{-4,68},{5,50},{-12,87},{-8,72}},
 {{-12,84},{-3,68},{-23,110},{-16,89}},
 {{-7,62},{1,50},{-24,105},{-9,69}},
 {{-7,65},{6,42},{-10,78},{-1,59}},
 {{8,61},{-4,81},{-20,112},{5,66}},
 {{5,56},{1,63},{-17,99},{4,57}},
 {{-2,66},{-4,70},{-78,127},{-4,71}},
 {{1,64},{0,67},{-70,127},{-2,71}},
 {{0,61},{2,57},{-50,127},{2,58}},
 {{-2,78},{-2,76},{-46,127},{-1,74}},
 {{1,50},{11,35},{-4,66},{-4,44}},
 {{7,52},{4,64},{-5,78},{-1,69}},
 {{10,35},{1,61},{-4,71},{0,62}},
 {{0,44},{11,35},{-8,72},{-7,51}},
 {{11,38},{18,25},{2,59},{-4,47}},
 {{1,45},{12,24},{-1,55},{-6,42}},
 {{0,46},{13,29},{-7,70},{-3,41}},
 {{5,44},{13,36},{-6,75},{-6,53}},
 {{31,17},{-10,93},{-8,89},{8,76}},
 {{1,51},{-7,73},{-34,119},{-9,78}},
 {{7,50},{-2,73},{-3,75},{-11,83}},
 {{28,19},{13,46},{32,20},{9,52}},
 {{16,33},{9,49},{30,22},{0,67}},
 {{14,62},{-7,100},{-44,127},{-5,90}},
 {{-13,108},{9,53},{0,54},{1,67}},
 {{-15,100},{2,53},{-5,61},{-15,72}},
 {{-13,101},{5,53},{0,58},{-5,75}},
 {{-13,91},{-2,61},{-1,60},{-8,80}},
 {{-12,94},{0,56},{-3,61},{-21,83}},
 {{-10,88},{0,56},{-8,67},{-21,64}},
 {{-16,84},{-13,63},{-25,84},{-13,31}},
 {{-10,86},{-5,60},{-14,74},{-25,64}},
 {{-7,83},{-1,62},{-5,65},{-29,94}},
 {{-13,87},{4,57},{5,52},{9,75}},
 {{-19,94},{-6,69},{2,57},{17,63}},
 {{1,70},{4,57},{0,61},{-8,74}},
 {{0,72},{14,39},{-9,69},{-5,35}},
 {{-5,74},{4,51},{-11,70},{-2,27}},
 {{18,59},{13,68},{18,55},{13,91}},
 {{-8,102},{3,64},{-4,71},{3,65}},
 {{-15,100},{1,61},{0,58},{-7,69}},
 {{0,95},{9,63},{7,61},{8,77}},
 {{-4,75},{7,50},{9,41},{-10,66}},
 {{2,72},{16,39},{18,25},{3,62}},
 {{-11,75},{5,44},{9,32},{-3,68}},
 {{-3,71},{4,52},{5,43},{-20,81}},
 {{15,46},{11,48},{9,47},{0,30}},
 {{-13,69},{-5,60},{0,44},{1,7}},
 {{0,62},{-1,59},{0,51},{-3,23}},
 {{0,65},{0,59},{2,46},{-21,74}},
 {{21,37},{22,33},{19,38},{16,66}},
 {{-15,72},{5,44},{-4,66},{-23,124}},
 {{9,57},{14,43},{15,38},{17,37}},
 {{16,54},{-1,78},{12,42},{44,-18}},
 {{0,62},{0,60},{9,34},{50,-34}},
 {{12,72},{9,69},{0,89},{-22,127}},
		// 166-226 Table 9-20
 {{24,0},{11,28},{4,45},{4,39}},
 {{15,9},{2,40},{10,28},{0,42}},
 {{8,25},{3,44},{10,31},{7,34}},
 {{13,18},{0,49},{33,-11},{11,29}},
 {{15,9},{0,46},{52,-43},{8,31}},
 {{13,19},{2,44},{18,15},{6,37}},
 {{10,37},{2,51},{28,0},{7,42}},
 {{12,18},{0,47},{35,-22},{3,40}},
 {{6,29},{4,39},{38,-25},{8,33}},
 {{20,33},{2,62},{34,0},{13,43}},
 {{15,30},{6,46},{39,-18},{13,36}},
 {{4,45},{0,54},{32,-12},{4,47}},
 {{1,58},{3,54},{102,-94},{3,55}},
 {{0,62},{2,58},{0,0},{2,58}},
 {{7,61},{4,63},{56,-15},{6,60}},
 {{12,38},{6,51},{33,-4},{8,44}},
 {{11,45},{6,57},{29,10},{11,44}},
 {{15,39},{7,53},{37,-5},{14,42}},
 {{11,42},{6,52},{51,-29},{7,48}},
 {{13,44},{6,55},{39,-9},{4,56}},
 {{16,45},{11,45},{52,-34},{4,52}},
 {{12,41},{14,36},{69,-58},{13,37}},
 {{10,49},{8,53},{67,-63},{9,49}},
 {{30,34},{-1,82},{44,-5},{19,58}},
 {{18,42},{7,55},{32,7},{10,48}},
 {{10,55},{-3,78},{55,-29},{12,45}},
 {{17,51},{15,46},{32,1},{0,69}},
 {{17,46},{22,31},{0,0},{20,33}},
 {{0,89},{-1,84},{27,36},{8,63}},
 {{26,-19},{25,7},{33,-25},{35,-18}},
 {{22,-17},{30,-7},{34,-30},{33,-25}},
 {{26,-17},{28,3},{36,-28},{28,-3}},
 {{30,-25},{28,4},{38,-28},{24,10}},
 {{28,-20},{32,0},{38,-27},{27,0}},
 {{33,-23},{34,-1},{34,-18},{34,-14}},
 {{37,-27},{30,6},{35,-16},{52,-44}},
 {{33,-23},{30,6},{34,-14},{39,-24}},
 {{40,-28},{32,9},{32,-8},{19,17}},
 {{38,-17},{31,19},{37,-6},{31,25}},
 {{33,-11},{26,27},{35,0},{36,29}},
 {{40,-15},{26,30},{30,10},{24,33}},
 {{41,-6},{37,20},{28,18},{34,15}},
 {{38,1},{28,34},{26,25},{30,20}},
 {{41,17},{17,70},{29,41},{22,73}},
 {{30,-6},{1,67},{0,75},{20,34}},
 {{27,3},{5,59},{2,72},{19,31}},
 {{26,22},{9,67},{8,77},{27,44}},
 {{37,-16},{16,30},{14,35},{19,16}},
 {{35,-4},{18,32},{18,31},{15,36}},
 {{38,-8},{18,35},{17,35},{15,36}},
 {{38,-3},{22,29},{21,30},{21,28}},
 {{37,3},{24,31},{17,45},{25,21}},
 {{38,5},{23,38},{20,42},{30,20}},
 {{42,0},{18,43},{18,45},{31,12}},
 {{35,16},{20,41},{27,26},{27,16}},
 {{39,22},{11,63},{16,54},{24,42}},
 {{14,48},{9,59},{7,66},{0,93}},
 {{27,37},{9,64},{16,56},{14,56}},
 {{21,60},{-1,94},{11,73},{15,57}},
 {{12,68},{-2,89},{10,67},{26,38}},
 {{2,97},{-9,108},{-10,116},{-24,127}},
		// 227-275 Table 9-21
 {{-3,71},{-6,76},{-23,112},{-24,115}},
 {{-6,42},{-2,44},{-15,71},{-22,82}},
 {{-5,50},{0,45},{-7,61},{-9,62}},
 {{-3,54},{0,52},{0,53},{0,53}},
 {{-2,62},{-3,64},{-5,66},{0,59}},
 {{0,58},{-2,59},{-11,77},{-14,85}},
 {{1,63},{-4,70},{-9,80},{-13,89}},
 {{-2,72},{-4,75},{-9,84},{-13,94}},
 {{-1,74},{-8,82},{-10,87},{-11,92}},
 {{-9,91},{-17,102},{-34,127},{-29,127}},
 {{-5,67},{-9,77},{-21,101},{-21,100}},
 {{-5,27},{3,24},{-3,39},{-14,57}},
 {{-3,39},{0,42},{-5,53},{-12,67}},
 {{-2,44},{0,48},{-7,61},{-11,71}},
 {{0,46},{0,55},{-11,75},{-10,77}},
 {{-16,64},{-6,59},{-15,77},{-21,85}},
 {{-8,68},{-7,71},{-17,91},{-16,88}},
 {{-10,78},{-12,83},{-25,107},{-23,104}},
 {{-6,77},{-11,87},{-25,111},{-15,98}},
 {{-10,86},{-30,119},{-28,122},{-37,127}},
 {{-12,92},{1,58},{-11,76},{-10,82}},
 {{-15,55},{-3,29},{-10,44},{-8,48}},
 {{-10,60},{-1,36},{-10,52},{-8,61}},
 {{-6,62},{1,38},{-10,57},{-8,66}},
 {{-4,65},{2,43},{-9,58},{-7,70}},
 {{-12,73},{-6,55},{-16,72},{-14,75}},
 {{-8,76},{0,58},{-7,69},{-10,79}},
 {{-7,80},{0,64},{-4,69},{-9,83}},
 {{-9,88},{-3,74},{-5,74},{-12,92}},
 {{-17,110},{-10,90},{-9,86},{-18,108}},
 {{-11,97},{0,70},{2,66},{-4,79}},
 {{-20,84},{-4,29},{-9,34},{-22,69}},
 {{-11,79},{5,31},{1,32},{-16,75}},
 {{-6,73},{7,42},{11,31},{-2,58}},
 {{-4,74},{1,59},{5,52},{1,58}},
 {{-13,86},{-2,58},{-2,55},{-13,78}},
 {{-13,96},{-3,72},{-2,67},{-9,83}},
 {{-11,97},{-3,81},{0,73},{-4,81}},
 {{-19,117},{-11,97},{-8,89},{-13,99}},
 {{-8,78},{0,58},{3,52},{-13,81}},
 {{-5,33},{8,5},{7,4},{-6,38}},
 {{-4,48},{10,14},{10,8},{-13,62}},
 {{-2,53},{14,18},{17,8},{-6,58}},
 {{-3,62},{13,27},{16,19},{-2,59}},
 {{-13,71},{2,40},{3,37},{-16,73}},
 {{-10,79},{0,58},{-1,61},{-10,76}},
 {{-12,86},{-3,70},{-5,73},{-13,86}},
 {{-13,90},{-6,79},{-1,70},{-9,83}},
 {{-14,97},{-8,85},{-4,78},{-10,87}},
		// 276 no use
 {{CTX_NA,CTX_NA},{CTX_NA,CTX_NA},{CTX_NA,CTX_NA},{CTX_NA,CTX_NA}},
		// 277-337 Table 9-22
 {{-6,93},{-13,106},{-21,126},{-22,127}},
 {{-6,84},{-16,106},{-23,124},{-25,127}},
 {{-8,79},{-10,87},{-20,110},{-25,120}},
 {{0,66},{-21,114},{-26,126},{-27,127}},
 {{-1,71},{-18,110},{-25,124},{-19,114}},
 {{0,62},{-14,98},{-17,105},{-23,117}},
 {{-2,60},{-22,110},{-27,121},{-25,118}},
 {{-2,59},{-21,106},{-27,117},{-26,117}},
 {{-5,75},{-18,103},{-17,102},{-24,113}},
 {{-3,62},{-21,107},{-26,117},{-28,118}},
 {{-4,58},{-23,108},{-27,116},{-31,120}},
 {{-9,66},{-26,112},{-33,122},{-37,124}},
 {{-1,79},{-10,96},{-10,95},{-10,94}},
 {{0,71},{-12,95},{-14,100},{-15,102}},
 {{3,68},{-5,91},{-8,95},{-10,99}},
 {{10,44},{-9,93},{-17,111},{-13,106}},
 {{-7,62},{-22,94},{-28,114},{-50,127}},
 {{15,36},{-5,86},{-6,89},{-5,92}},
 {{14,40},{9,67},{-2,80},{17,57}},
 {{16,27},{-4,80},{-4,82},{-5,86}},
 {{12,29},{-10,85},{-9,85},{-13,94}},
 {{1,44},{-1,70},{-8,81},{-12,91}},
 {{20,36},{7,60},{-1,72},{-2,77}},
 {{18,32},{9,58},{5,64},{0,71}},
 {{5,42},{5,61},{1,67},{-1,73}},
 {{1,48},{12,50},{9,56},{4,64}},
 {{10,62},{15,50},{0,69},{-7,81}},
 {{17,46},{18,49},{1,69},{5,64}},
 {{9,64},{17,54},{7,69},{15,57}},
 {{-12,104},{10,41},{-7,69},{1,67}},
 {{-11,97},{7,46},{-6,67},{0,68}},
 {{-16,96},{-1,51},{-16,77},{-10,67}},
 {{-7,88},{7,49},{-2,64},{1,68}},
 {{-8,85},{8,52},{2,61},{0,77}},
 {{-7,85},{9,41},{-6,67},{2,64}},
 {{-9,85},{6,47},{-3,64},{0,68}},
 {{-13,88},{2,55},{2,57},{-5,78}},
 {{4,66},{13,41},{-3,65},{7,55}},
 {{-3,77},{10,44},{-3,66},{5,59}},
 {{-3,76},{6,50},{0,62},{2,65}},
 {{-6,76},{5,53},{9,51},{14,54}},
 {{10,58},{13,49},{-1,66},{15,44}},
 {{-1,76},{4,63},{-2,71},{5,60}},
 {{-1,83},{6,64},{-2,75},{2,70}},
 {{-7,99},{-2,69},{-1,70},{-2,76}},
 {{-14,95},{-2,59},{-9,72},{-18,86}},
 {{2,95},{6,70},{14,60},{12,70}},
 {{0,76},{10,44},{16,37},{5,64}},
 {{-5,74},{9,31},{0,47},{-12,70}},
 {{0,70},{12,43},{18,35},{11,55}},
 {{-11,75},{3,53},{11,37},{5,56}},
 {{1,68},{14,34},{12,41},{0,69}},
 {{0,65},{10,38},{10,41},{2,65}},
 {{-14,73},{-3,52},{2,48},{-6,74}},
 {{3,62},{13,40},{12,41},{5,54}},
 {{4,62},{17,32},{13,41},{7,54}},
 {{-1,68},{7,44},{0,59},{-6,76}},
 {{-13,75},{7,38},{3,50},{-11,82}},
 {{11,55},{13,50},{19,40},{-2,77}},
 {{5,64},{10,57},{3,66},{-2,77}},
 {{12,70},{26,43},{18,50},{25,42}},
		// 338-398 Table9-23
 {{15,6},{14,11},{19,-6},{17,-13}},
 {{6,19},{11,14},{18,-6},{16,-9}},
 {{7,16},{9,11},{14,0},{17,-12}},
 {{12,14},{18,11},{26,-12},{27,-21}},
 {{18,13},{21,9},{31,-16},{37,-30}},
 {{13,11},{23,-2},{33,-25},{41,-40}},
 {{13,15},{32,-15},{33,-22},{42,-41}},
 {{15,16},{32,-15},{37,-28},{48,-47}},
 {{12,23},{34,-21},{39,-30},{39,-32}},
 {{13,23},{39,-23},{42,-30},{46,-40}},
 {{15,20},{42,-33},{47,-42},{52,-51}},
 {{14,26},{41,-31},{45,-36},{46,-41}},
 {{14,44},{46,-28},{49,-34},{52,-39}},
 {{17,40},{38,-12},{41,-17},{43,-19}},
 {{17,47},{21,29},{32,9},{32,11}},
 {{24,17},{45,-24},{69,-71},{61,-55}},
 {{21,21},{53,-45},{63,-63},{56,-46}},
 {{25,22},{48,-26},{66,-64},{62,-50}},
 {{31,27},{65,-43},{77,-74},{81,-67}},
 {{22,29},{43,-19},{54,-39},{45,-20}},
 {{19,35},{39,-10},{52,-35},{35,-2}},
 {{14,50},{30,9},{41,-10},{28,15}},
 {{10,57},{18,26},{36,0},{34,1}},
 {{7,63},{20,27},{40,-1},{39,1}},
 {{-2,77},{0,57},{30,14},{30,17}},
 {{-4,82},{-14,82},{28,26},{20,38}},
 {{-3,94},{-5,75},{23,37},{18,45}},
 {{9,69},{-19,97},{12,55},{15,54}},
 {{-12,109},{-35,125},{11,65},{0,79}},
 {{36,-35},{27,0},{37,-33},{36,-16}},
 {{36,-34},{28,0},{39,-36},{37,-14}},
 {{32,-26},{31,-4},{40,-37},{37,-17}},
 {{37,-30},{27,6},{38,-30},{32,1}},
 {{44,-32},{34,8},{46,-33},{34,15}},
 {{34,-18},{30,10},{42,-30},{29,15}},
 {{34,-15},{24,22},{40,-24},{24,25}},
 {{40,-15},{33,19},{49,-29},{34,22}},
 {{33,-7},{22,32},{38,-12},{31,16}},
 {{35,-5},{26,31},{40,-10},{35,18}},
 {{33,0},{21,41},{38,-3},{31,28}},
 {{38,2},{26,44},{46,-5},{33,41}},
 {{33,13},{23,47},{31,20},{36,28}},
 {{23,35},{16,65},{29,30},{27,47}},
 {{13,58},{14,71},{25,44},{21,62}},
 {{29,-3},{8,60},{12,48},{18,31}},
 {{26,0},{6,63},{11,49},{19,26}},
 {{22,30},{17,65},{26,45},{36,24}},
 {{31,-7},{21,24},{22,22},{24,23}},
 {{35,-15},{23,20},{23,22},{27,16}},
 {{34,-3},{26,23},{27,21},{24,30}},
 {{34,3},{27,32},{33,20},{31,29}},
 {{36,-1},{28,23},{26,28},{22,41}},
 {{34,5},{28,24},{30,24},{22,42}},
 {{32,11},{23,40},{27,34},{16,60}},
 {{35,5},{24,32},{18,42},{15,52}},
 {{34,12},{28,29},{25,39},{14,60}},
 {{39,11},{23,42},{18,50},{3,78}},
 {{30,29},{19,57},{12,70},{-16,123}},
 {{34,26},{22,53},{21,54},{21,53}},
 {{29,39},{22,61},{14,71},{22,56}},
 {{19,66},{11,86},{11,83},{25,61}},
 {{31,21},{12,40},{25,32},{21,33}},
 {{31,31},{11,51},{21,49},{19,50}},
 {{25,50},{14,59},{21,54},{17,61}},
		// 402-459 Table 9-24
 {{-17,120},{-4,79},{-5,85},{-3,78}},
 {{-20,112},{-7,71},{-6,81},{-8,74}},
 {{-18,114},{-5,69},{-10,77},{-9,72}},
 {{-11,85},{-9,70},{-7,81},{-10,72}},
 {{-15,92},{-8,66},{-17,80},{-18,75}},
 {{-14,89},{-10,68},{-18,73},{-12,71}},
 {{-26,71},{-19,73},{-4,74},{-11,63}},
 {{-15,81},{-12,69},{-10,83},{-5,70}},
 {{-14,80},{-16,70},{-9,71},{-17,75}},
 {{0,68},{-15,67},{-9,67},{-14,72}},
 {{-14,70},{-20,62},{-1,61},{-16,67}},
 {{-24,56},{-19,70},{-8,66},{-8,53}},
 {{-23,68},{-16,66},{-14,66},{-14,59}},
 {{-24,50},{-22,65},{0,59},{-9,52}},
 {{-11,74},{-20,63},{2,59},{-11,68}},
 {{23,-13},{9,-2},{17,-10},{9,-2}},
 {{26,-13},{26,-9},{32,-13},{30,-10}},
 {{40,-15},{33,-9},{42,-9},{31,-4}},
 {{49,-14},{39,-7},{49,-5},{33,-1}},
 {{44,3},{41,-2},{53,0},{33,7}},
 {{45,6},{45,3},{64,3},{31,12}},
 {{44,34},{49,9},{68,10},{37,23}},
 {{33,54},{45,27},{66,27},{31,38}},
 {{19,82},{36,59},{47,57},{20,64}},
 {{-3,75},{-6,66},{-5,71},{-9,71}},
 {{-1,23},{-7,35},{0,24},{-7,37}},
 {{1,34},{-7,42},{-1,36},{-8,44}},
 {{1,43},{-8,45},{-2,42},{-11,49}},
 {{0,54},{-5,48},{-2,52},{-10,56}},
 {{-2,55},{-12,56},{-9,57},{-12,59}},
 {{0,61},{-6,60},{-6,63},{-8,63}},
 {{1,64},{-5,62},{-4,65},{-9,67}},
 {{0,68},{-8,66},{-4,67},{-6,68}},
 {{-9,92},{-8,76},{-7,82},{-10,79}},
 {{-14,106},{-5,85},{-3,81},{-3,78}},
 {{-13,97},{-6,81},{-3,76},{-8,74}},
 {{-15,90},{-10,77},{-7,72},{-9,72}},
 {{-12,90},{-7,81},{-6,78},{-10,72}},
 {{-18,88},{-17,80},{-12,72},{-18,75}},
 {{-10,73},{-18,73},{-14,68},{-12,71}},
 {{-9,79},{-4,74},{-3,70},{-11,63}},
 {{-14,86},{-10,83},{-6,76},{-5,70}},
 {{-10,73},{-9,71},{-5,66},{-17,75}},
 {{-10,70},{-9,67},{-5,62},{-14,72}},
 {{-10,69},{-1,61},{0,57},{-16,67}},
 {{-5,66},{-8,66},{-4,61},{-8,53}},
 {{-9,64},{-14,66},{-9,60},{-14,59}},
 {{-5,58},{0,59},{1,54},{-9,52}},
 {{2,59},{2,59},{2,58},{-11,68}},
 {{21,-10},{21,-13},{17,-10},{9,-2}},
 {{24,-11},{33,-14},{32,-13},{30,-10}},
 {{28,-8},{39,-7},{42,-9},{31,-4}},
 {{28,-1},{46,-2},{49,-5},{33,-1}},
 {{29,3},{51,2},{53,0},{33,7}},
 {{29,9},{60,6},{64,3},{31,12}},
 {{35,20},{61,17},{68,10},{37,23}},
 {{29,36},{55,34},{66,27},{31,38}},
 {{14,67},{42,62},{47,57},{20,64}},
};

void WelsCabacGlobalInit(SDecoderContext* pCtx){
	for(int32_t iModel=0; iModel<4; iModel++){
		for(int32_t iQp=0; iQp<=WELS_QP_MAX; iQp++)
			for(int32_t iIdx=0; iIdx<WELS_CONTEXT_COUNT; iIdx++){
				int32_t m=g_kiCabacGlobalContextIdx[iIdx][iModel][0];
				int32_t n=g_kiCabacGlobalContextIdx[iIdx][iModel][1];
				int32_t iPreCtxState=WELS_CLIP3((((m*iQp)>>4)+n),1,126);
				uint8_t uiValMps=0;
				uint8_t uiStateIdx=0;
				if(iPreCtxState<=63){
					uiStateIdx=63-iPreCtxState;
					uiValMps=0;
				}else{
					uiStateIdx=iPreCtxState-64;
					uiValMps=1;
				}
				pCtx->sWelsCabacContexts[iModel][iQp][iIdx].uiState=uiStateIdx;
				pCtx->sWelsCabacContexts[iModel][iQp][iIdx].uiMPS=uiValMps;
			}
	}
	pCtx->bCabacInited=true;
}

// -------------------1. context initialization
void WelsCabacContextInit(SDecoderContext* pCtx,uint8_t eSliceType,int32_t iCabacInitIdc,int32_t iQp){
	int32_t iIdx=pCtx->eSliceType==I_SLICE ? 0 : iCabacInitIdc+1;
	if(!pCtx->bCabacInited){
		WelsCabacGlobalInit(pCtx);
	}
	memcpy(pCtx->pCabacCtx,pCtx->sWelsCabacContexts[iIdx][iQp],WELS_CONTEXT_COUNT*sizeof(SWelsCabacCtx));
}

const uint8_t g_kuiMatrixV[6][8][8]={		// generated from equation 8-317,8-318
	{
		{20,19,25,19,20,19,25,19},
	{19,18,24,18,19,18,24,18},
	{25,24,32,24,25,24,32,24},
	{19,18,24,18,19,18,24,18},
	{20,19,25,19,20,19,25,19},
	{19,18,24,18,19,18,24,18},
	{25,24,32,24,25,24,32,24},
	{19,18,24,18,19,18,24,18}
	},
 {
	 {22,21,28,21,22,21,28,21},
	{21,19,26,19,21,19,26,19},
	{28,26,35,26,28,26,35,26},
	{21,19,26,19,21,19,26,19},
	{22,21,28,21,22,21,28,21},
	{21,19,26,19,21,19,26,19},
	{28,26,35,26,28,26,35,26},
	{21,19,26,19,21,19,26,19}
 },
 {
	 {26,24,33,24,26,24,33,24},
	{24,23,31,23,24,23,31,23},
	{33,31,42,31,33,31,42,31},
	{24,23,31,23,24,23,31,23},
	{26,24,33,24,26,24,33,24},
	{24,23,31,23,24,23,31,23},
	{33,31,42,31,33,31,42,31},
	{24,23,31,23,24,23,31,23}
 },
 {
	 {28,26,35,26,28,26,35,26},
	{26,25,33,25,26,25,33,25},
	{35,33,45,33,35,33,45,33},
	{26,25,33,25,26,25,33,25},
	{28,26,35,26,28,26,35,26},
	{26,25,33,25,26,25,33,25},
	{35,33,45,33,35,33,45,33},
	{26,25,33,25,26,25,33,25}
 },
 {
	 {32,30,40,30,32,30,40,30},
	{30,28,38,28,30,28,38,28},
	{40,38,51,38,40,38,51,38},
	{30,28,38,28,30,28,38,28},
	{32,30,40,30,32,30,40,30},
	{30,28,38,28,30,28,38,28},
	{40,38,51,38,40,38,51,38},
	{30,28,38,28,30,28,38,28}
 },
 {
	 {36,34,46,34,36,34,46,34},
	{34,32,43,32,34,32,43,32},
	{46,43,58,43,46,43,58,43},
	{34,32,43,32,34,32,43,32},
	{36,34,46,34,36,34,46,34},
	{34,32,43,32,34,32,43,32},
	{46,43,58,43,46,43,58,43},
	{34,32,43,32,34,32,43,32}
 }
};

// Calculate deqaunt coeff scaling list value
int32_t WelsCalcDeqCoeffScalingList(SDecoderContext* pCtx){
	if(pCtx->pSps->bSeqScalingMatrixPresentFlag || pCtx->pPps->bPicScalingMatrixPresentFlag){
		pCtx->bUseScalingList=true;

		if(!pCtx->bDequantCoeff4x4Init || (pCtx->iDequantCoeffPpsid!=pCtx->pPps->iPpsId)){
			int i,q,x,y;
			// Init dequant coeff value for different QP
			for(i=0; i<6; i++){
				pCtx->pDequant_coeff4x4[i]=pCtx->pDequant_coeff_buffer4x4[i];
				pCtx->pDequant_coeff8x8[i]=pCtx->pDequant_coeff_buffer8x8[i];
				for(q=0; q<51; q++){
					for(x=0; x<16; x++){
						pCtx->pDequant_coeff4x4[i][q][x]=pCtx->pPps->bPicScalingMatrixPresentFlag ? pCtx->pPps->iScalingList4x4[i][x]*
							g_kuiDequantCoeff[q][x&0x07] : pCtx->pSps->iScalingList4x4[i][x]*g_kuiDequantCoeff[q][x&0x07];
					}
					for(y=0; y<64; y++){
						pCtx->pDequant_coeff8x8[i][q][y]=pCtx->pPps->bPicScalingMatrixPresentFlag ? pCtx->pPps->iScalingList8x8[i][y]*
							g_kuiMatrixV[q%6][y/8][y%8] : pCtx->pSps->iScalingList8x8[i][y]*g_kuiMatrixV[q%6][y/8][y%8];
					}
				}
			}
			pCtx->bDequantCoeff4x4Init=true;
			pCtx->iDequantCoeffPpsid=pCtx->pPps->iPpsId;
		}
	}else
		pCtx->bUseScalingList=false;
	return ERR_NONE;
}

#ifndef MB_XY_T
#define MB_XY_T int32_t
#endif// MB_XY_T

// brief Convert kMbXy to slice group idc correspondingly
// param pFmo Wels fmo context
// param kMbXy kMbXy to be converted
// return slice group idc-successful;-1-failed;
int32_t FmoMbToSliceGroup(SFmo* pFmo,const MB_XY_T kiMbXy){
	const int32_t kiMbNum=pFmo->iCountMbNum;
	const uint8_t* kpMbMap=pFmo->pMbAllocMap;

	if(kiMbXy<0 || kiMbXy>=kiMbNum || kpMbMap==NULL)
		return-1;

	return kpMbMap[kiMbXy];
}

// brief Get successive mb to be processed with given current kMbXy
// param pFmo Wels fmo context
// param kMbXy current kMbXy
// return iNextMb-successful;-1-failed;
MB_XY_T FmoNextMb(SFmo* pFmo,const MB_XY_T kiMbXy){
	const int32_t kiTotalMb=pFmo->iCountMbNum;
	const uint8_t* kpMbMap=pFmo->pMbAllocMap;
	MB_XY_T iNextMb=kiMbXy;
	const uint8_t kuiSliceGroupIdc=(uint8_t)FmoMbToSliceGroup(pFmo,kiMbXy);

	if(kuiSliceGroupIdc==(uint8_t)(-1))
		return-1;

	do{
		++iNextMb;
		if(iNextMb>=kiTotalMb){
			iNextMb=-1;
			break;
		}
		if(kpMbMap[iNextMb]==kuiSliceGroupIdc){
			break;
		}
	} while(1);

	// -1: No further MB in this slice (could be end of picture)
	return iNextMb;
}

int32_t WelsDecodeSlice(SDecoderContext* pCtx,bool bFirstSliceInLayer,SNalUnit* pNalCur){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SFmo* pFmo=pCtx->pFmo;
	int32_t iRet;
	int32_t iNextMbXyIndex,iSliceIdc;

	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeaderExt* pSliceHeaderExt=&pSlice->sSliceHeaderExt;
	SSliceHeader* pSliceHeader=&pSliceHeaderExt->sSliceHeader;
	int32_t iMbX,iMbY;
	const int32_t kiCountNumMb=pSliceHeader->pSps->uiTotalMbCount;		// need to be correct when fmo or multi slice
	uint32_t uiEosFlag=0;
	PWelsDecMbFunc pDecMbFunc;

	pSlice->iTotalMbInCurSlice=0;		// initialize at the starting of slice decoding.

	if(pCtx->pPps->bEntropyCodingModeFlag){
		if(pSlice->sSliceHeaderExt.bAdaptiveMotionPredFlag || 
			pSlice->sSliceHeaderExt.bAdaptiveBaseModeFlag || 
			pSlice->sSliceHeaderExt.bAdaptiveResidualPredFlag){
			FATAL(
					 "WelsDecodeSlice()::::ILP flag exist,not supported with CABAC enabled!");
			pCtx->iErrorCode|=dsBitstreamError;
			return dsBitstreamError;
		}
		if(P_SLICE==pSliceHeader->eSliceType)
			pDecMbFunc=WelsDecodeMbCabacPSlice;
		else if(B_SLICE==pSliceHeader->eSliceType)
			pDecMbFunc=WelsDecodeMbCabacBSlice;
		else		// I_SLICE. B_SLICE is being supported
			pDecMbFunc=WelsDecodeMbCabacISlice;
	}else{
		if(P_SLICE==pSliceHeader->eSliceType){
			pDecMbFunc=WelsDecodeMbCavlcPSlice;
		}else
		if(B_SLICE==pSliceHeader->eSliceType){
			pDecMbFunc=WelsDecodeMbCavlcBSlice;
		}else{		// I_SLICE
			pDecMbFunc=WelsDecodeMbCavlcISlice;
		}
	}

	if(pSliceHeader->pPps->bConstainedIntraPredFlag){
		pCtx->pFillInfoCacheIntraNxNFunc=WelsFillCacheConstrain1IntraNxN;
		pCtx->pMapNxNNeighToSampleFunc=WelsMapNxNNeighToSampleConstrain1;
		pCtx->pMap16x16NeighToSampleFunc=WelsMap16x16NeighToSampleConstrain1;
	}else{
		pCtx->pFillInfoCacheIntraNxNFunc=WelsFillCacheConstrain0IntraNxN;
		pCtx->pMapNxNNeighToSampleFunc=WelsMapNxNNeighToSampleNormal;
		pCtx->pMap16x16NeighToSampleFunc=WelsMap16x16NeighToSampleNormal;
	}

	pCtx->eSliceType=pSliceHeader->eSliceType;
	if(pCurDqLayer->sLayerInfo.pPps->bEntropyCodingModeFlag==1){
		int32_t iQp=pSlice->sSliceHeaderExt.sSliceHeader.iSliceQp;
		int32_t iCabacInitIdc=pSlice->sSliceHeaderExt.sSliceHeader.iCabacInitIdc;
		WelsCabacContextInit(pCtx,pSlice->eSliceType,iCabacInitIdc,iQp);
		// InitCabacCtx (pCtx->pCabacCtx,pSlice->eSliceType,iCabacInitIdc,iQp);
		pSlice->iLastDeltaQp=0;
		WELS_READ_VERIFY(InitCabacDecEngineFromBS(pCtx->pCabacDecEngine,pCtx->pCurDqLayer->pBitStringAux));
	}
	// try to calculate the dequant_coeff
	WelsCalcDeqCoeffScalingList(pCtx);

	iNextMbXyIndex=pSliceHeader->iFirstMbInSlice;
	iMbX=iNextMbXyIndex%pCurDqLayer->iMbWidth;
	iMbY=iNextMbXyIndex/pCurDqLayer->iMbWidth;		// error is introduced by multiple slices case,11/23/2009
	pSlice->iMbSkipRun=-1;
	iSliceIdc=(pSliceHeader->iFirstMbInSlice<<7)+pCurDqLayer->uiLayerDqId;

	pCurDqLayer->iMbX=iMbX;
	pCurDqLayer->iMbY=iMbY;
	pCurDqLayer->iMbXyIndex=iNextMbXyIndex;

	do{
		if((-1==iNextMbXyIndex) || (iNextMbXyIndex>=kiCountNumMb)){		// slice group boundary or end of a frame
			break;
		}

		pCurDqLayer->pSliceIdc[iNextMbXyIndex]=iSliceIdc;
		pCtx->bMbRefConcealed=false;
		iRet=pDecMbFunc(pCtx,pNalCur,uiEosFlag);
		pCurDqLayer->pMbRefConcealedFlag[iNextMbXyIndex]=pCtx->bMbRefConcealed;
		if(iRet!=ERR_NONE){
			return iRet;
		}

		++pSlice->iTotalMbInCurSlice;
		if(uiEosFlag){		// end of slice
			break;
		}
		if(pSliceHeader->pPps->uiNumSliceGroups>1){
			iNextMbXyIndex=FmoNextMb(pFmo,iNextMbXyIndex);
		}else{
			++iNextMbXyIndex;
		}
		iMbX=iNextMbXyIndex%pCurDqLayer->iMbWidth;
		iMbY=iNextMbXyIndex/pCurDqLayer->iMbWidth;
		pCurDqLayer->iMbX=iMbX;
		pCurDqLayer->iMbY=iMbY;
		pCurDqLayer->iMbXyIndex=iNextMbXyIndex;
	} while(1);

	return ERR_NONE;
}

inline void HandleReferenceLostL0(SDecoderContext* pCtx,SNalUnit* pCurNal){
	if(0==pCurNal->sNalHeaderExt.uiTemporalId){
		pCtx->bReferenceLostAtT0Flag=true;
	}
	pCtx->iErrorCode|=dsBitstreamError;
}

#define NO_SUPPORTED_FILTER_IDX (-1)
#define LEFT_FLAG_BIT 0
#define TOP_FLAG_BIT 1
#define LEFT_FLAG_MASK 0x01
#define TOP_FLAG_MASK 0x02

#define SAME_MB_DIFF_REFIDX
#define g_kuiAlphaTable(x) g_kuiAlphaTable[(x)+12]
#define g_kiBetaTable(x) g_kiBetaTable[(x)+12]
#define g_kiTc0Table(x) g_kiTc0Table[(x)+12]

#define MB_BS_MV(pRefPic0,pRefPic1,iMotionVector,iMbXy,iMbBn,iIndex,iNeighIndex) \
(\
 ( pRefPic0 !=pRefPic1)  || \
 ( WELS_ABS( iMotionVector[iMbXy][iIndex][0]-iMotionVector[iMbBn][iNeighIndex][0] ) >=4 )  || \
 ( WELS_ABS( iMotionVector[iMbXy][iIndex][1]-iMotionVector[iMbBn][iNeighIndex][1] ) >=4 )\
)

#if defined(SAME_MB_DIFF_REFIDX)
#define SMB_EDGE_MV(pRefPics,iMotionVector,iIndex,iNeighIndex) \
(\
 ( pRefPics[iIndex] !=pRefPics[iNeighIndex] ) || (\
 ( WELS_ABS( iMotionVector[iIndex][0]-iMotionVector[iNeighIndex][0] ) &(~3) ) |\
 ( WELS_ABS( iMotionVector[iIndex][1]-iMotionVector[iNeighIndex][1] ) &(~3) ))\
)
#else
#define SMB_EDGE_MV(iRefIndex,iMotionVector,iIndex,iNeighIndex) \
(\
 !!(( WELS_ABS( iMotionVector[iIndex][0]-iMotionVector[iNeighIndex][0] ) &(~3) ) |( WELS_ABS( iMotionVector[iIndex][1]-iMotionVector[iNeighIndex][1] ) &(~3) ))\
)
#endif

#define BS_EDGE(bsx1,pRefPics,iMotionVector,iIndex,iNeighIndex) \
( (bsx1|SMB_EDGE_MV(pRefPics,iMotionVector,iIndex,iNeighIndex))<<((uint8_t)(!!bsx1)))

#define GET_ALPHA_BETA_FROM_QP(iQp,iAlphaOffset,iBetaOffset,iIndex,iAlpha,iBeta) \
{\
 iIndex=(iQp+iAlphaOffset);\
 iAlpha=g_kuiAlphaTable(iIndex);\
 iBeta=g_kiBetaTable((iQp+iBetaOffset));\
}

static const uint8_t g_kuiAlphaTable[52+24]={		// this table refers to Table 8-16 in H.264/AVC standard
	0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,4,4,5,6,
	7,8,9,10,12,13,15,17,20,22,
	25,28,32,36,40,45,50,56,63,71,
	80,90,101,113,127,144,162,182,203,226,
	255,255
	,255,255,255,255,255,255,255,255,255,255,255,255
};

static const int8_t g_kiBetaTable[52+24]={		// this table refers to Table 8-16 in H.264/AVC standard
	0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,2,2,2,3,
	3,3,3,4,4,4,6,6,7,7,
	8,8,9,9,10,10,11,11,12,12,
	13,13,14,14,15,15,16,16,17,17,
	18,18
	,18,18,18,18,18,18,18,18,18,18,18,18
};

static const int8_t g_kiTc0Table[52+24][4]={		// this table refers Table 8-17 in H.264/AVC standard
	{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},
 {-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},
 {-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},
 {-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},
 {-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,0},{-1,0,0,1},
 {-1,0,0,1},{-1,0,0,1},{-1,0,0,1},{-1,0,1,1},{-1,0,1,1},{-1,1,1,1},
 {-1,1,1,1},{-1,1,1,1},{-1,1,1,1},{-1,1,1,2},{-1,1,1,2},{-1,1,1,2},
 {-1,1,1,2},{-1,1,2,3},{-1,1,2,3},{-1,2,2,3},{-1,2,2,4},{-1,2,3,4},
 {-1,2,3,4},{-1,3,3,5},{-1,3,4,6},{-1,3,4,6},{-1,4,5,7},{-1,4,5,8},
 {-1,4,6,9},{-1,5,7,10},{-1,6,8,11},{-1,6,8,13},{-1,7,10,14},{-1,8,11,16},
 {-1,9,12,18},{-1,10,13,20},{-1,11,15,23},{-1,13,17,25}
 ,{-1,13,17,25},{-1,13,17,25},{-1,13,17,25},{-1,13,17,25},{-1,13,17,25},{-1,13,17,25}
	,{-1,13,17,25},{-1,13,17,25},{-1,13,17,25},{-1,13,17,25},{-1,13,17,25},{-1,13,17,25}
};

static const uint8_t g_kuiTableBIdx[2][8]={
	{
		0,4,8,12,
		3,7,11,15
	},

 {
	 0,1,2,3,
	 12,13,14,15
 },
};

static const uint8_t g_kuiTableB8x8Idx[2][16]={
	{
		0,1,4,5,8,9,12,13,		// 0 1 | 2 3
		2,3,6,7,10,11,14,15		// 4 5 | 6 7
	}, 	// ------------
	// 8 9 | 10 11
 {
			// 12 13 | 14 15
	 0,1,4,5,2,3,6,7,
	 8,9,12,13,10,11,14,15
 },
};
// fix Bugzilla 1486223
#define TC0_TBL_LOOKUP(tc,iIndexA,pBS,bChroma) \
{\
 tc[0]=g_kiTc0Table(iIndexA)[pBS[0] & 3]+bChroma;\
 tc[1]=g_kiTc0Table(iIndexA)[pBS[1] & 3]+bChroma;\
 tc[2]=g_kiTc0Table(iIndexA)[pBS[2] & 3]+bChroma;\
 tc[3]=g_kiTc0Table(iIndexA)[pBS[3] & 3]+bChroma;\
}

void FilteringEdgeLumaIntraV(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);

	if(iAlpha|iBeta){
		pFilter->pLoopf->pfLumaDeblockingEQ4Hor(pPix,iStride,iAlpha,iBeta);
	}
	return;
}
void FilteringEdgeLumaIntraH(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;

	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,
							iBeta);

	if(iAlpha|iBeta){
		pFilter->pLoopf->pfLumaDeblockingEQ4Ver(pPix,iStride,iAlpha,iBeta);
	}
	return;
}


void FilteringEdgeLumaHV(PDqLayer pCurDqLayer,SDeblockingFilter* pFilter,int32_t iBoundryFlag){
	int32_t iMbXyIndex=pCurDqLayer->iMbXyIndex;
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;
	int32_t iMbWidth=pCurDqLayer->iMbWidth;
	int32_t iLineSize=pFilter->iCsStride[0];

	uint8_t* pDestY;
	int32_t iCurQp;
	int32_t iIndexA,iAlpha,iBeta;

	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBSx4,4,4);

	pDestY=pFilter->pCsData[0]+((iMbY*iLineSize+iMbX)<<4);
	iCurQp=pCurDqLayer->pLumaQp[iMbXyIndex];

	*(uint32_t*)uiBSx4=0x03030303;

	// luma v
	if(iBoundryFlag&LEFT_FLAG_MASK){
		pFilter->iLumaQP=(iCurQp+pCurDqLayer->pLumaQp[iMbXyIndex-1]+1)>>1;
		FilteringEdgeLumaIntraV(pFilter,pDestY,iLineSize,NULL);
	}
	pFilter->iLumaQP=iCurQp;
	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
	if(iAlpha|iBeta){
		TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,0);

		if(!pCurDqLayer->pTransformSize8x8Flag[iMbXyIndex]){
			pFilter->pLoopf->pfLumaDeblockingLT4Hor(&pDestY[1<<2],iLineSize,iAlpha,iBeta,iTc);
		}

		pFilter->pLoopf->pfLumaDeblockingLT4Hor(&pDestY[2<<2],iLineSize,iAlpha,iBeta,iTc);

		if(!pCurDqLayer->pTransformSize8x8Flag[iMbXyIndex]){
			pFilter->pLoopf->pfLumaDeblockingLT4Hor(&pDestY[3<<2],iLineSize,iAlpha,iBeta,iTc);
		}
	}

	// luma h
	if(iBoundryFlag&TOP_FLAG_MASK){
		pFilter->iLumaQP=(iCurQp+pCurDqLayer->pLumaQp[iMbXyIndex-iMbWidth]+1)>>1;
		FilteringEdgeLumaIntraH(pFilter,pDestY,iLineSize,NULL);
	}

	pFilter->iLumaQP=iCurQp;
	if(iAlpha|iBeta){
		if(!pCurDqLayer->pTransformSize8x8Flag[iMbXyIndex]){
			pFilter->pLoopf->pfLumaDeblockingLT4Ver(&pDestY[(1<<2)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
		}

		pFilter->pLoopf->pfLumaDeblockingLT4Ver(&pDestY[(2<<2)*iLineSize],iLineSize,iAlpha,iBeta,iTc);

		if(!pCurDqLayer->pTransformSize8x8Flag[iMbXyIndex]){
			pFilter->pLoopf->pfLumaDeblockingLT4Ver(&pDestY[(3<<2)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
		}
	}
}
void FilteringEdgeChromaIntraV(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){		// QP of cb and cr are the same




		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			pFilter->pLoopf->pfChromaDeblockingEQ4Hor(pPixCb,pPixCr,iStride,iAlpha,iBeta);
		}
	}else{

		for(int i=0; i<2; i++){


			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				uint8_t* pPixCbCr=(i==0) ? pPixCb : pPixCr;
				pFilter->pLoopf->pfChromaDeblockingEQ4Hor2(pPixCbCr,iStride,iAlpha,iBeta);
			}
		}

	}
	return;
}

void FilteringEdgeChromaIntraH(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){

		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);

		if(iAlpha|iBeta){
			pFilter->pLoopf->pfChromaDeblockingEQ4Ver(pPixCb,pPixCr,iStride,iAlpha,iBeta);
		}
	}else{

		for(int i=0; i<2; i++){

			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);

			if(iAlpha|iBeta){
				uint8_t* pPixCbCr=(i==0) ? pPixCb : pPixCr;
				pFilter->pLoopf->pfChromaDeblockingEQ4Ver2(pPixCbCr,iStride,iAlpha,iBeta);
			}

		}
	}
	return;
}


void FilteringEdgeChromaHV(PDqLayer pCurDqLayer,SDeblockingFilter* pFilter,int32_t iBoundryFlag){
	int32_t iMbXyIndex=pCurDqLayer->iMbXyIndex;
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;
	int32_t iMbWidth=pCurDqLayer->iMbWidth;
	int32_t iLineSize=pFilter->iCsStride[1];

	uint8_t* pDestCb;
	uint8_t* pDestCr;
	// int32_t iCurQp;
	int8_t* pCurQp;
	int32_t iIndexA,iAlpha,iBeta;

	ENFORCE_STACK_ALIGN_1D(int8_t,iTc,4,16);
	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBSx4,4,4);

	pDestCb=pFilter->pCsData[1]+((iMbY*iLineSize+iMbX)<<3);
	pDestCr=pFilter->pCsData[2]+((iMbY*iLineSize+iMbX)<<3);
	pCurQp=pCurDqLayer->pChromaQp[iMbXyIndex];

	*(uint32_t*)uiBSx4=0x03030303;


	// chroma v
	if(iBoundryFlag&LEFT_FLAG_MASK){

		for(int i=0; i<2; i++){
			pFilter->iChromaQP[i]=(pCurQp[i]+pCurDqLayer->pChromaQp[iMbXyIndex-1][i]+1)>>1;

		}
		FilteringEdgeChromaIntraV(pFilter,pDestCb,pDestCr,iLineSize,NULL);
	}

	pFilter->iChromaQP[0]=pCurQp[0];
	pFilter->iChromaQP[1]=pCurQp[1];
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,1);
			pFilter->pLoopf->pfChromaDeblockingLT4Hor(&pDestCb[2<<1],&pDestCr[2<<1],iLineSize,iAlpha,iBeta,iTc);
		}
	}else{

		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				uint8_t* pDestCbCr=(i==0) ? &pDestCb[2<<1] : &pDestCr[2<<1];
				TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,1);
				pFilter->pLoopf->pfChromaDeblockingLT4Hor2(pDestCbCr,iLineSize,iAlpha,iBeta,iTc);
			}

		}
	}

	// chroma h

	if(iBoundryFlag&TOP_FLAG_MASK){
		for(int i=0; i<2; i++){
			pFilter->iChromaQP[i]=(pCurQp[i]+pCurDqLayer->pChromaQp[iMbXyIndex-iMbWidth][i]+1)>>1;
		}
		FilteringEdgeChromaIntraH(pFilter,pDestCb,pDestCr,iLineSize,NULL);
	}

	pFilter->iChromaQP[0]=pCurQp[0];
	pFilter->iChromaQP[1]=pCurQp[1];

	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,1);
			pFilter->pLoopf->pfChromaDeblockingLT4Ver(&pDestCb[(2<<1)*iLineSize],&pDestCr[(2<<1)*iLineSize],iLineSize,iAlpha,iBeta,iTc);
		}
	}else{
		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				TC0_TBL_LOOKUP(iTc,iIndexA,uiBSx4,1);
				uint8_t* pDestCbCr=(i==0) ? &pDestCb[(2<<1)*iLineSize] : &pDestCr[(2<<1)*iLineSize];
				pFilter->pLoopf->pfChromaDeblockingLT4Ver2(pDestCbCr,iLineSize,iAlpha,iBeta,iTc);
			}
		}


	}
}

// merge h&v lookup table operation to save performance
static void DeblockingIntraMb(PDqLayer pCurDqLayer,SDeblockingFilter* pFilter,int32_t iBoundryFlag){
	FilteringEdgeLumaHV(pCurDqLayer,pFilter,iBoundryFlag);
	FilteringEdgeChromaHV(pCurDqLayer,pFilter,iBoundryFlag);
}

inline int8_t* GetPNzc(PDqLayer pCurDqLayer,int32_t iMbXy){
	if(pCurDqLayer->pDec!=NULL && pCurDqLayer->pDec->pNzc!=NULL){
		return pCurDqLayer->pDec->pNzc[iMbXy];
	}
	return pCurDqLayer->pNzc[iMbXy];
}

uint32_t DeblockingBSliceBsMarginalMBAvcbase(SDeblockingFilter* pFilter,PDqLayer pCurDqLayer,int32_t iEdge,int32_t iNeighMb,int32_t iMbXy){
	int32_t i,j;
	uint32_t uiBSx4;
	uint8_t* pBS=(uint8_t*)(&uiBSx4);
	const uint8_t* pBIdx=&g_kuiTableBIdx[iEdge][0];
	const uint8_t* pBnIdx=&g_kuiTableBIdx[iEdge][4];
	const uint8_t* pB8x8Idx=&g_kuiTableB8x8Idx[iEdge][0];
	const uint8_t* pBn8x8Idx=&g_kuiTableB8x8Idx[iEdge][8];
	
	if(pCurDqLayer->pTransformSize8x8Flag[iMbXy] && pCurDqLayer->pTransformSize8x8Flag[iNeighMb]){
		for(i=0; i<2; i++){
			uint8_t uiNzc=0;
			for(j=0; uiNzc==0 && j<4; j++){
				uiNzc|=(GetPNzc(pCurDqLayer,iMbXy)[*(pB8x8Idx+j)]|GetPNzc(pCurDqLayer,iNeighMb)[*(pBn8x8Idx+j)]);
			}
			if(uiNzc){
				pBS[i<<1]=pBS[1+(i<<1)]=2;
			}else{
				pBS[i<<1]=pBS[1+(i<<1)]=1;
				for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
					if(pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][*pB8x8Idx]>REF_NOT_IN_LIST && pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][*pBn8x8Idx]>REF_NOT_IN_LIST){
						int8_t(*iRefIdx)[MB_BLOCK4x4_NUM]=pCurDqLayer->pDec->pRefIndex[listIdx];
						SPicture* ref0=pFilter->pRefPics[listIdx][iRefIdx[iMbXy][*pB8x8Idx]];
						SPicture* ref1=pFilter->pRefPics[listIdx][iRefIdx[iNeighMb][*pBn8x8Idx]];
						pBS[i<<1]=pBS[1+(i<<1)]=MB_BS_MV(ref0,ref1,pCurDqLayer->pDec->pMv[listIdx],iMbXy,iNeighMb,*pB8x8Idx,*pBn8x8Idx);
						break;
					}
				}
			}
			pB8x8Idx+=4;
			pBn8x8Idx+=4;
		}
	}else
	if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
		for(i=0; i<2; i++){
			uint8_t uiNzc=0;
			for(j=0; uiNzc==0 && j<4; j++){
				uiNzc|=GetPNzc(pCurDqLayer,iMbXy)[*(pB8x8Idx+j)];
			}
			for(j=0; j<2; j++){
				if(uiNzc|GetPNzc(pCurDqLayer,iNeighMb)[*pBnIdx]){
					pBS[j+(i<<1)]=2;
				}else{
					pBS[j+(i<<1)]=1;
					for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
						if(pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][*pB8x8Idx]>REF_NOT_IN_LIST && pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][*pBnIdx]>REF_NOT_IN_LIST){
							int8_t(*iRefIdx)[MB_BLOCK4x4_NUM]=pCurDqLayer->pDec->pRefIndex[listIdx];
							SPicture* ref0=pFilter->pRefPics[listIdx][iRefIdx[iMbXy][*pB8x8Idx]];
							SPicture* ref1=pFilter->pRefPics[listIdx][iRefIdx[iNeighMb][*pBnIdx]];
							pBS[j+(i<<1)]=MB_BS_MV(ref0,ref1,pCurDqLayer->pDec->pMv[listIdx],iMbXy,iNeighMb,*pB8x8Idx,*pBnIdx);
							break;
						}
					}
				}
				pBnIdx++;
			}
			pB8x8Idx+=4;
		}
	}else
	if(pCurDqLayer->pTransformSize8x8Flag[iNeighMb]){
		for(i=0; i<2; i++){
			uint8_t uiNzc=0;
			for(j=0; uiNzc==0 && j<4; j++){
				uiNzc|=GetPNzc(pCurDqLayer,iNeighMb)[*(pBn8x8Idx+j)];
			}
			for(j=0; j<2; j++){
				if(uiNzc|GetPNzc(pCurDqLayer,iMbXy)[*pBIdx]){
					pBS[j+(i<<1)]=2;
				}else{
					pBS[j+(i<<1)]=1;
					for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
						if(pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][*pBIdx]>REF_NOT_IN_LIST && pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][*pBn8x8Idx]>REF_NOT_IN_LIST){
							int8_t(*iRefIdx)[MB_BLOCK4x4_NUM]=pCurDqLayer->pDec->pRefIndex[listIdx];
							SPicture* ref0=pFilter->pRefPics[listIdx][iRefIdx[iMbXy][*pBIdx]];
							SPicture* ref1=pFilter->pRefPics[listIdx][iRefIdx[iNeighMb][*pBn8x8Idx]];
							pBS[j+(i<<1)]=MB_BS_MV(ref0,ref1,pCurDqLayer->pDec->pMv[listIdx],iMbXy,iNeighMb,*pBIdx,*pBn8x8Idx);
							break;
						}
					}
				}
				pBIdx++;
			}
			pBn8x8Idx+=4;
		}
	}else{
		// only 4x4 transform
		for(i=0; i<4; i++){
			if(GetPNzc(pCurDqLayer,iMbXy)[*pBIdx]|GetPNzc(pCurDqLayer,iNeighMb)[*pBnIdx]){
				pBS[i]=2;
			}else{
				pBS[i]=1;
				for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
					if(pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][*pBIdx]>REF_NOT_IN_LIST && pCurDqLayer->pDec->pRefIndex[listIdx][iMbXy][*pBnIdx]>REF_NOT_IN_LIST){
						int8_t(*iRefIdx)[MB_BLOCK4x4_NUM]=pCurDqLayer->pDec->pRefIndex[listIdx];
						SPicture* ref0=pFilter->pRefPics[listIdx][iRefIdx[iMbXy][*pBIdx]];
						SPicture* ref1=pFilter->pRefPics[listIdx][iRefIdx[iNeighMb][*pBnIdx]];
						pBS[i]=MB_BS_MV(ref0,ref1,pCurDqLayer->pDec->pMv[listIdx],iMbXy,iNeighMb,*pBIdx,*pBnIdx);
						break;
					}
				}
			}
			pBIdx++;
			pBnIdx++;
		}
	}

	return uiBSx4;
}
uint32_t DeblockingBsMarginalMBAvcbase(SDeblockingFilter* pFilter,PDqLayer pCurDqLayer,int32_t iEdge,int32_t iNeighMb,int32_t iMbXy){
	int32_t i,j;
	uint32_t uiBSx4;
	uint8_t* pBS=(uint8_t*)(&uiBSx4);
	const uint8_t* pBIdx=&g_kuiTableBIdx[iEdge][0];
	const uint8_t* pBnIdx=&g_kuiTableBIdx[iEdge][4];
	const uint8_t* pB8x8Idx=&g_kuiTableB8x8Idx[iEdge][0];
	const uint8_t* pBn8x8Idx=&g_kuiTableB8x8Idx[iEdge][8];
	int8_t(*iRefIdx)[MB_BLOCK4x4_NUM]=pCurDqLayer->pDec!=NULL ? pCurDqLayer->pDec->pRefIndex[LIST_0] :pCurDqLayer->pRefIndex[LIST_0];

	if(pCurDqLayer->pTransformSize8x8Flag[iMbXy] && pCurDqLayer->pTransformSize8x8Flag[iNeighMb]){
		for(i=0; i<2; i++){
			uint8_t uiNzc=0;
			for(j=0; uiNzc==0 && j<4; j++){
				uiNzc|=(GetPNzc(pCurDqLayer,iMbXy)[*(pB8x8Idx+j)]|GetPNzc(pCurDqLayer,iNeighMb)[*(pBn8x8Idx+j)]);
			}
			if(uiNzc){
				pBS[i<<1]=pBS[1+(i<<1)]=2;
			}else{
				SPicture* ref0=(iRefIdx[iMbXy][*pB8x8Idx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iMbXy][*pB8x8Idx]] : NULL;
				SPicture* ref1=(iRefIdx[iNeighMb][*pBn8x8Idx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iNeighMb][*pBn8x8Idx]] :NULL;
				pBS[i<<1]=pBS[1+(i<<1)]=MB_BS_MV(ref0,ref1,pCurDqLayer->pDec->pMv[LIST_0],iMbXy,iNeighMb,*pB8x8Idx,*pBn8x8Idx);
			}
			pB8x8Idx+=4;
			pBn8x8Idx+=4;
		}
	}else
	if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
		for(i=0; i<2; i++){
			uint8_t uiNzc=0;
			for(j=0; uiNzc==0 && j<4; j++){
				uiNzc|=GetPNzc(pCurDqLayer,iMbXy)[*(pB8x8Idx+j)];
			}
			for(j=0; j<2; j++){
				if(uiNzc|GetPNzc(pCurDqLayer,iNeighMb)[*pBnIdx]){
					pBS[j+(i<<1)]=2;
				}else{
					SPicture* ref0=(iRefIdx[iMbXy][*pB8x8Idx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iMbXy][*pB8x8Idx]] : NULL;
					SPicture* ref1=(iRefIdx[iNeighMb][*pBnIdx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iNeighMb][*pBnIdx]] : NULL;
					pBS[j+(i<<1)]=MB_BS_MV(ref0,ref1,(pCurDqLayer->pDec!=NULL ? pCurDqLayer->pDec->pMv[LIST_0] : pCurDqLayer->pMv[LIST_0]),iMbXy,iNeighMb,*pB8x8Idx,*pBnIdx);
				}
				pBnIdx++;
			}
			pB8x8Idx+=4;
		}
	}else
	if(pCurDqLayer->pTransformSize8x8Flag[iNeighMb]){
		for(i=0; i<2; i++){
			uint8_t uiNzc=0;
			for(j=0; uiNzc==0 && j<4; j++){
				uiNzc|=GetPNzc(pCurDqLayer,iNeighMb)[*(pBn8x8Idx+j)];
			}
			for(j=0; j<2; j++){
				if(uiNzc|GetPNzc(pCurDqLayer,iMbXy)[*pBIdx]){
					pBS[j+(i<<1)]=2;
				}else{
					SPicture* ref0=(iRefIdx[iMbXy][*pBIdx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iMbXy][*pBIdx]] : NULL;
					SPicture* ref1=(iRefIdx[iNeighMb][*pBn8x8Idx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iNeighMb][*pBn8x8Idx]] :NULL;
					pBS[j+(i<<1)]=MB_BS_MV(ref0,ref1,(pCurDqLayer->pDec!=NULL ? pCurDqLayer->pDec->pMv[LIST_0] : pCurDqLayer->pMv[LIST_0]),iMbXy,iNeighMb,*pBIdx,*pBn8x8Idx);
				}
				pBIdx++;
			}
			pBn8x8Idx+=4;
		}
	}else{
		// only 4x4 transform
		for(i=0; i<4; i++){
			if(GetPNzc(pCurDqLayer,iMbXy)[*pBIdx]|GetPNzc(pCurDqLayer,iNeighMb)[*pBnIdx]){
				pBS[i]=2;
			}else{
				SPicture* ref0=(iRefIdx[iMbXy][*pBIdx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iMbXy][*pBIdx]] : NULL;
				SPicture* ref1=(iRefIdx[iNeighMb][*pBnIdx]>REF_NOT_IN_LIST) ? pFilter->pRefPics[LIST_0][iRefIdx[iNeighMb][*pBnIdx]] : NULL;
				pBS[i]=MB_BS_MV(ref0,ref1,(pCurDqLayer->pDec!=NULL ? pCurDqLayer->pDec->pMv[LIST_0] : pCurDqLayer->pMv[LIST_0]),iMbXy,iNeighMb,*pBIdx,*pBnIdx);
			}
			pBIdx++;
			pBnIdx++;
		}
	}

	return uiBSx4;
}

void inline DeblockingBSInsideMBAvsbase(int8_t* pNnzTab,uint8_t nBS[2][4][4],int32_t iLShiftFactor){
	uint32_t uiNnz32b0,uiNnz32b1,uiNnz32b2,uiNnz32b3;

	uiNnz32b0=*(uint32_t*)(pNnzTab+0);
	uiNnz32b1=*(uint32_t*)(pNnzTab+4);
	uiNnz32b2=*(uint32_t*)(pNnzTab+8);
	uiNnz32b3=*(uint32_t*)(pNnzTab+12);

	nBS[0][1][0]=(pNnzTab[0]|pNnzTab[1])<<iLShiftFactor;
	nBS[0][2][0]=(pNnzTab[1]|pNnzTab[2])<<iLShiftFactor;
	nBS[0][3][0]=(pNnzTab[2]|pNnzTab[3])<<iLShiftFactor;

	nBS[0][1][1]=(pNnzTab[4]|pNnzTab[5])<<iLShiftFactor;
	nBS[0][2][1]=(pNnzTab[5]|pNnzTab[6])<<iLShiftFactor;
	nBS[0][3][1]=(pNnzTab[6]|pNnzTab[7])<<iLShiftFactor;
	*(uint32_t*)nBS[1][1]=(uiNnz32b0|uiNnz32b1)<<iLShiftFactor;

	nBS[0][1][2]=(pNnzTab[8]|pNnzTab[9])<<iLShiftFactor;
	nBS[0][2][2]=(pNnzTab[9]|pNnzTab[10])<<iLShiftFactor;
	nBS[0][3][2]=(pNnzTab[10]|pNnzTab[11])<<iLShiftFactor;
	*(uint32_t*)nBS[1][2]=(uiNnz32b1|uiNnz32b2)<<iLShiftFactor;

	nBS[0][1][3]=(pNnzTab[12]|pNnzTab[13])<<iLShiftFactor;
	nBS[0][2][3]=(pNnzTab[13]|pNnzTab[14])<<iLShiftFactor;
	nBS[0][3][3]=(pNnzTab[14]|pNnzTab[15])<<iLShiftFactor;
	*(uint32_t*)nBS[1][3]=(uiNnz32b2|uiNnz32b3)<<iLShiftFactor;
}

// // // pNonZeroCount[16+8] mapping scan index
const uint8_t g_kuiMbCountScan4Idx[24]={
	// 0 1 | 4 5 luma 8*8 block pNonZeroCount[16+8]
	0,1,4,5,		// 2 3 | 6 7 0 | 1 0 1 2 3
	2,3,6,7,		// --------------- --------- 4 5 6 7
	8,9,12,13,		// 8 9 | 12 13 2 | 3 8 9 10 11
	10,11,14,15,		// 10 11 | 14 15-----------------------------> 12 13 14 15
	16,17,20,21,		// ---------------- chroma 8*8 block 16 17 18 19
	18,19,22,23		// 16 17 | 20 21 0 1 20 21 22 23
};

void inline DeblockingBSInsideMBAvsbase8x8(int8_t* pNnzTab,uint8_t nBS[2][4][4],int32_t iLShiftFactor){
	int8_t i8x8NnzTab[4];
	for(int32_t i=0; i<4; i++){
		int32_t iBlkIdx=i<<2;
		i8x8NnzTab[i]=(pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+1]]|
						 pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+2]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+3]]);
	}

	// vertical
	nBS[0][2][0]=nBS[0][2][1]=(i8x8NnzTab[0]|i8x8NnzTab[1])<<iLShiftFactor;
	nBS[0][2][2]=nBS[0][2][3]=(i8x8NnzTab[2]|i8x8NnzTab[3])<<iLShiftFactor;
	// horizontal
	nBS[1][2][0]=nBS[1][2][1]=(i8x8NnzTab[0]|i8x8NnzTab[2])<<iLShiftFactor;
	nBS[1][2][2]=nBS[1][2][3]=(i8x8NnzTab[1]|i8x8NnzTab[3])<<iLShiftFactor;
}

void static inline DeblockingBSliceBSInsideMBNormal(SDeblockingFilter* pFilter,PDqLayer pCurDqLayer,uint8_t nBS[2][4][4],int8_t* pNnzTab,int32_t iMbXy){
	uint32_t uiNnz32b0,uiNnz32b1,uiNnz32b2,uiNnz32b3;
	void* iRefs[LIST_A][MB_BLOCK4x4_NUM];

	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBsx4,4,4);
	int8_t i8x8NnzTab[4];
	int l;

	for(l=0; l<LIST_A; l++){
		int8_t* iRefIdx=pCurDqLayer->pDec->pRefIndex[l][iMbXy];
		int i;
		// Look up each reference picture based on indices
		for(i=0; i<MB_BLOCK4x4_NUM; i++){
			if(iRefIdx[i]>REF_NOT_IN_LIST)
				iRefs[l][i]=pFilter->pRefPics[l][iRefIdx[i]];
			else
				iRefs[l][i]=NULL;
		}
	}

	if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
		for(int32_t i=0; i<4; i++){
			int32_t iBlkIdx=i<<2;
			i8x8NnzTab[i]=(pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+1]]|
							 pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+2]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+3]]);
		}
		// vertical
		int8_t iIndex=g_kuiMbCountScan4Idx[1<<2];
		int8_t iNeigborIndex=g_kuiMbCountScan4Idx[0];
		nBS[0][2][0]=nBS[0][2][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][iIndex] && iRefs[listIdx][iNeigborIndex]){
				nBS[0][2][0]=nBS[0][2][1]=BS_EDGE((i8x8NnzTab[0]|i8x8NnzTab[1]),iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],iIndex,iNeigborIndex);
				break;
			}
		}
		iIndex=g_kuiMbCountScan4Idx[3<<2];
		iNeigborIndex=g_kuiMbCountScan4Idx[2<<2];
		nBS[0][2][2]=nBS[0][2][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][iIndex] && iRefs[listIdx][iNeigborIndex]){
				nBS[0][2][2]=nBS[0][2][3]=BS_EDGE((i8x8NnzTab[2]|i8x8NnzTab[3]),iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],iIndex,iNeigborIndex);
				break;
			}
		}

		// horizontal
		iIndex=g_kuiMbCountScan4Idx[2<<2];
		iNeigborIndex=g_kuiMbCountScan4Idx[0];
		nBS[1][2][0]=nBS[1][2][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][iIndex] && iRefs[listIdx][iNeigborIndex]){
				nBS[1][2][0]=nBS[1][2][1]=BS_EDGE((i8x8NnzTab[0]|i8x8NnzTab[2]),iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],iIndex,iNeigborIndex);
				break;
			}
		}

		iIndex=g_kuiMbCountScan4Idx[3<<2];
		iNeigborIndex=g_kuiMbCountScan4Idx[1<<2];
		nBS[1][2][2]=nBS[1][2][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][iIndex] && iRefs[listIdx][iNeigborIndex]){
				nBS[1][2][2]=nBS[1][2][3]=BS_EDGE((i8x8NnzTab[1]|i8x8NnzTab[3]),iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],iIndex,iNeigborIndex);
				break;
			}
		}
	}else{
		uiNnz32b0=*(uint32_t*)(pNnzTab+0);
		uiNnz32b1=*(uint32_t*)(pNnzTab+4);
		uiNnz32b2=*(uint32_t*)(pNnzTab+8);
		uiNnz32b3=*(uint32_t*)(pNnzTab+12);

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[i]|pNnzTab[i+1];
		nBS[0][1][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][1] && iRefs[listIdx][0]){
				nBS[0][1][0]=BS_EDGE(uiBsx4[0],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],1,0);
				break;
			}
		}
		nBS[0][2][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][2] && iRefs[listIdx][1]){
				nBS[0][2][0]=BS_EDGE(uiBsx4[1],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],2,1);
				break;
			}
		}
		nBS[0][3][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][3] && iRefs[listIdx][2]){
				nBS[0][3][0]=BS_EDGE(uiBsx4[2],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],3,2);
				break;
			}
		}

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[4+i]|pNnzTab[4+i+1];
		nBS[0][1][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][5] && iRefs[listIdx][4]){
				nBS[0][1][1]=BS_EDGE(uiBsx4[0],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],5,4);
				break;
			}
		}
		nBS[0][2][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][6] && iRefs[listIdx][5]){
				nBS[0][2][1]=BS_EDGE(uiBsx4[1],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],6,5);
				break;
			}
		}
		nBS[0][3][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][7] && iRefs[listIdx][6]){
				nBS[0][3][1]=BS_EDGE(uiBsx4[2],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],7,6);
				break;
			}
		}

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[8+i]|pNnzTab[8+i+1];
		nBS[0][1][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][9] && iRefs[listIdx][8]){
				nBS[0][1][2]=BS_EDGE(uiBsx4[0],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],9,8);
				break;
			}
		}
		nBS[0][2][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][10] && iRefs[listIdx][9]){
				nBS[0][2][2]=BS_EDGE(uiBsx4[1],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],10,9);
				break;
			}
		}
		nBS[0][3][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][11] && iRefs[listIdx][10]){
				nBS[0][3][2]=BS_EDGE(uiBsx4[2],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],11,10);
				break;
			}
		}

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[12+i]|pNnzTab[12+i+1];
		nBS[0][1][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][13] && iRefs[listIdx][12]){
				nBS[0][1][3]=BS_EDGE(uiBsx4[0],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],13,12);
				break;
			}
		}
		nBS[0][2][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][14] && iRefs[listIdx][13]){
				nBS[0][2][3]=BS_EDGE(uiBsx4[1],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],14,13);
				break;
			}
		}
		nBS[0][3][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][15] && iRefs[listIdx][14]){
				nBS[0][3][3]=BS_EDGE(uiBsx4[2],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],15,14);
				break;
			}
		}

		// horizontal
		*(uint32_t*)uiBsx4=(uiNnz32b0|uiNnz32b1);
		nBS[1][1][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][4] && iRefs[listIdx][0]){
				nBS[1][1][0]=BS_EDGE(uiBsx4[0],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],4,0);
				break;
			}
		}
		nBS[1][1][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][5] && iRefs[listIdx][1]){
				nBS[1][1][1]=BS_EDGE(uiBsx4[1],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],5,1);
				break;
			}
		}
		nBS[1][1][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][6] && iRefs[listIdx][2]){
				nBS[1][1][2]=BS_EDGE(uiBsx4[2],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],6,2);
				break;
			}
		}
		nBS[1][1][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][7] && iRefs[listIdx][3]){
				nBS[1][1][3]=BS_EDGE(uiBsx4[3],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],7,3);
				break;
			}
		}

		*(uint32_t*)uiBsx4=(uiNnz32b1|uiNnz32b2);
		nBS[1][2][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][8] && iRefs[listIdx][4]){
				nBS[1][2][0]=BS_EDGE(uiBsx4[0],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],8,4);
				break;
			}
		}
		nBS[1][2][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][9] && iRefs[listIdx][5]){
				nBS[1][2][1]=BS_EDGE(uiBsx4[1],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],9,5);
				break;
			}
		}
		nBS[1][2][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][10] && iRefs[listIdx][6]){
				nBS[1][2][2]=BS_EDGE(uiBsx4[2],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],10,6);
				break;
			}
		}
		nBS[1][2][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][11] && iRefs[listIdx][7]){
				nBS[1][2][3]=BS_EDGE(uiBsx4[3],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],11,7);
				break;
			}
		}

		*(uint32_t*)uiBsx4=(uiNnz32b2|uiNnz32b3);
		nBS[1][3][0]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][12] && iRefs[listIdx][8]){
				nBS[1][3][0]=BS_EDGE(uiBsx4[0],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],12,8);
				break;
			}
		}
		nBS[1][3][1]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][13] && iRefs[listIdx][9]){
				nBS[1][3][1]=BS_EDGE(uiBsx4[1],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],13,9);
				break;
			}
		}
		nBS[1][3][2]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][14] && iRefs[listIdx][10]){
				nBS[1][3][2]=BS_EDGE(uiBsx4[2],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],14,10);
				break;
			}
		}
		nBS[1][3][3]=1;
		for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
			if(iRefs[listIdx][15] && iRefs[listIdx][11]){
				nBS[1][3][3]=BS_EDGE(uiBsx4[3],iRefs[listIdx],pCurDqLayer->pDec->pMv[listIdx][iMbXy],15,11);
				break;
			}
		}
	}
}

void static inline DeblockingBSInsideMBNormal(SDeblockingFilter* pFilter,PDqLayer pCurDqLayer,uint8_t nBS[2][4][4],int8_t* pNnzTab,int32_t iMbXy){
	uint32_t uiNnz32b0,uiNnz32b1,uiNnz32b2,uiNnz32b3;
	int8_t* iRefIdx=pCurDqLayer->pDec->pRefIndex[LIST_0][iMbXy];
	void* iRefs[MB_BLOCK4x4_NUM];
	int i;
	ENFORCE_STACK_ALIGN_1D(uint8_t,uiBsx4,4,4);

	int8_t i8x8NnzTab[4];

	// Look up each reference picture based on indices
	for(i=0; i<MB_BLOCK4x4_NUM; i++){
		if(iRefIdx[i]>REF_NOT_IN_LIST)
			iRefs[i]=pFilter->pRefPics[LIST_0][iRefIdx[i]];
		else
			iRefs[i]=NULL;
	}

	if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
		for(int32_t i=0; i<4; i++){
			int32_t iBlkIdx=i<<2;
			i8x8NnzTab[i]=(pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+1]]|
							 pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+2]]|pNnzTab[g_kuiMbCountScan4Idx[iBlkIdx+3]]);
		}
		// vertical
		nBS[0][2][0]=nBS[0][2][1]=BS_EDGE((i8x8NnzTab[0]|i8x8NnzTab[1]),iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],g_kuiMbCountScan4Idx[1<<2],g_kuiMbCountScan4Idx[0]);
		nBS[0][2][2]=nBS[0][2][3]=BS_EDGE((i8x8NnzTab[2]|i8x8NnzTab[3]),iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],g_kuiMbCountScan4Idx[3<<2],g_kuiMbCountScan4Idx[2<<2]);

		// horizontal
		nBS[1][2][0]=nBS[1][2][1]=BS_EDGE((i8x8NnzTab[0]|i8x8NnzTab[2]),iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],g_kuiMbCountScan4Idx[2<<2],g_kuiMbCountScan4Idx[0]);
		nBS[1][2][2]=nBS[1][2][3]=BS_EDGE((i8x8NnzTab[1]|i8x8NnzTab[3]),iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],g_kuiMbCountScan4Idx[3<<2],g_kuiMbCountScan4Idx[1<<2]);
	}else{
		uiNnz32b0=*(uint32_t*)(pNnzTab+0);
		uiNnz32b1=*(uint32_t*)(pNnzTab+4);
		uiNnz32b2=*(uint32_t*)(pNnzTab+8);
		uiNnz32b3=*(uint32_t*)(pNnzTab+12);

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[i]|pNnzTab[i+1];
		nBS[0][1][0]=BS_EDGE(uiBsx4[0],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],1,0);
		nBS[0][2][0]=BS_EDGE(uiBsx4[1],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],2,1);
		nBS[0][3][0]=BS_EDGE(uiBsx4[2],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],3,2);

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[4+i]|pNnzTab[4+i+1];
		nBS[0][1][1]=BS_EDGE(uiBsx4[0],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],5,4);
		nBS[0][2][1]=BS_EDGE(uiBsx4[1],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],6,5);
		nBS[0][3][1]=BS_EDGE(uiBsx4[2],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],7,6);

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[8+i]|pNnzTab[8+i+1];
		nBS[0][1][2]=BS_EDGE(uiBsx4[0],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],9,8);
		nBS[0][2][2]=BS_EDGE(uiBsx4[1],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],10,9);
		nBS[0][3][2]=BS_EDGE(uiBsx4[2],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],11,10);

		for(int i=0; i<3; i++)
			uiBsx4[i]=pNnzTab[12+i]|pNnzTab[12+i+1];
		nBS[0][1][3]=BS_EDGE(uiBsx4[0],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],13,12);
		nBS[0][2][3]=BS_EDGE(uiBsx4[1],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],14,13);
		nBS[0][3][3]=BS_EDGE(uiBsx4[2],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],15,14);

		// horizontal
		*(uint32_t*)uiBsx4=(uiNnz32b0|uiNnz32b1);
		nBS[1][1][0]=BS_EDGE(uiBsx4[0],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],4,0);
		nBS[1][1][1]=BS_EDGE(uiBsx4[1],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],5,1);
		nBS[1][1][2]=BS_EDGE(uiBsx4[2],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],6,2);
		nBS[1][1][3]=BS_EDGE(uiBsx4[3],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],7,3);

		*(uint32_t*)uiBsx4=(uiNnz32b1|uiNnz32b2);
		nBS[1][2][0]=BS_EDGE(uiBsx4[0],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],8,4);
		nBS[1][2][1]=BS_EDGE(uiBsx4[1],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],9,5);
		nBS[1][2][2]=BS_EDGE(uiBsx4[2],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],10,6);
		nBS[1][2][3]=BS_EDGE(uiBsx4[3],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],11,7);

		*(uint32_t*)uiBsx4=(uiNnz32b2|uiNnz32b3);
		nBS[1][3][0]=BS_EDGE(uiBsx4[0],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],12,8);
		nBS[1][3][1]=BS_EDGE(uiBsx4[1],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],13,9);
		nBS[1][3][2]=BS_EDGE(uiBsx4[2],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],14,10);
		nBS[1][3][3]=BS_EDGE(uiBsx4[3],iRefs,pCurDqLayer->pDec->pMv[LIST_0][iMbXy],15,11);
	}
}

void FilteringEdgeLumaV(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,tc,4,16);

	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);

	if(iAlpha|iBeta){
		TC0_TBL_LOOKUP(tc,iIndexA,pBS,0);
		pFilter->pLoopf->pfLumaDeblockingLT4Hor(pPix,iStride,iAlpha,iBeta,tc);
	}
	return;
}

void FilteringEdgeChromaV(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,tc,4,16);
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			TC0_TBL_LOOKUP(tc,iIndexA,pBS,1);
			pFilter->pLoopf->pfChromaDeblockingLT4Hor(pPixCb,pPixCr,iStride,iAlpha,iBeta,tc);
		}
	}else{
		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				uint8_t* pPixCbCr=(i==0) ? pPixCb : pPixCr;
				TC0_TBL_LOOKUP(tc,iIndexA,pBS,1);
				pFilter->pLoopf->pfChromaDeblockingLT4Hor2(pPixCbCr,iStride,iAlpha,iBeta,tc);
			}
		}
	}
	return;
}
void FilteringEdgeLumaH(SDeblockingFilter* pFilter,uint8_t* pPix,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,tc,4,16);
	GET_ALPHA_BETA_FROM_QP(pFilter->iLumaQP,pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
	if(iAlpha|iBeta){
		TC0_TBL_LOOKUP(tc,iIndexA,pBS,0);
		pFilter->pLoopf->pfLumaDeblockingLT4Ver(pPix,iStride,iAlpha,iBeta,tc);
	}
	return;
}
void FilteringEdgeChromaH(SDeblockingFilter* pFilter,uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,uint8_t* pBS){
	int32_t iIndexA;
	int32_t iAlpha;
	int32_t iBeta;
	ENFORCE_STACK_ALIGN_1D(int8_t,tc,4,16);
	if(pFilter->iChromaQP[0]==pFilter->iChromaQP[1]){
		GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[0],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
		if(iAlpha|iBeta){
			TC0_TBL_LOOKUP(tc,iIndexA,pBS,1);
			pFilter->pLoopf->pfChromaDeblockingLT4Ver(pPixCb,pPixCr,iStride,iAlpha,iBeta,tc);
		}
	}else{
		for(int i=0; i<2; i++){
			GET_ALPHA_BETA_FROM_QP(pFilter->iChromaQP[i],pFilter->iSliceAlphaC0Offset,pFilter->iSliceBetaOffset,iIndexA,iAlpha,iBeta);
			if(iAlpha|iBeta){
				uint8_t* pPixCbCr=(i==0) ? pPixCb : pPixCr;
				TC0_TBL_LOOKUP(tc,iIndexA,pBS,1);
				pFilter->pLoopf->pfChromaDeblockingLT4Ver2(pPixCbCr,iStride,iAlpha,iBeta,tc);
			}
		}
	}
	return;
}

static void DeblockingInterMb(PDqLayer pCurDqLayer,SDeblockingFilter* pFilter,uint8_t nBS[2][4][4],int32_t iBoundryFlag){
	int32_t iMbXyIndex=pCurDqLayer->iMbXyIndex;
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;

	int32_t iCurLumaQp=pCurDqLayer->pLumaQp[iMbXyIndex];
	// int32_t* iCurChromaQp=pCurDqLayer->pChromaQp[iMbXyIndex];
	int8_t* pCurChromaQp=pCurDqLayer->pChromaQp[iMbXyIndex];
	int32_t iLineSize=pFilter->iCsStride[0];
	int32_t iLineSizeUV=pFilter->iCsStride[1];

	uint8_t* pDestY,* pDestCb,* pDestCr;
	pDestY=pFilter->pCsData[0]+((iMbY*iLineSize+iMbX)<<4);
	pDestCb=pFilter->pCsData[1]+((iMbY*iLineSizeUV+iMbX)<<3);
	pDestCr=pFilter->pCsData[2]+((iMbY*iLineSizeUV+iMbX)<<3);

	// Vertical margin
	if(iBoundryFlag&LEFT_FLAG_MASK){
		int32_t iLeftXyIndex=iMbXyIndex-1;
		pFilter->iLumaQP=(iCurLumaQp+pCurDqLayer->pLumaQp[iLeftXyIndex]+1)>>1;
		for(int i=0; i<2; i++){
			pFilter->iChromaQP[i]=(pCurChromaQp[i]+pCurDqLayer->pChromaQp[iLeftXyIndex][i]+1)>>1;
		}
		if(nBS[0][0][0]==0x04){
			FilteringEdgeLumaIntraV(pFilter,pDestY,iLineSize,NULL);
			FilteringEdgeChromaIntraV(pFilter,pDestCb,pDestCr,iLineSizeUV,NULL);
		}else{
			if(*(uint32_t*)nBS[0][0]!=0){
				FilteringEdgeLumaV(pFilter,pDestY,iLineSize,nBS[0][0]);
				FilteringEdgeChromaV(pFilter,pDestCb,pDestCr,iLineSizeUV,nBS[0][0]);
			}
		}
	}

	pFilter->iLumaQP=iCurLumaQp;
	pFilter->iChromaQP[0]=pCurChromaQp[0];
	pFilter->iChromaQP[1]=pCurChromaQp[1];

	if(*(uint32_t*)nBS[0][1]!=0 && !pCurDqLayer->pTransformSize8x8Flag[iMbXyIndex]){
		FilteringEdgeLumaV(pFilter,&pDestY[1<<2],iLineSize,nBS[0][1]);
	}

	if(*(uint32_t*)nBS[0][2]!=0){
		FilteringEdgeLumaV(pFilter,&pDestY[2<<2],iLineSize,nBS[0][2]);
		FilteringEdgeChromaV(pFilter,&pDestCb[2<<1],&pDestCr[2<<1],iLineSizeUV,nBS[0][2]);
	}

	if(*(uint32_t*)nBS[0][3]!=0 && !pCurDqLayer->pTransformSize8x8Flag[iMbXyIndex]){
		FilteringEdgeLumaV(pFilter,&pDestY[3<<2],iLineSize,nBS[0][3]);
	}

	if(iBoundryFlag&TOP_FLAG_MASK){
		int32_t iTopXyIndex=iMbXyIndex-pCurDqLayer->iMbWidth;
		pFilter->iLumaQP=(iCurLumaQp+pCurDqLayer->pLumaQp[iTopXyIndex]+1)>>1;
		for(int i=0; i<2; i++){
			pFilter->iChromaQP[i]=(pCurChromaQp[i]+pCurDqLayer->pChromaQp[iTopXyIndex][i]+1)>>1;
		}

		if(nBS[1][0][0]==0x04){
			FilteringEdgeLumaIntraH(pFilter,pDestY,iLineSize,NULL);
			FilteringEdgeChromaIntraH(pFilter,pDestCb,pDestCr,iLineSizeUV,NULL);
		}else{
			if(*(uint32_t*)nBS[1][0]!=0){
				FilteringEdgeLumaH(pFilter,pDestY,iLineSize,nBS[1][0]);
				FilteringEdgeChromaH(pFilter,pDestCb,pDestCr,iLineSizeUV,nBS[1][0]);
			}
		}
	}

	pFilter->iLumaQP=iCurLumaQp;
	pFilter->iChromaQP[0]=pCurChromaQp[0];
	pFilter->iChromaQP[1]=pCurChromaQp[1];

	if(*(uint32_t*)nBS[1][1]!=0 && !pCurDqLayer->pTransformSize8x8Flag[iMbXyIndex]){
		FilteringEdgeLumaH(pFilter,&pDestY[(1<<2)*iLineSize],iLineSize,nBS[1][1]);
	}

	if(*(uint32_t*)nBS[1][2]!=0){
		FilteringEdgeLumaH(pFilter,&pDestY[(2<<2)*iLineSize],iLineSize,nBS[1][2]);
		FilteringEdgeChromaH(pFilter,&pDestCb[(2<<1)*iLineSizeUV],&pDestCr[(2<<1)*iLineSizeUV],iLineSizeUV,nBS[1][2]);
	}

	if(*(uint32_t*)nBS[1][3]!=0 && !pCurDqLayer->pTransformSize8x8Flag[iMbXyIndex]){
		FilteringEdgeLumaH(pFilter,&pDestY[(3<<2)*iLineSize],iLineSize,nBS[1][3]);
	}
}

void WelsDeblockingMb(PDqLayer pCurDqLayer,SDeblockingFilter* pFilter,int32_t iBoundryFlag){
	uint8_t nBS[2][4][4]={{{0}}};

	int32_t iMbXyIndex=pCurDqLayer->iMbXyIndex;
	uint32_t iCurMbType=pCurDqLayer->pDec!=NULL ? pCurDqLayer->pDec->pMbType[iMbXyIndex] :
		pCurDqLayer->pMbType[iMbXyIndex];
	int32_t iMbNb;

	SSlice* pSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pSlice->sSliceHeaderExt.sSliceHeader;
	bool bBSlice=pSliceHeader->eSliceType==B_SLICE;

	switch(iCurMbType){
		case MB_TYPE_INTRA4x4:
		case MB_TYPE_INTRA8x8:
		case MB_TYPE_INTRA16x16:
		case MB_TYPE_INTRA_PCM:
			DeblockingIntraMb(pCurDqLayer,pFilter,iBoundryFlag);
			break;
		default:

			if(iBoundryFlag&LEFT_FLAG_MASK){
				iMbNb=iMbXyIndex-1;
				uint32_t uiMbType=pCurDqLayer->pDec!=NULL ? pCurDqLayer->pDec->pMbType[iMbNb] : pCurDqLayer->pMbType[iMbNb];
				if(bBSlice){
					*(uint32_t*)nBS[0][0]=IS_INTRA(uiMbType) ? 0x04040404 :
						DeblockingBSliceBsMarginalMBAvcbase(
						 pFilter,pCurDqLayer,0,iMbNb,iMbXyIndex);
				}else{
					*(uint32_t*)nBS[0][0]=IS_INTRA(uiMbType) ? 0x04040404 : DeblockingBsMarginalMBAvcbase(
											 pFilter,pCurDqLayer,0,iMbNb,iMbXyIndex);
				}
			}else{
				*(uint32_t*)nBS[0][0]=0;
			}
			if(iBoundryFlag&TOP_FLAG_MASK){
				iMbNb=iMbXyIndex-pCurDqLayer->iMbWidth;
				uint32_t uiMbType=pCurDqLayer->pDec!=NULL ? pCurDqLayer->pDec->pMbType[iMbNb] : pCurDqLayer->pMbType[iMbNb];
				if(bBSlice){
					*(uint32_t*)nBS[1][0]=IS_INTRA(uiMbType) ? 0x04040404 :
						DeblockingBSliceBsMarginalMBAvcbase(
						 pFilter,pCurDqLayer,1,iMbNb,iMbXyIndex);
				}else{
					*(uint32_t*)nBS[1][0]=IS_INTRA(uiMbType) ? 0x04040404 : DeblockingBsMarginalMBAvcbase(
											 pFilter,pCurDqLayer,1,iMbNb,iMbXyIndex);
				}
			}else{
				*(uint32_t*)nBS[1][0]=0;
			}
			// SKIP MB_16x16 or others
			if(IS_SKIP(iCurMbType)){
				*(uint32_t*)nBS[0][1]=*(uint32_t*)nBS[0][2]=*(uint32_t*)nBS[0][3]=
					*(uint32_t*)nBS[1][1]=*(uint32_t*)nBS[1][2]=*(uint32_t*)nBS[1][3]=0;
			}else{
				if(IS_INTER_16x16(iCurMbType)){
					if(!pCurDqLayer->pTransformSize8x8Flag[pCurDqLayer->iMbXyIndex]){
						DeblockingBSInsideMBAvsbase(GetPNzc(pCurDqLayer,iMbXyIndex),nBS,1);
					}else{
						DeblockingBSInsideMBAvsbase8x8(GetPNzc(pCurDqLayer,iMbXyIndex),nBS,1);
					}
				}else{

					if(bBSlice){
						DeblockingBSliceBSInsideMBNormal(pFilter,pCurDqLayer,nBS,GetPNzc(pCurDqLayer,iMbXyIndex),iMbXyIndex);
					}else{
						DeblockingBSInsideMBNormal(pFilter,pCurDqLayer,nBS,GetPNzc(pCurDqLayer,iMbXyIndex),iMbXyIndex);
					}
				}
			}
			DeblockingInterMb(pCurDqLayer,pFilter,nBS,iBoundryFlag);
			break;
	}
}

void WelsFillRecNeededMbInfo(SDecoderContext* pCtx,bool bOutput,PDqLayer pCurDqLayer){
	SPicture* pCurPic=pCtx->pDec;
	int32_t iLumaStride=pCurPic->iLinesize[0];
	int32_t iChromaStride=pCurPic->iLinesize[1];
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;

	pCurDqLayer->iLumaStride=iLumaStride;
	pCurDqLayer->iChromaStride=iChromaStride;

	if(bOutput){
		pCurDqLayer->pPred[0]=pCurPic->pData[0]+((iMbY*iLumaStride+iMbX)<<4);
		pCurDqLayer->pPred[1]=pCurPic->pData[1]+((iMbY*iChromaStride+iMbX)<<3);
		pCurDqLayer->pPred[2]=pCurPic->pData[2]+((iMbY*iChromaStride+iMbX)<<3);
	}
}

int32_t RecChroma(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,PDqLayer pDqLayer){
	int32_t iChromaStride=pCtx->pCurDqLayer->pDec->iLinesize[1];
	PIdctFourResAddPredFunc pIdctFourResAddPredFunc=pCtx->pIdctFourResAddPredFunc;

	uint8_t i=0;
	uint8_t uiCbpC=pDqLayer->pCbp[iMBXY]>>4;

	if(1==uiCbpC || 2==uiCbpC){
		for(i=0; i<2; i++){
			int16_t* pRS=pScoeffLevel+256+(i<<6);
			uint8_t* pPred=pDqLayer->pPred[i+1];
			const int8_t* pNzc=pDqLayer->pNzc[iMBXY]+16+2*i;

			// 1 chroma is divided 4 4x4_block to idct
			pIdctFourResAddPredFunc(pPred,iChromaStride,pRS,pNzc);
		}
	}

	return ERR_NONE;
}

int32_t RecI16x16Mb(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,PDqLayer pDqLayer){
	// decoder use,encoder no use
	int8_t iI16x16PredMode=pDqLayer->pIntraPredMode[iMBXY][7];
	int8_t iChromaPredMode=pDqLayer->pChromaPredMode[iMBXY];
	PGetIntraPredFunc* pGetIChromaPredFunc=pCtx->pGetIChromaPredFunc;
	PGetIntraPredFunc* pGetI16x16LumaPredFunc=pCtx->pGetI16x16LumaPredFunc;
	int32_t iUVStride=pCtx->pCurDqLayer->pDec->iLinesize[1];

	// common use by decoder&encoder
	int32_t iYStride=pDqLayer->iLumaStride;
	int16_t* pRS=pScoeffLevel;

	uint8_t* pPred=pDqLayer->pPred[0];

	PIdctFourResAddPredFunc pIdctFourResAddPredFunc=pCtx->pIdctFourResAddPredFunc;

	// decode i16x16 y
	pGetI16x16LumaPredFunc[iI16x16PredMode](pPred,iYStride);

	// 1 mb is divided 16 4x4_block to idct
	const int8_t* pNzc=pDqLayer->pNzc[iMBXY];
	pIdctFourResAddPredFunc(pPred+0*iYStride+0,iYStride,pRS+0*64,pNzc+0);
	pIdctFourResAddPredFunc(pPred+0*iYStride+8,iYStride,pRS+1*64,pNzc+2);
	pIdctFourResAddPredFunc(pPred+8*iYStride+0,iYStride,pRS+2*64,pNzc+8);
	pIdctFourResAddPredFunc(pPred+8*iYStride+8,iYStride,pRS+3*64,pNzc+10);

	// decode intra mb cb&cr
	pPred=pDqLayer->pPred[1];
	pGetIChromaPredFunc[iChromaPredMode](pPred,iUVStride);
	pPred=pDqLayer->pPred[2];
	pGetIChromaPredFunc[iChromaPredMode](pPred,iUVStride);
	RecChroma(iMBXY,pCtx,pScoeffLevel,pDqLayer);

	return ERR_NONE;
}
int32_t RecI8x8Luma(int32_t iMbXy,SDecoderContext* pCtx,int16_t* pScoeffLevel,PDqLayer pDqLayer){
	// prediction info
	uint8_t* pPred=pDqLayer->pPred[0];

	int32_t iLumaStride=pDqLayer->iLumaStride;
	int32_t* pBlockOffset=pCtx->iDecBlockOffsetArray;
	PGetIntraPred8x8Func* pGetI8x8LumaPredFunc=pCtx->pGetI8x8LumaPredFunc;

	int8_t* pIntra8x8PredMode=pDqLayer->pIntra4x4FinalMode[iMbXy];		// I_NxN
	int16_t* pRS=pScoeffLevel;
	// itransform info
	PIdctResAddPredFunc pIdctResAddPredFunc=pCtx->pIdctResAddPredFunc8x8;

	uint8_t i=0;
	bool bTLAvail[4],bTRAvail[4];
	// Top-Right : Left : Top-Left : Top
	bTLAvail[0]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x02);
	bTLAvail[1]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x01);
	bTLAvail[2]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x04);
	bTLAvail[3]=true;

	bTRAvail[0]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x01);
	bTRAvail[1]=!!(pDqLayer->pIntraNxNAvailFlag[iMbXy]&0x08);
	bTRAvail[2]=true;
	bTRAvail[3]=false;

	for(i=0; i<4; i++){

		uint8_t* pPredI8x8=pPred+pBlockOffset[i<<2];
		uint8_t uiMode=pIntra8x8PredMode[g_kuiScan4[i<<2]];

		pGetI8x8LumaPredFunc[uiMode](pPredI8x8,iLumaStride,bTLAvail[i],bTRAvail[i]);

		int32_t iIndex=g_kuiMbCountScan4Idx[i<<2];
		if(pDqLayer->pNzc[iMbXy][iIndex] || pDqLayer->pNzc[iMbXy][iIndex+1] || pDqLayer->pNzc[iMbXy][iIndex+4]
			 || pDqLayer->pNzc[iMbXy][iIndex+5]){
			int16_t* pRSI8x8=&pRS[i<<6];
			pIdctResAddPredFunc(pPredI8x8,iLumaStride,pRSI8x8);
		}
	}

	return ERR_NONE;
}
int32_t RecI4x4Chroma(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,PDqLayer pDqLayer){
	int32_t iChromaStride=pCtx->pCurDqLayer->pDec->iLinesize[1];

	int8_t iChromaPredMode=pDqLayer->pChromaPredMode[iMBXY];

	PGetIntraPredFunc* pGetIChromaPredFunc=pCtx->pGetIChromaPredFunc;

	uint8_t* pPred=pDqLayer->pPred[1];

	pGetIChromaPredFunc[iChromaPredMode](pPred,iChromaStride);
	pPred=pDqLayer->pPred[2];
	pGetIChromaPredFunc[iChromaPredMode](pPred,iChromaStride);

	RecChroma(iMBXY,pCtx,pScoeffLevel,pDqLayer);

	return ERR_NONE;
}

int32_t RecI8x8Mb(int32_t iMbXy,SDecoderContext* pCtx,int16_t* pScoeffLevel,PDqLayer pDqLayer){
	RecI8x8Luma(iMbXy,pCtx,pScoeffLevel,pDqLayer);
	RecI4x4Chroma(iMbXy,pCtx,pScoeffLevel,pDqLayer);
	return ERR_NONE;
}

int32_t RecI4x4Luma(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,PDqLayer pDqLayer){
	// prediction info
	uint8_t* pPred=pDqLayer->pPred[0];

	int32_t iLumaStride=pDqLayer->iLumaStride;
	int32_t* pBlockOffset=pCtx->iDecBlockOffsetArray;
	PGetIntraPredFunc* pGetI4x4LumaPredFunc=pCtx->pGetI4x4LumaPredFunc;

	int8_t* pIntra4x4PredMode=pDqLayer->pIntra4x4FinalMode[iMBXY];
	int16_t* pRS=pScoeffLevel;
	// itransform info
	PIdctResAddPredFunc pIdctResAddPredFunc=pCtx->pIdctResAddPredFunc;


	uint8_t i=0;
	for(i=0; i<16; i++){

		uint8_t* pPredI4x4=pPred+pBlockOffset[i];
		uint8_t uiMode=pIntra4x4PredMode[g_kuiScan4[i]];

		pGetI4x4LumaPredFunc[uiMode](pPredI4x4,iLumaStride);

		if(pDqLayer->pNzc[iMBXY][g_kuiMbCountScan4Idx[i]]){
			int16_t* pRSI4x4=&pRS[i<<4];
			pIdctResAddPredFunc(pPredI4x4,iLumaStride,pRSI4x4);
		}
	}

	return ERR_NONE;
}



int32_t RecI4x4Mb(int32_t iMBXY,SDecoderContext* pCtx,int16_t* pScoeffLevel,PDqLayer pDqLayer){
	RecI4x4Luma(iMBXY,pCtx,pScoeffLevel,pDqLayer);
	RecI4x4Chroma(iMBXY,pCtx,pScoeffLevel,pDqLayer);
	return ERR_NONE;
}


int32_t WelsMbIntraPredictionConstruction(SDecoderContext* pCtx,PDqLayer pCurDqLayer,bool bOutput){
	// seems IPCM should not enter this path
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;

	WelsFillRecNeededMbInfo(pCtx,bOutput,pCurDqLayer);

	if(IS_INTRA16x16(pCurDqLayer->pDec->pMbType[iMbXy])){
		RecI16x16Mb(iMbXy,pCtx,pCurDqLayer->pScaledTCoeff[iMbXy],pCurDqLayer);
	}else
	if(IS_INTRA8x8(pCurDqLayer->pDec->pMbType[iMbXy])){
		RecI8x8Mb(iMbXy,pCtx,pCurDqLayer->pScaledTCoeff[iMbXy],pCurDqLayer);
	}else
	if(IS_INTRA4x4(pCurDqLayer->pDec->pMbType[iMbXy])){
		RecI4x4Mb(iMbXy,pCtx,pCurDqLayer->pScaledTCoeff[iMbXy],pCurDqLayer);
	}
	return ERR_NONE;
}

static bool CheckRefPics(const SDecoderContext* pCtx){
	int32_t listCount=1;
	if(pCtx->eSliceType==B_SLICE){
		++listCount;
	}
	for(int32_t list=LIST_0; list<listCount;++list){
		int32_t shortRefCount=pCtx->sRefPic.uiShortRefCount[list];
		for(int32_t refIdx=0; refIdx<shortRefCount;++refIdx){
			if(!pCtx->sRefPic.pShortRefList[list][refIdx]){
				return false;
			}
		}
		int32_t longRefCount=pCtx->sRefPic.uiLongRefCount[list];
		for(int32_t refIdx=0; refIdx<longRefCount;++refIdx){
			if(!pCtx->sRefPic.pLongRefList[list][refIdx]){
				return false;
			}
		}
	}
	return true;
}


#define WELS_B_MB_REC_VERIFY(uiRet) do{ \
 uint32_t uiRetTmp=(uint32_t)uiRet; \
 if( uiRetTmp !=ERR_NONE ) \
 return uiRetTmp; \
}while(0)

typedef struct TagMCRefMember{
	uint8_t* pDstY;
	uint8_t* pDstU;
	uint8_t* pDstV;

	uint8_t* pSrcY;
	uint8_t* pSrcU;
	uint8_t* pSrcV;

	int32_t iSrcLineLuma;
	int32_t iSrcLineChroma;

	int32_t iDstLineLuma;
	int32_t iDstLineChroma;

	int32_t iPicWidth;
	int32_t iPicHeight;
} sMCRefMember;
#ifndef MC_FLOW_SIMPLE_JUDGE
#define MC_FLOW_SIMPLE_JUDGE 1
#endif		// MC_FLOW_SIMPLE_JUDGE

// according to current 8*8 block ref_index to gain reference picture
static inline int32_t GetRefPic(sMCRefMember* pMCRefMem,SDecoderContext* pCtx,const int8_t& iRefIdx,int32_t listIdx){
	SPicture* pRefPic;

	if(iRefIdx>=0){
		pRefPic=pCtx->sRefPic.pRefList[listIdx][iRefIdx];

		if(pRefPic!=NULL){
			pMCRefMem->iSrcLineLuma=pRefPic->iLinesize[0];
			pMCRefMem->iSrcLineChroma=pRefPic->iLinesize[1];

			pMCRefMem->pSrcY=pRefPic->pData[0];
			pMCRefMem->pSrcU=pRefPic->pData[1];
			pMCRefMem->pSrcV=pRefPic->pData[2];
			if(!pMCRefMem->pSrcY || !pMCRefMem->pSrcU || !pMCRefMem->pSrcV){
				return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_DATA,ERR_INFO_REFERENCE_PIC_LOST);
			}
			return ERR_NONE;
		}
	}
	return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_DATA,ERR_INFO_REFERENCE_PIC_LOST);
}

void BaseMC(SDecoderContext* pCtx,sMCRefMember* pMCRefMem,const int32_t& listIdx,const int8_t& iRefIdx,int32_t iXOffset,int32_t iYOffset,SMcFunc* pMCFunc,int32_t iBlkWidth,int32_t iBlkHeight,int16_t iMVs[2]){
	int32_t iFullMVx=(iXOffset<<2)+iMVs[0];		// quarter pixel
	int32_t iFullMVy=(iYOffset<<2)+iMVs[1];
	iFullMVx=WELS_CLIP3(iFullMVx,((-PADDING_LENGTH+2)*(1<<2)),((pMCRefMem->iPicWidth+PADDING_LENGTH-19)*(1<<2)));
	iFullMVy=WELS_CLIP3(iFullMVy,((-PADDING_LENGTH+2)*(1<<2)),((pMCRefMem->iPicHeight+PADDING_LENGTH-19)*(1<<2)));

	int32_t iSrcPixOffsetLuma=(iFullMVx>>2)+(iFullMVy>>2)*pMCRefMem->iSrcLineLuma;
	int32_t iSrcPixOffsetChroma=(iFullMVx>>3)+(iFullMVy>>3)*pMCRefMem->iSrcLineChroma;

	int32_t iBlkWidthChroma=iBlkWidth>>1;
	int32_t iBlkHeightChroma=iBlkHeight>>1;

	uint8_t* pSrcY=pMCRefMem->pSrcY+iSrcPixOffsetLuma;
	uint8_t* pSrcU=pMCRefMem->pSrcU+iSrcPixOffsetChroma;
	uint8_t* pSrcV=pMCRefMem->pSrcV+iSrcPixOffsetChroma;
	uint8_t* pDstY=pMCRefMem->pDstY;
	uint8_t* pDstU=pMCRefMem->pDstU;
	uint8_t* pDstV=pMCRefMem->pDstV;

	pMCFunc->pMcLumaFunc(pSrcY,pMCRefMem->iSrcLineLuma,pDstY,pMCRefMem->iDstLineLuma,iFullMVx,iFullMVy,iBlkWidth,iBlkHeight);
	pMCFunc->pMcChromaFunc(pSrcU,pMCRefMem->iSrcLineChroma,pDstU,pMCRefMem->iDstLineChroma,iFullMVx,iFullMVy,iBlkWidthChroma,iBlkHeightChroma);
	pMCFunc->pMcChromaFunc(pSrcV,pMCRefMem->iSrcLineChroma,pDstV,pMCRefMem->iDstLineChroma,iFullMVx,iFullMVy,iBlkWidthChroma,iBlkHeightChroma);

}

static void WeightPrediction(PDqLayer pCurDqLayer,sMCRefMember* pMCRefMem,int32_t listIdx,int32_t iRefIdx,int32_t iBlkWidth,int32_t iBlkHeight){
	int32_t iLog2denom,iWoc,iOoc;
	int32_t iPredTemp,iLineStride;
	int32_t iPixel=0;
	uint8_t* pDst;
	// luma
	iLog2denom=pCurDqLayer->pPredWeightTable->uiLumaLog2WeightDenom;
	iWoc=pCurDqLayer->pPredWeightTable->sPredList[listIdx].iLumaWeight[iRefIdx];
	iOoc=pCurDqLayer->pPredWeightTable->sPredList[listIdx].iLumaOffset[iRefIdx];
	iLineStride=pMCRefMem->iDstLineLuma;

	for(int i=0; i<iBlkHeight; i++){
		for(int j=0; j<iBlkWidth; j++){
			iPixel=j+i*(iLineStride);
			if(iLog2denom>=1){
				iPredTemp=((pMCRefMem->pDstY[iPixel]*iWoc+(1<<(iLog2denom-1)))>>iLog2denom)+iOoc;

				pMCRefMem->pDstY[iPixel]=WELS_CLIP3(iPredTemp,0,255);
			}else{
				iPredTemp=pMCRefMem->pDstY[iPixel]*iWoc+iOoc;

				pMCRefMem->pDstY[iPixel]=WELS_CLIP3(iPredTemp,0,255);

			}
		}
	}
	// UV
	iBlkWidth=iBlkWidth>>1;
	iBlkHeight=iBlkHeight>>1;
	iLog2denom=pCurDqLayer->pPredWeightTable->uiChromaLog2WeightDenom;
	iLineStride=pMCRefMem->iDstLineChroma;
	for(int i=0; i<2; i++){
		// iLog2denom=pCurDqLayer->pPredWeightTable->uiChromaLog2WeightDenom;
		iWoc=pCurDqLayer->pPredWeightTable->sPredList[listIdx].iChromaWeight[iRefIdx][i];
		iOoc=pCurDqLayer->pPredWeightTable->sPredList[listIdx].iChromaOffset[iRefIdx][i];
		pDst=i ? pMCRefMem->pDstV : pMCRefMem->pDstU;
		// iLineStride=pMCRefMem->iDstLineChroma;

		for(int i=0; i<iBlkHeight; i++){
			for(int j=0; j<iBlkWidth; j++){
				iPixel=j+i*(iLineStride);
				if(iLog2denom>=1){
					iPredTemp=((pDst[iPixel]*iWoc+(1<<(iLog2denom-1)))>>iLog2denom)+iOoc;

					pDst[iPixel]=WELS_CLIP3(iPredTemp,0,255);
				}else{
					iPredTemp=pDst[iPixel]*iWoc+iOoc;

					pDst[iPixel]=WELS_CLIP3(iPredTemp,0,255);

				}
			}

		}


	}
}

int32_t GetInterPred(uint8_t* pPredY,uint8_t* pPredCb,uint8_t* pPredCr,SDecoderContext* pCtx){
	sMCRefMember pMCRefMem;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SMcFunc* pMCFunc=&pCtx->sMcFunc;

	int32_t iMBXY=pCurDqLayer->iMbXyIndex;

	int16_t iMVs[2]={0};

	uint32_t iMBType=pCurDqLayer->pDec->pMbType[iMBXY];

	int32_t iMBOffsetX=pCurDqLayer->iMbX<<4;
	int32_t iMBOffsetY=pCurDqLayer->iMbY<<4;

	int32_t iDstLineLuma=pCtx->pDec->iLinesize[0];
	int32_t iDstLineChroma=pCtx->pDec->iLinesize[1];

	int32_t iBlk8X,iBlk8Y,iBlk4X,iBlk4Y,i,j,iIIdx,iJIdx;

	pMCRefMem.iPicWidth=(pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.iMbWidth<<4);
	pMCRefMem.iPicHeight=(pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.iMbHeight<<4);

	pMCRefMem.pDstY=pPredY;
	pMCRefMem.pDstU=pPredCb;
	pMCRefMem.pDstV=pPredCr;

	pMCRefMem.iDstLineLuma=iDstLineLuma;
	pMCRefMem.iDstLineChroma=iDstLineChroma;

	int8_t iRefIndex=0;

	switch(iMBType){
		case MB_TYPE_SKIP:
		case MB_TYPE_16x16:
			iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][0][0];
			iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][0][1];
			iRefIndex=pCurDqLayer->pDec->pRefIndex[0][iMBXY][0];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX,iMBOffsetY,pMCFunc,16,16,iMVs);

			if(pCurDqLayer->bUseWeightPredictionFlag){
				iRefIndex=pCurDqLayer->pDec->pRefIndex[0][iMBXY][0];
				WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,16,16);
			}
			break;
		case MB_TYPE_16x8:
			iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][0][0];
			iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][0][1];
			iRefIndex=pCurDqLayer->pDec->pRefIndex[0][iMBXY][0];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX,iMBOffsetY,pMCFunc,16,8,iMVs);

			if(pCurDqLayer->bUseWeightPredictionFlag){
				WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,16,8);
			}

			iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][8][0];
			iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][8][1];
			iRefIndex=pCurDqLayer->pDec->pRefIndex[0][iMBXY][8];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			pMCRefMem.pDstY=pPredY+(iDstLineLuma<<3);
			pMCRefMem.pDstU=pPredCb+(iDstLineChroma<<2);
			pMCRefMem.pDstV=pPredCr+(iDstLineChroma<<2);
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX,iMBOffsetY+8,pMCFunc,16,8,iMVs);

			if(pCurDqLayer->bUseWeightPredictionFlag){
				WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,16,8);
			}
			break;
		case MB_TYPE_8x16:
			iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][0][0];
			iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][0][1];
			iRefIndex=pCurDqLayer->pDec->pRefIndex[0][iMBXY][0];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX,iMBOffsetY,pMCFunc,8,16,iMVs);
			if(pCurDqLayer->bUseWeightPredictionFlag){
				WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,8,16);
			}

			iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][2][0];
			iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][2][1];
			iRefIndex=pCurDqLayer->pDec->pRefIndex[0][iMBXY][2];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
			pMCRefMem.pDstY=pPredY+8;
			pMCRefMem.pDstU=pPredCb+4;
			pMCRefMem.pDstV=pPredCr+4;
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iMBOffsetX+8,iMBOffsetY,pMCFunc,8,16,iMVs);

			if(pCurDqLayer->bUseWeightPredictionFlag){
				WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,8,16);
			}
			break;
		case MB_TYPE_8x8:
		case MB_TYPE_8x8_REF0:
		{
			uint32_t iSubMBType;
			int32_t iXOffset,iYOffset;
			uint8_t* pDstY,* pDstU,* pDstV;
			for(i=0; i<4; i++){
				iSubMBType=pCurDqLayer->pSubMbType[iMBXY][i];
				iBlk8X=(i&1)<<3;
				iBlk8Y=(i>>1)<<3;
				iXOffset=iMBOffsetX+iBlk8X;
				iYOffset=iMBOffsetY+iBlk8Y;

				iIIdx=((i>>1)<<3)+((i&1)<<1);
				iRefIndex=pCurDqLayer->pDec->pRefIndex[0][iMBXY][iIIdx];
				WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,LIST_0));
				pDstY=pPredY+iBlk8X+iBlk8Y*iDstLineLuma;
				pDstU=pPredCb+(iBlk8X>>1)+(iBlk8Y>>1)*iDstLineChroma;
				pDstV=pPredCr+(iBlk8X>>1)+(iBlk8Y>>1)*iDstLineChroma;
				pMCRefMem.pDstY=pDstY;
				pMCRefMem.pDstU=pDstU;
				pMCRefMem.pDstV=pDstV;
				switch(iSubMBType){
					case SUB_MB_TYPE_8x8:
						iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx][1];
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset,iYOffset,pMCFunc,8,8,iMVs);
						if(pCurDqLayer->bUseWeightPredictionFlag){

							WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,8,8);
						}

						break;
					case SUB_MB_TYPE_8x4:
						iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx][1];
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset,iYOffset,pMCFunc,8,4,iMVs);
						if(pCurDqLayer->bUseWeightPredictionFlag){

							WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,8,4);
						}


						iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx+4][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx+4][1];
						pMCRefMem.pDstY+=(iDstLineLuma<<2);
						pMCRefMem.pDstU+=(iDstLineChroma<<1);
						pMCRefMem.pDstV+=(iDstLineChroma<<1);
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset,iYOffset+4,pMCFunc,8,4,iMVs);
						if(pCurDqLayer->bUseWeightPredictionFlag){

							WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,8,4);
						}

						break;
					case SUB_MB_TYPE_4x8:
						iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx][1];
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset,iYOffset,pMCFunc,4,8,iMVs);
						if(pCurDqLayer->bUseWeightPredictionFlag){

							WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,4,8);
						}


						iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx+1][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx+1][1];
						pMCRefMem.pDstY+=4;
						pMCRefMem.pDstU+=2;
						pMCRefMem.pDstV+=2;
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset+4,iYOffset,pMCFunc,4,8,iMVs);
						if(pCurDqLayer->bUseWeightPredictionFlag){
							WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,4,8);
						}

						break;
					case SUB_MB_TYPE_4x4:
					{
						for(j=0; j<4; j++){
							int32_t iUVLineStride;
							iJIdx=((j>>1)<<2)+(j&1);
							iBlk4X=(j&1)<<2;
							iBlk4Y=(j>>1)<<2;
							iUVLineStride=(iBlk4X>>1)+(iBlk4Y>>1)*iDstLineChroma;
							pMCRefMem.pDstY=pDstY+iBlk4X+iBlk4Y*iDstLineLuma;
							pMCRefMem.pDstU=pDstU+iUVLineStride;
							pMCRefMem.pDstV=pDstV+iUVLineStride;
							iMVs[0]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx+iJIdx][0];
							iMVs[1]=pCurDqLayer->pDec->pMv[0][iMBXY][iIIdx+iJIdx][1];
							BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex,iXOffset+iBlk4X,iYOffset+iBlk4Y,pMCFunc,4,4,iMVs);
							if(pCurDqLayer->bUseWeightPredictionFlag){
								WeightPrediction(pCurDqLayer,&pMCRefMem,LIST_0,iRefIndex,4,4);
							}
						}
					}
					break;
					default:
						break;
				}
			}
		}
		break;
		default:
			break;
	}
	return ERR_NONE;
}

static void BiWeightPrediction(PDqLayer pCurDqLayer,sMCRefMember* pMCRefMem,sMCRefMember* pTempMCRefMem,int32_t iRefIdx1,int32_t iRefIdx2,bool bWeightedBipredIdcIs1,int32_t iBlkWidth,int32_t iBlkHeight){
	int32_t iWoc1=0,iOoc1=0,iWoc2=0,iOoc2=0;
	int32_t iPredTemp,iLineStride;
	int32_t iPixel=0;
	// luma
	int32_t iLog2denom=pCurDqLayer->pPredWeightTable->uiLumaLog2WeightDenom;
	if(bWeightedBipredIdcIs1){
		iWoc1=pCurDqLayer->pPredWeightTable->sPredList[LIST_0].iLumaWeight[iRefIdx1];
		iOoc1=pCurDqLayer->pPredWeightTable->sPredList[LIST_0].iLumaOffset[iRefIdx1];
		iWoc2=pCurDqLayer->pPredWeightTable->sPredList[LIST_1].iLumaWeight[iRefIdx2];
		iOoc2=pCurDqLayer->pPredWeightTable->sPredList[LIST_1].iLumaOffset[iRefIdx2];
	}else{
		iWoc1=pCurDqLayer->pPredWeightTable->iImplicitWeight[iRefIdx1][iRefIdx2];
		iWoc2=64-iWoc1;
	}
	iLineStride=pMCRefMem->iDstLineLuma;

	for(int i=0; i<iBlkHeight; i++){
		for(int j=0; j<iBlkWidth; j++){
			iPixel=j+i*(iLineStride);
			iPredTemp=((pMCRefMem->pDstY[iPixel]*iWoc1+pTempMCRefMem->pDstY[iPixel]*iWoc2+(1<<iLog2denom))>>
						 (iLog2denom+1))+((iOoc1+iOoc2+1)>>1);
			pMCRefMem->pDstY[iPixel]=WELS_CLIP3(iPredTemp,0,255);
		}
	}

	// UV
	iBlkWidth=iBlkWidth>>1;
	iBlkHeight=iBlkHeight>>1;
	iLog2denom=pCurDqLayer->pPredWeightTable->uiChromaLog2WeightDenom;
	iLineStride=pMCRefMem->iDstLineChroma;

	uint8_t* pDst;
	uint8_t* pTempDst;
	for(int k=0; k<2; k++){
		if(bWeightedBipredIdcIs1){
			iWoc1=pCurDqLayer->pPredWeightTable->sPredList[LIST_0].iChromaWeight[iRefIdx1][k];
			iOoc1=pCurDqLayer->pPredWeightTable->sPredList[LIST_0].iChromaOffset[iRefIdx1][k];
			iWoc2=pCurDqLayer->pPredWeightTable->sPredList[LIST_1].iChromaWeight[iRefIdx2][k];
			iOoc2=pCurDqLayer->pPredWeightTable->sPredList[LIST_1].iChromaOffset[iRefIdx2][k];
		}
		pDst=k ? pMCRefMem->pDstV : pMCRefMem->pDstU;
		pTempDst=k ? pTempMCRefMem->pDstV : pTempMCRefMem->pDstU;
		for(int i=0; i<iBlkHeight; i++){
			for(int j=0; j<iBlkWidth; j++){
				iPixel=j+i*(iLineStride);
				iPredTemp=((pDst[iPixel]*iWoc1+pTempDst[iPixel]*iWoc2+(1<<iLog2denom))>>(iLog2denom+1))+((
					iOoc1+iOoc2+1)>>1);
				pDst[iPixel]=WELS_CLIP3(iPredTemp,0,255);
			}
		}
	}
}

static void BiPrediction(PDqLayer pCurDqLayer,sMCRefMember* pMCRefMem,sMCRefMember* pTempMCRefMem,int32_t iBlkWidth,int32_t iBlkHeight){
	int32_t iPredTemp,iLineStride;
	int32_t iPixel=0;
	// luma
	iLineStride=pMCRefMem->iDstLineLuma;

	for(int i=0; i<iBlkHeight; i++){
		for(int j=0; j<iBlkWidth; j++){
			iPixel=j+i*(iLineStride);
			iPredTemp=(pMCRefMem->pDstY[iPixel]+pTempMCRefMem->pDstY[iPixel]+1)>>1;
			pMCRefMem->pDstY[iPixel]=WELS_CLIP3(iPredTemp,0,255);
		}
	}

	// UV
	iBlkWidth=iBlkWidth>>1;
	iBlkHeight=iBlkHeight>>1;
	iLineStride=pMCRefMem->iDstLineChroma;

	uint8_t* pDst;
	uint8_t* pTempDst;
	for(int k=0; k<2; k++){
		pDst=k ? pMCRefMem->pDstV : pMCRefMem->pDstU;
		pTempDst=k ? pTempMCRefMem->pDstV : pTempMCRefMem->pDstU;
		// iLineStride=pMCRefMem->iDstLineChroma;

		for(int i=0; i<iBlkHeight; i++){
			for(int j=0; j<iBlkWidth; j++){
				iPixel=j+i*(iLineStride);
				iPredTemp=(pDst[iPixel]+pTempDst[iPixel]+1)>>1;
				pDst[iPixel]=WELS_CLIP3(iPredTemp,0,255);
			}
		}
	}
}

int32_t GetInterBPred(uint8_t* pPredYCbCr[3],uint8_t* pTempPredYCbCr[3],SDecoderContext* pCtx){
	sMCRefMember pMCRefMem;
	sMCRefMember pTempMCRefMem;

	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SMcFunc* pMCFunc=&pCtx->sMcFunc;

	int32_t iMBXY=pCurDqLayer->iMbXyIndex;

	int16_t iMVs[2]={0};

	uint32_t iMBType=pCurDqLayer->pDec->pMbType[iMBXY];

	int32_t iMBOffsetX=pCurDqLayer->iMbX<<4;
	int32_t iMBOffsetY=pCurDqLayer->iMbY<<4;

	int32_t iDstLineLuma=pCtx->pDec->iLinesize[0];
	int32_t iDstLineChroma=pCtx->pDec->iLinesize[1];


	pMCRefMem.iPicWidth=(pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.iMbWidth<<4);
	pMCRefMem.iPicHeight=(pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.iMbHeight<<4);

	pMCRefMem.pDstY=pPredYCbCr[0];
	pMCRefMem.pDstU=pPredYCbCr[1];
	pMCRefMem.pDstV=pPredYCbCr[2];

	pMCRefMem.iDstLineLuma=iDstLineLuma;
	pMCRefMem.iDstLineChroma=iDstLineChroma;

	pTempMCRefMem=pMCRefMem;
	pTempMCRefMem.pDstY=pTempPredYCbCr[0];
	pTempMCRefMem.pDstU=pTempPredYCbCr[1];
	pTempMCRefMem.pDstV=pTempPredYCbCr[2];


	int8_t iRefIndex0=0;
	int8_t iRefIndex1=0;
	int8_t iRefIndex=0;

	bool bWeightedBipredIdcIs1=pCurDqLayer->sLayerInfo.pPps->uiWeightedBipredIdc==1;

	if(IS_INTER_16x16(iMBType)){
		if(IS_TYPE_L0(iMBType) && IS_TYPE_L1(iMBType)){
			iMVs[0]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][0][0];
			iMVs[1]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][0][1];
			iRefIndex0=pCurDqLayer->pDec->pRefIndex[LIST_0][iMBXY][0];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex0,LIST_0));
			BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex0,iMBOffsetX,iMBOffsetY,pMCFunc,16,16,iMVs);

			iMVs[0]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][0][0];
			iMVs[1]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][0][1];
			iRefIndex1=pCurDqLayer->pDec->pRefIndex[LIST_1][iMBXY][0];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pTempMCRefMem,pCtx,iRefIndex1,LIST_1));
			BaseMC(pCtx,&pTempMCRefMem,LIST_1,iRefIndex1,iMBOffsetX,iMBOffsetY,pMCFunc,16,16,iMVs);
			if(pCurDqLayer->bUseWeightedBiPredIdc){
				BiWeightPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,iRefIndex0,iRefIndex1,bWeightedBipredIdcIs1,16,16);
			}else{
				BiPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,16,16);
			}
		}else{
			int32_t listIdx=(iMBType&MB_TYPE_P0L0) ? LIST_0 : LIST_1;
			iMVs[0]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][0][0];
			iMVs[1]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][0][1];
			iRefIndex=pCurDqLayer->pDec->pRefIndex[listIdx][iMBXY][0];
			WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,listIdx));
			BaseMC(pCtx,&pMCRefMem,listIdx,iRefIndex,iMBOffsetX,iMBOffsetY,pMCFunc,16,16,iMVs);
			if(bWeightedBipredIdcIs1){
				WeightPrediction(pCurDqLayer,&pMCRefMem,listIdx,iRefIndex,16,16);
			}
		}
	}else
	if(IS_INTER_16x8(iMBType)){
		for(int32_t i=0; i<2;++i){
			int32_t iPartIdx=i<<3;
			uint32_t listCount=0;
			int32_t lastListIdx=LIST_0;
			for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
				if(IS_DIR(iMBType,i,listIdx)){
					lastListIdx=listIdx;
					iMVs[0]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iPartIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iPartIdx][1];
					iRefIndex=pCurDqLayer->pDec->pRefIndex[listIdx][iMBXY][iPartIdx];
					WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,listIdx));
					if(i){
						pMCRefMem.pDstY+=(iDstLineLuma<<3);
						pMCRefMem.pDstU+=(iDstLineChroma<<2);
						pMCRefMem.pDstV+=(iDstLineChroma<<2);
					}
					BaseMC(pCtx,&pMCRefMem,listIdx,iRefIndex,iMBOffsetX,iMBOffsetY+iPartIdx,pMCFunc,16,8,iMVs);
					if(++listCount==2){
						iMVs[0]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iPartIdx][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iPartIdx][1];
						iRefIndex1=pCurDqLayer->pDec->pRefIndex[LIST_1][iMBXY][iPartIdx];
						WELS_B_MB_REC_VERIFY(GetRefPic(&pTempMCRefMem,pCtx,iRefIndex1,LIST_1));
						if(i){
							pTempMCRefMem.pDstY+=(iDstLineLuma<<3);
							pTempMCRefMem.pDstU+=(iDstLineChroma<<2);
							pTempMCRefMem.pDstV+=(iDstLineChroma<<2);
						}
						BaseMC(pCtx,&pTempMCRefMem,LIST_1,iRefIndex1,iMBOffsetX,iMBOffsetY+iPartIdx,pMCFunc,16,8,iMVs);
						if(pCurDqLayer->bUseWeightedBiPredIdc){
							iRefIndex0=pCurDqLayer->pDec->pRefIndex[LIST_0][iMBXY][iPartIdx];
							iRefIndex1=pCurDqLayer->pDec->pRefIndex[LIST_1][iMBXY][iPartIdx];
							BiWeightPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,iRefIndex0,iRefIndex1,bWeightedBipredIdcIs1,16,8);
						}else{
							BiPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,16,8);
						}
					}
				}
			}
			if(listCount==1){
				if(bWeightedBipredIdcIs1){
					iRefIndex=pCurDqLayer->pDec->pRefIndex[lastListIdx][iMBXY][iPartIdx];
					WeightPrediction(pCurDqLayer,&pMCRefMem,lastListIdx,iRefIndex,16,8);
				}
			}
		}
	}else
	if(IS_INTER_8x16(iMBType)){
		for(int32_t i=0; i<2;++i){
			uint32_t listCount=0;
			int32_t lastListIdx=LIST_0;
			for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
				if(IS_DIR(iMBType,i,listIdx)){
					lastListIdx=listIdx;
					iMVs[0]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][i<<1][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][i<<1][1];
					iRefIndex=pCurDqLayer->pDec->pRefIndex[listIdx][iMBXY][i<<1];
					WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,listIdx));
					if(i){
						pMCRefMem.pDstY+=8;
						pMCRefMem.pDstU+=4;
						pMCRefMem.pDstV+=4;
					}
					BaseMC(pCtx,&pMCRefMem,listIdx,iRefIndex,iMBOffsetX+(i ? 8 : 0),iMBOffsetY,pMCFunc,8,16,iMVs);
					if(++listCount==2){
						iMVs[0]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][i<<1][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][i<<1][1];
						iRefIndex1=pCurDqLayer->pDec->pRefIndex[LIST_1][iMBXY][i<<1];
						WELS_B_MB_REC_VERIFY(GetRefPic(&pTempMCRefMem,pCtx,iRefIndex1,LIST_1));
						if(i){
							pTempMCRefMem.pDstY+=8;
							pTempMCRefMem.pDstU+=4;
							pTempMCRefMem.pDstV+=4;
						}
						BaseMC(pCtx,&pTempMCRefMem,LIST_1,iRefIndex1,iMBOffsetX+(i ? 8 : 0),iMBOffsetY,pMCFunc,8,16,iMVs);
						if(pCurDqLayer->bUseWeightedBiPredIdc){
							iRefIndex0=pCurDqLayer->pDec->pRefIndex[LIST_0][iMBXY][i<<1];
							iRefIndex1=pCurDqLayer->pDec->pRefIndex[LIST_1][iMBXY][i<<1];
							BiWeightPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,iRefIndex0,iRefIndex1,bWeightedBipredIdcIs1,8,16);
						}else{
							BiPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,8,16);
						}
					}
				}
			}
			if(listCount==1){
				if(bWeightedBipredIdcIs1){
					iRefIndex=pCurDqLayer->pDec->pRefIndex[lastListIdx][iMBXY][i<<1];
					WeightPrediction(pCurDqLayer,&pMCRefMem,lastListIdx,iRefIndex,8,16);
				}
			}
		}
	}else
	if(IS_Inter_8x8(iMBType)){
		int32_t iBlk8X,iBlk8Y,iBlk4X,iBlk4Y,iIIdx,iJIdx;
		uint32_t iSubMBType;
		int32_t iXOffset,iYOffset;
		uint8_t* pDstY,* pDstU,* pDstV;
		uint8_t* pDstY2,* pDstU2,* pDstV2;
		for(int32_t i=0; i<4; i++){
			iSubMBType=pCurDqLayer->pSubMbType[iMBXY][i];
			iBlk8X=(i&1)<<3;
			iBlk8Y=(i>>1)<<3;
			iXOffset=iMBOffsetX+iBlk8X;
			iYOffset=iMBOffsetY+iBlk8Y;

			iIIdx=((i>>1)<<3)+((i&1)<<1);

			pDstY=pPredYCbCr[0]+iBlk8X+iBlk8Y*iDstLineLuma;
			pDstU=pPredYCbCr[1]+(iBlk8X>>1)+(iBlk8Y>>1)*iDstLineChroma;
			pDstV=pPredYCbCr[2]+(iBlk8X>>1)+(iBlk8Y>>1)*iDstLineChroma;
			pMCRefMem.pDstY=pDstY;
			pMCRefMem.pDstU=pDstU;
			pMCRefMem.pDstV=pDstV;

			pTempMCRefMem=pMCRefMem;
			pDstY2=pTempPredYCbCr[0]+iBlk8X+iBlk8Y*iDstLineLuma;
			pDstU2=pTempPredYCbCr[1]+(iBlk8X>>1)+(iBlk8Y>>1)*iDstLineChroma;
			pDstV2=pTempPredYCbCr[2]+(iBlk8X>>1)+(iBlk8Y>>1)*iDstLineChroma;

			pTempMCRefMem.pDstY=pDstY2;
			pTempMCRefMem.pDstU=pDstU2;
			pTempMCRefMem.pDstV=pDstV2;

			if((IS_TYPE_L0(iSubMBType) && IS_TYPE_L1(iSubMBType))){
				iRefIndex0=pCurDqLayer->pDec->pRefIndex[LIST_0][iMBXY][iIIdx];
				WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex0,LIST_0));

				iRefIndex1=pCurDqLayer->pDec->pRefIndex[LIST_1][iMBXY][iIIdx];
				WELS_B_MB_REC_VERIFY(GetRefPic(&pTempMCRefMem,pCtx,iRefIndex1,LIST_1));
			}else{
				int32_t listIdx=IS_TYPE_L0(iSubMBType) ? LIST_0 : LIST_1;
				iRefIndex=pCurDqLayer->pDec->pRefIndex[listIdx][iMBXY][iIIdx];
				WELS_B_MB_REC_VERIFY(GetRefPic(&pMCRefMem,pCtx,iRefIndex,listIdx));
			}

			if(IS_SUB_8x8(iSubMBType)){
				if(IS_TYPE_L0(iSubMBType) && IS_TYPE_L1(iSubMBType)){
					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx][1];
					BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex0,iXOffset,iYOffset,pMCFunc,8,8,iMVs);

					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx][1];
					BaseMC(pCtx,&pTempMCRefMem,LIST_1,iRefIndex1,iXOffset,iYOffset,pMCFunc,8,8,iMVs);

					if(pCurDqLayer->bUseWeightedBiPredIdc){
						BiWeightPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,iRefIndex0,iRefIndex1,bWeightedBipredIdcIs1,8,8);
					}else{
						BiPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,8,8);
					}
				}else{
					int32_t listIdx=IS_TYPE_L0(iSubMBType) ? LIST_0 : LIST_1;
					iMVs[0]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx][1];
					iRefIndex=pCurDqLayer->pDec->pRefIndex[listIdx][iMBXY][iIIdx];
					BaseMC(pCtx,&pMCRefMem,listIdx,iRefIndex,iXOffset,iYOffset,pMCFunc,8,8,iMVs);
					if(bWeightedBipredIdcIs1){
						WeightPrediction(pCurDqLayer,&pMCRefMem,listIdx,iRefIndex,8,8);
					}
				}
			}else
			if(IS_SUB_8x4(iSubMBType)){
				if(IS_TYPE_L0(iSubMBType) && IS_TYPE_L1(iSubMBType)){		// B_Bi_8x4
					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx][1];
					BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex0,iXOffset,iYOffset,pMCFunc,8,4,iMVs);
					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx][1];
					BaseMC(pCtx,&pTempMCRefMem,LIST_1,iRefIndex1,iXOffset,iYOffset,pMCFunc,8,4,iMVs);

					if(pCurDqLayer->bUseWeightedBiPredIdc){
						BiWeightPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,iRefIndex0,iRefIndex1,bWeightedBipredIdcIs1,8,4);
					}else{
						BiPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,8,4);
					}

					pMCRefMem.pDstY+=(iDstLineLuma<<2);
					pMCRefMem.pDstU+=(iDstLineChroma<<1);
					pMCRefMem.pDstV+=(iDstLineChroma<<1);
					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx+4][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx+4][1];
					BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex0,iXOffset,iYOffset+4,pMCFunc,8,4,iMVs);

					pTempMCRefMem.pDstY+=(iDstLineLuma<<2);
					pTempMCRefMem.pDstU+=(iDstLineChroma<<1);
					pTempMCRefMem.pDstV+=(iDstLineChroma<<1);
					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx+4][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx+4][1];
					BaseMC(pCtx,&pTempMCRefMem,LIST_1,iRefIndex1,iXOffset,iYOffset+4,pMCFunc,8,4,iMVs);

					if(pCurDqLayer->bUseWeightedBiPredIdc){
						BiWeightPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,iRefIndex0,iRefIndex1,bWeightedBipredIdcIs1,8,4);
					}else{
						BiPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,8,4);
					}
				}else{		// B_L0_8x4 B_L1_8x4
					int32_t listIdx=IS_TYPE_L0(iSubMBType) ? LIST_0 : LIST_1;
					iMVs[0]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx][1];
					iRefIndex=pCurDqLayer->pDec->pRefIndex[listIdx][iMBXY][iIIdx];
					BaseMC(pCtx,&pMCRefMem,listIdx,iRefIndex,iXOffset,iYOffset,pMCFunc,8,4,iMVs);
					pMCRefMem.pDstY+=(iDstLineLuma<<2);
					pMCRefMem.pDstU+=(iDstLineChroma<<1);
					pMCRefMem.pDstV+=(iDstLineChroma<<1);
					iMVs[0]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx+4][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx+4][1];
					BaseMC(pCtx,&pMCRefMem,listIdx,iRefIndex,iXOffset,iYOffset+4,pMCFunc,8,4,iMVs);
					if(bWeightedBipredIdcIs1){
						WeightPrediction(pCurDqLayer,&pMCRefMem,listIdx,iRefIndex,8,4);
					}
				}
			}else
			if(IS_SUB_4x8(iSubMBType)){
				if(IS_TYPE_L0(iSubMBType) && IS_TYPE_L1(iSubMBType)){		// B_Bi_4x8
					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx][1];
					BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex0,iXOffset,iYOffset,pMCFunc,4,8,iMVs);
					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx][1];
					BaseMC(pCtx,&pTempMCRefMem,LIST_1,iRefIndex1,iXOffset,iYOffset,pMCFunc,4,8,iMVs);

					if(pCurDqLayer->bUseWeightedBiPredIdc){
						BiWeightPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,iRefIndex0,iRefIndex1,bWeightedBipredIdcIs1,4,8);
					}else{
						BiPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,4,8);
					}

					pMCRefMem.pDstY+=4;
					pMCRefMem.pDstU+=2;
					pMCRefMem.pDstV+=2;
					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx+1][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx+1][1];
					BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex0,iXOffset+4,iYOffset,pMCFunc,4,8,iMVs);

					pTempMCRefMem.pDstY+=4;
					pTempMCRefMem.pDstU+=2;
					pTempMCRefMem.pDstV+=2;
					iMVs[0]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx+1][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx+1][1];
					BaseMC(pCtx,&pTempMCRefMem,LIST_1,iRefIndex1,iXOffset+4,iYOffset,pMCFunc,4,8,iMVs);

					if(pCurDqLayer->bUseWeightedBiPredIdc){
						BiWeightPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,iRefIndex0,iRefIndex1,bWeightedBipredIdcIs1,4,8);
					}else{
						BiPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,4,8);
					}
				}else{		// B_L0_4x8 B_L1_4x8
					int32_t listIdx=IS_TYPE_L0(iSubMBType) ? LIST_0 : LIST_1;
					iMVs[0]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx][1];
					iRefIndex=pCurDqLayer->pDec->pRefIndex[listIdx][iMBXY][iIIdx];
					BaseMC(pCtx,&pMCRefMem,listIdx,iRefIndex,iXOffset,iYOffset,pMCFunc,4,8,iMVs);
					pMCRefMem.pDstY+=4;
					pMCRefMem.pDstU+=2;
					pMCRefMem.pDstV+=2;
					iMVs[0]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx+1][0];
					iMVs[1]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx+1][1];
					BaseMC(pCtx,&pMCRefMem,listIdx,iRefIndex,iXOffset+4,iYOffset,pMCFunc,4,8,iMVs);
					if(bWeightedBipredIdcIs1){
						WeightPrediction(pCurDqLayer,&pMCRefMem,listIdx,iRefIndex,4,8);
					}
				}
			}else
			if(IS_SUB_4x4(iSubMBType)){
				if(IS_TYPE_L0(iSubMBType) && IS_TYPE_L1(iSubMBType)){
					for(int32_t j=0; j<4; j++){
						int32_t iUVLineStride;
						iJIdx=((j>>1)<<2)+(j&1);

						iBlk4X=(j&1)<<2;
						iBlk4Y=(j>>1)<<2;

						iUVLineStride=(iBlk4X>>1)+(iBlk4Y>>1)*iDstLineChroma;
						pMCRefMem.pDstY=pDstY+iBlk4X+iBlk4Y*iDstLineLuma;
						pMCRefMem.pDstU=pDstU+iUVLineStride;
						pMCRefMem.pDstV=pDstV+iUVLineStride;

						iMVs[0]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx+iJIdx][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[LIST_0][iMBXY][iIIdx+iJIdx][1];
						BaseMC(pCtx,&pMCRefMem,LIST_0,iRefIndex0,iXOffset+iBlk4X,iYOffset+iBlk4Y,pMCFunc,4,4,iMVs);

						pTempMCRefMem.pDstY=pDstY2+iBlk8X+iBlk8Y*iDstLineLuma;
						pTempMCRefMem.pDstU=pDstU2+iUVLineStride;
						pTempMCRefMem.pDstV=pDstV2+iUVLineStride;;

						iMVs[0]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx+iJIdx][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[LIST_1][iMBXY][iIIdx+iJIdx][1];
						BaseMC(pCtx,&pTempMCRefMem,LIST_1,iRefIndex1,iXOffset+iBlk4X,iYOffset+iBlk4Y,pMCFunc,4,4,iMVs);

						if(pCurDqLayer->bUseWeightedBiPredIdc){
							BiWeightPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,iRefIndex0,iRefIndex1,bWeightedBipredIdcIs1,4,4);
						}else{
							BiPrediction(pCurDqLayer,&pMCRefMem,&pTempMCRefMem,4,4);
						}
					}
				}else{
					int32_t listIdx=IS_TYPE_L0(iSubMBType) ? LIST_0 : LIST_1;
					iRefIndex=pCurDqLayer->pDec->pRefIndex[listIdx][iMBXY][iIIdx];
					for(int32_t j=0; j<4; j++){
						int32_t iUVLineStride;
						iJIdx=((j>>1)<<2)+(j&1);

						iBlk4X=(j&1)<<2;
						iBlk4Y=(j>>1)<<2;

						iUVLineStride=(iBlk4X>>1)+(iBlk4Y>>1)*iDstLineChroma;
						pMCRefMem.pDstY=pDstY+iBlk4X+iBlk4Y*iDstLineLuma;
						pMCRefMem.pDstU=pDstU+iUVLineStride;
						pMCRefMem.pDstV=pDstV+iUVLineStride;

						iMVs[0]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx+iJIdx][0];
						iMVs[1]=pCurDqLayer->pDec->pMv[listIdx][iMBXY][iIIdx+iJIdx][1];
						BaseMC(pCtx,&pMCRefMem,listIdx,iRefIndex,iXOffset+iBlk4X,iYOffset+iBlk4Y,pMCFunc,4,4,iMVs);
						if(bWeightedBipredIdcIs1){
							WeightPrediction(pCurDqLayer,&pMCRefMem,listIdx,iRefIndex,4,4);
						}
					}
				}
			}
		}
	}
	return ERR_NONE;
}

int32_t WelsMbInterPrediction(SDecoderContext* pCtx,PDqLayer pCurDqLayer){
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;
	uint8_t* pDstY,* pDstCb,* pDstCr;

	int32_t iLumaStride=pCtx->pDec->iLinesize[0];
	int32_t iChromaStride=pCtx->pDec->iLinesize[1];

	pDstY=pCurDqLayer->pDec->pData[0]+((iMbY*iLumaStride+iMbX)<<4);
	pDstCb=pCurDqLayer->pDec->pData[1]+((iMbY*iChromaStride+iMbX)<<3);
	pDstCr=pCurDqLayer->pDec->pData[2]+((iMbY*iChromaStride+iMbX)<<3);

	if(pCtx->eSliceType==P_SLICE){
		WELS_B_MB_REC_VERIFY(GetInterPred(pDstY,pDstCb,pDstCr,pCtx));
	}else{
		if(pCtx->pTempDec==NULL)
			pCtx->pTempDec=AllocPicture(pCtx,pCtx->pSps->iMbWidth<<4,pCtx->pSps->iMbHeight<<4);
		uint8_t* pTempDstYCbCr[3];
		uint8_t* pDstYCbCr[3];
		pTempDstYCbCr[0]=pCtx->pTempDec->pData[0]+((iMbY*iLumaStride+iMbX)<<4);
		pTempDstYCbCr[1]=pCtx->pTempDec->pData[1]+((iMbY*iChromaStride+iMbX)<<3);
		pTempDstYCbCr[2]=pCtx->pTempDec->pData[2]+((iMbY*iChromaStride+iMbX)<<3);
		pDstYCbCr[0]=pDstY;
		pDstYCbCr[1]=pDstCb;
		pDstYCbCr[2]=pDstCr;
		WELS_B_MB_REC_VERIFY(GetInterBPred(pDstYCbCr,pTempDstYCbCr,pCtx));
	}
	return ERR_NONE;
}

int32_t WelsMbInterSampleConstruction(SDecoderContext* pCtx,PDqLayer pCurDqLayer,uint8_t* pDstY,uint8_t* pDstU,uint8_t* pDstV,int32_t iStrideL,int32_t iStrideC){
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	int32_t i,iIndex,iOffset;

	if(pCurDqLayer->pTransformSize8x8Flag[iMbXy]){
		for(i=0; i<4; i++){
			iIndex=g_kuiMbCountScan4Idx[i<<2];
			if(pCurDqLayer->pNzc[iMbXy][iIndex] || pCurDqLayer->pNzc[iMbXy][iIndex+1] || pCurDqLayer->pNzc[iMbXy][iIndex+4]
				 || pCurDqLayer->pNzc[iMbXy][iIndex+5]){
				iOffset=((iIndex>>2)<<2)*iStrideL+((iIndex%4)<<2);
				pCtx->pIdctResAddPredFunc8x8(pDstY+iOffset,iStrideL,pCurDqLayer->pScaledTCoeff[iMbXy]+(i<<6));
			}
		}
	}else{
		// luma.
		const int8_t* pNzc=pCurDqLayer->pNzc[iMbXy];
		int16_t* pScaledTCoeff=pCurDqLayer->pScaledTCoeff[iMbXy];
		pCtx->pIdctFourResAddPredFunc(pDstY+0*iStrideL+0,iStrideL,pScaledTCoeff+0*64,pNzc+0);
		pCtx->pIdctFourResAddPredFunc(pDstY+0*iStrideL+8,iStrideL,pScaledTCoeff+1*64,pNzc+2);
		pCtx->pIdctFourResAddPredFunc(pDstY+8*iStrideL+0,iStrideL,pScaledTCoeff+2*64,pNzc+8);
		pCtx->pIdctFourResAddPredFunc(pDstY+8*iStrideL+8,iStrideL,pScaledTCoeff+3*64,pNzc+10);
	}

	const int8_t* pNzc=pCurDqLayer->pNzc[iMbXy];
	int16_t* pScaledTCoeff=pCurDqLayer->pScaledTCoeff[iMbXy];
	// Cb.
	pCtx->pIdctFourResAddPredFunc(pDstU,iStrideC,pScaledTCoeff+4*64,pNzc+16);
	// Cr.
	pCtx->pIdctFourResAddPredFunc(pDstV,iStrideC,pScaledTCoeff+5*64,pNzc+18);

	return ERR_NONE;
}

int32_t WelsMbInterConstruction(SDecoderContext* pCtx,PDqLayer pCurDqLayer){
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbY=pCurDqLayer->iMbY;
	uint8_t* pDstY,* pDstCb,* pDstCr;

	int32_t iLumaStride=pCtx->pDec->iLinesize[0];
	int32_t iChromaStride=pCtx->pDec->iLinesize[1];

	pDstY=pCurDqLayer->pDec->pData[0]+((iMbY*iLumaStride+iMbX)<<4);
	pDstCb=pCurDqLayer->pDec->pData[1]+((iMbY*iChromaStride+iMbX)<<3);
	pDstCr=pCurDqLayer->pDec->pData[2]+((iMbY*iChromaStride+iMbX)<<3);

	if(pCtx->eSliceType==P_SLICE){
		WELS_B_MB_REC_VERIFY(GetInterPred(pDstY,pDstCb,pDstCr,pCtx));
	}else{
		if(pCtx->pTempDec==NULL)
			pCtx->pTempDec=AllocPicture(pCtx,pCtx->pSps->iMbWidth<<4,pCtx->pSps->iMbHeight<<4);
		uint8_t* pTempDstYCbCr[3];
		uint8_t* pDstYCbCr[3];
		pTempDstYCbCr[0]=pCtx->pTempDec->pData[0]+((iMbY*iLumaStride+iMbX)<<4);
		pTempDstYCbCr[1]=pCtx->pTempDec->pData[1]+((iMbY*iChromaStride+iMbX)<<3);
		pTempDstYCbCr[2]=pCtx->pTempDec->pData[2]+((iMbY*iChromaStride+iMbX)<<3);
		pDstYCbCr[0]=pDstY;
		pDstYCbCr[1]=pDstCb;
		pDstYCbCr[2]=pDstCr;
		WELS_B_MB_REC_VERIFY(GetInterBPred(pDstYCbCr,pTempDstYCbCr,pCtx));
	}
	WelsMbInterSampleConstruction(pCtx,pCurDqLayer,pDstY,pDstCb,pDstCr,iLumaStride,iChromaStride);

	pCtx->sBlockFunc.pWelsSetNonZeroCountFunc(pCurDqLayer->pNzc[pCurDqLayer->iMbXyIndex]);		// set all none-zero nzc to 1; dbk can be opti!
	return ERR_NONE;
}

int32_t WelsTargetMbConstruction(SDecoderContext* pCtx){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	if(MB_TYPE_INTRA_PCM==pCurDqLayer->pDec->pMbType[pCurDqLayer->iMbXyIndex]){
		// already decoded and reconstructed when parsing
		return ERR_NONE;
	}else
	if(IS_INTRA(pCurDqLayer->pDec->pMbType[pCurDqLayer->iMbXyIndex])){
		WelsMbIntraPredictionConstruction(pCtx,pCurDqLayer,1);
	}else
	if(IS_INTER(pCurDqLayer->pDec->pMbType[pCurDqLayer->iMbXyIndex])){		// InterMB
		if(0==pCurDqLayer->pCbp[pCurDqLayer->iMbXyIndex]){		// uiCbp==0 include SKIP
			if(!CheckRefPics(pCtx)){
				return ERR_INFO_MB_RECON_FAIL;
			}
			return WelsMbInterPrediction(pCtx,pCurDqLayer);
		}else{
			WelsMbInterConstruction(pCtx,pCurDqLayer);
		}
	}else{
		uprintf("WelsTargetMbConstruction():::::Unknown MB type: %d",pCurDqLayer->pDec->pMbType[pCurDqLayer->iMbXyIndex]);
		return ERR_INFO_MB_RECON_FAIL;
	}

	return ERR_NONE;
}

int32_t DeblockingAvailableNoInterlayer(PDqLayer pCurDqLayer,int32_t iFilterIdc){
	int32_t iMbY=pCurDqLayer->iMbY;
	int32_t iMbX=pCurDqLayer->iMbX;
	int32_t iMbXy=pCurDqLayer->iMbXyIndex;
	bool bLeftFlag=false;
	bool bTopFlag=false;

	if(2==iFilterIdc){
		bLeftFlag=(iMbX>0) && (pCurDqLayer->pSliceIdc[iMbXy]==pCurDqLayer->pSliceIdc[iMbXy-1]);
		bTopFlag=(iMbY>0) && (pCurDqLayer->pSliceIdc[iMbXy]==pCurDqLayer->pSliceIdc[iMbXy-pCurDqLayer->iMbWidth]);
	}else{		// if ( 0==iFilterIdc )
		bLeftFlag=(iMbX>0);
		bTopFlag=(iMbY>0);
	}
	return (bLeftFlag<<LEFT_FLAG_BIT)|(bTopFlag<<TOP_FLAG_BIT);
}

// brief AVC slice deblocking filtering target layer
// param dec Wels avc decoder context
// return NONE
void WelsDeblockingFilterSlice(SDecoderContext* pCtx,PDeblockingFilterMbFunc pDeblockMb){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SSliceHeaderExt* pSliceHeaderExt=&pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt;
	int32_t iMbWidth=pCurDqLayer->iMbWidth;
	int32_t iTotalMbCount=pSliceHeaderExt->sSliceHeader.pSps->uiTotalMbCount;

	SDeblockingFilter pFilter;
	memset(&pFilter,0,sizeof(pFilter));
	SFmo* pFmo=pCtx->pFmo;
	int32_t iNextMbXyIndex=0;
	int32_t iTotalNumMb=pCurDqLayer->sLayerInfo.sSliceInLayer.iTotalMbInCurSlice;
	int32_t iCountNumMb=0;
	int32_t iBoundryFlag;
	int32_t iFilterIdc=pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.uiDisableDeblockingFilterIdc;

	// Step1: parameters set
	pFilter.pCsData[0]=pCtx->pDec->pData[0];
	pFilter.pCsData[1]=pCtx->pDec->pData[1];
	pFilter.pCsData[2]=pCtx->pDec->pData[2];

	pFilter.iCsStride[0]=pCtx->pDec->iLinesize[0];
	pFilter.iCsStride[1]=pCtx->pDec->iLinesize[1];

	pFilter.eSliceType=(EWelsSliceType)pCurDqLayer->sLayerInfo.sSliceInLayer.eSliceType;

	pFilter.iSliceAlphaC0Offset=pSliceHeaderExt->sSliceHeader.iSliceAlphaC0Offset;
	pFilter.iSliceBetaOffset=pSliceHeaderExt->sSliceHeader.iSliceBetaOffset;

	pFilter.pLoopf=&pCtx->sDeblockingFunc;
	pFilter.pRefPics[0]=pCtx->sRefPic.pRefList[0];
	pFilter.pRefPics[1]=pCtx->sRefPic.pRefList[1];

	// Step2: macroblock deblocking
	if(0==iFilterIdc || 2==iFilterIdc){
		iNextMbXyIndex=pSliceHeaderExt->sSliceHeader.iFirstMbInSlice;
		pCurDqLayer->iMbX=iNextMbXyIndex%iMbWidth;
		pCurDqLayer->iMbY=iNextMbXyIndex/iMbWidth;
		pCurDqLayer->iMbXyIndex=iNextMbXyIndex;

		do{
			iBoundryFlag=DeblockingAvailableNoInterlayer(pCurDqLayer,iFilterIdc);

			pDeblockMb(pCurDqLayer,&pFilter,iBoundryFlag);

			++iCountNumMb;
			if(iCountNumMb>=iTotalNumMb){
				break;
			}

			if(pSliceHeaderExt->sSliceHeader.pPps->uiNumSliceGroups>1){
				iNextMbXyIndex=FmoNextMb(pFmo,iNextMbXyIndex);
			}else{
				++iNextMbXyIndex;
			}
			if(-1==iNextMbXyIndex || iNextMbXyIndex>=iTotalMbCount){		// slice group boundary or end of a frame
				break;
			}

			pCurDqLayer->iMbX=iNextMbXyIndex%iMbWidth;
			pCurDqLayer->iMbY=iNextMbXyIndex/iMbWidth;
			pCurDqLayer->iMbXyIndex=iNextMbXyIndex;
		} while(1);
	}
}

int32_t WelsTargetSliceConstruction(SDecoderContext* pCtx){
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	SSlice* pCurSlice=&pCurDqLayer->sLayerInfo.sSliceInLayer;
	SSliceHeader* pSliceHeader=&pCurSlice->sSliceHeaderExt.sSliceHeader;

	int32_t iTotalMbTargetLayer=pSliceHeader->pSps->uiTotalMbCount;

	int32_t iCurLayerWidth=pCurDqLayer->iMbWidth<<4;
	int32_t iCurLayerHeight=pCurDqLayer->iMbHeight<<4;

	int32_t iNextMbXyIndex=0;
	SFmo* pFmo=pCtx->pFmo;

	int32_t iTotalNumMb=pCurSlice->iTotalMbInCurSlice;
	int32_t iCountNumMb=0;
	PDeblockingFilterMbFunc pDeblockMb=WelsDeblockingMb;

	if(!pCtx->sSpsPpsCtx.bAvcBasedFlag && iCurLayerWidth!=pCtx->iCurSeqIntervalMaxPicWidth){
		return ERR_INFO_WIDTH_MISMATCH;
	}

	iNextMbXyIndex=pSliceHeader->iFirstMbInSlice;
	pCurDqLayer->iMbX=iNextMbXyIndex%pCurDqLayer->iMbWidth;
	pCurDqLayer->iMbY=iNextMbXyIndex/pCurDqLayer->iMbWidth;
	pCurDqLayer->iMbXyIndex=iNextMbXyIndex;

	if(0==iNextMbXyIndex){
		pCurDqLayer->pDec->iSpsId=pCtx->pSps->iSpsId;
		pCurDqLayer->pDec->iPpsId=pCtx->pPps->iPpsId;

		pCurDqLayer->pDec->uiQualityId=pCurDqLayer->sLayerInfo.sNalHeaderExt.uiQualityId;
	}

	do{
		if(iCountNumMb>=iTotalNumMb){
			break;
		}

		if(WelsTargetMbConstruction(pCtx)){
			uprintf("WelsTargetSliceConstruction():::MB(%d,%d) construction error. pCurSlice_type:%d",pCurDqLayer->iMbX,pCurDqLayer->iMbY,pCurSlice->eSliceType);
			return ERR_INFO_MB_RECON_FAIL;
		}

		++iCountNumMb;
		if(!pCurDqLayer->pMbCorrectlyDecodedFlag[iNextMbXyIndex]){		// already con-ed,overwrite
			pCurDqLayer->pMbCorrectlyDecodedFlag[iNextMbXyIndex]=true;
			pCtx->pDec->iMbEcedPropNum+=(pCurDqLayer->pMbRefConcealedFlag[iNextMbXyIndex] ? 1 : 0);
			++pCtx->iTotalNumMbRec;
		}

		if(pCtx->iTotalNumMbRec>iTotalMbTargetLayer){
			uprintf("WelsTargetSliceConstruction():::pCtx->iTotalNumMbRec:%d,iTotalMbTargetLayer:%d",pCtx->iTotalNumMbRec,iTotalMbTargetLayer);

			return ERR_INFO_MB_NUM_EXCEED_FAIL;
		}

		if(pSliceHeader->pPps->uiNumSliceGroups>1){
			iNextMbXyIndex=FmoNextMb(pFmo,iNextMbXyIndex);
		}else{
			++iNextMbXyIndex;
		}
		if(-1==iNextMbXyIndex || iNextMbXyIndex>=iTotalMbTargetLayer){		// slice group boundary or end of a frame
			break;
		}
		pCurDqLayer->iMbX=iNextMbXyIndex%pCurDqLayer->iMbWidth;
		pCurDqLayer->iMbY=iNextMbXyIndex/pCurDqLayer->iMbWidth;
		pCurDqLayer->iMbXyIndex=iNextMbXyIndex;
	} while(1);

	pCtx->pDec->iWidthInPixel=iCurLayerWidth;
	pCtx->pDec->iHeightInPixel=iCurLayerHeight;

	if((pCurSlice->eSliceType!=I_SLICE) && (pCurSlice->eSliceType!=P_SLICE) && (pCurSlice->eSliceType!=B_SLICE))
		return ERR_NONE;		// no error but just ignore the type unsupported

	if(1==pSliceHeader->uiDisableDeblockingFilterIdc
		 || pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.iTotalMbInCurSlice<=0){
		return ERR_NONE;	// NO_SUPPORTED_FILTER_IDX
	}else{
		WelsDeblockingFilterSlice(pCtx,pDeblockMb);
	}
	// any other filter_idc not supported here,7/22/2010

	return ERR_NONE;
}

inline int32_t WelsDecodeConstructSlice(SDecoderContext* pCtx,SNalUnit* pCurNal){
	int32_t iRet=WelsTargetSliceConstruction(pCtx);

	if(iRet){
		HandleReferenceLostL0(pCtx,pCurNal);
	}

	return iRet;
}

bool CheckRefPicturesComplete(SDecoderContext* pCtx){
	// Multi Reference,RefIdx may differ
	bool bAllRefComplete=true;
	int32_t iRealMbIdx=pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.iFirstMbInSlice;
	for(int32_t iMbIdx=0; bAllRefComplete
		  && iMbIdx<pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.iTotalMbInCurSlice; iMbIdx++){
		switch(pCtx->pCurDqLayer->pDec->pMbType[iRealMbIdx]){
			case MB_TYPE_SKIP:
			case MB_TYPE_16x16:
				bAllRefComplete&=
					pCtx->sRefPic.pRefList[LIST_0][pCtx->pCurDqLayer->pDec->pRefIndex[0][iRealMbIdx][0]]->bIsComplete;
				break;

			case MB_TYPE_16x8:
				bAllRefComplete&=
					pCtx->sRefPic.pRefList[LIST_0][pCtx->pCurDqLayer->pDec->pRefIndex[0][iRealMbIdx][0]]->bIsComplete;
				bAllRefComplete&=
					pCtx->sRefPic.pRefList[LIST_0][pCtx->pCurDqLayer->pDec->pRefIndex[0][iRealMbIdx][8]]->bIsComplete;
				break;

			case MB_TYPE_8x16:
				bAllRefComplete&=
					pCtx->sRefPic.pRefList[LIST_0][pCtx->pCurDqLayer->pDec->pRefIndex[0][iRealMbIdx][0]]->bIsComplete;
				bAllRefComplete&=
					pCtx->sRefPic.pRefList[LIST_0][pCtx->pCurDqLayer->pDec->pRefIndex[0][iRealMbIdx][2]]->bIsComplete;
				break;

			case MB_TYPE_8x8:
			case MB_TYPE_8x8_REF0:
				bAllRefComplete&=
					pCtx->sRefPic.pRefList[LIST_0][pCtx->pCurDqLayer->pDec->pRefIndex[0][iRealMbIdx][0]]->bIsComplete;
				bAllRefComplete&=
					pCtx->sRefPic.pRefList[LIST_0][pCtx->pCurDqLayer->pDec->pRefIndex[0][iRealMbIdx][2]]->bIsComplete;
				bAllRefComplete&=
					pCtx->sRefPic.pRefList[LIST_0][pCtx->pCurDqLayer->pDec->pRefIndex[0][iRealMbIdx][8]]->bIsComplete;
				bAllRefComplete&=
					pCtx->sRefPic.pRefList[LIST_0][pCtx->pCurDqLayer->pDec->pRefIndex[0][iRealMbIdx][10]]->bIsComplete;
				break;

			default:
				break;
		}
		iRealMbIdx=(pCtx->pPps->uiNumSliceGroups>1) ? FmoNextMb(pCtx->pFmo,iRealMbIdx) :
			(pCtx->pCurDqLayer->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.iFirstMbInSlice+iMbIdx);
		if(iRealMbIdx==-1)		// caused by abnormal return of FmoNextMb()
			return false;
	}

	return bAllRefComplete;
}

bool NeedErrorCon(SDecoderContext* pCtx){
	bool bNeedEC=false;
	int32_t iMbNum=pCtx->pSps->iMbWidth*pCtx->pSps->iMbHeight;
	for(int32_t i=0; i<iMbNum;++i){
		if(!pCtx->pCurDqLayer->pMbCorrectlyDecodedFlag[i]){
			bNeedEC=true;
			break;
		}
	}
	return bNeedEC;
}

// Do error concealment using frame copy method
void DoErrorConFrameCopy(SDecoderContext* pCtx){
	SPicture* pDstPic=pCtx->pDec;
	SPicture* pSrcPic=pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb;
	uint32_t uiHeightInPixelY=(pCtx->pSps->iMbHeight)<<4;
	int32_t iStrideY=pDstPic->iLinesize[0];
	int32_t iStrideUV=pDstPic->iLinesize[1];
	pCtx->pDec->iMbEcedNum=pCtx->pSps->iMbWidth*pCtx->pSps->iMbHeight;
	if((pCtx->pParam->eEcActiveIdc==ERROR_CON_FRAME_COPY) && (pCtx->pCurDqLayer->sLayerInfo.sNalHeaderExt.bIdrFlag))
		pSrcPic=NULL;		// no cross IDR method,should fill in data instead of copy
	if(pSrcPic==NULL){		// no ref pic,assign specific data to picture
		memset(pDstPic->pData[0],128,uiHeightInPixelY*iStrideY);
		memset(pDstPic->pData[1],128,(uiHeightInPixelY>>1)*iStrideUV);
		memset(pDstPic->pData[2],128,(uiHeightInPixelY>>1)*iStrideUV);
	}else
	if(pSrcPic==pDstPic){
		uprintf("DoErrorConFrameCopy()::EC memcpy overlap.");
	}else{		// has ref pic here
		memcpy(pDstPic->pData[0],pSrcPic->pData[0],uiHeightInPixelY*iStrideY);
		memcpy(pDstPic->pData[1],pSrcPic->pData[1],(uiHeightInPixelY>>1)*iStrideUV);
		memcpy(pDstPic->pData[2],pSrcPic->pData[2],(uiHeightInPixelY>>1)*iStrideUV);
	}
}
// Do error concealment using slice copy method
void DoErrorConSliceCopy(SDecoderContext* pCtx){
	int32_t iMbWidth=(int32_t)pCtx->pSps->iMbWidth;
	int32_t iMbHeight=(int32_t)pCtx->pSps->iMbHeight;
	SPicture* pDstPic=pCtx->pDec;
	SPicture* pSrcPic=pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb;
	if((pCtx->pParam->eEcActiveIdc==ERROR_CON_SLICE_COPY) && (pCtx->pCurDqLayer->sLayerInfo.sNalHeaderExt.bIdrFlag))
		pSrcPic=NULL;		// no cross IDR method,should fill in data instead of copy

	// uint8_t *pDstData[3],*pSrcData[3];
	bool* pMbCorrectlyDecodedFlag=pCtx->pCurDqLayer->pMbCorrectlyDecodedFlag;
	// Do slice copy late
	int32_t iMbXyIndex;
	uint8_t* pSrcData,* pDstData;
	uint32_t iSrcStride;		// =pSrcPic->iLinesize[0];
	uint32_t iDstStride=pDstPic->iLinesize[0];
	if(pSrcPic==pDstPic){
		uprintf("DoErrorConSliceCopy()::EC memcpy overlap.");
		return;
	}
	for(int32_t iMbY=0; iMbY<iMbHeight;++iMbY){
		for(int32_t iMbX=0; iMbX<iMbWidth;++iMbX){
			iMbXyIndex=iMbY*iMbWidth+iMbX;
			if(!pMbCorrectlyDecodedFlag[iMbXyIndex]){
				pCtx->pDec->iMbEcedNum++;
				if(pSrcPic!=NULL){
					iSrcStride=pSrcPic->iLinesize[0];
					// Y component
					pDstData=pDstPic->pData[0]+iMbY*16*iDstStride+iMbX*16;
					pSrcData=pSrcPic->pData[0]+iMbY*16*iSrcStride+iMbX*16;
					pCtx->sCopyFunc.pCopyLumaFunc(pDstData,iDstStride,pSrcData,iSrcStride);
					// U component
					pDstData=pDstPic->pData[1]+iMbY*8*iDstStride/2+iMbX*8;
					pSrcData=pSrcPic->pData[1]+iMbY*8*iSrcStride/2+iMbX*8;
					pCtx->sCopyFunc.pCopyChromaFunc(pDstData,iDstStride/2,pSrcData,iSrcStride/2);
					// V component
					pDstData=pDstPic->pData[2]+iMbY*8*iDstStride/2+iMbX*8;
					pSrcData=pSrcPic->pData[2]+iMbY*8*iSrcStride/2+iMbX*8;
					pCtx->sCopyFunc.pCopyChromaFunc(pDstData,iDstStride/2,pSrcData,iSrcStride/2);
				}else{		// pSrcPic==NULL
					// Y component
					pDstData=pDstPic->pData[0]+iMbY*16*iDstStride+iMbX*16;
					for(int32_t i=0; i<16;++i){
						memset(pDstData,128,16);
						pDstData+=iDstStride;
					}
					// U component
					pDstData=pDstPic->pData[1]+iMbY*8*iDstStride/2+iMbX*8;
					for(int32_t i=0; i<8;++i){
						memset(pDstData,128,8);
						pDstData+=iDstStride/2;
					}
					// V component
					pDstData=pDstPic->pData[2]+iMbY*8*iDstStride/2+iMbX*8;
					for(int32_t i=0; i<8;++i){
						memset(pDstData,128,8);
						pDstData+=iDstStride/2;
					}
				}		// 
			}		// !pMbCorrectlyDecodedFlag[iMbXyIndex]
		}		// iMbX
	}		// iMbY
}
void GetAvilInfoFromCorrectMb(SDecoderContext* pCtx){
	int32_t iMbWidth=(int32_t)pCtx->pSps->iMbWidth;
	int32_t iMbHeight=(int32_t)pCtx->pSps->iMbHeight;
	bool* pMbCorrectlyDecodedFlag=pCtx->pCurDqLayer->pMbCorrectlyDecodedFlag;
	PDqLayer pCurDqLayer=pCtx->pCurDqLayer;
	int32_t iInterMbCorrectNum[16];
	int32_t iMbXyIndex;

	int8_t iRefIdx;
	memset(pCtx->iECMVs,0,sizeof(int32_t)*32);
	memset(pCtx->pECRefPic,0,sizeof(SPicture*)*16);
	memset(iInterMbCorrectNum,0,sizeof(int32_t)*16);

	for(int32_t iMbY=0; iMbY<iMbHeight;++iMbY){
		for(int32_t iMbX=0; iMbX<iMbWidth;++iMbX){
			iMbXyIndex=iMbY*iMbWidth+iMbX;
			if(pMbCorrectlyDecodedFlag[iMbXyIndex] && IS_INTER(pCurDqLayer->pDec->pMbType[iMbXyIndex])){
				uint32_t iMBType=pCurDqLayer->pDec->pMbType[iMbXyIndex];
				switch(iMBType){
					case MB_TYPE_SKIP:
					case MB_TYPE_16x16:
						iRefIdx=pCurDqLayer->pDec->pRefIndex[0][iMbXyIndex][0];
						pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][0][0];
						pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][0][1];
						pCtx->pECRefPic[iRefIdx]=pCtx->sRefPic.pRefList[LIST_0][iRefIdx];
						iInterMbCorrectNum[iRefIdx]++;
						break;
					case MB_TYPE_16x8:
						iRefIdx=pCurDqLayer->pDec->pRefIndex[0][iMbXyIndex][0];
						pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][0][0];
						pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][0][1];
						pCtx->pECRefPic[iRefIdx]=pCtx->sRefPic.pRefList[LIST_0][iRefIdx];
						iInterMbCorrectNum[iRefIdx]++;

						iRefIdx=pCurDqLayer->pDec->pRefIndex[0][iMbXyIndex][8];
						pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][8][0];
						pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][8][1];
						pCtx->pECRefPic[iRefIdx]=pCtx->sRefPic.pRefList[LIST_0][iRefIdx];
						iInterMbCorrectNum[iRefIdx]++;
						break;
					case MB_TYPE_8x16:
						iRefIdx=pCurDqLayer->pDec->pRefIndex[0][iMbXyIndex][0];
						pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][0][0];
						pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][0][1];
						pCtx->pECRefPic[iRefIdx]=pCtx->sRefPic.pRefList[LIST_0][iRefIdx];
						iInterMbCorrectNum[iRefIdx]++;

						iRefIdx=pCurDqLayer->pDec->pRefIndex[0][iMbXyIndex][2];
						pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][2][0];
						pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][2][1];
						pCtx->pECRefPic[iRefIdx]=pCtx->sRefPic.pRefList[LIST_0][iRefIdx];
						iInterMbCorrectNum[iRefIdx]++;
						break;
					case MB_TYPE_8x8:
					case MB_TYPE_8x8_REF0:
					{
						uint32_t iSubMBType;
						int32_t i,j,iIIdx,iJIdx;

						for(i=0; i<4; i++){
							iSubMBType=pCurDqLayer->pSubMbType[iMbXyIndex][i];
							iIIdx=((i>>1)<<3)+((i&1)<<1);
							iRefIdx=pCurDqLayer->pDec->pRefIndex[0][iMbXyIndex][iIIdx];
							pCtx->pECRefPic[iRefIdx]=pCtx->sRefPic.pRefList[LIST_0][iRefIdx];
							switch(iSubMBType){
								case SUB_MB_TYPE_8x8:
									pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx][0];
									pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx][1];
									iInterMbCorrectNum[iRefIdx]++;

									break;
								case SUB_MB_TYPE_8x4:
									pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx][0];
									pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx][1];


									pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx+4][0];
									pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx+4][1];
									iInterMbCorrectNum[iRefIdx]+=2;

									break;
								case SUB_MB_TYPE_4x8:
									pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx][0];
									pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx][1];


									pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx+1][0];
									pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx+1][1];
									iInterMbCorrectNum[iRefIdx]+=2;
									break;
								case SUB_MB_TYPE_4x4:
								{
									for(j=0; j<4; j++){
										iJIdx=((j>>1)<<2)+(j&1);
										pCtx->iECMVs[iRefIdx][0]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx+iJIdx][0];
										pCtx->iECMVs[iRefIdx][1]+=pCurDqLayer->pDec->pMv[0][iMbXyIndex][iIIdx+iJIdx][1];
									}
									iInterMbCorrectNum[iRefIdx]+=4;
								}
								break;
								default:
									break;
							}
						}
					}
					break;
					default:
						break;
				}
			}		// pMbCorrectlyDecodedFlag[iMbXyIndex]
		}		// iMbX
	}		// iMbY
	for(int32_t i=0; i<16; i++){
		if(iInterMbCorrectNum[i]){
			pCtx->iECMVs[i][0]=pCtx->iECMVs[i][0]/iInterMbCorrectNum[i];
			pCtx->iECMVs[i][1]=pCtx->iECMVs[i][1]/iInterMbCorrectNum[i];
		}
	}
}
// Do error concealment using slice MV copy method
void DoMbECMvCopy(SDecoderContext* pCtx,SPicture* pDec,SPicture* pRef,int32_t iMbXy,int32_t iMbX,int32_t iMbY,sMCRefMember* pMCRefMem){
	if(pDec==pRef){
		return;		// for protection,shall never go into this logic,error info printed outside.
	}
	int16_t iMVs[2];
	int32_t iMbXInPix=iMbX<<4;
	int32_t iMbYInPix=iMbY<<4;
	int32_t iScale0;
	int32_t iScale1;
	uint8_t* pDst[3];
	int32_t iCurrPoc=pDec->iFramePoc;
	pDst[0]=pDec->pData[0]+iMbXInPix+iMbYInPix*pMCRefMem->iDstLineLuma;
	pDst[1]=pDec->pData[1]+(iMbXInPix>>1)+(iMbYInPix>>1)*pMCRefMem->iDstLineChroma;
	pDst[2]=pDec->pData[2]+(iMbXInPix>>1)+(iMbYInPix>>1)*pMCRefMem->iDstLineChroma;
	if(pDec->bIdrFlag==true || pCtx->pECRefPic[0]==NULL){
		uint8_t* pSrcData;
		// Y component
		pSrcData=pMCRefMem->pSrcY+iMbY*16*pMCRefMem->iSrcLineLuma+iMbX*16;
		pCtx->sCopyFunc.pCopyLumaFunc(pDst[0],pMCRefMem->iDstLineLuma,pSrcData,pMCRefMem->iSrcLineLuma);
		// U component
		pSrcData=pMCRefMem->pSrcU+iMbY*8*pMCRefMem->iSrcLineChroma+iMbX*8;
		pCtx->sCopyFunc.pCopyChromaFunc(pDst[1],pMCRefMem->iDstLineChroma,pSrcData,pMCRefMem->iSrcLineChroma);
		// V component
		pSrcData=pMCRefMem->pSrcV+iMbY*8*pMCRefMem->iSrcLineChroma+iMbX*8;
		pCtx->sCopyFunc.pCopyChromaFunc(pDst[2],pMCRefMem->iDstLineChroma,pSrcData,pMCRefMem->iSrcLineChroma);
		return;
	}

	if(pCtx->pECRefPic[0]){
		if(pCtx->pECRefPic[0]==pRef){
			iMVs[0]=pCtx->iECMVs[0][0];
			iMVs[1]=pCtx->iECMVs[0][1];
		}else{
			iScale0=pCtx->pECRefPic[0]->iFramePoc-iCurrPoc;
			iScale1=pRef->iFramePoc-iCurrPoc;
			iMVs[0]=iScale0==0 ? 0 : pCtx->iECMVs[0][0]*iScale1/iScale0;
			iMVs[1]=iScale0==0 ? 0 : pCtx->iECMVs[0][1]*iScale1/iScale0;
		}
		pMCRefMem->pDstY=pDst[0];
		pMCRefMem->pDstU=pDst[1];
		pMCRefMem->pDstV=pDst[2];
		int32_t iFullMVx=(iMbXInPix<<2)+iMVs[0];		// quarter pixel
		int32_t iFullMVy=(iMbYInPix<<2)+iMVs[1];
		// only use to be output pixels to EC;
		int32_t iPicWidthLeftLimit=0;
		int32_t iPicHeightTopLimit=0;
		int32_t iPicWidthRightLimit=pMCRefMem->iPicWidth;
		int32_t iPicHeightBottomLimit=pMCRefMem->iPicHeight;
		if(pCtx->pSps->bFrameCroppingFlag){
			iPicWidthLeftLimit=0+pCtx->sFrameCrop.iLeftOffset*2;
			iPicWidthRightLimit=(pMCRefMem->iPicWidth-pCtx->sFrameCrop.iRightOffset*2);
			iPicHeightTopLimit=0+pCtx->sFrameCrop.iTopOffset*2;
			iPicHeightBottomLimit=(pMCRefMem->iPicHeight-pCtx->sFrameCrop.iTopOffset*2);
		}
		// further make sure no need to expand picture
		int32_t iMinLeftOffset=(iPicWidthLeftLimit+2)*(1<<2);
		int32_t iMaxRightOffset=((iPicWidthRightLimit-18)*(1<<2));
		int32_t iMinTopOffset=(iPicHeightTopLimit+2)*(1<<2);
		int32_t iMaxBottomOffset=((iPicHeightBottomLimit-18)*(1<<2));
		if(iFullMVx<iMinLeftOffset){
			iFullMVx=(iFullMVx>>2)*(1<<2);
			iFullMVx=WELS_MAX(iPicWidthLeftLimit,iFullMVx);
		}else
		if(iFullMVx>iMaxRightOffset){
			iFullMVx=(iFullMVx>>2)*(1<<2);
			iFullMVx=WELS_MIN(((iPicWidthRightLimit-16)*(1<<2)),iFullMVx);
		}
		if(iFullMVy<iMinTopOffset){
			iFullMVy=(iFullMVy>>2)*(1<<2);
			iFullMVy=WELS_MAX(iPicHeightTopLimit,iFullMVy);
		}else
		if(iFullMVy>iMaxBottomOffset){
			iFullMVy=(iFullMVy>>2)*(1<<2);
			iFullMVy=WELS_MIN(((iPicHeightBottomLimit-16)*(1<<2)),iFullMVy);
		}
		iMVs[0]=iFullMVx-(iMbXInPix<<2);
		iMVs[1]=iFullMVy-(iMbYInPix<<2);
		BaseMC(pCtx,pMCRefMem,-1,-1,iMbXInPix,iMbYInPix,&pCtx->sMcFunc,16,16,iMVs);
	}
	return;
}



void DoErrorConSliceMVCopy(SDecoderContext* pCtx){
	int32_t iMbWidth=(int32_t)pCtx->pSps->iMbWidth;
	int32_t iMbHeight=(int32_t)pCtx->pSps->iMbHeight;
	SPicture* pDstPic=pCtx->pDec;
	SPicture* pSrcPic=pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb;

	bool* pMbCorrectlyDecodedFlag=pCtx->pCurDqLayer->pMbCorrectlyDecodedFlag;
	int32_t iMbXyIndex;
	uint8_t* pDstData;
	uint32_t iDstStride=pDstPic->iLinesize[0];
	sMCRefMember sMCRefMem;
	if(pSrcPic!=NULL){
		sMCRefMem.iSrcLineLuma=pSrcPic->iLinesize[0];
		sMCRefMem.iSrcLineChroma=pSrcPic->iLinesize[1];
		sMCRefMem.pSrcY=pSrcPic->pData[0];
		sMCRefMem.pSrcU=pSrcPic->pData[1];
		sMCRefMem.pSrcV=pSrcPic->pData[2];
		sMCRefMem.iDstLineLuma=pDstPic->iLinesize[0];
		sMCRefMem.iDstLineChroma=pDstPic->iLinesize[1];
		sMCRefMem.iPicWidth=pDstPic->iWidthInPixel;
		sMCRefMem.iPicHeight=pDstPic->iHeightInPixel;
		if(pDstPic==pSrcPic){
			// output error info,EC will be ignored in DoMbECMvCopy
			uprintf("DoErrorConSliceMVCopy()::EC memcpy overlap.");
			return;
		}
	}

	for(int32_t iMbY=0; iMbY<iMbHeight;++iMbY){
		for(int32_t iMbX=0; iMbX<iMbWidth;++iMbX){
			iMbXyIndex=iMbY*iMbWidth+iMbX;
			if(!pMbCorrectlyDecodedFlag[iMbXyIndex]){
				pCtx->pDec->iMbEcedNum++;
				if(pSrcPic!=NULL){
					DoMbECMvCopy(pCtx,pDstPic,pSrcPic,iMbXyIndex,iMbX,iMbY,&sMCRefMem);
				}else{		// pSrcPic==NULL
					// Y component
					pDstData=pDstPic->pData[0]+iMbY*16*iDstStride+iMbX*16;
					for(int32_t i=0; i<16;++i){
						memset(pDstData,128,16);
						pDstData+=iDstStride;
					}
					// U component
					pDstData=pDstPic->pData[1]+iMbY*8*iDstStride/2+iMbX*8;
					for(int32_t i=0; i<8;++i){
						memset(pDstData,128,8);
						pDstData+=iDstStride/2;
					}
					// V component
					pDstData=pDstPic->pData[2]+iMbY*8*iDstStride/2+iMbX*8;
					for(int32_t i=0; i<8;++i){
						memset(pDstData,128,8);
						pDstData+=iDstStride/2;
					}
				}		// 

			}		// !pMbCorrectlyDecodedFlag[iMbXyIndex]
		}		// iMbX
	}		// iMbY
}

// ImplementErrorConceal
// Do actual error concealment
void ImplementErrorCon(SDecoderContext* pCtx){
	if(ERROR_CON_DISABLE==pCtx->pParam->eEcActiveIdc){
		pCtx->iErrorCode|=dsBitstreamError;
		return;
	}else
	if((ERROR_CON_FRAME_COPY==pCtx->pParam->eEcActiveIdc)
			  || (ERROR_CON_FRAME_COPY_CROSS_IDR==pCtx->pParam->eEcActiveIdc)){
		DoErrorConFrameCopy(pCtx);
	}else
	if((ERROR_CON_SLICE_COPY==pCtx->pParam->eEcActiveIdc)
			  || (ERROR_CON_SLICE_COPY_CROSS_IDR==pCtx->pParam->eEcActiveIdc)
			  || (ERROR_CON_SLICE_COPY_CROSS_IDR_FREEZE_RES_CHANGE==pCtx->pParam->eEcActiveIdc)){
		DoErrorConSliceCopy(pCtx);
	}else
	if((ERROR_CON_SLICE_MV_COPY_CROSS_IDR==pCtx->pParam->eEcActiveIdc)
			  || (ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE==pCtx->pParam->eEcActiveIdc)){
		GetAvilInfoFromCorrectMb(pCtx);
		DoErrorConSliceMVCopy(pCtx);
	}		// TODO add other EC methods here in the future
	pCtx->iErrorCode|=dsDataErrorConcealed;
	pCtx->pDec->bIsComplete=false;		// Set complete flag to false after do EC.
}

static inline int32_t DecodeFrameConstruction(SDecoderContext* pCtx,uint8_t** ppDst,SBufferInfo* pDstInfo){
	PDqLayer pCurDq=pCtx->pCurDqLayer;
	SPicture* pPic=pCtx->pDec;

	const int32_t kiWidth=pCurDq->iMbWidth<<4;
	const int32_t kiHeight=pCurDq->iMbHeight<<4;

	const int32_t kiTotalNumMbInCurLayer=pCurDq->iMbWidth*pCurDq->iMbHeight;
	bool bFrameCompleteFlag=true;

	if(pPic->bNewSeqBegin){
		memcpy(&(pCtx->sFrameCrop),&(pCurDq->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.pSps->sFrameCrop),sizeof(SPosOffset));
		pCtx->bParamSetsLostFlag=false;
		if(pCtx->iTotalNumMbRec==kiTotalNumMbInCurLayer){
			pCtx->bPrintFrameErrorTraceFlag=true;
			//uprintf("DecodeFrameConstruction(): will output first frame of new sequence,%d x %d,crop_left:%d,crop_right:%d,crop_top:%d,crop_bottom:%d,ignored error packet:%d.",kiWidth,kiHeight,pCtx->sFrameCrop.iLeftOffset,pCtx->sFrameCrop.iRightOffset,pCtx->sFrameCrop.iTopOffset,pCtx->sFrameCrop.iBottomOffset,pCtx->iIgnoredErrorInfoPacketCount);
			pCtx->iIgnoredErrorInfoPacketCount=0;
		}
	}

	const int32_t kiActualWidth=kiWidth-(pCtx->sFrameCrop.iLeftOffset+pCtx->sFrameCrop.iRightOffset)*2;
	const int32_t kiActualHeight=kiHeight-(pCtx->sFrameCrop.iTopOffset+pCtx->sFrameCrop.iBottomOffset)*2;


	if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
		FATAL("WTF");
		// if ((pCtx->pDecoderStatistics->uiWidth !=(unsigned int) kiActualWidth)
		// || (pCtx->pDecoderStatistics->uiHeight !=(unsigned int) kiActualHeight)) {
		// pCtx->pDecoderStatistics->uiResolutionChangeTimes++;
		// pCtx->pDecoderStatistics->uiWidth=kiActualWidth;
		// pCtx->pDecoderStatistics->uiHeight=kiActualHeight;
		// }
		// UpdateDecStatNoFreezingInfo (pCtx);
	}

	if(pCtx->iTotalNumMbRec!=kiTotalNumMbInCurLayer){
		uprintf("DecodeFrameConstruction(): iTotalNumMbRec:%d,total_num_mb_sps:%d,cur_layer_mb_width:%d,cur_layer_mb_height:%d ",pCtx->iTotalNumMbRec,kiTotalNumMbInCurLayer,pCurDq->iMbWidth,pCurDq->iMbHeight);
		bFrameCompleteFlag=false;		// return later after output buffer is done
		if(pCtx->bInstantDecFlag){		// no-delay decoding,wait for new slice
			return ERR_INFO_MB_NUM_INADEQUATE;
		}
	}else
	if(pCurDq->sLayerInfo.sNalHeaderExt.bIdrFlag
			  && (pCtx->iErrorCode==dsErrorFree)){		// complete non-ECed IDR frame done
		pCtx->pDec->bIsComplete=true;
		pCtx->bFreezeOutput=false;
	}

	pCtx->iTotalNumMbRec=0;

	// // // output:::normal path
	pDstInfo->uiOutYuvTimeStamp=pPic->uiTimeStamp;
	ppDst[0]=pPic->pData[0];
	ppDst[1]=pPic->pData[1];
	ppDst[2]=pPic->pData[2];

	pDstInfo->UsrData.sSystemBuffer.iFormat=videoFormatI420;

	pDstInfo->UsrData.sSystemBuffer.iWidth=kiActualWidth;
	pDstInfo->UsrData.sSystemBuffer.iHeight=kiActualHeight;
	pDstInfo->UsrData.sSystemBuffer.iStride[0]=pPic->iLinesize[0];
	pDstInfo->UsrData.sSystemBuffer.iStride[1]=pPic->iLinesize[1];
	ppDst[0]=ppDst[0]+pCtx->sFrameCrop.iTopOffset*2*pPic->iLinesize[0]+pCtx->sFrameCrop.iLeftOffset*2;
	ppDst[1]=ppDst[1]+pCtx->sFrameCrop.iTopOffset*pPic->iLinesize[1]+pCtx->sFrameCrop.iLeftOffset;
	ppDst[2]=ppDst[2]+pCtx->sFrameCrop.iTopOffset*pPic->iLinesize[1]+pCtx->sFrameCrop.iLeftOffset;
	for(int i=0; i<3;++i){
		pDstInfo->pDst[i]=ppDst[i];
	}
	pDstInfo->iBufferStatus=1;
	bool bOutResChange=false;
	bOutResChange=(pCtx->iLastImgWidthInPixel!=pDstInfo->UsrData.sSystemBuffer.iWidth)
		 || (pCtx->iLastImgHeightInPixel!=pDstInfo->UsrData.sSystemBuffer.iHeight);
	pCtx->iLastImgWidthInPixel=pDstInfo->UsrData.sSystemBuffer.iWidth;
	pCtx->iLastImgHeightInPixel=pDstInfo->UsrData.sSystemBuffer.iHeight;
	if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE)		// no buffer output if EC is disabled and frame incomplete
		pDstInfo->iBufferStatus=(int32_t)(bFrameCompleteFlag
											  && pPic->bIsComplete);		// When EC disable,ECed picture not output
	else if((pCtx->pParam->eEcActiveIdc==ERROR_CON_SLICE_COPY_CROSS_IDR_FREEZE_RES_CHANGE
		 || pCtx->pParam->eEcActiveIdc==ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE)
			  && pCtx->iErrorCode && bOutResChange)
		pCtx->bFreezeOutput=true;

	if(pDstInfo->iBufferStatus==0){
		if(!bFrameCompleteFlag)
			pCtx->iErrorCode|=dsBitstreamError;
		return ERR_INFO_MB_NUM_INADEQUATE;
	}
	if(pCtx->bFreezeOutput){
		pDstInfo->iBufferStatus=0;
		if(pPic->bNewSeqBegin){
			uprintf("DecodeFrameConstruction():New sequence detected,but freezed,correct MBs (%d) out of whole MBs (%d).",kiTotalNumMbInCurLayer-pCtx->iMbEcedNum,kiTotalNumMbInCurLayer);
		}
	}
	pCtx->iMbEcedNum=pPic->iMbEcedNum;
	pCtx->iMbNum=pPic->iMbNum;
	pCtx->iMbEcedPropNum=pPic->iMbEcedPropNum;
	if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
		// FATAL("STATGONE");
	// if (pDstInfo->iBufferStatus && ((pCtx->pDecoderStatistics->uiWidth !=(unsigned int) kiActualWidth)
	// || (pCtx->pDecoderStatistics->uiHeight !=(unsigned int) kiActualHeight))) {
	// pCtx->pDecoderStatistics->uiResolutionChangeTimes++;
	// pCtx->pDecoderStatistics->uiWidth=kiActualWidth;
	// pCtx->pDecoderStatistics->uiHeight=kiActualHeight;
	// }
	// UpdateDecStat (pCtx,pDstInfo->iBufferStatus !=0);
	}
	return ERR_NONE;
}

static int32_t AddLongTermToList(SRefPic* pRefPic,SPicture* pPic,int32_t iLongTermFrameIdx,uint32_t uiLongTermPicNum){
	int32_t i=0;

	pPic->bUsedAsRef=true;
	pPic->bIsLongRef=true;
	pPic->iLongTermFrameIdx=iLongTermFrameIdx;
	pPic->uiLongTermPicNum=uiLongTermPicNum;
	if(pRefPic->uiLongRefCount[LIST_0]==0){
		pRefPic->pLongRefList[LIST_0][pRefPic->uiLongRefCount[LIST_0]]=pPic;
	}else{
		for(i=0; i<pRefPic->uiLongRefCount[LIST_0]; i++){
			if(!pRefPic->pLongRefList[LIST_0][i]){
				return ERR_INFO_INVALID_PTR;
			}
			if(pRefPic->pLongRefList[LIST_0][i]->iLongTermFrameIdx>pPic->iLongTermFrameIdx){
				break;
			}
		}
		memmove(&pRefPic->pLongRefList[LIST_0][i+1],&pRefPic->pLongRefList[LIST_0][i],(pRefPic->uiLongRefCount[LIST_0]-i)*sizeof(SPicture*));
		pRefPic->pLongRefList[LIST_0][i]=pPic;
	}

	pRefPic->uiLongRefCount[LIST_0]++;
	return ERR_NONE;
}

static SPicture* WelsDelShortFromList(SRefPic* pRefPic,int32_t iFrameNum){
	int32_t i=0;
	int32_t iMoveSize=0;
	SPicture* pPic=NULL;

	for(i=0; i<pRefPic->uiShortRefCount[LIST_0]; i++){
		if(pRefPic->pShortRefList[LIST_0][i]->iFrameNum==iFrameNum){
			iMoveSize=pRefPic->uiShortRefCount[LIST_0]-i-1;
			pPic=pRefPic->pShortRefList[LIST_0][i];
			pPic->bUsedAsRef=false;
			pRefPic->pShortRefList[LIST_0][i]=NULL;
			if(iMoveSize>0){
				memmove(&pRefPic->pShortRefList[LIST_0][i],&pRefPic->pShortRefList[LIST_0][i+1],iMoveSize*sizeof(SPicture*));
			}
			pRefPic->uiShortRefCount[LIST_0]--;
			pRefPic->pShortRefList[LIST_0][pRefPic->uiShortRefCount[LIST_0]]=NULL;
			break;
		}
	}
	return pPic;
}

static SPicture* WelsDelShortFromListSetUnref(SRefPic* pRefPic,int32_t iFrameNum){
	SPicture* pPic=WelsDelShortFromList(pRefPic,iFrameNum);
	if(pPic){
		SetUnRef(pPic);
	}
	return pPic;
}

static SPicture* WelsDelLongFromList(SRefPic* pRefPic,uint32_t uiLongTermFrameIdx){
	SPicture* pPic=NULL;
	int32_t i=0;
	for(i=0; i<pRefPic->uiLongRefCount[LIST_0]; i++){
		pPic=pRefPic->pLongRefList[LIST_0][i];
		if(pPic->iLongTermFrameIdx==(int32_t)uiLongTermFrameIdx){
			int32_t iMoveSize=pRefPic->uiLongRefCount[LIST_0]-i-1;
			pPic->bUsedAsRef=false;
			pPic->bIsLongRef=false;
			if(iMoveSize>0){
				memmove(&pRefPic->pLongRefList[LIST_0][i],&pRefPic->pLongRefList[LIST_0][i+1],iMoveSize*sizeof(SPicture*));
			}
			pRefPic->uiLongRefCount[LIST_0]--;
			pRefPic->pLongRefList[LIST_0][pRefPic->uiLongRefCount[LIST_0]]=NULL;
			return pPic;
		}
	}
	return NULL;
}

static SPicture* WelsDelLongFromListSetUnref(SRefPic* pRefPic,uint32_t uiLongTermFrameIdx){
	SPicture* pPic=WelsDelLongFromList(pRefPic,uiLongTermFrameIdx);
	if(pPic){
		SetUnRef(pPic);
	}
	return pPic;
}

static int32_t MarkAsLongTerm(SRefPic* pRefPic,int32_t iFrameNum,int32_t iLongTermFrameIdx,uint32_t uiLongTermPicNum){
	SPicture* pPic=NULL;
	int32_t i=0;
	int32_t iRet=ERR_NONE;
	WelsDelLongFromListSetUnref(pRefPic,iLongTermFrameIdx);

	for(i=0; i<pRefPic->uiRefCount[LIST_0]; i++){
		pPic=pRefPic->pRefList[LIST_0][i];
		if(pPic->iFrameNum==iFrameNum && !pPic->bIsLongRef){
			iRet=AddLongTermToList(pRefPic,pPic,iLongTermFrameIdx,uiLongTermPicNum);
			break;
		}
	}

	return iRet;
}

static int32_t MMCOProcess(SDecoderContext* pCtx,SRefPic* pRefPic,uint32_t uiMmcoType,int32_t iShortFrameNum,uint32_t uiLongTermPicNum,int32_t iLongTermFrameIdx,int32_t iMaxLongTermFrameIdx){
	SPicture* pPic=NULL;
	int32_t i=0;
	int32_t iRet=ERR_NONE;

	switch(uiMmcoType){
		case MMCO_SHORT2UNUSED:
			pPic=WelsDelShortFromListSetUnref(pRefPic,iShortFrameNum);
			if(pPic==NULL){
				uprintf("MMCO_SHORT2UNUSED: delete an empty entry from short term list");
			}
			break;
		case MMCO_LONG2UNUSED:
			pPic=WelsDelLongFromListSetUnref(pRefPic,uiLongTermPicNum);
			if(pPic==NULL){
				uprintf("MMCO_LONG2UNUSED: delete an empty entry from long term list");
			}
			break;
		case MMCO_SHORT2LONG:
			if(iLongTermFrameIdx>pRefPic->iMaxLongTermFrameIdx){
				return ERR_INFO_INVALID_MMCO_LONG_TERM_IDX_EXCEED_MAX;
			}
			pPic=WelsDelShortFromList(pRefPic,iShortFrameNum);
			if(pPic==NULL){
				uprintf("MMCO_LONG2LONG: delete an empty entry from short term list");
				break;
			}
			WelsDelLongFromListSetUnref(pRefPic,iLongTermFrameIdx);
			pCtx->bCurAuContainLtrMarkSeFlag=true;
			pCtx->iFrameNumOfAuMarkedLtr=iShortFrameNum;
			uprintf("ex_mark_avc():::MMCO_SHORT2LONG:::LTR marking....iFrameNum: %d",pCtx->iFrameNumOfAuMarkedLtr);

			MarkAsLongTerm(pRefPic,iShortFrameNum,iLongTermFrameIdx,uiLongTermPicNum);
			break;
		case MMCO_SET_MAX_LONG:
			pRefPic->iMaxLongTermFrameIdx=iMaxLongTermFrameIdx;
			for(i=0; i<pRefPic->uiLongRefCount[LIST_0]; i++){
				if(pRefPic->pLongRefList[LIST_0][i]->iLongTermFrameIdx>pRefPic->iMaxLongTermFrameIdx){
					WelsDelLongFromListSetUnref(pRefPic,pRefPic->pLongRefList[LIST_0][i]->iLongTermFrameIdx);
				}
			}
			break;
		case MMCO_RESET:
			WelsResetRefPic(pCtx);
			pCtx->pLastDecPicInfo->bLastHasMmco5=true;
			break;
		case MMCO_LONG:
			if(iLongTermFrameIdx>pRefPic->iMaxLongTermFrameIdx){
				return ERR_INFO_INVALID_MMCO_LONG_TERM_IDX_EXCEED_MAX;
			}
			WelsDelLongFromListSetUnref(pRefPic,iLongTermFrameIdx);
			if(pRefPic->uiLongRefCount[LIST_0]+pRefPic->uiShortRefCount[LIST_0]>=WELS_MAX(1,pCtx->pSps->iNumRefFrames)){
				return ERR_INFO_INVALID_MMCO_REF_NUM_OVERFLOW;
			}
			pCtx->bCurAuContainLtrMarkSeFlag=true;
			pCtx->iFrameNumOfAuMarkedLtr=pCtx->iFrameNum;
			uprintf("ex_mark_avc():::MMCO_LONG:::LTR marking....iFrameNum: %d", pCtx->iFrameNum);
			iRet=AddLongTermToList(pRefPic,pCtx->pDec,iLongTermFrameIdx,uiLongTermPicNum);
			break;
		default:
			break;
	}

	return iRet;
}

static int32_t MMCO(SDecoderContext* pCtx,SRefPic* pRefPic,SRefPicMarking* pRefPicMarking){
	SSps* pSps=pCtx->pCurDqLayer->sLayerInfo.pSps;
	int32_t i=0;
	int32_t iRet=ERR_NONE;
	for(i=0; i<MAX_MMCO_COUNT && pRefPicMarking->sMmcoRef[i].uiMmcoType!=MMCO_END; i++){
		uint32_t uiMmcoType=pRefPicMarking->sMmcoRef[i].uiMmcoType;
		int32_t iShortFrameNum=(pCtx->iFrameNum-pRefPicMarking->sMmcoRef[i].iDiffOfPicNum)&((
			1<<pSps->uiLog2MaxFrameNum)-1);
		uint32_t uiLongTermPicNum=pRefPicMarking->sMmcoRef[i].uiLongTermPicNum;
		int32_t iLongTermFrameIdx=pRefPicMarking->sMmcoRef[i].iLongTermFrameIdx;
		int32_t iMaxLongTermFrameIdx=pRefPicMarking->sMmcoRef[i].iMaxLongTermFrameIdx;
		if(uiMmcoType>MMCO_LONG){
			return ERR_INFO_INVALID_MMCO_OPCODE_BASE;
		}
		iRet=MMCOProcess(pCtx,pRefPic,uiMmcoType,iShortFrameNum,uiLongTermPicNum,iLongTermFrameIdx,iMaxLongTermFrameIdx);
		if(iRet!=ERR_NONE){
			return iRet;
		}
	}
	if(i==MAX_MMCO_COUNT){		// although Rec does not handle this condition,we here prohibit too many MMCO op
		return ERR_INFO_INVALID_MMCO_NUM;
	}

	return ERR_NONE;
}

static int32_t SlidingWindow(SDecoderContext* pCtx,SRefPic* pRefPic){
	SPicture* pPic=NULL;
	int32_t i=0;

	if(pRefPic->uiShortRefCount[LIST_0]+pRefPic->uiLongRefCount[LIST_0]>=pCtx->pSps->iNumRefFrames){
		if(pRefPic->uiShortRefCount[LIST_0]==0){
			FATAL("No reference picture in short term list when sliding window");
			return ERR_INFO_INVALID_MMCO_REF_NUM_NOT_ENOUGH;
		}
		for(i=pRefPic->uiShortRefCount[LIST_0]-1; i>=0; i--){
			pPic=WelsDelShortFromList(pRefPic,pRefPic->pShortRefList[LIST_0][i]->iFrameNum);
			if(pPic){
				SetUnRef(pPic);
				break;
			}else{
				return ERR_INFO_INVALID_MMCO_REF_NUM_OVERFLOW;
			}
		}
	}
	return ERR_NONE;
}

int32_t GetLTRFrameIndex(SRefPic* pRefPic,int32_t iAncLTRFrameNum){
	int32_t iLTRFrameIndex=-1;
	SPicture* pPic;
	for(int i=0; i<pRefPic->uiLongRefCount[0];++i){
		pPic=pRefPic->pLongRefList[LIST_0][i];
		if(pPic->iFrameNum==iAncLTRFrameNum){
			return (pPic->iLongTermFrameIdx);
		}
	}
	return iLTRFrameIndex;
}

static int32_t RemainOneBufferInDpbForEC(SDecoderContext* pCtx,SRefPic* pRefPic){
	int32_t iRet=ERR_NONE;
	if(pRefPic->uiShortRefCount[0]+pRefPic->uiLongRefCount[0]<pCtx->pSps->iNumRefFrames)
		return iRet;

	if(pRefPic->uiShortRefCount[0]>0){
		iRet=SlidingWindow(pCtx,pRefPic);
	}else{		// all LTR,remove the smallest long_term_frame_idx
		int32_t iLongTermFrameIdx=0;
		int32_t iMaxLongTermFrameIdx=pRefPic->iMaxLongTermFrameIdx;
		int32_t iCurrLTRFrameIdx=GetLTRFrameIndex(pRefPic,pCtx->iFrameNumOfAuMarkedLtr);
		while((pRefPic->uiLongRefCount[0]>=pCtx->pSps->iNumRefFrames) && (iLongTermFrameIdx<=iMaxLongTermFrameIdx)){
			if(iLongTermFrameIdx==iCurrLTRFrameIdx){
				iLongTermFrameIdx++;
				continue;
			}
			WelsDelLongFromListSetUnref(pRefPic,iLongTermFrameIdx);
			iLongTermFrameIdx++;
		}
	}
	if(pRefPic->uiShortRefCount[0]+pRefPic->uiLongRefCount[0]>=
		pCtx->pSps->iNumRefFrames){		// fail to remain one empty buffer in DPB
		uprintf("RemainOneBufferInDpbForEC(): empty one DPB failed for EC!");
		iRet=ERR_INFO_REF_COUNT_OVERFLOW;
	}

	return iRet;
}
int32_t WelsMarkAsRef(SDecoderContext* pCtx){
	SPicture* pDec=pCtx->pDec;
	SRefPic* pRefPic=&pCtx->sRefPic;
	SRefPicMarking* pRefPicMarking=pCtx->pCurDqLayer->pRefPicMarking;
	SAccessUnit* pCurAU=pCtx->pAccessUnitList;
	bool bIsIDRAU=false;
	uint32_t j;

	int32_t iRet=ERR_NONE;

	pDec->uiQualityId=pCtx->pCurDqLayer->sLayerInfo.sNalHeaderExt.uiQualityId;
	pDec->uiTemporalId=pCtx->pCurDqLayer->sLayerInfo.sNalHeaderExt.uiTemporalId;
	pDec->iSpsId=pCtx->pSps->iSpsId;
	pDec->iPpsId=pCtx->pPps->iPpsId;

	for(j=pCurAU->uiStartPos; j<=pCurAU->uiEndPos; j++){
		if(pCurAU->pNalUnitsList[j]->sNalHeaderExt.sNalUnitHeader.eNalUnitType==NAL_UNIT_CODED_SLICE_IDR
			 || pCurAU->pNalUnitsList[j]->sNalHeaderExt.bIdrFlag){
			bIsIDRAU=true;
			break;
		}
	}
	if(bIsIDRAU){
		if(pRefPicMarking->bLongTermRefFlag){
			pRefPic->iMaxLongTermFrameIdx=0;
			AddLongTermToList(pRefPic,pDec,0,0);
		}else{
			pRefPic->iMaxLongTermFrameIdx=-1;
		}
	}else{
		if(pRefPicMarking->bAdaptiveRefPicMarkingModeFlag){
			iRet=MMCO(pCtx,pRefPic,pRefPicMarking);
			if(iRet!=ERR_NONE){
				if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
					iRet=RemainOneBufferInDpbForEC(pCtx,pRefPic);
					WELS_VERIFY_RETURN_IF(iRet,iRet);
				}else{
					return iRet;
				}
			}

			if(pCtx->pLastDecPicInfo->bLastHasMmco5){
				pDec->iFrameNum=0;
				pDec->iFramePoc=0;
			}

		}else{
			iRet=SlidingWindow(pCtx,pRefPic);
			if(iRet!=ERR_NONE){
				if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
					iRet=RemainOneBufferInDpbForEC(pCtx,pRefPic);
					WELS_VERIFY_RETURN_IF(iRet,iRet);
				}else{
					return iRet;
				}
			}
		}
	}

	if(!pDec->bIsLongRef){
		if(pRefPic->uiLongRefCount[LIST_0]+pRefPic->uiShortRefCount[LIST_0]>=WELS_MAX(1,pCtx->pSps->iNumRefFrames)){
			if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
				iRet=RemainOneBufferInDpbForEC(pCtx,pRefPic);
				WELS_VERIFY_RETURN_IF(iRet,iRet);
			}else{
				return ERR_INFO_INVALID_MMCO_REF_NUM_OVERFLOW;
			}
		}
		iRet=AddShortTermToList(pRefPic,pDec);
	}

	return iRet;
}

int32_t DecodeCurrentAccessUnit(SDecoderContext* pCtx,uint8_t** ppDst,SBufferInfo* pDstInfo){
	SNalUnit* pNalCur=pCtx->pNalCur=NULL;
	SAccessUnit* pCurAu=pCtx->pAccessUnitList;

	int32_t iIdx=pCurAu->uiStartPos;
	int32_t iEndIdx=pCurAu->uiEndPos;

	int32_t iPpsId=0;
	int32_t iRet=ERR_NONE;

	bool bAllRefComplete=true;		// Assume default all ref picutres are complete

	const uint8_t kuiTargetLayerDqId=GetTargetDqId(pCtx->uiTargetDqId,pCtx->pParam);
	const uint8_t kuiDependencyIdMax=(kuiTargetLayerDqId&0x7F)>>4;
	int16_t iLastIdD=-1,iLastIdQ=-1;
	int16_t iCurrIdD=0,iCurrIdQ=0;
	pCtx->uiNalRefIdc=0;
	bool bFreshSliceAvailable=true;		// Another fresh slice comingup for given dq layer,for multiple slices in case of header parts of slices sometimes loss over error-prone channels,8/14/2008

	// update pCurDqLayer at the starting of AU decoding
	if(pCtx->bInitialDqLayersMem || pCtx->pCurDqLayer==NULL){
		pCtx->pCurDqLayer=pCtx->pDqLayersList[0];
	}

	InitCurDqLayerData(pCtx,pCtx->pCurDqLayer);

	pNalCur=pCurAu->pNalUnitsList[iIdx];
	while(iIdx<=iEndIdx){
		PDqLayer dq_cur=pCtx->pCurDqLayer;
		SLayerInfo pLayerInfo;
		SSliceHeaderExt* pShExt=NULL;
		SSliceHeader* pSh=NULL;

		bool isNewFrame=true;
		if(pCtx->pDec==NULL){
			pCtx->pDec=PrefetchPic(pCtx->pPicBuff);
			if(pCtx->iTotalNumMbRec!=0)
				pCtx->iTotalNumMbRec=0;

			if(NULL==pCtx->pDec){
				FATAL(
						 "DecodeCurrentAccessUnit()::::::PrefetchPic ERROR,pSps->iNumRefFrames:%d.",pCtx->pSps->iNumRefFrames);
				// The error code here need to be separated from the dsOutOfMemory
				pCtx->iErrorCode|=dsOutOfMemory;
				return ERR_INFO_REF_COUNT_OVERFLOW;
			}
			pCtx->pDec->bNewSeqBegin=pCtx->bNewSeqBegin;		// set flag for start decoding
		}else
		if(pCtx->iTotalNumMbRec==0){		// pDec !=NULL,already start
			pCtx->pDec->bNewSeqBegin=pCtx->bNewSeqBegin;		// set flag for start decoding
		}
		pCtx->pDec->uiTimeStamp=pNalCur->uiTimeStamp;

		if(pCtx->iTotalNumMbRec==0){		// Picture start to decode
			for(int32_t i=0; i<LAYER_NUM_EXCHANGEABLE;++i)
				memset(pCtx->sMb.pSliceIdc[i],0xff,(pCtx->sMb.iMbWidth*pCtx->sMb.iMbHeight*sizeof(int32_t)));
			memset(pCtx->pCurDqLayer->pMbCorrectlyDecodedFlag,0,pCtx->pSps->iMbWidth*pCtx->pSps->iMbHeight*sizeof(bool));
			memset(pCtx->pCurDqLayer->pMbRefConcealedFlag,0,pCtx->pSps->iMbWidth*pCtx->pSps->iMbHeight*sizeof(bool));
			memset(pCtx->pDec->pRefPic[LIST_0],0,sizeof(SPicture*)*MAX_DPB_COUNT);
			memset(pCtx->pDec->pRefPic[LIST_1],0,sizeof(SPicture*)*MAX_DPB_COUNT);
			pCtx->pDec->iMbNum=pCtx->pSps->iMbWidth*pCtx->pSps->iMbHeight;
			pCtx->pDec->iMbEcedNum=0;
			pCtx->pDec->iMbEcedPropNum=0;
		}
		pCtx->bRPLRError=false;
		GetI4LumaIChromaAddrTable(pCtx->iDecBlockOffsetArray,pCtx->pDec->iLinesize[0],pCtx->pDec->iLinesize[1]);

		if(pNalCur->sNalHeaderExt.uiLayerDqId>kuiTargetLayerDqId){		// confirmed pNalCur will never be NULL
			break;		// Per formance it need not to decode the remaining bits any more due to given uiLayerDqId required,9/2/2009
		}

		memset(&pLayerInfo,0,sizeof(SLayerInfo));

		// Loop decoding for slices (even FMO and/ multiple slices) within a dq layer
		while(iIdx<=iEndIdx){
			bool bReconstructSlice;
			iCurrIdQ=pNalCur->sNalHeaderExt.uiQualityId;
			iCurrIdD=pNalCur->sNalHeaderExt.uiDependencyId;
			pSh=&pNalCur->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader;
			pShExt=&pNalCur->sNalData.sVclNal.sSliceHeaderExt;
			pCtx->bRPLRError=false;
			bReconstructSlice=CheckSliceNeedReconstruct(pNalCur->sNalHeaderExt.uiLayerDqId,kuiTargetLayerDqId);

			memcpy(&pLayerInfo.sNalHeaderExt,&pNalCur->sNalHeaderExt,sizeof(SNalUnitHeaderExt));

			pCtx->pDec->iFrameNum=pSh->iFrameNum;
			pCtx->pDec->iFramePoc=pSh->iPicOrderCntLsb;		// still can not obtain correct,because current do not support POCtype 2
			pCtx->pDec->bIdrFlag=pNalCur->sNalHeaderExt.bIdrFlag;
			pCtx->pDec->eSliceType=pSh->eSliceType;

			memcpy(&pLayerInfo.sSliceInLayer.sSliceHeaderExt,pShExt,sizeof(SSliceHeaderExt));
			pLayerInfo.sSliceInLayer.bSliceHeaderExtFlag=pNalCur->sNalData.sVclNal.bSliceHeaderExtFlag;
			pLayerInfo.sSliceInLayer.eSliceType=pSh->eSliceType;
			pLayerInfo.sSliceInLayer.iLastMbQp=pSh->iSliceQp;
			dq_cur->pBitStringAux=&pNalCur->sNalData.sVclNal.sSliceBitsRead;
			pCtx->uiNalRefIdc=pNalCur->sNalHeaderExt.sNalUnitHeader.uiNalRefIdc;

			iPpsId=pSh->iPpsId;
			pLayerInfo.pPps=pSh->pPps;
			pLayerInfo.pSps=pSh->pSps;
			pLayerInfo.pSubsetSps=pShExt->pSubsetSps;
			pCtx->pFmo=&pCtx->sFmoList[iPpsId];
			iRet=FmoParamUpdate(pCtx->pFmo,pLayerInfo.pSps,pLayerInfo.pPps,&pCtx->iActiveFmoNum);
			if(ERR_NONE!=iRet){
				if(iRet==ERR_INFO_OUT_OF_MEMORY){
					pCtx->iErrorCode|=dsOutOfMemory;
					FATAL("DecodeCurrentAccessUnit(),Fmo param alloc failed");
				}else{
					pCtx->iErrorCode|=dsBitstreamError;
					uprintf("DecodeCurrentAccessUnit(),FmoParamUpdate failed,eSliceType: %d.",pSh->eSliceType);
				}
				return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_FMO_INIT_FAIL);
			}
			bFreshSliceAvailable=(iCurrIdD!=iLastIdD || iCurrIdQ!=iLastIdQ);		// do not need condition of (first_mb==0) due multiple slices might be disorder
			WelsDqLayerDecodeStart(pCtx,pNalCur,pLayerInfo.pSps,pLayerInfo.pPps);
			if((iLastIdD<0) || (iLastIdD==iCurrIdD)){
				InitDqLayerInfo(dq_cur,&pLayerInfo,pNalCur,pCtx->pDec);
				if(!dq_cur->sLayerInfo.pSps->bGapsInFrameNumValueAllowedFlag){
					const bool kbIdrFlag=dq_cur->sLayerInfo.sNalHeaderExt.bIdrFlag || (dq_cur->sLayerInfo.sNalHeaderExt.sNalUnitHeader.eNalUnitType==NAL_UNIT_CODED_SLICE_IDR);
					// Subclause 8.2.5.2 Decoding process for gaps in frame_num
					int32_t iPrevFrameNum=pCtx->pLastDecPicInfo->iPrevFrameNum;
					if(!kbIdrFlag && pSh->iFrameNum!=iPrevFrameNum && pSh->iFrameNum!=((iPrevFrameNum+1)&((1<<dq_cur->sLayerInfo.pSps->uiLog2MaxFrameNum)-1))){
						uprintf("referencing pictures lost due frame gaps exist,prev_frame_num: %d,curr_frame_num: %d",iPrevFrameNum,pSh->iFrameNum);
						bAllRefComplete=false;
						pCtx->iErrorCode|=dsRefLost;
						if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
							pCtx->bParamSetsLostFlag=true;
							return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_REFERENCE_PIC_LOST);
						}
					}
				}
				if(iCurrIdD==kuiDependencyIdMax && iCurrIdQ==BASE_QUALITY_ID && isNewFrame){
					iRet=InitRefPicList(pCtx,pCtx->uiNalRefIdc,pSh->iPicOrderCntLsb);
					if(iRet){
						pCtx->bRPLRError=true;
						bAllRefComplete=false;		// RPLR error,set ref pictures complete flag false
						HandleReferenceLost(pCtx,pNalCur);
						uprintf("reference picture introduced by this frame is lost during transmission! uiTId: %d",pNalCur->sNalHeaderExt.uiTemporalId);
						if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
							if(pCtx->iTotalNumMbRec==0)
								pCtx->pDec=NULL;
							return iRet;
						}
					}
				}
				// calculate Colocated mv scaling factor for temporal direct prediction
				if(pSh->eSliceType==B_SLICE && !pSh->iDirectSpatialMvPredFlag)
					ComputeColocatedTemporalScaling(pCtx);
				iRet=WelsDecodeSlice(pCtx,bFreshSliceAvailable,pNalCur);
				// uprintf("pCtx->eSliceType %d frameNum %d\n",pCtx->eSliceType,pCtx->iFrameNum);
				// Output good store_base reconstruction when enhancement quality layer occurred error for MGS key picture case
				if(iRet!=ERR_NONE){
					uprintf("DecodeCurrentAccessUnit() failed (%d) in frame: %d uiDId: %d uiQId: %d",iRet,pSh->iFrameNum,iCurrIdD,iCurrIdQ);
					bAllRefComplete=false;
					HandleReferenceLostL0(pCtx,pNalCur);
					if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
						if(pCtx->iTotalNumMbRec==0)
							pCtx->pDec=NULL;
						return iRet;
					}
				}

				if(bReconstructSlice){
					if((iRet=WelsDecodeConstructSlice(pCtx,pNalCur))!=ERR_NONE){
						pCtx->pDec->bIsComplete=false;		// reconstruction error,directly set the flag false
						return iRet;
					}
				}
				if(bAllRefComplete && pCtx->eSliceType!=I_SLICE){
					if(pCtx->sRefPic.uiRefCount[LIST_0]>0){
						bAllRefComplete&=CheckRefPicturesComplete(pCtx);
					}else{
						bAllRefComplete=false;
					}
				}
			}
			//uprintf("cur_frame : %d\tiCurrIdD : %d\n ",dq_cur->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.iFrameNum,iCurrIdD);
			iLastIdD=iCurrIdD;
			iLastIdQ=iCurrIdQ;
			// pNalUnitsList overflow.
			++iIdx;
			if(iIdx<=iEndIdx){
				pNalCur=pCurAu->pNalUnitsList[iIdx];
			}else{
				pNalCur=NULL;
			}
			if(pNalCur==NULL || iLastIdD!=pNalCur->sNalHeaderExt.uiDependencyId || iLastIdQ!=pNalCur->sNalHeaderExt.uiQualityId)
				break;
		}

		// Set the current dec picture complete flag. The flag will be reset when current picture need do ErrorCon.
		pCtx->pDec->bIsComplete=bAllRefComplete;
		if(!pCtx->pDec->bIsComplete){		// Ref pictures ECed,result in ECed
			pCtx->iErrorCode|=dsDataErrorConcealed;
		}

		// A dq layer decoded here
		//uprintf("POC: #%d,FRAME: #%d,D: %d,Q: %d,T: %d,P: %d,%d\n",pSh->iPicOrderCntLsb,pSh->iFrameNum,iCurrIdD,iCurrIdQ,dq_cur->sLayerInfo.sNalHeaderExt.uiTemporalId,dq_cur->sLayerInfo.sNalHeaderExt.uiPriorityId,dq_cur->sLayerInfo.sSliceInLayer.sSliceHeaderExt.sSliceHeader.iSliceQp);
		if(dq_cur->uiLayerDqId==kuiTargetLayerDqId){
			if(!pCtx->bInstantDecFlag){
				// Do error concealment here
				if((NeedErrorCon(pCtx)) && (pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE)){
					ImplementErrorCon(pCtx);
					pCtx->iTotalNumMbRec=pCtx->pSps->iMbWidth*pCtx->pSps->iMbHeight;
					pCtx->pDec->iSpsId=pCtx->pSps->iSpsId;
					pCtx->pDec->iPpsId=pCtx->pPps->iPpsId;
				}
			}

			iRet=DecodeFrameConstruction(pCtx,ppDst,pDstInfo);
			if(iRet){
				return iRet;
			}

			pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb=pCtx->pDec;		// store latest decoded picture for EC
			pCtx->bUsedAsRef=pCtx->uiNalRefIdc>0;
			if(pCtx->bUsedAsRef){
				for(int32_t listIdx=LIST_0; listIdx<LIST_A;++listIdx){
					uint32_t i=0;
					while(i<MAX_DPB_COUNT && pCtx->sRefPic.pRefList[listIdx][i]){
						pCtx->pDec->pRefPic[listIdx][i]=pCtx->sRefPic.pRefList[listIdx][i];
						++i;
					}
				}
				iRet=WelsMarkAsRef(pCtx);
				if(iRet!=ERR_NONE){
					if(iRet==ERR_INFO_DUPLICATE_FRAME_NUM)
						pCtx->iErrorCode|=dsBitstreamError;
					if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
						pCtx->pDec=NULL;
						return iRet;
					}
				}
				ExpandReferencingPicture(pCtx->pDec->pData,pCtx->pDec->iWidthInPixel,pCtx->pDec->iHeightInPixel,pCtx->pDec->iLinesize,pCtx->sExpandPicFunc.pfExpandLumaPicture,pCtx->sExpandPicFunc.pfExpandChromaPicture);
			}
			pCtx->pDec=NULL;		// after frame decoding,always set to NULL
		}

		// need update frame_num due current frame is well decoded
		if(pCurAu->pNalUnitsList[pCurAu->uiStartPos]->sNalHeaderExt.sNalUnitHeader.uiNalRefIdc>0)
			pCtx->pLastDecPicInfo->iPrevFrameNum=pSh->iFrameNum;
		if(pCtx->pLastDecPicInfo->bLastHasMmco5)
			pCtx->pLastDecPicInfo->iPrevFrameNum=0;
	}
	return ERR_NONE;
}

void ResetCurrentAccessUnit(SDecoderContext* pCtx){
	SAccessUnit* pCurAu=pCtx->pAccessUnitList;
	pCurAu->uiStartPos=0;
	pCurAu->uiEndPos=0;
	pCurAu->bCompletedAuFlag=false;
	if(pCurAu->uiActualUnitsNum>0){
		uint32_t iIdx=0;
		const uint32_t kuiActualNum=pCurAu->uiActualUnitsNum;
		// a more simpler method to do nal units list management prefered here
		const uint32_t kuiAvailNum=pCurAu->uiAvailUnitsNum;
		const uint32_t kuiLeftNum=kuiAvailNum-kuiActualNum;

		// Swapping active nal unit nodes of succeeding AU with leading of list
		while(iIdx<kuiLeftNum){
			SNalUnit* t=pCurAu->pNalUnitsList[kuiActualNum+iIdx];
			pCurAu->pNalUnitsList[kuiActualNum+iIdx]=pCurAu->pNalUnitsList[iIdx];
			pCurAu->pNalUnitsList[iIdx]=t;
			++iIdx;
		}
		pCurAu->uiActualUnitsNum=pCurAu->uiAvailUnitsNum=kuiLeftNum;
	}
}

void WelsDecodeAccessUnitEnd(SDecoderContext* pCtx){
	// save previous header info
	SAccessUnit* pCurAu=pCtx->pAccessUnitList;
	SNalUnit* pCurNal=pCurAu->pNalUnitsList[pCurAu->uiEndPos];
	memcpy(&pCtx->pLastDecPicInfo->sLastNalHdrExt,&pCurNal->sNalHeaderExt,sizeof(SNalUnitHeaderExt));
	memcpy(&pCtx->pLastDecPicInfo->sLastSliceHeader,&pCurNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader,sizeof(SSliceHeader));
	// uninitialize context of current access unit and rbsp buffer clean
	ResetCurrentAccessUnit(pCtx);
}

// brief Enumerate return type
typedef enum{
	cmResultSuccess,	// successful
	cmInitParaError,	// parameters are invalid
	cmUnknownReason,
	cmMallocMemeError,	// malloc a memory error
	cmInitExpected,	// initial action is expected
	cmUnsupportedData
} CM_RETURN;

PlainH264Decoder::PlainH264Decoder(void) :
	m_uiDecodeTimeStamp(0),
	m_pPicBuff(NULL),
	m_bParamSetsLostFlag(false),
	m_bFreezeOutput(false),
	m_DecCtxActiveCount(0),
	m_iLastBufferedIdx(0){
	m_pCtx=0;
	ResetReorderingPictureBuffers(&m_sReoderingStatus,m_sPictInfoList,true);
}

// Description:
// class PlainH264Decoder destructor function,destroy allocced memory
// Input parameters: none
// return: none
PlainH264Decoder::~PlainH264Decoder(){
	UninitDecoder();
}

long PlainH264Decoder::Initialize(const SDecodingParam* pParam){
	int iRet=ERR_NONE;
	if(pParam==NULL){
		FATAL("PlainH264Decoder::Initialize(),invalid input argument.");
		return cmInitParaError;
	}

	// H.264 decoder initialization,including memory allocation,then open it ready to decode
	iRet=InitDecoder(pParam);
	if(iRet)
		return iRet;

	return cmResultSuccess;
}

long PlainH264Decoder::Uninitialize(){
	UninitDecoder();

	return ERR_NONE;
}

void WelsResetRefPicWithoutUnRef(SDecoderContext* pCtx){
	int32_t i=0;
	SRefPic* pRefPic=&pCtx->sRefPic;
	pCtx->sRefPic.uiLongRefCount[LIST_0]=pCtx->sRefPic.uiShortRefCount[LIST_0]=0;

	pRefPic->uiRefCount[LIST_0]=0;
	pRefPic->uiRefCount[LIST_1]=0;

	for(i=0; i<MAX_DPB_COUNT; i++){
		pRefPic->pShortRefList[LIST_0][i]=NULL;
	}
	pRefPic->uiShortRefCount[LIST_0]=0;

	for(i=0; i<MAX_DPB_COUNT; i++){
		pRefPic->pLongRefList[LIST_0][i]=NULL;
	}
	pRefPic->uiLongRefCount[LIST_0]=0;
}

void PlainH264Decoder::UninitDecoder(void){
	WelsResetRefPicWithoutUnRef(m_pCtx);
	UninitDecoderCtx(m_pCtx);
}

// brief Uninitialize Wels Flexible Macroblock Ordering (FMO) list
// param pFmo Wels base fmo ptr to be uninitialized
// param kiCnt count number of PPS per list
// param kiAvail count available number of PPS in list
// return NONE
void UninitFmoList(SFmo* pFmo,const int32_t kiCnt,const int32_t kiAvail){
	SFmo* pIter=pFmo;
	int32_t i=0;
	int32_t iFreeNodes=0;

	if(NULL==pIter || kiAvail<=0 || kiCnt<kiAvail)
		return;

	while(i<kiCnt){
		if(pIter!=NULL && pIter->bActiveFlag){
			if(NULL!=pIter->pMbAllocMap){
				WelsFree(pIter->pMbAllocMap);
				pIter->pMbAllocMap=NULL;
			}
			pIter->iSliceGroupCount=0;
			pIter->iSliceGroupType=-1;
			pIter->iCountMbNum=0;
			pIter->bActiveFlag=false;
			++iFreeNodes;
			if(iFreeNodes>=kiAvail)
				break;
		}
		++pIter;
		++i;
	}
}

// brief reset fmo list due to got Sps now
// param pCtx decoder context
// return count number of fmo context units are reset
int32_t ResetFmoList(SDecoderContext* pCtx){
	int32_t iCountNum=0;
	if(NULL!=pCtx){
		// Fixed memory leak due to PPS_ID might not be continuous sometimes,1/5/2010
		UninitFmoList(&pCtx->sFmoList[0],MAX_PPS_COUNT,pCtx->iActiveFmoNum);
		iCountNum=pCtx->iActiveFmoNum;
		pCtx->iActiveFmoNum=0;
	}
	return iCountNum;
}




// free memory dynamically allocated during decoder
void WelsFreeDynamicMemory(SDecoderContext* pCtx){

	// free dq layer memory
	UninitialDqLayersContext(pCtx);

	// free FMO memory
	ResetFmoList(pCtx);

	// free ref-pic list & picture memory
	WelsResetRefPic(pCtx);

	SPicBuff** pPicBuff=&pCtx->pPicBuff;
	if(NULL!=pPicBuff && NULL!=*pPicBuff){
		DestroyPicBuff(pCtx,pPicBuff);
	}

	if(pCtx->pTempDec){
		FreePicture(pCtx->pTempDec);
		pCtx->pTempDec=NULL;
	}

	// added for safe memory
	pCtx->iImgWidthInPixel=0;
	pCtx->iImgHeightInPixel=0;
	pCtx->iLastImgWidthInPixel=0;
	pCtx->iLastImgHeightInPixel=0;
	pCtx->bFreezeOutput=true;
	pCtx->bHaveGotMemory=false;

	// free CABAC memory
	WelsFree(pCtx->pCabacDecEngine);
}

// WelsFreeStaticMemory
// Free memory introduced in WelsInitStaticMemory at destruction of decoder.
void WelsFreeStaticMemory(SDecoderContext* pCtx){
	if(pCtx==NULL)
		return;


	MemFreeNalList(&pCtx->pAccessUnitList);

	if(pCtx->sRawData.pHead){
		WelsFree(pCtx->sRawData.pHead);
	}
	pCtx->sRawData.pHead=NULL;
	pCtx->sRawData.pEnd=NULL;
	pCtx->sRawData.pStartPos=NULL;
	pCtx->sRawData.pCurPos=NULL;
	if(NULL!=pCtx->pParam){
		WelsFree(pCtx->pParam);
		pCtx->pParam=NULL;
	}
}

// brief Close decoder
void WelsCloseDecoder(SDecoderContext* pCtx){
	WelsFreeDynamicMemory(pCtx);

	WelsFreeStaticMemory(pCtx);

	pCtx->bParamSetsLostFlag=false;
	pCtx->bNewSeqBegin=false;
	pCtx->bPrintFrameErrorTraceFlag=false;
}

// brief Uninitialize Wels decoder parameters and memory
// param pCtx input context to be uninitialized at release stage
// return NONE
void WelsEndDecoder(SDecoderContext* pCtx){
	// close decoder
	WelsCloseDecoder(pCtx);
}

#define VERSION_NUMBER "openh264 default: 1.4"

void PlainH264Decoder::UninitDecoderCtx(SDecoderContext* pCtx){
	if(pCtx!=NULL){
		uprintf("PlainH264Decoder::UninitDecoderCtx(),openh264 codec version=%s.",VERSION_NUMBER);
		WelsEndDecoder(pCtx);
		if(NULL!=pCtx){
			WelsFree(pCtx);
			pCtx=NULL;
		}
		m_pCtx=NULL;
	}
}

// fill last decoded picture info
void WelsDecoderLastDecPicInfoDefaults(SWelsLastDecPicInfo& sLastDecPicInfo){
	sLastDecPicInfo.iPrevPicOrderCntMsb=0;
	sLastDecPicInfo.iPrevPicOrderCntLsb=0;
	sLastDecPicInfo.pPreviousDecodedPictureInDpb=NULL;
	sLastDecPicInfo.iPrevFrameNum=-1;
	sLastDecPicInfo.bLastHasMmco5=false;
}

// the return value of this function is not suitable,it need report failure info to upper layer.
int32_t PlainH264Decoder::InitDecoder(const SDecodingParam* pParam){
	uprintf("PlainH264Decoder::init_decoder(),openh264 codec version=%s",VERSION_NUMBER);
	memset(&m_sLastDecPicInfo,0,sizeof(SWelsLastDecPicInfo));
	memset(&m_sVlcTable,0,sizeof(SVlcTable));
	// UninitDecoder();
	WelsDecoderLastDecPicInfoDefaults(m_sLastDecPicInfo);
	InitDecoderCtx(pParam);
	m_bParamSetsLostFlag=false;
	m_bFreezeOutput=false;
	return cmResultSuccess;
}

// fill data fields in default for decoder context
void WelsDecoderDefaults(SDecoderContext* pCtx){
	//int32_t iCpuCores=1;
	pCtx->pArgDec=NULL;

	pCtx->bHaveGotMemory=false;				// not ever request memory blocks for decoder context related
	pCtx->uiCpuFlag=0;

	pCtx->bAuReadyFlag=0; 					// au data is not ready
	pCtx->bCabacInited=false;

	pCtx->uiCpuFlag=0;						// WelsCPUFeatureDetect (&iCpuCores);

	pCtx->iImgWidthInPixel=0;
	pCtx->iImgHeightInPixel=0;				// alloc picture data when picture size is available
	pCtx->iLastImgWidthInPixel=0;
	pCtx->iLastImgHeightInPixel=0;
	pCtx->bFreezeOutput=true;

	pCtx->iFrameNum=-1;
	pCtx->pLastDecPicInfo->iPrevFrameNum=-1;
	pCtx->iErrorCode=ERR_NONE;

	pCtx->pDec=NULL;

	pCtx->pTempDec=NULL;

	WelsResetRefPic(pCtx);

	pCtx->iActiveFmoNum=0;

	pCtx->pPicBuff=NULL;

	pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb=NULL;
	pCtx->bUseScalingList=false;
	pCtx->iFeedbackNalRefIdc=-1;		// initialize
	pCtx->pLastDecPicInfo->iPrevPicOrderCntMsb=0;
	pCtx->pLastDecPicInfo->iPrevPicOrderCntLsb=0;

}

// fill data fields in SPS and PPS default for decoder context
void WelsDecoderSpsPpsDefaults(SWelsDecoderSpsPpsCTX& sSpsPpsCtx){
	sSpsPpsCtx.bSpsExistAheadFlag=false;
	sSpsPpsCtx.bSubspsExistAheadFlag=false;
	sSpsPpsCtx.bPpsExistAheadFlag=false;
	sSpsPpsCtx.bAvcBasedFlag=true;
	sSpsPpsCtx.iSpsErrorIgnored=0;
	sSpsPpsCtx.iSubSpsErrorIgnored=0;
	sSpsPpsCtx.iPpsErrorIgnored=0;
	sSpsPpsCtx.iPPSInvalidNum=0;
	sSpsPpsCtx.iPPSLastInvalidId=-1;
	sSpsPpsCtx.iSPSInvalidNum=0;
	sSpsPpsCtx.iSPSLastInvalidId=-1;
	sSpsPpsCtx.iSubSPSInvalidNum=0;
	sSpsPpsCtx.iSubSPSLastInvalidId=-1;
	sSpsPpsCtx.iSeqId=-1;
}

// Copy functions
void WelsCopy4x4_c(uint8_t* pDst,int32_t iStrideD,uint8_t* pSrc,int32_t iStrideS){
	const int32_t kiSrcStride2=iStrideS<<1;
	const int32_t kiSrcStride3=iStrideS+kiSrcStride2;
	const int32_t kiDstStride2=iStrideD<<1;
	const int32_t kiDstStride3=iStrideD+kiDstStride2;
	ST32(pDst,LD32(pSrc));
	ST32(pDst+iStrideD,LD32(pSrc+iStrideS));
	ST32(pDst+kiDstStride2,LD32(pSrc+kiSrcStride2));
	ST32(pDst+kiDstStride3,LD32(pSrc+kiSrcStride3));
}
void WelsCopy8x4_c(uint8_t* pDst,int32_t iStrideD,uint8_t* pSrc,int32_t iStrideS){
	WelsCopy4x4_c(pDst,iStrideD,pSrc,iStrideS);
	WelsCopy4x4_c(pDst+4,iStrideD,pSrc+4,iStrideS);
}
void WelsCopy4x8_c(uint8_t* pDst,int32_t iStrideD,uint8_t* pSrc,int32_t iStrideS){
	WelsCopy4x4_c(pDst,iStrideD,pSrc,iStrideS);
	WelsCopy4x4_c(pDst+(iStrideD<<2),iStrideD,pSrc+(iStrideS<<2),iStrideS);
}
void WelsCopy8x8_c(uint8_t* pDst,int32_t iStrideD,uint8_t* pSrc,int32_t iStrideS){
	int32_t i;
	for(i=0; i<4; i++){
		ST32(pDst,LD32(pSrc));
		ST32(pDst+4,LD32(pSrc+4));
		ST32(pDst+iStrideD,LD32(pSrc+iStrideS));
		ST32(pDst+iStrideD+4,LD32(pSrc+iStrideS+4));
		pDst+=iStrideD<<1;
		pSrc+=iStrideS<<1;
	}
}
void WelsCopy8x16_c(uint8_t* pDst,int32_t iStrideD,uint8_t* pSrc,int32_t iStrideS){
	int32_t i;
	for(i=0; i<8;++i){
		ST32(pDst,LD32(pSrc));
		ST32(pDst+4,LD32(pSrc+4));
		ST32(pDst+iStrideD,LD32(pSrc+iStrideS));
		ST32(pDst+iStrideD+4,LD32(pSrc+iStrideS+4));
		pDst+=iStrideD<<1;
		pSrc+=iStrideS<<1;
	}
}
void WelsCopy16x8_c(uint8_t* pDst,int32_t iStrideD,uint8_t* pSrc,int32_t iStrideS){
	int32_t i;
	for(i=0; i<8; i++){
		ST32(pDst,LD32(pSrc));
		ST32(pDst+4,LD32(pSrc+4));
		ST32(pDst+8,LD32(pSrc+8));
		ST32(pDst+12,LD32(pSrc+12));
		pDst+=iStrideD;
		pSrc+=iStrideS;
	}
}
void WelsCopy16x16_c(uint8_t* pDst,int32_t iStrideD,uint8_t* pSrc,int32_t iStrideS){
	int32_t i;
	for(i=0; i<16; i++){
		ST32(pDst,LD32(pSrc));
		ST32(pDst+4,LD32(pSrc+4));
		ST32(pDst+8,LD32(pSrc+8));
		ST32(pDst+12,LD32(pSrc+12));
		pDst+=iStrideD;
		pSrc+=iStrideS;
	}
}


void InitErrorCon(SDecoderContext* pCtx){
	if((pCtx->pParam->eEcActiveIdc==ERROR_CON_SLICE_COPY)
		 || (pCtx->pParam->eEcActiveIdc==ERROR_CON_SLICE_COPY_CROSS_IDR)
		 || (pCtx->pParam->eEcActiveIdc==ERROR_CON_SLICE_MV_COPY_CROSS_IDR)
		 || (pCtx->pParam->eEcActiveIdc==ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE)
		 || (pCtx->pParam->eEcActiveIdc==ERROR_CON_SLICE_COPY_CROSS_IDR_FREEZE_RES_CHANGE)){
		if((pCtx->pParam->eEcActiveIdc!=ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE)
			 && (pCtx->pParam->eEcActiveIdc!=ERROR_CON_SLICE_COPY_CROSS_IDR_FREEZE_RES_CHANGE)){
			pCtx->bFreezeOutput=false;
		}
		pCtx->sCopyFunc.pCopyLumaFunc=WelsCopy16x16_c;
		pCtx->sCopyFunc.pCopyChromaFunc=WelsCopy8x8_c;

#if defined(X86_ASM)
		if(pCtx->uiCpuFlag&WELS_CPU_MMXEXT){
			pCtx->sCopyFunc.pCopyChromaFunc=WelsCopy8x8_mmx;		// aligned
		}

		if(pCtx->uiCpuFlag&WELS_CPU_SSE2){
			pCtx->sCopyFunc.pCopyLumaFunc=WelsCopy16x16_sse2;		// this is aligned copy;
		}
#endif		// X86_ASM

#if defined(HAVE_NEON)
		if(pCtx->uiCpuFlag&WELS_CPU_NEON){
			pCtx->sCopyFunc.pCopyLumaFunc=WelsCopy16x16_neon;		// aligned
			pCtx->sCopyFunc.pCopyChromaFunc=WelsCopy8x8_neon;		// aligned
		}
#endif		// HAVE_NEON

#if defined(HAVE_NEON_AARCH64)
		if(pCtx->uiCpuFlag&WELS_CPU_NEON){
			pCtx->sCopyFunc.pCopyLumaFunc=WelsCopy16x16_AArch64_neon;		// aligned
			pCtx->sCopyFunc.pCopyChromaFunc=WelsCopy8x8_AArch64_neon;		// aligned
		}
#endif		// HAVE_NEON_AARCH64
	}		// TODO add more methods here
	return;
}

// brief configure decoder parameters
int32_t DecoderConfigParam(SDecoderContext* pCtx,const SDecodingParam* kpParam){
	if(NULL==pCtx || NULL==kpParam)
		return ERR_INFO_INVALID_PARAM;
	memcpy(pCtx->pParam,kpParam,sizeof(SDecodingParam));
	if((pCtx->pParam->eEcActiveIdc>ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE) || (pCtx->pParam->eEcActiveIdc<ERROR_CON_DISABLE)){
		uprintf("eErrorConMethod (%d) not in range: (%d-%d). Set as default value: (%d).",pCtx->pParam->eEcActiveIdc,ERROR_CON_DISABLE,ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE,ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE);
		pCtx->pParam->eEcActiveIdc=ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE;
	}
	InitErrorCon(pCtx);
	if(VIDEO_BITSTREAM_SVC==pCtx->pParam->sVideoProperty.eVideoBsType || VIDEO_BITSTREAM_AVC==pCtx->pParam->sVideoProperty.eVideoBsType){
		pCtx->eVideoType=pCtx->pParam->sVideoProperty.eVideoBsType;
	}else{
		pCtx->eVideoType=VIDEO_BITSTREAM_DEFAULT;
	}
	uprintf("eVideoType: %d",pCtx->eVideoType);
	return ERR_NONE;
}

void WelsNonZeroCount_c(int8_t* pNonZeroCount){
	int32_t i;
	for(i=0; i<24; i++){
		pNonZeroCount[i]=!!pNonZeroCount[i];
	}
}

void WelsBlockInit(int16_t* pBlock,int iW,int iH,int iStride,uint8_t uiVal){
	int32_t i;
	int16_t* pDst=pBlock;
	for(i=0; i<iH; i++){
		memset(pDst,uiVal,iW*sizeof(int16_t));
		pDst+=iStride;
	}
}

void WelsBlockZero16x16_c(int16_t* pBlock,int32_t iStride){
	WelsBlockInit(pBlock,16,16,iStride,0);
}

void WelsBlockZero8x8_c(int16_t* pBlock,int32_t iStride){
	WelsBlockInit(pBlock,8,8,iStride,0);
}

void WelsBlockFuncInit(SBlockFunc* pFunc,int32_t iCpu){
	pFunc->pWelsSetNonZeroCountFunc=WelsNonZeroCount_c;
	pFunc->pWelsBlockZero16x16Func=WelsBlockZero16x16_c;
	pFunc->pWelsBlockZero8x8Func=WelsBlockZero8x8_c;
}

void WelsI16x16LumaPredV_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	const uint64_t kuiTop1=LD64A8(pPred-kiStride);
	const uint64_t kuiTop2=LD64A8(pPred-kiStride+8);
	uint8_t i=15;
	do{
		ST64A8(pPred+iTmp,kuiTop1);
		ST64A8(pPred+iTmp+8,kuiTop2);
		iTmp-=kiStride;
	} while(i-->0);
}

void WelsI16x16LumaPredH_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	uint8_t i=15;

	do{
		const uint8_t kuiVal8=pPred[iTmp-1];
		const uint64_t kuiVal64=0x0101010101010101ULL*kuiVal8;

		ST64A8(pPred+iTmp,kuiVal64);
		ST64A8(pPred+iTmp+8,kuiVal64);

		iTmp-=kiStride;
	} while(i-->0);
}

#define I4x4_COUNT 4
#define I8x8_COUNT 8
#define I16x16_COUNT 16

void WelsI16x16LumaPredDc_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t uiMean=0;

	// caculate the kMean value
	do{
		iSum+=pPred[-1+iTmp]+pPred[-kiStride+i];
		iTmp-=kiStride;
	} while(i-->0);
	uiMean=(16+iSum)>>5;
	iTmp=(kiStride<<4)-kiStride;
	i=15;
	do{
		memset(&pPred[iTmp],uiMean,I16x16_COUNT);
		iTmp-=kiStride;
	} while(i-->0);
}


void WelsI16x16LumaPredPlane_c(uint8_t* pPred,const int32_t kiStride){
	int32_t a=0,b=0,c=0,H=0,V=0;
	int32_t i,j;
	uint8_t* pTop=&pPred[-kiStride];
	uint8_t* pLeft=&pPred[-1];
	for(i=0; i<8; i++){
		H+=(i+1)*(pTop[8+i]-pTop[6-i]);
		V+=(i+1)*(pLeft[(8+i)*kiStride]-pLeft[(6-i)*kiStride]);
	}
	a=(pLeft[15*kiStride]+pTop[15])<<4;
	b=(5*H+32)>>6;
	c=(5*V+32)>>6;
	for(i=0; i<16; i++){
		for(j=0; j<16; j++){
			int32_t iTmp=(a+b*(j-7)+c*(i-7)+16)>>5;
			iTmp=WelsClip1(iTmp);
			pPred[j]=iTmp;
		}
		pPred+=kiStride;
	}
}

void WelsI16x16LumaPredDcLeft_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint64_t uiMean64=0;
	uint8_t uiMean=0;
	uint8_t i=15;
	// caculate the kMean value
	do{
		iSum+=pPred[-1+iTmp];
		iTmp-=kiStride;
	} while(i-->0);
	uiMean=(8+iSum)>>4;
	uiMean64=0x0101010101010101ULL*uiMean;
	iTmp=(kiStride<<4)-kiStride;
	i=15;
	do{
		ST64A8(pPred+iTmp,uiMean64);
		ST64A8(pPred+iTmp+8,uiMean64);
		iTmp-=kiStride;
	} while(i-->0);
}

void WelsI16x16LumaPredDcTop_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t uiMean=0;
	// caculate the kMean value
	do{
		iSum+=pPred[-kiStride+i];
	} while(i-->0);
	uiMean=(8+iSum)>>4;
	i=15;
	do{
		memset(&pPred[iTmp],uiMean,I16x16_COUNT);
		iTmp-=kiStride;
	} while(i-->0);
}

void WelsI16x16LumaPredDcNA_c(uint8_t* pPred,const int32_t kiStride){
	const uint64_t kuiDC64=0x8080808080808080ULL;
	int32_t iTmp=(kiStride<<4)-kiStride;
	uint8_t i=15;
	do{
		ST64A8(pPred+iTmp,kuiDC64);
		ST64A8(pPred+iTmp+8,kuiDC64);

		iTmp-=kiStride;
	} while(i-->0);
}

typedef void (*PFillingPred) (uint8_t* pPred,uint8_t* pSrc);
typedef void (*PFillingPred1to16) (uint8_t* pPred,const uint8_t kuiSrc);

static inline void WelsFillingPred8to16_c(uint8_t* pPred,uint8_t* pSrc){
	ST64(pPred,LD64(pSrc));
	ST64(pPred+8,LD64(pSrc));
}
static inline void WelsFillingPred8x2to16_c(uint8_t* pPred,uint8_t* pSrc){
	ST64(pPred,LD64(pSrc));
	ST64(pPred+8,LD64(pSrc+8));
}
static inline void WelsFillingPred1to16_c(uint8_t* pPred,const uint8_t kuiSrc){
	const uint8_t kuiSrc8[8]={kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc,kuiSrc};
	ST64(pPred,LD64(kuiSrc8));
	ST64(pPred+8,LD64(kuiSrc8));
}

#define WelsFillingPred8to16 WelsFillingPred8to16_c
#define WelsFillingPred8x2to16 WelsFillingPred8x2to16_c
#define WelsFillingPred1to16 WelsFillingPred1to16_c

void WelsIChromaPredDc_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiL1=kiStride-1;
	const int32_t kiL2=kiL1+kiStride;
	const int32_t kiL3=kiL2+kiStride;
	const int32_t kiL4=kiL3+kiStride;
	const int32_t kiL5=kiL4+kiStride;
	const int32_t kiL6=kiL5+kiStride;
	const int32_t kiL7=kiL6+kiStride;
	// caculate the kMean value
	const uint8_t kuiM1=(pPred[-kiStride]+pPred[1-kiStride]+pPred[2-kiStride]+pPred[3-kiStride]+pPred[-1]+pPred[kiL1]+pPred[kiL2]+pPred[kiL3]+4)>>3;
	const uint32_t kuiSum2=pPred[4-kiStride]+pPred[5-kiStride]+pPred[6-kiStride]+pPred[7-kiStride];
	const uint32_t kuiSum3=pPred[kiL4]+pPred[kiL5]+pPred[kiL6]+pPred[kiL7];
	const uint8_t kuiM2=(kuiSum2+2)>>2;
	const uint8_t kuiM3=(kuiSum3+2)>>2;
	const uint8_t kuiM4=(kuiSum2+kuiSum3+4)>>3;
	const uint8_t kuiMUP[8]={kuiM1,kuiM1,kuiM1,kuiM1,kuiM2,kuiM2,kuiM2,kuiM2};
	const uint8_t kuiMDown[8]={kuiM3,kuiM3,kuiM3,kuiM3,kuiM4,kuiM4,kuiM4,kuiM4};
	const uint64_t kuiUP64=LD64(kuiMUP);
	const uint64_t kuiDN64=LD64(kuiMDown);
	ST64A8(pPred,kuiUP64);
	ST64A8(pPred+kiL1+1,kuiUP64);
	ST64A8(pPred+kiL2+1,kuiUP64);
	ST64A8(pPred+kiL3+1,kuiUP64);
	ST64A8(pPred+kiL4+1,kuiDN64);
	ST64A8(pPred+kiL5+1,kuiDN64);
	ST64A8(pPred+kiL6+1,kuiDN64);
	ST64A8(pPred+kiL7+1,kuiDN64);
}

void WelsI16x16LumaPredPlane_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	int32_t iLTshift=0,iTopshift=0,iLeftshift=0,iTopSum=0,iLeftSum=0;
	int32_t i,j;
	uint8_t* pTop=&pRef[-kiStride];
	uint8_t* pLeft=&pRef[-1];
	int32_t iPredStride=16;
	for(i=0; i<8; i++){
		iTopSum+=(i+1)*(pTop[8+i]-pTop[6-i]);
		iLeftSum+=(i+1)*(pLeft[(8+i)*kiStride]-pLeft[(6-i)*kiStride]);
	}
	iLTshift=(pLeft[15*kiStride]+pTop[15])<<4;
	iTopshift=(5*iTopSum+32)>>6;
	iLeftshift=(5*iLeftSum+32)>>6;
	for(i=0; i<16; i++){
		for(j=0; j<16; j++){
			pPred[j]=WelsClip1((iLTshift+iTopshift*(j-7)+iLeftshift*(i-7)+16)>>5);
		}
		pPred+=iPredStride;
	}
}

void WelsI16x16LumaPredDc_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	int32_t iStridex15=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t iMean=0;
	// caculate the iMean value
	do{
		iSum+=pRef[-1+iStridex15]+pRef[-kiStride+i];
		iStridex15-=kiStride;
	} while(i-->0);
	iMean=(16+iSum)>>5;
	memset(pPred,iMean,256);
}


void WelsI16x16LumaPredDcTop_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t iMean=0;
	// caculate the iMean value
	do{
		iSum+=pRef[-kiStride+i];
	} while(i-->0);
	iMean=(8+iSum)>>4;
	memset(pPred,iMean,256);
}

void WelsI16x16LumaPredDcLeft_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	int32_t iStridex15=(kiStride<<4)-kiStride;
	int32_t iSum=0;
	uint8_t i=15;
	uint8_t iMean=0;
	// caculate the iMean value
	do{
		iSum+=pRef[-1+iStridex15];
		iStridex15-=kiStride;
	} while(i-->0);
	iMean=(8+iSum)>>4;
	memset(pPred,iMean,256);
}

void WelsI16x16LumaPredDcNA_c(uint8_t* pPred,uint8_t* pRef,const int32_t kiStride){
	memset(pPred,0x80,256);
}

void WelsI4x4LumaPredV_c(uint8_t* pPred,const int32_t kiStride){
	const uint32_t kuiVal=LD32A4(pPred-kiStride);
	ST32A4(pPred,kuiVal);
	ST32A4(pPred+kiStride,kuiVal);
	ST32A4(pPred+(kiStride<<1),kuiVal);
	ST32A4(pPred+(kiStride<<1)+kiStride,kuiVal);
}

void WelsI4x4LumaPredH_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride2+kiStride;
	const uint32_t kuiL0=0x01010101U*pPred[-1];
	const uint32_t kuiL1=0x01010101U*pPred[-1+kiStride];
	const uint32_t kuiL2=0x01010101U*pPred[-1+kiStride2];
	const uint32_t kuiL3=0x01010101U*pPred[-1+kiStride3];
	ST32A4(pPred,kuiL0);
	ST32A4(pPred+kiStride,kuiL1);
	ST32A4(pPred+kiStride2,kuiL2);
	ST32A4(pPred+kiStride3,kuiL3);
}

void WelsI4x4LumaPredDc_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride2+kiStride;
	const uint8_t kuiMean=(pPred[-1]+pPred[-1+kiStride]+pPred[-1+kiStride2]+pPred[-1+kiStride3]+pPred[-kiStride]+pPred[-kiStride+1]+pPred[-kiStride+2]+pPred[-kiStride+3]+4)>>3;
	const uint32_t kuiMean32=0x01010101U*kuiMean;
	ST32A4(pPred,kuiMean32);
	ST32A4(pPred+kiStride,kuiMean32);
	ST32A4(pPred+kiStride2,kuiMean32);
	ST32A4(pPred+kiStride3,kuiMean32);
}

void WelsI4x4LumaPredDcLeft_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride2+kiStride;
	const uint8_t kuiMean=(pPred[-1]+pPred[-1+kiStride]+pPred[-1+kiStride2]+pPred[-1+kiStride3]+2)>>2;
	const uint32_t kuiMean32=0x01010101U*kuiMean;
	ST32A4(pPred,kuiMean32);
	ST32A4(pPred+kiStride,kuiMean32);
	ST32A4(pPred+kiStride2,kuiMean32);
	ST32A4(pPred+kiStride3,kuiMean32);
}

void WelsI4x4LumaPredDcTop_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride2+kiStride;
	const uint8_t kuiMean=(pPred[-kiStride]+pPred[-kiStride+1]+pPred[-kiStride+2]+pPred[-kiStride+3]+2)>>2;
	const uint32_t kuiMean32=0x01010101U*kuiMean;
	ST32A4(pPred,kuiMean32);
	ST32A4(pPred+kiStride,kuiMean32);
	ST32A4(pPred+kiStride2,kuiMean32);
	ST32A4(pPred+kiStride3,kuiMean32);
}

void WelsI4x4LumaPredDcNA_c(uint8_t* pPred,const int32_t kiStride){
	const uint32_t kuiDC32=0x80808080U;
	ST32A4(pPred,kuiDC32);
	ST32A4(pPred+kiStride,kuiDC32);
	ST32A4(pPred+(kiStride<<1),kuiDC32);
	ST32A4(pPred+(kiStride<<1)+kiStride,kuiDC32);
}

// down pLeft
void WelsI4x4LumaPredDDL_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	// get pTop
	uint8_t* ptop=&pPred[-kiStride];
	const uint8_t kuiT0=*ptop;
	const uint8_t kuiT1=*(ptop+1);
	const uint8_t kuiT2=*(ptop+2);
	const uint8_t kuiT3=*(ptop+3);
	const uint8_t kuiT4=*(ptop+4);
	const uint8_t kuiT5=*(ptop+5);
	const uint8_t kuiT6=*(ptop+6);
	const uint8_t kuiT7=*(ptop+7);
	const uint8_t kuiDDL0=(2+kuiT0+kuiT2+(kuiT1<<1))>>2;		// kDDL0
	const uint8_t kuiDDL1=(2+kuiT1+kuiT3+(kuiT2<<1))>>2;		// kDDL1
	const uint8_t kuiDDL2=(2+kuiT2+kuiT4+(kuiT3<<1))>>2;		// kDDL2
	const uint8_t kuiDDL3=(2+kuiT3+kuiT5+(kuiT4<<1))>>2;		// kDDL3
	const uint8_t kuiDDL4=(2+kuiT4+kuiT6+(kuiT5<<1))>>2;		// kDDL4
	const uint8_t kuiDDL5=(2+kuiT5+kuiT7+(kuiT6<<1))>>2;		// kDDL5
	const uint8_t kuiDDL6=(2+kuiT6+kuiT7+(kuiT7<<1))>>2;		// kDDL6
	const uint8_t kuiList[8]={kuiDDL0,kuiDDL1,kuiDDL2,kuiDDL3,kuiDDL4,kuiDDL5,kuiDDL6,0};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+1));
	ST32A4(pPred+kiStride2,LD32(kuiList+2));
	ST32A4(pPred+kiStride3,LD32(kuiList+3));
}

// down pLeft
void WelsI4x4LumaPredDDLTop_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	// get pTop
	uint8_t* ptop=&pPred[-kiStride];
	const uint8_t kuiT0=*ptop;
	const uint8_t kuiT1=*(ptop+1);
	const uint8_t kuiT2=*(ptop+2);
	const uint8_t kuiT3=*(ptop+3);
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiT23=1+kuiT2+kuiT3;
	const uint16_t kuiT33=1+(kuiT3<<1);
	const uint8_t kuiDLT0=(kuiT01+kuiT12)>>2;		// kDLT0
	const uint8_t kuiDLT1=(kuiT12+kuiT23)>>2;		// kDLT1
	const uint8_t kuiDLT2=(kuiT23+kuiT33)>>2;		// kDLT2
	const uint8_t kuiDLT3=kuiT33>>1; 	// kDLT3
	const uint8_t kuiList[8]={kuiDLT0,kuiDLT1,kuiDLT2,kuiDLT3,kuiDLT3,kuiDLT3,kuiDLT3,kuiDLT3};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+1));
	ST32A4(pPred+kiStride2,LD32(kuiList+2));
	ST32A4(pPred+kiStride3,LD32(kuiList+3));
}

// down right
void WelsI4x4LumaPredDDR_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	uint8_t* ptopleft=&pPred[-(kiStride+1)];
	uint8_t* pleft=&pPred[-1];
	const uint8_t kuiLT=*ptopleft;
	// get pLeft and pTop
	const uint8_t kuiL0=*pleft;
	const uint8_t kuiL1=*(pleft+kiStride);
	const uint8_t kuiL2=*(pleft+kiStride2);
	const uint8_t kuiL3=*(pleft+kiStride3);
	const uint8_t kuiT0=*(ptopleft+1);
	const uint8_t kuiT1=*(ptopleft+2);
	const uint8_t kuiT2=*(ptopleft+3);
	const uint8_t kuiT3=*(ptopleft+4);
	const uint16_t kuiTL0=1+kuiLT+kuiL0;
	const uint16_t kuiLT0=1+kuiLT+kuiT0;
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiT23=1+kuiT2+kuiT3;
	const uint16_t kuiL01=1+kuiL0+kuiL1;
	const uint16_t kuiL12=1+kuiL1+kuiL2;
	const uint16_t kuiL23=1+kuiL2+kuiL3;
	const uint8_t kuiDDR0=(kuiTL0+kuiLT0)>>2;		// kuiDDR0
	const uint8_t kuiDDR1=(kuiLT0+kuiT01)>>2;		// kuiDDR1
	const uint8_t kuiDDR2=(kuiT01+kuiT12)>>2;		// kuiDDR2
	const uint8_t kuiDDR3=(kuiT12+kuiT23)>>2;		// kuiDDR3
	const uint8_t kuiDDR4=(kuiTL0+kuiL01)>>2;		// kuiDDR4
	const uint8_t kuiDDR5=(kuiL01+kuiL12)>>2;		// kuiDDR5
	const uint8_t kuiDDR6=(kuiL12+kuiL23)>>2;		// kuiDDR6
	const uint8_t kuiList[8]={kuiDDR6,kuiDDR5,kuiDDR4,kuiDDR0,kuiDDR1,kuiDDR2,kuiDDR3,0};

	ST32A4(pPred,LD32(kuiList+3));
	ST32A4(pPred+kiStride,LD32(kuiList+2));
	ST32A4(pPred+kiStride2,LD32(kuiList+1));
	ST32A4(pPred+kiStride3,LD32(kuiList));
}

// vertical pLeft
void WelsI4x4LumaPredVL_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	uint8_t* ptopleft=&pPred[-(kiStride+1)];
	// get pTop
	const uint8_t kuiT0=*(ptopleft+1);
	const uint8_t kuiT1=*(ptopleft+2);
	const uint8_t kuiT2=*(ptopleft+3);
	const uint8_t kuiT3=*(ptopleft+4);
	const uint8_t kuiT4=*(ptopleft+5);
	const uint8_t kuiT5=*(ptopleft+6);
	const uint8_t kuiT6=*(ptopleft+7);
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiT23=1+kuiT2+kuiT3;
	const uint16_t kuiT34=1+kuiT3+kuiT4;
	const uint16_t kuiT45=1+kuiT4+kuiT5;
	const uint16_t kuiT56=1+kuiT5+kuiT6;
	const uint8_t kuiVL0=kuiT01>>1; 	// kuiVL0
	const uint8_t kuiVL1=kuiT12>>1; 	// kuiVL1
	const uint8_t kuiVL2=kuiT23>>1; 	// kuiVL2
	const uint8_t kuiVL3=kuiT34>>1; 	// kuiVL3
	const uint8_t kuiVL4=kuiT45>>1; 	// kuiVL4
	const uint8_t kuiVL5=(kuiT01+kuiT12)>>2;		// kuiVL5
	const uint8_t kuiVL6=(kuiT12+kuiT23)>>2;		// kuiVL6
	const uint8_t kuiVL7=(kuiT23+kuiT34)>>2;		// kuiVL7
	const uint8_t kuiVL8=(kuiT34+kuiT45)>>2;		// kuiVL8
	const uint8_t kuiVL9=(kuiT45+kuiT56)>>2;		// kuiVL9
	const uint8_t kuiList[10]={kuiVL0,kuiVL1,kuiVL2,kuiVL3,kuiVL4,kuiVL5,kuiVL6,kuiVL7,kuiVL8,kuiVL9};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+5));
	ST32A4(pPred+kiStride2,LD32(kuiList+1));
	ST32A4(pPred+kiStride3,LD32(kuiList+6));
}

// vertical pLeft
void WelsI4x4LumaPredVLTop_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	uint8_t* ptopleft=&pPred[-(kiStride+1)];
	// get pTop
	const uint8_t kuiT0=*(ptopleft+1);
	const uint8_t kuiT1=*(ptopleft+2);
	const uint8_t kuiT2=*(ptopleft+3);
	const uint8_t kuiT3=*(ptopleft+4);
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiT23=1+kuiT2+kuiT3;
	const uint16_t kuiT33=1+(kuiT3<<1);
	const uint8_t kuiVL0=kuiT01>>1;
	const uint8_t kuiVL1=kuiT12>>1;
	const uint8_t kuiVL2=kuiT23>>1;
	const uint8_t kuiVL3=kuiT33>>1;
	const uint8_t kuiVL4=(kuiT01+kuiT12)>>2;
	const uint8_t kuiVL5=(kuiT12+kuiT23)>>2;
	const uint8_t kuiVL6=(kuiT23+kuiT33)>>2;
	const uint8_t kuiVL7=kuiVL3;
	const uint8_t kuiList[10]={kuiVL0,kuiVL1,kuiVL2,kuiVL3,kuiVL3,kuiVL4,kuiVL5,kuiVL6,kuiVL7,kuiVL7};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+5));
	ST32A4(pPred+kiStride2,LD32(kuiList+1));
	ST32A4(pPred+kiStride3,LD32(kuiList+6));
}


// vertical right
void WelsI4x4LumaPredVR_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	const uint8_t kuiLT=pPred[-kiStride-1];
	// get pLeft and pTop
	const uint8_t kuiL0=pPred[-1];
	const uint8_t kuiL1=pPred[kiStride-1];
	const uint8_t kuiL2=pPred[kiStride2-1];
	const uint8_t kuiT0=pPred[-kiStride];
	const uint8_t kuiT1=pPred[1-kiStride];
	const uint8_t kuiT2=pPred[2-kiStride];
	const uint8_t kuiT3=pPred[3-kiStride];
	const uint8_t kuiVR0=(1+kuiLT+kuiT0)>>1;		// kuiVR0
	const uint8_t kuiVR1=(1+kuiT0+kuiT1)>>1;		// kuiVR1
	const uint8_t kuiVR2=(1+kuiT1+kuiT2)>>1;		// kuiVR2
	const uint8_t kuiVR3=(1+kuiT2+kuiT3)>>1;		// kuiVR3
	const uint8_t kuiVR4=(2+kuiL0+(kuiLT<<1)+kuiT0)>>2;		// kuiVR4
	const uint8_t kuiVR5=(2+kuiLT+(kuiT0<<1)+kuiT1)>>2;		// kuiVR5
	const uint8_t kuiVR6=(2+kuiT0+(kuiT1<<1)+kuiT2)>>2;		// kuiVR6
	const uint8_t kuiVR7=(2+kuiT1+(kuiT2<<1)+kuiT3)>>2;		// kuiVR7
	const uint8_t kuiVR8=(2+kuiLT+(kuiL0<<1)+kuiL1)>>2;		// kuiVR8
	const uint8_t kuiVR9=(2+kuiL0+(kuiL1<<1)+kuiL2)>>2;		// kuiVR9
	const uint8_t kuiList[10]={kuiVR8,kuiVR0,kuiVR1,kuiVR2,kuiVR3,kuiVR9,kuiVR4,kuiVR5,kuiVR6,kuiVR7};

	ST32A4(pPred,LD32(kuiList+1));
	ST32A4(pPred+kiStride,LD32(kuiList+6));
	ST32A4(pPred+kiStride2,LD32(kuiList));
	ST32A4(pPred+kiStride3,LD32(kuiList+5));
}

// horizontal up
void WelsI4x4LumaPredHU_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	// get pLeft
	const uint8_t kuiL0=pPred[-1];
	const uint8_t kuiL1=pPred[kiStride-1];
	const uint8_t kuiL2=pPred[kiStride2-1];
	const uint8_t kuiL3=pPred[kiStride3-1];
	const uint16_t kuiL01=1+kuiL0+kuiL1;
	const uint16_t kuiL12=1+kuiL1+kuiL2;
	const uint16_t kuiL23=1+kuiL2+kuiL3;
	const uint8_t kuiHU0=kuiL01>>1;
	const uint8_t kuiHU1=(kuiL01+kuiL12)>>2;
	const uint8_t kuiHU2=kuiL12>>1;
	const uint8_t kuiHU3=(kuiL12+kuiL23)>>2;
	const uint8_t kuiHU4=kuiL23>>1;
	const uint8_t kuiHU5=(1+kuiL23+(kuiL3<<1))>>2;
	const uint8_t kuiList[10]={kuiHU0,kuiHU1,kuiHU2,kuiHU3,kuiHU4,kuiHU5,kuiL3,kuiL3,kuiL3,kuiL3};

	ST32A4(pPred,LD32(kuiList));
	ST32A4(pPred+kiStride,LD32(kuiList+2));
	ST32A4(pPred+kiStride2,LD32(kuiList+4));
	ST32A4(pPred+kiStride3,LD32(kuiList+6));
}

// horizontal down
void WelsI4x4LumaPredHD_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	const uint8_t kuiLT=pPred[-(kiStride+1)];
	// get pLeft and pTop
	const uint8_t kuiL0=pPred[-1];
	const uint8_t kuiL1=pPred[-1+kiStride];
	const uint8_t kuiL2=pPred[-1+kiStride2];
	const uint8_t kuiL3=pPred[-1+kiStride3];
	const uint8_t kuiT0=pPred[-kiStride];
	const uint8_t kuiT1=pPred[-kiStride+1];
	const uint8_t kuiT2=pPred[-kiStride+2];
	const uint16_t kuiTL0=1+kuiLT+kuiL0;
	const uint16_t kuiLT0=1+kuiLT+kuiT0;
	const uint16_t kuiT01=1+kuiT0+kuiT1;
	const uint16_t kuiT12=1+kuiT1+kuiT2;
	const uint16_t kuiL01=1+kuiL0+kuiL1;
	const uint16_t kuiL12=1+kuiL1+kuiL2;
	const uint16_t kuiL23=1+kuiL2+kuiL3;
	const uint8_t kuiHD0=kuiTL0>>1;
	const uint8_t kuiHD1=(kuiTL0+kuiLT0)>>2;
	const uint8_t kuiHD2=(kuiLT0+kuiT01)>>2;
	const uint8_t kuiHD3=(kuiT01+kuiT12)>>2;
	const uint8_t kuiHD4=kuiL01>>1;
	const uint8_t kuiHD5=(kuiTL0+kuiL01)>>2;
	const uint8_t kuiHD6=kuiL12>>1;
	const uint8_t kuiHD7=(kuiL01+kuiL12)>>2;
	const uint8_t kuiHD8=kuiL23>>1;
	const uint8_t kuiHD9=(kuiL12+kuiL23)>>2;
	const uint8_t kuiList[10]={kuiHD8,kuiHD9,kuiHD6,kuiHD7,kuiHD4,kuiHD5,kuiHD0,kuiHD1,kuiHD2,kuiHD3};

	ST32A4(pPred,LD32(kuiList+6));
	ST32A4(pPred+kiStride,LD32(kuiList+4));
	ST32A4(pPred+kiStride2,LD32(kuiList+2));
	ST32A4(pPred+kiStride3,LD32(kuiList));
}

void WelsI8x8LumaPredV_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	uint64_t uiTop=0;
	int32_t iStride[8];
	uint8_t uiPixelFilterT[8];
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	// 8-89
	for(i=7; i>=0; i--){
		uiTop=((uiTop<<8)|uiPixelFilterT[i]);
	}
	for(i=0; i<8; i++){
		ST64A8(pPred+kiStride*i,uiTop);
	}
}

void WelsI8x8LumaPredH_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	uint64_t uiLeft;
	int32_t iStride[8];
	uint8_t uiPixelFilterL[8];
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterL[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2) : ((pPred[-1]*3+pPred[-1+iStride[1]]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	// 8-90
	for(i=0; i<8; i++){
		uiLeft=0x0101010101010101ULL*uiPixelFilterL[i];
		ST64A8(pPred+iStride[i],uiLeft);
	}
}

void WelsI8x8LumaPredDc_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	int32_t iStride[8];
	uint8_t uiPixelFilterL[8];
	uint8_t uiPixelFilterT[8];
	uint16_t uiTotal=0;
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterL[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2) : ((pPred[-1]*3+pPred[-1+iStride[1]]+2)>>2);
	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	// 8-91
	for(i=0; i<8; i++){
		uiTotal+=uiPixelFilterL[i];
		uiTotal+=uiPixelFilterT[i];
	}
	const uint8_t kuiMean=((uiTotal+8)>>4);
	const uint64_t kuiMean64=0x0101010101010101ULL*kuiMean;
	for(i=0; i<8; i++){
		ST64A8(pPred+iStride[i],kuiMean64);
	}
}

void WelsI8x8LumaPredDcLeft_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	int32_t iStride[8];
	uint8_t uiPixelFilterL[8];
	uint16_t uiTotal=0;
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterL[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2) : ((pPred[-1]*3+pPred[-1+iStride[1]]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	// 8-92
	for(i=0; i<8; i++){
		uiTotal+=uiPixelFilterL[i];
	}
	const uint8_t kuiMean=((uiTotal+4)>>3);
	const uint64_t kuiMean64=0x0101010101010101ULL*kuiMean;
	for(i=0; i<8; i++){
		ST64A8(pPred+iStride[i],kuiMean64);
	}
}

void WelsI8x8LumaPredDcTop_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	int32_t iStride[8];
	uint8_t uiPixelFilterT[8];
	uint16_t uiTotal=0;
	int32_t i;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	// 8-93
	for(i=0; i<8; i++){
		uiTotal+=uiPixelFilterT[i];
	}
	const uint8_t kuiMean=((uiTotal+4)>>3);
	const uint64_t kuiMean64=0x0101010101010101ULL*kuiMean;
	for(i=0; i<8; i++){
		ST64A8(pPred+iStride[i],kuiMean64);
	}
}

void WelsI8x8LumaPredDcNA_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// for normal 8 bit depth,8-94
	const uint64_t kuiDC64=0x8080808080808080ULL;
	int32_t iStride[8];
	int32_t i;
	ST64A8(pPred,kuiDC64);
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
		ST64A8(pPred+iStride[i],kuiDC64);
	}
}

// down pLeft
void WelsI8x8LumaPredDDL_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// Top and Top-right available
	int32_t iStride[8];
	uint8_t uiPixelFilterT[16];
	int32_t i,j;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<15; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterT[15]=((pPred[14-kiStride]+pPred[15-kiStride]*3+2)>>2);
	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			if(i==7 && j==7){		// 8-95
				pPred[j+iStride[i]]=(uiPixelFilterT[14]+3*uiPixelFilterT[15]+2)>>2;
			}else{		// 8-96
				pPred[j+iStride[i]]=(uiPixelFilterT[i+j]+(uiPixelFilterT[i+j+1]<<1)+uiPixelFilterT[i+j+2]+2)>>2;
			}
		}
	}
}

// down pLeft
void WelsI8x8LumaPredDDLTop_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// Top available and Top-right unavailable
	int32_t iStride[8];
	uint8_t uiPixelFilterT[16];
	int32_t i,j;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((
		pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	// p[x,-1] x=8...15 are replaced with p[7,-1]
	uiPixelFilterT[7]=((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	for(i=8; i<16; i++){
		uiPixelFilterT[i]=pPred[7-kiStride];
	}

	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			if(i==7 && j==7){		// 8-95
				pPred[j+iStride[i]]=(uiPixelFilterT[14]+3*uiPixelFilterT[15]+2)>>2;
			}else{		// 8-96
				pPred[j+iStride[i]]=(uiPixelFilterT[i+j]+(uiPixelFilterT[i+j+1]<<1)+uiPixelFilterT[i+j+2]+2)>>2;
			}
		}
	}
}

// down right
void WelsI8x8LumaPredDDR_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// The TopLeft,Top,Left are all available under this mode
	int32_t iStride[8];
	uint8_t uiPixelFilterTL;
	uint8_t uiPixelFilterL[8];
	uint8_t uiPixelFilterT[8];
	int32_t i,j;
	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}
	uiPixelFilterTL=(pPred[-1]+(pPred[-1-kiStride]<<1)+pPred[-kiStride]+2)>>2;
	uiPixelFilterL[0]=((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2);
	uiPixelFilterT[0]=((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((
		pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);

	for(i=0; i<8; i++){		// y
		// 8-98,x < y-1
		for(j=0; j<(i-1); j++){
			pPred[j+iStride[i]]=(uiPixelFilterL[i-j-2]+(uiPixelFilterL[i-j-1]<<1)+uiPixelFilterL[i-j]+2)>>2;
		}
		// 8-98,special case,x==y-1
		if(i>=1){
			j=i-1;
			pPred[j+iStride[i]]=(uiPixelFilterTL+(uiPixelFilterL[0]<<1)+uiPixelFilterL[1]+2)>>2;
		}
		// 8-99,x==y
		j=i;
		pPred[j+iStride[i]]=(uiPixelFilterT[0]+(uiPixelFilterTL<<1)+uiPixelFilterL[0]+2)>>2;
		// 8-97,special case,x==y+1
		if(i<7){
			j=i+1;
			pPred[j+iStride[i]]=(uiPixelFilterTL+(uiPixelFilterT[0]<<1)+uiPixelFilterT[1]+2)>>2;
		}
		for(j=i+2; j<8; j++){		// 8-97,x > y+1
			pPred[j+iStride[i]]=(uiPixelFilterT[j-i-2]+(uiPixelFilterT[j-i-1]<<1)+uiPixelFilterT[j-i]+2)>>2;
		}
	}
}

// vertical pLeft
void WelsI8x8LumaPredVL_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// Top and Top-right available
	int32_t iStride[8];
	uint8_t uiPixelFilterT[16];
	int32_t i,j;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((
		pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<15; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterT[15]=((pPred[14-kiStride]+pPred[15-kiStride]*3+2)>>2);

	for(i=0; i<8; i++){		// y
		if((i&0x01)==0){		// 8-108
			for(j=0; j<8; j++){		// x
				pPred[j+iStride[i]]=(uiPixelFilterT[j+(i>>1)]+uiPixelFilterT[j+(i>>1)+1]+1)>>1;
			}
		}else{		// 8-109
			for(j=0; j<8; j++){		// x
				pPred[j+iStride[i]]=(uiPixelFilterT[j+(i>>1)]+(uiPixelFilterT[j+(i>>1)+1]<<1)+uiPixelFilterT[j+(i>>1)+2]+2)>>2;
			}
		}
	}
}

// vertical pLeft
void WelsI8x8LumaPredVLTop_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// Top available and Top-right unavailable
	int32_t iStride[8];
	uint8_t uiPixelFilterT[16];
	int32_t i,j;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterT[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2) : ((
		pPred[-kiStride]*3+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	// p[x,-1] x=8...15 are replaced with p[7,-1]
	uiPixelFilterT[7]=((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);
	for(i=8; i<16; i++){
		uiPixelFilterT[i]=pPred[7-kiStride];
	}

	for(i=0; i<8; i++){		// y
		if((i&0x01)==0){		// 8-108
			for(j=0; j<8; j++){		// x
				pPred[j+iStride[i]]=(uiPixelFilterT[j+(i>>1)]+uiPixelFilterT[j+(i>>1)+1]+1)>>1;
			}
		}else{		// 8-109
			for(j=0; j<8; j++){		// x
				pPred[j+iStride[i]]=(uiPixelFilterT[j+(i>>1)]+(uiPixelFilterT[j+(i>>1)+1]<<1)+uiPixelFilterT[j+(i>>1)+2]+2)>>2;
			}
		}
	}
}

// vertical right
void WelsI8x8LumaPredVR_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// The TopLeft,Top,Left are always available under this mode
	int32_t iStride[8];
	uint8_t uiPixelFilterTL;
	uint8_t uiPixelFilterL[8];
	uint8_t uiPixelFilterT[8];
	int32_t i,j;
	int32_t izVR,izVRDiv;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterTL=(pPred[-1]+(pPred[-1-kiStride]<<1)+pPred[-kiStride]+2)>>2;

	uiPixelFilterL[0]=((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2);
	uiPixelFilterT[0]=((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>2);
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);

	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			izVR=(j<<1)-i;		// 2 * x-y
			izVRDiv=j-(i>>1);
			if(izVR>=0){
				if((izVR&0x01)==0){		// 8-100
					if(izVRDiv>0){
						pPred[j+iStride[i]]=(uiPixelFilterT[izVRDiv-1]+uiPixelFilterT[izVRDiv]+1)>>1;
					}else{
						pPred[j+iStride[i]]=(uiPixelFilterTL+uiPixelFilterT[0]+1)>>1;
					}
				}else{		// 8-101
					if(izVRDiv>1){
						pPred[j+iStride[i]]=(uiPixelFilterT[izVRDiv-2]+(uiPixelFilterT[izVRDiv-1]<<1)+uiPixelFilterT[izVRDiv]+2)
							>>2;
					}else{
						pPred[j+iStride[i]]=(uiPixelFilterTL+(uiPixelFilterT[0]<<1)+uiPixelFilterT[1]+2)>>2;
					}
				}
			}else
			if(izVR==-1){		// 8-102
				pPred[j+iStride[i]]=(uiPixelFilterL[0]+(uiPixelFilterTL<<1)+uiPixelFilterT[0]+2)>>2;
			}else
			if(izVR<-2){		// 8-103
				pPred[j+iStride[i]]=(uiPixelFilterL[-izVR-1]+(uiPixelFilterL[-izVR-2]<<1)+uiPixelFilterL[-izVR-3]+2)
					>>2;
			}else{		// izVR==-2,8-103,special case
				pPred[j+iStride[i]]=(uiPixelFilterL[1]+(uiPixelFilterL[0]<<1)+uiPixelFilterTL+2)>>2;
			}
		}
	}
}

// horizontal up
void WelsI8x8LumaPredHU_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	int32_t iStride[8];
	uint8_t uiPixelFilterL[8];
	int32_t i,j;
	int32_t izHU;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterL[0]=bTLAvail ? ((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2) : ((
		pPred[-1]*3+pPred[-1+iStride[1]]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>
							 2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);

	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			izHU=j+(i<<1);		// x+2 * y
			if(izHU<13){
				if((izHU&0x01)==0){		// 8-110
					pPred[j+iStride[i]]=(uiPixelFilterL[izHU>>1]+uiPixelFilterL[1+(izHU>>1)]+1)>>1;
				}else{		// 8-111
					pPred[j+iStride[i]]=(uiPixelFilterL[izHU>>1]+(uiPixelFilterL[1+(izHU>>1)]<<1)+uiPixelFilterL[2+
											 (izHU>>1)]+2)>>2;
				}
			}else
			if(izHU==13){		// 8-112
				pPred[j+iStride[i]]=(uiPixelFilterL[6]+3*uiPixelFilterL[7]+2)>>2;
			}else{		// 8-113
				pPred[j+iStride[i]]=uiPixelFilterL[7];
			}
		}
	}
}

// horizontal down
void WelsI8x8LumaPredHD_c(uint8_t* pPred,const int32_t kiStride,bool bTLAvail,bool bTRAvail){
	// The TopLeft,Top,Left are all available under this mode
	int32_t iStride[8];
	uint8_t uiPixelFilterTL;
	uint8_t uiPixelFilterL[8];
	uint8_t uiPixelFilterT[8];
	int32_t i,j;
	int32_t izHD,izHDDiv;

	for(iStride[0]=0,i=1; i<8; i++){
		iStride[i]=iStride[i-1]+kiStride;
	}

	uiPixelFilterTL=(pPred[-1]+(pPred[-1-kiStride]<<1)+pPred[-kiStride]+2)>>2;

	uiPixelFilterL[0]=((pPred[-1-kiStride]+(pPred[-1]<<1)+pPred[-1+iStride[1]]+2)>>2);
	uiPixelFilterT[0]=((pPred[-1-kiStride]+(pPred[-kiStride]<<1)+pPred[1-kiStride]+2)>>2);
	for(i=1; i<7; i++){
		uiPixelFilterL[i]=((pPred[-1+iStride[i-1]]+(pPred[-1+iStride[i]]<<1)+pPred[-1+iStride[i+1]]+2)>>
							 2);
		uiPixelFilterT[i]=((pPred[i-1-kiStride]+(pPred[i-kiStride]<<1)+pPred[i+1-kiStride]+2)>>2);
	}
	uiPixelFilterL[7]=((pPred[-1+iStride[6]]+pPred[-1+iStride[7]]*3+2)>>2);
	uiPixelFilterT[7]=bTRAvail ? ((pPred[6-kiStride]+(pPred[7-kiStride]<<1)+pPred[8-kiStride]+2)>>2) : ((
		pPred[6-kiStride]+pPred[7-kiStride]*3+2)>>2);

	for(i=0; i<8; i++){		// y
		for(j=0; j<8; j++){		// x
			izHD=(i<<1)-j;		// 2*y-x
			izHDDiv=i-(j>>1);
			if(izHD>=0){
				if((izHD&0x01)==0){		// 8-104
					if(izHDDiv==0){
						pPred[j+iStride[i]]=(uiPixelFilterTL+uiPixelFilterL[0]+1)>>1;
					}else{
						pPred[j+iStride[i]]=(uiPixelFilterL[izHDDiv-1]+uiPixelFilterL[izHDDiv]+1)>>1;
					}
				}else{		// 8-105
					if(izHDDiv==1){
						pPred[j+iStride[i]]=(uiPixelFilterTL+(uiPixelFilterL[0]<<1)+uiPixelFilterL[1]+2)>>2;
					}else{
						pPred[j+iStride[i]]=(uiPixelFilterL[izHDDiv-2]+(uiPixelFilterL[izHDDiv-1]<<1)+uiPixelFilterL[izHDDiv]+2)
							>>2;
					}
				}
			}else
			if(izHD==-1){		// 8-106
				pPred[j+iStride[i]]=(uiPixelFilterL[0]+(uiPixelFilterTL<<1)+uiPixelFilterT[0]+2)>>2;
			}else
			if(izHD<-2){		// 8-107
				pPred[j+iStride[i]]=(uiPixelFilterT[-izHD-1]+(uiPixelFilterT[-izHD-2]<<1)+uiPixelFilterT[-izHD-3]+2)
					>>2;
			}else{		// 8-107 special case,izHD==-2
				pPred[j+iStride[i]]=(uiPixelFilterT[1]+(uiPixelFilterT[0]<<1)+uiPixelFilterTL+2)>>2;
			}
		}
	}
}

void WelsIChromaPredH_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<3)-kiStride;
	uint8_t i=7;

	do{
		const uint8_t kuiVal8=pPred[iTmp-1];
		const uint64_t kuiVal64=0x0101010101010101ULL*kuiVal8;

		ST64A8(pPred+iTmp,kuiVal64);

		iTmp-=kiStride;
	} while(i-->0);
}

void WelsIChromaPredV_c(uint8_t* pPred,const int32_t kiStride){
	const uint64_t kuiVal64=LD64A8(&pPred[-kiStride]);
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride4=kiStride2<<1;

	ST64A8(pPred,kuiVal64);
	ST64A8(pPred+kiStride,kuiVal64);
	ST64A8(pPred+kiStride2,kuiVal64);
	ST64A8(pPred+kiStride2+kiStride,kuiVal64);
	ST64A8(pPred+kiStride4,kuiVal64);
	ST64A8(pPred+kiStride4+kiStride,kuiVal64);
	ST64A8(pPred+kiStride4+kiStride2,kuiVal64);
	ST64A8(pPred+(kiStride<<3)-kiStride,kuiVal64);
}

void WelsIChromaPredPlane_c(uint8_t* pPred,const int32_t kiStride){
	int32_t a=0,b=0,c=0,H=0,V=0;
	int32_t i,j;
	uint8_t* pTop=&pPred[-kiStride];
	uint8_t* pLeft=&pPred[-1];

	for(i=0; i<4; i++){
		H+=(i+1)*(pTop[4+i]-pTop[2-i]);
		V+=(i+1)*(pLeft[(4+i)*kiStride]-pLeft[(2-i)*kiStride]);
	}

	a=(pLeft[7*kiStride]+pTop[7])<<4;
	b=(17*H+16)>>5;
	c=(17*V+16)>>5;

	for(i=0; i<8; i++){
		for(j=0; j<8; j++){
			int32_t iTmp=(a+b*(j-3)+c*(i-3)+16)>>5;
			iTmp=WelsClip1(iTmp);
			pPred[j]=iTmp;
		}
		pPred+=kiStride;
	}
}

void WelsIChromaPredDcLeft_c(uint8_t* pPred,const int32_t kiStride){
	const int32_t kiL1=-1+kiStride;
	const int32_t kiL2=kiL1+kiStride;
	const int32_t kiL3=kiL2+kiStride;
	const int32_t kiL4=kiL3+kiStride;
	const int32_t kiL5=kiL4+kiStride;
	const int32_t kiL6=kiL5+kiStride;
	const int32_t kiL7=kiL6+kiStride;
	// caculate the kMean value
	const uint8_t kuiMUP=(pPred[-1]+pPred[kiL1]+pPred[kiL2]+pPred[kiL3]+2)>>2;
	const uint8_t kuiMDown=(pPred[kiL4]+pPred[kiL5]+pPred[kiL6]+pPred[kiL7]+2)>>2;
	const uint64_t kuiUP64=0x0101010101010101ULL*kuiMUP;
	const uint64_t kuiDN64=0x0101010101010101ULL*kuiMDown;

	ST64A8(pPred,kuiUP64);
	ST64A8(pPred+kiL1+1,kuiUP64);
	ST64A8(pPred+kiL2+1,kuiUP64);
	ST64A8(pPred+kiL3+1,kuiUP64);
	ST64A8(pPred+kiL4+1,kuiDN64);
	ST64A8(pPred+kiL5+1,kuiDN64);
	ST64A8(pPred+kiL6+1,kuiDN64);
	ST64A8(pPred+kiL7+1,kuiDN64);
}

void WelsIChromaPredDcTop_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<3)-kiStride;
	// caculate the kMean value
	const uint8_t kuiM1=(pPred[-kiStride]+pPred[1-kiStride]+pPred[2-kiStride]+pPred[3-kiStride]+2)>>2;
	const uint8_t kuiM2=(pPred[4-kiStride]+pPred[5-kiStride]+pPred[6-kiStride]+pPred[7-kiStride]+2)>>
		2;
	const uint8_t kuiM[8]={kuiM1,kuiM1,kuiM1,kuiM1,kuiM2,kuiM2,kuiM2,kuiM2};

	uint8_t i=7;

	do{
		ST64A8(pPred+iTmp,LD64(kuiM));

		iTmp-=kiStride;
	} while(i-->0);
}

void WelsIChromaPredDcNA_c(uint8_t* pPred,const int32_t kiStride){
	int32_t iTmp=(kiStride<<3)-kiStride;
	const uint64_t kuiDC64=0x8080808080808080ULL;
	uint8_t i=7;

	do{
		ST64A8(pPred+iTmp,kuiDC64);

		iTmp-=kiStride;
	} while(i-->0);
}

// NOTE::: p_RS should NOT be modified and it will lead to mismatch with JSVM.
// so should allocate kA array to store the temporary value (idct).
void IdctResAddPred_c(uint8_t* pPred,const int32_t kiStride,int16_t* pRs){
	int16_t iSrc[16];

	uint8_t* pDst=pPred;
	const int32_t kiStride2=kiStride<<1;
	const int32_t kiStride3=kiStride+kiStride2;
	int32_t i;

	for(i=0; i<4; i++){
		const int32_t kiY=i<<2;
		const int32_t kiT0=pRs[kiY]+pRs[kiY+2];
		const int32_t kiT1=pRs[kiY]-pRs[kiY+2];
		const int32_t kiT2=(pRs[kiY+1]>>1)-pRs[kiY+3];
		const int32_t kiT3=pRs[kiY+1]+(pRs[kiY+3]>>1);

		iSrc[kiY]=kiT0+kiT3;
		iSrc[kiY+1]=kiT1+kiT2;
		iSrc[kiY+2]=kiT1-kiT2;
		iSrc[kiY+3]=kiT0-kiT3;
	}

	for(i=0; i<4; i++){
		int32_t kT1=iSrc[i]+iSrc[i+8];
		int32_t kT2=iSrc[i+4]+(iSrc[i+12]>>1);
		int32_t kT3=(32+kT1+kT2)>>6;
		int32_t kT4=(32+kT1-kT2)>>6;

		pDst[i]=WelsClip1(kT3+pPred[i]);
		pDst[i+kiStride3]=WelsClip1(kT4+pPred[i+kiStride3]);

		kT1=iSrc[i]-iSrc[i+8];
		kT2=(iSrc[i+4]>>1)-iSrc[i+12];
		pDst[i+kiStride]=WelsClip1(((32+kT1+kT2)>>6)+pDst[i+kiStride]);
		pDst[i+kiStride2]=WelsClip1(((32+kT1-kT2)>>6)+pDst[i+kiStride2]);
	}
}

template<void pfIdctResAddPred(uint8_t* pPred,int32_t iStride,int16_t* pRs)>
void IdctFourResAddPred_(uint8_t* pPred,int32_t iStride,int16_t* pRs,const int8_t* pNzc){
	if(pNzc[0] || pRs[0*16])
		pfIdctResAddPred(pPred+0*iStride+0,iStride,pRs+0*16);
	if(pNzc[1] || pRs[1*16])
		pfIdctResAddPred(pPred+0*iStride+4,iStride,pRs+1*16);
	if(pNzc[4] || pRs[2*16])
		pfIdctResAddPred(pPred+4*iStride+0,iStride,pRs+2*16);
	if(pNzc[5] || pRs[3*16])
		pfIdctResAddPred(pPred+4*iStride+4,iStride,pRs+3*16);
}

void IdctResAddPred8x8_c(uint8_t* pPred,const int32_t kiStride,int16_t* pRs){
	// To make the ASM code easy to write,should using one funciton to apply hor and ver together,such as we did on HEVC
	// Ugly code,just for easy debug,the final version need optimization
	int16_t p[8],b[8];
	int16_t a[4];

	int16_t iTmp[64];
	int16_t iRes[64];

	// Horizontal
	for(int i=0; i<8; i++){
		for(int j=0; j<8; j++){
			p[j]=pRs[j+(i<<3)];
		}
		a[0]=p[0]+p[4];
		a[1]=p[0]-p[4];
		a[2]=p[6]-(p[2]>>1);
		a[3]=p[2]+(p[6]>>1);

		b[0]=a[0]+a[3];
		b[2]=a[1]-a[2];
		b[4]=a[1]+a[2];
		b[6]=a[0]-a[3];

		a[0]=-p[3]+p[5]-p[7]-(p[7]>>1);
		a[1]=p[1]+p[7]-p[3]-(p[3]>>1);
		a[2]=-p[1]+p[7]+p[5]+(p[5]>>1);
		a[3]=p[3]+p[5]+p[1]+(p[1]>>1);

		b[1]=a[0]+(a[3]>>2);
		b[3]=a[1]+(a[2]>>2);
		b[5]=a[2]-(a[1]>>2);
		b[7]=a[3]-(a[0]>>2);

		iTmp[0+(i<<3)]=b[0]+b[7];
		iTmp[1+(i<<3)]=b[2]-b[5];
		iTmp[2+(i<<3)]=b[4]+b[3];
		iTmp[3+(i<<3)]=b[6]+b[1];
		iTmp[4+(i<<3)]=b[6]-b[1];
		iTmp[5+(i<<3)]=b[4]-b[3];
		iTmp[6+(i<<3)]=b[2]+b[5];
		iTmp[7+(i<<3)]=b[0]-b[7];
	}

	// Vertical
	for(int i=0; i<8; i++){
		for(int j=0; j<8; j++){
			p[j]=iTmp[i+(j<<3)];
		}

		a[0]=p[0]+p[4];
		a[1]=p[0]-p[4];
		a[2]=p[6]-(p[2]>>1);
		a[3]=p[2]+(p[6]>>1);

		b[0]=a[0]+a[3];
		b[2]=a[1]-a[2];
		b[4]=a[1]+a[2];
		b[6]=a[0]-a[3];

		a[0]=-p[3]+p[5]-p[7]-(p[7]>>1);
		a[1]=p[1]+p[7]-p[3]-(p[3]>>1);
		a[2]=-p[1]+p[7]+p[5]+(p[5]>>1);
		a[3]=p[3]+p[5]+p[1]+(p[1]>>1);


		b[1]=a[0]+(a[3]>>2);
		b[7]=a[3]-(a[0]>>2);
		b[3]=a[1]+(a[2]>>2);
		b[5]=a[2]-(a[1]>>2);

		iRes[(0<<3)+i]=b[0]+b[7];
		iRes[(1<<3)+i]=b[2]-b[5];
		iRes[(2<<3)+i]=b[4]+b[3];
		iRes[(3<<3)+i]=b[6]+b[1];
		iRes[(4<<3)+i]=b[6]-b[1];
		iRes[(5<<3)+i]=b[4]-b[3];
		iRes[(6<<3)+i]=b[2]+b[5];
		iRes[(7<<3)+i]=b[0]-b[7];
	}

	uint8_t* pDst=pPred;
	for(int i=0; i<8; i++){
		for(int j=0; j<8; j++){
			pDst[i*kiStride+j]=WelsClip1(((32+iRes[(i<<3)+j])>>6)+pDst[i*kiStride+j]);
		}
	}

}

void InitPredFunc(SDecoderContext* pCtx,uint32_t uiCpuFlag){
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_V]=WelsI16x16LumaPredV_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_H]=WelsI16x16LumaPredH_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_DC]=WelsI16x16LumaPredDc_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_P]=WelsI16x16LumaPredPlane_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_DC_L]=WelsI16x16LumaPredDcLeft_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_DC_T]=WelsI16x16LumaPredDcTop_c;
	pCtx->pGetI16x16LumaPredFunc[I16_PRED_DC_128]=WelsI16x16LumaPredDcNA_c;

	pCtx->pGetI4x4LumaPredFunc[I4_PRED_V]=WelsI4x4LumaPredV_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_H]=WelsI4x4LumaPredH_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DC]=WelsI4x4LumaPredDc_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DC_L]=WelsI4x4LumaPredDcLeft_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DC_T]=WelsI4x4LumaPredDcTop_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DC_128]=WelsI4x4LumaPredDcNA_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DDL]=WelsI4x4LumaPredDDL_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DDL_TOP]=WelsI4x4LumaPredDDLTop_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_DDR]=WelsI4x4LumaPredDDR_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_VL]=WelsI4x4LumaPredVL_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_VL_TOP]=WelsI4x4LumaPredVLTop_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_VR]=WelsI4x4LumaPredVR_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_HU]=WelsI4x4LumaPredHU_c;
	pCtx->pGetI4x4LumaPredFunc[I4_PRED_HD]=WelsI4x4LumaPredHD_c;

	pCtx->pGetI8x8LumaPredFunc[I4_PRED_V]=WelsI8x8LumaPredV_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_H]=WelsI8x8LumaPredH_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DC]=WelsI8x8LumaPredDc_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DC_L]=WelsI8x8LumaPredDcLeft_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DC_T]=WelsI8x8LumaPredDcTop_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DC_128]=WelsI8x8LumaPredDcNA_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DDL]=WelsI8x8LumaPredDDL_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DDL_TOP]=WelsI8x8LumaPredDDLTop_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_DDR]=WelsI8x8LumaPredDDR_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_VL]=WelsI8x8LumaPredVL_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_VL_TOP]=WelsI8x8LumaPredVLTop_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_VR]=WelsI8x8LumaPredVR_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_HU]=WelsI8x8LumaPredHU_c;
	pCtx->pGetI8x8LumaPredFunc[I4_PRED_HD]=WelsI8x8LumaPredHD_c;

	pCtx->pGetIChromaPredFunc[C_PRED_DC]=WelsIChromaPredDc_c;
	pCtx->pGetIChromaPredFunc[C_PRED_H]=WelsIChromaPredH_c;
	pCtx->pGetIChromaPredFunc[C_PRED_V]=WelsIChromaPredV_c;
	pCtx->pGetIChromaPredFunc[C_PRED_P]=WelsIChromaPredPlane_c;
	pCtx->pGetIChromaPredFunc[C_PRED_DC_L]=WelsIChromaPredDcLeft_c;
	pCtx->pGetIChromaPredFunc[C_PRED_DC_T]=WelsIChromaPredDcTop_c;
	pCtx->pGetIChromaPredFunc[C_PRED_DC_128]=WelsIChromaPredDcNA_c;

	pCtx->pIdctResAddPredFunc=IdctResAddPred_c;
	pCtx->pIdctFourResAddPredFunc=IdctFourResAddPred_<IdctResAddPred_c>;

	pCtx->pIdctResAddPredFunc8x8=IdctResAddPred8x8_c;

}

// h: iOffset=1 / v: iOffset=iSrcStride
static inline int32_t FilterInput8bitWithStride_c(const uint8_t* pSrc,const int32_t kiOffset){
	const int32_t kiOffset1=kiOffset;
	const int32_t kiOffset2=(kiOffset<<1);
	const int32_t kiOffset3=kiOffset+kiOffset2;
	const uint32_t kuiPix05=*(pSrc-kiOffset2)+*(pSrc+kiOffset3);
	const uint32_t kuiPix14=*(pSrc-kiOffset1)+*(pSrc+kiOffset2);
	const uint32_t kuiPix23=*(pSrc)+*(pSrc+kiOffset1);
	return (kuiPix05-((kuiPix14<<2)+kuiPix14)+(kuiPix23<<4)+(kuiPix23<<2));
}

// horizontal filter to gain half sample,that is (2,0) location in quarter sample
static inline void McHorVer20_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	int32_t i,j;
	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth; j++){
			pDst[j]=WelsClip1((FilterInput8bitWithStride_c(pSrc+j,1)+16)>>5);
		}
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

// vertical filter to gain half sample,that is (0,2) location in quarter sample
static inline void McHorVer02_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	int32_t i,j;
	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth; j++){
			pDst[j]=WelsClip1((FilterInput8bitWithStride_c(pSrc+j,iSrcStride)+16)>>5);
		}
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void PixelAvg_c(uint8_t* pDst,int32_t iDstStride,const uint8_t* pSrcA,int32_t iSrcAStride,const uint8_t* pSrcB,int32_t iSrcBStride,int32_t iWidth,int32_t iHeight){
	int32_t i,j;
	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth; j++){
			pDst[j]=(pSrcA[j]+pSrcB[j]+1)>>1;
		}
		pDst+=iDstStride;
		pSrcA+=iSrcAStride;
		pSrcB+=iSrcBStride;
	}
}

static inline int32_t HorFilterInput16bit_c(const int16_t* pSrc){
	int32_t iPix05=pSrc[0]+pSrc[5];
	int32_t iPix14=pSrc[1]+pSrc[4];
	int32_t iPix23=pSrc[2]+pSrc[3];
	return (iPix05-(iPix14*5)+(iPix23*20));
}

// horizontal and vertical filter to gain half sample,that is (2,2) location in quarter sample
static inline void McHorVer22_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	int16_t iTmp[17+5];
	int32_t i,j,k;

	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth+5; j++){
			iTmp[j]=FilterInput8bitWithStride_c(pSrc-2+j,iSrcStride);
		}
		for(k=0; k<iWidth; k++){
			pDst[k]=WelsClip1((HorFilterInput16bit_c(&iTmp[k])+512)>>10);
		}
		pSrc+=iSrcStride;
		pDst+=iDstStride;
	}
}

// iA=(8-dx) * (8-dy);
// iB=dx * (8-dy);
// iC=(8-dx) * dy;
// iD=dx * dy
static const uint8_t g_kuiABCD[8][8][4]={		// g_kA[dy][dx],g_kB[dy][dx],g_kC[dy][dx],g_kD[dy][dx]
	{
		{64,0,0,0},{56,8,0,0},{48,16,0,0},{40,24,0,0},
	{32,32,0,0},{24,40,0,0},{16,48,0,0},{8,56,0,0}
	},
 {
	 {56,0,8,0},{49,7,7,1},{42,14,6,2},{35,21,5,3},
	{28,28,4,4},{21,35,3,5},{14,42,2,6},{7,49,1,7}
 },
 {
	 {48,0,16,0},{42,6,14,2},{36,12,12,4},{30,18,10,6},
	{24,24,8,8},{18,30,6,10},{12,36,4,12},{6,42,2,14}
 },
 {
	 {40,0,24,0},{35,5,21,3},{30,10,18,6},{25,15,15,9},
	{20,20,12,12},{15,25,9,15},{10,30,6,18},{5,35,3,21}
 },
 {
	 {32,0,32,0},{28,4,28,4},{24,8,24,8},{20,12,20,12},
	{16,16,16,16},{12,20,12,20},{8,24,8,24},{4,28,4,28}
 },
 {
	 {24,0,40,0},{21,3,35,5},{18,6,30,10},{15,9,25,15},
	{12,12,20,20},{9,15,15,25},{6,18,10,30},{3,21,5,35}
 },
 {
	 {16,0,48,0},{14,2,42,6},{12,4,36,12},{10,6,30,18},
	{8,8,24,24},{6,10,18,30},{4,12,12,36},{2,14,6,42}
 },
 {
	 {8,0,56,0},{7,1,49,7},{6,2,42,14},{5,3,35,21},
	{4,4,28,28},{3,5,21,35},{2,6,14,42},{1,7,7,49}
 }
};

static inline void McChromaWithFragMv_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight){
	int32_t i,j;
	int32_t iA,iB,iC,iD;
	const uint8_t* pSrcNext=pSrc+iSrcStride;
	const uint8_t* pABCD=g_kuiABCD[iMvY&0x07][iMvX&0x07];
	iA=pABCD[0];
	iB=pABCD[1];
	iC=pABCD[2];
	iD=pABCD[3];
	for(i=0; i<iHeight; i++){
		for(j=0; j<iWidth; j++){
			pDst[j]=(iA*pSrc[j]+iB*pSrc[j+1]+iC*pSrcNext[j]+iD*pSrcNext[j+1]+32)>>6;
		}
		pDst+=iDstStride;
		pSrc=pSrcNext;
		pSrcNext+=iSrcStride;
	}
}

static inline void McCopyWidthEq16_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight){
	int32_t i;
	for(i=0; i<iHeight; i++){
		ST64A8(pDst,LD64(pSrc));
		ST64A8(pDst+8,LD64(pSrc+8));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopyWidthEq8_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight){
	int32_t i;
	for(i=0; i<iHeight; i++){
		ST64A8(pDst,LD64(pSrc));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopyWidthEq4_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight){
	int32_t i;
	for(i=0; i<iHeight; i++){
		ST32A4(pDst,LD32(pSrc));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}

static inline void McCopyWidthEq2_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iHeight){
	int32_t i;
	for(i=0; i<iHeight; i++){		// iWidth==2 only for chroma
		ST16A2(pDst,LD16(pSrc));
		pDst+=iDstStride;
		pSrc+=iSrcStride;
	}
}


static inline void McCopy_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	if(iWidth==16)
		McCopyWidthEq16_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
	else if(iWidth==8)
		McCopyWidthEq8_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
	else if(iWidth==4)
		McCopyWidthEq4_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
	else		// here iWidth==2
		McCopyWidthEq2_c(pSrc,iSrcStride,pDst,iDstStride,iHeight);
}


void McChroma_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight)
	// pSrc has been added the offset of mv
{
	const int32_t kiD8x=iMvX&0x07;
	const int32_t kiD8y=iMvY&0x07;
	if(0==kiD8x && 0==kiD8y)
		McCopy_c(pSrc,iSrcStride,pDst,iDstStride,iWidth,iHeight);
	else
		McChromaWithFragMv_c(pSrc,iSrcStride,pDst,iDstStride,iMvX,iMvY,iWidth,iHeight);
}

typedef void (*PWelsMcWidthHeightFunc) (const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight);

//luma MC
static inline void McHorVer01_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiTmp[256];
	McHorVer02_c(pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
}
static inline void McHorVer03_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiTmp[256];
	McHorVer02_c(pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc+iSrcStride,iSrcStride,uiTmp,16,iWidth,iHeight);
}
static inline void McHorVer10_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc,iSrcStride,uiTmp,16,iWidth,iHeight);
}
static inline void McHorVer11_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}
static inline void McHorVer12_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiVerTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer02_c(pSrc,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiVerTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer13_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc+iSrcStride,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}
static inline void McHorVer21_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer23_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer20_c(pSrc+iSrcStride,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer30_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,pSrc+1,iSrcStride,uiHorTmp,16,iWidth,iHeight);
}
static inline void McHorVer31_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc+1,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}
static inline void McHorVer32_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiVerTmp[256];
	uint8_t uiCtrTmp[256];
	McHorVer02_c(pSrc+1,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	McHorVer22_c(pSrc,iSrcStride,uiCtrTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiVerTmp,16,uiCtrTmp,16,iWidth,iHeight);
}
static inline void McHorVer33_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int32_t iWidth,int32_t iHeight){
	uint8_t uiHorTmp[256];
	uint8_t uiVerTmp[256];
	McHorVer20_c(pSrc+iSrcStride,iSrcStride,uiHorTmp,16,iWidth,iHeight);
	McHorVer02_c(pSrc+1,iSrcStride,uiVerTmp,16,iWidth,iHeight);
	PixelAvg_c(pDst,iDstStride,uiHorTmp,16,uiVerTmp,16,iWidth,iHeight);
}

void McLuma_c(const uint8_t* pSrc,int32_t iSrcStride,uint8_t* pDst,int32_t iDstStride,int16_t iMvX,int16_t iMvY,int32_t iWidth,int32_t iHeight)
	// pSrc has been added the offset of mv
{
	static const PWelsMcWidthHeightFunc pWelsMcFunc[4][4]={		// [x][y]
		{McCopy_c,McHorVer01_c,McHorVer02_c,McHorVer03_c},
	 {McHorVer10_c,McHorVer11_c,McHorVer12_c,McHorVer13_c},
	 {McHorVer20_c,McHorVer21_c,McHorVer22_c,McHorVer23_c},
	 {McHorVer30_c,McHorVer31_c,McHorVer32_c,McHorVer33_c},
	};

	pWelsMcFunc[iMvX&0x03][iMvY&0x03](pSrc,iSrcStride,pDst,iDstStride,iWidth,iHeight);
}

void InitMcFunc(SMcFunc* pMcFuncs,uint32_t uiCpuFlag){
	pMcFuncs->pfLumaHalfpelHor=McHorVer20_c;
	pMcFuncs->pfLumaHalfpelVer=McHorVer02_c;
	pMcFuncs->pfLumaHalfpelCen=McHorVer22_c;
	pMcFuncs->pfSampleAveraging=PixelAvg_c;
	pMcFuncs->pMcChromaFunc=McChroma_c;
	pMcFuncs->pMcLumaFunc=McLuma_c;
}

// rewrite it (split into luma & chroma) that is helpful for mmx/sse2 optimization perform,9/27/2009
static inline void ExpandPictureLuma_c(uint8_t* pDst,const int32_t kiStride,const int32_t kiPicW,const int32_t kiPicH){
	uint8_t* pTmp=pDst;
	uint8_t* pDstLastLine=pTmp+(kiPicH-1)*kiStride;
	const int32_t kiPaddingLen=PADDING_LENGTH;
	const uint8_t kuiTL=pTmp[0];
	const uint8_t kuiTR=pTmp[kiPicW-1];
	const uint8_t kuiBL=pDstLastLine[0];
	const uint8_t kuiBR=pDstLastLine[kiPicW-1];
	int32_t i=0;

	do{
		const int32_t kiStrides=(1+i)*kiStride;
		uint8_t* pTop=pTmp-kiStrides;
		uint8_t* pBottom=pDstLastLine+kiStrides;

		// pad pTop and pBottom
		memcpy(pTop,pTmp,kiPicW); 	// confirmed_safe_unsafe_usage
		memcpy(pBottom,pDstLastLine,kiPicW);

		// pad corners
		memset(pTop-kiPaddingLen,kuiTL,kiPaddingLen);		// pTop left
		memset(pTop+kiPicW,kuiTR,kiPaddingLen);		// pTop right
		memset(pBottom-kiPaddingLen,kuiBL,kiPaddingLen);		// pBottom left
		memset(pBottom+kiPicW,kuiBR,kiPaddingLen);		// pBottom right

		++i;
	} while(i<kiPaddingLen);

	// pad left and right
	i=0;
	do{
		memset(pTmp-kiPaddingLen,pTmp[0],kiPaddingLen);
		memset(pTmp+kiPicW,pTmp[kiPicW-1],kiPaddingLen);

		pTmp+=kiStride;
		++i;
	} while(i<kiPicH);
}


void InitExpandPictureFunc(SExpandPicFunc* pExpandPicFunc,const uint32_t kuiCPUFlag){
	pExpandPicFunc->pfExpandLumaPicture=ExpandPictureLuma_c;
	pExpandPicFunc->pfExpandChromaPicture[0]=ExpandPictureChroma_c;
	pExpandPicFunc->pfExpandChromaPicture[1]=ExpandPictureChroma_c;
}


// C code only
void DeblockLumaLt4_c(uint8_t* pPix,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta,int8_t* pTc){
	for(int32_t i=0; i<16; i++){
		int32_t iTc0=pTc[i>>2];
		if(iTc0>=0){
			int32_t p0=pPix[-iStrideX];
			int32_t p1=pPix[-2*iStrideX];
			int32_t p2=pPix[-3*iStrideX];
			int32_t q0=pPix[0];
			int32_t q1=pPix[iStrideX];
			int32_t q2=pPix[2*iStrideX];
			bool bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
			bool bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
			bool bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
			int32_t iTc=iTc0;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
				bool bDetaP2P0=WELS_ABS(p2-p0)<iBeta;
				bool bDetaQ2Q0=WELS_ABS(q2-q0)<iBeta;
				if(bDetaP2P0){
					pPix[-2*iStrideX]=p1+WELS_CLIP3((p2+((p0+q0+1)>>1)-(p1*(1<<1)))>>1,-iTc0,iTc0);
					iTc++;
				}
				if(bDetaQ2Q0){
					pPix[iStrideX]=q1+WELS_CLIP3((q2+((p0+q0+1)>>1)-(q1*(1<<1)))>>1,-iTc0,iTc0);
					iTc++;
				}
				int32_t iDeta=WELS_CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc,iTc);
				pPix[-iStrideX]=WelsClip1(p0+iDeta);
				pPix[0]=WelsClip1(q0-iDeta);
			}
		}
		pPix+=iStrideY;
	}
}
void DeblockLumaEq4_c(uint8_t* pPix,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta){
	int32_t p0,p1,p2,q0,q1,q2;
	int32_t iDetaP0Q0;
	bool bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0; i<16; i++){
		p0=pPix[-iStrideX];
		p1=pPix[-2*iStrideX];
		p2=pPix[-3*iStrideX];
		q0=pPix[0];
		q1=pPix[iStrideX];
		q2=pPix[2*iStrideX];
		iDetaP0Q0=WELS_ABS(p0-q0);
		bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
		bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
		if((iDetaP0Q0<iAlpha) && bDetaP1P0 && bDetaQ1Q0){
			if(iDetaP0Q0<((iAlpha>>2)+2)){
				bool bDetaP2P0=WELS_ABS(p2-p0)<iBeta;
				bool bDetaQ2Q0=WELS_ABS(q2-q0)<iBeta;
				if(bDetaP2P0){
					const int32_t p3=pPix[-4*iStrideX];
					pPix[-iStrideX]=(p2+(p1*(1<<1))+(p0*(1<<1))+(q0*(1<<1))+q1+4)>>3;		// p0
					pPix[-2*iStrideX]=(p2+p1+p0+q0+2)>>2; 	// p1
					pPix[-3*iStrideX]=((p3*(1<<1))+p2+(p2*(1<<1))+p1+p0+q0+4)>>3;		// p2
				}else{
					pPix[-1*iStrideX]=((p1*(1<<1))+p0+q1+2)>>2; 	// p0
				}
				if(bDetaQ2Q0){
					const int32_t q3=pPix[3*iStrideX];
					pPix[0]=(p1+(p0*(1<<1))+(q0*(1<<1))+(q1*(1<<1))+q2+4)>>3;		// q0
					pPix[iStrideX]=(p0+q0+q1+q2+2)>>2; 	// q1
					pPix[2*iStrideX]=((q3*(1<<1))+q2+(q2*(1<<1))+q1+q0+p0+4)>>3;		// q2
				}else{
					pPix[0]=((q1*(1<<1))+q0+p1+2)>>2; 	// q0
				}
			}else{
				pPix[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;		// p0
				pPix[0]=((q1*(1<<1))+q0+p1+2)>>2;		// q0
			}
		}
		pPix+=iStrideY;
	}
}

void DeblockLumaLt4V_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockLumaLt4_c(pPix,iStride,1,iAlpha,iBeta,tc);
}
void DeblockLumaLt4H_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockLumaLt4_c(pPix,1,iStride,iAlpha,iBeta,tc);
}
void DeblockLumaEq4V_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockLumaEq4_c(pPix,iStride,1,iAlpha,iBeta);
}
void DeblockLumaEq4H_c(uint8_t* pPix,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockLumaEq4_c(pPix,1,iStride,iAlpha,iBeta);
}
void DeblockChromaLt4_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta,int8_t* pTc){
	int32_t p0,p1,q0,q1,iDeta;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;

	for(int32_t i=0; i<8; i++){
		int32_t iTc0=pTc[i>>1];
		if(iTc0>0){
			p0=pPixCb[-iStrideX];
			p1=pPixCb[-2*iStrideX];
			q0=pPixCb[0];
			q1=pPixCb[iStrideX];

			bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
			bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
			bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
				iDeta=WELS_CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc0,iTc0);
				pPixCb[-iStrideX]=WelsClip1(p0+iDeta);
				pPixCb[0]=WelsClip1(q0-iDeta);
			}


			p0=pPixCr[-iStrideX];
			p1=pPixCr[-2*iStrideX];
			q0=pPixCr[0];
			q1=pPixCr[iStrideX];

			bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
			bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
			bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;

			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
				iDeta=WELS_CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc0,iTc0);
				pPixCr[-iStrideX]=WelsClip1(p0+iDeta);
				pPixCr[0]=WelsClip1(q0-iDeta);
			}
		}
		pPixCb+=iStrideY;
		pPixCr+=iStrideY;
	}
}
void DeblockChromaEq4_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta){
	int32_t p0,p1,q0,q1;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0; i<8; i++){
		// cb
		p0=pPixCb[-iStrideX];
		p1=pPixCb[-2*iStrideX];
		q0=pPixCb[0];
		q1=pPixCb[iStrideX];
		bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
		bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
		bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
		if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
			pPixCb[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;
			pPixCb[0]=((q1*(1<<1))+q0+p1+2)>>2;
		}

		// cr
		p0=pPixCr[-iStrideX];
		p1=pPixCr[-2*iStrideX];
		q0=pPixCr[0];
		q1=pPixCr[iStrideX];
		bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
		bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
		bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
		if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
			pPixCr[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;
			pPixCr[0]=((q1*(1<<1))+q0+p1+2)>>2;
		}
		pPixCr+=iStrideY;
		pPixCb+=iStrideY;
	}
}
void DeblockChromaLt4V_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockChromaLt4_c(pPixCb,pPixCr,iStride,1,iAlpha,iBeta,tc);
}
void DeblockChromaLt4H_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockChromaLt4_c(pPixCb,pPixCr,1,iStride,iAlpha,iBeta,tc);
}
void DeblockChromaEq4V_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockChromaEq4_c(pPixCb,pPixCr,iStride,1,iAlpha,iBeta);
}
void DeblockChromaEq4H_c(uint8_t* pPixCb,uint8_t* pPixCr,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockChromaEq4_c(pPixCb,pPixCr,1,iStride,iAlpha,iBeta);
}

void DeblockChromaLt42_c(uint8_t* pPixCbCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta,int8_t* pTc){
	int32_t p0,p1,q0,q1,iDeta;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;

	for(int32_t i=0; i<8; i++){
		int32_t iTc0=pTc[i>>1];
		if(iTc0>0){
			p0=pPixCbCr[-iStrideX];
			p1=pPixCbCr[-2*iStrideX];
			q0=pPixCbCr[0];
			q1=pPixCbCr[iStrideX];

			bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
			bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
			bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
			if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
				iDeta=WELS_CLIP3((((q0-p0)*(1<<2))+(p1-q1)+4)>>3,-iTc0,iTc0);
				pPixCbCr[-iStrideX]=WelsClip1(p0+iDeta);
				pPixCbCr[0]=WelsClip1(q0-iDeta);
			}


		}
		pPixCbCr+=iStrideY;
	}
}
void DeblockChromaEq42_c(uint8_t* pPixCbCr,int32_t iStrideX,int32_t iStrideY,int32_t iAlpha,int32_t iBeta){
	int32_t p0,p1,q0,q1;
	bool bDetaP0Q0,bDetaP1P0,bDetaQ1Q0;
	for(int32_t i=0; i<8; i++){
		p0=pPixCbCr[-iStrideX];
		p1=pPixCbCr[-2*iStrideX];
		q0=pPixCbCr[0];
		q1=pPixCbCr[iStrideX];
		bDetaP0Q0=WELS_ABS(p0-q0)<iAlpha;
		bDetaP1P0=WELS_ABS(p1-p0)<iBeta;
		bDetaQ1Q0=WELS_ABS(q1-q0)<iBeta;
		if(bDetaP0Q0 && bDetaP1P0 && bDetaQ1Q0){
			pPixCbCr[-iStrideX]=((p1*(1<<1))+p0+q1+2)>>2;
			pPixCbCr[0]=((q1*(1<<1))+q0+p1+2)>>2;
		}

		pPixCbCr+=iStrideY;
	}
}

void DeblockChromaLt4V2_c(uint8_t* pPixCbCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockChromaLt42_c(pPixCbCr,iStride,1,iAlpha,iBeta,tc);
}
void DeblockChromaLt4H2_c(uint8_t* pPixCbCr,int32_t iStride,int32_t iAlpha,int32_t iBeta,int8_t* tc){
	DeblockChromaLt42_c(pPixCbCr,1,iStride,iAlpha,iBeta,tc);
}
void DeblockChromaEq4V2_c(uint8_t* pPixCbCr,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockChromaEq42_c(pPixCbCr,iStride,1,iAlpha,iBeta);
}
void DeblockChromaEq4H2_c(uint8_t* pPixCbCr,int32_t iStride,int32_t iAlpha,int32_t iBeta){
	DeblockChromaEq42_c(pPixCbCr,1,iStride,iAlpha,iBeta);
}

// brief deblocking module initialize
// param pf
// cpu
void DeblockingInit(SDeblockingFunc* pFunc,int32_t iCpu){
	pFunc->pfLumaDeblockingLT4Ver=DeblockLumaLt4V_c;
	pFunc->pfLumaDeblockingEQ4Ver=DeblockLumaEq4V_c;
	pFunc->pfLumaDeblockingLT4Hor=DeblockLumaLt4H_c;
	pFunc->pfLumaDeblockingEQ4Hor=DeblockLumaEq4H_c;

	pFunc->pfChromaDeblockingLT4Ver=DeblockChromaLt4V_c;
	pFunc->pfChromaDeblockingEQ4Ver=DeblockChromaEq4V_c;
	pFunc->pfChromaDeblockingLT4Hor=DeblockChromaLt4H_c;
	pFunc->pfChromaDeblockingEQ4Hor=DeblockChromaEq4H_c;

	pFunc->pfChromaDeblockingLT4Ver2=DeblockChromaLt4V2_c;
	pFunc->pfChromaDeblockingEQ4Ver2=DeblockChromaEq4V2_c;
	pFunc->pfChromaDeblockingLT4Hor2=DeblockChromaLt4H2_c;
	pFunc->pfChromaDeblockingEQ4Hor2=DeblockChromaEq4H2_c;

}

void InitDecFuncs(SDecoderContext* pCtx,uint32_t uiCpuFlag){
	WelsBlockFuncInit(&pCtx->sBlockFunc,uiCpuFlag);
	InitPredFunc(pCtx,uiCpuFlag);
	InitMcFunc(&(pCtx->sMcFunc),uiCpuFlag);
	InitExpandPictureFunc(&(pCtx->sExpandPicFunc),uiCpuFlag);
	DeblockingInit(&pCtx->sDeblockingFunc,uiCpuFlag);
}

const uint8_t g_kuiVlcChromaTable[256][2]={
	{13,7},{13,7},{12,8},{11,8},{8,7},{8,7},{7,7},{7,7},{10,6},{10,6},{10,6},{10,6},{6,6},{6,6},{6,6},{6,6},// 15
 {3,6},{3,6},{3,6},{3,6},{9,6},{9,6},{9,6},{9,6},{4,6},{4,6},{4,6},{4,6},{1,6},{1,6},{1,6},{1,6},// 31
 {5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},// 47
 {5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},// 63
 {0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},// 79
 {0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},// 95
 {0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},// 111
 {0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},// 127
 {2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},// 143
 {2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},// 159
 {2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},// 175
 {2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},// 191
 {2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},// 207
 {2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},// 223
 {2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},// 239
 {2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1},{2,1}		// 255
};

const uint8_t g_kuiVlcTable_0[256][2]
={		// [0] means the index of vlc table,[1] means the length of vlc code [256] value means the value of 8bits
	{0,0},{0,0},{0,0},{0,0},{21,8},{12,8},{7,8},{3,8},{17,7},{17,7},{8,7},{8,7},{13,6},{13,6},{13,6},{13,6},// 15
 {4,6},{4,6},{4,6},{4,6},{1,6},{1,6},{1,6},{1,6},{9,5},{9,5},{9,5},{9,5},{9,5},{9,5},{9,5},{9,5},// 31
 {5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},// 47
 {5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},// 63
 {2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},// 79
 {2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},// 95
 {2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},// 111
 {2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},// 127
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 143
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 159
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 175
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 191
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 207
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 223
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 239
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}		// 255
};

const uint8_t g_kuiVlcTable_1[256][2]={		// checked no error--
	{0,0},{0,0},{0,0},{0,0},{14,8},{20,8},{19,8},{10,8},{29,7},{29,7},{16,7},{16,7},{15,7},{15,7},{6,7},{6,7},// 15
 {25,6},{25,6},{25,6},{25,6},{12,6},{12,6},{12,6},{12,6},{11,6},{11,6},{11,6},{11,6},{3,6},{3,6},{3,6},{3,6},// 31
 {21,6},{21,6},{21,6},{21,6},{8,6},{8,6},{8,6},{8,6},{7,6},{7,6},{7,6},{7,6},{1,6},{1,6},{1,6},{1,6},// 47
 {17,5},{17,5},{17,5},{17,5},{17,5},{17,5},{17,5},{17,5},{4,5},{4,5},{4,5},{4,5},{4,5},{4,5},{4,5},{4,5},// 63
 {13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},// 79
 {9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},// 95
 {5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},// 111
 {5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},// 127
 {2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},// 143
 {2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},// 159
 {2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},// 175
 {2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},{2,2},// 191
 {0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},// 207
 {0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},// 223
 {0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},// 239
 {0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2},{0,2}		// 255

};

const uint8_t g_kuiVlcTable_2[256][2]={		// checked no error--
	{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{0,0},{45,8},{40,8},{35,8},{30,8},{41,8},{36,8},{31,8},{26,8},// 15
 {22,7},{22,7},{18,7},{18,7},{32,7},{32,7},{14,7},{14,7},{37,7},{37,7},{28,7},{28,7},{27,7},{27,7},{10,7},{10,7},// 31
 {6,6},{6,6},{6,6},{6,6},{24,6},{24,6},{24,6},{24,6},{23,6},{23,6},{23,6},{23,6},{3,6},{3,6},{3,6},{3,6},// 47
 {33,6},{33,6},{33,6},{33,6},{20,6},{20,6},{20,6},{20,6},{19,6},{19,6},{19,6},{19,6},{1,6},{1,6},{1,6},{1,6},// 63
 {15,5},{15,5},{15,5},{15,5},{15,5},{15,5},{15,5},{15,5},{16,5},{16,5},{16,5},{16,5},{16,5},{16,5},{16,5},{16,5},// 79
 {11,5},{11,5},{11,5},{11,5},{11,5},{11,5},{11,5},{11,5},{12,5},{12,5},{12,5},{12,5},{12,5},{12,5},{12,5},{12,5},// 95
 {7,5},{7,5},{7,5},{7,5},{7,5},{7,5},{7,5},{7,5},{29,5},{29,5},{29,5},{29,5},{29,5},{29,5},{29,5},{29,5},// 111
 {8,5},{8,5},{8,5},{8,5},{8,5},{8,5},{8,5},{8,5},{4,5},{4,5},{4,5},{4,5},{4,5},{4,5},{4,5},{4,5},// 127
 {25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},{25,4},// 143
 {21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},{21,4},// 159
 {17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},{17,4},// 175
 {13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},{13,4},// 191
 {9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},{9,4},// 207
 {5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},{5,4},// 223
 {2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},{2,4},// 239
 {0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4},{0,4}		// 255
};

const uint8_t g_kuiVlcTable_3[64][2]={		// read 6 bits		// corrected
	{1,6},{2,6},{0,0},{0,6},{3,6},{4,6},{5,6},{0,0},{6,6},{7,6},{8,6},{9,6},{10,6},{11,6},{12,6},{13,6},// 15
 {14,6},{15,6},{16,6},{17,6},{18,6},{19,6},{20,6},{21,6},{22,6},{23,6},{24,6},{25,6},{26,6},{27,6},{28,6},{29,6},// 31
 {30,6},{31,6},{32,6},{33,6},{34,6},{35,6},{36,6},{37,6},{38,6},{39,6},{40,6},{41,6},{42,6},{43,6},{44,6},{45,6},// 47
 {46,6},{47,6},{48,6},{49,6},{50,6},{51,6},{52,6},{53,6},{54,6},{55,6},{56,6},{57,6},{58,6},{59,6},{60,6},{61,6},// 63
};

const uint8_t g_kuiVlcTable_0_0[256][2]={		// read 8 bits		// for g_kuiVlcTable_0[0]		// checked no error--
	{0,0},{0,0},{47,7},{47,7},{58,8},{60,8},{59,8},{54,8},{61,8},{56,8},{55,8},{50,8},{57,8},{52,8},{51,8},{46,8},// 15
 {53,7},{53,7},{48,7},{48,7},{43,7},{43,7},{42,7},{42,7},{49,7},{49,7},{44,7},{44,7},{39,7},{39,7},{38,7},{38,7},// 31
 {45,6},{45,6},{45,6},{45,6},{40,6},{40,6},{40,6},{40,6},{35,6},{35,6},{35,6},{35,6},{34,6},{34,6},{34,6},{34,6},// 47
 {41,6},{41,6},{41,6},{41,6},{36,6},{36,6},{36,6},{36,6},{31,6},{31,6},{31,6},{31,6},{30,6},{30,6},{30,6},{30,6},// 63
 {26,5},{26,5},{26,5},{26,5},{26,5},{26,5},{26,5},{26,5},{32,5},{32,5},{32,5},{32,5},{32,5},{32,5},{32,5},{32,5},// 79
 {27,5},{27,5},{27,5},{27,5},{27,5},{27,5},{27,5},{27,5},{22,5},{22,5},{22,5},{22,5},{22,5},{22,5},{22,5},{22,5},// 95
 {37,5},{37,5},{37,5},{37,5},{37,5},{37,5},{37,5},{37,5},{28,5},{28,5},{28,5},{28,5},{28,5},{28,5},{28,5},{28,5},// 111
 {23,5},{23,5},{23,5},{23,5},{23,5},{23,5},{23,5},{23,5},{18,5},{18,5},{18,5},{18,5},{18,5},{18,5},{18,5},{18,5},// 127
 {33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},// 143
 {33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},{33,3},// 159
 {24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},// 175
 {24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},{24,3},// 191
 {19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},// 207
 {19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},{19,3},// 223
 {14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},// 239
 {14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3},{14,3}		// 255
};

const uint8_t g_kuiVlcTable_0_1[4][2]={		// read 2 bits		// for g_kuiVlcTable_0[1]		// checked no error--
	{29,2},{20,2},{15,2},{10,2}
};

const uint8_t g_kuiVlcTable_0_2[2][2]={		// read 1 bit		// for g_kuiVlcTable_0[2]		// checked no error--
	{25,1},{16,1}
};

const uint8_t g_kuiVlcTable_0_3[2][2]={		// read 1 bit		// for g_kuiVlcTable_0[3]		// checked no error--
	{11,1},{6,1}
};

const uint8_t g_kuiVlcTable_1_0[64][2]={		// read 6 bits		// for g_kuiVlcTable_1[0]		// checked no error--
	{0,0},{0,0},{57,5},{57,5},{61,6},{60,6},{59,6},{58,6},{55,6},{54,6},{56,6},{51,6},{52,5},{52,5},{50,5},{50,5},// 15
 {53,5},{53,5},{48,5},{48,5},{47,5},{47,5},{46,5},{46,5},{49,5},{49,5},{44,5},{44,5},{43,5},{43,5},{42,5},{42,5},// 31
 {38,4},{38,4},{38,4},{38,4},{40,4},{40,4},{40,4},{40,4},{39,4},{39,4},{39,4},{39,4},{34,4},{34,4},{34,4},{34,4},// 47
 {45,4},{45,4},{45,4},{45,4},{36,4},{36,4},{36,4},{36,4},{35,4},{35,4},{35,4},{35,4},{30,4},{30,4},{30,4},{30,4}		// 63
};

const uint8_t g_kuiVlcTable_1_1[8][2]={		// read 3 bits		// for g_kuiVlcTable_1[1]		// checked no error--
	{41,3},{32,3},{31,3},{26,3},{37,3},{28,3},{27,3},{22,3}
};

const uint8_t g_kuiVlcTable_1_2[2][2]={		// read 1 bit		// for g_kuiVlcTable_1[2]		// checked no error--
	{33,1},{24,1}
};

const uint8_t g_kuiVlcTable_1_3[2][2]={		// read 1 bit		// for g_kuiVlcTable_1[3]		// checked no error--
	{23,1},{18,1}
};

const uint8_t g_kuiVlcTable_2_0[4][2]={		// read 2 bits		// for g_kuiVlcTable_2[0]		// checked
	{0,0},{58,2},{61,2},{60,2}
};

const uint8_t g_kuiVlcTable_2_1[4][2]={		// read 2 bits		// for g_kuiVlcTable_2[1]		// checked
	{59,2},{54,2},{57,2},{56,2}
};

const uint8_t g_kuiVlcTable_2_2[4][2]={		// read 2 bits		// for g_kuiVlcTable_2[2]		// checked
	{55,2},{50,2},{53,2},{52,2}
};

const uint8_t g_kuiVlcTable_2_3[4][2]={		// read 2 bits		// for g_kuiVlcTable_2[3]		// checked
	{51,2},{46,2},{47,1},{47,1}
};

const uint8_t g_kuiVlcTable_2_4[2][2]={		// read 1 bit		// for g_kuiVlcTable_2[4]		// checked
	{42,1},{48,1}
};

const uint8_t g_kuiVlcTable_2_5[2][2]={		// read 1 bit		// for g_kuiVlcTable_2[5]		// checked
	{43,1},{38,1}
};

const uint8_t g_kuiVlcTable_2_6[2][2]={		// read 1 bit		// for g_kuiVlcTable_2[6]		// checked no error--
	{49,1},{44,1}
};

const uint8_t g_kuiVlcTable_2_7[2][2]={		// read 1 bit		// for g_kuiVlcTable_2[7]		// checked no error--
	{39,1},{34,1}
};

const uint8_t g_kuiZeroLeftTable0[2][2]={		// read 1 bits
	{1,1},{0,1}
};

const uint8_t g_kuiZeroLeftTable1[4][2]={		// read 2 bits
	{2,2},{1,2},{0,1},{0,1}
};

const uint8_t g_kuiZeroLeftTable2[4][2]={		// read 2 bits
	{3,2},{2,2},{1,2},{0,2}
};

const uint8_t g_kuiZeroLeftTable3[8][2]={		// read 3 bits
	{4,3},{3,3},{2,2},{2,2},{1,2},{1,2},{0,2},{0,2}
};

const uint8_t g_kuiZeroLeftTable4[8][2]={		// read 3 bits
	{5,3},{4,3},{3,3},{2,3},{1,2},{1,2},{0,2},{0,2}
};

const uint8_t g_kuiZeroLeftTable5[8][2]={		// read 3 bits
	{1,3},{2,3},{4,3},{3,3},{6,3},{5,3},{0,2},{0,2}
};

const uint8_t g_kuiZeroLeftTable6[8][2]={		// read 3 bits
	{7,3},{6,3},{5,3},{4,3},{3,3},{2,3},{1,3},{0,3}
};

const uint8_t g_kuiTotalZerosTable0[512][2]={		// read 9 bits,generated by tzVlcIndex=1 in Table 9-7 in H.264/AVC standard
	{0,0},{15,9},{14,9},{13,9},{12,8},{12,8},{11,8},{11,8},{10,7},{10,7},{10,7},{10,7},{9,7},{9,7},{9,7},{9,7},// 15
 {8,6},{8,6},{8,6},{8,6},{8,6},{8,6},{8,6},{8,6},{7,6},{7,6},{7,6},{7,6},{7,6},{7,6},{7,6},{7,6},// 31
 {6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},{6,5},// 47
 {5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},{5,5},// 63
 {4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},// 79
 {4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},{4,4},// 95
 {3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},// 111
 {3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},{3,4},// 127
 {2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},// 143
 {2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},// 159
 {2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},// 175
 {2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},// 191
 {1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},// 207
 {1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},// 223
 {1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},// 239
 {1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},// 255
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 271
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 287
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 303
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 319
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 335
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 351
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 367
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 383
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 399
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 415
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 431
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 447
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 463
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 479
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},// 495
 {0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1},{0,1}		// 511
};

const uint8_t g_kuiTotalZerosTable1[64][2]={		// read 6 bits,generated by tzVlcIndex=2 in Table 9-7 in H.264/AVC standard
	{14,6},{13,6},{12,6},{11,6},{10,5},{10,5},{9,5},{9,5},{8,4},{8,4},{8,4},{8,4},{7,4},{7,4},{7,4},{7,4},// 15
 {6,4},{6,4},{6,4},{6,4},{5,4},{5,4},{5,4},{5,4},{4,3},{4,3},{4,3},{4,3},{4,3},{4,3},{4,3},{4,3},// 31
 {3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},// 47
 {1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{0,3},{0,3},{0,3},{0,3},{0,3},{0,3},{0,3},{0,3}		// 63
};

const uint8_t g_kuiTotalZerosTable2[64][2]={		// read 6 bits,generated by tzVlcIndex=3 in Table 9-7 in H.264/AVC standard
	{13,6},{11,6},{12,5},{12,5},{10,5},{10,5},{9,5},{9,5},{8,4},{8,4},{8,4},{8,4},{5,4},{5,4},{5,4},{5,4},// 15
 {4,4},{4,4},{4,4},{4,4},{0,4},{0,4},{0,4},{0,4},{7,3},{7,3},{7,3},{7,3},{7,3},{7,3},{7,3},{7,3},// 31
 {6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},// 47
 {2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3},{1,3}		// 63
};

const uint8_t g_kuiTotalZerosTable3[32][2]={		// read 5 bits,generated by tzVlcIndex=4 in Table 9-7 in H.264/AVC standard
	{12,5},{11,5},{10,5},{0,5},{9,4},{9,4},{7,4},{7,4},{3,4},{3,4},{2,4},{2,4},{8,3},{8,3},{8,3},{8,3},// 15
 {6,3},{6,3},{6,3},{6,3},{5,3},{5,3},{5,3},{5,3},{4,3},{4,3},{4,3},{4,3},{1,3},{1,3},{1,3},{1,3},// 31
};

const uint8_t g_kuiTotalZerosTable4[32][2]={		// read 5 bits,generated by tzVlcIndex=5 in Table 9-7 in H.264/AVC standard
	{11,5},{9,5},{10,4},{10,4},{8,4},{8,4},{2,4},{2,4},{1,4},{1,4},{0,4},{0,4},{7,3},{7,3},{7,3},{7,3},// 15
 {6,3},{6,3},{6,3},{6,3},{5,3},{5,3},{5,3},{5,3},{4,3},{4,3},{4,3},{4,3},{3,3},{3,3},{3,3},{3,3}		// 31
};

const uint8_t g_kuiTotalZerosTable5[64][2]={		// read 6 bits,generated by tzVlcIndex=6 in Table 9-7 in H.264/AVC standard
	{10,6},{0,6},{1,5},{1,5},{8,4},{8,4},{8,4},{8,4},{9,3},{9,3},{9,3},{9,3},{9,3},{9,3},{9,3},{9,3},// 15
 {7,3},{7,3},{7,3},{7,3},{7,3},{7,3},{7,3},{7,3},{6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{6,3},// 31
 {5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{4,3},{4,3},{4,3},{4,3},{4,3},{4,3},{4,3},{4,3},// 47
 {3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3}		// 63
};

const uint8_t g_kuiTotalZerosTable6[64][2]={		// read 6 bits,generated by tzVlcIndex=7 in Table 9-7 in H.264/AVC standard
	{9,6},{0,6},{1,5},{1,5},{7,4},{7,4},{7,4},{7,4},{8,3},{8,3},{8,3},{8,3},{8,3},{8,3},{8,3},{8,3},// 15
 {6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{4,3},{4,3},{4,3},{4,3},{4,3},{4,3},{4,3},{4,3},// 31
 {3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},{2,3},// 47
 {5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2}		// 63
};

const uint8_t g_kuiTotalZerosTable7[64][2]={		// read 6 bits,generated by tzVlcIndex=8 in Table 9-7 in H.264/AVC standard
	{8,6},{0,6},{2,5},{2,5},{1,4},{1,4},{1,4},{1,4},{7,3},{7,3},{7,3},{7,3},{7,3},{7,3},{7,3},{7,3},// 15
 {6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{6,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},{3,3},// 31
 {5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},// 47
 {4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2}		// 63
};

const uint8_t g_kuiTotalZerosTable8[64][2]={		// read 6 bits,generated by tzVlcIndex=9 in Table 9-7 in H.264/AVC standard
	{1,6},{0,6},{7,5},{7,5},{2,4},{2,4},{2,4},{2,4},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},{5,3},// 15
 {6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},{6,2},// 31
 {4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},// 47
 {3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2}		// 63
};

const uint8_t g_kuiTotalZerosTable9[32][2]={		// read 5 bits,generated by tzVlcIndex=10 in Table 9-7 in H.264/AVC standard
	{1,5},{0,5},{6,4},{6,4},{2,3},{2,3},{2,3},{2,3},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},{5,2},// 15
 {4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{4,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2},{3,2}		// 31
};

const uint8_t g_kuiTotalZerosTable10[16][2]={		// read 4 bits,generated by tzVlcIndex=11 in Table 9-7 in H.264/AVC standard
	{0,4},{1,4},{2,3},{2,3},{3,3},{3,3},{5,3},{5,3},{4,1},{4,1},{4,1},{4,1},{4,1},{4,1},{4,1},{4,1}		// 15
};

const uint8_t g_kuiTotalZerosTable11[16][2]={		// read 4 bits,generated by tzVlcIndex=12 in Table 9-7 in H.264/AVC standard
	{0,4},{1,4},{4,3},{4,3},{2,2},{2,2},{2,2},{2,2},{3,1},{3,1},{3,1},{3,1},{3,1},{3,1},{3,1},{3,1}		// 15
};

const uint8_t g_kuiTotalZerosTable12[8][2]={		// read 3 bits,generated by tzVlcIndex=13 in Table 9-7 in H.264/AVC standard
	{0,3},{1,3},{3,2},{3,2},{2,1},{2,1},{2,1},{2,1}		// 8
};

const uint8_t g_kuiTotalZerosTable13[4][2]={		// read 2 bits,generated by tzVlcIndex=14 in Table 9-7 in H.264/AVC standard
	{0,2},{1,2},{2,1},{2,1}
};

const uint8_t g_kuiTotalZerosTable14[2][2]={		// read 1 bits generated by tzVlcIndex=15 in Table 9-7 in H.264/AVC standard
	{0,1},{1,1}
};

const uint8_t g_kuiTotalZerosChromaTable0[8][2]={		// read 3 bits,generated by tzVlcIndex=1 in Table 9-9(a) in H.264/AVC standard
	{3,3},{2,3},{1,2},{1,2},{0,1},{0,1},{0,1},{0,1}
};

const uint8_t g_kuiTotalZerosChromaTable1[4][2]={		// read 2 bits,generated by tzVlcIndex=2 in Table 9-9(a) in H.264/AVC standard
	{2,2},{1,2},{0,1},{0,1}
};

const uint8_t g_kuiTotalZerosChromaTable2[2][2]={		// read 1 bits,generated by tzVlcIndex=3 in Table 9-9(a) in H.264/AVC standard
	{1,1},{0,1}
};

static inline void InitVlcTable(SVlcTable* pVlcTable){
	pVlcTable->kpChromaCoeffTokenVlcTable=g_kuiVlcChromaTable;

	pVlcTable->kpCoeffTokenVlcTable[0][0]=g_kuiVlcTable_0;
	pVlcTable->kpCoeffTokenVlcTable[0][1]=g_kuiVlcTable_1;
	pVlcTable->kpCoeffTokenVlcTable[0][2]=g_kuiVlcTable_2;
	pVlcTable->kpCoeffTokenVlcTable[0][3]=g_kuiVlcTable_3;

	pVlcTable->kpCoeffTokenVlcTable[1][0]=g_kuiVlcTable_0_0;
	pVlcTable->kpCoeffTokenVlcTable[1][1]=g_kuiVlcTable_0_1;
	pVlcTable->kpCoeffTokenVlcTable[1][2]=g_kuiVlcTable_0_2;
	pVlcTable->kpCoeffTokenVlcTable[1][3]=g_kuiVlcTable_0_3;

	pVlcTable->kpCoeffTokenVlcTable[2][0]=g_kuiVlcTable_1_0;
	pVlcTable->kpCoeffTokenVlcTable[2][1]=g_kuiVlcTable_1_1;
	pVlcTable->kpCoeffTokenVlcTable[2][2]=g_kuiVlcTable_1_2;
	pVlcTable->kpCoeffTokenVlcTable[2][3]=g_kuiVlcTable_1_3;

	pVlcTable->kpCoeffTokenVlcTable[3][0]=g_kuiVlcTable_2_0;
	pVlcTable->kpCoeffTokenVlcTable[3][1]=g_kuiVlcTable_2_1;
	pVlcTable->kpCoeffTokenVlcTable[3][2]=g_kuiVlcTable_2_2;
	pVlcTable->kpCoeffTokenVlcTable[3][3]=g_kuiVlcTable_2_3;
	pVlcTable->kpCoeffTokenVlcTable[3][4]=g_kuiVlcTable_2_4;
	pVlcTable->kpCoeffTokenVlcTable[3][5]=g_kuiVlcTable_2_5;
	pVlcTable->kpCoeffTokenVlcTable[3][6]=g_kuiVlcTable_2_6;
	pVlcTable->kpCoeffTokenVlcTable[3][7]=g_kuiVlcTable_2_7;

	pVlcTable->kpZeroTable[0]=g_kuiZeroLeftTable0;
	pVlcTable->kpZeroTable[1]=g_kuiZeroLeftTable1;
	pVlcTable->kpZeroTable[2]=g_kuiZeroLeftTable2;
	pVlcTable->kpZeroTable[3]=g_kuiZeroLeftTable3;
	pVlcTable->kpZeroTable[4]=g_kuiZeroLeftTable4;
	pVlcTable->kpZeroTable[5]=g_kuiZeroLeftTable5;
	pVlcTable->kpZeroTable[6]=g_kuiZeroLeftTable6;

	pVlcTable->kpTotalZerosTable[0][0]=g_kuiTotalZerosTable0;
	pVlcTable->kpTotalZerosTable[0][1]=g_kuiTotalZerosTable1;
	pVlcTable->kpTotalZerosTable[0][2]=g_kuiTotalZerosTable2;
	pVlcTable->kpTotalZerosTable[0][3]=g_kuiTotalZerosTable3;
	pVlcTable->kpTotalZerosTable[0][4]=g_kuiTotalZerosTable4;
	pVlcTable->kpTotalZerosTable[0][5]=g_kuiTotalZerosTable5;
	pVlcTable->kpTotalZerosTable[0][6]=g_kuiTotalZerosTable6;
	pVlcTable->kpTotalZerosTable[0][7]=g_kuiTotalZerosTable7;
	pVlcTable->kpTotalZerosTable[0][8]=g_kuiTotalZerosTable8;
	pVlcTable->kpTotalZerosTable[0][9]=g_kuiTotalZerosTable9;
	pVlcTable->kpTotalZerosTable[0][10]=g_kuiTotalZerosTable10;
	pVlcTable->kpTotalZerosTable[0][11]=g_kuiTotalZerosTable11;
	pVlcTable->kpTotalZerosTable[0][12]=g_kuiTotalZerosTable12;
	pVlcTable->kpTotalZerosTable[0][13]=g_kuiTotalZerosTable13;
	pVlcTable->kpTotalZerosTable[0][14]=g_kuiTotalZerosTable14;
	pVlcTable->kpTotalZerosTable[1][0]=g_kuiTotalZerosChromaTable0;
	pVlcTable->kpTotalZerosTable[1][1]=g_kuiTotalZerosChromaTable1;
	pVlcTable->kpTotalZerosTable[1][2]=g_kuiTotalZerosChromaTable2;

}

// brief Open decoder
int32_t WelsOpenDecoder(SDecoderContext* pCtx){
	int iRet=ERR_NONE;
	// function pointers
	InitDecFuncs(pCtx,pCtx->uiCpuFlag);

	// vlc tables
	InitVlcTable(pCtx->pVlcTable);

	// static memory
	iRet=WelsInitStaticMemory(pCtx);
	if(ERR_NONE!=iRet){
		pCtx->iErrorCode|=dsOutOfMemory;
		uprintf("WelsInitStaticMemory() failed in WelsOpenDecoder().");
		return iRet;
	}

	pCtx->bParamSetsLostFlag=true;
	pCtx->bNewSeqBegin=true;
	pCtx->bPrintFrameErrorTraceFlag=true;
	pCtx->iIgnoredErrorInfoPacketCount=0;
	return iRet;
}

// brief Initialize Wels decoder parameters and memory
// param pCtx input context to be initialized at first stage
// return 0-successed
// return 1-failed
int32_t WelsInitDecoder(SDecoderContext* pCtx){
	if(pCtx==NULL){
		return ERR_INFO_INVALID_PTR;
	}

	// open decoder
	return WelsOpenDecoder(pCtx);
}

// the return value of this function is not suitable,it need report failure info to upper layer.
int32_t PlainH264Decoder::InitDecoderCtx(const SDecodingParam* pParam){
	uprintf( "PlainH264Decoder::init_decoder(),openh264 codec version=%s",VERSION_NUMBER);
	// reset decoder context
	UninitDecoderCtx(m_pCtx);
	m_pCtx=(SDecoderContext*)WelsMallocz(sizeof(SDecoderContext));
	// fill in default value into context
	m_pCtx->pLastDecPicInfo=&m_sLastDecPicInfo;
	m_pCtx->pVlcTable=&m_sVlcTable;
	m_pCtx->pPictInfoList=m_sPictInfoList;
	m_pCtx->pPictReoderingStatus=&m_sReoderingStatus;
	WelsDecoderDefaults(m_pCtx);
	WelsDecoderSpsPpsDefaults(m_pCtx->sSpsPpsCtx);
	// check param and update decoder context
	m_pCtx->pParam=(SDecodingParam*)WelsMallocz(sizeof(SDecodingParam));
	WELS_VERIFY_RETURN_PROC_IF(cmMallocMemeError,(NULL==m_pCtx->pParam),UninitDecoderCtx(m_pCtx));
	int32_t iRet=DecoderConfigParam(m_pCtx,pParam);
	WELS_VERIFY_RETURN_IFNEQ(iRet,cmResultSuccess);
	// init decoder
	WELS_VERIFY_RETURN_PROC_IF(cmMallocMemeError,WelsInitDecoder(m_pCtx),UninitDecoderCtx(m_pCtx))
		m_pCtx->pPicBuff=NULL;
	return cmResultSuccess;
}


// Set Option
long PlainH264Decoder::SetOption(DECODER_OPTION eOptID,void* pOption){
	int iVal=0;
	if(eOptID==DECODER_OPTION_NUM_OF_THREADS){
		if(pOption!=NULL){
			FATAL("No threads");
		}
		return cmResultSuccess;
	}
	SDecoderContext* pDecContext=m_pCtx;
	if(pDecContext==NULL && eOptID!=DECODER_OPTION_TRACE_LEVEL && 
		eOptID!=DECODER_OPTION_TRACE_CALLBACK && eOptID!=DECODER_OPTION_TRACE_CALLBACK_CONTEXT)
		return dsInitialOptExpected;
	if(eOptID==DECODER_OPTION_END_OF_STREAM){		// Indicate bit-stream of the final frame to be decoded
		if(pOption==NULL)
			return cmInitParaError;

		iVal=*((int*)pOption);		// boolean value for whether enabled End Of Stream flag

		if(pDecContext==NULL) return dsInitialOptExpected;

		pDecContext->bEndOfStreamFlag=iVal ? true : false;

		return cmResultSuccess;
	}else
	if(eOptID==DECODER_OPTION_ERROR_CON_IDC){		// Indicate error concealment status
		if(pOption==NULL)
			return cmInitParaError;

		if(pDecContext==NULL) return dsInitialOptExpected;

		iVal=*((int*)pOption);		// int value for error concealment idc
		iVal=WELS_CLIP3(iVal,(int32_t)ERROR_CON_DISABLE,(int32_t)ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE);
		pDecContext->pParam->eEcActiveIdc=(ERROR_CON_IDC)iVal;
		InitErrorCon(pDecContext);
		uprintf("PlainH264Decoder::SetOption for ERROR_CON_IDC=%d.",iVal);

		return cmResultSuccess;
	}else
	if(eOptID==DECODER_OPTION_TRACE_LEVEL){
		//if(m_pWelsTrace){
		//	uint32_t level=*((uint32_t*)pOption);
		//	m_pWelsTrace->SetTraceLevel(level);
		//}
		return cmResultSuccess;
	}else
	if(eOptID==DECODER_OPTION_TRACE_CALLBACK){
		//if(m_pWelsTrace){
		//	WelsTraceCallback callback=*((WelsTraceCallback*)pOption);
		//	m_pWelsTrace->SetTraceCallback(callback);
		//	uprintf("PlainH264Decoder::SetOption():DECODER_OPTION_TRACE_CALLBACK callback=%p.",callback);
		//}
		return cmResultSuccess;
	}else
	if(eOptID==DECODER_OPTION_TRACE_CALLBACK_CONTEXT){
		//if(m_pWelsTrace){
		//	void* ctx=*((void**)pOption);
		//	m_pWelsTrace->SetTraceCallbackContext(ctx);
		//}
		return cmResultSuccess;
	}else
	if(eOptID==DECODER_OPTION_GET_STATISTICS){
		uprintf("PlainH264Decoder::SetOption():DECODER_OPTION_GET_STATISTICS: this option is get-only!");
		return cmInitParaError;
	}else
	if(eOptID==DECODER_OPTION_STATISTICS_LOG_INTERVAL){
		if(pOption){
			if(pDecContext==NULL) return dsInitialOptExpected;
			return cmResultSuccess;
		}
	}else
	if(eOptID==DECODER_OPTION_GET_SAR_INFO){
		uprintf("PlainH264Decoder::SetOption():DECODER_OPTION_GET_SAR_INFO: this option is get-only!");
		return cmInitParaError;
	}
	return cmInitParaError;
}

// rief Structure for sample aspect ratio (SAR) info in VUI
typedef struct TagVuiSarInfo{
	unsigned int uiSarWidth; 	// SAR width
	unsigned int uiSarHeight; 	// SAR height
	bool bOverscanAppropriateFlag;	// SAR overscan flag
} SVuiSarInfo,* PVuiSarInfo;
// Get Option
long PlainH264Decoder::GetOption(DECODER_OPTION eOptID,void* pOption){
	int iVal=0;
	if(DECODER_OPTION_NUM_OF_THREADS==eOptID){
		FATAL("NO THREADS!");
		return cmResultSuccess;
	}
	SDecoderContext* pDecContext=m_pCtx;
	if(pDecContext==NULL)
		return cmInitExpected;

	if(pOption==NULL)
		return cmInitParaError;

	if(DECODER_OPTION_END_OF_STREAM==eOptID){
		iVal=pDecContext->bEndOfStreamFlag;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_IDR_PIC_ID==eOptID){
		iVal=pDecContext->uiCurIdrPicId;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_FRAME_NUM==eOptID){
		iVal=pDecContext->iFrameNum;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_LTR_MARKING_FLAG==eOptID){
		iVal=pDecContext->bCurAuContainLtrMarkSeFlag;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_LTR_MARKED_FRAME_NUM==eOptID){
		iVal=pDecContext->iFrameNumOfAuMarkedLtr;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_VCL_NAL==eOptID){		// feedback whether or not have VCL NAL in current AU
		iVal=pDecContext->iFeedbackVclNalInAu;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_TEMPORAL_ID==eOptID){		// if have VCL NAL in current AU,then feedback the temporal ID
		iVal=pDecContext->iFeedbackTidInAu;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_IS_REF_PIC==eOptID){
		iVal=pDecContext->iFeedbackNalRefIdc;
		if(iVal>0)
			iVal=1;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_ERROR_CON_IDC==eOptID){
		iVal=(int)pDecContext->pParam->eEcActiveIdc;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_GET_STATISTICS==eOptID){		// get decoder statistics info for real time debugging
		FATAL("WTF");
	}else
	if(eOptID==DECODER_OPTION_STATISTICS_LOG_INTERVAL){
		if(pOption){
			FATAL("WTF");
		}
	}else
	if(DECODER_OPTION_GET_SAR_INFO==eOptID){		// get decoder SAR info in VUI
		PVuiSarInfo pVuiSarInfo=(static_cast<PVuiSarInfo> (pOption));
		memset(pVuiSarInfo,0,sizeof(SVuiSarInfo));
		if(!pDecContext->pSps){
			return cmInitExpected;
		}else{
			pVuiSarInfo->uiSarWidth=pDecContext->pSps->sVui.uiSarWidth;
			pVuiSarInfo->uiSarHeight=pDecContext->pSps->sVui.uiSarHeight;
			pVuiSarInfo->bOverscanAppropriateFlag=pDecContext->pSps->sVui.bOverscanAppropriateFlag;
			return cmResultSuccess;
		}
	}else
	if(DECODER_OPTION_PROFILE==eOptID){
		if(!pDecContext->pSps){
			return cmInitExpected;
		}
		iVal=(int)pDecContext->pSps->uiProfileIdc;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_LEVEL==eOptID){
		if(!pDecContext->pSps){
			return cmInitExpected;
		}
		iVal=(int)pDecContext->pSps->uiLevelIdc;
		*((int*)pOption)=iVal;
		return cmResultSuccess;
	}else
	if(DECODER_OPTION_NUM_OF_FRAMES_REMAINING_IN_BUFFER==eOptID){
		FATAL("Threads?");
		return cmResultSuccess;
	}

	return cmInitParaError;
}

// brief Start Code Prefix (0x 00 00 00 01) detection
// param pBuf bitstream payload buffer
// param pOffset offset between NAL rbsp and original bitsteam that
// start code prefix is seperated from.
// param iBufSize count size of buffer
// return RBSP buffer of start code prefix exclusive
uint8_t* DetectStartCodePrefix(const uint8_t* kpBuf,int32_t* pOffset,int32_t iBufSize){
	uint8_t* pBits=(uint8_t*)kpBuf;

	do{
		int32_t iIdx=0;
		while((iIdx<iBufSize) && (!(*pBits))){
			++pBits;
			++iIdx;
		}
		if(iIdx>=iBufSize) break;

		++iIdx;
		++pBits;

		if((iIdx>=3) && ((*(pBits-1))==0x1)){
			*pOffset=(int32_t)(((uintptr_t)pBits)-((uintptr_t)kpBuf));
			return pBits;
		}

		iBufSize-=iIdx;
	} while(1);

	return NULL;
}


// DecodeNalHeaderExt
// Trigger condition: NAL_UNIT_TYPE=NAL_UNIT_PREFIX or NAL_UNIT_CODED_SLICE_EXT
// Parameter:
// pNal: target NALUnit ptr
// pSrc: NAL Unit bitstream
void DecodeNalHeaderExt(SNalUnit* pNal,uint8_t* pSrc){
	SNalUnitHeaderExt* pHeaderExt=&pNal->sNalHeaderExt;

	uint8_t uiCurByte=*pSrc;
	pHeaderExt->bIdrFlag=!!(uiCurByte&0x40);
	pHeaderExt->uiPriorityId=uiCurByte&0x3F;

	uiCurByte=*(++pSrc);
	pHeaderExt->iNoInterLayerPredFlag=uiCurByte>>7;
	pHeaderExt->uiDependencyId=(uiCurByte&0x70)>>4;
	pHeaderExt->uiQualityId=uiCurByte&0x0F;
	uiCurByte=*(++pSrc);
	pHeaderExt->uiTemporalId=uiCurByte>>5;
	pHeaderExt->bUseRefBasePicFlag=!!(uiCurByte&0x10);
	pHeaderExt->bDiscardableFlag=!!(uiCurByte&0x08);
	pHeaderExt->bOutputFlag=!!(uiCurByte&0x04);
	pHeaderExt->uiReservedThree2Bits=uiCurByte&0x03;
	pHeaderExt->uiLayerDqId=(pHeaderExt->uiDependencyId<<4)|pHeaderExt->uiQualityId;
}



int32_t ParseRefBasePicMarking(SBitStringAux* pBs,PRefBasePicMarking pRefBasePicMarking){
	uint32_t uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// adaptive_ref_base_pic_marking_mode_flag
	const bool kbAdaptiveMarkingModeFlag=!!uiCode;
	pRefBasePicMarking->bAdaptiveRefBasePicMarkingModeFlag=kbAdaptiveMarkingModeFlag;
	if(kbAdaptiveMarkingModeFlag){
		int32_t iIdx=0;
		do{
			WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// MMCO_base
			const uint32_t kuiMmco=uiCode;

			pRefBasePicMarking->mmco_base[iIdx].uiMmcoType=kuiMmco;

			if(kuiMmco==MMCO_END)
				break;

			if(kuiMmco==MMCO_SHORT2UNUSED){
				WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// difference_of_base_pic_nums_minus1
				pRefBasePicMarking->mmco_base[iIdx].uiDiffOfPicNums=1+uiCode;
				pRefBasePicMarking->mmco_base[iIdx].iShortFrameNum=0;
			}else
			if(kuiMmco==MMCO_LONG2UNUSED){
				WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// long_term_base_pic_num
				pRefBasePicMarking->mmco_base[iIdx].uiLongTermPicNum=uiCode;
			}
			++iIdx;
		} while(iIdx<MAX_MMCO_COUNT);
	}
	return ERR_NONE;
}

int32_t ParsePrefixNalUnit(SDecoderContext* pCtx,SBitStringAux* pBs){
	SNalUnit* pCurNal=&pCtx->sSpsPpsCtx.sPrefixNal;
	uint32_t uiCode;

	if(pCurNal->sNalHeaderExt.sNalUnitHeader.uiNalRefIdc!=0){
		SNalUnitHeaderExt* head_ext=&pCurNal->sNalHeaderExt;
		PPrefixNalUnit sPrefixNal=&pCurNal->sNalData.sPrefixNal;
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// store_ref_base_pic_flag
		sPrefixNal->bStoreRefBasePicFlag=!!uiCode;
		if((head_ext->bUseRefBasePicFlag || sPrefixNal->bStoreRefBasePicFlag) && !head_ext->bIdrFlag){
			WELS_READ_VERIFY(ParseRefBasePicMarking(pBs,&sPrefixNal->sRefPicBaseMarking));
		}
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// additional_prefix_nal_unit_extension_flag
		sPrefixNal->bPrefixNalUnitAdditionalExtFlag=!!uiCode;
		if(sPrefixNal->bPrefixNalUnitAdditionalExtFlag){
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// additional_prefix_nal_unit_extension_data_flag
			sPrefixNal->bPrefixNalUnitExtFlag=!!uiCode;
		}
	}
	return ERR_NONE;
}

int32_t ExpandNalUnitList(SAccessUnit** ppAu,const int32_t kiOrgSize,const int32_t kiExpSize){
	if(kiExpSize<=kiOrgSize)
		return ERR_INFO_INVALID_PARAM;
	else{
		SAccessUnit* pTmp=NULL;
		int32_t iIdx=0;
		int32_t iRet=ERR_NONE;
		if((iRet=MemInitNalList(&pTmp,kiExpSize))!=ERR_NONE)		// request new list with expanding
			return iRet;

		do{
			memcpy(pTmp->pNalUnitsList[iIdx],(*ppAu)->pNalUnitsList[iIdx],sizeof(SNalUnit));
			++iIdx;
		} while(iIdx<kiOrgSize);

		pTmp->uiCountUnitsNum=kiExpSize;
		pTmp->uiAvailUnitsNum=(*ppAu)->uiAvailUnitsNum;
		pTmp->uiActualUnitsNum=(*ppAu)->uiActualUnitsNum;
		pTmp->uiEndPos=(*ppAu)->uiEndPos;
		pTmp->bCompletedAuFlag=(*ppAu)->bCompletedAuFlag;

		MemFreeNalList(ppAu);		// free old list
		*ppAu=pTmp;
		return ERR_NONE;
	}
}

// MemGetNextNal
// Get next NAL Unit for using.
// Need expand NAL Unit list if exceeding count number of available NAL Units withing an Access Unit
SNalUnit* MemGetNextNal(SAccessUnit** ppAu){
	SAccessUnit* pAu=*ppAu;
	SNalUnit* pNu=NULL;

	if(pAu->uiAvailUnitsNum>=pAu->uiCountUnitsNum){		// need expand list
		const uint32_t kuiExpandingSize=pAu->uiCountUnitsNum+(MAX_NAL_UNIT_NUM_IN_AU>>1);
		if(ExpandNalUnitList(ppAu,pAu->uiCountUnitsNum,kuiExpandingSize))
			return NULL;		// out of memory
		pAu=*ppAu;
	}

	pNu=pAu->pNalUnitsList[pAu->uiAvailUnitsNum++];		// ready for next nal position

	memset(pNu,0,sizeof(SNalUnit));		// Please do not remove this for cache intend!!

	return pNu;
}

// clear current corrupted NAL from pNalUnitsList
void ForceClearCurrentNal(SAccessUnit* pAu){
	if(pAu->uiAvailUnitsNum>0)
		--pAu->uiAvailUnitsNum;
}

// Copy relative syntax elements of NALUnitHeaderExt,sRefPicBaseMarking and bStoreRefBasePicFlag in prefix nal unit.
// pSrc: mark as decoded prefix NAL
// ppDst: succeeded VCL NAL based AVC (I/P Slice)
bool PrefetchNalHeaderExtSyntax(SDecoderContext* pCtx,SNalUnit* const kppDst,SNalUnit* const kpSrc){
	SNalUnitHeaderExt* pNalHdrExtD=NULL;
	SNalUnitHeaderExt* pNalHdrExtS=NULL;
	SSliceHeaderExt* pShExtD=NULL;
	PPrefixNalUnit pPrefixS=NULL;
	SSps* pSps=NULL;
	int32_t iIdx=0;

	if(kppDst==NULL || kpSrc==NULL)
		return false;

	pNalHdrExtD=&kppDst->sNalHeaderExt;
	pNalHdrExtS=&kpSrc->sNalHeaderExt;
	pShExtD=&kppDst->sNalData.sVclNal.sSliceHeaderExt;
	pPrefixS=&kpSrc->sNalData.sPrefixNal;
	pSps=&pCtx->sSpsPpsCtx.sSpsBuffer[pCtx->sSpsPpsCtx.sPpsBuffer[pShExtD->sSliceHeader.iPpsId].iSpsId];

	pNalHdrExtD->uiDependencyId=pNalHdrExtS->uiDependencyId;
	pNalHdrExtD->uiQualityId=pNalHdrExtS->uiQualityId;
	pNalHdrExtD->uiTemporalId=pNalHdrExtS->uiTemporalId;
	pNalHdrExtD->uiPriorityId=pNalHdrExtS->uiPriorityId;
	pNalHdrExtD->bIdrFlag=pNalHdrExtS->bIdrFlag;
	pNalHdrExtD->iNoInterLayerPredFlag=pNalHdrExtS->iNoInterLayerPredFlag;
	pNalHdrExtD->bDiscardableFlag=pNalHdrExtS->bDiscardableFlag;
	pNalHdrExtD->bOutputFlag=pNalHdrExtS->bOutputFlag;
	pNalHdrExtD->bUseRefBasePicFlag=pNalHdrExtS->bUseRefBasePicFlag;
	pNalHdrExtD->uiLayerDqId=pNalHdrExtS->uiLayerDqId;

	pShExtD->bStoreRefBasePicFlag=pPrefixS->bStoreRefBasePicFlag;
	memcpy(&pShExtD->sRefBasePicMarking,&pPrefixS->sRefPicBaseMarking,sizeof(SRefBasePicMarking));
	if(pShExtD->sRefBasePicMarking.bAdaptiveRefBasePicMarkingModeFlag){
		PRefBasePicMarking pRefBasePicMarking=&pShExtD->sRefBasePicMarking;
		iIdx=0;
		do{
			if(pRefBasePicMarking->mmco_base[iIdx].uiMmcoType==MMCO_END)
				break;
			if(pRefBasePicMarking->mmco_base[iIdx].uiMmcoType==MMCO_SHORT2UNUSED)
				pRefBasePicMarking->mmco_base[iIdx].iShortFrameNum=(pShExtD->sSliceHeader.iFrameNum-
					pRefBasePicMarking->mmco_base[iIdx].uiDiffOfPicNums)&((1<<pSps->uiLog2MaxFrameNum)-1);
			++iIdx;
		} while(iIdx<MAX_MMCO_COUNT);
	}

	return true;
}

// Predeclared function routines ..
int32_t ParseRefPicListReordering(SBitStringAux* pBs,SSliceHeader* pSh){
	int32_t iList=0;
	const EWelsSliceType keSt=pSh->eSliceType;
	SRefPicListReorderSyn* pRefPicListReordering=&pSh->pRefPicListReordering;
	SSps* pSps=pSh->pSps;
	uint32_t uiCode;
	if(keSt==I_SLICE || keSt==SI_SLICE)
		return ERR_NONE;
	// Common syntaxs for P or B slices: list0,list1 followed if B slices used.
	do{
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// ref_pic_list_modification_flag_l0
		pRefPicListReordering->bRefPicListReorderingFlag[iList]=!!uiCode;

		if(pRefPicListReordering->bRefPicListReorderingFlag[iList]){
			int32_t iIdx=0;
			do{
				WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// modification_of_pic_nums_idc
				const uint32_t kuiIdc=uiCode;

				// Fixed the referrence list reordering crash issue.(fault kIdc value > 3 case)---
				if((iIdx>=MAX_REF_PIC_COUNT) || (kuiIdc>3)){
					return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_REF_REORDERING);
				}
				pRefPicListReordering->sReorderingSyn[iList][iIdx].uiReorderingOfPicNumsIdc=kuiIdc;
				if(kuiIdc==3)
					break;

				if(iIdx>=pSh->uiRefCount[iList] || iIdx>=MAX_REF_PIC_COUNT)
					return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_REF_REORDERING);

				if(kuiIdc==0 || kuiIdc==1){
					// abs_diff_pic_num_minus1 should be in range 0 to MaxPicNum-1,MaxPicNum is derived as
					// 2^(4+log2_max_frame_num_minus4)
					WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// abs_diff_pic_num_minus1
					WELS_CHECK_SE_UPPER_ERROR_NOLOG(uiCode,(uint32_t)(1<<pSps->uiLog2MaxFrameNum),"abs_diff_pic_num_minus1",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_REF_REORDERING));
					pRefPicListReordering->sReorderingSyn[iList][iIdx].uiAbsDiffPicNumMinus1=uiCode;		// uiAbsDiffPicNumMinus1
				}else
				if(kuiIdc==2){
					WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// long_term_pic_num
					pRefPicListReordering->sReorderingSyn[iList][iIdx].uiLongTermPicNum=uiCode;
				}

				++iIdx;
			} while(true);
		}
		if(keSt!=B_SLICE)
			break;
		++iList;
	} while(iList<LIST_A);

	return ERR_NONE;
}

int32_t ParsePredWeightedTable(SBitStringAux* pBs,SSliceHeader* pSh){
	uint32_t uiCode;
	int32_t iList=0;
	int32_t iCode;
	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));
	WELS_CHECK_SE_BOTH_ERROR_NOLOG(uiCode,0,7,"luma_log2_weight_denom",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_LUMA_LOG2_WEIGHT_DENOM));
	pSh->sPredWeightTable.uiLumaLog2WeightDenom=uiCode;
	if(pSh->pSps->uiChromaArrayType!=0){
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));
		WELS_CHECK_SE_BOTH_ERROR_NOLOG(uiCode,0,7,"chroma_log2_weight_denom",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_CHROMA_LOG2_WEIGHT_DENOM));
		pSh->sPredWeightTable.uiChromaLog2WeightDenom=uiCode;
	}

	if((pSh->sPredWeightTable.uiLumaLog2WeightDenom|pSh->sPredWeightTable.uiChromaLog2WeightDenom)>7)
		return ERR_NONE;

	do{

		for(int i=0; i<pSh->uiRefCount[iList]; i++){
			// luma
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));
			if(!!uiCode){

				WELS_READ_VERIFY(BsGetSe(pBs,&iCode));
				WELS_CHECK_SE_BOTH_ERROR_NOLOG(iCode,-128,127,"luma_weight",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_LUMA_WEIGHT));
				pSh->sPredWeightTable.sPredList[iList].iLumaWeight[i]=iCode;

				WELS_READ_VERIFY(BsGetSe(pBs,&iCode));
				WELS_CHECK_SE_BOTH_ERROR_NOLOG(iCode,-128,127,"luma_offset",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_LUMA_OFFSET));
				pSh->sPredWeightTable.sPredList[iList].iLumaOffset[i]=iCode;
			}else{
				pSh->sPredWeightTable.sPredList[iList].iLumaWeight[i]=1<<(pSh->sPredWeightTable.uiLumaLog2WeightDenom);
				pSh->sPredWeightTable.sPredList[iList].iLumaOffset[i]=0;

			}
			// chroma
			if(pSh->pSps->uiChromaArrayType==0)
				continue;

			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));
			if(!!uiCode){
				for(int j=0; j<2; j++){
					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));
					WELS_CHECK_SE_BOTH_ERROR_NOLOG(iCode,-128,127,"chroma_weight",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_CHROMA_WEIGHT));
					pSh->sPredWeightTable.sPredList[iList].iChromaWeight[i][j]=iCode;
					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));
					WELS_CHECK_SE_BOTH_ERROR_NOLOG(iCode,-128,127,"chroma_offset",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_CHROMA_OFFSET));
					pSh->sPredWeightTable.sPredList[iList].iChromaOffset[i][j]=iCode;
				}
			}else{
				for(int j=0; j<2; j++){
					pSh->sPredWeightTable.sPredList[iList].iChromaWeight[i][j]=1<<(pSh->sPredWeightTable.uiChromaLog2WeightDenom);
					pSh->sPredWeightTable.sPredList[iList].iChromaOffset[i][j]=0;
				}
			}

		}
		++iList;
		if(pSh->eSliceType!=B_SLICE){
			break;
		}
	} while(iList<LIST_A);	// TODO: SUPPORT LIST_A
	return ERR_NONE;
}


int32_t ParseDecRefPicMarking(SDecoderContext* pCtx,SBitStringAux* pBs,SSliceHeader* pSh,SSps* pSps,const bool kbIdrFlag){
	SRefPicMarking* const kpRefMarking=&pSh->sRefMarking;
	uint32_t uiCode;
	if(kbIdrFlag){
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// no_output_of_prior_pics_flag
		kpRefMarking->bNoOutputOfPriorPicsFlag=!!uiCode;
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// long_term_reference_flag
		kpRefMarking->bLongTermRefFlag=!!uiCode;
	}else{
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// adaptive_ref_pic_marking_mode_flag
		kpRefMarking->bAdaptiveRefPicMarkingModeFlag=!!uiCode;
		if(kpRefMarking->bAdaptiveRefPicMarkingModeFlag){
			int32_t iIdx=0;
			bool bAllowMmco5=true,bMmco4Exist=false,bMmco5Exist=false,bMmco6Exist=false;
			do{
				WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// memory_management_control_operation
				const uint32_t kuiMmco=uiCode;

				kpRefMarking->sMmcoRef[iIdx].uiMmcoType=kuiMmco;
				if(kuiMmco==MMCO_END)
					break;

				if(kuiMmco==MMCO_SHORT2UNUSED || kuiMmco==MMCO_SHORT2LONG){
					bAllowMmco5=false;
					WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// difference_of_pic_nums_minus1
					kpRefMarking->sMmcoRef[iIdx].iDiffOfPicNum=1+uiCode;
					kpRefMarking->sMmcoRef[iIdx].iShortFrameNum=(pSh->iFrameNum-kpRefMarking->sMmcoRef[iIdx].iDiffOfPicNum)&((
						1<<pSps->uiLog2MaxFrameNum)-1);
				}else
				if(kuiMmco==MMCO_LONG2UNUSED){
					bAllowMmco5=false;
					WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// long_term_pic_num
					kpRefMarking->sMmcoRef[iIdx].uiLongTermPicNum=uiCode;
				}
				if(kuiMmco==MMCO_SHORT2LONG || kuiMmco==MMCO_LONG){
					if(kuiMmco==MMCO_LONG){
						WELS_VERIFY_RETURN_IF(-1,bMmco6Exist);
						bMmco6Exist=true;
					}
					WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// long_term_frame_idx
					kpRefMarking->sMmcoRef[iIdx].iLongTermFrameIdx=uiCode;
				}else
				if(kuiMmco==MMCO_SET_MAX_LONG){
					WELS_VERIFY_RETURN_IF(-1,bMmco4Exist);
					bMmco4Exist=true;
					WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// max_long_term_frame_idx_plus1
					kpRefMarking->sMmcoRef[iIdx].iMaxLongTermFrameIdx=-1+uiCode;
				}else
				if(kuiMmco==MMCO_RESET){
					WELS_VERIFY_RETURN_IF(-1,(!bAllowMmco5 || bMmco5Exist));
					bMmco5Exist=true;

					pCtx->pLastDecPicInfo->iPrevPicOrderCntLsb=0;
					pCtx->pLastDecPicInfo->iPrevPicOrderCntMsb=0;
					pSh->iPicOrderCntLsb=0;
					if(pCtx->pSliceHeader)
						pCtx->pSliceHeader->iPicOrderCntLsb=0;
				}
				++iIdx;

			} while(iIdx<MAX_MMCO_COUNT);
		}
	}

	return ERR_NONE;
}

bool FillDefaultSliceHeaderExt(SSliceHeaderExt* pShExt,SNalUnitHeaderExt* pNalExt){
	if(pShExt==NULL || pNalExt==NULL)
		return false;

	if(pNalExt->iNoInterLayerPredFlag || pNalExt->uiQualityId>0)
		pShExt->bBasePredWeightTableFlag=false;
	else
		pShExt->bBasePredWeightTableFlag=true;
	pShExt->uiRefLayerDqId=(uint8_t)-1;
	pShExt->uiDisableInterLayerDeblockingFilterIdc=0;
	pShExt->iInterLayerSliceAlphaC0Offset=0;
	pShExt->iInterLayerSliceBetaOffset=0;
	pShExt->bConstrainedIntraResamplingFlag=false;
	pShExt->uiRefLayerChromaPhaseXPlus1Flag=0;
	pShExt->uiRefLayerChromaPhaseYPlus1=1;
	// memset(&pShExt->sScaledRefLayer,0,sizeof(SPosOffset));

	pShExt->iScaledRefLayerPicWidthInSampleLuma=pShExt->sSliceHeader.iMbWidth<<4;
	pShExt->iScaledRefLayerPicHeightInSampleLuma=pShExt->sSliceHeader.iMbHeight<<4;

	pShExt->bSliceSkipFlag=false;
	pShExt->bAdaptiveBaseModeFlag=false;
	pShExt->bDefaultBaseModeFlag=false;
	pShExt->bAdaptiveMotionPredFlag=false;
	pShExt->bDefaultMotionPredFlag=false;
	pShExt->bAdaptiveResidualPredFlag=false;
	pShExt->bDefaultResidualPredFlag=false;
	pShExt->bTCoeffLevelPredFlag=false;
	pShExt->uiScanIdxStart=0;
	pShExt->uiScanIdxEnd=15;

	return true;
}

#define SLICE_HEADER_IDR_PIC_ID_MAX 65535
#define SLICE_HEADER_REDUNDANT_PIC_CNT_MAX 127
#define SLICE_HEADER_ALPHAC0_BETA_OFFSET_MIN -12
#define SLICE_HEADER_ALPHAC0_BETA_OFFSET_MAX 12
#define SLICE_HEADER_INTER_LAYER_ALPHAC0_BETA_OFFSET_MIN -12
#define SLICE_HEADER_INTER_LAYER_ALPHAC0_BETA_OFFSET_MAX 12
#define MAX_NUM_REF_IDX_L0_ACTIVE_MINUS1 15
#define MAX_NUM_REF_IDX_L1_ACTIVE_MINUS1 15
#define SLICE_HEADER_CABAC_INIT_IDC_MAX 2

// decode_slice_header_avc
// Parse slice header of bitstream in avc for storing data structure
int32_t ParseSliceHeaderSyntaxs(SDecoderContext* pCtx,SBitStringAux* pBs,const bool kbExtensionFlag){
	SNalUnit* const kpCurNal=pCtx->pAccessUnitList->pNalUnitsList[pCtx->pAccessUnitList->uiAvailUnitsNum-1];
	SNalUnitHeaderExt* pNalHeaderExt=NULL;
	SSliceHeader* pSliceHead=NULL;
	SSliceHeaderExt* pSliceHeadExt=NULL;
	SSubsetSps* pSubsetSps=NULL;
	SSps* pSps=NULL;
	SPps* pPps=NULL;
	EWelsNalUnitType eNalType=static_cast<EWelsNalUnitType> (0);
	int32_t iPpsId=0;
	int32_t iRet=ERR_NONE;
	uint8_t uiSliceType=0;
	uint8_t uiQualityId=BASE_QUALITY_ID;
	bool bIdrFlag=false;
	bool bSgChangeCycleInvolved=false;		// involved slice group change cycle ?
	uint32_t uiCode;
	int32_t iCode;

	if(kpCurNal==NULL){
		return ERR_INFO_OUT_OF_MEMORY;
	}

	pNalHeaderExt=&kpCurNal->sNalHeaderExt;
	pSliceHead=&kpCurNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader;
	eNalType=pNalHeaderExt->sNalUnitHeader.eNalUnitType;

	pSliceHeadExt=&kpCurNal->sNalData.sVclNal.sSliceHeaderExt;

	if(pSliceHeadExt){
		SRefBasePicMarking sBaseMarking;
		const bool kbStoreRefBaseFlag=pSliceHeadExt->bStoreRefBasePicFlag;
		memcpy(&sBaseMarking,&pSliceHeadExt->sRefBasePicMarking,sizeof(SRefBasePicMarking));
		memset(pSliceHeadExt,0,sizeof(SSliceHeaderExt));
		pSliceHeadExt->bStoreRefBasePicFlag=kbStoreRefBaseFlag;
		memcpy(&pSliceHeadExt->sRefBasePicMarking,&sBaseMarking,sizeof(SRefBasePicMarking));
	}

	kpCurNal->sNalData.sVclNal.bSliceHeaderExtFlag=kbExtensionFlag;

	// first_mb_in_slice
	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// first_mb_in_slice
	WELS_CHECK_SE_UPPER_ERROR(uiCode,36863u,"first_mb_in_slice",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_FIRST_MB_IN_SLICE));
	pSliceHead->iFirstMbInSlice=uiCode;

	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// slice_type
	uiSliceType=uiCode;
	if(uiSliceType>9){
		uprintf("slice type too large (%d) at first_mb(%d)",uiSliceType,pSliceHead->iFirstMbInSlice);
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_TYPE);
	}
	if(uiSliceType>4)
		uiSliceType-=5;

	if((NAL_UNIT_CODED_SLICE_IDR==eNalType) && (I_SLICE!=uiSliceType)){
		uprintf("Invalid slice type(%d) in IDR picture. ",uiSliceType);
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_TYPE);
	}

	if(kbExtensionFlag){
		if(uiSliceType>2){
			uprintf("Invalid slice type(%d).",uiSliceType);
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_TYPE);
		}
	}

	pSliceHead->eSliceType=static_cast <EWelsSliceType> (uiSliceType);

	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// pic_parameter_set_id
	WELS_CHECK_SE_UPPER_ERROR(uiCode,(MAX_PPS_COUNT-1),"iPpsId out of range",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_PPS_ID_OVERFLOW));
	iPpsId=uiCode;

	// add check PPS available here
	if(pCtx->sSpsPpsCtx.bPpsAvailFlags[iPpsId]==false){
		if(pCtx->sSpsPpsCtx.iPPSLastInvalidId!=iPpsId){
			uprintf("PPS id (%d) is invalid,previous id (%d) error ignored (%d)!",iPpsId,pCtx->sSpsPpsCtx.iPPSLastInvalidId,pCtx->sSpsPpsCtx.iPPSInvalidNum);
			pCtx->sSpsPpsCtx.iPPSLastInvalidId=iPpsId;
			pCtx->sSpsPpsCtx.iPPSInvalidNum=0;
		}else{
			pCtx->sSpsPpsCtx.iPPSInvalidNum++;
		}
		pCtx->iErrorCode|=dsNoParamSets;
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_PPS_ID);
	}
	pCtx->sSpsPpsCtx.iPPSLastInvalidId=-1;

	pPps=&pCtx->sSpsPpsCtx.sPpsBuffer[iPpsId];

	if(pPps->uiNumSliceGroups==0){
		uprintf("Invalid PPS referenced");
		pCtx->iErrorCode|=dsNoParamSets;
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_NO_PARAM_SETS);
	}

	if(kbExtensionFlag){
		pSubsetSps=&pCtx->sSpsPpsCtx.sSubsetSpsBuffer[pPps->iSpsId];
		pSps=&pSubsetSps->sSps;
		if(pCtx->sSpsPpsCtx.bSubspsAvailFlags[pPps->iSpsId]==false){
			// pCtx->pDecoderStatistics->iSubSpsReportErrorNum++;
			if(pCtx->sSpsPpsCtx.iSubSPSLastInvalidId!=pPps->iSpsId){
				uprintf("Sub SPS id (%d) is invalid,previous id (%d) error ignored (%d)!",pPps->iSpsId,pCtx->sSpsPpsCtx.iSubSPSLastInvalidId,pCtx->sSpsPpsCtx.iSubSPSInvalidNum);
				pCtx->sSpsPpsCtx.iSubSPSLastInvalidId=pPps->iSpsId;
				pCtx->sSpsPpsCtx.iSubSPSInvalidNum=0;
			}else{
				pCtx->sSpsPpsCtx.iSubSPSInvalidNum++;
			}
			pCtx->iErrorCode|=dsNoParamSets;
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SPS_ID);
		}
		pCtx->sSpsPpsCtx.iSubSPSLastInvalidId=-1;
	}else{
		if(pCtx->sSpsPpsCtx.bSpsAvailFlags[pPps->iSpsId]==false){
			// pCtx->pDecoderStatistics->iSpsReportErrorNum++;
			if(pCtx->sSpsPpsCtx.iSPSLastInvalidId!=pPps->iSpsId){
				uprintf("SPS id (%d) is invalid,previous id (%d) error ignored (%d)!",pPps->iSpsId,pCtx->sSpsPpsCtx.iSPSLastInvalidId,pCtx->sSpsPpsCtx.iSPSInvalidNum);
				pCtx->sSpsPpsCtx.iSPSLastInvalidId=pPps->iSpsId;
				pCtx->sSpsPpsCtx.iSPSInvalidNum=0;
			}else{
				pCtx->sSpsPpsCtx.iSPSInvalidNum++;
			}
			pCtx->iErrorCode|=dsNoParamSets;
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SPS_ID);
		}
		pCtx->sSpsPpsCtx.iSPSLastInvalidId=-1;
		pSps=&pCtx->sSpsPpsCtx.sSpsBuffer[pPps->iSpsId];
	}
	pSliceHead->iPpsId=iPpsId;
	pSliceHead->iSpsId=pPps->iSpsId;
	pSliceHead->pPps=pPps;
	pSliceHead->pSps=pSps;

	pSliceHeadExt->pSubsetSps=pSubsetSps;

	if(pSps->iNumRefFrames==0){
		if((uiSliceType!=I_SLICE) && (uiSliceType!=SI_SLICE)){
			uprintf("slice_type (%d) not supported for num_ref_frames=0.",uiSliceType);
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_TYPE);
		}
	}
	bIdrFlag=(!kbExtensionFlag && eNalType==NAL_UNIT_CODED_SLICE_IDR) || (kbExtensionFlag && pNalHeaderExt->bIdrFlag);
	pSliceHead->bIdrFlag=bIdrFlag;
	if(pSps->uiLog2MaxFrameNum==0){
		uprintf("non existing SPS referenced");
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_NO_PARAM_SETS);
	}
	// check first_mb_in_slice
	WELS_CHECK_SE_UPPER_ERROR((uint32_t)(pSliceHead->iFirstMbInSlice),(pSps->uiTotalMbCount-1),"first_mb_in_slice",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_FIRST_MB_IN_SLICE));
	WELS_READ_VERIFY(BsGetBits(pBs,pSps->uiLog2MaxFrameNum,&uiCode));		// frame_num
	pSliceHead->iFrameNum=uiCode;

	pSliceHead->bFieldPicFlag=false;
	pSliceHead->bBottomFiledFlag=false;
	if(!pSps->bFrameMbsOnlyFlag){
		uprintf("ParseSliceHeaderSyntaxs(): frame_mbs_only_flag=%d not supported. ",pSps->bFrameMbsOnlyFlag);
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_MBAFF);
	}
	pSliceHead->iMbWidth=pSps->iMbWidth;
	pSliceHead->iMbHeight=pSps->iMbHeight/(1+pSliceHead->bFieldPicFlag);

	if(bIdrFlag){
		if(pSliceHead->iFrameNum!=0){
			uprintf("ParseSliceHeaderSyntaxs(),invaild frame number: %d due to IDR frame introduced!",pSliceHead->iFrameNum);
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_FRAME_NUM);
		}
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// idr_pic_id
		// standard 7.4.3 idr_pic_id should be in range 0 to 65535,inclusive.
		WELS_CHECK_SE_UPPER_ERROR(uiCode,SLICE_HEADER_IDR_PIC_ID_MAX,"idr_pic_id",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_IDR_PIC_ID));
		pSliceHead->uiIdrPicId=uiCode;
		pCtx->uiCurIdrPicId=pSliceHead->uiIdrPicId;
	}

	pSliceHead->iDeltaPicOrderCntBottom=0;
	pSliceHead->iDeltaPicOrderCnt[0]=
		pSliceHead->iDeltaPicOrderCnt[1]=0;
	if(pSps->uiPocType==0){
		WELS_READ_VERIFY(BsGetBits(pBs,pSps->iLog2MaxPocLsb,&uiCode));		// pic_order_cnt_lsb
		const int32_t iMaxPocLsb=1<<(pSps->iLog2MaxPocLsb);
		pSliceHead->iPicOrderCntLsb=uiCode;
		if(pPps->bPicOrderPresentFlag && !pSliceHead->bFieldPicFlag){
			WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// delta_pic_order_cnt_bottom
			pSliceHead->iDeltaPicOrderCntBottom=iCode;
		}
		// Calculate poc if necessary
		int32_t pocLsb=pSliceHead->iPicOrderCntLsb;
		if(pSliceHead->bIdrFlag || kpCurNal->sNalHeaderExt.sNalUnitHeader.eNalUnitType==NAL_UNIT_CODED_SLICE_IDR){
			pCtx->pLastDecPicInfo->iPrevPicOrderCntMsb=0;
			pCtx->pLastDecPicInfo->iPrevPicOrderCntLsb=0;
		}
		int32_t pocMsb;
		if(pocLsb<pCtx->pLastDecPicInfo->iPrevPicOrderCntLsb
			 && pCtx->pLastDecPicInfo->iPrevPicOrderCntLsb-pocLsb>=iMaxPocLsb/2)
			pocMsb=pCtx->pLastDecPicInfo->iPrevPicOrderCntMsb+iMaxPocLsb;
		else if(pocLsb>pCtx->pLastDecPicInfo->iPrevPicOrderCntLsb
				  && pocLsb-pCtx->pLastDecPicInfo->iPrevPicOrderCntLsb>iMaxPocLsb/2)
			pocMsb=pCtx->pLastDecPicInfo->iPrevPicOrderCntMsb-iMaxPocLsb;
		else
			pocMsb=pCtx->pLastDecPicInfo->iPrevPicOrderCntMsb;
		pSliceHead->iPicOrderCntLsb=pocMsb+pocLsb;

		if(pPps->bPicOrderPresentFlag && !pSliceHead->bFieldPicFlag){
			pSliceHead->iPicOrderCntLsb+=pSliceHead->iDeltaPicOrderCntBottom;
		}

		if(kpCurNal->sNalHeaderExt.sNalUnitHeader.uiNalRefIdc!=0){
			pCtx->pLastDecPicInfo->iPrevPicOrderCntLsb=pocLsb;
			pCtx->pLastDecPicInfo->iPrevPicOrderCntMsb=pocMsb;
		}
		// End of Calculating poc
	}else
	if(pSps->uiPocType==1 && !pSps->bDeltaPicOrderAlwaysZeroFlag){
		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// delta_pic_order_cnt[ 0 ]
		pSliceHead->iDeltaPicOrderCnt[0]=iCode;
		if(pPps->bPicOrderPresentFlag && !pSliceHead->bFieldPicFlag){
			WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// delta_pic_order_cnt[ 1 ]
			pSliceHead->iDeltaPicOrderCnt[1]=iCode;
		}
	}
	pSliceHead->iRedundantPicCnt=0;
	if(pPps->bRedundantPicCntPresentFlag){
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// redundant_pic_cnt
		// standard section 7.4.3,redundant_pic_cnt should be in range 0 to 127,inclusive.
		WELS_CHECK_SE_UPPER_ERROR(uiCode,SLICE_HEADER_REDUNDANT_PIC_CNT_MAX,"redundant_pic_cnt",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_REDUNDANT_PIC_CNT));
		pSliceHead->iRedundantPicCnt=uiCode;
		if(pSliceHead->iRedundantPicCnt>0){
			uprintf("Redundant picture not supported!");
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_REDUNDANT_PIC_CNT);
		}
	}

	if(B_SLICE==uiSliceType){
		// fix me: it needs to use the this flag somewhere for B-Sclice
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// direct_spatial_mv_pred_flag
		pSliceHead->iDirectSpatialMvPredFlag=uiCode;
	}

	// set defaults,might be overriden a few line later
	pSliceHead->uiRefCount[0]=pPps->uiNumRefIdxL0Active;
	pSliceHead->uiRefCount[1]=pPps->uiNumRefIdxL1Active;

	bool bReadNumRefFlag=(P_SLICE==uiSliceType || B_SLICE==uiSliceType);
	if(kbExtensionFlag){
		bReadNumRefFlag&=(BASE_QUALITY_ID==pNalHeaderExt->uiQualityId);
	}
	if(bReadNumRefFlag){
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// num_ref_idx_active_override_flag
		pSliceHead->bNumRefIdxActiveOverrideFlag=!!uiCode;
		if(pSliceHead->bNumRefIdxActiveOverrideFlag){
			WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// num_ref_idx_l0_active_minus1
			WELS_CHECK_SE_UPPER_ERROR(uiCode,MAX_NUM_REF_IDX_L0_ACTIVE_MINUS1,"num_ref_idx_l0_active_minus1",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_NUM_REF_IDX_L0_ACTIVE_MINUS1));
			pSliceHead->uiRefCount[0]=1+uiCode;
			if(B_SLICE==uiSliceType){
				WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// num_ref_idx_l1_active_minus1
				WELS_CHECK_SE_UPPER_ERROR(uiCode,MAX_NUM_REF_IDX_L1_ACTIVE_MINUS1,"num_ref_idx_l1_active_minus1",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_NUM_REF_IDX_L1_ACTIVE_MINUS1));
				pSliceHead->uiRefCount[1]=1+uiCode;
			}
		}
	}

	if(pSliceHead->uiRefCount[0]>MAX_REF_PIC_COUNT || pSliceHead->uiRefCount[1]>MAX_REF_PIC_COUNT){
		uprintf("reference overflow");
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_REF_COUNT_OVERFLOW);
	}

	if(BASE_QUALITY_ID==uiQualityId){
		iRet=ParseRefPicListReordering(pBs,pSliceHead);
		if(iRet!=ERR_NONE){
			uprintf("invalid ref pPic list reordering syntaxs!");
			return iRet;
		}

		if((pPps->bWeightedPredFlag && uiSliceType==P_SLICE) || (pPps->uiWeightedBipredIdc==1 && uiSliceType==B_SLICE)){
			iRet=ParsePredWeightedTable(pBs,pSliceHead);
			if(iRet!=ERR_NONE){
				uprintf("invalid weighted prediction syntaxs!");
				return iRet;
			}
		}

		if(kbExtensionFlag){
			if(pNalHeaderExt->iNoInterLayerPredFlag || pNalHeaderExt->uiQualityId>0)
				pSliceHeadExt->bBasePredWeightTableFlag=false;
			else
				pSliceHeadExt->bBasePredWeightTableFlag=true;
		}

		if(kpCurNal->sNalHeaderExt.sNalUnitHeader.uiNalRefIdc!=0){
			iRet=ParseDecRefPicMarking(pCtx,pBs,pSliceHead,pSps,bIdrFlag);
			if(iRet!=ERR_NONE){
				return iRet;
			}

			if(kbExtensionFlag && !pSubsetSps->sSpsSvcExt.bSliceHeaderRestrictionFlag){
				WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// store_ref_base_pic_flag
				pSliceHeadExt->bStoreRefBasePicFlag=!!uiCode;
				if((pNalHeaderExt->bUseRefBasePicFlag || pSliceHeadExt->bStoreRefBasePicFlag) && !bIdrFlag){
					uprintf("ParseSliceHeaderSyntaxs(): bUseRefBasePicFlag or bStoreRefBasePicFlag=1 not supported.");
					return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_ILP);
				}
			}
		}
	}

	if(pPps->bEntropyCodingModeFlag){
		if(pSliceHead->eSliceType!=I_SLICE && pSliceHead->eSliceType!=SI_SLICE){
			WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));
			WELS_CHECK_SE_UPPER_ERROR(uiCode,SLICE_HEADER_CABAC_INIT_IDC_MAX,"cabac_init_idc",ERR_INFO_INVALID_CABAC_INIT_IDC);
			pSliceHead->iCabacInitIdc=uiCode;
		}else
			pSliceHead->iCabacInitIdc=0;
	}

	WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// slice_qp_delta
	pSliceHead->iSliceQpDelta=iCode;
	pSliceHead->iSliceQp=pPps->iPicInitQp+pSliceHead->iSliceQpDelta;
	if(pSliceHead->iSliceQp<0 || pSliceHead->iSliceQp > 51){
		uprintf("QP %d out of range",pSliceHead->iSliceQp);
		return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_QP);
	}

	// FIXME qscale / qp ... stuff
	if(!kbExtensionFlag){
		if(uiSliceType==SP_SLICE || uiSliceType==SI_SLICE){
			uprintf("SP/SI not supported");
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_SPSI);
		}
	}

	pSliceHead->uiDisableDeblockingFilterIdc=0;
	pSliceHead->iSliceAlphaC0Offset=0;
	pSliceHead->iSliceBetaOffset=0;
	if(pPps->bDeblockingFilterControlPresentFlag){
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// disable_deblocking_filter_idc
		pSliceHead->uiDisableDeblockingFilterIdc=uiCode;
		// refer to JVT-X201wcm1.doc G.7.4.3.4--2010.4.20
		if(pSliceHead->uiDisableDeblockingFilterIdc>6){
			uprintf("disable_deblock_filter_idc (%d) out of range [0,6]",pSliceHead->uiDisableDeblockingFilterIdc);
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_DBLOCKING_IDC);
		}
		if(pSliceHead->uiDisableDeblockingFilterIdc!=1){
			WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// slice_alpha_c0_offset_div2
			pSliceHead->iSliceAlphaC0Offset=iCode*2;
			WELS_CHECK_SE_BOTH_ERROR(pSliceHead->iSliceAlphaC0Offset,SLICE_HEADER_ALPHAC0_BETA_OFFSET_MIN,SLICE_HEADER_ALPHAC0_BETA_OFFSET_MAX,"slice_alpha_c0_offset_div2 * 2",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_ALPHA_C0_OFFSET_DIV2));
			WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// slice_beta_offset_div2
			pSliceHead->iSliceBetaOffset=iCode*2;
			WELS_CHECK_SE_BOTH_ERROR(pSliceHead->iSliceBetaOffset,SLICE_HEADER_ALPHAC0_BETA_OFFSET_MIN,SLICE_HEADER_ALPHAC0_BETA_OFFSET_MAX,"slice_beta_offset_div2 * 2",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_BETA_OFFSET_DIV2));
		}
	}

	bSgChangeCycleInvolved=(pPps->uiNumSliceGroups>1 && pPps->uiSliceGroupMapType>=3
							  && pPps->uiSliceGroupMapType<=5);
	if(kbExtensionFlag && bSgChangeCycleInvolved)
		bSgChangeCycleInvolved=(bSgChangeCycleInvolved && (uiQualityId==BASE_QUALITY_ID));
	if(bSgChangeCycleInvolved){
		if(pPps->uiSliceGroupChangeRate>0){
			const int32_t kiNumBits=(int32_t)WELS_CEIL(log(static_cast<double> (1+pPps->uiPicSizeInMapUnits/
				pPps->uiSliceGroupChangeRate)));
			WELS_READ_VERIFY(BsGetBits(pBs,kiNumBits,&uiCode));		// lice_group_change_cycle
			pSliceHead->iSliceGroupChangeCycle=uiCode;
		}else
			pSliceHead->iSliceGroupChangeCycle=0;
	}

	if(!kbExtensionFlag){
		FillDefaultSliceHeaderExt(pSliceHeadExt,pNalHeaderExt);
	}else{
		// Extra syntax elements newly introduced
		pSliceHeadExt->pSubsetSps=pSubsetSps;

		if(!pNalHeaderExt->iNoInterLayerPredFlag && BASE_QUALITY_ID==uiQualityId){
			// the following should be deleted for CODE_CLEAN
			WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// ref_layer_dq_id
			pSliceHeadExt->uiRefLayerDqId=uiCode;
			if(pSubsetSps->sSpsSvcExt.bInterLayerDeblockingFilterCtrlPresentFlag){
				WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// disable_inter_layer_deblocking_filter_idc
				pSliceHeadExt->uiDisableInterLayerDeblockingFilterIdc=uiCode;
				// refer to JVT-X201wcm1.doc G.7.4.3.4--2010.4.20
				if(pSliceHeadExt->uiDisableInterLayerDeblockingFilterIdc>6){
					uprintf("disable_inter_layer_deblock_filter_idc (%d) out of range [0,6]",pSliceHeadExt->uiDisableInterLayerDeblockingFilterIdc);
					return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_DBLOCKING_IDC);
				}
				if(pSliceHeadExt->uiDisableInterLayerDeblockingFilterIdc!=1){
					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// inter_layer_slice_alpha_c0_offset_div2
					pSliceHeadExt->iInterLayerSliceAlphaC0Offset=iCode*2;
					WELS_CHECK_SE_BOTH_ERROR(pSliceHeadExt->iInterLayerSliceAlphaC0Offset,SLICE_HEADER_INTER_LAYER_ALPHAC0_BETA_OFFSET_MIN,SLICE_HEADER_INTER_LAYER_ALPHAC0_BETA_OFFSET_MAX,"inter_layer_alpha_c0_offset_div2 * 2",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_ALPHA_C0_OFFSET_DIV2));
					WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// inter_layer_slice_beta_offset_div2
					pSliceHeadExt->iInterLayerSliceBetaOffset=iCode*2;
					WELS_CHECK_SE_BOTH_ERROR(pSliceHeadExt->iInterLayerSliceBetaOffset,SLICE_HEADER_INTER_LAYER_ALPHAC0_BETA_OFFSET_MIN,SLICE_HEADER_INTER_LAYER_ALPHAC0_BETA_OFFSET_MAX,"inter_layer_slice_beta_offset_div2 * 2",GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_INVALID_SLICE_BETA_OFFSET_DIV2));
				}
			}

			pSliceHeadExt->uiRefLayerChromaPhaseXPlus1Flag=pSubsetSps->sSpsSvcExt.uiSeqRefLayerChromaPhaseXPlus1Flag;
			pSliceHeadExt->uiRefLayerChromaPhaseYPlus1=pSubsetSps->sSpsSvcExt.uiSeqRefLayerChromaPhaseYPlus1;

			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constrained_intra_resampling_flag
			pSliceHeadExt->bConstrainedIntraResamplingFlag=!!uiCode;

			{
				SPosOffset pos;
				pos.iLeftOffset=pSubsetSps->sSpsSvcExt.sSeqScaledRefLayer.iLeftOffset;
				pos.iTopOffset=pSubsetSps->sSpsSvcExt.sSeqScaledRefLayer.iTopOffset*(2-pSps->bFrameMbsOnlyFlag);
				pos.iRightOffset=pSubsetSps->sSpsSvcExt.sSeqScaledRefLayer.iRightOffset;
				pos.iBottomOffset=pSubsetSps->sSpsSvcExt.sSeqScaledRefLayer.iBottomOffset*(2-pSps->bFrameMbsOnlyFlag);
				// memcpy(&pSliceHeadExt->sScaledRefLayer,&pos,sizeof(SPosOffset));	// confirmed_safe_unsafe_usage
				pSliceHeadExt->iScaledRefLayerPicWidthInSampleLuma=(pSliceHead->iMbWidth<<4)-
					(pos.iLeftOffset+pos.iRightOffset);
				pSliceHeadExt->iScaledRefLayerPicHeightInSampleLuma=(pSliceHead->iMbHeight<<4)-
					(pos.iTopOffset+pos.iBottomOffset)/(1+pSliceHead->bFieldPicFlag);
			}
		}else
		if(uiQualityId>BASE_QUALITY_ID){
			uprintf("MGS not supported.");
			return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_MGS);
		}else{
			pSliceHeadExt->uiRefLayerDqId=(uint8_t)-1;
		}

		pSliceHeadExt->bSliceSkipFlag=false;
		pSliceHeadExt->bAdaptiveBaseModeFlag=false;
		pSliceHeadExt->bDefaultBaseModeFlag=false;
		pSliceHeadExt->bAdaptiveMotionPredFlag=false;
		pSliceHeadExt->bDefaultMotionPredFlag=false;
		pSliceHeadExt->bAdaptiveResidualPredFlag=false;
		pSliceHeadExt->bDefaultResidualPredFlag=false;
		if(pNalHeaderExt->iNoInterLayerPredFlag)
			pSliceHeadExt->bTCoeffLevelPredFlag=false;
		else
			pSliceHeadExt->bTCoeffLevelPredFlag=pSubsetSps->sSpsSvcExt.bSeqTCoeffLevelPredFlag;

		if(!pNalHeaderExt->iNoInterLayerPredFlag){
			WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// slice_skip_flag
			pSliceHeadExt->bSliceSkipFlag=!!uiCode;
			if(pSliceHeadExt->bSliceSkipFlag){
				uprintf("bSliceSkipFlag==1 not supported.");
				return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_SLICESKIP);
			}else{
				WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// adaptive_base_mode_flag
				pSliceHeadExt->bAdaptiveBaseModeFlag=!!uiCode;
				if(!pSliceHeadExt->bAdaptiveBaseModeFlag){
					WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// default_base_mode_flag
					pSliceHeadExt->bDefaultBaseModeFlag=!!uiCode;
				}
				if(!pSliceHeadExt->bDefaultBaseModeFlag){
					WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// adaptive_motion_prediction_flag
					pSliceHeadExt->bAdaptiveMotionPredFlag=!!uiCode;
					if(!pSliceHeadExt->bAdaptiveMotionPredFlag){
						WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// default_motion_prediction_flag
						pSliceHeadExt->bDefaultMotionPredFlag=!!uiCode;
					}
				}

				WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// adaptive_residual_prediction_flag
				pSliceHeadExt->bAdaptiveResidualPredFlag=!!uiCode;
				if(!pSliceHeadExt->bAdaptiveResidualPredFlag){
					WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// default_residual_prediction_flag
					pSliceHeadExt->bDefaultResidualPredFlag=!!uiCode;
				}
			}
			if(pSubsetSps->sSpsSvcExt.bAdaptiveTCoeffLevelPredFlag){
				WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// tcoeff_level_prediction_flag
				pSliceHeadExt->bTCoeffLevelPredFlag=!!uiCode;
			}
		}

		if(!pSubsetSps->sSpsSvcExt.bSliceHeaderRestrictionFlag){
			WELS_READ_VERIFY(BsGetBits(pBs,4,&uiCode));		// scan_idx_start
			pSliceHeadExt->uiScanIdxStart=uiCode;
			WELS_READ_VERIFY(BsGetBits(pBs,4,&uiCode));		// scan_idx_end
			pSliceHeadExt->uiScanIdxEnd=uiCode;
			if(pSliceHeadExt->uiScanIdxStart!=0 || pSliceHeadExt->uiScanIdxEnd!=15){
				uprintf("uiScanIdxStart (%d) !=0 and uiScanIdxEnd (%d) !=15 not supported here",pSliceHeadExt->uiScanIdxStart,pSliceHeadExt->uiScanIdxEnd);
				return GENERATE_ERROR_NO(ERR_LEVEL_SLICE_HEADER,ERR_INFO_UNSUPPORTED_MGS);
			}
		}else{
			pSliceHeadExt->uiScanIdxStart=0;
			pSliceHeadExt->uiScanIdxEnd=15;
		}
	}

	return ERR_NONE;
}

static bool CheckNextAuNewSeq(SDecoderContext* pCtx,const SNalUnit* kpCurNal,const SSps* kpSps){
	const SNalUnitHeaderExt* kpCurNalHeaderExt=&kpCurNal->sNalHeaderExt;
	if(pCtx->sSpsPpsCtx.pActiveLayerSps[kpCurNalHeaderExt->uiDependencyId]!=NULL && pCtx->sSpsPpsCtx.pActiveLayerSps[kpCurNalHeaderExt->uiDependencyId]!=kpSps)
		return true;
	if(kpCurNalHeaderExt->bIdrFlag)
		return true;
	return false;
}


bool CheckAccessUnitBoundary1(SDecoderContext* pCtx,const SNalUnit* kpCurNal,const SNalUnit* kpLastNal,const SSps* kpSps){
	const SNalUnitHeaderExt* kpLastNalHeaderExt=&kpLastNal->sNalHeaderExt;
	const SNalUnitHeaderExt* kpCurNalHeaderExt=&kpCurNal->sNalHeaderExt;
	const SSliceHeader* kpLastSliceHeader=&kpLastNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader;
	const SSliceHeader* kpCurSliceHeader=&kpCurNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader;
	if(pCtx->sSpsPpsCtx.pActiveLayerSps[kpCurNalHeaderExt->uiDependencyId]!=NULL && pCtx->sSpsPpsCtx.pActiveLayerSps[kpCurNalHeaderExt->uiDependencyId]!=kpSps){
		return true;		// the active sps changed,new sequence begins,so the current au is ready
	}

	// Sub-clause 7.1.4.1.1 temporal_id
	if(kpLastNalHeaderExt->uiTemporalId!=kpCurNalHeaderExt->uiTemporalId){
		return true;
	}
	if(kpLastSliceHeader->iFrameNum!=kpCurSliceHeader->iFrameNum)
		return true;
	// Subclause 7.4.1.2.5
	if(kpLastSliceHeader->iRedundantPicCnt>kpCurSliceHeader->iRedundantPicCnt)
		return true;

	// Subclause G7.4.1.2.4
	if(kpLastNalHeaderExt->uiDependencyId>kpCurNalHeaderExt->uiDependencyId)
		return true;
	// Subclause 7.4.1.2.4
	if(kpLastNalHeaderExt->uiDependencyId==kpCurNalHeaderExt->uiDependencyId && kpLastSliceHeader->iPpsId!=kpCurSliceHeader->iPpsId)
		return true;
	if(kpLastSliceHeader->bFieldPicFlag!=kpCurSliceHeader->bFieldPicFlag)
		return true;
	if(kpLastSliceHeader->bBottomFiledFlag!=kpCurSliceHeader->bBottomFiledFlag)
		return true;
	if((kpLastNalHeaderExt->sNalUnitHeader.uiNalRefIdc!=NRI_PRI_LOWEST)!=(kpCurNalHeaderExt->sNalUnitHeader.uiNalRefIdc!=NRI_PRI_LOWEST))
		return true;
	if(kpLastNalHeaderExt->bIdrFlag!=kpCurNalHeaderExt->bIdrFlag)
		return true;
	if(kpCurNalHeaderExt->bIdrFlag){
		if(kpLastSliceHeader->uiIdrPicId!=kpCurSliceHeader->uiIdrPicId)
			return true;
	}
	if(kpSps->uiPocType==0){
		if(kpLastSliceHeader->iPicOrderCntLsb!=kpCurSliceHeader->iPicOrderCntLsb)
			return true;
		if(kpLastSliceHeader->iDeltaPicOrderCntBottom!=kpCurSliceHeader->iDeltaPicOrderCntBottom)
			return true;
	}else
		if(kpSps->uiPocType==1){
			if(kpLastSliceHeader->iDeltaPicOrderCnt[0]!=kpCurSliceHeader->iDeltaPicOrderCnt[0])
				return true;
			if(kpLastSliceHeader->iDeltaPicOrderCnt[1]!=kpCurSliceHeader->iDeltaPicOrderCnt[1])
				return true;
		}
	return false;
}

bool CheckAccessUnitBoundary(SDecoderContext* pCtx,const SNalUnit* kpCurNal,const SNalUnit* kpLastNal,const SSps* kpSps){
	const SNalUnitHeaderExt* kpLastNalHeaderExt=&kpLastNal->sNalHeaderExt;
	const SNalUnitHeaderExt* kpCurNalHeaderExt=&kpCurNal->sNalHeaderExt;
	const SSliceHeader* kpLastSliceHeader=&kpLastNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader;
	const SSliceHeader* kpCurSliceHeader=&kpCurNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader;
	if(pCtx->sSpsPpsCtx.pActiveLayerSps[kpCurNalHeaderExt->uiDependencyId]!=NULL
		 && pCtx->sSpsPpsCtx.pActiveLayerSps[kpCurNalHeaderExt->uiDependencyId]!=kpSps){
		return true;		// the active sps changed,new sequence begins,so the current au is ready
	}

	// Sub-clause 7.1.4.1.1 temporal_id
	if(kpLastNalHeaderExt->uiTemporalId!=kpCurNalHeaderExt->uiTemporalId){
		return true;
	}
	if(kpLastSliceHeader->iFrameNum!=kpCurSliceHeader->iFrameNum)
		return true;
	// Subclause 7.4.1.2.5
	if(kpLastSliceHeader->iRedundantPicCnt>kpCurSliceHeader->iRedundantPicCnt)
		return true;

	// Subclause G7.4.1.2.4
	if(kpLastNalHeaderExt->uiDependencyId>kpCurNalHeaderExt->uiDependencyId)
		return true;
	// Subclause 7.4.1.2.4
	if(kpLastNalHeaderExt->uiDependencyId==kpCurNalHeaderExt->uiDependencyId
		 && kpLastSliceHeader->iPpsId!=kpCurSliceHeader->iPpsId)
		return true;
	if(kpLastSliceHeader->bFieldPicFlag!=kpCurSliceHeader->bFieldPicFlag)
		return true;
	if(kpLastSliceHeader->bBottomFiledFlag!=kpCurSliceHeader->bBottomFiledFlag)
		return true;
	if((kpLastNalHeaderExt->sNalUnitHeader.uiNalRefIdc!=NRI_PRI_LOWEST)!=(kpCurNalHeaderExt->sNalUnitHeader.uiNalRefIdc
		!=NRI_PRI_LOWEST))
		return true;
	if(kpLastNalHeaderExt->bIdrFlag!=kpCurNalHeaderExt->bIdrFlag)
		return true;
	if(kpCurNalHeaderExt->bIdrFlag){
		if(kpLastSliceHeader->uiIdrPicId!=kpCurSliceHeader->uiIdrPicId)
			return true;
	}
	if(kpSps->uiPocType==0){
		if(kpLastSliceHeader->iPicOrderCntLsb!=kpCurSliceHeader->iPicOrderCntLsb)
			return true;
		if(kpLastSliceHeader->iDeltaPicOrderCntBottom!=kpCurSliceHeader->iDeltaPicOrderCntBottom)
			return true;
	}else
	if(kpSps->uiPocType==1){
		if(kpLastSliceHeader->iDeltaPicOrderCnt[0]!=kpCurSliceHeader->iDeltaPicOrderCnt[0])
			return true;
		if(kpLastSliceHeader->iDeltaPicOrderCnt[1]!=kpCurSliceHeader->iDeltaPicOrderCnt[1])
			return true;
	}

	return false;
}

// brief to parse nal unit
// param pCtx decoder context
// param pNalUnitHeader parsed result of NAL Unit Header to output
// param pSrcRbsp bitstream buffer to input
// param iSrcRbspLen length size of bitstream buffer payload
// param pSrcNal
// param iSrcNalLen
// param pConsumedBytes consumed bytes during parsing
// return decoded bytes payload,might be (pSrcRbsp+1) if no escapes
uint8_t* ParseNalHeader(SDecoderContext* pCtx,SNalUnitHeader* pNalUnitHeader,uint8_t* pSrcRbsp,int32_t iSrcRbspLen,uint8_t* pSrcNal,int32_t iSrcNalLen,int32_t* pConsumedBytes){
	SNalUnit* pCurNal=NULL;
	uint8_t* pNal=pSrcRbsp;
	int32_t iNalSize=iSrcRbspLen;
	SBitStringAux* pBs=NULL;
	bool bExtensionFlag=false;
	int32_t iErr=ERR_NONE;
	int32_t iBitSize=0;
	pNalUnitHeader->eNalUnitType=NAL_UNIT_UNSPEC_0;	// SHOULD init it. because pCtx->sCurNalHead is common variable.

	// remove the consecutive ZERO at the end of current NAL in the reverse order.--2011.6.1
	{
		int32_t iIndex=iSrcRbspLen-1;
		uint8_t uiBsZero=0;
		while(iIndex>=0){
			uiBsZero=pSrcRbsp[iIndex];
			if(0==uiBsZero){
				--iNalSize;
				++(*pConsumedBytes);
				--iIndex;
			}else{
				break;
			}
		}
	}

	pNalUnitHeader->uiForbiddenZeroBit=(uint8_t)(pNal[0]>>7);		// uiForbiddenZeroBit
	if(pNalUnitHeader->uiForbiddenZeroBit){		// 2010.4.14
		pCtx->iErrorCode|=dsBitstreamError;
		return NULL;		// uiForbiddenZeroBit should always equal to 0
	}

	pNalUnitHeader->uiNalRefIdc=(uint8_t)(pNal[0]>>5);		// uiNalRefIdc
	pNalUnitHeader->eNalUnitType=(EWelsNalUnitType)(pNal[0]&0x1f);		// eNalUnitType

	++pNal;
	--iNalSize;
	++(*pConsumedBytes);

	if(!(IS_SEI_NAL(pNalUnitHeader->eNalUnitType) || IS_SPS_NAL(pNalUnitHeader->eNalUnitType) || IS_AU_DELIMITER_NAL(pNalUnitHeader->eNalUnitType) || pCtx->sSpsPpsCtx.bSpsExistAheadFlag)){
		if(pCtx->bPrintFrameErrorTraceFlag && pCtx->sSpsPpsCtx.iSpsErrorIgnored==0){
			//uprintf("parse_nal(),no exist Sequence Parameter Sets ahead of sequence when try to decode NAL(type:%d).",pNalUnitHeader->eNalUnitType);
		}else{
			pCtx->sSpsPpsCtx.iSpsErrorIgnored++;
		}
		// pCtx->pDecoderStatistics->iSpsNoExistNalNum++;
		pCtx->iErrorCode=dsNoParamSets;
		return NULL;
	}
	pCtx->sSpsPpsCtx.iSpsErrorIgnored=0;
	if(!(IS_SEI_NAL(pNalUnitHeader->eNalUnitType) || IS_PARAM_SETS_NALS(pNalUnitHeader->eNalUnitType) || IS_AU_DELIMITER_NAL(pNalUnitHeader->eNalUnitType) || pCtx->sSpsPpsCtx.bPpsExistAheadFlag)){
		if(pCtx->bPrintFrameErrorTraceFlag && pCtx->sSpsPpsCtx.iPpsErrorIgnored==0){
			//uprintf("parse_nal(),no exist Picture Parameter Sets ahead of sequence when try to decode NAL(type:%d).",pNalUnitHeader->eNalUnitType);
		}else{
			pCtx->sSpsPpsCtx.iPpsErrorIgnored++;
		}
		// pCtx->pDecoderStatistics->iPpsNoExistNalNum++;
		pCtx->iErrorCode=dsNoParamSets;
		return NULL;
	}
	pCtx->sSpsPpsCtx.iPpsErrorIgnored=0;
	if((IS_VCL_NAL_AVC_BASE(pNalUnitHeader->eNalUnitType) && !(pCtx->sSpsPpsCtx.bSpsExistAheadFlag
		 || pCtx->sSpsPpsCtx.bPpsExistAheadFlag)) || (IS_NEW_INTRODUCED_SVC_NAL(pNalUnitHeader->eNalUnitType) && !(pCtx->sSpsPpsCtx.bSpsExistAheadFlag || pCtx->sSpsPpsCtx.bSubspsExistAheadFlag || pCtx->sSpsPpsCtx.bPpsExistAheadFlag))){
		if(pCtx->bPrintFrameErrorTraceFlag && pCtx->sSpsPpsCtx.iSubSpsErrorIgnored==0){
			//uprintf("ParseNalHeader(),no exist Parameter Sets ahead of sequence when try to decode slice(type:%d).",pNalUnitHeader->eNalUnitType);
		}else{
			pCtx->sSpsPpsCtx.iSubSpsErrorIgnored++;
		}
		// pCtx->pDecoderStatistics->iSubSpsNoExistNalNum++;
		pCtx->iErrorCode|=dsNoParamSets;
		return NULL;
	}
	pCtx->sSpsPpsCtx.iSubSpsErrorIgnored=0;

	switch(pNalUnitHeader->eNalUnitType){
		case NAL_UNIT_AU_DELIMITER:
		case NAL_UNIT_SEI:
			if(pCtx->pAccessUnitList->uiAvailUnitsNum>0){
				pCtx->pAccessUnitList->uiEndPos=pCtx->pAccessUnitList->uiAvailUnitsNum-1;
				pCtx->bAuReadyFlag=true;
			}
			break;

		case NAL_UNIT_PREFIX:
			pCurNal=&pCtx->sSpsPpsCtx.sPrefixNal;
			pCurNal->uiTimeStamp=pCtx->uiTimeStamp;

			if(iNalSize<NAL_UNIT_HEADER_EXT_SIZE){
				SAccessUnit* pCurAu=pCtx->pAccessUnitList;
				uint32_t uiAvailNalNum=pCurAu->uiAvailUnitsNum;

				if(uiAvailNalNum>0){
					pCurAu->uiEndPos=uiAvailNalNum-1;
					if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
						pCtx->bAuReadyFlag=true;
					}
				}
				pCurNal->sNalData.sPrefixNal.bPrefixNalCorrectFlag=false;
				pCtx->iErrorCode|=dsBitstreamError;
				return NULL;
			}

			DecodeNalHeaderExt(pCurNal,pNal);
			if((pCurNal->sNalHeaderExt.uiQualityId!=0) || (pCurNal->sNalHeaderExt.bUseRefBasePicFlag!=0)){
				uprintf("ParseNalHeader() in Prefix Nal Unit:uiQualityId (%d) !=0,bUseRefBasePicFlag (%d) !=0,not supported!",pCurNal->sNalHeaderExt.uiQualityId,pCurNal->sNalHeaderExt.bUseRefBasePicFlag);
				SAccessUnit* pCurAu=pCtx->pAccessUnitList;
				uint32_t uiAvailNalNum=pCurAu->uiAvailUnitsNum;

				if(uiAvailNalNum>0){
					pCurAu->uiEndPos=uiAvailNalNum-1;
					if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
						pCtx->bAuReadyFlag=true;
					}
				}
				pCurNal->sNalData.sPrefixNal.bPrefixNalCorrectFlag=false;
				pCtx->iErrorCode|=dsBitstreamError;
				return NULL;
			}

			pNal+=NAL_UNIT_HEADER_EXT_SIZE;
			iNalSize-=NAL_UNIT_HEADER_EXT_SIZE;
			*pConsumedBytes+=NAL_UNIT_HEADER_EXT_SIZE;

			pCurNal->sNalHeaderExt.sNalUnitHeader.uiForbiddenZeroBit=pNalUnitHeader->uiForbiddenZeroBit;
			pCurNal->sNalHeaderExt.sNalUnitHeader.uiNalRefIdc=pNalUnitHeader->uiNalRefIdc;
			pCurNal->sNalHeaderExt.sNalUnitHeader.eNalUnitType=pNalUnitHeader->eNalUnitType;
			if(pNalUnitHeader->uiNalRefIdc!=0){
				pBs=&pCtx->sBs;
				iBitSize=(iNalSize<<3)-BsGetTrailingBits(pNal+iNalSize-1);		// convert into bit

				iErr=DecInitBits(pBs,pNal,iBitSize);
				if(iErr){
					uprintf("NAL_UNIT_PREFIX: DecInitBits() fail due invalid access.");
					pCtx->iErrorCode|=dsBitstreamError;
					return NULL;
				}
				ParsePrefixNalUnit(pCtx,pBs);
			}
			pCurNal->sNalData.sPrefixNal.bPrefixNalCorrectFlag=true;

			break;
		case NAL_UNIT_CODED_SLICE_EXT:
			bExtensionFlag=true;
		case NAL_UNIT_CODED_SLICE:
		case NAL_UNIT_CODED_SLICE_IDR:
		{
			SAccessUnit* pCurAu=NULL;
			uint32_t uiAvailNalNum;
			pCurNal=MemGetNextNal(&pCtx->pAccessUnitList);
			if(NULL==pCurNal){
				uprintf("MemGetNextNal() fail due out of memory.");
				pCtx->iErrorCode|=dsOutOfMemory;
				return NULL;
			}
			pCurNal->uiTimeStamp=pCtx->uiTimeStamp;
			pCurNal->sNalHeaderExt.sNalUnitHeader.uiForbiddenZeroBit=pNalUnitHeader->uiForbiddenZeroBit;
			pCurNal->sNalHeaderExt.sNalUnitHeader.uiNalRefIdc=pNalUnitHeader->uiNalRefIdc;
			pCurNal->sNalHeaderExt.sNalUnitHeader.eNalUnitType=pNalUnitHeader->eNalUnitType;
			pCurAu=pCtx->pAccessUnitList;
			uiAvailNalNum=pCurAu->uiAvailUnitsNum;


			if(pNalUnitHeader->eNalUnitType==NAL_UNIT_CODED_SLICE_EXT){
				if(iNalSize<NAL_UNIT_HEADER_EXT_SIZE){
					ForceClearCurrentNal(pCurAu);

					if(uiAvailNalNum>1){
						pCurAu->uiEndPos=uiAvailNalNum-2;
						if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
							pCtx->bAuReadyFlag=true;
						}
					}
					pCtx->iErrorCode|=dsBitstreamError;
					return NULL;
				}

				DecodeNalHeaderExt(pCurNal,pNal);
				if(pCurNal->sNalHeaderExt.uiQualityId!=0 || pCurNal->sNalHeaderExt.bUseRefBasePicFlag){
					if(pCurNal->sNalHeaderExt.uiQualityId!=0)
						uprintf("ParseNalHeader():uiQualityId (%d) !=0,MGS not supported!",pCurNal->sNalHeaderExt.uiQualityId);
					if(pCurNal->sNalHeaderExt.bUseRefBasePicFlag!=0)
						uprintf("ParseNalHeader():bUseRefBasePicFlag (%d) !=0,MGS not supported!",pCurNal->sNalHeaderExt.bUseRefBasePicFlag);

					ForceClearCurrentNal(pCurAu);

					if(uiAvailNalNum>1){
						pCurAu->uiEndPos=uiAvailNalNum-2;
						if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
							pCtx->bAuReadyFlag=true;
						}
					}
					pCtx->iErrorCode|=dsBitstreamError;
					return NULL;
				}
				pNal+=NAL_UNIT_HEADER_EXT_SIZE;
				iNalSize-=NAL_UNIT_HEADER_EXT_SIZE;
				*pConsumedBytes+=NAL_UNIT_HEADER_EXT_SIZE;

			}else{
				if(NAL_UNIT_PREFIX==pCtx->sSpsPpsCtx.sPrefixNal.sNalHeaderExt.sNalUnitHeader.eNalUnitType){
					if(pCtx->sSpsPpsCtx.sPrefixNal.sNalData.sPrefixNal.bPrefixNalCorrectFlag){
						PrefetchNalHeaderExtSyntax(pCtx,pCurNal,&pCtx->sSpsPpsCtx.sPrefixNal);
					}
				}
				pCurNal->sNalHeaderExt.bIdrFlag=(NAL_UNIT_CODED_SLICE_IDR==pNalUnitHeader->eNalUnitType) ? true :false;		// SHOULD update this flag for AVC if no prefix NAL
				pCurNal->sNalHeaderExt.iNoInterLayerPredFlag=1;
			}

			pBs=&pCurAu->pNalUnitsList[uiAvailNalNum-1]->sNalData.sVclNal.sSliceBitsRead;
			iBitSize=(iNalSize<<3)-BsGetTrailingBits(pNal+iNalSize-1);		// convert into bit
			iErr=DecInitBits(pBs,pNal,iBitSize);
			if(iErr){
				ForceClearCurrentNal(pCurAu);
				if(uiAvailNalNum>1){
					pCurAu->uiEndPos=uiAvailNalNum-2;
					if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
						pCtx->bAuReadyFlag=true;
					}
				}
				uprintf("NAL_UNIT_CODED_SLICE: DecInitBits() fail due invalid access.");
				pCtx->iErrorCode|=dsBitstreamError;
				return NULL;
			}
			iErr=ParseSliceHeaderSyntaxs(pCtx,pBs,bExtensionFlag);
			if(iErr!=ERR_NONE){
				if((uiAvailNalNum==1) && (pCurNal->sNalHeaderExt.bIdrFlag)){		// IDR parse error
					ResetActiveSPSForEachLayer(pCtx);
				}
				// if current NAL occur error when parsing,should clean it from pNalUnitsList
				// otherwise,when Next good NAL decoding,this corrupt NAL is considered as normal NAL and lead to decoder crash
				ForceClearCurrentNal(pCurAu);

				if(uiAvailNalNum>1){
					pCurAu->uiEndPos=uiAvailNalNum-2;
					if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE){
						pCtx->bAuReadyFlag=true;
					}
				}
				pCtx->iErrorCode|=dsBitstreamError;
				return NULL;
			}

			if((uiAvailNalNum==1)
				 && CheckNextAuNewSeq(pCtx,pCurNal,pCurNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.pSps)){
				ResetActiveSPSForEachLayer(pCtx);
			}
			if((uiAvailNalNum>1) && 
				CheckAccessUnitBoundary(pCtx,pCurAu->pNalUnitsList[uiAvailNalNum-1],pCurAu->pNalUnitsList[uiAvailNalNum-2],pCurAu->pNalUnitsList[uiAvailNalNum-1]->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.pSps)){
				pCurAu->uiEndPos=uiAvailNalNum-2;
				pCtx->bAuReadyFlag=true;
				pCtx->bNextNewSeqBegin=CheckNextAuNewSeq(pCtx,pCurNal,pCurNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.pSps);

			}
		}
		break;
		default:
			break;
	}

	return pNal;
}

// table A-1-Level limits
const SLevelLimits g_ksLevelLimits[LEVEL_NUMBER]={
	{LEVEL_1_0,1485,99,396,64,175,-256,255,2,0x7fff},/* level 1 */
 {LEVEL_1_B,1485,99,396,128,350,-256,255,2,0x7fff},/* level 1.b */
 {LEVEL_1_1,3000,396,900,192,500,-512,511,2,0x7fff},/* level 1.1 */
 {LEVEL_1_2,6000,396,2376,384,1000,-512,511,2,0x7fff},/* level 1.2 */
 {LEVEL_1_3,11880,396,2376,768,2000,-512,511,2,0x7fff},/* level 1.3 */

 {LEVEL_2_0,11880,396,2376,2000,2000,-512,511,2,0x7fff},/* level 2 */
 {LEVEL_2_1,19800,792,4752,4000,4000,-1024,1023,2,0x7fff},/* level 2.1 */
 {LEVEL_2_2,20250,1620,8100,4000,4000,-1024,1023,2,0x7fff},/* level 2.2 */

 {LEVEL_3_0,40500,1620,8100,10000,10000,-1024,1023,2,32},/* level 3 */
 {LEVEL_3_1,108000,3600,18000,14000,14000,-2048,2047,4,16},/* level 3.1 */
 {LEVEL_3_2,216000,5120,20480,20000,20000,-2048,2047,4,16},/* level 3.2 */

 {LEVEL_4_0,245760,8192,32768,20000,25000,-2048,2047,4,16},/* level 4 */
 {LEVEL_4_1,245760,8192,32768,50000,62500,-2048,2047,2,16},/* level 4.1 */
 {LEVEL_4_2,522240,8704,34816,50000,62500,-2048,2047,2,16},/* level 4.2 */

 {LEVEL_5_0,589824,22080,110400,135000,135000,-2048,2047,2,16},/* level 5 */
 {LEVEL_5_1,983040,36864,184320,240000,240000,-2048,2047,2,16},/* level 5.1 */
 {LEVEL_5_2,2073600,36864,184320,240000,240000,-2048,2047,2,16} /* level 5.2 */
};

const SLevelLimits* GetLevelLimits(int32_t iLevelIdx,bool bConstraint3){
	switch(iLevelIdx){
		case 9:
			return &g_ksLevelLimits[1];
		case 10:
			return &g_ksLevelLimits[0];
		case 11:
			if(bConstraint3)
				return &g_ksLevelLimits[1];
			else
				return &g_ksLevelLimits[2];
		case 12:
			return &g_ksLevelLimits[3];
		case 13:
			return &g_ksLevelLimits[4];
		case 20:
			return &g_ksLevelLimits[5];
		case 21:
			return &g_ksLevelLimits[6];
		case 22:
			return &g_ksLevelLimits[7];
		case 30:
			return &g_ksLevelLimits[8];
		case 31:
			return &g_ksLevelLimits[9];
		case 32:
			return &g_ksLevelLimits[10];
		case 40:
			return &g_ksLevelLimits[11];
		case 41:
			return &g_ksLevelLimits[12];
		case 42:
			return &g_ksLevelLimits[13];
		case 50:
			return &g_ksLevelLimits[14];
		case 51:
			return &g_ksLevelLimits[15];
		case 52:
			return &g_ksLevelLimits[16];
		default:
			return NULL;
	}
	return NULL;
}

// default scaling list matrix value of 4x4
const uint8_t g_kuiDequantScaling4x4Default[2][16]={
	{6,13,20,28,13,20,28,32,20,28,32,37,28,32,37,42},
 {10,14,20,24,14,20,24,27,20,24,27,30,24,27,30,34}
};

// default scaling list matrix value of 8x8
const uint8_t g_kuiDequantScaling8x8Default[2][64]={
	{6,10,13,16,18,23,25,27,10,11,16,18,23,25,27,29,
	13,16,18,23,25,27,29,31,
	16,18,23,25,27,29,31,33,
	18,23,25,27,29,31,33,36,
	23,25,27,29,31,33,36,38,
	25,27,29,31,33,36,38,40,
	27,29,31,33,36,38,40,42},
	{9,13,15,17,19,21,22,24,
	13,13,17,19,21,22,24,25,
	15,17,19,21,22,24,25,27,
	17,19,21,22,24,25,27,28,
	19,21,22,24,25,27,28,30,
	21,22,24,25,27,28,30,32,
	22,24,25,27,28,30,32,33,
	24,25,27,28,30,32,33,35}
};

#define SPS_LOG2_MAX_FRAME_NUM_MINUS4_MAX 12
#define SPS_LOG2_MAX_PIC_ORDER_CNT_LSB_MINUS4_MAX 12
#define SPS_NUM_REF_FRAMES_IN_PIC_ORDER_CNT_CYCLE_MAX 255
#define SPS_MAX_NUM_REF_FRAMES_MAX 16
#define PPS_PIC_INIT_QP_QS_MIN 0
#define PPS_PIC_INIT_QP_QS_MAX 51
#define PPS_CHROMA_QP_INDEX_OFFSET_MIN -12
#define PPS_CHROMA_QP_INDEX_OFFSET_MAX 12
#define SCALING_LIST_DELTA_SCALE_MAX 127
#define SCALING_LIST_DELTA_SCALE_MIN -128

// brief to parse scalinglist message payload
// param pps sps scaling list matrix message to be parsed output
// param pBsAux bitstream reader auxiliary
// return 0-successed
// 1-failed
// note Call it in case scaling list matrix present at sps or pps level
int32_t SetScalingListValue(uint8_t* pScalingList,int iScalingListNum,bool* bUseDefaultScalingMatrixFlag,SBitStringAux* pBsAux){		// reserved Sei_Msg type
	int iLastScale=8;
	int iNextScale=8;
	int iDeltaScale;
	int32_t iCode;
	int32_t iIdx;
	for(int j=0; j<iScalingListNum; j++){
		if(iNextScale!=0){
			WELS_READ_VERIFY(BsGetSe(pBsAux,&iCode));
			WELS_CHECK_SE_BOTH_ERROR_NOLOG(iCode,SCALING_LIST_DELTA_SCALE_MIN,SCALING_LIST_DELTA_SCALE_MAX,"DeltaScale",
											ERR_SCALING_LIST_DELTA_SCALE);
			iDeltaScale=iCode;
			iNextScale=(iLastScale+iDeltaScale+256)%256;
			*bUseDefaultScalingMatrixFlag=(j==0 && iNextScale==0);
			if(*bUseDefaultScalingMatrixFlag)
				break;
		}
		iIdx=iScalingListNum==16 ? g_kuiZigzagScan[j] : g_kuiZigzagScan8x8[j];
		pScalingList[iIdx]=(iNextScale==0) ? iLastScale : iNextScale;
		iLastScale=pScalingList[iIdx];
	}
	return ERR_NONE;
}

int32_t ParseScalingList(SSps* pSps,SBitStringAux* pBs,bool bPPS,const bool kbTrans8x8ModeFlag,bool* pScalingListPresentFlag,uint8_t(*iScalingList4x4)[16],uint8_t(*iScalingList8x8)[64]){
	uint32_t uiScalingListNum;
	uint32_t uiCode;

	bool bUseDefaultScalingMatrixFlag4x4=false;
	bool bUseDefaultScalingMatrixFlag8x8=false;
	bool bInit=false;
	const uint8_t* defaultScaling[4];

	if(!bPPS){		// sps scaling_list
		uiScalingListNum=(pSps->uiChromaFormatIdc!=3) ? 8 : 12;
	}else{		// pps scaling_list
		uiScalingListNum=6+(int32_t)kbTrans8x8ModeFlag*((pSps->uiChromaFormatIdc!=3) ? 2 : 6);
		bInit=pSps->bSeqScalingMatrixPresentFlag;
	}

	// Init default_scaling_list value for sps or pps
	defaultScaling[0]=bInit ? pSps->iScalingList4x4[0] : g_kuiDequantScaling4x4Default[0];
	defaultScaling[1]=bInit ? pSps->iScalingList4x4[3] : g_kuiDequantScaling4x4Default[1];
	defaultScaling[2]=bInit ? pSps->iScalingList8x8[0] : g_kuiDequantScaling8x8Default[0];
	defaultScaling[3]=bInit ? pSps->iScalingList8x8[1] : g_kuiDequantScaling8x8Default[1];

	for(unsigned int i=0; i<uiScalingListNum; i++){
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));
		pScalingListPresentFlag[i]=!!uiCode;
		if(!!uiCode){
			if(i<6){// 4x4 scaling list
				WELS_READ_VERIFY(SetScalingListValue(iScalingList4x4[i],16,&bUseDefaultScalingMatrixFlag4x4,pBs));
				if(bUseDefaultScalingMatrixFlag4x4){
					bUseDefaultScalingMatrixFlag4x4=false;
					memcpy(iScalingList4x4[i],g_kuiDequantScaling4x4Default[i/3],sizeof(uint8_t)*16);
				}


			}else{
				WELS_READ_VERIFY(SetScalingListValue(iScalingList8x8[i-6],64,&bUseDefaultScalingMatrixFlag8x8,pBs));

				if(bUseDefaultScalingMatrixFlag8x8){
					bUseDefaultScalingMatrixFlag8x8=false;
					memcpy(iScalingList8x8[i-6],g_kuiDequantScaling8x8Default[(i-6)&1],sizeof(uint8_t)*64);
				}
			}

		}else{
			if(i<6){
				if((i!=0) && (i!=3))
					memcpy(iScalingList4x4[i],iScalingList4x4[i-1],sizeof(uint8_t)*16);
				else
					memcpy(iScalingList4x4[i],defaultScaling[i/3],sizeof(uint8_t)*16);

			}else{
				if((i==6) || (i==7))
					memcpy(iScalingList8x8[i-6],defaultScaling[(i&1)+2],sizeof(uint8_t)*64);
				else
					memcpy(iScalingList8x8[i-6],iScalingList8x8[i-8],sizeof(uint8_t)*64);

			}
		}
	}
	return ERR_NONE;

}

struct sSar{
	uint32_t uiWidth;
	uint32_t uiHeight;
} ;
static const sSar g_ksVuiSampleAspectRatio[17]={		// Table E-1
	{0,0},{1,1},{12,11},{10,11},{16,11},// 0~4
 {40,33},{24,11},{20,11},{32,11},{80,33},// 5~9
 {18,11},{15,11},{64,33},{160,99},{4,3},// 10~14
 {3,2},{2,1} 	// 15~16
};


#define VUI_MAX_CHROMA_LOG_TYPE_TOP_BOTTOM_FIELD_MAX 5
#define VUI_NUM_UNITS_IN_TICK_MIN 1
#define VUI_TIME_SCALE_MIN 1
#define VUI_MAX_BYTES_PER_PIC_DENOM_MAX 16
#define VUI_MAX_BITS_PER_MB_DENOM_MAX 16
#define VUI_LOG2_MAX_MV_LENGTH_HOR_MAX 16
#define VUI_LOG2_MAX_MV_LENGTH_VER_MAX 16
#define VUI_MAX_DEC_FRAME_BUFFERING_MAX 16
int32_t ParseVui(SDecoderContext* pCtx,SSps* pSps,SBitStringAux* pBsAux){
	uint32_t uiCode;
	SVui* pVui=&pSps->sVui;
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// aspect_ratio_info_present_flag
	pVui->bAspectRatioInfoPresentFlag=!!uiCode;
	if(pSps->sVui.bAspectRatioInfoPresentFlag){
		WELS_READ_VERIFY(BsGetBits(pBsAux,8,&uiCode));		// aspect_ratio_idc
		pVui->uiAspectRatioIdc=uiCode;
		if(pVui->uiAspectRatioIdc<17){
			pVui->uiSarWidth=g_ksVuiSampleAspectRatio[pVui->uiAspectRatioIdc].uiWidth;
			pVui->uiSarHeight=g_ksVuiSampleAspectRatio[pVui->uiAspectRatioIdc].uiHeight;
		}else
		if(pVui->uiAspectRatioIdc==EXTENDED_SAR){
			WELS_READ_VERIFY(BsGetBits(pBsAux,16,&uiCode));		// sar_width
			pVui->uiSarWidth=uiCode;
			WELS_READ_VERIFY(BsGetBits(pBsAux,16,&uiCode));		// sar_height
			pVui->uiSarHeight=uiCode;
		}
	}
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// overscan_info_present_flag
	pVui->bOverscanInfoPresentFlag=!!uiCode;
	if(pVui->bOverscanInfoPresentFlag){
		WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// overscan_appropriate_flag
		pVui->bOverscanAppropriateFlag=!!uiCode;
	}
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// video_signal_type_present_flag
	pVui->bVideoSignalTypePresentFlag=!!uiCode;
	if(pVui->bVideoSignalTypePresentFlag){
		WELS_READ_VERIFY(BsGetBits(pBsAux,3,&uiCode));		// video_format
		pVui->uiVideoFormat=uiCode;
		WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// video_full_range_flag
		pVui->bVideoFullRangeFlag=!!uiCode;
		WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// colour_description_present_flag
		pVui->bColourDescripPresentFlag=!!uiCode;
		if(pVui->bColourDescripPresentFlag){
			WELS_READ_VERIFY(BsGetBits(pBsAux,8,&uiCode));		// colour_primaries
			pVui->uiColourPrimaries=uiCode;
			WELS_READ_VERIFY(BsGetBits(pBsAux,8,&uiCode));		// transfer_characteristics
			pVui->uiTransferCharacteristics=uiCode;
			WELS_READ_VERIFY(BsGetBits(pBsAux,8,&uiCode));		// matrix_coefficients
			pVui->uiMatrixCoeffs=uiCode;
		}
	}
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// chroma_loc_info_present_flag
	pVui->bChromaLocInfoPresentFlag=!!uiCode;
	if(pVui->bChromaLocInfoPresentFlag){
		WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// chroma_sample_loc_type_top_field
		pVui->uiChromaSampleLocTypeTopField=uiCode;
		WELS_CHECK_SE_UPPER_WARNING(pVui->uiChromaSampleLocTypeTopField,VUI_MAX_CHROMA_LOG_TYPE_TOP_BOTTOM_FIELD_MAX,"chroma_sample_loc_type_top_field");
		WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// chroma_sample_loc_type_bottom_field
		pVui->uiChromaSampleLocTypeBottomField=uiCode;
		WELS_CHECK_SE_UPPER_WARNING(pVui->uiChromaSampleLocTypeBottomField,VUI_MAX_CHROMA_LOG_TYPE_TOP_BOTTOM_FIELD_MAX,"chroma_sample_loc_type_bottom_field");
	}
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// timing_info_present_flag
	pVui->bTimingInfoPresentFlag=!!uiCode;
	if(pVui->bTimingInfoPresentFlag){
		uint32_t uiTmp=0;
		WELS_READ_VERIFY(BsGetBits(pBsAux,16,&uiCode));		// num_units_in_tick
		uiTmp=(uiCode<<16);
		WELS_READ_VERIFY(BsGetBits(pBsAux,16,&uiCode));		// num_units_in_tick
		uiTmp|=uiCode;
		pVui->uiNumUnitsInTick=uiTmp;
		WELS_CHECK_SE_LOWER_WARNING(pVui->uiNumUnitsInTick,VUI_NUM_UNITS_IN_TICK_MIN,"num_units_in_tick");
		WELS_READ_VERIFY(BsGetBits(pBsAux,16,&uiCode));		// time_scale
		uiTmp=(uiCode<<16);
		WELS_READ_VERIFY(BsGetBits(pBsAux,16,&uiCode));		// time_scale
		uiTmp|=uiCode;
		pVui->uiTimeScale=uiTmp;
		WELS_CHECK_SE_LOWER_WARNING(pVui->uiNumUnitsInTick,VUI_TIME_SCALE_MIN,"time_scale");
		WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// fixed_frame_rate_flag
		pVui->bFixedFrameRateFlag=!!uiCode;
	}
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// nal_hrd_parameters_present_flag
	pVui->bNalHrdParamPresentFlag=!!uiCode;
	if(pVui->bNalHrdParamPresentFlag){		// Add HRD parse. the values are not being used though.
#ifdef _PARSE_NALHRD_VCLHRD_PARAMS_
		int32_t cpb_cnt_minus1=BsGetUe(pBsAux,&uiCode);
		/*bit_rate_scale=*/BsGetBits(pBsAux,4,&uiCode);
		/*cpb_size_scale=*/BsGetBits(pBsAux,4,&uiCode);
		for(int32_t i=0; i<=cpb_cnt_minus1; i++){
			/*bit_rate_value_minus1[i]=*/BsGetUe(pBsAux,&uiCode);
			/*cpb_size_value_minus1[i]=*/BsGetUe(pBsAux,&uiCode);
			/*cbr_flag[i]=*/BsGetOneBit(pBsAux,&uiCode);
		}
		/*initial_cpb_removal_delay_length_minus1=*/BsGetBits(pBsAux,5,&uiCode);
		/*cpb_removal_delay_length_minus1=*/BsGetBits(pBsAux,5,&uiCode);
		/*dpb_output_delay_length_minus1=*/BsGetBits(pBsAux,5,&uiCode);
		/*time_offset_length=*/BsGetBits(pBsAux,5,&uiCode);
#else
		uprintf("nal_hrd_parameters_present_flag=1 not supported.\n");
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_VUI_HRD);
#endif
	}
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// vcl_hrd_parameters_present_flag
	pVui->bVclHrdParamPresentFlag=!!uiCode;
	if(pVui->bVclHrdParamPresentFlag){// Add HRD parse. the values are not being used though.
#ifdef _PARSE_NALHRD_VCLHRD_PARAMS_
		int32_t cpb_cnt_minus1=BsGetUe(pBsAux,&uiCode);
		/*bit_rate_scale=*/BsGetBits(pBsAux,4,&uiCode);
		/*cpb_size_scale=*/BsGetBits(pBsAux,4,&uiCode);
		for(int32_t i=0; i<=cpb_cnt_minus1; i++){
			/*bit_rate_value_minus1[i]=*/BsGetUe(pBsAux,&uiCode);
			/*cpb_size_value_minus1[i]=*/BsGetUe(pBsAux,&uiCode);
			/*cbr_flag[i]=*/BsGetOneBit(pBsAux,&uiCode);
		}
		/*initial_cpb_removal_delay_length_minus1=*/BsGetBits(pBsAux,5,&uiCode);
		/*cpb_removal_delay_length_minus1=*/BsGetBits(pBsAux,5,&uiCode);
		/*dpb_output_delay_length_minus1=*/BsGetBits(pBsAux,5,&uiCode);
		/*time_offset_length=*/BsGetBits(pBsAux,5,&uiCode);
#else
		uprintf("vcl_hrd_parameters_present_flag=1 not supported.");
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_VUI_HRD);
#endif
	}
#ifdef _PARSE_NALHRD_VCLHRD_PARAMS_
	if(pVui->bNalHrdParamPresentFlag|pVui->bVclHrdParamPresentFlag){
		/*low_delay_hrd_flag=*/BsGetOneBit(pBsAux,&uiCode);
	}
#endif
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// pic_struct_present_flag
	pVui->bPicStructPresentFlag=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// bitstream_restriction_flag
	pVui->bBitstreamRestrictionFlag=!!uiCode;
	if(pVui->bBitstreamRestrictionFlag){
		WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// motion_vectors_over_pic_boundaries_flag
		pVui->bMotionVectorsOverPicBoundariesFlag=!!uiCode;
		WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// max_bytes_per_pic_denom
		pVui->uiMaxBytesPerPicDenom=uiCode;
		WELS_CHECK_SE_UPPER_WARNING(pVui->uiMaxBytesPerPicDenom,VUI_MAX_BYTES_PER_PIC_DENOM_MAX,"max_bytes_per_pic_denom");
		WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// max_bits_per_mb_denom
		pVui->uiMaxBitsPerMbDenom=uiCode;
		WELS_CHECK_SE_UPPER_WARNING(pVui->uiMaxBitsPerMbDenom,VUI_MAX_BITS_PER_MB_DENOM_MAX,"max_bits_per_mb_denom");
		WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// log2_max_mv_length_horizontal
		pVui->uiLog2MaxMvLengthHorizontal=uiCode;
		WELS_CHECK_SE_UPPER_WARNING(pVui->uiLog2MaxMvLengthHorizontal,VUI_LOG2_MAX_MV_LENGTH_HOR_MAX,"log2_max_mv_length_horizontal");
		WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// log2_max_mv_length_vertical
		pVui->uiLog2MaxMvLengthVertical=uiCode;
		WELS_CHECK_SE_UPPER_WARNING(pVui->uiLog2MaxMvLengthVertical,VUI_LOG2_MAX_MV_LENGTH_VER_MAX,"log2_max_mv_length_vertical");
		WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// max_num_reorder_frames
		pVui->uiMaxNumReorderFrames=uiCode;
		WELS_CHECK_SE_UPPER_WARNING(pVui->uiMaxNumReorderFrames,VUI_MAX_DEC_FRAME_BUFFERING_MAX,"max_num_reorder_frames");
		WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// max_dec_frame_buffering
		pVui->uiMaxDecFrameBuffering=uiCode;
		WELS_CHECK_SE_UPPER_WARNING(pVui->uiMaxDecFrameBuffering,VUI_MAX_DEC_FRAME_BUFFERING_MAX,"max_num_reorder_frames");
	}
	return ERR_NONE;
}

#define SUBSET_SPS_SEQ_SCALED_REF_LAYER_LEFT_OFFSET_MIN -32768
#define SUBSET_SPS_SEQ_SCALED_REF_LAYER_LEFT_OFFSET_MAX 32767
#define SUBSET_SPS_SEQ_SCALED_REF_LAYER_TOP_OFFSET_MIN -32768
#define SUBSET_SPS_SEQ_SCALED_REF_LAYER_TOP_OFFSET_MAX 32767
#define SUBSET_SPS_SEQ_SCALED_REF_LAYER_RIGHT_OFFSET_MIN -32768
#define SUBSET_SPS_SEQ_SCALED_REF_LAYER_RIGHT_OFFSET_MAX 32767
#define SUBSET_SPS_SEQ_SCALED_REF_LAYER_BOTTOM_OFFSET_MIN -32768
#define SUBSET_SPS_SEQ_SCALED_REF_LAYER_BOTTOM_OFFSET_MAX 32767

int32_t DecodeSpsSvcExt(SDecoderContext* pCtx,SSubsetSps* pSpsExt,SBitStringAux* pBs){
	SSpsSvcExt* pExt=NULL;
	uint32_t uiCode;
	int32_t iCode;

	pExt=&pSpsExt->sSpsSvcExt;

	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// inter_layer_deblocking_filter_control_present_flag
	pExt->bInterLayerDeblockingFilterCtrlPresentFlag=!!uiCode;
	WELS_READ_VERIFY(BsGetBits(pBs,2,&uiCode));		// extended_spatial_scalability_idc
	pExt->uiExtendedSpatialScalability=uiCode;
	if(pExt->uiExtendedSpatialScalability>2){
		uprintf("DecodeSpsSvcExt():extended_spatial_scalability (%d) !=0,ESS not supported!",pExt->uiExtendedSpatialScalability);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_ESS);
	}

	pExt->uiChromaPhaseXPlus1Flag=
		0;		// FIXME: Incoherent with JVT X201 standard (=1),but conformance to JSVM (=0) implementation.
	pExt->uiChromaPhaseYPlus1=1;

	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// chroma_phase_x_plus1_flag
	pExt->uiChromaPhaseXPlus1Flag=uiCode;
	WELS_READ_VERIFY(BsGetBits(pBs,2,&uiCode));		// chroma_phase_y_plus1
	pExt->uiChromaPhaseYPlus1=uiCode;

	pExt->uiSeqRefLayerChromaPhaseXPlus1Flag=pExt->uiChromaPhaseXPlus1Flag;
	pExt->uiSeqRefLayerChromaPhaseYPlus1=pExt->uiChromaPhaseYPlus1;
	memset(&pExt->sSeqScaledRefLayer,0,sizeof(SPosOffset));

	if(pExt->uiExtendedSpatialScalability==1){
		SPosOffset* const kpPos=&pExt->sSeqScaledRefLayer;
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// seq_ref_layer_chroma_phase_x_plus1_flag
		pExt->uiSeqRefLayerChromaPhaseXPlus1Flag=uiCode;
		WELS_READ_VERIFY(BsGetBits(pBs,2,&uiCode));		// seq_ref_layer_chroma_phase_y_plus1
		pExt->uiSeqRefLayerChromaPhaseYPlus1=uiCode;

		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// seq_scaled_ref_layer_left_offset
		kpPos->iLeftOffset=iCode;
		WELS_CHECK_SE_BOTH_WARNING(kpPos->iLeftOffset,SUBSET_SPS_SEQ_SCALED_REF_LAYER_LEFT_OFFSET_MIN,SUBSET_SPS_SEQ_SCALED_REF_LAYER_LEFT_OFFSET_MAX,"seq_scaled_ref_layer_left_offset");
		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// seq_scaled_ref_layer_top_offset
		kpPos->iTopOffset=iCode;
		WELS_CHECK_SE_BOTH_WARNING(kpPos->iTopOffset,SUBSET_SPS_SEQ_SCALED_REF_LAYER_TOP_OFFSET_MIN,SUBSET_SPS_SEQ_SCALED_REF_LAYER_TOP_OFFSET_MAX,"seq_scaled_ref_layer_top_offset");
		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// seq_scaled_ref_layer_right_offset
		kpPos->iRightOffset=iCode;
		WELS_CHECK_SE_BOTH_WARNING(kpPos->iRightOffset,SUBSET_SPS_SEQ_SCALED_REF_LAYER_RIGHT_OFFSET_MIN,SUBSET_SPS_SEQ_SCALED_REF_LAYER_RIGHT_OFFSET_MAX,"seq_scaled_ref_layer_right_offset");
		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// seq_scaled_ref_layer_bottom_offset
		kpPos->iBottomOffset=iCode;
		WELS_CHECK_SE_BOTH_WARNING(kpPos->iBottomOffset,SUBSET_SPS_SEQ_SCALED_REF_LAYER_BOTTOM_OFFSET_MIN,SUBSET_SPS_SEQ_SCALED_REF_LAYER_BOTTOM_OFFSET_MAX,"seq_scaled_ref_layer_bottom_offset");
	}
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// seq_tcoeff_level_prediction_flag
	pExt->bSeqTCoeffLevelPredFlag=!!uiCode;
	pExt->bAdaptiveTCoeffLevelPredFlag=false;
	if(pExt->bSeqTCoeffLevelPredFlag){
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// adaptive_tcoeff_level_prediction_flag
		pExt->bAdaptiveTCoeffLevelPredFlag=!!uiCode;
	}
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// slice_header_restriction_flag
	pExt->bSliceHeaderRestrictionFlag=!!uiCode;
	return ERR_NONE;
}

bool CheckSpsActive(SDecoderContext* pCtx,SSps* pSps,bool bUseSubsetFlag){
	for(int i=0; i<MAX_LAYER_NUM; i++){
		if(pCtx->sSpsPpsCtx.pActiveLayerSps[i]==pSps)
			return true;
	}
	// Pre-active,will be used soon
	if(bUseSubsetFlag){
		if(pSps->iMbWidth>0 && pSps->iMbHeight>0 && pCtx->sSpsPpsCtx.bSubspsAvailFlags[pSps->iSpsId]){
			if(pCtx->iTotalNumMbRec>0){
				return true;
			}
			if(pCtx->pAccessUnitList->uiAvailUnitsNum>0){
				int i=0,iNum=(int32_t)pCtx->pAccessUnitList->uiAvailUnitsNum;
				while(i<iNum){
					SNalUnit* pNalUnit=pCtx->pAccessUnitList->pNalUnitsList[i];
					if(pNalUnit->sNalData.sVclNal.bSliceHeaderExtFlag){		// ext data
						SSps* pNextUsedSps=pNalUnit->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.pSps;
						if(pNextUsedSps->iSpsId==pSps->iSpsId)
							return true;
					}
					++i;
				}
			}
		}
	}else{
		if(pSps->iMbWidth>0 && pSps->iMbHeight>0 && pCtx->sSpsPpsCtx.bSpsAvailFlags[pSps->iSpsId]){
			if(pCtx->iTotalNumMbRec>0){
				return true;
			}
			if(pCtx->pAccessUnitList->uiAvailUnitsNum>0){
				int i=0,iNum=(int32_t)pCtx->pAccessUnitList->uiAvailUnitsNum;
				while(i<iNum){
					SNalUnit* pNalUnit=pCtx->pAccessUnitList->pNalUnitsList[i];
					if(!pNalUnit->sNalData.sVclNal.bSliceHeaderExtFlag){		// non-ext data
						SSps* pNextUsedSps=pNalUnit->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader.pSps;
						if(pNextUsedSps->iSpsId==pSps->iSpsId)
							return true;
					}
					++i;
				}
			}
		}
	}
	return false;
}

// brief to parse Sequence Parameter Set (SPS)
// param pCtx Decoder context
// param pBsAux bitstream reader auxiliary
// param pPicWidth picture width current Sps represented
// param pPicHeight picture height current Sps represented
// return 0-successed
// 1-failed
// note Call it in case eNalUnitType is SPS.
int32_t ParseSps(SDecoderContext* pCtx,SBitStringAux* pBsAux,int32_t* pPicWidth,int32_t* pPicHeight,uint8_t* pSrcNal,const int32_t kSrcNalLen){
	SBitStringAux* pBs=pBsAux;
	SSubsetSps sTempSubsetSps;
	SSps* pSps=NULL;
	SSubsetSps* pSubsetSps=NULL;
	SNalUnitHeader* pNalHead=&pCtx->sCurNalHead;
	ProfileIdc uiProfileIdc;
	uint8_t uiLevelIdc;
	int32_t iSpsId;
	uint32_t uiCode;
	int32_t iCode;
	int32_t iRet=ERR_NONE;
	bool bConstraintSetFlags[6]={false};
	const bool kbUseSubsetFlag=IS_SUBSET_SPS_NAL(pNalHead->eNalUnitType);

	WELS_READ_VERIFY(BsGetBits(pBs,8,&uiCode));		// profile_idc
	uiProfileIdc=uiCode;
	if(uiProfileIdc!=PRO_BASELINE && uiProfileIdc!=PRO_MAIN && uiProfileIdc!=PRO_SCALABLE_BASELINE
		 && uiProfileIdc!=PRO_SCALABLE_HIGH
		 && uiProfileIdc!=PRO_EXTENDED && uiProfileIdc!=PRO_HIGH){
		uprintf("SPS ID can not be supported!\n");
		return false;
	}
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set0_flag
	bConstraintSetFlags[0]=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set1_flag
	bConstraintSetFlags[1]=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set2_flag
	bConstraintSetFlags[2]=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set3_flag
	bConstraintSetFlags[3]=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set4_flag
	bConstraintSetFlags[4]=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// constraint_set5_flag
	bConstraintSetFlags[5]=!!uiCode;
	WELS_READ_VERIFY(BsGetBits(pBs,2,&uiCode));		// reserved_zero_2bits,equal to 0
	WELS_READ_VERIFY(BsGetBits(pBs,8,&uiCode));		// level_idc
	uiLevelIdc=uiCode;
	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// seq_parameter_set_id
	if(uiCode>=MAX_SPS_COUNT){		// Modified to check invalid negative iSpsId,12/1/2009
		uprintf(" iSpsId is out of range! \n");
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_SPS_ID_OVERFLOW);
	}
	iSpsId=uiCode;
	pSubsetSps=&sTempSubsetSps;
	pSps=&sTempSubsetSps.sSps;
	memset(pSubsetSps,0,sizeof(SSubsetSps));
	// Use the level 5.2 for compatibility
	const SLevelLimits* pSMaxLevelLimits=GetLevelLimits(52,false);
	const SLevelLimits* pSLevelLimits=GetLevelLimits(uiLevelIdc,bConstraintSetFlags[3]);
	if(NULL==pSLevelLimits){
		uprintf("ParseSps(): level_idx (%d).\n",uiLevelIdc);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_NON_BASELINE);
	}else pSps->pSLevelLimits=pSLevelLimits;
	// syntax elements in default
	pSps->uiChromaFormatIdc=1;
	pSps->uiChromaArrayType=1;

	pSps->uiProfileIdc=uiProfileIdc;
	pSps->uiLevelIdc=uiLevelIdc;
	pSps->iSpsId=iSpsId;

	if(PRO_SCALABLE_BASELINE==uiProfileIdc || PRO_SCALABLE_HIGH==uiProfileIdc || PRO_HIGH==uiProfileIdc || PRO_HIGH10==uiProfileIdc || PRO_HIGH422==uiProfileIdc || PRO_HIGH444==uiProfileIdc || PRO_CAVLC444==uiProfileIdc || 44==uiProfileIdc){

		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// chroma_format_idc
		pSps->uiChromaFormatIdc=uiCode;
		if(pSps->uiChromaFormatIdc>1){
			uprintf("ParseSps(): chroma_format_idc (%d) <=1 supported.",pSps->uiChromaFormatIdc);
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_NON_BASELINE);

		}// To support 4:0:0; 4:2:0
		pSps->uiChromaArrayType=pSps->uiChromaFormatIdc;
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// bit_depth_luma_minus8
		if(uiCode!=0){
			uprintf("ParseSps(): bit_depth_luma (%d) Only 8 bit supported.",8+uiCode);
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_NON_BASELINE);
		}
		pSps->uiBitDepthLuma=8;

		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// bit_depth_chroma_minus8
		if(uiCode!=0){
			uprintf("ParseSps(): bit_depth_chroma (%d). Only 8 bit supported.",8+uiCode);
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_NON_BASELINE);
		}
		pSps->uiBitDepthChroma=8;

		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// qpprime_y_zero_transform_bypass_flag
		pSps->bQpPrimeYZeroTransfBypassFlag=!!uiCode;
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// seq_scaling_matrix_present_flag
		pSps->bSeqScalingMatrixPresentFlag=!!uiCode;

		if(pSps->bSeqScalingMatrixPresentFlag){
			WELS_READ_VERIFY(ParseScalingList(pSps,pBs,0,0,pSps->bSeqScalingListPresentFlag,pSps->iScalingList4x4,pSps->iScalingList8x8));
		}
	}
	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// log2_max_frame_num_minus4
	WELS_CHECK_SE_UPPER_ERROR(uiCode,SPS_LOG2_MAX_FRAME_NUM_MINUS4_MAX,"log2_max_frame_num_minus4",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_LOG2_MAX_FRAME_NUM_MINUS4));
	pSps->uiLog2MaxFrameNum=LOG2_MAX_FRAME_NUM_OFFSET+uiCode;
	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// pic_order_cnt_type
	pSps->uiPocType=uiCode;

	if(0==pSps->uiPocType){
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// log2_max_pic_order_cnt_lsb_minus4
		// log2_max_pic_order_cnt_lsb_minus4 should be in range 0 to 12,inclusive. (sec. 7.4.3)
		WELS_CHECK_SE_UPPER_ERROR(uiCode,SPS_LOG2_MAX_PIC_ORDER_CNT_LSB_MINUS4_MAX,"log2_max_pic_order_cnt_lsb_minus4",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_LOG2_MAX_PIC_ORDER_CNT_LSB_MINUS4));
		pSps->iLog2MaxPocLsb=LOG2_MAX_PIC_ORDER_CNT_LSB_OFFSET+uiCode;		// log2_max_pic_order_cnt_lsb_minus4

	}else
	if(1==pSps->uiPocType){
		int32_t i;
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// delta_pic_order_always_zero_flag
		pSps->bDeltaPicOrderAlwaysZeroFlag=!!uiCode;
		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// offset_for_non_ref_pic
		pSps->iOffsetForNonRefPic=iCode;
		WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// offset_for_top_to_bottom_field
		pSps->iOffsetForTopToBottomField=iCode;
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// num_ref_frames_in_pic_order_cnt_cycle
		WELS_CHECK_SE_UPPER_ERROR(uiCode,SPS_NUM_REF_FRAMES_IN_PIC_ORDER_CNT_CYCLE_MAX,"num_ref_frames_in_pic_order_cnt_cycle",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_NUM_REF_FRAME_IN_PIC_ORDER_CNT_CYCLE));
		pSps->iNumRefFramesInPocCycle=uiCode;
		for(i=0; i<pSps->iNumRefFramesInPocCycle; i++){
			WELS_READ_VERIFY(BsGetSe(pBs,&iCode));		// offset_for_ref_frame[ i ]
			pSps->iOffsetForRefFrame[i]=iCode;
		}
	}
	if(pSps->uiPocType>2){
		uprintf(" illegal pic_order_cnt_type: %d ! ",pSps->uiPocType);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_POC_TYPE);
	}

	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// max_num_ref_frames
	pSps->iNumRefFrames=uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// gaps_in_frame_num_value_allowed_flag
	pSps->bGapsInFrameNumValueAllowedFlag=!!uiCode;
	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// pic_width_in_mbs_minus1
	pSps->iMbWidth=PIC_WIDTH_IN_MBS_OFFSET+uiCode;
	if(pSps->iMbWidth>MAX_MB_SIZE || pSps->iMbWidth==0){
		FATAL("pic_width_in_mbs(%d) invalid!",pSps->iMbWidth);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
	}
	if(((uint64_t)pSps->iMbWidth*(uint64_t)pSps->iMbWidth)>(uint64_t)(8*pSLevelLimits->uiMaxFS)){
		if(((uint64_t)pSps->iMbWidth*(uint64_t)pSps->iMbWidth)>(uint64_t)(8*pSMaxLevelLimits->uiMaxFS)){
			FATAL("the pic_width_in_mbs exceeds the level limits!");
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
		}else{
			uprintf("the pic_width_in_mbs exceeds the level limits!");
		}
	}
	WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// pic_height_in_map_units_minus1
	pSps->iMbHeight=PIC_HEIGHT_IN_MAP_UNITS_OFFSET+uiCode;
	if(pSps->iMbHeight>MAX_MB_SIZE || pSps->iMbHeight==0){
		FATAL("pic_height_in_mbs(%d) invalid!",pSps->iMbHeight);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
	}
	if(((uint64_t)pSps->iMbHeight*(uint64_t)pSps->iMbHeight)>(uint64_t)(8*pSLevelLimits->uiMaxFS)){
		if(((uint64_t)pSps->iMbHeight*(uint64_t)pSps->iMbHeight)>(uint64_t)(8*pSMaxLevelLimits->uiMaxFS)){
			FATAL("the pic_height_in_mbs exceeds the level limits!");
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
		}else{
			uprintf("the pic_height_in_mbs exceeds the level limits!");
		}
	}
	uint64_t uiTmp64=(uint64_t)pSps->iMbWidth*(uint64_t)pSps->iMbHeight;
	if(uiTmp64>(uint64_t)pSLevelLimits->uiMaxFS){
		if(uiTmp64>(uint64_t)pSMaxLevelLimits->uiMaxFS){
			FATAL("the total count of mb exceeds the level limits!");
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_MB_SIZE);
		}else{
			uprintf("the total count of mb exceeds the level limits!");
		}
	}
	pSps->uiTotalMbCount=(uint32_t)uiTmp64;
	WELS_CHECK_SE_UPPER_ERROR(pSps->iNumRefFrames,SPS_MAX_NUM_REF_FRAMES_MAX,"max_num_ref_frames",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_MAX_NUM_REF_FRAMES));
	// here we check max_num_ref_frames
	uint32_t uiMaxDpbMbs=pSLevelLimits->uiMaxDPBMbs;
	uint32_t uiMaxDpbFrames=uiMaxDpbMbs/pSps->uiTotalMbCount;
	if(uiMaxDpbFrames>SPS_MAX_NUM_REF_FRAMES_MAX)
		uiMaxDpbFrames=SPS_MAX_NUM_REF_FRAMES_MAX;
	if((uint32_t)pSps->iNumRefFrames>uiMaxDpbFrames){
		uprintf(" max_num_ref_frames exceeds level limits!");
	}
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// frame_mbs_only_flag
	pSps->bFrameMbsOnlyFlag=!!uiCode;
	if(!pSps->bFrameMbsOnlyFlag){
		uprintf("ParseSps(): frame_mbs_only_flag (%d) not supported.",pSps->bFrameMbsOnlyFlag);
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_MBAFF);
	}
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// direct_8x8_inference_flag
	pSps->bDirect8x8InferenceFlag=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// frame_cropping_flag
	pSps->bFrameCroppingFlag=!!uiCode;
	if(pSps->bFrameCroppingFlag){
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// frame_crop_left_offset
		pSps->sFrameCrop.iLeftOffset=uiCode;
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// frame_crop_right_offset
		pSps->sFrameCrop.iRightOffset=uiCode;
		if((pSps->sFrameCrop.iLeftOffset+pSps->sFrameCrop.iRightOffset)>((int32_t)pSps->iMbWidth*16/2)){
			FATAL("frame_crop_left_offset+frame_crop_right_offset exceeds limits!");
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_CROPPING_DATA);
		}
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// frame_crop_top_offset
		pSps->sFrameCrop.iTopOffset=uiCode;
		WELS_READ_VERIFY(BsGetUe(pBs,&uiCode));		// frame_crop_bottom_offset
		pSps->sFrameCrop.iBottomOffset=uiCode;
		if((pSps->sFrameCrop.iTopOffset+pSps->sFrameCrop.iBottomOffset)>((int32_t)pSps->iMbHeight*16/2)){
			FATAL("frame_crop_top_offset+frame_crop_right_offset exceeds limits!");
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_CROPPING_DATA);
		}
	}else{
		pSps->sFrameCrop.iLeftOffset=0;		// frame_crop_left_offset
		pSps->sFrameCrop.iRightOffset=0;		// frame_crop_right_offset
		pSps->sFrameCrop.iTopOffset=0;		// frame_crop_top_offset
		pSps->sFrameCrop.iBottomOffset=0;		// frame_crop_bottom_offset
	}
	WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));		// vui_parameters_present_flag
	pSps->bVuiParamPresentFlag=!!uiCode;
	if(pSps->bVuiParamPresentFlag){
		int iRetVui=ParseVui(pCtx,pSps,pBsAux);
		if(iRetVui==GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_VUI_HRD)){
			if(kbUseSubsetFlag){		// Currently do no support VUI with HRD enable in subsetSPS
				FATAL("hrd parse in vui of subsetSPS is not supported!");
				return iRetVui;
			}
		}else{
			WELS_READ_VERIFY(iRetVui);
		}
	}
	// Check if SPS SVC extension applicated
	if(kbUseSubsetFlag && (PRO_SCALABLE_BASELINE==uiProfileIdc || PRO_SCALABLE_HIGH==uiProfileIdc)){
		if((iRet=DecodeSpsSvcExt(pCtx,pSubsetSps,pBs))!=ERR_NONE){
			return iRet;
		}
		WELS_READ_VERIFY(BsGetOneBit(pBs,&uiCode));									// svc_vui_parameters_present_flag
		pSubsetSps->bSvcVuiParamPresentFlag=!!uiCode;
		//if(pSubsetSps->bSvcVuiParamPresentFlag){}
	}
	if(PRO_SCALABLE_BASELINE==uiProfileIdc || PRO_SCALABLE_HIGH==uiProfileIdc)
		pCtx->sSpsPpsCtx.bAvcBasedFlag=false;

	*pPicWidth=pSps->iMbWidth<<4;
	*pPicHeight=pSps->iMbHeight<<4;
	SSps* pTmpSps=NULL;
	if(kbUseSubsetFlag){
		pTmpSps=&pCtx->sSpsPpsCtx.sSubsetSpsBuffer[iSpsId].sSps;
	}else{
		pTmpSps=&pCtx->sSpsPpsCtx.sSpsBuffer[iSpsId];
	}
	if(CheckSpsActive(pCtx,pTmpSps,kbUseSubsetFlag)){
		// we are overwriting the active sps,copy a temp buffer
		if(kbUseSubsetFlag){
			if(memcmp(&pCtx->sSpsPpsCtx.sSubsetSpsBuffer[iSpsId],pSubsetSps,sizeof(SSubsetSps))!=0){
				if(pCtx->pAccessUnitList->uiAvailUnitsNum>0){
					memcpy(&pCtx->sSpsPpsCtx.sSubsetSpsBuffer[MAX_SPS_COUNT],pSubsetSps,sizeof(SSubsetSps));
					pCtx->bAuReadyFlag=true;
					pCtx->pAccessUnitList->uiEndPos=pCtx->pAccessUnitList->uiAvailUnitsNum-1;
					pCtx->sSpsPpsCtx.iOverwriteFlags|=OVERWRITE_SUBSETSPS;
				}else
				if((pCtx->pSps!=NULL) && (pCtx->pSps->iSpsId==pSubsetSps->sSps.iSpsId)){
					memcpy(&pCtx->sSpsPpsCtx.sSubsetSpsBuffer[MAX_SPS_COUNT],pSubsetSps,sizeof(SSubsetSps));
					pCtx->sSpsPpsCtx.iOverwriteFlags|=OVERWRITE_SUBSETSPS;
				}else{
					memcpy(&pCtx->sSpsPpsCtx.sSubsetSpsBuffer[iSpsId],pSubsetSps,sizeof(SSubsetSps));
				}
			}
		}else{
			if(memcmp(&pCtx->sSpsPpsCtx.sSpsBuffer[iSpsId],pSps,sizeof(SSps))!=0){
				if(pCtx->pAccessUnitList->uiAvailUnitsNum>0){
					memcpy(&pCtx->sSpsPpsCtx.sSpsBuffer[MAX_SPS_COUNT],pSps,sizeof(SSps));
					pCtx->sSpsPpsCtx.iOverwriteFlags|=OVERWRITE_SPS;
					pCtx->bAuReadyFlag=true;
					pCtx->pAccessUnitList->uiEndPos=pCtx->pAccessUnitList->uiAvailUnitsNum-1;
				}else
				if((pCtx->pSps!=NULL) && (pCtx->pSps->iSpsId==pSps->iSpsId)){
					memcpy(&pCtx->sSpsPpsCtx.sSpsBuffer[MAX_SPS_COUNT],pSps,sizeof(SSps));
					pCtx->sSpsPpsCtx.iOverwriteFlags|=OVERWRITE_SPS;
				}else{
					memcpy(&pCtx->sSpsPpsCtx.sSpsBuffer[iSpsId],pSps,sizeof(SSps));
				}
			}
		}
	}else
	if(kbUseSubsetFlag){// Not overwrite active sps,just copy to final place
		memcpy(&pCtx->sSpsPpsCtx.sSubsetSpsBuffer[iSpsId],pSubsetSps,sizeof(SSubsetSps));
		pCtx->sSpsPpsCtx.bSubspsAvailFlags[iSpsId]=true;
		pCtx->sSpsPpsCtx.bSubspsExistAheadFlag=true;
	}else{
		memcpy(&pCtx->sSpsPpsCtx.sSpsBuffer[iSpsId],pSps,sizeof(SSps));
		pCtx->sSpsPpsCtx.bSpsAvailFlags[iSpsId]=true;
		pCtx->sSpsPpsCtx.bSpsExistAheadFlag=true;
	}
	return ERR_NONE;
}


// Check whether there is more rbsp data for processing
static inline bool CheckMoreRBSPData(SBitStringAux* pBsAux){
	if((pBsAux->iBits-((pBsAux->pCurBuf-pBsAux->pStartBuf-2)<<3)-pBsAux->iLeftBits)>1){
		return true;
	}else{
		return false;
	}
}

// brief to parse Picture Parameter Set (PPS)
// param pCtx Decoder context
// param pPpsList pps list
// param pBsAux bitstream reader auxiliary
// return 0-successed
// 1-failed
// note Call it in case eNalUnitType is PPS.
int32_t ParsePps(SDecoderContext* pCtx,SPps* pPpsList,SBitStringAux* pBsAux,uint8_t* pSrcNal,const int32_t kSrcNalLen){
	SPps* pPps=NULL;
	SPps sTempPps;
	uint32_t uiPpsId=0;
	uint32_t iTmp;
	uint32_t uiCode;
	int32_t iCode;

	WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));			// pic_parameter_set_id
	uiPpsId=uiCode;
	if(uiPpsId>=MAX_PPS_COUNT){
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_PPS_ID_OVERFLOW);
	}
	pPps=&sTempPps;
	memset(pPps,0,sizeof(SPps));

	pPps->iPpsId=uiPpsId;
	WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));			// seq_parameter_set_id
	pPps->iSpsId=uiCode;

	if(pPps->iSpsId>=MAX_SPS_COUNT){
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_SPS_ID_OVERFLOW);
	}

	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// entropy_coding_mode_flag
	pPps->bEntropyCodingModeFlag=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// bottom_field_pic_order_in_frame_present_flag
	pPps->bPicOrderPresentFlag=!!uiCode;

	WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));			// num_slice_groups_minus1
	pPps->uiNumSliceGroups=NUM_SLICE_GROUPS_OFFSET+uiCode;

	if(pPps->uiNumSliceGroups>MAX_SLICEGROUP_IDS){
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_SLICEGROUP);
	}

	if(pPps->uiNumSliceGroups>1){
		WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// slice_group_map_type
		pPps->uiSliceGroupMapType=uiCode;
		if(pPps->uiSliceGroupMapType>1){
			uprintf("ParsePps(): slice_group_map_type (%d): support only 0,1.",pPps->uiSliceGroupMapType);
			return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_UNSUPPORTED_FMOTYPE);
		}
		switch(pPps->uiSliceGroupMapType){
			case 0:
				for(iTmp=0; iTmp<pPps->uiNumSliceGroups; iTmp++){
					WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));		// run_length_minus1[ iGroup ]
					pPps->uiRunLength[iTmp]=RUN_LENGTH_OFFSET+uiCode;
				}
				break;
			default:
				break;
		}
	}

	WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));				// num_ref_idx_l0_default_active_minus1
	pPps->uiNumRefIdxL0Active=NUM_REF_IDX_L0_DEFAULT_ACTIVE_OFFSET+uiCode;
	WELS_READ_VERIFY(BsGetUe(pBsAux,&uiCode));				// num_ref_idx_l1_default_active_minus1
	pPps->uiNumRefIdxL1Active=NUM_REF_IDX_L1_DEFAULT_ACTIVE_OFFSET+uiCode;

	if(pPps->uiNumRefIdxL0Active>MAX_REF_PIC_COUNT || pPps->uiNumRefIdxL1Active>MAX_REF_PIC_COUNT){
		return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_REF_COUNT_OVERFLOW);
	}

	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));			// weighted_pred_flag
	pPps->bWeightedPredFlag=!!uiCode;
	WELS_READ_VERIFY(BsGetBits(pBsAux,2,&uiCode));			// weighted_bipred_idc
	pPps->uiWeightedBipredIdc=uiCode;
// weighted_bipred_idc > 0 NOT supported now,but no impact when we ignore it

	WELS_READ_VERIFY(BsGetSe(pBsAux,&iCode));				// pic_init_qp_minus26
	pPps->iPicInitQp=PIC_INIT_QP_OFFSET+iCode;
	WELS_CHECK_SE_BOTH_ERROR(pPps->iPicInitQp,PPS_PIC_INIT_QP_QS_MIN,PPS_PIC_INIT_QP_QS_MAX,"pic_init_qp_minus26+26",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_PIC_INIT_QP));
	WELS_READ_VERIFY(BsGetSe(pBsAux,&iCode));				// pic_init_qs_minus26
	pPps->iPicInitQs=PIC_INIT_QS_OFFSET+iCode;
	WELS_CHECK_SE_BOTH_ERROR(pPps->iPicInitQs,PPS_PIC_INIT_QP_QS_MIN,PPS_PIC_INIT_QP_QS_MAX,"pic_init_qs_minus26+26",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_PIC_INIT_QS));
	WELS_READ_VERIFY(BsGetSe(pBsAux,&iCode));				// chroma_qp_index_offset,cb
	pPps->iChromaQpIndexOffset[0]=iCode;
	WELS_CHECK_SE_BOTH_ERROR(pPps->iChromaQpIndexOffset[0],PPS_CHROMA_QP_INDEX_OFFSET_MIN,PPS_CHROMA_QP_INDEX_OFFSET_MAX,"chroma_qp_index_offset",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_CHROMA_QP_INDEX_OFFSET));
	pPps->iChromaQpIndexOffset[1]=pPps->iChromaQpIndexOffset[0];	// init cr qp offset
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));			// deblocking_filter_control_present_flag
	pPps->bDeblockingFilterControlPresentFlag=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));			// constrained_intra_pred_flag
	pPps->bConstainedIntraPredFlag=!!uiCode;
	WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));			// redundant_pic_cnt_present_flag
	pPps->bRedundantPicCntPresentFlag=!!uiCode;

	if(CheckMoreRBSPData(pBsAux)){
		WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// transform_8x8_mode_flag
		pPps->bTransform8x8ModeFlag=!!uiCode;
		WELS_READ_VERIFY(BsGetOneBit(pBsAux,&uiCode));		// pic_scaling_matrix_present_flag
		pPps->bPicScalingMatrixPresentFlag=!!uiCode;
		if(pPps->bPicScalingMatrixPresentFlag){
			if(pCtx->sSpsPpsCtx.bSpsAvailFlags[pPps->iSpsId]){
				WELS_READ_VERIFY(ParseScalingList(&pCtx->sSpsPpsCtx.sSpsBuffer[pPps->iSpsId],pBsAux,1,pPps->bTransform8x8ModeFlag,pPps->bPicScalingListPresentFlag,pPps->iScalingList4x4,pPps->iScalingList8x8));
			}else{
				uprintf("ParsePps(): sps_id (%d) does not exist for scaling_list. This PPS (%d) is marked as invalid.",pPps->iSpsId,pPps->iPpsId);
				return GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_SPS_ID);
			}
		}
		WELS_READ_VERIFY(BsGetSe(pBsAux,&iCode));		// second_chroma_qp_index_offset
		pPps->iChromaQpIndexOffset[1]=iCode;
		WELS_CHECK_SE_BOTH_ERROR(pPps->iChromaQpIndexOffset[1],PPS_CHROMA_QP_INDEX_OFFSET_MIN,PPS_CHROMA_QP_INDEX_OFFSET_MAX,"chroma_qp_index_offset",GENERATE_ERROR_NO(ERR_LEVEL_PARAM_SETS,ERR_INFO_INVALID_CHROMA_QP_INDEX_OFFSET));
	}

	if(pCtx->pPps!=NULL && pCtx->pPps->iPpsId==pPps->iPpsId){
		if(memcmp(pCtx->pPps,pPps,sizeof(*pPps))!=0){
			memcpy(&pCtx->sSpsPpsCtx.sPpsBuffer[MAX_PPS_COUNT],pPps,sizeof(SPps));
			pCtx->sSpsPpsCtx.iOverwriteFlags|=OVERWRITE_PPS;
			if(pCtx->pAccessUnitList->uiAvailUnitsNum>0){
				pCtx->bAuReadyFlag=true;
				pCtx->pAccessUnitList->uiEndPos=pCtx->pAccessUnitList->uiAvailUnitsNum-1;
			}
		}
	}else{
		memcpy(&pCtx->sSpsPpsCtx.sPpsBuffer[uiPpsId],pPps,sizeof(SPps));
		pCtx->sSpsPpsCtx.bPpsAvailFlags[uiPpsId]=true;
	}
	return ERR_NONE;
}

// brief to parse NON VCL NAL Units
// param pCtx decoder context
// param rbsp rbsp buffer of NAL Unit
// param src_len length of rbsp buffer
// return 0-successed
// 1-failed
int32_t ParseNonVclNal(SDecoderContext* pCtx,uint8_t* pRbsp,const int32_t kiSrcLen,uint8_t* pSrcNal,const int32_t kSrcNalLen){
	SBitStringAux* pBs=NULL;
	EWelsNalUnitType eNalType=NAL_UNIT_UNSPEC_0;		// make initial value as unspecified
	int32_t iPicWidth=0;
	int32_t iPicHeight=0;
	int32_t iBitSize=0;
	int32_t iErr=ERR_NONE;
	if(kiSrcLen<=0)
		return iErr;

	pBs=&pCtx->sBs;		// SBitStringAux instance for non VCL NALs decoding
	iBitSize=(kiSrcLen<<3)-BsGetTrailingBits(pRbsp+kiSrcLen-1);		// convert into bit
	eNalType=pCtx->sCurNalHead.eNalUnitType;

	switch(eNalType){
		case NAL_UNIT_SPS:
		case NAL_UNIT_SUBSET_SPS:
			if(iBitSize>0){
				iErr=DecInitBits(pBs,pRbsp,iBitSize);
				if(ERR_NONE!=iErr){
					if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE)
						pCtx->iErrorCode|=dsNoParamSets;
					else
						pCtx->iErrorCode|=dsBitstreamError;
					return iErr;
				}
			}
			iErr=ParseSps(pCtx,pBs,&iPicWidth,&iPicHeight,pSrcNal,kSrcNalLen);
			if(ERR_NONE!=iErr){		// modified for pSps/pSubsetSps invalid,12/1/2009
				if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE)
					pCtx->iErrorCode|=dsNoParamSets;
				else
					pCtx->iErrorCode|=dsBitstreamError;
				return iErr;
			}
			break;
		case NAL_UNIT_PPS:
			if(iBitSize>0){
				iErr=DecInitBits(pBs,pRbsp,iBitSize);
				if(ERR_NONE!=iErr){
					if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE)
						pCtx->iErrorCode|=dsNoParamSets;
					else
						pCtx->iErrorCode|=dsBitstreamError;
					return iErr;
				}
			}
			iErr=ParsePps(pCtx,&pCtx->sSpsPpsCtx.sPpsBuffer[0],pBs,pSrcNal,kSrcNalLen);
			if(ERR_NONE!=iErr){		// modified for pps invalid,12/1/2009
				if(pCtx->pParam->eEcActiveIdc==ERROR_CON_DISABLE)
					pCtx->iErrorCode|=dsNoParamSets;
				else
					pCtx->iErrorCode|=dsBitstreamError;
				return iErr;
			}
			pCtx->sSpsPpsCtx.bPpsExistAheadFlag=true;
			++(pCtx->sSpsPpsCtx.iSeqId);
			break;
		case NAL_UNIT_SEI:
			break;
		case NAL_UNIT_PREFIX:
			break;
		case NAL_UNIT_CODED_SLICE_DPA:
		case NAL_UNIT_CODED_SLICE_DPB:
		case NAL_UNIT_CODED_SLICE_DPC:

			break;

		default:
			break;
	}

	return iErr;
}

bool CheckAccessUnitBoundaryExt(SNalUnitHeaderExt* pLastNalHdrExt,SNalUnitHeaderExt* pCurNalHeaderExt,SSliceHeader* pLastSliceHeader,SSliceHeader* pCurSliceHeader){
	const SSps* kpSps=pCurSliceHeader->pSps;
	// Sub-clause 7.1.4.1.1 temporal_id
	if(pLastNalHdrExt->uiTemporalId!=pCurNalHeaderExt->uiTemporalId){
		return true;
	}
	// Subclause 7.4.1.2.5
	if(pLastSliceHeader->iRedundantPicCnt>pCurSliceHeader->iRedundantPicCnt)
		return true;

	// Subclause G7.4.1.2.4
	if(pLastNalHdrExt->uiDependencyId>pCurNalHeaderExt->uiDependencyId)
		return true;
	if(pLastNalHdrExt->uiQualityId>pCurNalHeaderExt->uiQualityId)
		return true;

	// Subclause 7.4.1.2.4
	if(pLastSliceHeader->iFrameNum!=pCurSliceHeader->iFrameNum)
		return true;
	if(pLastSliceHeader->iPpsId!=pCurSliceHeader->iPpsId)
		return true;
	if(pLastSliceHeader->pSps->iSpsId!=pCurSliceHeader->pSps->iSpsId)
		return true;
	if(pLastSliceHeader->bFieldPicFlag!=pCurSliceHeader->bFieldPicFlag)
		return true;
	if(pLastSliceHeader->bBottomFiledFlag!=pCurSliceHeader->bBottomFiledFlag)
		return true;
	if((pLastNalHdrExt->sNalUnitHeader.uiNalRefIdc!=NRI_PRI_LOWEST)!=(pCurNalHeaderExt->sNalUnitHeader.uiNalRefIdc!=NRI_PRI_LOWEST))
		return true;
	if(pLastNalHdrExt->bIdrFlag!=pCurNalHeaderExt->bIdrFlag)
		return true;
	if(pCurNalHeaderExt->bIdrFlag){
		if(pLastSliceHeader->uiIdrPicId!=pCurSliceHeader->uiIdrPicId)
			return true;
	}
	if(kpSps->uiPocType==0){
		if(pLastSliceHeader->iPicOrderCntLsb!=pCurSliceHeader->iPicOrderCntLsb)
			return true;
		if(pLastSliceHeader->iDeltaPicOrderCntBottom!=pCurSliceHeader->iDeltaPicOrderCntBottom)
			return true;
	}else
	if(kpSps->uiPocType==1){
		if(pLastSliceHeader->iDeltaPicOrderCnt[0]!=pCurSliceHeader->iDeltaPicOrderCnt[0])
			return true;
		if(pLastSliceHeader->iDeltaPicOrderCnt[1]!=pCurSliceHeader->iDeltaPicOrderCnt[1])
			return true;
	}
	if(memcmp(pLastSliceHeader->pPps,pCurSliceHeader->pPps,sizeof(SPps))!=0	 || memcmp(pLastSliceHeader->pSps,pCurSliceHeader->pSps,sizeof(SSps))!=0){
		return true;
	}
	return false;
}

// Mark erroneous frame as Ref Pic into DPB
int32_t MarkECFrameAsRef(SDecoderContext* pCtx){
	int32_t iRet=WelsMarkAsRef(pCtx);
	// Under EC mode,the ERR_INFO_DUPLICATE_FRAME_NUM does not need to be process
	if(iRet!=ERR_NONE){
		return iRet;
	}
	ExpandReferencingPicture(pCtx->pDec->pData,pCtx->pDec->iWidthInPixel,pCtx->pDec->iHeightInPixel,pCtx->pDec->iLinesize,pCtx->sExpandPicFunc.pfExpandLumaPicture,pCtx->sExpandPicFunc.pfExpandChromaPicture);
	return ERR_NONE;
}

bool CheckAndFinishLastPic(SDecoderContext* pCtx,uint8_t** ppDst,SBufferInfo* pDstInfo){
	SAccessUnit* pAu=pCtx->pAccessUnitList;
	bool bAuBoundaryFlag=false;
	if(IS_VCL_NAL(pCtx->sCurNalHead.eNalUnitType,1)){		// VCL data,AU list should have data
		SNalUnit* pCurNal=pAu->pNalUnitsList[pAu->uiEndPos];
		bAuBoundaryFlag=(pCtx->iTotalNumMbRec!=0) && (CheckAccessUnitBoundaryExt(&pCtx->pLastDecPicInfo->sLastNalHdrExt,&pCurNal->sNalHeaderExt,&pCtx->pLastDecPicInfo->sLastSliceHeader,&pCurNal->sNalData.sVclNal.sSliceHeaderExt.sSliceHeader));
	}else{		// non VCL
		if(pCtx->sCurNalHead.eNalUnitType==NAL_UNIT_AU_DELIMITER){
			bAuBoundaryFlag=true;
		}else
		if(pCtx->sCurNalHead.eNalUnitType==NAL_UNIT_SEI){
			bAuBoundaryFlag=true;
		}else
		if(pCtx->sCurNalHead.eNalUnitType==NAL_UNIT_SPS){
			bAuBoundaryFlag=!!(pCtx->sSpsPpsCtx.iOverwriteFlags&OVERWRITE_SPS);
		}else
		if(pCtx->sCurNalHead.eNalUnitType==NAL_UNIT_SUBSET_SPS){
			bAuBoundaryFlag=!!(pCtx->sSpsPpsCtx.iOverwriteFlags&OVERWRITE_SUBSETSPS);
		}else
		if(pCtx->sCurNalHead.eNalUnitType==NAL_UNIT_PPS){
			bAuBoundaryFlag=!!(pCtx->sSpsPpsCtx.iOverwriteFlags&OVERWRITE_PPS);
		}
		if(bAuBoundaryFlag && pCtx->pAccessUnitList->uiAvailUnitsNum!=0){		// Construct remaining data first
			ConstructAccessUnit(pCtx,ppDst,pDstInfo);
		}
	}
	// Do Error Concealment here
	if(bAuBoundaryFlag && (pCtx->iTotalNumMbRec!=0) && NeedErrorCon(pCtx)){		// AU ready but frame not completely reconed
		if(pCtx->pParam->eEcActiveIdc!=ERROR_CON_DISABLE){
			ImplementErrorCon(pCtx);
			pCtx->iTotalNumMbRec=pCtx->pSps->iMbWidth*pCtx->pSps->iMbHeight;
			pCtx->pDec->iSpsId=pCtx->pSps->iSpsId;
			pCtx->pDec->iPpsId=pCtx->pPps->iPpsId;

			DecodeFrameConstruction(pCtx,ppDst,pDstInfo);
			pCtx->pLastDecPicInfo->pPreviousDecodedPictureInDpb=pCtx->pDec;		// save ECed pic for future use
			if(pCtx->pLastDecPicInfo->sLastNalHdrExt.sNalUnitHeader.uiNalRefIdc>0){
				if(MarkECFrameAsRef(pCtx)==ERR_INFO_INVALID_PTR){
					pCtx->iErrorCode|=dsRefListNullPtrs;
					return false;
				}
			}
		}else
		if(DecodeFrameConstruction(pCtx,ppDst,pDstInfo)){
			if((pCtx->pLastDecPicInfo->sLastNalHdrExt.sNalUnitHeader.uiNalRefIdc>0) && (pCtx->pLastDecPicInfo->sLastNalHdrExt.uiTemporalId==0))
				pCtx->iErrorCode|=dsNoParamSets;
			else
				pCtx->iErrorCode|=dsBitstreamError;
			pCtx->pDec=NULL;
			return false;
		}
		pCtx->pDec=NULL;
		if(pAu->pNalUnitsList[pAu->uiStartPos]->sNalHeaderExt.sNalUnitHeader.uiNalRefIdc>0)
			pCtx->pLastDecPicInfo->iPrevFrameNum=pCtx->pLastDecPicInfo->sLastSliceHeader.iFrameNum;		// save frame_num
		if(pCtx->pLastDecPicInfo->bLastHasMmco5)
			pCtx->pLastDecPicInfo->iPrevFrameNum=0;
	}
	return ERR_NONE;
}

static void WriteBackActiveParameters(SDecoderContext* pCtx){
	if(pCtx->sSpsPpsCtx.iOverwriteFlags&OVERWRITE_PPS){
		memcpy(&pCtx->sSpsPpsCtx.sPpsBuffer[pCtx->sSpsPpsCtx.sPpsBuffer[MAX_PPS_COUNT].iPpsId],&pCtx->sSpsPpsCtx.sPpsBuffer[MAX_PPS_COUNT],sizeof(SPps));
	}
	if(pCtx->sSpsPpsCtx.iOverwriteFlags&OVERWRITE_SPS){
		memcpy(&pCtx->sSpsPpsCtx.sSpsBuffer[pCtx->sSpsPpsCtx.sSpsBuffer[MAX_SPS_COUNT].iSpsId],&pCtx->sSpsPpsCtx.sSpsBuffer[MAX_SPS_COUNT],sizeof(SSps));
		pCtx->bNewSeqBegin=true;
	}
	if(pCtx->sSpsPpsCtx.iOverwriteFlags&OVERWRITE_SUBSETSPS){
		memcpy(&pCtx->sSpsPpsCtx.sSubsetSpsBuffer[pCtx->sSpsPpsCtx.sSubsetSpsBuffer[MAX_SPS_COUNT].sSps.iSpsId],&pCtx->sSpsPpsCtx.sSubsetSpsBuffer[MAX_SPS_COUNT],sizeof(SSubsetSps));
		pCtx->bNewSeqBegin=true;
	}
	pCtx->sSpsPpsCtx.iOverwriteFlags=OVERWRITE_NONE;
}

// DecodeFinishUpdate
// decoder finish decoding,update active parameter sets and new seq status
void DecodeFinishUpdate(SDecoderContext* pCtx){
	pCtx->bNewSeqBegin=false;
	WriteBackActiveParameters(pCtx);
	pCtx->bNewSeqBegin=pCtx->bNewSeqBegin || pCtx->bNextNewSeqBegin;
	pCtx->bNextNewSeqBegin=false;		// reset it
	if(pCtx->bNewSeqBegin)
		ResetActiveSPSForEachLayer(pCtx);
}

// brief First entrance to decoding core interface.
// param pCtx decoder context
// param pBufBs bit streaming buffer
// param kBsLen size in bytes length of bit streaming buffer input
// param ppDst picture payload data to be output
// param pDstBufInfo buf information of ouput data
// return 0-successed
// return 1-failed
int32_t WelsDecodeBsInit(SDecoderContext* pCtx,const uint8_t* kpBsBuf,const int32_t kiBsLen,uint8_t** ppDst,SBufferInfo* pDstBufInfo){
	SDataBuffer* pRawData=&pCtx->sRawData;
	int32_t iSrcIdx=0;			// the index of source bit-stream till now after parsing one or more NALs
	int32_t iSrcConsumed=0;		// consumed bit count of source bs
	int32_t iDstIdx=0;			// the size of current NAL after 0x03 removal and 00 00 01 removal
	int32_t iSrcLength=0;		// the total size of current AU or NAL
	int32_t iRet=0;
	int32_t iConsumedBytes=0;
	int32_t iOffset=0;
	uint8_t* pSrcNal=NULL;
	uint8_t* pDstNal=NULL;
	uint8_t* pNalPayload=NULL;
	if(NULL==DetectStartCodePrefix(kpBsBuf,&iOffset,kiBsLen)){
		FATAL("Unable to find start 0,0,0,1 prefix");
		pCtx->iErrorCode|=dsBitstreamError;
		return dsBitstreamError;
	}
	pSrcNal=const_cast<uint8_t*> (kpBsBuf)+iOffset;
	iSrcLength=kiBsLen-iOffset;
	if((kiBsLen+4)>(pRawData->pEnd-pRawData->pCurPos)){
		pRawData->pCurPos=pRawData->pHead;
	}
	// copy raw data from source buffer (application) to raw data buffer (codec inside)
	// 0x03 removal and extract all of NAL Unit from current raw data
	pDstNal=pRawData->pCurPos;
	bool bNalStartBytes=false;
	while(iSrcConsumed<iSrcLength){
		if((2+iSrcConsumed<iSrcLength) && (0==LD16(pSrcNal+iSrcIdx)) && (pSrcNal[2+iSrcIdx]<=0x03)){
			if(bNalStartBytes && (pSrcNal[2+iSrcIdx]!=0x00 && pSrcNal[2+iSrcIdx]!=0x01)){
				pCtx->iErrorCode|=dsBitstreamError;
				return pCtx->iErrorCode;
			}
			if(pSrcNal[2+iSrcIdx]==0x02){
				pCtx->iErrorCode|=dsBitstreamError;
				return pCtx->iErrorCode;
			}else
			if(pSrcNal[2+iSrcIdx]==0x00){
				pDstNal[iDstIdx++]=pSrcNal[iSrcIdx++];
				iSrcConsumed++;
				bNalStartBytes=true;
			}else
			if(pSrcNal[2+iSrcIdx]==0x03){
				if((3+iSrcConsumed<iSrcLength) && pSrcNal[3+iSrcIdx]>0x03){
					pCtx->iErrorCode|=dsBitstreamError;
					return pCtx->iErrorCode;
				}else{
					ST16(pDstNal+iDstIdx,0);
					iDstIdx+=2;
					iSrcIdx+=3;
					iSrcConsumed+=3;
				}
			}else{		// 0x01
				bNalStartBytes=false;
				iConsumedBytes=0;
				pDstNal[iDstIdx]=pDstNal[iDstIdx+1]=pDstNal[iDstIdx+2]=pDstNal[iDstIdx+3]=0;		// set 4 reserved bytes to zero
				pNalPayload=ParseNalHeader(pCtx,&pCtx->sCurNalHead,pDstNal,iDstIdx,pSrcNal-3,iSrcIdx+3,&iConsumedBytes);
				if(pNalPayload){		// parse correct
					if(IS_PARAM_SETS_NALS(pCtx->sCurNalHead.eNalUnitType)){
						iRet=ParseNonVclNal(pCtx,pNalPayload,iDstIdx-iConsumedBytes,pSrcNal-3,iSrcIdx+3);
					}
					CheckAndFinishLastPic(pCtx,ppDst,pDstBufInfo);
					if(pCtx->bAuReadyFlag && pCtx->pAccessUnitList->uiAvailUnitsNum!=0){
						FATAL("INIT SHOULD NOT COME HERE");
						ConstructAccessUnit(pCtx,ppDst,pDstBufInfo);
					}
				}
				DecodeFinishUpdate(pCtx);
				if((dsOutOfMemory|dsNoParamSets)&pCtx->iErrorCode){
					pCtx->bParamSetsLostFlag=true;
					if(dsOutOfMemory&pCtx->iErrorCode){
						return pCtx->iErrorCode;
					}
				}
				if(iRet){
					iRet=0;
					if(dsNoParamSets&pCtx->iErrorCode){
						pCtx->bParamSetsLostFlag=true;
					}
					return pCtx->iErrorCode;
				}
				pDstNal+=(iDstIdx+4);		// init,increase 4 reserved zero bytes,used to store the next NAL
				if((iSrcLength-iSrcConsumed+4)>(pRawData->pEnd-pDstNal)){
					pDstNal=pRawData->pCurPos=pRawData->pHead;
				}else{
					pRawData->pCurPos=pDstNal;
				}
				pSrcNal+=iSrcIdx+3;
				iSrcConsumed+=3;
				iSrcIdx=0;
				iDstIdx=0;		// reset 0,used to statistic the length of next NAL
			}
			continue;
		}
		pDstNal[iDstIdx++]=pSrcNal[iSrcIdx++];
		iSrcConsumed++;
	}
	// last NAL decoding
	iConsumedBytes=0;
	pDstNal[iDstIdx]=pDstNal[iDstIdx+1]=pDstNal[iDstIdx+2]=pDstNal[iDstIdx+3]=0;		// set 4 reserved bytes to zero
	pRawData->pCurPos=pDstNal+iDstIdx+4;												// init,increase 4 reserved zero bytes,used to store the next NAL
	pNalPayload=ParseNalHeader(pCtx,&pCtx->sCurNalHead,pDstNal,iDstIdx,pSrcNal-3,iSrcIdx+3,&iConsumedBytes);
	if(pNalPayload){																	// parse correct
		if(IS_PARAM_SETS_NALS(pCtx->sCurNalHead.eNalUnitType)){
			iRet=ParseNonVclNal(pCtx,pNalPayload,iDstIdx-iConsumedBytes,pSrcNal-3,iSrcIdx+3);
		}
		CheckAndFinishLastPic(pCtx,ppDst,pDstBufInfo);
		if(pCtx->bAuReadyFlag && pCtx->pAccessUnitList->uiAvailUnitsNum!=0){
			//ConstructAccessUnit(pCtx,ppDst,pDstBufInfo);
			FATAL("INIT SHOULD NOT COME HERE");
		}
	}
	DecodeFinishUpdate(pCtx);
	if((dsOutOfMemory|dsNoParamSets)&pCtx->iErrorCode){
		pCtx->bParamSetsLostFlag=true;
		return pCtx->iErrorCode;
	}
	if(iRet){
		iRet=0;
		if(dsNoParamSets&pCtx->iErrorCode){
			pCtx->bParamSetsLostFlag=true;
		}
		return pCtx->iErrorCode;
	}
	return pCtx->iErrorCode;
}

DECODING_STATE PlainH264Decoder::DecodeFrame(const unsigned char* kpSrc,const int kiSrcLen,unsigned char** ppDst,SBufferInfo* pDstInfo){
	{
		m_pCtx->bEndOfStreamFlag=false;
		ppDst[0]=ppDst[1]=ppDst[2]=NULL;
		m_pCtx->iErrorCode=dsErrorFree;									// initialize at the starting of AU decoding.
		m_pCtx->iFeedbackVclNalInAu=FEEDBACK_UNKNOWN_NAL;				// initialize
		uint64_t uiInBsTimeStamp=pDstInfo->uiInBsTimeStamp;
		memset(pDstInfo,0,sizeof(SBufferInfo));
		pDstInfo->uiInBsTimeStamp=uiInBsTimeStamp;
		m_pCtx->bReferenceLostAtT0Flag=false;							// initialize for LTR
		m_pCtx->bCurAuContainLtrMarkSeFlag=false;
		m_pCtx->iFrameNumOfAuMarkedLtr=0;
		m_pCtx->iFrameNum=-1;
		m_pCtx->iFeedbackTidInAu=-1;
		m_pCtx->iFeedbackNalRefIdc=-1;
		pDstInfo->uiOutYuvTimeStamp=0;
		m_pCtx->uiTimeStamp=pDstInfo->uiInBsTimeStamp;
		WelsDecodeBsInit(m_pCtx,kpSrc,kiSrcLen,ppDst,pDstInfo);			// iErrorCode has been modified in this function
	}
	{
		m_pCtx->bEndOfStreamFlag=true;
		m_pCtx->bInstantDecFlag=true;
		ppDst[0]=ppDst[1]=ppDst[2]=NULL;
		m_pCtx->iErrorCode=dsErrorFree;									// initialize at the starting of AU decoding.
		m_pCtx->iFeedbackVclNalInAu=FEEDBACK_UNKNOWN_NAL;				// initialize
		uint64_t uiInBsTimeStamp=pDstInfo->uiInBsTimeStamp;
		memset(pDstInfo,0,sizeof(SBufferInfo));
		pDstInfo->uiInBsTimeStamp=uiInBsTimeStamp;
		m_pCtx->bReferenceLostAtT0Flag=false;							// initialize for LTR
		m_pCtx->bCurAuContainLtrMarkSeFlag=false;
		m_pCtx->iFrameNumOfAuMarkedLtr=0;
		m_pCtx->iFrameNum=-1;
		m_pCtx->iFeedbackTidInAu=-1;
		m_pCtx->iFeedbackNalRefIdc=-1;
		pDstInfo->uiOutYuvTimeStamp=0;
		m_pCtx->uiTimeStamp=pDstInfo->uiInBsTimeStamp;
		if(!m_pCtx->pAccessUnitList->uiAvailUnitsNum) {
//			FATAL("WTF");
		}else{
			m_pCtx->pAccessUnitList->uiEndPos=m_pCtx->pAccessUnitList->uiAvailUnitsNum-1;
			ConstructAccessUnit(m_pCtx,ppDst,pDstInfo);
		}
		DecodeFinishUpdate(m_pCtx);
	}
	return (DECODING_STATE)m_pCtx->iErrorCode;
}


};	// namespace NewDec
