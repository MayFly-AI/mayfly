#ifndef _MATHTYPES_H_
#define _MATHTYPES_H_
#pragma once

#include "shared/types.h"


#include <float.h>
#include <math.h>
#include <memory.h>
#include <limits.h>

#define PI 3.14159265358979323846f
#define HALFPI (3.14159265358979323846f/2.0f)
#define PI2 (3.14159265358979323846f*2.0f)

#ifdef __cplusplus

inline int Abs(int a) {
	if(a>=0)return a;
	return -a;
}

void SeedRandom(uint32_t seed);
float RandomUnitFloat();

static constexpr float FEPSILON=(1.0f / 4096.0f);
static constexpr float HEPSILON=(1.0f / 256.0f);

#define FHUGE 1e38f
#define DHUGE 1.79769e+308
#ifndef MIN
#define MIN(x, y) (((x) < (y))? (x) : (y))
#endif

#ifndef MAX
#define MAX(x, y) (((x) > (y))? (x) : (y))
#endif

#ifndef BOUND
#define BOUND(min,x,max) ((x)<(min)?(min):(x)>(max)?(max):(x))
#endif

#ifndef CLAMP
#define CLAMP(min,x,max) ((x)<(min)?(min):(x)>(max)?(max):(x))
#endif

#ifndef SATURATE
#define SATURATE(x) ((x)<0?0:(x)>1?1:(x))
#endif

#define SATURATE_ADD_UCHAR(a, b) (((b)>UCHAR_MAX-(a)) ? UCHAR_MAX : ((a)+(b)))
#define SATURATE_ADD_UINT(x, y) (((y)>UINT_MAX-(x)) ? UINT_MAX : ((x)+(y)))
#define SATURATE_ADD_USHORT(x, y) (((y)>SHRT_MAX-(x)) ? USHRT_MAX : ((x)+(y)))

#define FPI 3.1415926535897f
#define DEGREES_TO_RADIANS(__ANGLE__) ( (__ANGLE__) / 180.0f * 3.14159265358979323846f )
#define RADIANS_TO_DEGREES(__ANGLE__) ( (__ANGLE__) * (180.0f / 3.14159265358979323846f) )

inline float srgb_encode(float f) {
    if(f <=0.0031308f)
        return 12.92f*f;
    else
        return 1.055f*powf(f,1.f/2.4f)-0.055f;
}

template <typename T> inline T round_up(T k,T alignment) {
    return (k+alignment-1) & ~(alignment-1);
}

template<typename _T> inline _T Clamp(const _T& Value, const _T& Min, const _T& Max) {
	if(Value<Min) {
		return Min;
	} else if(Value>Max) {
		return Max;
	} else {
		return Value;
	}
}

inline float FMax(float x,float y) {
	if(x>y)return x;
	return y;
	//return x<y?x:y;
}
inline float FClamp(float fValue,float fMin,float fMax) {
	return Clamp<float>(fValue,fMin,fMax);
}


template <class T> inline bool ArrayEquals(const T* a, const T* b, int size)
{
	for(int i=0; i<size; i++)
	{
		if(a[i]!=b[i])
		{
			return false;
		}
	}
	return true;
}

struct float16_s {
	union {
		struct {
			uint16_t m : 10;
			uint16_t e : 5;
			uint16_t s : 1;
		} bits;
		uint16_t all;
	};
	inline float16_s() { all=0; }
	inline float16_s(float f);
	inline float16_s(const float16_s& o) : all(o.all) {}
	inline operator float() const;
};

typedef float16_s f16;

void float32to16(float x,float16_s * f16);
float float16to32(float16_s f16);
void f32Tof16(uint16_t* pf16,const float* pf32,int elements);
void f16Tof32(float* pf32,const uint16_t* pf16,int elements);
float16_s::float16_s(float f) { float32to16(f,this); }
float16_s::operator float() const { return float16to32(*this); }

inline float F16ToF32(uint16_t h) {
	return float16to32(h);
	//return (float)(((h&0x8000)<<16) | (((h&0x7c00)+0x1C000)<<13) | ((h&0x03FF)<<13));
}
inline uint16_t F32ToF16(float f) {
	float16_s t;
	float32to16(f,&t);
	return (uint16_t)t;
	//uint32_t x=*((uint32_t*)&f);
	//return (uint16_t)(((x>>16)&0x8000)|((((x&0x7f800000)-0x38000000)>>13)&0x7c00)|((x>>13)&0x03ff));
}

struct VI2 {
	int x,y;
	VI2(){x=y=0;}
	VI2(int _x,int _y){x=_x;y=_y;}
};
struct VI4 {
	int x,y,z,w;
	VI4(){x=y=z=w=0;}
	VI4(int _x,int _y,int _z,int _w){x=_x;y=_y;z=_z;w=_w;}
};

class V2I {
	public:
		inline V2I() {};
		inline V2I(int in_x,int in_y) {x=in_x;y=in_y;}
		bool operator==(const V2I& v) const { return(x==v.x && y==v.y); }
		bool operator!=(const V2I& v) const { return(x!=v.x || y!=v.y); }
		int x;
		int y;
};

class V2ISize {
	public:
		inline V2ISize() {};
		inline V2ISize(int in_width,int in_height) {width=in_width;height=in_height;}
		inline V2ISize(const V2I& v){width=v.x;height=v.y;}
		bool operator==(const V2ISize& v) const { return(width==v.width && height==v.height); }
		bool operator!=(const V2ISize& v) const { return(width!=v.width || height!=v.height); }
		int width;
		int height;
};

/*
 * support for V2, V3, V4, M44.
 *
 * FIXME 1) EAxis and GetAxis are used by Euler angle functions in old_lin_alg.h, so they need to be defined prior to inclusion.
 *          On the other hand, GetAxis needs V3 to be defined.
 *
 *       2) GetAngle is used by VectorToEulerPY(const V3& v)
 *
 *       Should this then also be moved to old_lin_alg.h ?
 */

enum EAxis {
	eXAxis=0,
	eYAxis=1,
	eZAxis=2,
	eNegXAxis=3,
	eNegYAxis=4,
	eNegZAxis=5
};

inline class V3 GetAxis(const EAxis eAxis);

inline float GetAngle(const float x,const float y);

// storage classes.
class SV2;
class SV3;
class SV3;

// vector classes.
class V2;
class V3;
class V4;

// matrix classes.
class M33;
class M44;

// functions
float Min(float a, float b); // used for Min(const V2&, const V2&)
float Max(float a, float b); // used for Max(const V2&, const V2&)

/*
 * V2 support.
 */

//Storage class for vector class
class SV2 {
public:
	float x;
	float y;
};

class V2 : public SV2
{
public:
	inline V2() {};
	V2(const V2& v)=default;

	inline explicit V2(const float *v) {x=v[0];y=v[1];}
	inline V2(float in_x, float in_y) {x=in_x;y=in_y;}
	inline explicit V2(float in) { x=in;y=in; }

	inline V2& operator=(const SV2& v){x=v.x;y=v.y;return *this;}
	V2(const SV2& v){x=v.x;y=v.y;}

	inline V2& operator=(const class V4& v);

	// assignment operators
	V2& operator=(const V2& v)		{ x=v.x; y=v.y; return *this; }
	V2& operator +=(const V2& v)		{ (*this)=(*this)+v; return *this; }
	V2& operator -=(const V2& v)		{ (*this)=(*this)-v; return *this; }
	V2& operator *=(const V2& v)		{ (*this)=(*this)*v; return *this; }
	V2& operator /=(const V2& v)		{ (*this)=(*this)/v; return *this; }
	V2& operator *=(float s)			{ (*this)=(*this)*s; return *this; }
	V2& operator /=(float s)			{ (*this)=(*this)/s; return *this; }
	V2& operator +=(float s)			{ (*this)=(*this)+s; return *this; }
	V2& operator -=(float s)			{ (*this)=(*this)-s; return *this; }

	// unary operators
	V2 operator+() const				{ return *this; }
	V2 operator-() const				{ return V2(-x, -y); }

	// binary operators
	V2 operator+(const V2& v) const	{ return V2(x+v.x, y+v.y); }
	V2 operator-(const V2& v) const	{ return V2(x-v.x, y-v.y); }
	V2 operator * (const V2& v) const	{ return V2(x*v.x, y*v.y); }
	V2 operator / (const V2& v) const	{ return V2(x/v.x, y/v.y); }
	V2 operator * (float s) const		{ return V2(x*s, y*s); }
	V2 operator / (float s) const		{ return V2(x/s, y/s); }
	V2 operator + (float s) const		{ return V2(x+s, y+s); }
	V2 operator - (float s) const		{ return V2(x-s, y-s); }

	bool operator==(const V2& v) const { return(x==v.x && y==v.y); }
	bool operator!=(const V2& v) const { return(x!=v.x || y!=v.y); }

	operator float* ()					{ return &x; }
	operator const float* () const		{ return &x; }
	V2 Floor() { return V2(floorf(x),floorf(y)); }

	V3 VX0Y()const;

	//conversions
	V2I ConvertToV2I()const				{	return V2I(int(x),int(y)); }
};

static inline V2 operator-(const float a,const V2& b) { return V2(a,a)-b; }
//V2 operator+(const float a,const V2& b) { return V2(a,a)+b; }
static inline V2 operator/(const float a,const V2& b) { return V2(a,a)/b; }

inline V2 operator * (float s, const V2& v)
{
	return v*s;
}

inline V2 VMul(const V2& v0,const V2& v1)
{
	return v0*v1;
}
inline V2 VMul(const V2& v0,float fValue)
{
	return v0*V2(fValue,fValue);
}

inline V2 VMax(const V2& v0,const V2& v1) {
	V2 vres;
	if(v0.x>v1.x) {
		vres.x=v0.x;
	}else{
		vres.x=v1.x;
	}
	if(v0.y>v1.y) {
		vres.y=v0.y;
	}else{
		vres.y=v1.y;
	}
	return vres;
}

inline V2 VMin(const V2& v0,const V2& v1) {
	V2 vres;
	if(v0.x<v1.x) {
		vres.x=v0.x;
	}else{
		vres.x=v1.x;
	}
	if(v0.y<v1.y) {
		vres.y=v0.y;
	}else{
		vres.y=v1.y;
	}
	return vres;
}

inline float VDot(const V2& a, const V2& b) {
	return(a.x*b.x+a.y*b.y);
}

inline bool RoughlyEqual(const V2& a, const V2& b, const float fLimit=0.001f) {
	return (fabsf(a.x-b.x)<fLimit && fabsf(a.y-b.y)<fLimit);
}

inline V2 RadiansToDegrees(const V2& v) {
	return V2(RADIANS_TO_DEGREES(v.x),RADIANS_TO_DEGREES(v.y));
}

inline V2 DegreesToRadians(const V2& v) {
	return V2(DEGREES_TO_RADIANS(v.x),DEGREES_TO_RADIANS(v.y));
}

inline V2 VAbs(const V2& v) {
	return V2(fabsf(v.x),fabsf(v.y));
}

inline float VLength(const V2 &v) {
	return sqrtf(VDot(v, v));
}

inline float VLengthSquared(const V2 &v) {
	return VDot(v, v);
}

inline V2 VNeg(const V2& v) {
	return V2(-v.x,-v.y);
}

inline V2 VNormalize(const V2 &a) {
	float len=VLength(a);
	return len>0.0f ? a/len : a;
}

inline float VCross(const V2 &a, const	V2 &b) {
	return a.x*b.y-a.y*b.x;
}

inline V2 Min(const V2& a, const V2& b) {
	return V2(Min(a.x, b.x), Min(a.y, b.y));
}

inline V2 Max(const V2& a, const V2& b) {
	return V2(Max(a.x, b.x), Max(a.y, b.y));
}

inline V2 VLerp(const V2& a, const V2& b, float s) {
	return a * (1-s)+b * s;
}

inline V2 Rotate(const V2& v, float a) {
	float c=cosf(a);
	float s=sinf(a);
	return V2(c * v.x+s * v.y, -s * v.x+c * v.y);
}

inline float lerp(float a,float b,float t){return a+(b-a)*t;}
inline V2 lerp(const V2& a,const V2& b,float t){return a+(b-a)*V2(t,t);}

//V3 support.
//Storage class for vector class

class SV3 {
	public:
		float x;
		float y;
		float z;
};

class V3 : public SV3 {
	public:
		inline V3() {};
		V3(const V3& v)=default;

		inline explicit V3(const float* v){x=v[0];y=v[1];z=v[2];}
		inline V3(float in_x, float in_y, float in_z){x=in_x;y=in_y;z=in_z;}
		inline V3(const V2& v, float in_z){x=v.x;y=v.y;z=in_z;}

		inline V3& operator=(const SV3& v){x=v.x;y=v.y;z=v.z;return *this;}
		inline V3(const SV3& v){x=v.x;y=v.y;z=v.z;}

		inline V3& operator=(const class V4& v);

		// assignment operators
		V3& operator=(const V3& v)		{ x=v.x; y=v.y; z=v.z; return *this; }
		V3& operator +=(const V3& v)		{ (*this)=(*this)+v; return *this; }
		V3& operator -=(const V3& v)		{ (*this)=(*this)-v; return *this; }
		V3& operator *=(const V3& v)		{ (*this)=(*this)*v; return *this; }
		V3& operator /=(const V3& v)		{ (*this)=(*this)/v; return *this; }
		V3& operator *=(float s)			{ (*this)=(*this)*s; return *this; }
		V3& operator /=(float s)			{ (*this)=(*this)/s; return *this; }

		// unary operators
		V3 operator+() const				{ return *this; }
		V3 operator-() const				{ return V3(-x, -y, -z); }

		// binary operators
		V3 operator+(const V3& v) const	{ return V3(x+v.x, y+v.y, z+v.z); }
		V3 operator-(const V3& v) const	{ return V3(x-v.x, y-v.y, z-v.z); }
		V3 operator * (const V3& v) const	{ return V3(x*v.x, y*v.y, z*v.z); }
		V3 operator / (const V3& v) const	{ return V3(x/v.x, y/v.y, z/v.z); }
		V3 operator * (float s) const		{ return V3(x*s, y*s, z*s); }
		V3 operator / (float s) const		{ return V3(x/s, y/s, z/s); }

		bool operator==(const V3& v) const { return(x==v.x && y==v.y && z==v.z); }
		bool operator!=(const V3& v) const { return(x!=v.x || y!=v.y || z!=v.z); }

		operator float* ()					{ return &x; }
		operator const float* () const		{ return &x; }

		V2 VXY()const{return V2(x,y);}
		V2 VXZ()const{return V2(x,z);}
		V3 VX0Z()const{return V3(x,0,z);}
		V3 V0YZ()const{return V3(0,y,z);}
		bool IsZero() const { return (x == 0.f && y == 0.f && z == 0.f); }
};

inline V3 V2::VX0Y() const {
	return V3(x,0,y);
}

inline V3 operator * (float s, const V3& v) {
	return v*s;
}

inline V3 VMul(const V3& v0,const V3& v1) {
	return v0*v1;
}

inline V3 VMul(const V3& v0,float fValue) {
	return v0*V3(fValue,fValue,fValue);
}

inline V3 VSplat(float fValue) {
	return V3(fValue,fValue,fValue);
}

inline V3 VMax(const V3& v0,const V3& v1) {
	V3 vres;
	if(v0.x>v1.x) {
		vres.x=v0.x;
	}else{
		vres.x=v1.x;
	}
	if(v0.y>v1.y) {
		vres.y=v0.y;
	}else{
		vres.y=v1.y;
	}
	if(v0.z>v1.z) {
		vres.z=v0.z;
	}else{
		vres.z=v1.z;
	}
	return vres;
}

inline V3 VMin(const V3& v0,const V3& v1) {
	V3 vres;
	if(v0.x<v1.x) {
		vres.x=v0.x;
	}else{
		vres.x=v1.x;
	}
	if(v0.y<v1.y) {
		vres.y=v0.y;
	}else{
		vres.y=v1.y;
	}
	if(v0.z<v1.z) {
		vres.z=v0.z;
	}else{
		vres.z=v1.z;
	}
	return vres;
}

inline V3 VAbs(const V3& v) {
	return V3(fabsf(v.x),fabsf(v.y),fabsf(v.z));
}

inline float VDot(const V3& a, const V3& b)
{
	return(a.x*b.x+a.y*b.y+a.z*b.z);
}

inline V3 MulComponents(const V3 &a, const V3 &b)
{
	return a*b;
}

inline bool RoughlyEqual(const V3& a, const V3& b, const float fLimit=0.001f)
{
	return (fabsf(a.x-b.x)<fLimit && fabsf(a.y-b.y)<fLimit && fabsf(a.z-b.z)<fLimit);
}

inline V3 RadiansToDegrees(const V3& v) {
	return V3(RADIANS_TO_DEGREES(v.x),RADIANS_TO_DEGREES(v.y),RADIANS_TO_DEGREES(v.z));
}

inline V3 DegreesToRadians(const V3& v) {
	return V3(DEGREES_TO_RADIANS(v.x),DEGREES_TO_RADIANS(v.y),DEGREES_TO_RADIANS(v.z));
}

inline float VLength(const V3 &v)
{
	return sqrtf(VDot(v, v));
}

inline float VLengthXZ(const V3 &v)
{
	return VLength(V2(v.x, v.z));
}

inline float VLengthSquared(const V3 &v)
{
	return VDot(v, v);
}

inline float VLengthSquaredXZ(const V3 &v)
{
	return VLengthSquared(V2(v.x, v.z));
}

inline V3 VNeg(const V3& v) {
	return V3(-v.x,-v.y,-v.z);
}

inline V3 VNormalize(const V3 &a)
{
	float len=VLength(a);
	return len>0.0f ? a/len : a;
}

inline V3 VNormalizeXZ(const V3 &a)
{
	return VNormalize(V3(a.x, 0, a.z));
}

inline V3 VCross(const V3 &a, const V3 &b)
{
	V3 c;
	c.x=a.y * b.z-a.z * b.y;
	c.y=a.z * b.x-a.x * b.z;
	c.z=a.x * b.y-a.y * b.x;
	return c;
}

inline V3 Min(const V3& a, const V3& b)
{
	return V3(Min(a.x, b.x), Min(a.y, b.y), Min(a.z, b.z));
}

inline V3 Max(const V3& a, const V3& b)
{
	return V3(Max(a.x, b.x), Max(a.y, b.y), Max(a.z, b.z));
}

inline float MinXYZ(const V3 &v)
{
	return Min(Min(v.x, v.y), v.z);
}

inline float MaxXYZ(const V3 &v)
{
	return Max(Max(v.x, v.y), v.z);
}

inline V3 VLerp(const V3& a, const V3& b, float s)
{
	return a * (1-s)+b * s;
}

// returns vector clockwise perpendicular to the input vector, maintaining the y component
inline V3 PerpendicularXZ_CW(const V3& v)
{
	return V3(v.z, v.y, -v.x);
}

inline void UpdateBounds(const V3 &p, V3 &min, V3 &max)
{
	if (p.x<min.x) min.x=p.x;
    if (p.x>max.x) max.x=p.x;
	if (p.y<min.y) min.y=p.y;
    if (p.y>max.y) max.y=p.y;
	if (p.z<min.z) min.z=p.z;
    if (p.z>max.z) max.z=p.z;
}

//  | x cos a-y sin a|   |x'|
//  | x sin a+y cos a|=|y'|
//  |         z        |   |z'|
inline V3 RotateZ(const V3& v, float a) {
	float c=cosf(a);
	float s=sinf(a);
	return V3(c*v.x+s*v.y, c*v.y-s*v.x, v.z);
}

//  | x cos a+z sin a|   |x'|
//  |         y        |=|y'|
//  |-x sin a+z cos a|   |z'|
inline V3 RotateY(const V3& v, float a) {
	float c=cosf(a);
	float s=sinf(a);
	return V3(c*v.x+s*v.z,v.y,c*v.z-s*v.x);
}
//  |         x        |   |x'|
//  | y cos a-z sin a|=|y'|
//  | y sin a+z cos a|   |z'|

inline V3 RotateX(const V3& v, float a) {
	float c=cosf(a);
	float s=sinf(a);
	return V3(v.x,c*v.y-s*v.z,s*v.y+c*v.z);
}

inline V2 VectorToEulerPY(const V3& v) {
	V3 n=VNormalize(v);
	float yaw=GetAngle(n.x,-n.z);
	float pitch=acosf(n.y)-DEGREES_TO_RADIANS(90.0f);
	return V2(pitch,yaw);
}

inline V3 EulerPYToVector(const V2& e) {
	V2 v=Rotate(V2(1,0),e.x);
	return RotateY(V3(v.x,v.y,0),e.y);
}

inline V2 VectorToEulerPY(const V3& v, EAxis up, EAxis ref) {
	const float z = VDot(v, GetAxis(up));
	const float x = VDot(v, GetAxis(ref));
	const V3 refHat = VCross(GetAxis(up), GetAxis(ref));
	const float y = VDot(v, refHat);
	const float yaw = atan2f(y, x);
	const V3 xy = v - z * GetAxis(up);
	const float lenXY = VLength(xy);
	const float pitch = atan2f(z, lenXY);
	return V2(pitch, yaw);
}

inline V3 EulerPYToVector(const V2& e, EAxis up, EAxis ref) {
	const float sinPitch=sinf(e.x);
	const float cosPitch=cosf(e.x);
	const float cosYaw=cosf(e.y);
	const float sinYaw=sinf(e.y);
	const V3 refHat = VCross(GetAxis(up), GetAxis(ref));
	return GetAxis(up) * sinPitch + GetAxis(ref) * cosPitch * cosYaw + refHat * cosPitch * sinYaw;
}

/*
 * V4 support.
 */

//Storage class for vector class
class SV4 {
public:
	float x;
	float y;
	float z;
	float w;
};

class V4 : public SV4 {
public:
	inline V4() {};
	V4(const V4& v)=default;

	inline explicit V4(const float *v) {x=v[0];y=v[1];z=v[2];w=v[3];}
	inline V4(float in_x, float in_y, float in_z, float in_w){x=in_x;y=in_y;z=in_z;w=in_w;}
	inline V4(const V3& v, float in_w){x=v.x;y=v.y;z=v.z;w=in_w;}
	inline V4(float xyzw){x=xyzw;y=xyzw;z=xyzw;w=xyzw;}

	//inline explicit V4(const float *v) : x(v[0]), y(v[1]), z(v[2]), w(v[3]) {}
	//inline V4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
	//inline V4(V3& v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
	//inline V4(float xyzw) : x(xyzw), y(xyzw), z(xyzw), w(xyzw) {}

	inline V4& operator=(const SV4& v){x=v.x;y=v.y;z=v.z;w=v.w;return *this;}
	inline V4(const SV4& v){x=v.x;y=v.y;z=v.z;w=v.w;}

	inline V3 XYZ()const{return V3(x,y,z);}

	// assignment operators
	V4& operator=(const V4& v)		{ x=v.x; y=v.y; z=v.z; w=v.w; return *this; }
	V4& operator +=(const V4& v)		{ (*this)=(*this)+v; return *this; }
	V4& operator -=(const V4& v)		{ (*this)=(*this)-v; return *this; }
	V4& operator *=(const V4& v)		{ (*this)=(*this)*v; return *this; }
	V4& operator /=(const V4& v)		{ (*this)=(*this)/v; return *this; }
	V4& operator *=(float s)			{ (*this)=(*this)*s; return *this; }
	V4& operator /=(float s)			{ (*this)=(*this)/s; return *this; }

	// unary operators
	V4 operator+() const				{ return *this; }
	V4 operator-() const				{ return V4(-x, -y, -z, -w); }

	// binary operators
	V4 operator+(const V4& v) const	{ return V4(x+v.x, y+v.y, z+v.z, w+v.w); }
	V4 operator-(const V4& v) const	{ return V4(x-v.x, y-v.y, z-v.z, w-v.w); }
	V4 operator * (const V4& v) const	{ return V4(x*v.x, y*v.y, z*v.z, w*v.w); }
	V4 operator / (const V4& v) const	{ return V4(x/v.x, y/v.y, z/v.z, w/v.w); }
	V4 operator * (float s) const		{ return V4(x*s, y*s, z*s, w*s); }
	V4 operator / (float s) const		{ return V4(x/s, y/s, z/s, w/s); }

	bool operator==(const V4& v) const { return(x==v.x && y==v.y && z==v.z && w==v.w); }
	bool operator!=(const V4& v) const { return(x!=v.x || y!=v.y || z!=v.z || w!=v.w); }

	operator float* ()					{ return &x; }
	operator const float* () const		{ return &x; }

	V3 VXYZ()const{return V3(x,y,z);}

};

inline V4 VSet(float x,float y,float z=0.0f,float w=1.0f) {
	return V4(x,y,z,w);
}

inline V3& V3::operator=(const V4& v) {
	x=v.x;y=v.y;z=v.z;return *this;
}

inline V2& V2::operator=(const V4& v) {
	x=v.x;y=v.y;return *this;
}

inline V4 operator * (float s, const V4& v)
{
	return v*s;
}

inline V4 VAbs(const V4& v) {
	return V4(fabsf(v.x),fabsf(v.y),fabsf(v.z),fabsf(v.w));
}

inline float VDot(const V4& a, const V4& b)
{
	return(a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w);
}

inline bool RoughlyEqual(const V4& a, const V4& b, const float fLimit = 0.001f)
{
	return (fabsf(a.x - b.x) < fLimit && fabsf(a.y - b.y) < fLimit && fabsf(a.z - b.z) < fLimit && fabsf(a.w - b.w) < fLimit);
}

inline float VLength(const V4 &v)
{
	return sqrtf(VDot(v, v));
}

inline float VLengthSquared(const V4 &v)
{
	return VDot(v, v);
}

inline V4 VNeg(const V4& v) {
	return V4(-v.x,-v.y,-v.z,-v.w);
}

inline V4 VNormalize(const V4 &a)
{
	float len=VLength(a);
	return len>0.0f ? a/len : a;
}

inline V4 Homogenize(const V4& v)
{
	return V4(v.x / v.w, v.y / v.w, v.z / v.w, 1.0f);
}

inline V4 VLerp(const V4& a, const V4& b, float s)
{
	return a * (1-s)+b * s;
}

inline const V3& V4ToV3(const V4& v)
{
	return *(const V3*)&v;
}

inline V3 HomogenizeToV3(const V4& v)
{
	return V3(v.x / v.w, v.y / v.w, v.z / v.w);
}

/*
inline uint32 V4ToColor(const V4& color)
{
	int r=int(color.x*255);
	int g=int(color.y*255);
	int b=int(color.z*255);
	int a=int(color.w*255);
	return D3DCOLOR_ARGB(a,r,g,b);
}
*/

inline float MinXYZW(const V4 &v)
{
	return Min(Min(v.x, v.y), Min(v.z, v.w));
}

inline float MaxXYZW(const V4 &v)
{
	return Max(Max(v.x, v.y), Max(v.z, v.w));
}

/*
 * M33 support.
 */

class M33 {
	public:
		inline M33() {}
		inline explicit M33(const float* data);
		inline M33(float e11, float e12, float e13, float e21, float e22, float e23, float e31, float e32, float e33);
		inline M33(const V3& xa,const V3& ya,const V3& za);
		static M33 Identity(){return M33(1,0,0,0,1,0,0,0,1);}
		inline operator float* ()							{ return m[0]; }
		inline operator const float * () const				{ return m[0]; }
		inline V3& XAxis() {
			return *(V3*)&m[0][0];
		}
		inline V3& YAxis() {
			return *(V3*)&m[1][0];
		}
		inline V3& ZAxis() {
			return *(V3*)&m[2][0];
		}
		/*
		inline V3& Axis(const int i) {
			return *(V3*)&m[i][0];
		}
		*/

		void SetAxis(V3 v, const int i) {
			m[i][0] = v.x;
			m[i][1] = v.y;
			m[i][2] = v.z;
		}

		inline const V3& XAxis() const {
			return *(V3*)&m[0][0];
		}
		inline const V3& YAxis() const {
			return *(V3*)&m[1][0];
		}
		inline const V3& ZAxis() const {
			return *(V3*)&m[2][0];
		}
		inline const V3& Axis(const int i) const {
			return *(V3*)&m[i][0];
		}

		struct e33 {
			float _11, _12, _13;
			float _21, _22, _23;
			float _31, _32, _33;
		};
		union {
			e33 e;
			float m[3][3];
		};
		inline M33& operator *=(float s) {
			for(int row=0; row<3; row++)
				for(int col=0; col<3; col++)
					m[row][col] *=s;
			return *this;
		}
		//M33 operator * (const M33& mat) const;

};

bool Verify(M33& m);

inline M33 RotateY(const M33& m, float angle) {
	float	cosA, sinA, tmp;

	cosA	=(float)cosf(angle);
	sinA	=(float)sinf(angle);

	M33 res=m;

	tmp	=res.m[0][0]*sinA;
	res.m[0][0]	*=cosA;
	res.m[0][0]	+=res.m[2][0]*sinA;
	res.m[2][0]	*=cosA;
	res.m[2][0]	-=tmp;

	tmp	=res.m[0][1]*sinA;
	res.m[0][1]	*=cosA;
	res.m[0][1]	+=res.m[2][1]*sinA;
	res.m[2][1]	*=cosA;
	res.m[2][1]	-=tmp;

	tmp	=res.m[0][2]*sinA;
	res.m[0][2]	*=cosA;
	res.m[0][2]	+=res.m[2][2]*sinA;
	res.m[2][2]	*=cosA;
	res.m[2][2]	-=tmp;
	return res;
}

inline M33 MTranspose(const M33& m) {
	M33 r;

	r.m[0][0]=m.m[0][0];
	r.m[1][0]=m.m[0][1];
	r.m[2][0]=m.m[0][2];

	r.m[0][1]=m.m[1][0];
	r.m[1][1]=m.m[1][1];
	r.m[2][1]=m.m[1][2];

	r.m[0][2]=m.m[2][0];
	r.m[1][2]=m.m[2][1];
	r.m[2][2]=m.m[2][2];

	return r;
}

inline M33 Multiply(const M33 &m1,const M33 &m2) {
	M33 r;
	for(int iCol=0; iCol < 3; iCol++) {
		r.m[0][iCol]=m1.m[0][0]*m2.m[0][iCol]+m1.m[0][1]*m2.m[1][iCol]+m1.m[0][2]*m2.m[2][iCol];
		r.m[1][iCol]=m1.m[1][0]*m2.m[0][iCol]+m1.m[1][1]*m2.m[1][iCol]+m1.m[1][2]*m2.m[2][iCol];
		r.m[2][iCol]=m1.m[2][0]*m2.m[0][iCol]+m1.m[2][1]*m2.m[1][iCol]+m1.m[2][2]*m2.m[2][iCol];
	}
	return r;
}

inline M33::M33(const V3& xa,const V3& ya,const V3& za) {
	m[0][0]=xa.x;
	m[0][1]=xa.y;
	m[0][2]=xa.z;
	m[1][0]=ya.x;
	m[1][1]=ya.y;
	m[1][2]=ya.z;
	m[2][0]=za.x;
	m[2][1]=za.y;
	m[2][2]=za.z;
}

inline M33::M33(float e11, float e12, float e13, float e21, float e22, float e23, float e31, float e32, float e33) {
	m[0][0]=e11;
	m[0][1]=e12;
	m[0][2]=e13;
	m[1][0]=e21;
	m[1][1]=e22;
	m[1][2]=e23;
	m[2][0]=e31;
	m[2][1]=e32;
	m[2][2]=e33;
}

inline M33 CreateMat33FromXAxisUserY(const V3& xa,const V3& ya) {
	V3 xan=VNormalize(xa);
	V3 zan=VNormalize(VCross(xan,ya));
	V3 yan=VNormalize(VCross(zan,xan));
	return M33(xan,yan,zan);
}

inline M33 CreateMat33FromYAxisUserZ(const V3& ya, const V3& za) {
	V3 yan=VNormalize(ya);
	V3 xan=VNormalize(VCross(yan, za));
	V3 zan=VNormalize(VCross(xan, yan));
	return M33(xan, yan, zan);
}

inline M33 CreateMat33FromZAxisUserX(const V3& za, const V3& xa) {
	V3 zan=VNormalize(za);
	V3 yan=VNormalize(VCross(zan, xa));
	V3 xan=VNormalize(VCross(yan, zan));
	return M33(xan, yan, zan);
}

inline M33 Rx(float theta) {
	M33 res;
	float s=sinf(theta);
	float c=cosf(theta);
	res.m[0][0]=1;res.m[0][1]=0;res.m[0][2]=0;
	res.m[1][0]=0;res.m[1][1]=c;res.m[1][2]=s;
	res.m[2][0]=0;res.m[2][1]=-s;res.m[2][2]=c;
	return res;
}

inline M33 Ry(float theta) {
	M33 res;
	float s=sinf(theta);
	float c=cosf(theta);
	res.m[0][0]=c;res.m[0][1]=0;res.m[0][2]=-s;
	res.m[1][0]=0;res.m[1][1]=1;res.m[1][2]=0;
	res.m[2][0]=s;res.m[2][1]=0;res.m[2][2]=c;
	return res;
}

inline M33 Rz(float theta) {
	M33 res;
	float s=sinf( theta );
	float c=cosf( theta );
	res.m[0][0]=c;res.m[0][1]=s;res.m[0][2]=0;
	res.m[1][0]=-s;res.m[1][1]=c;res.m[1][2]=0;
	res.m[2][0]=0;res.m[2][1]=0;res.m[2][2]=1;
	return res;
}

inline V3 RotationToEulerRPY(const M33& m) {
	float m00=m.m[0][0];
	float m10=m.m[1][0];
	float m20=m.m[2][0];
	float m21=m.m[2][1];
	float m22=m.m[2][2];
	V3 rpy;
	rpy.x=atan2f(-m21,m22);
	rpy.y=atan2f(m20,sqrtf(m21*m21+m22*m22));
	rpy.z=atan2f(-m10,m00);
	return rpy;
}

inline M33 EulerRPYToRotation(const V3& rpy) {
	return Multiply(Multiply(Rz(rpy.z),Ry(rpy.y)),Rx(rpy.x));
}

inline V3 Multiply(const V3 &in, const M33 &m)
{
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2];
	return out;
}
inline V3 Multiply(const M33 &m,const V3 &in)
{
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[0][1]+in.z*m.m[0][2];
	out.y=in.x*m.m[1][0]+in.y*m.m[1][1]+in.z*m.m[1][2];
	out.z=in.x*m.m[2][0]+in.y*m.m[2][1]+in.z*m.m[2][2];
	return out;
}

inline M33 EulerPYToMat(const V2& e, EAxis up, EAxis ref) {
	const float sinPitch=sinf(e.x);
	const float cosPitch=cosf(e.x);
	const float cosYaw=cosf(e.y);
	const float sinYaw=sinf(e.y);
	M33 m;
	const int nUp = up%3;
	const int nRef = ref%3;
	const V3 refHat = VCross(GetAxis(up), GetAxis(ref));
	const V3 vRef = GetAxis(up) * sinPitch + GetAxis(ref) * cosPitch * cosYaw + refHat * cosPitch * sinYaw;
	const V3 vUp = GetAxis(up) * cosPitch - GetAxis(ref) * sinPitch * cosYaw - refHat * sinPitch * sinYaw;
	const bool refCrossUp = ((nRef+1)%3) == nUp;
	const int nHat = refCrossUp ? ((nUp+1)%3) : (nRef+1)%3;
	V3 vHat;
	if(refCrossUp) {
		vHat = VCross(vRef, vUp);
	}
	else {
		vHat = VCross(vUp, vRef);
	}
	m.SetAxis(vUp, nUp);
	m.SetAxis(vRef, nRef);
	m.SetAxis(vHat, nHat);
	return m;
}

inline M33 EulerPYToMatChangeAxis(const V2& e, EAxis up, EAxis ref) {
	const float sinPitch=sinf(e.x);
	const float cosPitch=cosf(e.x);
	const float cosYaw=cosf(e.y);
	const float sinYaw=sinf(e.y);
	M33 m;
	const int nUp = up%3;
	const int nRef = ref%3;
	V3 upAxis(0.f, up > 2 ? -1.f : 1.f, 0.f);
	V3 refAxis(ref > 2 ? -1.f : 1.f, 0.f, 0.f);
	const V3 refHat = VCross(upAxis, refAxis);
	const V3 vRef = upAxis * sinPitch + refAxis * cosPitch * cosYaw + refHat * cosPitch * sinYaw;
	const V3 vUp = upAxis * cosPitch - refAxis * sinPitch * cosYaw - refHat * sinPitch * sinYaw;
	const bool refCrossUp = ((nRef+1)%3) == nUp;
	const int nHat = refCrossUp ? ((nUp+1)%3) : (nRef+1)%3;
	V3 vHat;
	if(refCrossUp) {
		vHat = VCross(vRef, vUp);
	}
	else {
		vHat = VCross(vUp, vRef);
	}
	m.SetAxis(vUp, nUp);
	m.SetAxis(vRef, nRef);
	m.SetAxis(vHat, nHat);
	return m;
}

inline void MEulerRPY(float* roll,float* pitch,float* yaw,const M33& m) {
	float m00 = m.m[0][0];
	float m10 = m.m[1][0];
	float m20 = m.m[2][0];
	float m21 = m.m[2][1];
	float m22 = m.m[2][2];
	*roll = atan2f(-m21, m22);
	*pitch = atan2f(m20, sqrtf(m21*m21 + m22*m22));
	*yaw = atan2f(-m10, m00);
}

inline void Renormalize(M33 &mat)
{
	V3 z=mat.ZAxis();
	V3 x=mat.XAxis();
	V3 y=VNormalize(VCross(z, x));
	mat.XAxis()=VNormalize(VCross(y, z));
	mat.YAxis()=y;
	mat.ZAxis()=VNormalize(z);
}

/*
 * M33 initialization helpers.
 */

inline M33 CreateMat33FromZAxisUserY(V3 zaxis,const V3& vUp) {
	zaxis=VNormalize(zaxis);
	V3 xa=VNormalize(VCross(vUp,zaxis));
	M33 m(xa,VCross(zaxis,xa),zaxis);
	return m;
}

M33 QuatToM33T(const V4& quat);
M33 QuatToM33(const V4& quat);
V4 M33ToQuatT(const M33& mat);
V4 M33ToQuat(const M33& mat);

//M33 QuatToM33(const V4& quat); // defined just below.
inline M33 CreateRotationMat(const V3& v0, const V3& v1) {
	const V3 vHalf = VNormalize(v0 + v1);
	const V4 q(VCross(v0, vHalf), VDot(v0, vHalf));
	return QuatToM33(q);
}

V4 Slerp(V4 p, V4 q2, float t);

/*
 * Quaterions.
 */
 /*
inline M33 QuatToM33(const V4& quat) {
	float xs, ys, zs, wx, wy, wz, xx, xy, xz, yy, yz, zz;
	xs=quat[0]*2.0f;  ys=quat[1]*2.0f;  zs=quat[2]*2.0f;
	wx=quat[3]*xs; wy=quat[3]*ys; wz=quat[3]*zs;
	xx=quat[0]*xs; xy=quat[0]*ys; xz=quat[0]*zs;
	yy=quat[1]*ys; yz=quat[1]*zs; zz=quat[2]*zs;
	M33 res;
	res.m[0][0]=1.0f-(yy+zz);
	res.m[0][1]=xy+wz;
	res.m[0][2]=xz-wy;
	res.m[1][0]=xy-wz;
	res.m[1][1]=1.0f-(xx+zz);
	res.m[1][2]=yz+wx;
	res.m[2][0]=xz+wy;
	res.m[2][1]=yz-wx;
	res.m[2][2]=1.0f-(xx+yy);
	return res;
}

inline V4 M33ToQuat(const M33& mat)
{
	V4 q;
	int nxt[3]={1,2,0};
	float tr,s;
	int i,j,k;
	tr=mat.m[0][0]+mat.m[1][1]+mat.m[2][2];
	if(tr > 0.0f)
	{
		s=sqrtf(tr+1.0f);
		q[3]=s*0.5f;
		s=0.5f/s;
		q[0]=(mat.m[1][2]-mat.m[2][1])*s;
		q[1]=(mat.m[2][0]-mat.m[0][2])*s;
		q[2]=(mat.m[0][1]-mat.m[1][0])*s;
	}
	else
	{
		i=0;
		if (mat.m[1][1] > mat.m[0][0]) i=1;
		if (mat.m[2][2] > mat.m[i][i]) i=2;
		j=nxt[i]; k=nxt[j];
		s=sqrtf( (mat.m[i][i]-(mat.m[j][j]+mat.m[k][k])) +1.0f);
		q[i]=s*0.5f;
		s=0.5f/s;
		q[3]=(mat.m[j][k]-mat.m[k][j])*s;
		q[j]=(mat.m[i][j]+mat.m[j][i])*s;
		q[k]=(mat.m[i][k]+mat.m[k][i])*s;
	}

	q[0]=q[0];
	q[1]=q[1];
	q[2]=q[2];

	return q;
}
*/

/*
 * M44 support.
 */

class M44 {
	public:
		inline M44() {}
		inline M44(const M44& m)=default;
		inline explicit M44(const float* data);
		inline M44(float e11, float e12, float e13, float e14,
						  float e21, float e22, float e23, float e24,
						  float e31, float e32, float e33, float e34,
						  float e41, float e42, float e43, float e44);
		inline M44(const V4& xa,const V4& ya,const V4& za,const V4& wa);

		// access grants
		//inline float& operator () (int row, int col)			{ return m[row][col]; }
		//inline float  operator () (int row, int col) const	{ return m[row][col]; }

		// casting operators
		inline operator float* ()							{ return m[0]; }
		inline operator const float * () const				{ return m[0]; }

		// assignment operators
		M44& operator=(const M44& mat);
		M44& operator *=(const M44& mat);
		M44& operator +=(const M44& mat);
		M44& operator -=(const M44& mat);
		M44& operator *=(float s);
		M44& operator /=(float s);

		// unary operators
		//M44 operator+() const										{ return *this; }
		//M44 operator-() const										{ M44 a=(*this); a*=-1.0f; return a; }

		// binary operators
		M44 operator * (const M44& mat) const;
		M44 operator+(const M44& mat) const						{ M44 a=(*this); a+=mat; return a; }
		M44 operator-(const M44& mat) const						{ M44 a=(*this); a-=mat; return a; }
		M44 operator * (float s) const								{ M44 a=(*this); a*=s; return a; }
		M44 operator / (float s) const								{ M44 a=(*this); a/=s; return a; }

		bool operator==(const M44& mat) const						{ return ArrayEquals(m[0], mat.m[0], 16); }
		bool operator!=(const M44& mat) const						{ return!ArrayEquals(m[0], mat.m[0], 16); }

		inline const V4& XAxis()const {
			return *(V4*)&m[0][0];
		}
		inline const V4& YAxis()const {
			return *(V4*)&m[1][0];
		}
		inline const V4& ZAxis()const {
			return *(V4*)&m[2][0];
		}
		inline const V4& WAxis()const {
			return *(V4*)&m[3][0];
		}

		inline const V4& Axis(const int axis) const {
			return *(V4*)&m[axis][0];
		}

		inline V4& XAxis() {
			return *(V4*)&m[0][0];
		}
		inline V4& YAxis() {
			return *(V4*)&m[1][0];
		}
		inline V4& ZAxis() {
			return *(V4*)&m[2][0];
		}
		inline V4& WAxis() {
			return *(V4*)&m[3][0];
		}


		inline void SetXAxis(const V4& vXAxis)const {
			*(V4*)&m[0][0]=vXAxis;
		}
		inline void SetYAxis(const V4& vYAxis)const {
			*(V4*)&m[1][0]=vYAxis;
		}
		inline void SetZAxis(const V4& vZAxis)const {
			*(V4*)&m[2][0] = vZAxis;
		}
		inline void SetWAxis(const V4& vWAxis)const {
			*(V4*)&m[3][0] = vWAxis;
		}

		/*
		struct e44 {
			float _11, _12, _13, _14;
			float _21, _22, _23, _24;
			float _31, _32, _33, _34;
			float _41, _42, _43, _44;
		};
		*/
		union {
			//e44 e;
			float m[4][4];
		};
};

inline M44::M44(const float* data)
{
	memcpy(*this, data, sizeof(M44));
}

inline M44 MTranspose(const M44& m) {
	M44 r;

	r.m[0][0]=m.m[0][0];
	r.m[1][0]=m.m[0][1];
	r.m[2][0]=m.m[0][2];
	r.m[3][0]=m.m[0][3];

	r.m[0][1]=m.m[1][0];
	r.m[1][1]=m.m[1][1];
	r.m[2][1]=m.m[1][2];
	r.m[3][1]=m.m[1][3];

	r.m[0][2]=m.m[2][0];
	r.m[1][2]=m.m[2][1];
	r.m[2][2]=m.m[2][2];
	r.m[3][2]=m.m[2][3];

	r.m[0][3]=m.m[3][0];
	r.m[1][3]=m.m[3][1];
	r.m[2][3]=m.m[3][2];
	r.m[3][3]=m.m[3][3];

	return r;
}

inline M44::M44(const V4& xa,const V4& ya,const V4& za,const V4& wa) {
	SetXAxis(xa);
	SetYAxis(ya);
	SetZAxis(za);
	SetWAxis(wa);
}

inline M44::M44(float e11, float e12, float e13, float e14,
					   float e21, float e22, float e23, float e24,
					   float e31, float e32, float e33, float e34,
					   float e41, float e42, float e43, float e44) {
	m[0][0]=e11;
	m[0][1]=e12;
	m[0][2]=e13;
	m[0][3]=e14;
	m[1][0]=e21;
	m[1][1]=e22;
	m[1][2]=e23;
	m[1][3]=e24;
	m[2][0]=e31;
	m[2][1]=e32;
	m[2][2]=e33;
	m[2][3]=e34;
	m[3][0]=e41;
	m[3][1]=e42;
	m[3][2]=e43;
	m[3][3]=e44;
}

inline M44& M44::operator=(const M44& mat)
{
	memcpy(*this, &mat, sizeof(M44));
	return *this;
}
/*
inline M44& M44::operator *=(const M44& mat)
{
	D3DXMatrixMultiply((D3DXMATRIX*)this, (const D3DXMATRIX*)this, (const D3DXMATRIX*)&mat);
	return *this;
}
*/
inline M44& M44::operator +=(const M44& mat)
{
	for(int row=0; row<4; row++)
		for(int col=0; col<4; col++)
			m[row][col] +=mat.m[row][col];
	return *this;
}

inline M44& M44::operator -=(const M44& mat)
{
	for(int row=0; row<4; row++)
		for(int col=0; col<4; col++)
			m[row][col] -=mat.m[row][col];
	return *this;
}

inline M44& M44::operator *=(float s)
{
	for(int row=0; row<4; row++)
		for(int col=0; col<4; col++)
			m[row][col] *=s;
	return *this;
}

inline M44& M44::operator /=(float s)
{
	for(int row=0; row<4; row++)
		for(int col=0; col<4; col++)
			m[row][col] /=s;
	return *this;
}

inline M44 operator * (float s, const M44& mat)
{
	return mat*s;
}

inline V4 TransformVec4(const V4 &in, const M44 &m)
{
	V4 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0]+in.w*m.m[3][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1]+in.w*m.m[3][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2]+in.w*m.m[3][2];
	out.w=in.x*m.m[0][3]+in.y*m.m[1][3]+in.z*m.m[2][3]+in.w*m.m[3][3];
	return out;
}

inline V4 TransformVec4(const M44 &m,const V4 &in)
{
	V4 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[0][1]+in.z*m.m[0][2]+in.w*m.m[0][3];
	out.y=in.x*m.m[1][0]+in.y*m.m[1][1]+in.z*m.m[1][2]+in.w*m.m[1][3];
	out.z=in.x*m.m[2][0]+in.y*m.m[2][1]+in.z*m.m[2][2]+in.w*m.m[2][3];
	out.w=in.x*m.m[3][0]+in.y*m.m[3][1]+in.z*m.m[3][2]+in.w*m.m[2][3];
	return out;
}

inline V4 operator * (const M44 &m, const V4 &in)
{
	return TransformVec4(in, m);
}

inline V4 operator * (const V4 &in, const M44 &m)
{
	return TransformVec4(in, m);
}

/*
const Vec4 operator *( const Mat44 &m, const Vec4 &v )
{
			Vec4 r;

			r.x=m.m[0][0]*v.x+m.m[0][1]*v.y+m.m[0][2]*v.z+m.m[0][3]*v.w;
			r.y=m.m[1][0]*v.x+m.m[1][1]*v.y+m.m[1][2]*v.z+m.m[1][3]*v.w;
			r.z=m.m[2][0]*v.x+m.m[2][1]*v.y+m.m[2][2]*v.z+m.m[2][3]*v.w;
			r.w=m.m[3][0]*v.x+m.m[3][1]*v.y+m.m[3][2]*v.z+m.m[3][3]*v.w;

			return r;
}
inline void Inverse(M44 &m, float *det=NULL)
{
	D3DXMatrixInverse((D3DXMATRIX*)&m,det,(const D3DXMATRIX*)&m);
}

inline void Inverse(M44 &dst, const M44 &in, float *det=NULL)
{
	D3DXMatrixInverse((D3DXMATRIX*)&dst,det,(const D3DXMATRIX*)&in);
}
*/

inline M44 MInverse(const M44 &m) {
    M44 r;

    float f22Det1=m.m[2][2]*m.m[3][3]-m.m[2][3]*m.m[3][2];
    float f22Det2=m.m[2][1]*m.m[3][3]-m.m[2][3]*m.m[3][1];
    float f22Det3=m.m[2][1]*m.m[3][2]-m.m[2][2]*m.m[3][1];
    float f22Det4=m.m[2][0]*m.m[3][3]-m.m[2][3]*m.m[3][0];
    float f22Det5=m.m[2][0]*m.m[3][2]-m.m[2][2]*m.m[3][0];
    float f22Det6=m.m[2][0]*m.m[3][1]-m.m[2][1]*m.m[3][0];
    float f22Det7=m.m[1][2]*m.m[3][3]-m.m[1][3]*m.m[3][2];
    float f22Det8=m.m[1][1]*m.m[3][3]-m.m[1][3]*m.m[3][1];
    float f22Det9=m.m[1][1]*m.m[3][2]-m.m[1][2]*m.m[3][1];
    float f22Det10=m.m[1][2]*m.m[2][3]-m.m[1][3]*m.m[2][2];
    float f22Det11=m.m[1][1]*m.m[2][3]-m.m[1][3]*m.m[2][1];
    float f22Det12=m.m[1][1]*m.m[2][2]-m.m[1][2]*m.m[2][1];
    float f22Det13=m.m[1][0]*m.m[3][3]-m.m[1][3]*m.m[3][0];
    float f22Det14=m.m[1][0]*m.m[3][2]-m.m[1][2]*m.m[3][0];
    float f22Det15=m.m[1][0]*m.m[2][3]-m.m[1][3]*m.m[2][0];
    float f22Det16=m.m[1][0]*m.m[2][2]-m.m[1][2]*m.m[2][0];
    float f22Det17=m.m[1][0]*m.m[3][1]-m.m[1][1]*m.m[3][0];
    float f22Det18=m.m[1][0]*m.m[2][1]-m.m[1][1]*m.m[2][0];

    float fFirst33Det=m.m[1][1]*f22Det1-m.m[1][2]*f22Det2+m.m[1][3]*f22Det3;
    float fSec33Det  =m.m[1][0]*f22Det1-m.m[1][2]*f22Det4+m.m[1][3]*f22Det5;
    float fThird33Det=m.m[1][0]*f22Det2-m.m[1][1]*f22Det4+m.m[1][3]*f22Det6;
    float fFourth33Det=m.m[1][0]*f22Det3-m.m[1][1]*f22Det5+m.m[1][2]*f22Det6;

    float fDet44=m.m[0][0]*fFirst33Det-m.m[0][1]*fSec33Det+m.m[0][2]*fThird33Det-m.m[0][3]*fFourth33Det;

    float s=1.0f / fDet44;

    r.m[0][0]=s * fFirst33Det;
    r.m[0][1]=-s * ( m.m[0][1]*f22Det1-m.m[0][2]*f22Det2+m.m[0][3]*f22Det3 );
    r.m[0][2]=s * ( m.m[0][1]*f22Det7-m.m[0][2]*f22Det8+m.m[0][3]*f22Det9 );
    r.m[0][3]=-s * ( m.m[0][1]*f22Det10-m.m[0][2]*f22Det11+m.m[0][3]*f22Det12 );

    r.m[1][0]=-s * fSec33Det;
    r.m[1][1]=s * ( m.m[0][0]*f22Det1-m.m[0][2]*f22Det4+m.m[0][3]*f22Det5 );
    r.m[1][2]=-s * ( m.m[0][0]*f22Det7-m.m[0][2]*f22Det13+m.m[0][3]*f22Det14 );
    r.m[1][3]=s * ( m.m[0][0]*f22Det10-m.m[0][2]*f22Det15+m.m[0][3]*f22Det16 );

    r.m[2][0]=s * fThird33Det;
    r.m[2][1]=-s * ( m.m[0][0]*f22Det2-m.m[0][1]*f22Det4+m.m[0][3]*f22Det6 );
    r.m[2][2]=s * ( m.m[0][0]*f22Det8-m.m[0][1]*f22Det13+m.m[0][3]*f22Det17 );
    r.m[2][3]=-s * ( m.m[0][0]*f22Det11-m.m[0][1]*f22Det15+m.m[0][3]*f22Det18 );

    r.m[3][0]=-s * fFourth33Det;
    r.m[3][1]=s * ( m.m[0][0]*f22Det3-m.m[0][1]*f22Det5+m.m[0][2]*f22Det6 );
    r.m[3][2]=-s * ( m.m[0][0]*f22Det9-m.m[0][1]*f22Det14+m.m[0][2]*f22Det17 );
    r.m[3][3]=s * ( m.m[0][0]*f22Det12-m.m[0][1]*f22Det16+m.m[0][2]*f22Det18 );

    return r;
}

inline M44 MAffineInverse(const M44& mOper1)
{
	M44 mResult;

	// The rotational part of the matrix is simply the transpose of the original matrix
	mResult.m[0][0]=mOper1.m[0][0];
	mResult.m[1][0]=mOper1.m[0][1];
	mResult.m[2][0]=mOper1.m[0][2];

	mResult.m[0][1]=mOper1.m[1][0];
	mResult.m[1][1]=mOper1.m[1][1];
	mResult.m[2][1]=mOper1.m[1][2];

	mResult.m[0][2]=mOper1.m[2][0];
	mResult.m[1][2]=mOper1.m[2][1];
	mResult.m[2][2]=mOper1.m[2][2];

	// The right column vector of the matrix should always be [ 0 0 0 1 ]
	// In most cases. . . you don't need this column at all because it'll
	// never be used in the program, but since this code is used with GL
	// and it does consider this column, it is here.
	mResult.m[0][3]=0;
	mResult.m[1][3]=0;
	mResult.m[2][3]=0;
	mResult.m[3][3]=1;

	// The translation components of the original matrix.
	float fTx=mOper1.m[3][0];
	float fTy=mOper1.m[3][1];
	float fTz=mOper1.m[3][2];

	// Result=-(Tm * Rm) to get the translation part of the inverse
	mResult.m[3][0]=-(mOper1.m[0][0] * fTx+mOper1.m[0][1] * fTy+mOper1.m[0][2] * fTz);
	mResult.m[3][1]=-(mOper1.m[1][0] * fTx+mOper1.m[1][1] * fTy+mOper1.m[1][2] * fTz);
	mResult.m[3][2]=-(mOper1.m[2][0] * fTx+mOper1.m[2][1] * fTy+mOper1.m[2][2] * fTz);

	return mResult;
}

inline M44 Multiply(const M44 &m1,const M44 &m2) {
	M44 r;
	for(int iCol=0; iCol < 4; iCol++) {
		r.m[0][iCol]=m1.m[0][0]*m2.m[0][iCol]+m1.m[0][1]*m2.m[1][iCol]+m1.m[0][2]*m2.m[2][iCol]+m1.m[0][3]*m2.m[3][iCol];
		r.m[1][iCol]=m1.m[1][0]*m2.m[0][iCol]+m1.m[1][1]*m2.m[1][iCol]+m1.m[1][2]*m2.m[2][iCol]+m1.m[1][3]*m2.m[3][iCol];
		r.m[2][iCol]=m1.m[2][0]*m2.m[0][iCol]+m1.m[2][1]*m2.m[1][iCol]+m1.m[2][2]*m2.m[2][iCol]+m1.m[2][3]*m2.m[3][iCol];
		r.m[3][iCol]=m1.m[3][0]*m2.m[0][iCol]+m1.m[3][1]*m2.m[1][iCol]+m1.m[3][2]*m2.m[2][iCol]+m1.m[3][3]*m2.m[3][iCol];
	}
	return r;
}

//bool operator==( const Mat44 &v1, const Mat44 &V2 ) {
//		   for ( int iCol=0; iCol < 4; iCol++ )
//					  if ( v1.m[0][iCol]!=V2.m[0][iCol] || v1.m[1][iCol]!=V2.m[1][iCol] || v1.m[2][iCol]!=V2.m[2][iCol] || v1.m[3][iCol]!=V2.m[3][iCol] ) return false;
//		   return true;
//}

/*
inline float GetDeterminant(const M44& m)
{
	return D3DXMatrixDeterminant((const D3DXMATRIX*)&m);
}
inline bool IsValid(const M44& m)
{
	bool invertible=GetDeterminant(m)!=0.0f;

	return	invertible && _finite(m.m[0][0]) && _finite(m.m[0][1]) && _finite(m.m[0][2]) && _finite(m.m[0][3]) &&
			_finite(m.m[1][0]) && _finite(m.m[1][1]) && _finite(m.m[1][2]) && _finite(m.m[1][3]) &&
			_finite(m.m[2][0]) && _finite(m.m[2][1]) && _finite(m.m[2][2]) && _finite(m.m[2][3]) &&
			_finite(m.m[3][0]) && _finite(m.m[3][1]) && _finite(m.m[3][2]) && _finite(m.m[3][3]);
}

inline void Transpose(M44 &m)
{
	D3DXMatrixTranspose((D3DXMATRIX*)&m,(const D3DXMATRIX*)&m);
}

inline void Transpose(M44 &dst, const M44 &in)
{
	D3DXMatrixTranspose((D3DXMATRIX*)&dst,(const D3DXMATRIX*)&in);
}

inline void	RotationAxisAngle(M44 &dst, const V3 &axis, float angle)
{
	D3DXMatrixRotationAxis((D3DXMATRIX*)&dst, (D3DXVECTOR3*)&axis, angle);
}

inline void	LookAtLH(M44 &dst, const V3& from, const V3& to, const V3& up)
{
	D3DXMatrixLookAtLH((D3DXMATRIX*)&dst,(D3DXVECTOR3*)&from,(D3DXVECTOR3*)&to,(D3DXVECTOR3*)&up);
}

inline void	PerspectiveFovLH(M44 &dst, float fov_y, float aspect, float znear, float zfar)
{
	D3DXMatrixPerspectiveFovLH((D3DXMATRIX*)&dst,fov_y,aspect,znear,zfar);
}

inline void OrthoLH(M44 &dst, float width, float height, float znear, float zfar)
{
	D3DXMatrixOrthoLH((D3DXMATRIX*)&dst, width, height, znear, zfar);
}

inline void OrthoOffCenterLH(M44 &dst, float minx, float maxx, float miny, float maxy, float minz, float maxz)
{
	D3DXMatrixOrthoOffCenterLH((D3DXMATRIX*)&dst, minx, maxx, miny, maxy, minz, maxz);
}
*/

inline M44 PreRotationMatrix(const V3& vAngle) {
	float sx = sinf(vAngle.x);
	float cx = cosf(vAngle.x);
	float sy = sinf(vAngle.y);
	float cy = cosf(vAngle.y);
	float sz = sinf(vAngle.z);
	float cz = cosf(vAngle.z);

	return {
		{ cy*cz, cz*sx*sy - cx*sz, cx*cz*sy + sx*sz, 0 },
		{ cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - cz*sx, 0 },
		{ -sy, cy*sx, cx*cy, 0 },
		{ 0, 0, 0, 1 }
	};
}

inline void SetTranslation4(M44 &mat, const V4 &vector)
{
	mat.m[3][0]=vector.x;
	mat.m[3][1]=vector.y;
	mat.m[3][2]=vector.z;
	mat.m[3][3]=vector.w;
}

inline void SetTranslation(M44 &mat, float x, float y, float z)
{
	mat.m[3][0]=x;
	mat.m[3][1]=y;
	mat.m[3][2]=z;
}

inline const V4& GetTranslation4(const M44& mat)
{
	return *(const V4*)&mat.m[3][0];
}

inline void Lerp(M44& out, const M44& in0, const M44& in1, float i) // 0 will give you all in0, 1 will give you all in1
{
	float	oi=1.0f-i;

	out.m[0][0]=in0.m[0][0]*oi+in1.m[0][0]*i;
	out.m[0][1]=in0.m[0][1]*oi+in1.m[0][1]*i;
	out.m[0][2]=in0.m[0][2]*oi+in1.m[0][2]*i;
	out.m[0][3]=in0.m[0][3]*oi+in1.m[0][3]*i;

	out.m[1][0]=in0.m[1][0]*oi+in1.m[1][0]*i;
	out.m[1][1]=in0.m[1][1]*oi+in1.m[1][1]*i;
	out.m[1][2]=in0.m[1][2]*oi+in1.m[1][2]*i;
	out.m[1][3]=in0.m[1][3]*oi+in1.m[1][3]*i;

	out.m[2][0]=in0.m[2][0]*oi+in1.m[2][0]*i;
	out.m[2][1]=in0.m[2][1]*oi+in1.m[2][1]*i;
	out.m[2][2]=in0.m[2][2]*oi+in1.m[2][2]*i;
	out.m[2][3]=in0.m[2][3]*oi+in1.m[2][3]*i;

	out.m[3][0]=in0.m[3][0]*oi+in1.m[3][0]*i;
	out.m[3][1]=in0.m[3][1]*oi+in1.m[3][1]*i;
	out.m[3][2]=in0.m[3][2]*oi+in1.m[3][2]*i;
	out.m[3][3]=in0.m[3][3]*oi+in1.m[3][3]*i;
}

inline M44 MTranspose33(const M44& m) {
	M44 m1;
	m1.m[0][0]=m.m[0][0];
	m1.m[0][1]=m.m[1][0];
	m1.m[0][2]=m.m[2][0];

	m1.m[0][3]=m.m[0][3];

	m1.m[1][0]=m.m[0][1];
	m1.m[1][1]=m.m[1][1];
	m1.m[1][2]=m.m[2][1];

	m1.m[1][3]=m.m[1][3];

	m1.m[2][0]=m.m[0][2];
	m1.m[2][1]=m.m[1][2];
	m1.m[2][2]=m.m[2][2];

	m1.m[2][3]=m.m[2][3];

	m1.m[3][0]=m.m[3][0];
	m1.m[3][1]=m.m[3][1];
	m1.m[3][2]=m.m[3][2];
	m1.m[3][3]=m.m[3][3];
	return m1;
}

inline void SetTranslation(M44 &mat, const V3 &vector)
{
	mat.m[3][0]=vector.x;
	mat.m[3][1]=vector.y;
	mat.m[3][2]=vector.z;
}

inline V3 MulNoTrans(const M44 &m, const V3 &in) // multiply V3 by V4 as if there was no transformation-useful for normals etc
{
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2];
	return out;
}

inline V3 MulTrans(const M44 &m, const V3 &in)
{
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0]+m.m[3][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1]+m.m[3][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2]+m.m[3][2];
	return out;
}

inline V3 TransformNormal(const V3 &in, const M44 &m)
{
	return MulNoTrans(m, in);
}

inline V3 TransformCoord(const V3 &in, const M44 &m)
{
	return MulTrans(m, in);
}

inline V3 TransformVec3(const V3 &in, const M44 &m)
{
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+in.z*m.m[2][0]+m.m[3][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+in.z*m.m[2][1]+m.m[3][1];
	out.z=in.x*m.m[0][2]+in.y*m.m[1][2]+in.z*m.m[2][2]+m.m[3][2];
	//out.w=in.x*m.m[0][3]+in.y*m.m[1][3]+in.z*m.m[2][3]+m.m[3][3];
	return out;
}
/*
inline V3 TransformVec3(const M44 &m,const V3 &in)
{
	V3 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[0][0]+in.z*m.m[0][2]+m.m[0][3];
	out.y=in.x*m.m[1][0]+in.y*m.m[1][0]+in.z*m.m[1][2]+m.m[1][3];
	out.z=in.x*m.m[2][0]+in.y*m.m[2][0]+in.z*m.m[2][2]+m.m[2][3];
	//out.w=in.x*m.m[3][0]+in.y*m.m[3][0]+in.z*m.m[3][2]+m.m[3][3];
	return out;
}
*/
inline V2 TransformCoord(const V2 &in, const M44 &m)
{
	// D3DXVec2TransformCoord() does division by w
	// V4 v=TransformVec4(V4(in.x, in.y, 0, 1), m);
	// V2 projected=V2(v.x, v.y)/v.w;
	// We can use a simpler version assuming w=1:
	V2 out;
	out.x=in.x*m.m[0][0]+in.y*m.m[1][0]+m.m[3][0];
	out.y=in.x*m.m[0][1]+in.y*m.m[1][1]+m.m[3][1];
	return out;
}

inline V3 operator * (const M44 &m, const V3 &in)
{
	return MulTrans(m, in);
}

inline V3 operator * (const V3 &in, const M44 &m)
{
	return MulTrans(m, in);
}

/*
 * M44 component access.
 */

inline const V3& GetX(const M44 &m)
{
	return *(const V3*)&m.m[0][0];
}

inline const V3& GetY(const M44 &m)
{
	return *(const V3*)&m.m[1][0];
}

inline const V3& GetZ(const M44 &m)
{
	return *(const V3*)&m.m[2][0];
}

inline const V3& GetT(const M44 &m)
{
	return *(const V3*)&m.m[3][0];
}

inline V3& GetX( M44 &m)
{
	return *(V3*)&m.m[0][0];
}

inline V3& GetY( M44 &m)
{
	return *(V3*)&m.m[1][0];
}

inline V3& GetZ( M44 &m)
{
	return *(V3*)&m.m[2][0];
}

inline V3& GetT( M44 &m)
{
	return *(V3*)&m.m[3][0];
}

inline const V3& GetTranslation(const M44& mat)
{
	return *(const V3*)&mat.m[3][0];
}

inline M33 MRotation(const M44& m) {
	M33 m1(	m.m[0][0],m.m[0][1],m.m[0][2],
			m.m[1][0],m.m[1][1],m.m[1][2],
			m.m[2][0],m.m[2][1],m.m[2][2]);
	return m1;
}

inline M44 MLoadIdentity() {
	return {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};
}

inline void MLoad(M44& m44,const M33& m) {
	m44.m[0][0]=m.m[0][0];
	m44.m[0][1]=m.m[0][1];
	m44.m[0][2]=m.m[0][2];
	m44.m[1][0]=m.m[1][0];
	m44.m[1][1]=m.m[1][1];
	m44.m[1][2]=m.m[1][2];
	m44.m[2][0]=m.m[2][0];
	m44.m[2][1]=m.m[2][1];
	m44.m[2][2]=m.m[2][2];
}

inline M44 MLoad(const M33& m) {
	M44 m1(	m.m[0][0],m.m[0][1],m.m[0][2],0,
			m.m[1][0],m.m[1][1],m.m[1][2],0,
			m.m[2][0],m.m[2][1],m.m[2][2],0,
			0,0,0,1);
	return m1;
}

inline M44 MLoad(const M33& m,const V3& v) {
	M44 m1(	m.m[0][0],m.m[0][1],m.m[0][2],0,
			m.m[1][0],m.m[1][1],m.m[1][2],0,
			m.m[2][0],m.m[2][1],m.m[2][2],0,
			v.x,v.y,v.z,1);
	return m1;
}

inline void CreateScalingAndTranslation(M44 &mat, const V3 &scale, const V3 &offset)
{
	mat.m[0][0]	=scale.x;	mat.m[0][1]	=0;		mat.m[0][2]	=0;		mat.m[0][3]	=0;
	mat.m[1][0]	=0;		mat.m[1][1]	=scale.y;	mat.m[1][2]	=0;		mat.m[1][3]	=0;
	mat.m[2][0]	=0;		mat.m[2][1]	=0;		mat.m[2][2]	=scale.z;	mat.m[2][3]	=0;
	mat.m[3][0]	=offset.x;	mat.m[3][1]	=offset.y;	mat.m[3][2]	=offset.z;	mat.m[3][3]	=1.0f;
}

inline void CreateTranslation(M44 &mat, const V3 &offset)
{
	CreateScalingAndTranslation(mat, V3(1,1,1), offset);
}

inline void CreateScaling(M44 &mat, const V3 &scale)
{
	CreateScalingAndTranslation(mat, scale, V3(0,0,0));
}

inline void CreateScaling(M44 &mat, float scale)
{
	CreateScaling(mat, V3(scale, scale, scale));
}

inline M44 CreateMatFromZAxisUserY(V3 zaxis,const V3& vUp,const V3& translation) {
	M44 m=MLoad(CreateMat33FromZAxisUserY(zaxis,vUp),translation);
	return m;
}

inline M44 CreateMatFromXAxis(V3 xaxis)
{
	xaxis=VNormalize(xaxis);
	V3 za=VNormalize(VCross(xaxis,V3(0,1,0)));
	M44 m = MLoadIdentity();
	GetX(m)=xaxis;
	GetY(m)=VCross(za,xaxis);
	GetZ(m)=za;
	return m;
}

inline M44 CreateMatFromYAxis(V3 yaxis)
{
	yaxis=VNormalize(yaxis);
	V3 xa=VNormalize(VCross(yaxis, V3(0,0,1)));
	M44 m = MLoadIdentity();
	GetY(m)=yaxis;
	GetZ(m)=VCross(xa, yaxis);
	GetX(m)=xa;
	return m;
}

inline M44 CreateMatFromZAxis(V3 zaxis,bool bLockY=true) {
	zaxis=VNormalize(zaxis);
	M44 m=MLoadIdentity();
	if(bLockY) {
		V3 xa=VNormalize(VCross(V3(0,1,0),zaxis));
		GetZ(m)=zaxis;
		GetY(m)=VCross(zaxis,xa);
		GetX(m)=xa;
	}else{
		V3 ya=VNormalize(VCross(zaxis, V3(1,0,0)));
		GetZ(m)=zaxis;
		GetX(m)=VCross(ya, zaxis);
		GetY(m)=ya;
	}
	return m;

}


inline void Renormalize(M44 &mat) {
	V3 z=GetZ(mat);
	V3 x=GetX(mat);
	V3 y=VNormalize(VCross(z,x));
	GetX(mat)=VNormalize(VCross(y,z));
	GetY(mat)=y;
	GetZ(mat)=VNormalize(z);
}

inline void LookAtLH( V3 eye,V3 target,V3 up,M44& m) {
	V3 zaxis=VNormalize(target-eye);
	V3 xaxis=VNormalize(VCross(up,zaxis));
	V3 yaxis=VCross( zaxis,xaxis);

	m.m[0][0]=xaxis.x;          m.m[0][1]=yaxis.x;          m.m[0][2]=zaxis.x;          m.m[0][3]=0;
	m.m[1][0]=xaxis.y;          m.m[1][1]=yaxis.y;          m.m[1][2]=zaxis.y;          m.m[1][3]=0;
	m.m[2][0]=xaxis.z;          m.m[2][1]=yaxis.z;          m.m[2][2]=zaxis.z;          m.m[2][3]=0;
	m.m[3][0]=-VDot(xaxis,eye);	m.m[3][1]=-VDot(yaxis,eye);	m.m[3][2]=-VDot(zaxis,eye);	m.m[3][3]=1;

}

inline void LookAtRH( V3 eye,V3 target,V3 up,M44& m) {
	V3 zaxis=VNormalize(eye-target);
	V3 xaxis=VNormalize(VCross(up,zaxis));
	V3 yaxis=VCross(zaxis,xaxis );

	m.m[0][0]=xaxis.x;				m.m[0][1]=yaxis.x;				m.m[0][2]=zaxis.x;				m.m[0][3]=0;
	m.m[1][0]=xaxis.y;				m.m[1][1]=yaxis.y;				m.m[1][2]=zaxis.y;				m.m[1][3]=0;
	m.m[2][0]=xaxis.z;				m.m[2][1]=yaxis.z;				m.m[2][2]=zaxis.z;				m.m[2][3]=0;
	m.m[3][0]=-VDot(xaxis,eye);		m.m[3][1]=-VDot(yaxis,eye);		m.m[3][2]=-VDot(zaxis,eye);		m.m[3][3]=1;
}

M44 PerspectiveFovLH( float fovy,float aspect,float zn,float zf );
M44 M44PerspectiveFovRH(float fFovY,float fAspectWByH,float fNear,float fFar);
M44 LookAtPlane(float fov,float Height,V3 plane_normal,V3 plane_pos,V3 WorldUp);
M44 M44PerspectiveRH(float fWidth,float fHeight,float fNear,float fFar);
M44 M44PerspectiveInfinite(float fLeft,float fRight,float fTop,float fBottom,float fNear);
M44 M44Ortho(float fLeft,float fRight,float fBottom,float fTop,float fNear,float fFar);

inline V3 GetAxis(const EAxis eAxis) {
	static const V3 axis[6]={
		{1.f, 0.f, 0.f},
		{0.f, 1.f, 0.f},
		{0.f, 0.f, 1.f},
		{-1.f, 0.f, 0.f},
		{0.f, -1.f, 0.f},
		{0.f, 0.f, -1.f}
	};
	return axis[eAxis];
}

class RectV2 {
	public:
		RectV2(float xa,float ya,float widtha,float heighta){x=xa;y=ya;width=widtha;height=heighta;}
		RectV2(const V2& tl,float w,float h){x=tl.x;y=tl.y;width=w;height=h;}
		RectV2(){x=0;y=0;width=0;height=0;}
		RectV2(const V2& tl,const V2& br){x=tl.x;y=tl.y;width=br.x-tl.x;height=br.y-tl.y;}
		V2 Center()const{return V2(x+(width*0.5f),y+(height*0.5f));}
		V2 tl()const{return V2(x,y);}
		V2 br()const{return V2(x+width,y+height);}
		V2 TopLeft()const{return V2(x,y);}
		V2 BottomRight()const{return V2(x+width,y+height);}
		V2 Size()const{return V2(width,height);}
		bool Inside(const V2& a)const {
			if(x<a.x && y<a.y && x+width>=a.x && y+height>=a.y)
				return true;
			return false;
		}
		RectV2 Intersection(const RectV2& b)const {
			RectV2 a=*this;
			float x1 = MAX(a.x, b.x), y1 = MAX(a.y, b.y);
			a.width = MIN(a.x + a.width, b.x + b.width) - x1;
			a.height = MIN(a.y + a.height, b.y + b.height) - y1;
			a.x = x1; a.y = y1;
			if( a.width <= 0 || a.height <= 0 )
				a = RectV2();
			return a;
		}
		RectV2 Union(const RectV2& b)const {
			RectV2 a=*this;
			float x1 = MIN(a.x, b.x), y1 = MIN(a.y, b.y);
			a.width = MAX(a.x + a.width, b.x + b.width) - x1;
			a.height = MAX(a.y + a.height, b.y + b.height) - y1;
			a.x = x1; a.y = y1;
			return a;
		}
		RectV2 operator & (const RectV2& a)const {
			return Intersection(a);
		}
		RectV2 operator | (const RectV2& a)const {
			return Union(a);
		}
		float area()const{return height*width;}
		float height;
		float width;
		float x;
		float y;
};

inline float sign(float v) {
	return v >=0.0f ? 1.0f : -1.0f;
}

inline bool RoughlyEqual(float a,float b,float fLimit=0.001f)
{
	return fabsf(a-b)<fLimit;
}

inline int Min(int a, int b)
{
	return a<b ? a:b;
}
inline int Max(int a, int b)
{
	return a>b ? a:b;
}

inline float Min(float a, float b)
{
	return a<b ? a:b;
}
inline float Max(float a, float b)
{
	return a>b ? a:b;
}

inline float VLerp(float a,float b, float s)
{
	return a * (1-s)+b * s;
}

bool vangpul(V3& vres,const V3& vsource,const V3& vdest,float maxmoveangle);

#define COL32A(r,g,b,a) ( ((int)a<<24)|(BOUND(0,r,255)<<16) | (BOUND(0,g,255)<<8) | (BOUND(0,b,255)) )

inline V3 uint322V3(uint32_t d)
{
	return V3(((d>>16)&255)/255.f,((d>>8)&255)/255.f,((d>>0)&255)/255.f);
}
inline V4 uint322V4(uint32_t d)
{
	return V4(((d>>24)&255)/255.f,((d>>16)&255)/255.f,((d>>8)&255)/255.f,((d>>0)&255)/255.f);
}
inline V4 VLoadABGR(uint32_t d)
{
	return V4(((d>>24)&255)/255.f,((d>>16)&255)/255.f,((d>>8)&255)/255.f,((d>>0)&255)/255.f);
}
inline V4 VLoadRGBA(uint32_t d) {
	return V4(((d>>0)&255)/255.f,((d>>8)&255)/255.f,((d>>16)&255)/255.f,((d>>24)&255)/255.f);
}

inline uint32_t V42uint32(const V4 &v,float scalar=255)
{
	int r=int(v.x*scalar);
	int g=int(v.y*scalar);
	int b=int(v.z*scalar);
	int a=int(v.w*scalar);
	return (BOUND(0,a,255)<<24)|(BOUND(0,r,255)<<16)|(BOUND(0,g,255)<<8)|(BOUND(0,b,255));
}


//angle bc = acos((b*b + c*c - a*a)/(2*b*c));
inline float CosRelaLLL(float a,float b,float c) {
	if(a+b<=c)return 0;
	return acosf((b*b+c*c-a*a)/(2*b*c));
}

//length a=sqr(b*b+c*c-2*b*c*cos(bc))
inline float CosRelaLAL(float b,float bc,float c) {
	return sqrtf(b*b+c*c-2*b*c*cosf(bc));
}


inline float CosRelaLLA(float a,float b,float ac) {
	float konstant,sinBC,bc,ab,sinAC;
	sinAC = sinf(ac);
	if (sinAC == 0.0f) {
		return fabsf(a);
	}
	konstant = b/sinAC;;
	sinBC = a / konstant;
	bc = asinf(sinBC);
	ab = PI - bc - ac;
	return sinf(ab) * konstant;
}


inline float GetAngle(const float x,const float y) {
	float v;
	if(x==0) {
		if(0<y) {
			v=DEGREES_TO_RADIANS(90.0f);
		}else{
			v=DEGREES_TO_RADIANS(270.0f);
		}
	}else{
		v=(float)atan(y/x);
		if(x<0) {
			v+=DEGREES_TO_RADIANS(180.0f);
		}else
		if(y<0) {
			v+=DEGREES_TO_RADIANS(360.0f);
		}
	}
	return v;
}

inline V3 SnapPosition(const V3& pos, const V3& snap_resolution) {
	V3 p=pos * V3(1.0f / snap_resolution.x, 1.0f / snap_resolution.y, 1.0f / snap_resolution.z);
	p.x=floorf(p.x+0.5f);
	p.y=floorf(p.y+0.5f);
	p.z=floorf(p.z+0.5f);
	return p * snap_resolution;
}

	inline  V3	NearClipPlaneToWorld(const M44& invViewProj, const V2& clip_pos)
	{
		V4	p(clip_pos.x,clip_pos.y,0.0f,1.0f);
		V4	tp=p*invViewProj;
		return V3(tp.x/tp.w,tp.y/tp.w,tp.z/tp.w);
	}

/* FIXME remove this (unused).
	inline void ClipSpaceToTextureSpace(M44& m)
	{
		PostScale(m, 0.5f, -0.5f, 1.0f, 1.0f); // scale all axes components non-homogeneously by (0.5, -0.5, 1.0, 1.0)
		PostTranslate(m, .5f, .5f, 0); // sets last column of m
	}
*/

/*
	inline V3 SnapPosition(const V3& pos, const V3& snap_resolution, const M44& transform)
	{
		M44 inv_transform;
		Inverse(inv_transform, transform);

		return MulTrans(transform, SnapPosition(MulTrans(inv_transform, pos), snap_resolution));
	}
*/
bool IntersectLineAABB(float* pT,float minX,float minY,float minZ,float maxX,float maxY,float maxZ,float linePosX,float linePosY,float linePosZ,float lineVecX,float lineVecY,float lineVecZ);
inline bool RayBoxIntersect(float* time,const V3& lp,const V3& ln,const V3& lens) {
	return IntersectLineAABB(time,-lens.x,-lens.y,-lens.z,lens.x,lens.y,lens.z,lp.x,lp.y,lp.z,ln.x,ln.y,ln.z);
}
void  VSnapToGrid(V3& v, float fGridSize=50.0f);
bool  AABBAABBIntersectXY(const V3& vAMin,const V3& vAMax, const V3& vBMin, const V3& vBMax, float fExtraSize=0.0f);
//bool  SegmentsIntersect(const V3& p1,const V3& p2,const V3& q1,const V3& q2);
float DistancePointRay(const V3& vPoint,const V3& vLineStart,const V3& vLineDirection);
float DistancePointLineSegment(const V3& vPoint,const V3& vLineStart,const V3& vLineEnd);
float DistancePointLineSegment(const V2& vPoint,const V2& vLineStart,const V2& vLineEnd);

bool PointInsidePoly(const V2& pt, const V2* verts, size_t nverts);

bool ClosestPointOnRayToRay(V3* p,const V3& rayPositionA,const V3& rayDirectionA,const V3& rayPositionB,const V3& rayDirectionB);
bool ProjectDistanceOnRayToRay(float* p,const V3& rayPositionA,const V3& rayDirectionA,const V3& rayPositionB,const V3& rayDirectionB);
float AABBROI(const V3& minA,const V3& maxA,const V3& minB,const V3& maxB);
float DistanceRays(const V3& rayPositionA,const V3& rayDirectionA,const V3& rayPositionB,const V3& rayDirectionB);

V3 NearestPointRay(const V3& point,const V3& start,const V3& direction);
V3 NearestPointLineSegment(const V3& vPoint,const V3& vLineStart,const V3& vLineEnd);
V2 NearestPointLineSegment(const V2& vPoint,const V2& vLineStart,const V2& vLineEnd);

bool IntersectLineAndPlane(V3& vIntersection, const V3& vLine0, const V3& vLine1, const V3& vVertex, const V3& vNormal);
bool IntersectRayAndPlane(V3* pvIntersection, const V3& vLinePosition, const V3& vLineDirection, const V3& vVertex, const V3& vNormal);
bool IntersectRaySphere(const V3& origin, const V3& direction, const V3& sphere_center, const float& sphere_radius, float& tmin, float& tmax);
bool IntersectRaySphere(const V2& origin, const V2& direction, const V2& sphere_center, const float& sphere_radius, float& tmin, float& tmax);
//bool CheckLineTriangleIntersection(V3* intersection,float* fraction,const V3& lineStart, const V3& lineEnd, const V3& vVertex0, const V3& vVertex1, const V3& vVertex2);
bool RayTriangleIntersect(float* t,const V3& P,const V3& w,const V3& v0,const V3& v1,const V3& v2,float b[3]);

float DistanceLineSegmentLineSegment(const V2& s0,const V2& e0,const V2& s1,const V2& e1);
float DistanceLineSegmentLineSegment(const V3& s0,const V3& e0,const V3& s1,const V3& e1);
bool NearestPointLineSegmentLineSegment(V3* p,const V3& s0,const V3& e0,const V3& s1,const V3& e1);
bool NearestPointLineSegmentLineSegment(V2* p,const V2& s0,const V2& e0,const V2& s1,const V2& e1);

struct XCapsule {
	V3 m_vp0;
	V3 m_vp1;
	float m_fRadius;
};
bool RayCapsuleIntersect(const V3& vStart, const V3& vDirection, const XCapsule* pCapsule);

uint32_t mRandomU32();

//**************************************************************************************
//* If the light's w-component is 0, the ray from the origin to the light represents a
//* directional light. If it is 1, the light is a point light.
//* P=normalize(Plane);
//* L=Light;
//* d=-dot(P, L)
//*
//*	P.a * L.x+d  P.a * L.y      P.a * L.z      P.a * L.w
//*	P.b * L.x      P.b * L.y+d  P.b * L.z      P.b * L.w
//*	P.c * L.x      P.c * L.y      P.c * L.z+d  P.c * L.w
//*	P.d * L.x      P.d * L.y      P.d * L.z      P.d * L.w+d
//*
//**************************************************************************************
inline M44 ShadowMatrix(const V4 &L,V4 &P)
{
	M44 m;
	float OO_D=1.0f/sqrtf(P.x * P.x+P.y * P.y+P.z * P.z);
	P.x=P.x * OO_D;
	P.y=P.y * OO_D;
	P.z=P.z * OO_D;
	P.w=P.w * OO_D;

	float d=-VDot(P,L);
	m.m[0][0]=P.x * L.x+d;	m.m[0][1]=(P.x * L.y);			m.m[0][2]=(P.x * L.z);			m.m[0][3]=P.x * L.w;
	m.m[1][0]=P.y * L.x;		m.m[1][1]=(P.y * L.y+d);		m.m[1][2]=(P.y * L.z);			m.m[1][3]=P.y * L.w;
	m.m[2][0]=P.z * L.x;		m.m[2][1]=(P.z * L.y);			m.m[2][2]=(P.z * L.z+d);		m.m[2][3]=(P.z * L.w);
	m.m[3][0]=P.w * L.x;		m.m[3][1]=(P.w * L.y);			m.m[3][2]=(P.w * L.z);			m.m[3][3]=P.w * L.w+d;
	return m;
}

class M23 {
	public:
		M23() {
			Identity();
		}
		void Identity() {
			m[0][0]=1.0f;
			m[0][1]=0.0f;
			m[1][0]=0.0f;
			m[1][1]=1.0f;
			m[2][0]=0.0f;
			m[2][1]=0.0f;
		}
		static M23 Mul(const M23 &m1,const M23 &m2) {
			M23 r;
			r.m[0][0]=m1.m[0][0]*m2.m[0][0]+m1.m[0][1]*m2.m[1][0];
			r.m[0][1]=m1.m[0][0]*m2.m[0][1]+m1.m[0][1]*m2.m[1][1];
			r.m[1][0]=m1.m[1][0]*m2.m[0][0]+m1.m[1][1]*m2.m[1][0];
			r.m[1][1]=m1.m[1][0]*m2.m[0][1]+m1.m[1][1]*m2.m[1][1];
			r.m[2][0]=m1.m[2][0]*m2.m[0][0]+m1.m[2][1]*m2.m[1][0]+m2.m[2][0];
			r.m[2][1]=m1.m[2][0]*m2.m[0][1]+m1.m[2][1]*m2.m[1][1]+m2.m[2][1];
			return r;
		}
		V2 TransformCoord(float x,float y)const {
			return V2(x*m[0][0]+y*m[1][0]+m[2][0],x*m[0][1]+y*m[1][1]+m[2][1]);
		}
		V2 TransformCoord(const V2& v)const {
			return V2(v.x*m[0][0]+v.y*m[1][0]+m[2][0],v.x*m[0][1]+v.y*m[1][1]+m[2][1]);
		}
		V3 TransformCoordV3(float x,float y)const {
			return V3(x*m[0][0]+y*m[1][0]+m[2][0],x*m[0][1]+y*m[1][1]+m[2][1],0.0f);
		}
		void PostTranslate(float x,float y) {
			M23 trans;
			trans.m[2][0]=x;
			trans.m[2][1]=y;
			*this=Mul(*this,trans);
		}
		void PreTranslate(float x,float y) {
			M23 trans;
			trans.m[2][0]=x;
			trans.m[2][1]=y;
			*this=Mul(trans,*this);
		}
		void PostScale(float x,float y) {
			M23 scl;
			scl.m[0][0]=x;
			scl.m[1][1]=y;
			*this=Mul(*this,scl);
		}
		void PreScale(float x,float y) {
			M23 scl;
			scl.m[0][0]=x;
			scl.m[1][1]=y;
			*this=Mul(scl,*this);
		}
		float m[3][2];
};

inline void PreRotate(M23 &mat, float angle) {
	float	cosA, sinA, tmp;
	cosA	=(float)cosf(angle);
	sinA	=(float)sinf(angle);
	tmp	=mat.m[0][0]*sinA;
	mat.m[0][0]	*=cosA;
	mat.m[0][0]	-=mat.m[1][0]*sinA;
	mat.m[1][0]	*=cosA;
	mat.m[1][0]	+=tmp;
	tmp	=mat.m[0][1]*sinA;
	mat.m[0][1]	*=cosA;
	mat.m[0][1]	-=mat.m[1][1]*sinA;
	mat.m[1][1]	*=cosA;
	mat.m[1][1]	+=tmp;
}

inline void PreXRotate(M44 &mat, float angle)
{
	float	cosA, sinA, tmp;

	cosA	=(float)cosf(angle);
	sinA	=(float)sinf(angle);

	tmp	=mat.m[1][0]*sinA;
	mat.m[1][0]	*=cosA;
	mat.m[1][0]	-=mat.m[2][0]*sinA;
	mat.m[2][0]	*=cosA;
	mat.m[2][0]	+=tmp;

	tmp	=mat.m[1][1]*sinA;
	mat.m[1][1]	*=cosA;
	mat.m[1][1]	-=mat.m[2][1]*sinA;
	mat.m[2][1]	*=cosA;
	mat.m[2][1]	+=tmp;

	tmp	=mat.m[1][2]*sinA;
	mat.m[1][2]	*=cosA;
	mat.m[1][2]	-=mat.m[2][2]*sinA;
	mat.m[2][2]	*=cosA;
	mat.m[2][2]	+=tmp;
}

inline void PreYRotate(M44 &mat, float angle)
{
	float	cosA, sinA, tmp;

	cosA	=(float)cosf(angle);
	sinA	=(float)sinf(angle);

	tmp	=mat.m[0][0]*sinA;
	mat.m[0][0]	*=cosA;
	mat.m[0][0]	+=mat.m[2][0]*sinA;
	mat.m[2][0]	*=cosA;
	mat.m[2][0]	-=tmp;

	tmp	=mat.m[0][1]*sinA;
	mat.m[0][1]	*=cosA;
	mat.m[0][1]	+=mat.m[2][1]*sinA;
	mat.m[2][1]	*=cosA;
	mat.m[2][1]	-=tmp;

	tmp	=mat.m[0][2]*sinA;
	mat.m[0][2]	*=cosA;
	mat.m[0][2]	+=mat.m[2][2]*sinA;
	mat.m[2][2]	*=cosA;
	mat.m[2][2]	-=tmp;
}

inline void PreZRotate(M44 &mat, float angle)
{
	float	cosA, sinA, tmp;

	cosA	=(float)cosf(angle);
	sinA	=(float)sinf(angle);

	tmp	=mat.m[0][0]*sinA;
	mat.m[0][0]	*=cosA;
	mat.m[0][0]	-=mat.m[1][0]*sinA;
	mat.m[1][0]	*=cosA;
	mat.m[1][0]	+=tmp;

	tmp	=mat.m[0][1]*sinA;
	mat.m[0][1]	*=cosA;
	mat.m[0][1]	-=mat.m[1][1]*sinA;
	mat.m[1][1]	*=cosA;
	mat.m[1][1]	+=tmp;

	tmp	=mat.m[0][2]*sinA;
	mat.m[0][2]	*=cosA;
	mat.m[0][2]	-=mat.m[1][2]*sinA;
	mat.m[1][2]	*=cosA;
	mat.m[1][2]	+=tmp;
}


inline float GetAngle(const V3& v0 ,const V3& v1) {
    const float l0 = VLength(v0);
    const float l1 = VLength(v1);
    const V3 v01 = v0 * l1;
    const V3 v10 = v1 * l0;
    return 2.f * atan2f(VLength(v01 - v10), VLength(v01 + v10));
}


inline float GetAngle(const V3& v0, const V3& v1, const V3& ref) {
    const float angle = GetAngle(v0, v1);
    const V3 cross = VCross(v0, v1);
    const bool posHalfSpace = VDot(ref, cross) >= 0.f;
    return posHalfSpace ? angle : -angle;
}



/*
class HermiteCurve {
	public:
		HermiteCurve();
		~HermiteCurve();
		void Reset();
		int CountPoints()const{if(!m_pPoint)return 0;return m_lNumberPoints;}
		void SetPoints(const V3* pvPositions,int lNumberPositions);
		V3 GetPositionAtLength(float fLength)const;
		float GetTotalLength()const{if(!m_pPoint)return 0;return m_fTotalLength;}
	protected:
		struct XPoints {
			V3 m_vPosition;
			V3 m_vVelocity;
			float m_fDistanceToNextPoint;
		};
		float m_fTotalLength;
		int m_lNumberPoints;
		XPoints* m_pPoint;
};

bool SphereCollision(float* pfMoveDistance,const V2& vSphereACenter,float fSphereARadius,const V2& vSphereAMovement,const V2& vSphereBCenter,float fSphereBRadius);

inline V3 VHermite(const V3& p0,const V3& v0,const V3& p1,const V3& v1, float t0, float t1, float t) {
	float dt=t1-t0;
	if (dt<=0) {t0=0;t1=1;dt=1.0f;}
	t=(t-t0)/dt;
	float tt=t*t;
	float ttt=tt*t;
	float h00=2*ttt-3*tt+1;
	float h10=ttt-2*tt+t;
	float h01=-2*ttt+3*tt;
	float h11=ttt-tt;
	return h00*p0+h01*p1+(h10*v0+h11*v1)*dt;
}
inline V3 VHermiteV(const V3& p0, const V3& v0, const V3& p1, const V3& v1, float t0, float t1, float t) {
	float dt=t1-t0;
	if (dt<=0) {t0=0;t1=1;dt=1.0f;}
	float dtInv=1.0f/dt;
	t=(t-t0)*dtInv;
	float tt=t*t;
	float h00=6*tt-6*t;
	float h10=3*tt-4*t+1;
	float h01=-6*tt+6*t;
	float h11=3*tt-2*t;
	return (h00*p0+h01*p1)*dtInv+h10*v0+h11*v1;
}
inline V3 VHermiteA(const V3& p0, const V3& v0, const V3& p1, const V3& v1, float t0, float t1, float t) {
	// A0=((-6p0 +6p1)*dtInv -4v0 -2v1)*dtInv
	// AT=((+6p0 -6p1)*dtInv +2v0 +4v1)*dtInv
	float dt=t1-t0;
	if (dt<=0) {t0=0;t1=1;dt=1.0f;}
	float dtInv=1.0f/dt;
	t=(t-t0)*dtInv;
	float h00=12*t-6;
	float h10=6*t-4;
	float h01=-12*t+6;
	float h11=6*t-2;
	return ((h00*p0+h01*p1)*dtInv+(h10*v0+h11*v1))*dtInv;
}



inline float VHermite(const float& p0,const float& v0,const float& p1,const float& v1, float t0, float t1, float t) {
	float dt=t1-t0;
	if (dt<=0) {t0=0;t1=1;dt=1.0f;}
	t=(t-t0)/dt;
	float tt=t*t;
	float ttt=tt*t;
	float h00=2*ttt-3*tt+1;
	float h10=ttt-2*tt+t;
	float h01=-2*ttt+3*tt;
	float h11=ttt-tt;
	return h00*p0+h01*p1+(h10*v0+h11*v1)*dt;
}
inline float VHermiteV(const float& p0, const float& v0, const float& p1, const float& v1, float t0, float t1, float t) {
	float dt=t1-t0;
	if (dt<=0) {t0=0;t1=1;dt=1.0f;}
	float dtInv=1.0f/dt;
	t=(t-t0)*dtInv;
	float tt=t*t;
	float h00=6*tt-6*t;
	float h10=3*tt-4*t+1;
	float h01=-6*tt+6*t;
	float h11=3*tt-2*t;
	return (h00*p0+h01*p1)*dtInv+h10*v0+h11*v1;
}
inline float VHermiteA(const float& p0, const float& v0, const float& p1, const float& v1, float t0, float t1, float t) {
	// A0=((-6p0 +6p1)*dtInv -4v0 -2v1)*dtInv
	// AT=((+6p0 -6p1)*dtInv +2v0 +4v1)*dtInv
	float dt=t1-t0;
	if (dt<=0) {t0=0;t1=1;dt=1.0f;}
	float dtInv=1.0f/dt;
	t=(t-t0)*dtInv;
	float h00=12*t-6;
	float h10=6*t-4;
	float h01=-12*t+6;
	float h11=6*t-2;
	return ((h00*p0+h01*p1)*dtInv+(h10*v0+h11*v1))*dtInv;
}

inline unsigned char ReadBilinearPixel(float fx,float fy,const unsigned char* map,int width,int height) {
	if(fx<0)return 0;
	if(fy<0)return 0;
	int xCuri=(int)(fx*65536.0f);
	int yCuri=(int)(fy*65536.0f);
	if(xCuri>=(width-1)<<16)
		return 0;
	if(yCuri>=(height-1)<<16)
		return 0;
	int x=xCuri>>16;
	int xFragi=(xCuri&0xffff)>>4;
	int y=yCuri>>16;
	int yFragi=(yCuri&0xffff)>>8;
	int omxFragi=0x1000-xFragi;
	int omyFragi=0x100-yFragi;
	const unsigned char* p0=map+y*width+x;
	const unsigned char* p1=p0+width;
	int s0i=omxFragi*omyFragi;
	int s1i=xFragi*omyFragi;
	int s2i=yFragi*omxFragi;
	int s3i=xFragi*yFragi;
	return (p0[0]*s0i+p0[1]*s1i+p1[0]*s2i+p1[1]*s3i)>>20;
}

uint64_t GetSampleTime(const uint64_t frameNum, const unsigned int fps, const uint64_t startTime = 0);
uint64_t GetFrameDuration(const uint64_t frameNum, const unsigned int fps);
uint64_t GetFrameNum(const uint64_t time, const unsigned int fps, const uint64_t startTime = 0);
uint64_t GetFrameNumNearest(const uint64_t time, const unsigned int fps, const uint64_t startTime = 0);

float CalcArea(const V2* points, const int w, const int h);
*/

#endif//__cplusplus



/** \} */

#endif//_MATHTYPES_H_
