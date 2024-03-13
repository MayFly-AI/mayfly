
#include "shared/types.h"
#include "shared/math.h"

#include <stdlib.h>
#include <time.h>
#include <math.h>

static unsigned long rand_x=123456789, rand_y=362436069, rand_z=521288629;

void SeedRandom(uint32_t seed) {
	srand((uint32_t)seed);
}

void InitRandom() {
	time_t rawtime;
	time(&rawtime);
	srand((uint32_t)rawtime);
}

unsigned long xorshf96(void) {          //period 2^96-1
	rand_x ^= rand_x << 16;
	rand_x ^= rand_x >> 5;
	rand_x ^= rand_x << 1;
	unsigned long t = rand_x;
	rand_x = rand_y;
	rand_y = rand_z;
	rand_z = t ^ rand_x ^ rand_y;
	return rand_z;
}

float RandomUnitFloat() {
	float r=(float)(xorshf96()&0xffff);
	return( r/65536.0f ) ;
}
uint32_t mRandomU32() {
	return (uint32_t)xorshf96();
}

typedef union {
	float v;
	struct {
		uint32_t m:23;
		uint32_t e:8;
		uint32_t s:1;
	} bits;
} float32_s;

void float32to16(float x,float16_s * f16) {
	float32_s f32={x}; // c99
	f16->bits.s=f32.bits.s;
	f16->bits.e=MAX(-15,MIN(16,(int)(f32.bits.e-127))) +15;
	f16->bits.m=f32.bits.m >> 13;
}
float float16to32(float16_s f16) {
	float32_s f32;
	f32.bits.s=f16.bits.s;
	f32.bits.e=(f16.bits.e-15)+127; // safe in this direction
	f32.bits.m=((uint32_t)f16.bits.m) << 13;
	return f32.v;
}
void f16Tof32(float* pf32,const uint16_t* pf16,int elements) {
	for(int i=0;i!=elements;i++) {
		float16_s* f16=(float16_s*)&pf16[i];
		pf32[i]=float16to32(*f16);
	}
}
void f32Tof16(uint16_t* pf16,const float* pf32,int elements) {
	for(int i=0;i!=elements;i++) {
		float16_s* f16=(float16_s*)&pf16[i];
		float32to16(pf32[i],f16);
	}
}

void VSnapToGrid(V3& v, float fGridSize) {
	v.x = (float)((int)(v.x/fGridSize + sign(v.x)*0.5f));
	v.x *= fGridSize;
	v.y = (float)((int)(v.y/fGridSize + sign(v.y)*0.5f));
	v.y *= fGridSize;
	v.z = (float)((int)(v.z/fGridSize + sign(v.z)*0.5f));
	v.z *= fGridSize;
}

bool IntersectLineAABB(float* pT,float minX,float minY,float minZ,float maxX,float maxY,float maxZ,float linePosX,float linePosY,float linePosZ,float lineVecX,float lineVecY,float lineVecZ) {
	float fTx=-1,fTy=-1,fTz=-1,fT;

	//XASSERTROOT(minX<=maxX && minY<=maxY && minZ<=maxZ);

	if(minX<=linePosX && linePosX<=maxX && minY<=linePosY && linePosY<=maxY && minZ<=linePosZ && linePosZ<=maxZ ) {
		*pT=0; return true;
	}

	if(lineVecX !=0)
		fTx=(lineVecX<0) ? (maxX-linePosX)/lineVecX : (minX-linePosX)/lineVecX;
	if(lineVecY !=0)
		fTy=(lineVecY<0) ? (maxY-linePosY)/lineVecY : (minY-linePosY)/lineVecY;
	if(lineVecZ !=0)
		fTz=(lineVecZ<0) ? (maxZ-linePosZ)/lineVecZ : (minZ-linePosZ)/lineVecZ;

	if(fTx>=fTy && fTx>=fTz) {
		fT=fTx;
		float resY=fTx*lineVecY+linePosY;
		float resZ=fTx*lineVecZ+linePosZ;
		if( minY>resY || resY>maxY || minZ>resZ || resZ>maxZ )
			return false;
	}else if(fTy >=fTz) {
		fT=fTy;
		float resZ=fTy*lineVecZ+linePosZ;
		float resX=fTy*lineVecX+linePosX;
		if( minZ>resZ || resZ>maxZ || minX>resX || resX>maxX )
			return false;
	}else{
		fT=fTz;
		float resX=fTz*lineVecX+linePosX;
		float resY=fTz*lineVecY+linePosY;
		if( minX>resX || resX>maxX || minY>resY || resY>maxY )
			return false;
	}
	*pT=fT;
	return (fT>=0 && fT<=1);
}

bool AABBAABBIntersectXY(const V3& vAMin,const V3& vAMax, const V3& vBMin, const V3& vBMax, float fExtraSize/*=0.0f*/)
{
	if(vAMax.x <= vBMin.x-fExtraSize || vAMin.x >= vBMax.x+fExtraSize) return false;
	if(vAMax.y <= vBMin.y-fExtraSize || vAMin.y >= vBMax.y+fExtraSize) return false;
	return true;
}

bool Segmentsintersect(const V3& p1, const V3& p2, const V3& q1, const V3& q2)
{
	V3 vPDir = p2-p1;
	V3 vP1Q1 = q1-p1;
	V3 vP1Q2 = q2-p1;
	V3 vCross1 = VCross(vPDir,vP1Q1);
	V3 vCross2 = VCross(vPDir,vP1Q2);
	if(VDot(vCross1,vCross2)>0.0f) return false;
	V3 vQDir = q2-q1;
	V3 vQ1P1 = p1-q1;
	V3 vQ1P2 = p2-q1;
	vCross1 = VCross(vQDir,vQ1P1);
	vCross2 = VCross(vQDir,vQ1P2);
	if(VDot(vCross1,vCross2)>0.0f) return false;
	return true;
}


//calculate point on line segment s0-e0 nearest to line segment s1-e1
bool NearestPointLineSegmentLineSegment(V3* p,const V3& s0,const V3& e0,const V3& s1,const V3& e1) {
	float l0=VLength(e0-s0);
	if(l0<FEPSILON) {
		*p=s0;
		return true;
	}
	float l1=VLength(e1-s1);
	if(l1<FEPSILON) {
		*p=NearestPointLineSegment(s0,s0,e0);
		return true;
	}
	V3 n0=(e0-s0)*1.0f/l0;
	//V3 n1=(e1-s1)*1.0f/l1;
	V3 i0;
	if(ClosestPointOnRayToRay(&i0,s0,e0-s0,s1,e1-s1)) {
		float t=VDot(i0-s0,n0);
		if(t<0) {
			*p=s0;
		}else
		if(t>l0) {
			*p=e0;
		}else{
			*p=i0;
		}
		return true;
	}
	return false;
}
bool NearestPointLineSegmentLineSegment(V2* p,const V2& s0,const V2& e0,const V2& s1,const V2& e1) {
	V3 i0;
	if(NearestPointLineSegmentLineSegment(&i0,V3(s0,0),V3(e0,0),V3(s1,0),V3(e1,0))) {
		*p=i0.VXY();
		return true;
	}
	return false;
}

//calculate shortest distance from line segment s0-e0 nearest to line segment s1-e1
float DistanceLineSegmentLineSegment(const V3& s0,const V3& e0,const V3& s1,const V3& e1) {
	V3 i0;
	if(NearestPointLineSegmentLineSegment(&i0,s0,e0,s1,e1)) {
		return DistancePointLineSegment(i0,s1,e1);
	}
	return DistancePointLineSegment(s0,s1,e1);	//parallel lines, any point will do
}
float DistanceLineSegmentLineSegment(const V2& s0,const V2& e0,const V2& s1,const V2& e1) {
	V2 i0;
	if(NearestPointLineSegmentLineSegment(&i0,s0,e0,s1,e1)) {
		return DistancePointLineSegment(i0,s1,e1);
	}
	return DistancePointLineSegment(s0,s1,e1);	//parallel lines, any point will do
}

V3 NearestPointRay(const V3& point,const V3& start,const V3& direction) {
	float d=VDot(point-start,direction);
	return start+direction*d;
}

V3 NearestPointLineSegment(const V3& vPoint,const V3& vLineStart,const V3& vLineEnd) {
	V3 vLine=vLineEnd-vLineStart;
	float fLength=VLength(vLine);
	vLine=VMul(vLine,1.0f/fLength);
	float fDot=VDot(vPoint-vLineStart,vLine);
	if(fDot<0) {
		return vLineStart;
	}
	if(fDot>fLength) {
		return vLineEnd;
	}
	return vLineStart+VMul(vLine,fDot);
}

V2 NearestPointLineSegment(const V2& vPoint,const V2& vLineStart,const V2& vLineEnd) {
	V2 vLine=vLineEnd-vLineStart;
	float fLength=VLength(vLine);
	vLine=VMul(vLine,1.0f/fLength);
	float fDot=VDot(vPoint-vLineStart,vLine);
	if(fDot<0) {
		return vLineStart;
	}
	if(fDot>fLength) {
		return vLineEnd;
	}
	return vLineStart+VMul(vLine,fDot);
}

float DistancePointLineSegment(const V3& vPoint,const V3& vLineStart,const V3& vLineEnd) {
	V3 vNearestPoint=NearestPointLineSegment(vPoint,vLineStart,vLineEnd);
	return VLength(vPoint-vNearestPoint);
}

float DistancePointLineSegment(const V2& p,const V2& s,const V2& e) {
	V2 np=NearestPointLineSegment(p,s,e);
	return VLength(p-np);
}

float DistancePointRay(const V3& vPoint,const V3& vLineStart,const V3& vLineDirection) {
	V3 vLineNormal=VNormalize(vLineDirection);
	V3 v=VCross(vLineNormal,vPoint-vLineStart);
	return VLength(v);
}

bool IntersectLineAndPlane(V3& intersection, const V3& lineStart, const V3& lineEnd, const V3& vVertex, const V3& vNormal) {
	V3 dir = lineEnd - lineStart;             // ray direction vector
	V3 w0 = lineStart - vVertex;
	float a = -VDot(vNormal,w0);
	float b = VDot(vNormal,dir);
	if (fabs(b) < 1e-20/*EPSILON*/) {
		// ray is parallel to triangle plane
		if (a == 0) {
			// ray lies in triangle plane
			return false;
		} else {
			// ray disjoint from plane
			return false;
		}
	}

	// get intersect point of ray with triangle plane
	float r = a / b;
	if (r < 0.0)                   // ray goes away from triangle
		return false;                  // => no intersect
	if( r > 1.0)
		return false;
	// for a segment, also test if (r > 1.0) => no intersect

	// intersect point of ray and plane
	intersection = lineStart + (dir * V3(r,r,r));
	return true;
}

bool IntersectRayAndPlane(V3* pintersection, const V3& vLine, const V3& dir, const V3& vVertex, const V3& vNormal) {
	//V3 dir = lineEnd - lineStart;             // ray direction vector
	V3 w0 = vLine - vVertex;
	float a = -VDot(vNormal,w0);
	float b = VDot(vNormal,dir);
	if (fabs(b) < 1e-20/*EPSILON*/) {
		// ray is parallel to triangle plane
		if (a == 0) {
			// ray lies in triangle plane
			return false;
		} else {
			// ray disjoint from plane
			return false;
		}
	}

	// get intersect point of ray with triangle plane
	float r = a / b;
	//if (r < 0.0)                   // ray goes away from triangle
	//	return false;                  // => no intersect
	// for a segment, also test if (r > 1.0) => no intersect

	// intersect point of ray and plane
	*pintersection = vLine + (dir * V3(r,r,r));
	return true;
}

// Compute barycentric coordinates (u,v,w) for
// point p with respect to triangle (a,b,c)
void Barycentric(float* u,float* v,float* w,const V3& p,const V3& a,const V3& b,const V3& c) {
	V3 v0=b-a,v1=c-a,v2=p-a;
	float d00=VDot(v0,v0);
	float d01=VDot(v0,v1);
	float d11=VDot(v1,v1);
	float d20=VDot(v2,v0);
	float d21=VDot(v2,v1);
	float denom=d00*d11-d01*d01;
	*v=(d11*d20-d01*d21)/denom;
	*w=(d00*d21-d01*d20)/denom;
	*u=1.0f-*v-*w;
}

bool RayTriangleIntersect(float* t,const V3& P,const V3& w,const V3& v0,const V3& v1,const V3& v2,float b[3]) {
// Edge vectors
	const V3& e_1=v1-v0;
	const V3& e_2=v2-v0;
// Face normal
	const V3& n=VCross(e_1,e_2);
	const V3& q=VCross(w,e_2);
	const float a=VDot(e_1,q);
// Backfacing or nearly parallel?
	if((VDot(n,w)>=0) || (fabsf(a)<=FEPSILON)) return false;
	const V3& s=(P-v0)/a;
	const V3& r=VCross(s,e_1);
	b[0]=VDot(s,q);
	b[1]=VDot(r,w);
	b[2]=1.0f-b[0]-b[1];
// Intersected outside triangle?
	if((b[0]<0.0f) || (b[1]<0.0f) || (b[2]<0.0f)) return false;
	*t=VDot(e_2,r);
	return (*t >= 0.0f);
}

bool IntersectDistanceRayAndPlane(float* distance,const V3& rayPosition,const V3& rayDirection,const V3& planePosition,const V3& planeNormal) {
	V3 w0=rayPosition-planePosition;
	float a=-VDot(planeNormal,w0);
	float b=VDot(planeNormal,rayDirection);
	if(fabs(b)<FEPSILON) {
		return false;
	}
	*distance=a/b;
	return true;
}
bool ProjectDistanceOnRayToRay(float* p,const V3& rayPositionA,const V3& rayDirectionA,const V3& rayPositionB,const V3& rayDirectionB) {
	float distance;
	V3 n=VCross(rayDirectionB,rayDirectionA);
	V3 ortho=VCross(rayDirectionB,n);
	ortho=VNormalize(ortho);
	if(IntersectDistanceRayAndPlane(&distance,rayPositionA,rayDirectionA,rayPositionB,ortho)) {
		*p=distance;
		return true;
	}
	return false;
}

bool PointInsidePoly(const V2& pt, const V2* verts, size_t nverts) {
	if (nverts < 3) return false;
	bool c = false;
	for(size_t i = 0, j = nverts-1; i < nverts; j = i++) {
		const V2& vi = verts[i];
		const V2& vj = verts[j];
		if(((vi.y >= pt.y) != (vj.y > pt.y)) &&
				(pt.x <= (vj.x-vi.x) * (pt.y-vi.y) / (vj.y-vi.y) + vi.x) )
			c = !c;
	}
	return c;
}

bool ClosestPointOnRayToRay(V3* p,const V3& rayPositionA,const V3& rayDirectionA,const V3& rayPositionB,const V3& rayDirectionB) {
	float projectDistance;
	if(!ProjectDistanceOnRayToRay(&projectDistance,rayPositionA,rayDirectionA,rayPositionB,rayDirectionB))
		return false;
	*p=rayPositionA+rayDirectionA*projectDistance;
	return true;
	/*
	float distance;
	V3 n=VCross(rayDirectionB,rayDirectionA);
	V3 ortho=VCross(rayDirectionB,n);
	ortho=VNormalize(ortho);
	if(IntersectDistanceRayAndPlane(&distance,rayPositionA,rayDirectionA,rayPositionB,ortho)) {
		*p=rayPositionA+rayDirectionA*distance;
		return true;
	}
	return false;
	*/
}

float DistanceRays(const V3& rayPositionA,const V3& rayDirectionA,const V3& rayPositionB,const V3& rayDirectionB) {
	V3 u = rayDirectionA;
	V3 v = rayDirectionB;
	V3 w = rayPositionA-rayPositionB;
	float a = VDot(u,u);         // always >= 0
	float b = VDot(u,v);
	float c = VDot(v,v);         // always >= 0
	float d = VDot(u,w);
	float e = VDot(v,w);
	float D = a*c - b*b;        // always >= 0
	float sc, tc;

	// compute the line parameters of the two closest points
	if(D<FEPSILON) {          // the lines are almost parallel
		sc = 0.0;
		tc = (b>c ? d/b : e/c);    // use the largest denominator
	}else{
		sc = (b*e - c*d) / D;
		tc = (a*e - b*d) / D;
	}

	// get the difference of the two closest points
	V3   dP = w + (sc * u) - (tc * v);  // =  L1(sc) - L2(tc)

	return VLength(dP);   // return the closest distance
}

inline float Overlap(float minA,float maxA,float minB,float maxB) {
	if(maxA<minB)
		return 0;
	if(minA>maxB)
		return 0;
	if(minA>=minB && maxA<=maxB) {		//A included 100% in B
		return maxA-minA;
	}
	if(minB>=minA && maxB<=maxA) {		//B included 100% in A
		return maxB-minB;
	}
	if(minA<minB) {
		return maxA-minB;
	}
	return maxB-minA;
}

float AABBROI(const V3& minA,const V3& maxA,const V3& minB,const V3& maxB) {
	V3 o;
	o.x=Overlap(minA.x,maxA.x,minB.x,maxB.x);
	o.y=Overlap(minA.y,maxA.y,minB.y,maxB.y);
	o.z=Overlap(minA.z,maxA.z,minB.z,maxB.z);
	float arealA=(maxA.x-minA.x)*(maxA.y-minA.y)*(maxA.z-minA.z);
	float arealB=(maxB.x-minB.x)*(maxB.y-minB.y)*(maxB.z-minB.z);
	float arealO=o.x*o.y*o.z;
	if(arealO==0)
		return 0;
	return arealO/(arealA+arealB-arealO);
}

M44 M44Ortho(float fLeft,float fRight,float fBottom,float fTop,float fNear,float fFar) {
   M44 mResult;

   float xx=2.f/(fRight-fLeft);
   float yy=2.f/(fTop-fBottom);
   float zz=1.f/(fFar-fNear);
   float tx=-(fRight+fLeft)/(fRight-fLeft);
   float ty=-(fTop+fBottom)/(fTop-fBottom);
   float tz=-fNear/(fFar-fNear);

   mResult.m[0][0]=xx;
   mResult.m[0][1]=0;
   mResult.m[0][2]=0;
   mResult.m[0][3]=0;

   mResult.m[1][0]=0;
   mResult.m[1][1]=yy;
   mResult.m[1][2]=0;
   mResult.m[1][3]=0;

   mResult.m[2][0]=0;
   mResult.m[2][1]=0;
   mResult.m[2][2]=zz;
   mResult.m[2][3]=0;

   mResult.m[3][0]=tx;
   mResult.m[3][1]=ty;
   mResult.m[3][2]=tz;
   mResult.m[3][3]=1;

   return mResult;
}
M44 M44PerspectiveInfinite(float fLeft,float fRight,float fTop,float fBottom,float fNear) {
   M44 mResult;
   mResult.m[0][0]=2.0f*1.0f/(fRight-fLeft);
   mResult.m[0][1]=0;
   mResult.m[0][2]=0;
   mResult.m[0][3]=0;

   mResult.m[1][0]=0;
   mResult.m[1][1]=2.0f*1.0f/(fTop-fBottom);
   mResult.m[1][2]=0;
   mResult.m[1][3]=0;

   mResult.m[2][0]=(fRight+fLeft)/(fRight-fLeft);
   mResult.m[2][1]=(fTop+fBottom)/(fTop-fBottom);
   mResult.m[2][2]=-1; // Infinite far
   mResult.m[2][3]=-1;

   mResult.m[3][0]=0;
   mResult.m[3][1]=0;
   mResult.m[3][2]=-1.0f*fNear;
   mResult.m[3][3]=0;
   return mResult;
}

M44 M44PerspectiveRH(float fWidth,float fHeight,float fNear,float fFar) {
	M44 mResult;

	float Q=fFar/(fFar-fNear);

	mResult.m[0][0]=2.0f*fNear/fWidth;
	mResult.m[0][1]=0;
	mResult.m[0][2]=0;
	mResult.m[0][3]=0;

	mResult.m[1][0]=0;
	mResult.m[1][1]=2.0f*fNear/fHeight;
	mResult.m[1][2]=0;
	mResult.m[1][3]=0;

	mResult.m[2][0]=0;
	mResult.m[2][1]=0;
	mResult.m[2][2]=-Q;
	mResult.m[2][3]=-1;

	mResult.m[3][0]=0;
	mResult.m[3][1]=0;
	mResult.m[3][2]=-Q*fNear;
	mResult.m[3][3]=0;

	return mResult;
}




M44 PerspectiveFovLH( float fovy,float aspect,float zn,float zf )
{
	float yScale = 1.0f/tanf(fovy*0.5f);
	float xScale = yScale / aspect;

	M44 m;
	m.m[0][0]=xScale;		m.m[0][1]=0;			m.m[0][2]=0;					m.m[0][3]=0;
	m.m[1][0]=0;			m.m[1][1]=yScale;		m.m[1][2]=0;					m.m[1][3]=0;
	m.m[2][0]=0;			m.m[2][1]=0;			m.m[2][2]=zf/(zf-zn);			m.m[2][3]=1;
	m.m[3][0]=0;			m.m[3][1]=0;			m.m[3][2]=-zn*zf/(zf-zn);		m.m[3][3]=0;
	return m;
}

M44 M44PerspectiveFovRH(float fFovY,float fAspectWByH,float fNear,float fFar) {
   float h=2.0f*fNear*tanf(fFovY/2.0f);
   float w=h*fAspectWByH;
   return M44PerspectiveRH(w,h,fNear,fFar);
}

//*****************************************************************************************************
//*
//*****************************************************************************************************
M44 LookAtPlane(float fov,float Height,V3 plane_normal,V3 plane_pos,V3 WorldUp)
{
	V3 zaxis = plane_normal;
	V3 xaxis = VNormalize(VCross(WorldUp,zaxis));
	V3 yaxis = VCross( zaxis,xaxis);

	float camdist=(Height*0.5f)/tanf(fov/2.0f);
	V3 eye=(plane_normal*camdist)+plane_pos;

	M44 m;
	m.m[0][0]=xaxis.x;			m.m[0][1]=-yaxis.x;			m.m[0][2]=zaxis.x;			m.m[0][3]=0;
	m.m[1][0]=xaxis.y;			m.m[1][1]=-yaxis.y;			m.m[1][2]=zaxis.y;			m.m[1][3]=0;
	m.m[2][0]=xaxis.z;			m.m[2][1]=-yaxis.z;			m.m[2][2]=zaxis.z;			m.m[2][3]=0;
	m.m[3][0]=-VDot(xaxis,eye);	m.m[3][1]=-VDot(yaxis,eye);	m.m[3][2]=-VDot(zaxis,eye);	m.m[3][3]=1;
	return m;
}

V4 Slerp(V4 p, V4 q2, float t) {
	float omega,cosom,sinfom,sclp,sclq;
	int i;
	V4 q = q2;
	V4 res;

	cosom = p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3];

	if (cosom < 0)
	{
		cosom = -cosom;
		q = -q2;
	}

	if( (1.0f + cosom) > 0 )
	{
		if( (1.0f - cosom) > 0 )
		{
			omega = acosf(cosom);
			sinfom = sinf(omega);
			sclp = sinf( (1.0f - t)*omega )/sinfom;
			sclq = sinf( t*omega )/sinfom;
		}
		else
		{
			sclp = 1.0f - t;
			sclq = t;
		}

		for (i=0;i!=4;i++) res[i] = sclp*p[i] + sclq*q[i];

	}
	else
	{
		res[0] = -p[1]; res[1] = p[0];
		res[2] = -p[3]; res[3] = p[2];
		sclp = sinf((1.0f - t)*PI/2);
		sclq = sinf(t*PI/2);
		for(i=0;i!=3;i++) res[i] = sclp*p[i] + sclq*res[i];
	}

	return res;
}


// A ray based on restriction #1
struct Ray
{
	V3 m_Origin;
	V3 m_Direction;
};

// A capsule based on restriction #2
struct Capsule
{
	V3 m_A;
	V3 m_B;
	float m_Radius;
};

// A simple sphere (helper struct)
struct Sphere
{
	V3 m_Center;
	float m_Radius;
};

// NOTE: This function doesn't calculate the normal because it's easily derived for a sphere (p - center).
bool IntersectRaySphere(const Ray& ray, const Sphere& sphere, float& tmin, float& tmax)
{
	V3 CO = ray.m_Origin - sphere.m_Center;

	float a = VDot(ray.m_Direction,ray.m_Direction);
	float b = 2.0f * VDot(CO,ray.m_Direction);
	float c = VDot(CO,CO) - (sphere.m_Radius * sphere.m_Radius);

	float discriminant = b * b - 4.0f * a * c;
	if(discriminant < 0.0f)
		return false;

	tmin = (-b - sqrtf(discriminant)) / (2.0f * a);
	tmax = (-b + sqrtf(discriminant)) / (2.0f * a);
	if(tmin > tmax)
	{
		float temp = tmin;
		tmin = tmax;
		tmax = temp;
	}

	return true;
}
bool IntersectRaySphere(const V3& origin, const V3& direction, const V3& sphere_center, const float& sphere_radius, float& tmin, float& tmax)
{
	Ray ray;
	ray.m_Origin =	origin;
	ray.m_Direction = direction;

	Sphere sphere;
	sphere.m_Center = sphere_center;
	sphere.m_Radius = sphere_radius;
	return IntersectRaySphere(ray, sphere, tmin, tmax);
}

bool IntersectRaySphere(const V2& origin, const V2& direction, const V2& sphere_center, const float& sphere_radius, float& tmin, float& tmax)
{
	Ray ray;
	ray.m_Origin =V3(origin.x,origin.y,0);
	ray.m_Direction = V3(direction.x,direction.y,0);

	Sphere sphere;
	sphere.m_Center = V3(sphere_center.x, sphere_center.y, 0);
	sphere.m_Radius = sphere_radius;
	return IntersectRaySphere(ray, sphere, tmin, tmax);
}

bool IntersectRayCapsule(const Ray& ray, const Capsule& capsule, V3& p1, V3& p2, V3& n1, V3& n2);

bool RayCapsuleIntersect(const V3& vStart, const V3& vDirection, const XCapsule* pCapsule) {
	V3 p1, p2, n1, n2;

	//V3 vDirection=vEnd-vStart;

	Ray ray;
	ray.m_Direction.x=vDirection.x;
	ray.m_Direction.y=vDirection.y;
	ray.m_Direction.z=vDirection.z;
	ray.m_Origin.x=vStart.x;
	ray.m_Origin.y=vStart.y;
	ray.m_Origin.z=vStart.z;
	Capsule capsule;
	capsule.m_A.x=pCapsule->m_vp0.x;
	capsule.m_A.y=pCapsule->m_vp0.y;
	capsule.m_A.z=pCapsule->m_vp0.z;
	capsule.m_B.x=pCapsule->m_vp1.x;
	capsule.m_B.y=pCapsule->m_vp1.y;
	capsule.m_B.z=pCapsule->m_vp1.z;
	capsule.m_Radius=pCapsule->m_fRadius;
	IntersectRayCapsule(ray,capsule,p1,p2,n1,n2);
	return 0;
}



bool IntersectRayCapsule(const Ray& ray, const Capsule& capsule, V3& p1, V3& p2, V3& n1, V3& n2)
{
	// Substituting equ. (1) - (6) to equ. (I) and solving for t' gives:
	//
	// t' = (t * dot(AB, d) + dot(AB, AO)) / dot(AB, AB); (7) or
	// t' = t * m + n where
	// m = dot(AB, d) / dot(AB, AB) and
	// n = dot(AB, AO) / dot(AB, AB)
	//
	V3 AB = capsule.m_B - capsule.m_A;
	V3 AO = ray.m_Origin - capsule.m_A;

	float AB_dot_d = VDot(AB,ray.m_Direction);
	float AB_dot_AO = VDot(AB,AO);
	float AB_dot_AB = VDot(AB,AB);

	float m = AB_dot_d / AB_dot_AB;
	float n = AB_dot_AO / AB_dot_AB;

	// Substituting (7) into (II) and solving for t gives:
	//
	// dot(Q, Q)*t^2 + 2*dot(Q, R)*t + (dot(R, R) - r^2) = 0
	// where
	// Q = d - AB * m
	// R = AO - AB * n
	V3 Q = ray.m_Direction - (AB * m);
	V3 R = AO - (AB * n);

	float a = VDot(Q,Q);
	float b = 2.0f * VDot(Q,R);
	float c = VDot(R,R) - (capsule.m_Radius * capsule.m_Radius);

	if(a == 0.0f)
	{
		// Special case: AB and ray direction are parallel. If there is an intersection it will be on the end spheres...
		// NOTE: Why is that?
		// Q = d - AB * m =>
		// Q = d - AB * (|AB|*|d|*cos(AB,d) / |AB|^2) => |d| == 1.0
		// Q = d - AB * (|AB|*cos(AB,d)/|AB|^2) =>
		// Q = d - AB * cos(AB, d) / |AB| =>
		// Q = d - unit(AB) * cos(AB, d)
		//
		// |Q| == 0 means Q = (0, 0, 0) or d = unit(AB) * cos(AB,d)
		// both d and unit(AB) are unit vectors, so cos(AB, d) = 1 => AB and d are parallel.
		//
		Sphere sphereA, sphereB;
		sphereA.m_Center = capsule.m_A;
		sphereA.m_Radius = capsule.m_Radius;
		sphereB.m_Center = capsule.m_B;
		sphereB.m_Radius = capsule.m_Radius;

		float atmin, atmax, btmin, btmax;
		if(	!IntersectRaySphere(ray, sphereA, atmin, atmax) ||
			!IntersectRaySphere(ray, sphereB, btmin, btmax))
		{
			// No intersection with one of the spheres means no intersection at all...
			return false;
		}

		if(atmin < btmin)
		{
			p1 = ray.m_Origin + (ray.m_Direction * atmin);
			n1 = VNormalize(p1 - capsule.m_A);
		}
		else
		{
			p1 = ray.m_Origin + (ray.m_Direction * btmin);
			n1 = VNormalize(p1 - capsule.m_B);
		}

		if(atmax > btmax)
		{
			p2 = ray.m_Origin + (ray.m_Direction * atmax);
			n2 = VNormalize(p2 - capsule.m_A);
		}
		else
		{
			p2 = ray.m_Origin + (ray.m_Direction * btmax);
			n2 = VNormalize(p2 - capsule.m_B);
		}

		return true;
	}

	float discriminant = b * b - 4.0f * a * c;
	if(discriminant < 0.0f)
	{
		// The ray doesn't hit the infinite cylinder defined by (A, B).
		// No intersection.
		return false;
	}

	float tmin = (-b - sqrtf(discriminant)) / (2.0f * a);
	float tmax = (-b + sqrtf(discriminant)) / (2.0f * a);
	if(tmin > tmax)
	{
		float temp = tmin;
		tmin = tmax;
		tmax = temp;
	}

	// Now check to see if K1 and K2 are inside the line segment defined by A,B
	float t_k1 = tmin * m + n;
	if(t_k1 < 0.0f)
	{
		// On sphere (A, r)...
		Sphere s;
		s.m_Center = capsule.m_A;
		s.m_Radius = capsule.m_Radius;

		float stmin, stmax;
		if(IntersectRaySphere(ray, s, stmin, stmax))
		{
			p1 = ray.m_Origin + (ray.m_Direction * stmin);
			n1 = VNormalize(p1 - capsule.m_A);
		}
		else
			return false;
	}
	else if(t_k1 > 1.0f)
	{
		// On sphere (B, r)...
		Sphere s;
		s.m_Center = capsule.m_B;
		s.m_Radius = capsule.m_Radius;

		float stmin, stmax;
		if(IntersectRaySphere(ray, s, stmin, stmax))
		{
			p1 = ray.m_Origin + (ray.m_Direction * stmin);
			n1 = VNormalize(p1 - capsule.m_B);
		}
		else
			return false;
	}
	else
	{
		// On the cylinder...
		p1 = ray.m_Origin + (ray.m_Direction * tmin);

		V3 k1 = capsule.m_A + AB * t_k1;
		n1 = VNormalize(p1 - k1);
	}

	float t_k2 = tmax * m + n;
	if(t_k2 < 0.0f)
	{
		// On sphere (A, r)...
		Sphere s;
		s.m_Center = capsule.m_A;
		s.m_Radius = capsule.m_Radius;

		float stmin, stmax;
		if(IntersectRaySphere(ray, s, stmin, stmax))
		{
			p2 = ray.m_Origin + (ray.m_Direction * stmax);
			n2 = VNormalize(p2 - capsule.m_A);
		}
		else
			return false;
	}
	else if(t_k2 > 1.0f)
	{
		// On sphere (B, r)...
		Sphere s;
		s.m_Center = capsule.m_B;
		s.m_Radius = capsule.m_Radius;

		float stmin, stmax;
		if(IntersectRaySphere(ray, s, stmin, stmax))
		{
			p2 = ray.m_Origin + (ray.m_Direction * stmax);
			n2 = VNormalize(p2 - capsule.m_B);
		}
		else
			return false;
	}
	else
	{
		p2 = ray.m_Origin + (ray.m_Direction * tmax);

		V3 k2 = capsule.m_A + AB * t_k2;
		n2 = VNormalize(p2 - k2);
	}

	return true;
}

#define v1pul(t1,t2,t)(((t2-t1)*t)+t1)

#define EPSILON 0.0001f

bool vangpul(V3& vres,const V3& vsource,const V3& vdest,float maxmoveangle)
{
	float lensource=VLength(vsource);
	float lendest=VLength(vdest);
	V3 v0=VNormalize(vsource);
	V3 v1=VNormalize(vdest);

	float dot=VDot(v0,v1);
	float currentangle=acosf(dot);
	if(currentangle>maxmoveangle)
	{
		V3 vortho;
		vortho=VMul(v0,dot);
		vortho=vortho-v1;
		if(VLength(vortho)<EPSILON)	//source and destination is opposite (no turn plane defined)
		{
			V3 vup(0,1.0f,0);
			vortho=VCross(vup,vdest);
		}
		vortho=VNormalize(vortho);
		vortho=VMul(vortho,-sinf(maxmoveangle));
		vres=VMul(v0,cosf(maxmoveangle));
		vres+=vortho;
		float t=0;
		if(currentangle>EPSILON)
		{
			t=maxmoveangle/currentangle;
		}
		float lencurrent=v1pul(lensource,lendest,t);
		vres=VMul(vres,lencurrent);
		return false;
	}else{
		vres=vdest;
	}
	return true;
}
/*
HermiteCurve::HermiteCurve() {
	m_fTotalLength=0;
	m_lNumberPoints=0;
	m_pPoint=0;
}
HermiteCurve::~HermiteCurve() {
	if(m_pPoint)
		delete m_pPoint;
}
void HermiteCurve::Reset()
{
	if(m_pPoint)
		delete m_pPoint;
	m_fTotalLength=0;
	m_lNumberPoints=0;
	m_pPoint=0;
}
void HermiteCurve::SetPoints(const V3* pvPositions,int lNumberPositions) {
	if(m_pPoint)
		delete m_pPoint;
	m_pPoint=0;
	m_lNumberPoints=lNumberPositions;
	m_fTotalLength=0;
	if(!m_lNumberPoints)
		return;
	m_pPoint=new XPoints[m_lNumberPoints];
	int a;
	for(a=0;a!=m_lNumberPoints;a++) {
		m_pPoint[a].m_vPosition=pvPositions[a];
		m_pPoint[a].m_vVelocity=V3(0,0,0);
	}

	for(a=1;a<m_lNumberPoints-1;a++) {
		V3 v0=m_pPoint[a-1].m_vPosition;
		V3 v1=m_pPoint[a].m_vPosition;
		V3 v2=m_pPoint[a+1].m_vPosition;
		V3 vPrev=v1-v0;
		V3 vNext=v2-v1;
		m_pPoint[a].m_vVelocity=VLerp(vPrev,vNext,0.5f)*3.0f;
	}

	for(a=0;a!=m_lNumberPoints-1;a++) {
		m_pPoint[a].m_fDistanceToNextPoint=VLength(m_pPoint[a+1].m_vPosition-m_pPoint[a].m_vPosition);
		m_fTotalLength+=m_pPoint[a].m_fDistanceToNextPoint;
	}
}
V3 HermiteCurve::GetPositionAtLength(float fLength)const {
	if(fLength<0)
		return m_pPoint[0].m_vPosition;
	if(fLength>m_fTotalLength)
		return m_pPoint[m_lNumberPoints-1].m_vPosition;
	int a;
	for(a=0;a!=m_lNumberPoints-1;a++) {
		if(fLength<m_pPoint[a].m_fDistanceToNextPoint)
			break;
		fLength-=m_pPoint[a].m_fDistanceToNextPoint;
	}
	//float t=fLength/m_pPoint[a].m_fDistanceToNextPoint;
	//uprintf("%f,%f t %f",m_pPoint[a].m_fDistanceToNextPoint,fLength,t);
	//V3 v0=Lerp(m_pPoint[a].m_vPosition,m_pPoint[a+1].m_vPosition,t);
	V3 v0=VHermite(m_pPoint[a].m_vPosition,m_pPoint[a].m_vVelocity,m_pPoint[a+1].m_vPosition,m_pPoint[a+1].m_vVelocity,0,m_pPoint[a].m_fDistanceToNextPoint,fLength);
	return v0;
}


bool SphereCollision(float* pfMoveDistance,const V2& vSphereACenter,float fSphereARadius,const V2& vSphereAMovement,const V2& vSphereBCenter,float fSphereBRadius)
{
	// FROM Pool Hall Lessons: Gamasutra
	// Early Escape test: if the length of the movevec is less
	// than distance between the centers of these circles minus
	// their radii, there's no way they can hit.
	float fDist=VLength(vSphereBCenter-vSphereACenter);
	float fSumRadii=fSphereARadius+fSphereBRadius;
	fDist-=fSumRadii;
	if(VLength(vSphereAMovement)<fDist)
	{
		return false;
	}
	// Normalize the movevec
	V2 vSphereADirection=VNormalize(vSphereAMovement);
	// Find C, the vector from the center of the moving
	// circle A to the center of B
	V2 vC=vSphereBCenter-vSphereACenter;
	// D = N . C = ||C|| * cos(angle between N and C)
	float fDot=VDot(vSphereADirection,vC);
	// Another early escape: Make sure that A is moving
	// towards B! If the dot product between the movevec and
	// B.center - A.center is less that or equal to 0,
	// A isn't isn't moving towards B
	if(fDot<=0)
	{
		return false;
	}
	// Find the length of the vector C
	float fLengthCenters=VLength(vC);
	float fDiffSquared=(fLengthCenters*fLengthCenters)-(fDot*fDot);
	// Escape test: if the closest that A will get to B
	// is more than the sum of their radii, there's no
	// way they are going collide
	float fSumRadiiSquared=fSumRadii*fSumRadii;
	if(fDiffSquared>=fSumRadiiSquared)
	{
		return false;
	}
	// We now have F and sumRadii, two sides of a right triangle.
	// Use these to find the third side, sqrt(T)
	float fThirdSide=fSumRadiiSquared-fDiffSquared;
	// If there is no such right triangle with sides length of
	// sumRadii and sqrt(f), T will probably be less than 0.
	// Better to check now than perform a square root of a
	// negative number.
	if(fThirdSide<0)
	{
		return false;
	}
	// Therefore the distance the circle has to travel along
	// movevec is D - sqrt(T)
	fDist=fDot-sqrtf(fThirdSide);
	// Get the magnitude of the movement vector
	if(fDist<0)
	{
		//Is inside
		return false;
	}
	float fLengthMovement=VLength(vSphereAMovement);
	// Finally, make sure that the distance A has to move
	// to touch B is not greater than the magnitude of the
	// movement vector.
	if(fLengthMovement<fDist)
	{
		return false;
	}
	*pfMoveDistance=fDist;
	return true;
}
*/



















#define DOTLIMIT 0.001f
#define LENLIMIT 0.0001f
/*
uint64_t GetFrameDuration(const uint64_t frameNum, const unsigned int fps) {
	const uint64_t t0 = GetSampleTime(frameNum, fps);
	const uint64_t t1 = GetSampleTime(frameNum+1, fps);
	return t1 - t0;
}

uint64_t GetSampleTime(const uint64_t frameNum, const unsigned int fps, const uint64_t startTime) {
	return (frameNum * 1000000ULL) / uint64_t(fps) + startTime;
}

uint64_t GetFrameNum(const uint64_t time, const unsigned int fps, const uint64_t startTime) {
	return ((time - startTime) * uint64_t(fps)) / 1000000ULL;
}

uint64 GetFrameNumNearest(const uint64_t time, const unsigned int fps, const uint64_t startTime) {
	const uint64_t halfFrameTime = 500000ULL / uint64_t(fps);
	return GetFrameNum(time + halfFrameTime, fps, startTime);
}
float CalcArea(const V2* points, const int w, const int h) {
	float a = 0.f;
	for(int j=0; j<h-1; j++) {
		for(int i=0; i<w-1; i++) {
			const int i0 = j * w + i;
			const int i1 = i0 + 1;
			const int i2 = i0 + w;
			const int i3 = i0 + w + 1;
			const V2 v01 = points[i1] - points[i0];
			const V2 v02 = points[i2] - points[i0];
			const float a0 = fabsf(VCross(v01, v02));
			const V2 v31 = points[i1] - points[i3];
			const V2 v32 = points[i2] - points[i3];
			const float a1 = fabsf(VCross(v31, v32));
			a += a0 + a1;
		}
	}
	return 0.5f * a;
}
*/














M33 QuatToM33T(const V4& quat) {
	float xs,ys,zs,wx,wy,wz,xx,xy,xz,yy,yz,zz;
	xs=quat[0]*2.0f;  ys=quat[1]*2.0f;  zs=quat[2]*2.0f;
	wx=quat[3]*xs; wy=quat[3]*ys; wz=quat[3]*zs;
	xx=quat[0]*xs; xy=quat[0]*ys; xz=quat[0]*zs;
	yy=quat[1]*ys; yz=quat[1]*zs; zz=quat[2]*zs;
	M33 res;
	res.m[0][0]=1.0f-(yy+zz);
	res.m[1][0]=xy+wz;
	res.m[2][0]=xz-wy;
	res.m[0][1]=xy-wz;
	res.m[1][1]=1.0f-(xx+zz);
	res.m[2][1]=yz+wx;
	res.m[0][2]=xz+wy;
	res.m[1][2]=yz-wx;
	res.m[2][2]=1.0f-(xx+yy);
	return res;
}

M33 QuatToM33(const V4& quat) {
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

V4 M33ToQuatT(const M33& mat) {
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

	q[0]=-q[0];
	q[1]=-q[1];
	q[2]=-q[2];

	return q;
}

V4 M33ToQuat(const M33& mat) {
	V4 q;
	int nxt[3]={1,2,0};
	float tr,s;
	int i,j,k;
	tr=mat.m[0][0]+mat.m[1][1]+mat.m[2][2];
	if(tr>0.0f) {
		s=sqrtf(tr+1.0f);
		q[3]=s*0.5f;
		s=0.5f/s;
		q[0]=(mat.m[1][2]-mat.m[2][1])*s;
		q[1]=(mat.m[2][0]-mat.m[0][2])*s;
		q[2]=(mat.m[0][1]-mat.m[1][0])*s;
	}else{
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


/*
//Linear fit using least squares
void fit() {
	int i;
	int n=4;
	double x[10]={50,70,100,120};
	double y[4]={12,15,21,25};
	double a,b;
	double xsum=0,x2sum=0,ysum=0,xysum=0;                //variables for sums/sigma of xi,yi,xi^2,xiyi etc
	for(i=0;i<n;i++) {
		xsum=xsum+x[i];									//calculate sigma(xi)
		ysum=ysum+y[i];									//calculate sigma(yi)
		x2sum=x2sum+pow(x[i],2);						//calculate sigma(x^2i)
		xysum=xysum+x[i]*y[i];							//calculate sigma(xi*yi)
	}
	a=(n*xysum-xsum*ysum)/(n*x2sum-xsum*xsum);			//calculate slope
	b=(x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum);		//calculate intercept
	double y_fit[countof(x)];							//an array to store the new fitted values of y    
	for(i=0;i<n;i++)
		y_fit[i]=a*x[i]+b;								//to calculate y(fitted) at given x points
	for(i=0;i<n;i++)
		uprintf("%d.%f,%f,%f\n",i,x[i],y[i],y_fit[i]);
}   
*/
