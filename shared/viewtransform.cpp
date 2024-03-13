
#include "shared/types.h"
#include "shared/math.h"
#include "viewtransform.h"

//ViewTransform
ViewTransform::ViewTransform() {
	m_width=1.0f;
	m_height=1.0f;
}
void ViewTransform::SetProjectionTransform(float width,float height,const M44& viewToWorld,const M44& projection) {
	m_width=width;
	m_height=height;
	m_projection=projection;
	m_projectionInverse=MInverse(projection);
	m_viewToWorld=viewToWorld;
}
void ViewTransform::SetProjectionTransform(const M44& viewToWorld,const M44& projection) {
	m_projection=projection;
	m_projectionInverse=MInverse(projection);
	m_viewToWorld=viewToWorld;
}

V2 ViewTransform::ScreenToPixel(const V2& screen)const {
	return V2(((screen.x*0.5f)+0.5f)*m_width,((screen.y*0.5f)+0.5f)*m_height);
}
V2 ViewTransform::PixelToScreen(const V2& pixel)const {
	return V2((pixel.x/m_width)-0.5f,(pixel.y/m_height)-0.5f)*2.0f;
}
V3 ViewTransform::ScreenToView(const V2& vScreen)const {
	V3 vLocalCursor=V3(vScreen.x,vScreen.y,-1);
	V3 v0=TransformCoord(vLocalCursor,m_projectionInverse);
	return V3(v0.x,v0.y,v0.z);
}
V2 ViewTransform::ViewToScreen(const V3& view)const {
	V4 v0=V4(view.x,view.y,view.z,1);
	V4 vLocalCursor=TransformVec4(v0,m_projection);
	V2 vScreen=V2(vLocalCursor.x*(1.0f/vLocalCursor.w),-vLocalCursor.y*(1.0f/vLocalCursor.w));
	return vScreen;
}
bool ViewTransform::ScreenVisible(const V2& screen)const {
	if(screen.x<-0.5f || screen.x>0.5f)
		return false;
	if(screen.y<-0.5f || screen.y>0.5f)
		return false;
	return true;
}
V3 ViewTransform::ViewToWorld(const V3& view)const {
	return TransformCoord(view,m_viewToWorld);
}
V3 ViewTransform::WorldToView(const V3& world)const {
	return TransformCoord(world,MAffineInverse(m_viewToWorld));
}
void ViewTransform::SetSize(int width,int height) {
	m_width=(float)width;
	m_height=(float)height;
}
void ViewTransform::PrintInfo()const {
	//uprintf("Near %g",m_near);
	//uprintf("Far %g",m_far);
	uprintf("Width %g",m_width);
	uprintf("Height %g",m_height);
	//uprintf("mFOV Radians %g",m_fOVRad);

	uprintf("Projection:");
	V4 v=m_projection.XAxis();
	uprintf("%g,%g,%g,%g",v.x,v.y,v.z,v.w);
	v=m_projection.YAxis();
	uprintf("%g,%g,%g,%g",v.x,v.y,v.z,v.w);
	v=m_projection.ZAxis();
	uprintf("%g,%g,%g,%g",v.x,v.y,v.z,v.w);
	v=m_projection.WAxis();
	uprintf("%g,%g,%g,%g",v.x,v.y,v.z,v.w);
	uprintf("ViewToWorld:");
	v=m_viewToWorld.XAxis();
	uprintf("%g,%g,%g,%g",v.x,v.y,v.z,v.w);
	v=m_viewToWorld.YAxis();
	uprintf("%g,%g,%g,%g",v.x,v.y,v.z,v.w);
	v=m_viewToWorld.ZAxis();
	uprintf("%g,%g,%g,%g",v.x,v.y,v.z,v.w);
	v=m_viewToWorld.WAxis();
	uprintf("%g,%g,%g,%g",v.x,v.y,v.z,v.w);
}
