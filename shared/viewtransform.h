#ifndef __VIEWTRANSFORM1__
#define __VIEWTRANSFORM1__

#include <vector>
#include <map>
#include "shared/types.h"
#include "shared/math.h"

class ViewTransform {
	public:
		ViewTransform();
		virtual void SetSize(int lWidth,int lHeight);
		virtual V2 GetSize()const{return V2(m_width,m_height);}
		virtual float GetWidth()const{return m_width;}
		virtual float GetHeight()const{return m_height;}
		virtual V2 ScreenSize()const{return V2(m_width,m_height);}
		virtual V2 ScreenToPixel(const V2& screen)const;
		virtual V2 PixelToScreen(const V2& pixel)const;
		virtual V2 ViewToScreen(const V3& view)const;
		virtual V3 ScreenToView(const V2& screen)const;
		virtual V3 ViewToWorld(const V3& view)const;
		virtual V3 WorldToView(const V3& world)const;
		virtual M44 Projection()const{return m_projection;}
		virtual M44 ProjectionInverse()const{return m_projectionInverse;}
		void SetProjectionTransform(const M44& viewToWorld,const M44& projection);
		void SetProjectionTransform(float width,float height,const M44& viewToWorld,const M44& projection);
		virtual const M44& GetViewToWorld()const{return m_viewToWorld;}
		bool ScreenVisible(const V2& screen)const;
		void PrintInfo()const;
	protected:
		float m_width;
		float m_height;
		M44 m_projection;
		M44 m_projectionInverse;
		M44 m_viewToWorld;
};

#endif//__VIEWTRANSFORM1__
