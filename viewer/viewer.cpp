
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <chrono>
#include <set>
#include <thread>
#include <assert.h>
#include <algorithm>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"
#include "shared/crc32.h"
#include "shared/dict.h"

#include "core/app.h"
#include "core/packet.h"
#include "core/transfer.h"
#include "core/sensorserver.h"
#include "core/sensorclient.h"
#include "memory/tensor.h"
#include "video/decoder.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_internal.h"
#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_glfw.h"

std::string g_defaultJson=R"(
{
    "id":"local-viewer",
    "debugTransfer":{
        "protocol":"udp",
        "bindAddress":{
            "ip":"0.0.0.0",
            "port":7778
        }
    },
    "debugConnections":[
        {
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":4001
                },
                "hostAddress":{
                    "ip":"127.0.0.1",
                    "port":7777
                }
            }
        },
        {
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":4002
                },
                "hostAddress":{
                    "ip":"127.0.0.1",
                    "port":7778
                }
            }
        }
    ],
    "services":[
        {
            "type":"sensorClient",
            "id":"viewer_client0",
            "decoder":{
                "type":"nvdecode"
            },
            "errorCorrection":{
                "type":"fec"
            },
            "transfers":[
                {
                    "protocol":"udp",
                    "bindAddress":{
                        "ip":"0.0.0.0",
                        "port":9001
                    },
                    "hostAddress":{
                        "ip":"127.0.0.1",
                        "port":8999
                    }
                }
            ]
        }
    ]
}
)";


inline ImVec4 ImLoad(const V4& v) {
	return ImVec4(v.x,v.y,v.z,v.w);
}
inline ImVec2 ImLoad(const V2& v) {
	return ImVec2(v.x,v.y);
}
inline V2 VLoad(const ImVec2& v) {
	return V2(v.x,v.y);
}

static void glfw_error_callback(int error,const char* description) {
	uprintf("Glfw Error %d: %s\n",error,description);
}

class Viewer : public App {
	public:
		Viewer() {
			m_defaultJson=g_defaultJson;
		}
		bool Display()const{return m_display;}
		virtual void MainLoop();
		bool InitDisplay(bool large);
		void CloseDisplay();
		void MainLoopDisplay();
	protected:
		bool m_display=false;
		void DrawGraphMulti(const GraphDatasets& datasets,double xmin,double xmax,double ymin,double ymax,bool columnChart,std::function<void(const char* clicked)> clicked);
		void DrawGraphTimers(const Dict& graph);
		bool m_verbose=false;
		GLFWwindow* m_window=0;

		struct DisplayClient {
			std::string m_id;

			struct StreamInfo {
				Dict m_info;
			};
			std::map<uint8_t,StreamInfo> m_streams;

			std::mutex m_lockFrame;

			struct VideoFrame {
				bool m_selected=false;
				uint8_t m_streamId=0xff;
				GLuint m_textureId=0;
				RectV2 m_rect={0,0,0,0};
				uint8_t* m_pixelsFrame=0;
				int m_width=0;
				int m_height=0;
				int m_channels=0;
				int m_indexTexture=0;
				int m_indexPixels=0;
				uint64_t m_timeCapture=0;
			};
			std::vector<VideoFrame> m_videoFrames;
		};
		std::map<std::string,DisplayClient*> m_idToDisplayClient;
		std::vector<std::string> m_interfaces;
		std::vector<AdaptorInfo*> m_remoteAdapters;

		std::mutex m_displayDevicesLock;
		std::set<std::string> m_hiddenDatasets;
		std::set<std::string> m_expandedComponents;
		std::string m_selectedId;
		std::string m_selectedIdDraw;
		void DrawGraphsJson(std::set<std::string>* hiddenDatasets,const Dict* graphs);
		void DrawVideo(const std::vector<DisplayClient*>& clients);
		void DrawSettings(const std::vector<DisplayClient*>& clients);
		void DrawStatus(int tick);
		void CopyPixels(const char* clientId,const Tensor* dec,const DecodedFrameInfo* info);
};

void Viewer::DrawGraphsJson(std::set<std::string>* hiddenDatasets,const Dict* graphs) {
	for(auto it=graphs->begin();it!=graphs->end();++it) {
		const Dict& graph=*it;
		const Dict& legend=*graph.Find("legend");
		const Dict& datasets=*graph.Find("datasets");
		double xmin,xmax;
		if(!graph.Get("xmin",&xmin,0.0))
			continue;
		if(!graph.Get("xmax",&xmax,0.0))
			continue;

		bool timers;
		legend.Get("timers",&timers,false);
		if(timers) {
			DrawGraphTimers(graph);
			continue;
		}

	    const uint32_t colors[]={0x5284dd, 0x68a855, 0x524ec4, 0xb37281, 0x607893, 0xc38bda, 0x8c8c8c,0x74b9cc,0xcdb564,0xb0724c};
		int cnt=0;
		GraphDatasets graphDatasets;
		graph.Get("formatX",&graphDatasets.m_formatX,"HH:MM:SS:MS");
		graph.Get("formatY",&graphDatasets.m_formatY,"us");
		for(auto it1=datasets.begin();it1!=datasets.end();++it1) {
			const Dict& dataset=*it1;
			graphDatasets.m_datasets.emplace_back();
			GraphDataset* graphDataset=&graphDatasets.m_datasets.back();
			dataset.Get("name",&graphDataset->m_name);
			std::string colorString;
			uint32_t color=colors[cnt];
			if(dataset.Get("color",&colorString)) {
				color=String2Color(colorString.c_str());
			}
			//uprintf("data %d\n",(int)data.Size());
			if(hiddenDatasets->find(graphDataset->m_name)==hiddenDatasets->end()) {
				graphDataset->m_color=(color&0x00ffffff)|0xff000000;
				const Dict* datax=dataset.Find("datax");
				const Dict* datay=dataset.Find("datay");
				if(datax && datay) {
					std::vector<double> vecx;
					std::vector<double> vecy;
					datax->GetTypedArray(&vecx);
					datay->GetTypedArray(&vecy);
					if(vecx.size()==vecy.size()) {
						for(int i=0;i!=(int)vecx.size();i++) {
							graphDataset->m_data.push_back({vecx[i],vecy[i]});
						}
					}else{
						FATAL("graph datax does not match datay");
					}
				}else{
					const Dict& data=*dataset.Find("data");
					for(auto it2=data.begin();it2!=data.end();++it2) {
						const Dict& child=*it2;
						double x,y;
						child.Get("x",&x);
						child.Get("y",&y);
						graphDataset->m_data.push_back({x,y});
						//uprintf("%f,%f\n",x,y);
					}
				}
			}else{
				graphDataset->m_color=(color&0x00ffffff)|0x7f000000;
			}
			cnt++;
		}
		if(xmin>=xmax)
			FATAL("Unable to draw graph");
		legend.Get("text",&graphDatasets.m_name);

		double ymin,ymax;
		graph.Get("ymin",&ymin,DHUGE);
		graph.Get("ymax",&ymax,-DHUGE);

		bool columnChart=false;
		graph.Get("columns",&columnChart);

		DrawGraphMulti(graphDatasets,xmin,xmax,ymin,ymax,columnChart,[&](const char* clicked){
			auto it=hiddenDatasets->find(clicked);
			if(it!=hiddenDatasets->end()) {
				hiddenDatasets->erase(it);
			}else{
				hiddenDatasets->insert(clicked);
			}
			//uprintf("clicked %s\n",clicked);
		});
	}
}

void Viewer::DrawGraphMulti(const GraphDatasets& datasets,double xmin,double xmax,double ymin,double ymax,bool columnChart,std::function<void(const char* clicked)> clicked) {
	float axisYWidth=150;
	ImVec2 vMin=ImGui::GetWindowContentRegionMin();
	ImVec2 vMax=ImGui::GetWindowContentRegionMax();
	ImVec2 frameSize(vMax.x-vMin.x,150);
	ImVec2 cursorPos=ImGui::GetCursorScreenPos();
	ImRect frameRect=ImRect(cursorPos,ImVec2(cursorPos.x+frameSize.x,cursorPos.y+frameSize.y));
	ImGui::ItemSize(frameRect);
	ImGuiID id=ImGui::GetID(datasets.m_name.c_str());
	ImDrawList* dl=ImGui::GetWindowDrawList();
	if(ImGui::ItemAdd(frameRect,id,&frameRect)) {
		uint32_t colorFrame=ImGui::GetColorU32(ImGuiCol_FrameBg);

		float textHeight=ImGui::GetTextLineHeight();
		float headlineHeight=textHeight;
		ImRect graphRect(frameRect);
		graphRect.Min.y+=headlineHeight;
		graphRect.Min.x+=axisYWidth;
		graphRect.Max.y-=textHeight;

		ImRect headlineRect(frameRect.Min,ImVec2(frameRect.Max.x,frameRect.Min.y+headlineHeight));
		ImRect axisXRect(ImVec2(graphRect.Min.x,graphRect.Max.y+1),frameRect.Max);
		ImRect axisYRect(ImVec2(frameRect.Min.x,frameRect.Min.y+headlineHeight),ImVec2(graphRect.Min.x,graphRect.Max.y));
		ImRect cornerRect(ImVec2(axisYRect.Min.x,axisYRect.Max.y),ImVec2(axisYRect.Max.x,axisXRect.Max.y));

		dl->AddRectFilled(graphRect.Min,graphRect.Max,0xff000000);
		dl->AddRectFilled(headlineRect.Min,headlineRect.Max,colorFrame);
		dl->AddRectFilled(axisYRect.Min,axisYRect.Max,colorFrame);
		dl->AddRectFilled(axisXRect.Min,axisXRect.Max,colorFrame);
		dl->AddRectFilled(cornerRect.Min,cornerRect.Max,colorFrame);

		ImGui::PushClipRect(headlineRect.Min,headlineRect.Max,true);
		ImVec2 textSize=ImGui::CalcTextSize(datasets.m_name.c_str());
		dl->AddText(ImVec2(((headlineRect.Min.x+headlineRect.Max.x)-textSize.x)/2,headlineRect.Min.y),0xffffffff,datasets.m_name.c_str());
		ImGui::PopClipRect();
		double axisYMax=0;
		double axisYMin=0;
		if(datasets.m_datasets.size()) {
			bool first=true;
			for(int i=0;i!=(int)datasets.m_datasets.size();i++) {
				for(int j=0;j!=(int)datasets.m_datasets[i].m_data.size();j++) {
					if(first) {
						first=false;
						axisYMin=datasets.m_datasets[i].m_data[j].m_y;
						axisYMax=axisYMin;
					}else{
						axisYMax=MAX(datasets.m_datasets[i].m_data[j].m_y,axisYMax);
						axisYMin=MIN(datasets.m_datasets[i].m_data[j].m_y,axisYMin);
					}
				}
			}
			if(ymin!=DHUGE) {
				axisYMin=ymin;
			}
			if(ymax!=-DHUGE) {
				axisYMax=ymax;
			}
		}
		char buffer[100];
		if(1) {
			uint64_t timeBegin=(uint64_t)xmin;
			uint64_t timeEnd=(uint64_t)xmax;
			double lineXPoses[64];
			int lineXPosesCount=0;

			ImVec2 textSize=ImGui::CalcTextSize("00:00:00:000");
			float onePixelTime=(float)(timeEnd-timeBegin)/(float)axisXRect.GetSize().x;
			int timeTextTimeWidth=(int)(onePixelTime*textSize.x);
			double count=(double)(timeEnd-timeBegin)/((double)timeTextTimeWidth+(onePixelTime*40));
			uint64_t timeModulo=(uint64_t)((double)(timeEnd-timeBegin)/count);
			uint64_t timeBeginEx=timeBegin-timeTextTimeWidth;
			uint64_t timeEndEx=timeEnd+timeTextTimeWidth;
			uint64_t timeShow=timeEndEx-(timeEndEx%Max(1,(int)timeModulo));

			ImGui::PushClipRect(axisXRect.Min,axisXRect.Max,true);
			while(timeShow>timeBeginEx) {
				double pos=(float)(((double)timeShow-(double)timeBegin)/((double)timeEnd-(double)timeBegin));
				uint64_t millisecondsSince1970=(uint64_t)timeShow;
				time_t rawtime;
				rawtime=(time_t)(millisecondsSince1970/1000);
				struct tm* ts=gmtime(&rawtime);
				if(datasets.m_formatX=="HH:MM:SS:MS") {
					snprintf(buffer,sizeof(buffer),"%02d:%02d:%02d:%03d",ts->tm_hour,ts->tm_min,ts->tm_sec,(int)(millisecondsSince1970%1000));
				}else
				if(datasets.m_formatX=="HH:MM:SS") {
					snprintf(buffer,sizeof(buffer),"%02d:%02d:%02d",ts->tm_hour,ts->tm_min,ts->tm_sec);
				}else{
					FATAL("graph unknown format x %s",datasets.m_formatX.c_str());
				}
				ImVec2 textSize=ImGui::CalcTextSize(buffer);
				double pixelX=axisXRect.Min.x+(graphRect.GetSize().x*pos);
				lineXPoses[lineXPosesCount++]=pixelX;
				dl->AddText(ImVec2((float)(pixelX-(textSize.x/2)),(float)axisXRect.Min.y),0xffffffff,buffer);
				timeShow-=timeModulo;
			}
			ImGui::PopClipRect();
			ImGui::PushClipRect(graphRect.Min,graphRect.Max,true);
			for(int i=0;i!=lineXPosesCount;i++) {
				dl->AddLine(ImVec2((float)lineXPoses[i],(float)graphRect.Min.y),ImVec2((float)lineXPoses[i],(float)graphRect.Max.y),0xff303030);
			}
			ImGui::PopClipRect();
		}

		if(axisYMax==axisYMin) {
			if(axisYMax)
				axisYMin-=10;
			axisYMax+=10;
		}
		int numberLinesY=4;

		double lineYPoses[64];
		int lineYPosesCount=0;
		double axisYBegin=axisYMin;
		double axisYEnd=axisYMax;

		double axisYStep=(axisYEnd-axisYBegin)/(float)(numberLinesY-1);
		double axisYCur=axisYBegin;
		double verticalMargin=axisYStep*0.5f;
		double offset=axisYCur-verticalMargin;
		ImGui::PushClipRect(axisYRect.Min,axisYRect.Max,true);
		double scale=axisYEnd-axisYBegin+verticalMargin*2;

		float height=graphRect.GetSize().y;
		scale=height/scale;

		for(int i=0;i!=numberLinesY;i++) {

			double posY0=((float)axisYCur-offset)*scale;
			double pixelY=graphRect.Max.y-posY0;
			lineYPoses[lineYPosesCount++]=pixelY;

			if(datasets.m_formatY=="us") {
				snprintf(buffer,sizeof(buffer),"%.0f us",(float)axisYCur);
			}else
			if(datasets.m_formatY=="bytes") {
				snprintf(buffer,sizeof(buffer),"%.0f bytes",(float)axisYCur);
			}else
			if(datasets.m_formatY=="kb") {
				snprintf(buffer,sizeof(buffer),"%.1f kb",(float)axisYCur);
			}else
			if(datasets.m_formatY=="mb") {
				snprintf(buffer,sizeof(buffer),"%.1f kb",(float)axisYCur);
			}else
			if(datasets.m_formatY=="kbs") {
				snprintf(buffer,sizeof(buffer),"%.1f kB/s",(float)axisYCur);
			}else
			if(datasets.m_formatY=="mbits") {
				snprintf(buffer,sizeof(buffer),"%.2f 	Mbit/s",(float)axisYCur);
			}else
			if(datasets.m_formatY=="prc") {
				snprintf(buffer,sizeof(buffer),"%.04f%%",(float)axisYCur);
			}else
			if(datasets.m_formatY=="int") {
				snprintf(buffer,sizeof(buffer),"%d",(int)axisYCur);
			}else
			if(datasets.m_formatY=="dBm") {
				snprintf(buffer,sizeof(buffer),"%d dBm",(int)axisYCur);
			}else{
				snprintf(buffer,sizeof(buffer),"%f",(float)axisYCur);
			}

			ImVec2 textSize=ImGui::CalcTextSize(buffer);
			dl->AddText(ImVec2((float)(axisYRect.Max.x-textSize.x),(float)(pixelY-(textHeight/2))),0xffffffff,buffer);

			axisYCur+=axisYStep;
		}

		ImGui::PopClipRect();
		ImGui::PushClipRect(graphRect.Min,graphRect.Max,true);
		for(int i=0;i!=lineYPosesCount;i++) {
			dl->AddLine(ImVec2((float)graphRect.Min.x,(float)lineYPoses[i]),ImVec2((float)graphRect.Max.x,(float)lineYPoses[i]),0xff303030);
		}
		if(columnChart) {
			for(int i=0;i!=(int)datasets.m_datasets.size();i++) {
				for(int j=0;j<(int)datasets.m_datasets[i].m_data.size();j++) {
					double posX1=(datasets.m_datasets[i].m_data[j].m_x-xmin)/(xmax-xmin);
					posX1=graphRect.Min.x+(graphRect.GetSize().x*posX1);
					double val1=datasets.m_datasets[i].m_data[j].m_y;
					if(val1 == 0) continue;
					double posY1=((float)val1-offset)*scale;
					dl->AddRectFilled(ImVec2((float)posX1-4,(float)(graphRect.Max.y)),ImVec2((float)posX1+4,(float)(graphRect.Max.y-posY1)),datasets.m_datasets[i].m_color);
				}
			}
		}else{
			for(int i=0;i!=(int)datasets.m_datasets.size();i++) {
				for(int j=1;j<(int)datasets.m_datasets[i].m_data.size();j++) {
					double posX0=(datasets.m_datasets[i].m_data[j-1].m_x-xmin)/(xmax-xmin);
					double posX1=(datasets.m_datasets[i].m_data[j].m_x-xmin)/(xmax-xmin);
					posX0=graphRect.Min.x+(graphRect.GetSize().x*posX0);
					posX1=graphRect.Min.x+(graphRect.GetSize().x*posX1);
					double val0=datasets.m_datasets[i].m_data[j-1].m_y;
					double val1=datasets.m_datasets[i].m_data[j].m_y;
					double posY0=((float)val0-offset)*scale;
					double posY1=((float)val1-offset)*scale;
					dl->AddLine(ImVec2((float)posX0,(float)(graphRect.Max.y-posY0)),ImVec2((float)posX1,(float)(graphRect.Max.y-posY1)),datasets.m_datasets[i].m_color);
				}
			}
		}
		ImGui::PopClipRect();
		ImVec2 pos=ImVec2(graphRect.Min.x+2,graphRect.Min.y+2);
		V2 mouse=VLoad(ImGui::GetMousePos());
		for(auto& n:datasets.m_datasets) {
			ImVec2 textSize=ImGui::CalcTextSize(n.m_name.c_str());
			ImRect rectTS(pos,pos);
			rectTS.Max.x+=textSize.x+2;
			rectTS.Max.y+=textSize.y+2;
			RectV2 rect=RectV2(VLoad(rectTS.Min),VLoad(rectTS.Max));
			if(rect.Inside(mouse)) {
				if(ImGui::IsMouseClicked(0))
					clicked(n.m_name.c_str());
			}
			dl->AddRectFilled(rectTS.Min,rectTS.Max,n.m_color);
			dl->AddText(ImVec2(pos.x+1,pos.y+1),0xffffffff,n.m_name.c_str());
			pos.x+=textSize.x+2+2;
		}
	}
}

void Viewer::DrawGraphTimers(const Dict& graph) {
	const Dict& legend=*graph.Find("legend");
	const Dict& dataset=*graph.Find("dataset");
	std::string name;
	legend.Get("text",&name);

	double xmin,xmax;
	if(!graph.Get("xmin",&xmin,0.0))
		FATAL("xmin not defined");
	if(!graph.Get("xmax",&xmax,0.0))
		FATAL("xmax not defined");

	float axisYWidth=150;
	ImVec2 vMin=ImGui::GetWindowContentRegionMin();
	ImVec2 vMax=ImGui::GetWindowContentRegionMax();
	ImVec2 frameSize(vMax.x-vMin.x,150);
	ImVec2 cursorPos=ImGui::GetCursorScreenPos();
	ImRect frameRect=ImRect(cursorPos,ImVec2(cursorPos.x+frameSize.x,cursorPos.y+frameSize.y));
	ImGui::ItemSize(frameRect);
	ImGuiID id=ImGui::GetID(name.c_str());
	ImDrawList* dl=ImGui::GetWindowDrawList();
	if(ImGui::ItemAdd(frameRect,id,&frameRect)) {
		uint32_t colorFrame=ImGui::GetColorU32(ImGuiCol_FrameBg);

		float textHeight=ImGui::GetTextLineHeight();
		float headlineHeight=textHeight;
		ImRect graphRect(frameRect);
		graphRect.Min.y+=headlineHeight;
		graphRect.Min.x+=axisYWidth;
		graphRect.Max.y-=textHeight;

		ImRect headlineRect(frameRect.Min,ImVec2(frameRect.Max.x,frameRect.Min.y+headlineHeight));
		ImRect axisXRect(ImVec2(graphRect.Min.x,graphRect.Max.y+1),frameRect.Max);
		ImRect axisYRect(ImVec2(frameRect.Min.x,frameRect.Min.y+headlineHeight),ImVec2(graphRect.Min.x,graphRect.Max.y));
		ImRect cornerRect(ImVec2(axisYRect.Min.x,axisYRect.Max.y),ImVec2(axisYRect.Max.x,axisXRect.Max.y));

		dl->AddRectFilled(graphRect.Min,graphRect.Max,0xff000000);
		dl->AddRectFilled(headlineRect.Min,headlineRect.Max,colorFrame);
		dl->AddRectFilled(axisYRect.Min,axisYRect.Max,colorFrame);
		dl->AddRectFilled(axisXRect.Min,axisXRect.Max,colorFrame);
		dl->AddRectFilled(cornerRect.Min,cornerRect.Max,colorFrame);

		ImGui::PushClipRect(headlineRect.Min,headlineRect.Max,true);
		ImVec2 textSize=ImGui::CalcTextSize(name.c_str());
		dl->AddText(ImVec2(((headlineRect.Min.x+headlineRect.Max.x)-textSize.x)/2,headlineRect.Min.y),0xffffffff,name.c_str());
		ImGui::PopClipRect();

		std::vector<double> datax;
		std::vector<double> datay;
		dataset.GetTypedArray("datax",&datax);
		dataset.GetTypedArray("datay",&datay);

		int stackCount=4;
		if(datay.size()!=datax.size()*4)
			FATAL("timers strack count does not match");

		double axisYMax=0;
		double axisYMin=0;
		bool first=true;
		int cnt=0;
		for(int i=0;i!=(int)datax.size();i++) {
			double sum=0;
			for(int j=0;j!=stackCount;j++) {
				sum+=datay[cnt++];
			}
			if(first) {
				first=false;
				axisYMin=sum;
				axisYMax=sum;
			}else{
				axisYMax=MAX(sum,axisYMax);
				axisYMin=MIN(sum,axisYMin);
			}
		}
		char buffer[100];
		{
			uint64_t timeBegin=(uint64_t)xmin;
			uint64_t timeEnd=(uint64_t)xmax;
			double lineXPoses[64];
			int lineXPosesCount=0;
			ImVec2 textSize=ImGui::CalcTextSize("00:00:00");
			float onePixelTime=(float)(timeEnd-timeBegin)/(float)axisXRect.GetSize().x;
			int timeTextTimeWidth=(int)(onePixelTime*textSize.x);
			double count=(double)(timeEnd-timeBegin)/((double)timeTextTimeWidth+(onePixelTime*40));
			uint64_t timeModulo=(uint64_t)((double)(timeEnd-timeBegin)/count);
			uint64_t timeBeginEx=timeBegin-timeTextTimeWidth;
			uint64_t timeEndEx=timeEnd+timeTextTimeWidth;
			uint64_t timeShow=timeEndEx-(timeEndEx%timeModulo);
			ImGui::PushClipRect(axisXRect.Min,axisXRect.Max,true);
			while(timeShow>timeBeginEx) {
				double pos=(float)(((double)timeShow-(double)timeBegin)/((double)timeEnd-(double)timeBegin));
				uint64_t millisecondsSince1970=(uint64_t)timeShow;
				time_t rawtime;
				rawtime=(time_t)(millisecondsSince1970/1000);
				struct tm* ts=gmtime(&rawtime);
				snprintf(buffer,sizeof(buffer),"%02d:%02d:%02d",ts->tm_hour,ts->tm_min,ts->tm_sec);
				ImVec2 textSize=ImGui::CalcTextSize(buffer);
				double pixelX=axisXRect.Min.x+(graphRect.GetSize().x*pos);
				if(lineXPosesCount>=countof(lineXPoses))
					FATAL("tf?");
				lineXPoses[lineXPosesCount++]=pixelX;
				dl->AddText(ImVec2((float)(pixelX-(textSize.x/2)),(float)axisXRect.Min.y),0xffffffff,buffer);
				timeShow-=timeModulo;
			}
			ImGui::PopClipRect();
			ImGui::PushClipRect(graphRect.Min,graphRect.Max,true);
			for(int i=0;i!=lineXPosesCount;i++) {
				dl->AddLine(ImVec2((float)lineXPoses[i],(float)graphRect.Min.y),ImVec2((float)lineXPoses[i],(float)graphRect.Max.y),0xff303030);
			}
			ImGui::PopClipRect();
		}
		if(axisYMax==axisYMin) {
			axisYMin-=10;
			axisYMax+=10;
		}
		axisYMin=0;

		int numberLinesY=4;

		double lineYPoses[64];
		int lineYPosesCount=0;
		double axisYBegin=axisYMin;
		double axisYEnd=axisYMax;

		double axisYStep=(axisYEnd-axisYBegin)/(float)(numberLinesY-1);
		double axisYCur=axisYBegin;
		double verticalMargin=axisYStep*0.5f;
		double offset=axisYCur-verticalMargin;
		ImGui::PushClipRect(axisYRect.Min,axisYRect.Max,true);
		double scale=axisYEnd-axisYBegin+verticalMargin*2;

		float height=graphRect.GetSize().y;
		scale=height/scale;

		for(int i=0;i!=numberLinesY;i++) {
			double posY0=((float)axisYCur-offset)*scale;
			double pixelY=graphRect.Max.y-posY0;
			lineYPoses[lineYPosesCount++]=pixelY;
			snprintf(buffer,sizeof(buffer),"%.0f us",(float)axisYCur);
			ImVec2 textSize=ImGui::CalcTextSize(buffer);
			dl->AddText(ImVec2((float)(axisYRect.Max.x-textSize.x),(float)(pixelY-(textHeight/2))),0xffffffff,buffer);
			axisYCur+=axisYStep;
		}

		ImGui::PopClipRect();
		ImGui::PushClipRect(graphRect.Min,graphRect.Max,true);
		for(int i=0;i!=lineYPosesCount;i++) {
			dl->AddLine(ImVec2((float)graphRect.Min.x,(float)lineYPoses[i]),ImVec2((float)graphRect.Max.x,(float)lineYPoses[i]),0xff303030);
		}
		ImGui::PopClipRect();
		//uint32_t colors[4]={0xff00ff00,0xff0000ff,0xffffff00,0xffff00ff};
		const uint32_t colors[]={0x5284dd, 0x68a855, 0x524ec4, 0xb37281, 0x607893, 0xc38bda, 0x8c8c8c,0x74b9cc,0xcdb564,0xb0724c};
		int sizex=(int)graphRect.GetSize().x;
		float prevStack[16];
		int prevx=0;
		first=true;
		for(int i=0;i!=(int)datax.size();i++) {
			double posX=(datax[i]-xmin)/(xmax-xmin);
			posX=((float)sizex*posX);
			int px=MIN((int)posX,sizex-1);
			//double posY=((double)0-offset)*scale;
			double val=0;
			float curStack[16];
			for(int j=0;j!=stackCount;j++) {
				val+=datay[i*stackCount+j];
				double posY1=((float)val-offset)*scale;
				curStack[j]=(float)posY1;
				//posY=posY1;
			}
			if(first) {
				first=false;
				for(int j=0;j!=stackCount;j++) {
					prevStack[j]=curStack[j];
				}
				prevx=px;
			}
			float stepStack[16];
			float dist=(float)(px-prevx+1);
			for(int j=0;j!=stackCount;j++) {
				stepStack[j]=(curStack[j]-prevStack[j])/dist;
			}
			while(prevx!=px) {
				if(prevx>=0) {
					float lnx=graphRect.Min.x+(float)prevx;
					float posY0=(0.0f-(float)offset)*(float)scale;
					for(int j=0;j!=stackCount;j++) {
						float posY1=prevStack[j];
						dl->AddLine(ImVec2(lnx,(float)(graphRect.Max.y-posY0)),ImVec2(lnx,(float)(graphRect.Max.y-posY1)),colors[j]|0xff000000);
						posY0=posY1;
					}
				}
				for(int j=0;j!=stackCount;j++) {
					prevStack[j]+=stepStack[j];
				}
				prevx++;
			}
			for(int j=0;j!=stackCount;j++) {
				prevStack[j]=curStack[j];
			}
			prevx=px;
		}
		ImVec2 pos=ImVec2(graphRect.Min.x+2,graphRect.Min.y+2);
		const Dict& infos=*dataset.Find("infos");
		for(auto it=infos.begin();it!=infos.end();++it) {
			const Dict& info=*it;
			info.Get("name",&name,"NA");
			std::string colorString;
			info.Get("color",&colorString,"#ffffff");
			uint32_t color=String2Color(colorString.c_str());
			ImVec2 textSize=ImGui::CalcTextSize(name.c_str());
			ImRect rectTS(pos,pos);
			rectTS.Max.x+=textSize.x+2;
			rectTS.Max.y+=textSize.y+2;
			//RectV2 rect=RectV2(VLoad(rectTS.Min),VLoad(rectTS.Max));
			dl->AddRectFilled(rectTS.Min,rectTS.Max,color|0xff000000);
			dl->AddText(ImVec2(pos.x+1,pos.y+1),0xffffffff,name.c_str());
			pos.x+=textSize.x+2+2;
		}

	}
}

void Viewer::MainLoop() {
	if(InitDisplay(true)) {
		MainLoopDisplay();
		CloseDisplay();
	}
}
void Viewer::DrawSettings(const std::vector<DisplayClient*>& clients) {
	if(!m_selectedId.size())
		return;

	Dict properties;
	if(!GetProperties(&properties,m_selectedId))
		FATAL("Viewer::DrawSettings no properties");

	m_displayDevicesLock.lock();
	if(m_selectedIdDraw!=m_selectedId) {
		m_selectedIdDraw=m_selectedId;
		if(!UpdateProperties(m_selectedId)) {
			uprintf("Unable to find id %s\n",m_selectedId.c_str());
		}
	}
	m_displayDevicesLock.unlock();

	if(!properties.Size())		//Not receivced yet
		return;
	bool objectsModified=false;
	for(auto it1=properties.begin();it1!=properties.end();++it1) {
		Dict* dict=&*it1;
		ImGui::TextUnformatted(dict->Name().c_str());

		Dict* schema=dict->Find("schema");
		Dict* values=dict->Find("values");
		if(!schema || !values) {
			//uprintf("schema and/or values not found in object\n");
			continue;
		}
		const Dict* schemaProperties=schema->Find("properties");
		if(!schemaProperties) {
			uprintf("properties not found in schema\n");
			return;
		}
		bool modified=false;
		for(auto it=values->begin();it!=values->end();++it) {
			Dict* value=&*it;
			const Dict* p=schemaProperties->Find(value->Name().c_str());
			std::string type;
			if(p && p->Get("type",&type)) {
				if(type=="boolean") {
					bool val;
					value->GetBool(&val);
					if(ImGui::Checkbox(value->Name().c_str(),&val)) {
						value->SetBool(val);
						modified=true;
					}
					//ImGui::Text("bolean %s",value->Name().c_str());
				}else
				if(type=="integer") {
					double step,fast;
					double minimum,maximum;
					p->Get("step",&step,1);
					p->Get("fast",&fast,10);
					p->Get("minimum",&minimum,0);
					p->Get("maximum",&maximum,100);
					//ImGui::Text("int %s",value->Name().c_str());
					double dval=0;
					value->GetDouble(&dval);
					int val=(int)dval;
					if(ImGui::InputInt(value->Name().c_str(),&val,(int)step,(int)fast,ImGuiInputTextFlags_EnterReturnsTrue)) {
						val=MAX(val,(int)minimum);
						val=MIN(val,(int)maximum);
						if(val!=(int)dval) {
							value->SetDouble((double)val);
							modified=true;
						}
					}
				}else
				if(type=="number") {
					const Dict* items=p->Find("items");
					if(items) {
						ImGui::Text("double items %s",value->Name().c_str());
					}else{
						double step,fast;
						double minimum,maximum;
						p->Get("step",&step,1);
						p->Get("fast",&fast,10);
						p->Get("minimum",&minimum,0);
						p->Get("maximum",&maximum,100);
						std::string fmt;
						p->Get("printFormat",&fmt,"%f");
						//ImGui::Text("int %s",value->Name().c_str());
						double dval=0;
						value->GetDouble(&dval);
						double val=dval;
						if(ImGui::InputDouble(value->Name().c_str(),&val,step,fast,fmt.c_str(),ImGuiInputTextFlags_EnterReturnsTrue)) {
							val=MAX(val,(int)minimum);
							val=MIN(val,(int)maximum);
							if(val!=dval) {
								value->SetDouble(val);
								modified=true;
							}
						}
						//ImGui::Text("double %s",value->Name().c_str());
					}
				}else
				if(type=="string") {
					const Dict* items=p->Find("items");
					if(items) {
						const Dict* itemsEnum=items->Find("enum");
						std::string v;
						value->GetString(&v);
						if(itemsEnum) {
							std::vector<std::string> strings;
							int index=0;
							for(auto it1=itemsEnum->begin();it1!=itemsEnum->end();++it1) {
								const Dict* d=&*it1;
								std::string s;
								d->GetString(&s);
								strings.push_back(s);
								if(strings[index]!=v)
									index++;
							}
							if(index==(int)strings.size())
								index=0;
							std::vector<const char*> selectionItems;
							for(auto& s:strings)
								selectionItems.push_back(s.c_str());

							std::string uniqueName=value->Name()+"###"+dict->Name();
							if(ImGui::Combo(uniqueName.c_str(),&index,selectionItems.data(), (int)selectionItems.size())) {
								uprintf("change mode %s->%s\n",v.c_str(),strings[index].c_str());
								modified=strings[index]!=v;
								value->SetString(strings[index].c_str());
							}
						}else{
							ImGui::Text("no enum! %s",value->Name().c_str());
						}
					}else{
						ImGui::Text("string %s",value->Name().c_str());
					}
				}else{
					ImGui::Text("UNSUPPORTED %s",value->Name().c_str());
				}
			}else{
				ImGui::BeginDisabled();
				ImGui::Text("%s",value->Name().c_str());
				ImGui::EndDisabled();
			}
			objectsModified|=modified;
			dict->Set("modified",modified);
		}
	}
	if(objectsModified) {
		SetProperties(properties,m_selectedId);
	}
}

void Viewer::DrawVideo(const std::vector<DisplayClient*>& clients) {
	V2 wpos=VLoad(ImGui::GetWindowPos());
	V2 crmin=VLoad(ImGui::GetWindowContentRegionMin());
	V2 crmax=VLoad(ImGui::GetWindowContentRegionMax());
	wpos+=crmin;
	ImDrawList* dl=ImGui::GetWindowDrawList();
	RectV2 rect=RectV2(wpos,wpos+(crmax-crmin));
	for(DisplayClient* cd:clients) {
		cd->m_lockFrame.lock();
		for(DisplayClient::VideoFrame& vf:cd->m_videoFrames) {
			if(vf.m_indexTexture==vf.m_indexPixels)
				break;
			vf.m_indexTexture=vf.m_indexPixels;
			if(vf.m_pixelsFrame) {
				if(!vf.m_textureId) {
					glGenTextures(1,&vf.m_textureId);
					glBindTexture(GL_TEXTURE_2D,vf.m_textureId);
					float color[]={.2f,.2f,.2f,0};
					glTexParameterfv(GL_TEXTURE_2D,GL_TEXTURE_BORDER_COLOR,color);
					glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_BORDER);
					glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_BORDER);
					glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
					glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
				}
				glBindTexture(GL_TEXTURE_2D,vf.m_textureId);
				if(vf.m_channels==4) {
					glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,vf.m_width,vf.m_height,0,GL_RGBA,GL_UNSIGNED_BYTE,vf.m_pixelsFrame);
				}else{
					glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,vf.m_width,vf.m_height,0,GL_RGB,GL_UNSIGNED_BYTE,vf.m_pixelsFrame);
				}
			}
		}
		cd->m_lockFrame.unlock();
	}
	for(DisplayClient* cd:clients) {
		for(DisplayClient::VideoFrame& vf:cd->m_videoFrames) {
			if(!vf.m_textureId)
				break;
			float w=(float)vf.m_width;
			float h=(float)vf.m_height;
			vf.m_rect=RectV2(wpos,wpos+V2(w,h)+V2(2,2));
			wpos.x+=w+1;
		}
	}
	V2 mouse=VLoad(ImGui::GetMousePos());
	if(rect.Inside(mouse)) {
		if(ImGui::IsMouseClicked(0)) {
			for(DisplayClient* cd:clients) {
				for(DisplayClient::VideoFrame& vf:cd->m_videoFrames) {
					vf.m_selected=false;
					if(!vf.m_textureId)
						continue;
					if(vf.m_rect.Inside(mouse)) {
						vf.m_selected=true;
					}
				}
			}
		}
	}
	for(DisplayClient* cd:clients) {
		for(DisplayClient::VideoFrame& vf:cd->m_videoFrames) {
			if(!vf.m_textureId)
				break;
			dl->AddImage((ImTextureID)(size_t)vf.m_textureId,ImLoad(vf.m_rect.tl()+V2(1,1)),ImLoad(vf.m_rect.br()-V2(1,1)),ImVec2(0,0),ImVec2(1,1));
			dl->AddText(ImVec2(vf.m_rect.tl().x+1,vf.m_rect.br().y-20),0xffffffff,stdx::format_string("stream %d client %s %dx%d",vf.m_streamId,cd->m_id.c_str(),vf.m_width,vf.m_height).c_str());
			if(!vf.m_selected)
				continue;
			float sx=vf.m_rect.width/3.0f;
			float sy=vf.m_rect.height/3.0f;
			V2 wpos=vf.m_rect.tl();
			float w=vf.m_rect.width-2;
			float h=vf.m_rect.height-2;
			dl->AddLine(ImLoad(wpos+V2(0,0)),ImLoad(wpos+V2(0,sy)),0xffffffff,1.0f);
			dl->AddLine(ImLoad(wpos+V2(0,0)),ImLoad(wpos+V2(sx,0)),0xffffffff,1.0f);
			dl->AddLine(ImLoad(wpos+V2(w+1,0-1)),ImLoad(wpos+V2(w+1,sy)),0xffffffff,1.0f);
			dl->AddLine(ImLoad(wpos+V2(w+1,0)),ImLoad(wpos+V2(w-sx,0)),0xffffffff,1.0f);
			dl->AddLine(ImLoad(wpos+V2(0,h+1)),ImLoad(wpos+V2(0,h-sy)),0xffffffff,1.0f);
			dl->AddLine(ImLoad(wpos+V2(0,h+1)),ImLoad(wpos+V2(sx,h+1)),0xffffffff,1.0f);
			dl->AddLine(ImLoad(wpos+V2(w+1,h+1)),ImLoad(wpos+V2(w+1,h-sy)),0xffffffff,1.0f);
			dl->AddLine(ImLoad(wpos+V2(w+1,h+1)),ImLoad(wpos+V2(w-sx,h+1)),0xffffffff,1.0f);
		}
	}
	for(DisplayClient* cd:clients) {
		cd->m_lockFrame.lock();
		for(const auto& [k,v]:cd->m_streams) {
			if(v.m_info.Size()) {
				int rssi;
				const Dict* rssiDict = v.m_info.Find("RSSI");
				if(rssiDict) {
					for(auto it1=rssiDict->begin();it1!=rssiDict->end();++it1) {
						const Dict& entry=*it1;
						std::string adapterName, ip;
						entry.Get("name",&adapterName);
						entry.Get("rssi",&rssi);
						entry.Get("ip",&ip);
						bool found=false;
						for(AdaptorInfo* ai:m_remoteAdapters) {
							if(ai->m_name==adapterName && ai->m_ip==ip) {
								found=true;
								ai->m_rssi.RegisterMax(GetTimeEpochMicroseconds(),rssi);
							}
						}
						if(!found) {
							AdaptorInfo* ai=new AdaptorInfo();
							ai->m_rssi.SetMaxTime(20*1000000);
							ai->m_name=adapterName;
							ai->m_ip=ip;
							m_remoteAdapters.push_back(ai);
							ai->m_rssi.RegisterMax(GetTimeEpochMicroseconds(),rssi);
						}
					}
				}
				else {
					uint32_t color=0xe0e0e0ff;
					ImGui::TextColored(ImLoad(uint322V4(color)),"%s",v.m_info.WriteToJson(false,0,4,false,true,"%.2f","%.3f").c_str());
				}
			}
		}
		cd->m_lockFrame.unlock();
	}
}
void Viewer::DrawStatus(int tick) {
	std::function<bool(const Dict& dict,int depth)> iterate=[&](const Dict& dict,int depth)->bool{
		if(dict.Name()=="schema" || dict.Name()=="graphs")
			return false;
		//uprintf("%stype \"%s\" name \"%s\" value \"%s\"\n",spaces.c_str(),dict.TypeName(),dict.Name().c_str(),dict.Value().c_str());
		std::string spaces=stdx::spaces(depth*4).c_str();
		if(dict.Type()==Dict::Object) {
			ImGui::Text("%s%s{",spaces.c_str(),dict.Name().size() ? (dict.Name()+":").c_str():"");
		}else
		if(dict.Type()==Dict::Array) {
			ImGui::Text("%s%s[",spaces.c_str(),dict.Name().size() ? (dict.Name()+":").c_str():"");
		}else
		if(dict.Type()==Dict::String) {
			ImGui::Text("%s%s\"%s\"",spaces.c_str(),dict.Name().size() ? (dict.Name()+":").c_str():"",dict.Value().c_str());
		}else{
			ImGui::Text("%s%s%s",spaces.c_str(),dict.Name().size() ? (dict.Name()+":").c_str():"",dict.Value().c_str());
		}
		const Dict* child=dict.First();
		while(child) {
			iterate(*child,depth+1);
			child=child->Next();
		}
		if(dict.Type()==Dict::Object) {
			ImGui::Text("%s}",spaces.c_str());
		}else
		if(dict.Type()==Dict::Array) {
			ImGui::Text("%s]",spaces.c_str());
		}
		return true;
	};
	ImGui::Text("interfaces");
	for(auto& i:m_interfaces) {
		ImGui::Text("    %s",i.c_str());
	}
	ImDrawList* dl=ImGui::GetWindowDrawList();
	//for(Device* device:m_devices) {
		Dict dict;
		GetDebugConnections(&dict);
		const Dict* connectionsDict=dict.Find("connections");
		if(!connectionsDict)
			return;
		for(auto it=connectionsDict->begin();it!=connectionsDict->end();++it) {
			const Dict* connectionDict=&*it;
			std::string id;
			connectionDict->Get("id",&id,"NA");
			const Dict* componentsDict=connectionDict->Find("components");
			if(!componentsDict || !componentsDict->Size()) {
				ImGui::Text("%s",id.c_str());
				continue;
			}
			auto itexpand=m_expandedComponents.find(id);
			bool expanded=itexpand!=m_expandedComponents.end() ? true:false;
			if(ImGui::Checkbox(id.c_str(),&expanded)) {
				if(expanded) {
					m_expandedComponents.insert(id);
				}else{
					m_expandedComponents.erase(itexpand);
				}
			}
			if(!expanded)
				continue;
			ImGui::Indent(32.0f);
			for(auto it1=componentsDict->begin();it1!=componentsDict->end();++it1) {
				const Dict* componentDict=&*it1;
				componentDict->Get("id",&id,"NA");
				const Dict* status=componentDict->Find("status");
				const Dict* settings=0;
				const Dict* p=status->First();
				while(p) {
					if(p->Name()!="schema" && p->Name()!="graphs") {
						settings=p;
						break;
					}
					p=p->Next();
				}
				if(m_selectedId==id) {
					ImVec2 p_min=ImGui::GetCursorScreenPos();
					ImVec2 p_max=ImVec2(p_min.x+ImGui::GetContentRegionAvail().x,p_min.y+ImGui::GetFrameHeight());
					const ImU32 col = ImGui::GetColorU32(ImGuiCol_Button);
					dl->AddRectFilled(p_min, p_max,col);
				}
				auto itexpand=m_expandedComponents.find(id);
				bool expanded=itexpand!=m_expandedComponents.end() ? true:false;
				std::string name=stdx::format_string("    %s",id.c_str());
				if(ImGui::Checkbox(name.c_str(),&expanded)) {
					SetStatusForConnection(id.c_str(),expanded);
					if(expanded) {
						m_expandedComponents.insert(id);
					}else{
						m_expandedComponents.erase(itexpand);
					}
					m_selectedId=id;
					settings=0;
				}
				if(expanded) {
					const Dict* d=settings;
					while(d) {											//Draw status json
						iterate(*d,0);
						d=d->Next();
					}
					std::string headerNameGraph="Graph###"+id;
					const Dict* graphs=status->Find("graphs");			//Draw graphs
					if(graphs)
						DrawGraphsJson(&m_hiddenDatasets,graphs);
				}
			}
			const Dict* log=connectionDict->Find("log");				//Draw log
			if(log) {
				ImGui::Text("Log");
				const Dict* entriesDict=log->Find("entries");
				std::string str;
				ImVec2 size=ImGui::GetContentRegionAvail();
				size.y=200;
				ImVec2 pmin=ImGui::GetCursorScreenPos();
				dl->AddRectFilled(pmin,ImVec2(pmin.x+size.x,pmin.y+size.y),0xff000000);
				//dl->AddRect(pmin,ImVec2(pmin.x+size.x,pmin.y+size.y),0xffffffff,0,0,2.0f);
				ImGui::BeginChild("test",size);
				for(auto it1=entriesDict->begin();it1!=entriesDict->end();++it1) {
					const Dict* entryDict=&*it1;
					entryDict->Get("str",&str);
					uint32_t color=0xc0c0c0ff;
					if(str.find("ERROR")!=std::string::npos) {
						color=0xdc3545ff;
					}else
					if(str.find("WARNING")!=std::string::npos) {
						color=0xffc107ff;
					}else
					if(str.find("NOTIFY")!=std::string::npos) {
						color=0x007bffff;
					}
					ImGui::TextColored(ImLoad(uint322V4(color)),"%s",str.c_str());
				}
				ImGui::EndChild();
			}
			ImGui::Unindent(32.0f);
		}
		if(!m_remoteAdapters.empty())
		{
			Dict graph_;
			Dict* graph=graph_.PushBack();
			uint64_t time=GetTimeEpochMicroseconds();
			uint64_t timeMilliseconds=time/1000L;
			graph->Set("xmax",(double)timeMilliseconds); // had to cast to double instead of int64_t to make DrawGraphsJson work
			graph->Set("xmin",(double)(timeMilliseconds-(20*1000)));
			graph->Set("formatX","HH:MM:SS");
			graph->Set("formatY","dBm");
			Dict* legend=graph->AddObjectNode("legend");
			legend->Set("text","RSSI");
			const char* colors[]={"#dd8452","#68a855","#524ec4","#b37281","#607893","#c38bda","#8c8c8c","#74b9cc","#cdb564","#b0724c"};
			legend->Set("color",colors[0]);
			Dict* datasets=graph->AddArrayNode("datasets");
			int cnt=0;
			for(AdaptorInfo* adaptor:m_remoteAdapters) {
				Dict* dataset=datasets->AddObjectNode();
				std::string name = adaptor->m_ip+":"+adaptor->m_name;
				dataset->Set("name",name.c_str());
				dataset->Set("color",colors[cnt++]);
				std::vector<double> datax;
				std::vector<double> datay;
				adaptor->m_rssi.Get(&datax,&datay,timeMilliseconds*1000);
				dataset->SetTypedArray("datax",datax.data(),datax.size());
				dataset->SetTypedArray("datay",datay.data(),datay.size());
			}
			DrawGraphsJson(&m_hiddenDatasets,&graph_);
		}
#if 0
		static uint64_t t1 = GetTimeEpochMicroseconds();
		uint64_t t2 = GetTimeEpochMicroseconds();
		static int dumpjson_count = 0;
		if(t2-t1 > 1000000L) {
			for(auto it = connectionsDict->begin(); it != connectionsDict->end(); ++it) {
				const Dict *connectionDict = &*it;
				std::string id;
				connectionDict->Get("id",&id,"NA");
				printf("connection %s\n", id.c_str());
				const Dict *componentsDict = connectionDict->Find("components");
				for(auto it1 = componentsDict->begin(); it1 != componentsDict->end(); ++it1) {
					const Dict *componentDict = &*it1;
					componentDict->Get("id", &id, "NA");
					const Dict *status = componentDict->Find("status");
					std::string dumpjson_str = status->WriteToJson();
					char dumpjson_fname[100];
					snprintf(dumpjson_fname, sizeof(dumpjson_fname), "/tmp/dump/dump_%s_%06d.json", id.c_str(), dumpjson_count);
					FILE *dumpjson_fid = fopen(dumpjson_fname, "w");
					fprintf(dumpjson_fid, "%s", dumpjson_str.c_str());
					fclose(dumpjson_fid);
					printf("dumping %s\n", dumpjson_fname);
				}
			}
			t1 = t2;
			dumpjson_count += 1;
		}
#endif
		//}
}

void MapColors(uint8_t* r, uint8_t* g, uint8_t* b, float value) {
	static float cmap[256][3] = {{0.18995,0.07176,0.23217},{0.19483,0.08339,0.26149},{0.19956,0.09498,0.29024},{0.20415,0.10652,0.31844},{0.20860,0.11802,0.34607},{0.21291,0.12947,0.37314},{0.21708,0.14087,0.39964},{0.22111,0.15223,0.42558},{0.22500,0.16354,0.45096},{0.22875,0.17481,0.47578},{0.23236,0.18603,0.50004},{0.23582,0.19720,0.52373},{0.23915,0.20833,0.54686},{0.24234,0.21941,0.56942},{0.24539,0.23044,0.59142},{0.24830,0.24143,0.61286},{0.25107,0.25237,0.63374},{0.25369,0.26327,0.65406},{0.25618,0.27412,0.67381},{0.25853,0.28492,0.69300},{0.26074,0.29568,0.71162},{0.26280,0.30639,0.72968},{0.26473,0.31706,0.74718},{0.26652,0.32768,0.76412},{0.26816,0.33825,0.78050},{0.26967,0.34878,0.79631},{0.27103,0.35926,0.81156},{0.27226,0.36970,0.82624},{0.27334,0.38008,0.84037},{0.27429,0.39043,0.85393},{0.27509,0.40072,0.86692},{0.27576,0.41097,0.87936},{0.27628,0.42118,0.89123},{0.27667,0.43134,0.90254},{0.27691,0.44145,0.91328},{0.27701,0.45152,0.92347},{0.27698,0.46153,0.93309},{0.27680,0.47151,0.94214},{0.27648,0.48144,0.95064},{0.27603,0.49132,0.95857},{0.27543,0.50115,0.96594},{0.27469,0.51094,0.97275},{0.27381,0.52069,0.97899},{0.27273,0.53040,0.98461},{0.27106,0.54015,0.98930},{0.26878,0.54995,0.99303},{0.26592,0.55979,0.99583},{0.26252,0.56967,0.99773},{0.25862,0.57958,0.99876},{0.25425,0.58950,0.99896},{0.24946,0.59943,0.99835},{0.24427,0.60937,0.99697},{0.23874,0.61931,0.99485},{0.23288,0.62923,0.99202},{0.22676,0.63913,0.98851},{0.22039,0.64901,0.98436},{0.21382,0.65886,0.97959},{0.20708,0.66866,0.97423},{0.20021,0.67842,0.96833},{0.19326,0.68812,0.96190},{0.18625,0.69775,0.95498},{0.17923,0.70732,0.94761},{0.17223,0.71680,0.93981},{0.16529,0.72620,0.93161},{0.15844,0.73551,0.92305},{0.15173,0.74472,0.91416},{0.14519,0.75381,0.90496},{0.13886,0.76279,0.89550},{0.13278,0.77165,0.88580},{0.12698,0.78037,0.87590},{0.12151,0.78896,0.86581},{0.11639,0.79740,0.85559},{0.11167,0.80569,0.84525},{0.10738,0.81381,0.83484},{0.10357,0.82177,0.82437},{0.10026,0.82955,0.81389},{0.09750,0.83714,0.80342},{0.09532,0.84455,0.79299},{0.09377,0.85175,0.78264},{0.09287,0.85875,0.77240},{0.09267,0.86554,0.76230},{0.09320,0.87211,0.75237},{0.09451,0.87844,0.74265},{0.09662,0.88454,0.73316},{0.09958,0.89040,0.72393},{0.10342,0.89600,0.71500},{0.10815,0.90142,0.70599},{0.11374,0.90673,0.69651},{0.12014,0.91193,0.68660},{0.12733,0.91701,0.67627},{0.13526,0.92197,0.66556},{0.14391,0.92680,0.65448},{0.15323,0.93151,0.64308},{0.16319,0.93609,0.63137},{0.17377,0.94053,0.61938},{0.18491,0.94484,0.60713},{0.19659,0.94901,0.59466},{0.20877,0.95304,0.58199},{0.22142,0.95692,0.56914},{0.23449,0.96065,0.55614},{0.24797,0.96423,0.54303},{0.26180,0.96765,0.52981},{0.27597,0.97092,0.51653},{0.29042,0.97403,0.50321},{0.30513,0.97697,0.48987},{0.32006,0.97974,0.47654},{0.33517,0.98234,0.46325},{0.35043,0.98477,0.45002},{0.36581,0.98702,0.43688},{0.38127,0.98909,0.42386},{0.39678,0.99098,0.41098},{0.41229,0.99268,0.39826},{0.42778,0.99419,0.38575},{0.44321,0.99551,0.37345},{0.45854,0.99663,0.36140},{0.47375,0.99755,0.34963},{0.48879,0.99828,0.33816},{0.50362,0.99879,0.32701},{0.51822,0.99910,0.31622},{0.53255,0.99919,0.30581},{0.54658,0.99907,0.29581},{0.56026,0.99873,0.28623},{0.57357,0.99817,0.27712},{0.58646,0.99739,0.26849},{0.59891,0.99638,0.26038},{0.61088,0.99514,0.25280},{0.62233,0.99366,0.24579},{0.63323,0.99195,0.23937},{0.64362,0.98999,0.23356},{0.65394,0.98775,0.22835},{0.66428,0.98524,0.22370},{0.67462,0.98246,0.21960},{0.68494,0.97941,0.21602},{0.69525,0.97610,0.21294},{0.70553,0.97255,0.21032},{0.71577,0.96875,0.20815},{0.72596,0.96470,0.20640},{0.73610,0.96043,0.20504},{0.74617,0.95593,0.20406},{0.75617,0.95121,0.20343},{0.76608,0.94627,0.20311},{0.77591,0.94113,0.20310},{0.78563,0.93579,0.20336},{0.79524,0.93025,0.20386},{0.80473,0.92452,0.20459},{0.81410,0.91861,0.20552},{0.82333,0.91253,0.20663},{0.83241,0.90627,0.20788},{0.84133,0.89986,0.20926},{0.85010,0.89328,0.21074},{0.85868,0.88655,0.21230},{0.86709,0.87968,0.21391},{0.87530,0.87267,0.21555},{0.88331,0.86553,0.21719},{0.89112,0.85826,0.21880},{0.89870,0.85087,0.22038},{0.90605,0.84337,0.22188},{0.91317,0.83576,0.22328},{0.92004,0.82806,0.22456},{0.92666,0.82025,0.22570},{0.93301,0.81236,0.22667},{0.93909,0.80439,0.22744},{0.94489,0.79634,0.22800},{0.95039,0.78823,0.22831},{0.95560,0.78005,0.22836},{0.96049,0.77181,0.22811},{0.96507,0.76352,0.22754},{0.96931,0.75519,0.22663},{0.97323,0.74682,0.22536},{0.97679,0.73842,0.22369},{0.98000,0.73000,0.22161},{0.98289,0.72140,0.21918},{0.98549,0.71250,0.21650},{0.98781,0.70330,0.21358},{0.98986,0.69382,0.21043},{0.99163,0.68408,0.20706},{0.99314,0.67408,0.20348},{0.99438,0.66386,0.19971},{0.99535,0.65341,0.19577},{0.99607,0.64277,0.19165},{0.99654,0.63193,0.18738},{0.99675,0.62093,0.18297},{0.99672,0.60977,0.17842},{0.99644,0.59846,0.17376},{0.99593,0.58703,0.16899},{0.99517,0.57549,0.16412},{0.99419,0.56386,0.15918},{0.99297,0.55214,0.15417},{0.99153,0.54036,0.14910},{0.98987,0.52854,0.14398},{0.98799,0.51667,0.13883},{0.98590,0.50479,0.13367},{0.98360,0.49291,0.12849},{0.98108,0.48104,0.12332},{0.97837,0.46920,0.11817},{0.97545,0.45740,0.11305},{0.97234,0.44565,0.10797},{0.96904,0.43399,0.10294},{0.96555,0.42241,0.09798},{0.96187,0.41093,0.09310},{0.95801,0.39958,0.08831},{0.95398,0.38836,0.08362},{0.94977,0.37729,0.07905},{0.94538,0.36638,0.07461},{0.94084,0.35566,0.07031},{0.93612,0.34513,0.06616},{0.93125,0.33482,0.06218},{0.92623,0.32473,0.05837},{0.92105,0.31489,0.05475},{0.91572,0.30530,0.05134},{0.91024,0.29599,0.04814},{0.90463,0.28696,0.04516},{0.89888,0.27824,0.04243},{0.89298,0.26981,0.03993},{0.88691,0.26152,0.03753},{0.88066,0.25334,0.03521},{0.87422,0.24526,0.03297},{0.86760,0.23730,0.03082},{0.86079,0.22945,0.02875},{0.85380,0.22170,0.02677},{0.84662,0.21407,0.02487},{0.83926,0.20654,0.02305},{0.83172,0.19912,0.02131},{0.82399,0.19182,0.01966},{0.81608,0.18462,0.01809},{0.80799,0.17753,0.01660},{0.79971,0.17055,0.01520},{0.79125,0.16368,0.01387},{0.78260,0.15693,0.01264},{0.77377,0.15028,0.01148},{0.76476,0.14374,0.01041},{0.75556,0.13731,0.00942},{0.74617,0.13098,0.00851},{0.73661,0.12477,0.00769},{0.72686,0.11867,0.00695},{0.71692,0.11268,0.00629},{0.70680,0.10680,0.00571},{0.69650,0.10102,0.00522},{0.68602,0.09536,0.00481},{0.67535,0.08980,0.00449},{0.66449,0.08436,0.00424},{0.65345,0.07902,0.00408},{0.64223,0.07380,0.00401},{0.63082,0.06868,0.00401},{0.61923,0.06367,0.00410},{0.60746,0.05878,0.00427},{0.59550,0.05399,0.00453},{0.58336,0.04931,0.00486},{0.57103,0.04474,0.00529},{0.55852,0.04028,0.00579},{0.54583,0.03593,0.00638},{0.53295,0.03169,0.00705},{0.51989,0.02756,0.00780},{0.50664,0.02354,0.00863},{0.49321,0.01963,0.00955},{0.47960,0.01583,0.01055}};
	int lo=std::min(255,std::max(0,int(255*value)));
	int hi=std::min(255,lo+1);
	float f=255.0f*value-lo;
	*r=(uint8_t)(255*(cmap[lo][0]+f*(cmap[hi][0]-cmap[lo][0])));
	*g=(uint8_t)(255*(cmap[lo][1]+f*(cmap[hi][1]-cmap[lo][1])));
	*b=(uint8_t)(255*(cmap[lo][2]+f*(cmap[hi][2]-cmap[lo][2])));
}

void Viewer::CopyPixels(const char* clientId,const Tensor* dec,const DecodedFrameInfo* info) {
	const int streamId=info->streamId;
	const int index=info->frameIndex;
	auto it=m_idToDisplayClient.find(clientId);
	if(it==m_idToDisplayClient.end()) {
		//FATAL("client %s not found",clientId);
		return;
	}
	DisplayClient::VideoFrame* vf=0;
	DisplayClient* dc=it->second;
	dc->m_lockFrame.lock();
	for(int i=0;i!=(int)dc->m_videoFrames.size();i++) {
		if(dc->m_videoFrames[i].m_streamId==streamId) {
			vf=&dc->m_videoFrames[i];
			break;
		}
	}
	//uprintf("client %s decoded %d\n",clientId,index);
	if(!vf) {
		dc->m_videoFrames.emplace_back();
		vf=&dc->m_videoFrames.back();
		vf->m_streamId=streamId;
	}

	if(dec->m_dimensions==3 && dec->m_elementSize==1 && dec->m_type==Tensor::AllocatorType::ALLOC_CPU) {
		// Assume this is uint8 RGB(A) data with
		const int height=dec->m_shape[0];
		const int width=dec->m_shape[1];
		const int channels=dec->m_shape[2];
		if(width!=vf->m_width||height!=vf->m_height||channels!=vf->m_channels) {
			delete[] vf->m_pixelsFrame;
			vf->m_pixelsFrame=0;
		}

		if(!vf->m_pixelsFrame)
			vf->m_pixelsFrame=new uint8_t[width*height*channels];
		vf->m_width=width;
		vf->m_height=height;
		vf->m_channels=channels;
		vf->m_indexPixels=index;
		memcpy(vf->m_pixelsFrame,dec->m_data,dec->ByteSize());
	}else
	if(dec->m_dimensions==2 && dec->m_elementSize==4 && dec->m_type==Tensor::AllocatorType::ALLOC_CPU) {
		// Assume this is ToF data with float range 0-4
		const int height=dec->m_shape[0];
		const int width=dec->m_shape[1];
		const int channels=3;
		if(!vf->m_pixelsFrame)
			vf->m_pixelsFrame=new uint8_t[height*width*channels];
		vf->m_width=width;
		vf->m_height=height;
		vf->m_channels=channels;
		vf->m_indexPixels=index;
		const float* val=(const float*)dec->m_data;
		for(int i=0;i<width*height;++i) {
			float f=CLAMP(0.0f,1.0f-val[i]/4.0f,1.0f);
			MapColors(vf->m_pixelsFrame+3*i+0,vf->m_pixelsFrame+3*i+1,vf->m_pixelsFrame+3*i+2,f);
		}
	}else
	if(dec->m_dimensions>=2) {
		const int height=dec->m_shape[0];
		const int width=dec->m_shape[1];
		const int channels=3;
		if(!vf->m_pixelsFrame)
			vf->m_pixelsFrame=new uint8_t[height*width*channels];
		vf->m_width=width;
		vf->m_height=height;
		vf->m_channels=channels;
		vf->m_indexPixels=index;
		for(int i=0;i<width*height;++i) {
			vf->m_pixelsFrame[3*i+0]=0;
			vf->m_pixelsFrame[3*i+1]=0;
			vf->m_pixelsFrame[3*i+2]=index&0xff;
		}
	}else
	{
		uprintf("unhandled tensor format: dim=%d, esize=%d, atype=%d, dims=%d\n", dec->m_dimensions, dec->m_elementSize, dec->m_type, dec->m_dimensions);
	}

	//if(vf->m_timeCapture)
	//	vf->m_fps=(float)((double)1000000/(double)((int64_t)timeCapture-(int64_t)vf->m_timeCapture));

	dc->m_lockFrame.unlock();
}

void Viewer::MainLoopDisplay() {
	int tick=0;
	std::vector<DisplayClient*> clients;
	{
		GetEthernetAdapterIPv4Adresses(&m_interfaces);
		for(auto& i:m_interfaces) {
			uprintf("Interface %s\n",i.c_str());
		}
		std::vector<std::string> clientsId=GetDisplayClients();
		for(const std::string& id:clientsId) {
			auto it=m_idToDisplayClient.find(id);
			if(it==m_idToDisplayClient.end()) {
				DisplayClient* dc=new DisplayClient();
				dc->m_id=id;
				m_idToDisplayClient.insert(std::make_pair(id,dc));
				uprintf("id :%s\n", id.c_str());
				clients.push_back(dc);
			}
		}
		SetFrameDecodedCallback(this,
			[this](Dict& frame,const void* data,int dataBytesize,void* arg){
				std::string format;
				frame.Get("streamFormat",&format);
				std::string name;
				frame.Get("name",&name);
				int streamId=0;
				if(!frame.Get("id",&streamId,0xff)) {
					uprintf("WARNING: Unable to get stream id\n");
					return;
				}
				std::scoped_lock sl(m_displayDevicesLock);
				auto it=m_idToDisplayClient.find(name.c_str());
				if(it==m_idToDisplayClient.end()) {
					uprintf("Unable to find stream %s\n",name.c_str());
					frame.Dump();
					return;
				}
				DisplayClient* dc=it->second;
				if(format=="json") {
						dc->m_lockFrame.lock();
						dc->m_streams[streamId].m_info.Copy(frame);
						dc->m_lockFrame.unlock();
						return;
				}
				std::string dataSourceType;
				frame.Get("dataSourceType",&dataSourceType,"NA");
				uprintf("WARNING: Unsupported data type %s stream format %s\n",dataSourceType.c_str(),format.c_str());
			},
			[this](const char* clientId,uint8_t streamId,Dict& streamInfo,std::vector<Tensor>& tensors,void* arg) {
//				streamInfo.Dump();
				const Dict* source=streamInfo.Find("source");
				const Dict* frames=source ? source->Find("frames"):0;

				if(!source || !frames) {
					std::scoped_lock sl(m_displayDevicesLock);
					auto it=m_idToDisplayClient.find(clientId);
					if(it==m_idToDisplayClient.end()) {
						uprintf("Unable to find stream %s\n",clientId);
						return;
					}
					DisplayClient* dc=it->second;
					dc->m_lockFrame.lock();
					dc->m_streams[streamId].m_info.Copy(streamInfo);
					dc->m_lockFrame.unlock();
					return;
				}
				std::vector<DecodedFrameInfo> decodedFrameInfos;
				for(size_t i=0;i<frames->Size();++i){
					const Dict& cam=frames->At(i);
					int streamId2,frameIndex;
					cam.Get("id",&streamId2);
					cam.Get("frameIndex",&frameIndex);
					int64_t captureTime;
					cam.Get("captureTime",&captureTime);
					decodedFrameInfos.push_back({streamId2,frameIndex,(uint64_t)captureTime});
				}

				std::scoped_lock sl(m_displayDevicesLock);
				for(size_t i=0;i<tensors.size();++i) {
					if(!tensors[i].m_data)
						continue;
					CopyPixels(clientId,&tensors[i],&decodedFrameInfos[i]);
				}
			}
		);
	}
	while(true) {
		if(glfwWindowShouldClose(m_window)) {
			PostEvent(App::EV_EXIT);
			break;
		}
		if(glfwGetKey(m_window,GLFW_KEY_ESCAPE)==GLFW_PRESS)
			glfwSetWindowShouldClose(m_window,true);

		auto time=std::chrono::system_clock::now();
		glfwPollEvents();

		PollEvents(false);

		if(ShouldExitMainLoop())
			break;
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		ImGuiID dockspace_id=ImGui::GetID("EditDockSpace");
		if(!tick || ImGui::DockBuilderGetNode(dockspace_id)==NULL)	{
			ImGui::DockBuilderRemoveNode(dockspace_id);
			ImVec2 dockspace_size=ImGui::GetMainViewport()->Size;
			ImGui::DockBuilderAddNode(dockspace_id,ImGuiDockNodeFlags_DockSpace);
			ImGui::DockBuilderSetNodeSize(dockspace_id,dockspace_size);
			if(!clients.size()) {
				ImGuiID dock_prop_id=dockspace_id;
				ImGui::DockBuilderDockWindow("Status",dock_prop_id);
			}else{
				ImGuiID dock_video_id=dockspace_id;
				ImGuiID dock_prop_id=ImGui::DockBuilderSplitNode(dock_video_id,ImGuiDir_Left,0.50f,NULL,&dock_video_id);
				ImGuiID dock_status_id=ImGui::DockBuilderSplitNode(dock_video_id,ImGuiDir_Down,0.35f,NULL,&dock_video_id);
				ImGui::DockBuilderDockWindow("Status",dock_prop_id);
				ImGui::DockBuilderDockWindow("Clients",dock_video_id);
				ImGui::DockBuilderDockWindow("Settings",dock_status_id);
			}
			ImGui::DockBuilderFinish(dockspace_id);
		}

		ImGuiViewport* viewport=ImGui::GetMainViewport();
		const ImGuiWindowClass* window_class=0;
		ImVec2 p=viewport->Pos;
		ImVec2 s=viewport->Size;

		ImGui::SetNextWindowPos(p);
		ImGui::SetNextWindowSize(s);
		ImGui::SetNextWindowViewport(viewport->ID);
		ImGui::SetNextWindowBgAlpha(0);

		ImGuiDockNodeFlags dockspace_flags=0;
		ImGuiWindowFlags host_window_flags=0;
		host_window_flags|=ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoDocking|ImGuiWindowFlags_NoBackground;
		host_window_flags|=ImGuiWindowFlags_NoBringToFrontOnFocus|ImGuiWindowFlags_NoNavFocus|ImGuiWindowFlags_MenuBar;

		ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize,0.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,ImVec2(0.0f,0.0f));
		ImGui::Begin("MainWindow",NULL,host_window_flags);
		ImGui::DockSpace(dockspace_id,ImVec2(0.0f,0.0f),dockspace_flags,window_class);
		ImGui::PopStyleVar(3);

		if(clients.size()) {
			ImGui::Begin("Clients");
			DrawVideo(clients);
			ImGui::End();
			ImGui::Begin("Settings");
			DrawSettings(clients);
			ImGui::End();
		}
		ImGui::Begin("Status");
		DrawStatus(tick);
		ImGui::End();

		ImGui::End();
		ImGui::Render();

		int windowWidth,windowHeight;
		glfwGetWindowSize(m_window,&windowWidth,&windowHeight);

		glViewport(0,0,windowWidth,windowHeight);
		glClearColor(.1,.1,.1,1);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		ImGuiIO& io=ImGui::GetIO(); (void)io;
		if(io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
			GLFWwindow* backup_current_context=glfwGetCurrentContext();
			ImGui::UpdatePlatformWindows();
			ImGui::RenderPlatformWindowsDefault();
			glfwMakeContextCurrent(backup_current_context);
		}
		std::this_thread::sleep_until(time+std::chrono::microseconds(1000000/60));		//limit frame rate
		glfwSwapBuffers(m_window);
		tick++;
	}
	m_displayDevicesLock.lock();
	ClearFrameDecodedCallback();
	ClearFrameEncodedCallback();
	for(auto [k,dc]:m_idToDisplayClient) {
		for(DisplayClient::VideoFrame& vf:dc->m_videoFrames) {
			if(vf.m_pixelsFrame)
				delete [] vf.m_pixelsFrame;
		}
		delete dc;
	}
	m_idToDisplayClient.clear();
	while(!ShouldExitMainLoop()) {
		PollEvents(true);
	}
	m_displayDevicesLock.unlock();
	for(AdaptorInfo* ai:m_remoteAdapters) {
		delete ai;
	}
	m_remoteAdapters.clear();
}

void Viewer::CloseDisplay() {
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwDestroyWindow(m_window);
	glfwTerminate();
}

bool Viewer::InitDisplay(bool large) {
	//const char* glsl_version="#version 130";
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
	const char* glsl_version="#version 300 es";
	// Setup window
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit()) {
		uprintf("glfwInit failed\n");
		return false;
	}
#if __APPLE__
    glsl_version="#version 330";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, true);
#else
	glfwWindowHint(GLFW_CLIENT_API,GLFW_OPENGL_ES_API);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,0);
#endif
	if(large) {
		m_window=glfwCreateWindow(1920,1280,"OpenGL ES Stream viewer",NULL,NULL);
	}else{
		m_window=glfwCreateWindow(640,480,"OpenGL ES Video server",NULL,NULL);
	}
	if(!m_window) {
		uprintf("Unable to create window\n");
		return false;
	}
	glfwMakeContextCurrent(m_window);
	glfwSwapInterval(1); // Enable vsync
	bool err=glewInit()!=GLEW_OK;
	if(err) {
		FATAL("Failed to initialize OpenGL loader!");
		return false;
	}
	uprintf("glsl_version %s\n",glsl_version);
	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io=ImGui::GetIO(); (void)io;
	io.ConfigFlags |=ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls
	//io.ConfigFlags |=ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
	io.ConfigFlags |=ImGuiConfigFlags_DockingEnable;           // Enable Docking
	io.ConfigFlags |=ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();
	// When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
	ImGuiStyle& style=ImGui::GetStyle();
	if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
		style.WindowRounding=0.0f;
		style.Colors[ImGuiCol_WindowBg].w=1.0f;
	}
	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(m_window,true);
	ImGui_ImplOpenGL3_Init(glsl_version);
	//io.Fonts->AddFontFromFileTTF(GetFileNameRemap("$(DATA)/fonts/inconsolata/InconsolataGo-Regular.ttf").c_str(),20.0f);
	//io.Fonts->AddFontDefault();
	io.Fonts->AddFontFromFileTTF(GetFileNameRemap("$(DATA)/fonts/roboto/Roboto-Medium.ttf").c_str(),20.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf",15.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf",16.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf",10.0f);
	return true;
}

int main(int argc, char *argv[]) {
	//void TestDict(); TestDict();

	if(DirectoryExists(GetExecutableDir()+"/data")) {
		AddFilePathRemap("$(DATA)",GetExecutableDir()+"/data");
	}else{
#ifdef CMAKE_SOURCE_DIR
		AddFilePathRemap("$(DATA)",std::string(CMAKE_SOURCE_DIR)+"/data");
#else
		AddFilePathRemap("$(DATA)",GetExecutableDir()+"/data");
#endif
	}
	AddFilePathRemap("$(EXE)",GetExecutableDir());
#ifdef WIN32
	if(DirectoryExists("d:/")) {
		AddFilePathRemap("$(HOME)","d:/");
	}else{
		AddFilePathRemap("$(HOME)",getenv("HOMEPATH"));
	}
#else
	AddFilePathRemap("$(HOME)",getenv("HOME"));
#endif

	const char* opts="s:";
	for(int i=1;i<argc;i++) {
		if(argv[i][0]!='-')
			continue;
		char opt=argv[i][1];
		const char* p=strchr(opts,opt);
		if(!p)
			continue;
		char* optarg=0;
		if(p[1]==':') {
			optarg=argv[++i];
			while(optarg[0]==' ')
				optarg++;
		}
		switch(opt) {
			case 's':
				uprintf("arg s=%d\n",atoi(optarg));
				break;
			default:
				fprintf(stderr,"Usage: %s [-s test]\n",argv[0]);
				exit(EXIT_FAILURE);
		}
	}
	std::string configFileName;
	if(argc>1) {
		configFileName=argv[argc-1];
	}else{
		configFileName="$(HOME)/config.json";
	}
	uprintf("exe directory %s\n",GetFileNameRemap("$(EXE)").c_str());
	uprintf("data directory %s\n",GetFileNameRemap("$(DATA)").c_str());

	Viewer app;
	return app.Run(configFileName);
}


#ifdef _WIN32
#undef APIENTRY
#include<windows.h>
#include "debugapi.h"
#include<crtdbg.h>

int __stdcall WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, char*, int nShowCmd) {
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	//_CrtSetBreakAlloc(13692);
    return main(__argc, __argv);
}
#endif
