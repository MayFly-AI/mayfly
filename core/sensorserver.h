#pragma once

#include "service.h"

class SensorClient;

typedef std::function<void(const char* id,const void* frame,int frameBytesize,int index,uint64_t timeCapture,void* arg)> TFrameEncodedCallbackFunc;

class SensorServer : public Service {
	public:
		virtual ~SensorServer(){}
		virtual eType Type(){return VIDEO_SERVER;}
		virtual void SetFrameEncodedCallback(void* arg,TFrameEncodedCallbackFunc cb)=0;
		virtual void GetStatus(Dict* status,bool includeSchema=true,bool includeGraphs=true)=0;
		virtual bool ConnectToClient(SensorClient* videoClient)=0;
		virtual void GetProperties(Dict* properties)=0;
};
SensorServer* CreateSensorServer(Dict* dict);
void DestroySensorServer(SensorServer* sensorServer);
