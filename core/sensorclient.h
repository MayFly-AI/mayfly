#pragma once

#include <string>
#include "shared/dict.h"
#include "service.h"
#include "packet.h"

typedef std::function<void(uint8_t hostIndex,uint8_t streamId,uint8_t dataSourceType,const uint8_t* frame,int frameByteSize,int index,uint64_t timeCapture,const DebugData& debugData,void* arg)> TFrameCallbackFunc;
typedef std::function<void(int frameIndex, uint8_t streamId, uint8_t dataSourceType, class NetTransfer* transfer, void* arg)> TLastMessageCallbackFunc;

#define MAX_NUMBER_HOSTS 16

class SensorClient : public Service {
	public:
		virtual eType Type(){return VIDEO_CLIENT;}
		virtual ~SensorClient(){}
		virtual void AddFrameCallback(void* arg,TFrameCallbackFunc cb)=0;
		virtual void AddLastMessageCallback(void* arg,TLastMessageCallbackFunc cb)=0;
		virtual void GetStatus(Dict* status,bool includeSchema=true,bool includeGraphs=true)=0;
		virtual void GetProperties(Dict* properties)=0;
};

SensorClient* CreateSensorClient(Dict* dict);
void DestroySensorClient(SensorClient* sensorClient);
