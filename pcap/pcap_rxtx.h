#pragma once
#include <stdint.h>

class PCAPSender {
public:
	virtual ~PCAPSender(){}
	virtual bool Send(const void* data,int dataByteSize)=0;
	virtual void SetId(uint8_t id)=0;
};

PCAPSender* CreatePCAPSender(const char* interfaceName,uint8_t id);
void DestroyPCAPSender(PCAPSender* sender);

struct ReceivedDataInfo {
	uint64_t m_time;
	int m_rssi;
};
typedef void(*recvcallback)(const uint8_t* data,int dataBytesize,const ReceivedDataInfo& recoverInfo,const void* arg);

class PCAPReceiver {
public:
	virtual ~PCAPReceiver(){}
	virtual void End()=0;
	virtual bool WaitForData(recvcallback cb,void* arg)=0;
	virtual void SetId(uint8_t id)=0;
};

PCAPReceiver* CreatePCAPReceiver(const char* interfaceName,uint8_t id);
void DestroyPCAPReceiver(PCAPReceiver* receiver);
