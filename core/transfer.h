#pragma once

#include <deque>
#include <map>
#include <stdint.h>
#include <functional>
#include "shared/net.h"
#include "shared/queue.h"
#include "history.h"

class Dict;
class NetTransfer;

typedef std::function<void(NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t time)> TDataCallbackFunc;
typedef std::function<void(NetTransfer* transfer,const char* sourceName,const char* errorName)> TErrorCallbackFunc;

class NetTransfer {
	public:
		virtual ~NetTransfer(){}
		virtual bool IsDestBroadcast()const=0;
		virtual void GetStatus(Dict* status)=0;
		virtual std::string Host()const=0;
		virtual uint8_t HostIndex()const=0;		//This is the host part of the bind address
		void SetDataCallback(TDataCallbackFunc cb){m_dataCallback=cb;}
		void SetErrorCallback(TErrorCallbackFunc cb){m_errorCallback=cb;}
		virtual void Begin(const Dict& settings)=0;
		virtual void End()=0;

		virtual int SendTo(const char* addressName,const void* data,int len)=0;
		virtual int SendToHost(const void* data,int len)=0;
		virtual int PreferedPacketBytesize()const=0;
		TDataCallbackFunc m_dataCallback=0;
		TErrorCallbackFunc m_errorCallback=0;

		//Statistics

		virtual void GetGraphs(Dict* graphs,uint64_t timeMilliseconds);

		History m_received;
		History m_send;

		volatile int m_packetsReceived=0;
		volatile int m_packetsSend=0;
		volatile int m_bytesReceived=0;
		volatile int m_bytesSend=0;
};

NetTransfer* CreateNetTransfer(const Dict* settings);
void DestroyNetTransfer(NetTransfer* nt);
