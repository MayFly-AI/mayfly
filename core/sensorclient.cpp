#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <map>
#include <chrono>
#include <thread>
#include <assert.h>
#include <algorithm>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"
#include "shared/crc32.h"
#include "shared/thread.h"

#include "app.h"
#include "transfer.h"
#include "packet.h"
#include "errorcorrection.h"
#include "sensorclient.h"
#include "datasource.h"

class TimeSync {
	public:
		void RegisterRoundtrip(uint64_t timeSendServer,uint64_t timeReceived,uint64_t timeLoopback,uint32_t timeSpendServer);
		int64_t m_deltaTimeServer=0;
		int64_t m_deltaTime=0;
		uint32_t m_deltaRoundTripTime=0;
		std::mutex m_roundtripLock;
		struct LatencyInfo {
			uint64_t m_time;
			int m_roundtripMicroseconds;
		};
		std::deque<LatencyInfo> m_roundTrips;
		void AddRoundTrip(uint64_t time,int roundtripMicroseconds){
			std::scoped_lock sl(m_roundtripLock);
			uint64_t timeNow=GetTimeEpochMicroseconds();
			while(m_roundTrips.size() && timeNow-m_roundTrips.front().m_time>HISTORY_AGE)
				m_roundTrips.pop_front();
			m_roundTrips.push_back({time,roundtripMicroseconds});
		}
};

void TimeSync::RegisterRoundtrip(uint64_t timeSendServer,uint64_t timeReceived,uint64_t timeLoopback,uint32_t timeSpendServer) {
	uint32_t rt=(uint32_t)(timeReceived-timeLoopback)-timeSpendServer;		//roundtrip=receivetime-sendtime
	uint64_t estimateTimeServerNow=timeSendServer+(rt/2);					//time server now=time server send + roundtrip/2
	if(!m_deltaRoundTripTime) {
		m_deltaRoundTripTime=rt;
		m_deltaTime=timeReceived;
		m_deltaTimeServer=(int64_t)(estimateTimeServerNow-timeReceived);
	}
	AddRoundTrip(timeReceived,(int)rt);
}

class ErrorCorrectionDecoders {
	public:
		~ErrorCorrectionDecoders() {
			for(int i=0;i!=countof(m_decoders);i++) {
				if(m_decoders[i])
					DestroyErrorCorrectionDecoder(m_decoders[i]);
				m_decoders[i]=0;
			}
		}
		uint8_t GetStreamIdFromIndex(uint8_t index) {
			return index/EC_TYPE_MAX;
		}
		ErrorCorrectionDecoder* Get(uint8_t streamId,uint8_t errorCorrectionType) {
			if(!m_decoders[streamId*EC_TYPE_MAX+errorCorrectionType]) {
				//uprintf("Created error correction decoder %s for stream id %d\n",ErrorCorrectionTypeToName(errorCorrectionType),streamId);
				m_decoders[streamId*EC_TYPE_MAX+errorCorrectionType]=CreateErrorCorrectionDecoder(errorCorrectionType);
			}
			return m_decoders[streamId*EC_TYPE_MAX+errorCorrectionType];
		}
		ErrorCorrectionDecoder* m_decoders[STREAM_ID_MAX*EC_TYPE_MAX]={0};
};

class SensorClientImpl : public SensorClient {
	public:
		virtual ~SensorClientImpl(){}
		virtual bool SaveConfig(Dict* config);
		virtual void GetStatus(Dict* status,bool includeSchema,bool includeGraphs);
		virtual const std::string& Id()const{return m_id;}
		virtual void AddFrameCallback(void* arg,TFrameCallbackFunc cb){m_frameCallbacks.push_back({cb,arg});}
		virtual void AddLastMessageCallback(void* arg,TLastMessageCallbackFunc cb){m_lastMessageCallbacks.push_back({cb,arg});}
		virtual void GetProperties(Dict* properties);
		virtual void SetProperties(const Dict& properties);
		bool Begin(Dict* dict);
		void End();

		void OnData(NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived);
		struct Frame {
			uint8_t m_hostIndex;
			uint8_t m_dataSourceType:DS_TYPE_BITS;
			uint8_t m_streamId:STREAM_ID_BITS;
			int m_index;
			std::vector<uint8_t> m_data;
			DebugData m_debugData;
			uint64_t m_timeCapture;
			uint64_t m_firstTime;
		};
		FixedQueueMT<Frame,8> m_frames;
	protected:
		std::string m_id;
		int m_frameIndex=0;

		std::atomic<bool> m_close;
		std::thread m_threadSend;
		bool m_verbose=false;
		struct FrameCallback {
			TFrameCallbackFunc m_callback;
			void* m_arg;
		};
		struct LastMessageCallback {
			TLastMessageCallbackFunc m_callback;
			void* m_arg;
		};
		std::vector<FrameCallback> m_frameCallbacks;
		std::vector<LastMessageCallback> m_lastMessageCallbacks;
		struct ActiveHost {
			NetTransfer* m_transfer=0;
			std::string m_sourceName;
			uint8_t m_index=0;
			int32_t m_lastFrameReceived[STREAM_ID_MAX]={0};
			uint64_t m_lastRequestTime=0;
			uint32_t m_pingCount=0;
			uint64_t m_lastPingTime=0;
			TimeSync m_timeSync;
			bool SendRequestFrames(uint64_t delay);
			bool SendPing(uint64_t delay);
		};
		mutable std::mutex m_activeHostsLock;
		std::map<std::string,ActiveHost*> m_activeHosts;
		volatile int m_currentFrameIndex[STREAM_ID_MAX]={0};
		volatile ActiveHost* m_currentFrameHost[STREAM_ID_MAX]={0};
		ErrorCorrectionDecoders m_errorCorrection;
		ErrorCorrectionDecoder::Statistics m_statistics;
		uint64_t m_timeLastStatistics=0;
};

void SensorClientImpl::OnData(NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived) {
	//uprintf("SensorClientImpl::OnData len %d from %s\n",dataBytesize,sourceName);
	PacketHeaderBase* header=(PacketHeaderBase*)data;
	header->SanityCheck(dataBytesize);
	if(header->IsDebugPacket()) {
		uprintf("Warning: SensorClientImpl::OnData received debug packet\n");
		return;
	}
	//Connectionless packets
	m_activeHostsLock.lock();
	auto it=m_activeHosts.find(sourceName);
	if(it==m_activeHosts.end()) {
		uprintf("client %s received data from unregistered host %s\n",Id().c_str(),sourceName);
		m_activeHostsLock.unlock();
		return;
	}
	ActiveHost* ah=&*it->second;
	if(header->m_type==PacketHeaderBase::FROM_SERVER_PONG) {
		PongFrameHeader* pfh=(PongFrameHeader*)header;
		//uprintf("Client received pong %d. Roundtrip transfer %dus\n",pfh->m_count,(timeReceived-pfh->m_timeLoopback)-pfh->m_timeSpendServer);
		ah->m_timeSync.RegisterRoundtrip(pfh->m_timeSend,timeReceived,pfh->m_timeLoopback,pfh->m_timeSpendServer);
		m_activeHostsLock.unlock();
		return;
	}
	int hostIndex=ah->m_index;
	ErrorCorrectionDecoder* ecd=0;
	if(header->m_type==PacketHeaderBase::FROM_SERVER_FRAME_CHUNK || header->m_type==PacketHeaderBase::FROM_SERVER_FRAME_CHUNK_RESEND) {
		PacketHeaderFrame* phf=(PacketHeaderFrame*)header;
		if(phf->m_errorCorrectionType!=EC_TYPE_FEC && phf->m_errorCorrectionType!=EC_TYPE_ARQ)
			FATAL("error correction");

		ecd=m_errorCorrection.Get(phf->m_streamId,phf->m_errorCorrectionType);
		int32_t frameIndex=ecd->GetPacketDataIndex(phf+1,dataBytesize-sizeof(PacketHeaderBase));
		if(m_currentFrameIndex[phf->m_streamId]<frameIndex) {
			m_currentFrameIndex[phf->m_streamId]=frameIndex;
			m_currentFrameHost[phf->m_streamId]=ah;
		}else{
			if(m_currentFrameHost[phf->m_streamId]!=ah) {
				m_activeHostsLock.unlock();
				return;
			}
		}
		if(frameIndex<ah->m_lastFrameReceived[phf->m_streamId]-10) {
			bool wrap=frameIndex==0 && ah->m_lastFrameReceived[phf->m_streamId]==0xffff;
			if(wrap){
				uprintf("Wrap around frameIndex\n");
			}else
			{
				uprintf("Received frame with much lower index: %d < %d. Server restart?\n", frameIndex, ah->m_lastFrameReceived[phf->m_streamId]);
				ecd->Reset();
				ah->m_lastFrameReceived[phf->m_streamId]=0;
			}
		}
		ah->m_lastFrameReceived[phf->m_streamId]=frameIndex;

		if(phf->m_type==PacketHeaderBase::FROM_SERVER_FRAME_CHUNK) {
			ecd->RequestRetransmit(phf->m_streamId,phf->m_dataSourceType,phf+1,dataBytesize,timeReceived,[&](const uint8_t* data,int dataBytesize){
				//uprintf("retransmit\n");
				ah->m_transfer->SendToHost(data,dataBytesize);
			});
		}
	}
	m_activeHostsLock.unlock();
	switch(header->m_type) {
		case PacketHeaderBase::FROM_SERVER_FRAME_CHUNK_RESEND:
		case PacketHeaderBase::FROM_SERVER_FRAME_CHUNK: {
			PacketHeaderFrame* phf=(PacketHeaderFrame*)header;
			if(phf->m_type==PacketHeaderBase::FROM_SERVER_FRAME_CHUNK_RESEND) {
				uprintf("Received retransmit\n");
			}
			ecd->Decode(phf+1,dataBytesize-(int)sizeof(PacketHeaderFrame),timeReceived,[&](int index,const uint8_t* data,int dataBytesize,const DebugData& debugData,uint64_t timeFirst){
				if(!DataSourceTypeValid(phf->m_dataSourceType))
					FATAL("SensorClientImpl::OnData m_dataSourceType %d not valid",phf->m_dataSourceType);
				//uprintf("Client %s Got all chunks for streamId %d frame %d dataSourceType %s\n",Id().c_str(),phf->m_streamId,index,StreamTypeToName(phf->m_dataSourceType));
				if(!m_frames.Push([&](Frame* frame){
					frame->m_hostIndex=hostIndex;
					frame->m_debugData=debugData;
					frame->m_timeCapture=debugData.m_timeCapture;
					frame->m_dataSourceType=phf->m_dataSourceType;
					frame->m_streamId=phf->m_streamId;
					frame->m_index=index;
					frame->m_data.resize(dataBytesize);
					frame->m_firstTime=timeFirst;
					memcpy(frame->m_data.data(),data,dataBytesize);
				})){
					uprintf("SensorClientImpl::OnData frames queue full\n");
				}
			});
			if(phf->m_last) {
				int32_t frameIndex=ecd->GetPacketDataIndex(phf+1,dataBytesize-sizeof(PacketHeaderFrame));
				//uprintf("last %s frameIndex %d\n",Id().c_str(),frameIndex);
				uint8_t hostSendIndex=frameIndex&(MAX_NUMBER_HOSTS-1);
				m_activeHostsLock.lock();
				for(auto const& [k,v]:m_activeHosts) {
					uint8_t hostIndex=v->m_transfer->HostIndex()&(MAX_NUMBER_HOSTS-1);
					if(hostIndex!=hostSendIndex)
						continue;
					if(v->SendRequestFrames(10*1000000L)) {
						//uprintf("client %s send request frames from receive data. Host index %d\n",Id().c_str(),hostIndex);
					}
					if(v->SendPing(1*1000000L)) {
						//uprintf("send ping from receive data. Host index %d\n",hostIndex);
					}
				}
				m_activeHostsLock.unlock();
				for(auto& cb:m_lastMessageCallbacks) {
					cb.m_callback(frameIndex,phf->m_streamId,phf->m_dataSourceType,transfer,cb.m_arg);
				}
			}
			if(m_verbose) {
				if(timeReceived-m_timeLastStatistics>5*1000000L) {
					std::string s=ecd->GetStatistics(&m_statistics);
					if(m_timeLastStatistics)
						uprintf("Decoder: %s\n",s.c_str());
					m_timeLastStatistics=timeReceived;
				}
			}
			break;
		}
		default: {
			uprintf("SensorClientImpl::OnData received unhandled packet type %d\n",header->m_type);
		}
	}
}

SensorClient* CreateSensorClient(Dict* dict) {
	SensorClientImpl* vc=new SensorClientImpl();
	vc->Begin(dict);
	return vc;
}
void DestroySensorClient(SensorClient* sensorClient) {
	SensorClientImpl* vc=(SensorClientImpl*)sensorClient;
	vc->End();
	delete vc;
}
void SensorClientImpl::GetProperties(Dict* properties) {
	std::string hard=R"({
		"schema":{
			"properties":{
				"display":{
					"type":"boolean"
				},
				"tal":{
					"type":"number",
					"minimum":0,
					"maximum":1,
					"default":0.5,
					"step":0.1,
					"fast":0.2,
					"printFormat":"%.2f"
				},
				"test": {
					"type":"integer",
					"minimum":50,
					"maximum":500,
					"items":{
						"type":"integer",
						"enum": [100,102,204]
					},
					"default": "100"
				}
			}
		}
	})";
	Dict* client0Dict=properties->AddObjectNode(Id());
	Dict dict;
	dict.ReadFromJson(hard);
	client0Dict->Set(dict);
	Dict* values=client0Dict->AddObjectNode("values");
	values->Set("test",102);
}
void SensorClientImpl::SetProperties(const Dict& properties) {
//	uprintf("SensorClientImpl::SetProperties\n");
	//const Dict* values=properties.Find("values");
	//bool display=m_display;
	//values->Get("display",&display);
/*
	double tal=m_talTest;
	values->Get("tal",&tal);
	if(tal!=m_talTest) {
		uprintf("update tal %f->%f\n",m_talTest,tal);
		m_talTest=tal;
		App::PostEvent(App::EV_SAVE);
	}
	if(display!=m_display) {
		//uprintf("update display %s->%s\n",m_display?"true":"false",display?"true":"false");
		m_display=display;
	}
*/
}

bool SensorClientImpl::SaveConfig(Dict* config) {
	std::string id;
	config->Get("id",&id);
	return true;
}

void SensorClientImpl::GetStatus(Dict* status,bool includeSchema,bool includeGraphs) {
	if(includeSchema) {
		status->ReadFromJson("{\"schema\":[{\"param\":\"tal\",\"type\":\"int\"}],\"client\":{},\"graphs\":[]");
	}else{
		status->ReadFromJson("{\"client\":{}");
	}
	Dict* settings=status->Find("client");
	settings->Set("id",m_id);
	//settings->Set("class",ClassName());
	if(m_owner) {
		std::string deviceId=m_owner->Id();
		settings->Set("deviceId",deviceId);
	}
	settings->Set("frame",m_frameIndex);

	//uprintf("m_frameIndex %d\n",m_frameIndex);

	Dict* transfers=settings->AddArrayNode("transfers");
	m_activeHostsLock.lock();
	for(auto const& [k,v]:m_activeHosts) {
		Dict status;
		v->m_transfer->GetStatus(&status);
		status.Set("best roundtrip",(int64_t)v->m_timeSync.m_deltaRoundTripTime);
		status.Set("server time delta",(int64_t)v->m_timeSync.m_deltaTimeServer);
		//status.Set("server time drift us per minute",(int64_t)v->m_timeSync.m_deltaTimeDrift);
		transfers->PushBack(status);
	}
	m_activeHostsLock.unlock();
	settings->Set("decodeQueue",m_frames.Size());

	Dict* graphs=status->Find("graphs");
	if(!graphs)
		return;

	uint64_t time=GetTimeEpochMicroseconds();
	uint64_t timeMilliseconds=time/1000L;

	m_activeHostsLock.lock();
	for(auto const& [k,v]:m_activeHosts) {
		v->m_transfer->GetGraphs(graphs,timeMilliseconds);
	}
	m_activeHostsLock.unlock();

	for(int i=0;i!=countof(m_errorCorrection.m_decoders);i++) {
		ErrorCorrectionDecoder* ecd=m_errorCorrection.m_decoders[i];
		if(!ecd)
			continue;
		uint8_t streamId=m_errorCorrection.GetStreamIdFromIndex(i);
		ecd->GetGraphs(graphs,timeMilliseconds,streamId);
	}
	uint64_t timeEnd=timeMilliseconds;
	uint64_t timeBegin=timeMilliseconds-(HISTORY_AGE/1000);


	Dict* graph=graphs->PushBack();
	graph->Set("xmax",(int64_t)timeEnd);
	graph->Set("xmin",(int64_t)timeBegin);
	graph->Set("ymin",0.0);
	graph->Set("formatX","HH:MM:SS");
	graph->Set("formatY","us");
	Dict* legend=graph->AddObjectNode("legend");
	legend->Set("text","Roundtrip time");
	legend->Set("color","#0000ff");
	Dict* datasets=graph->AddArrayNode("datasets");
	m_activeHostsLock.lock();
	const char* colors[]={"#dd8452","#68a855","#524ec4","#b37281","#607893","#c38bda","#8c8c8c","#74b9cc","#cdb564","#b0724c"};
	int cnt=0;
	for(auto const& [k,v]:m_activeHosts) {
		Dict* dataset=datasets->AddObjectNode();
		dataset->Set("name",v->m_sourceName);
		dataset->Set("color",colors[cnt++]);
		std::vector<double> datax;
		std::vector<double> datay;
		std::scoped_lock sl(v->m_timeSync.m_roundtripLock);
		if(v->m_timeSync.m_roundTrips.size()) {
			int i=(int)v->m_timeSync.m_roundTrips.size()-1;
			for(;i>0;i--)
				if(v->m_timeSync.m_roundTrips[i].m_time<(double)timeBegin)
					break;
			datax.reserve(v->m_timeSync.m_roundTrips.size());
			datay.reserve(v->m_timeSync.m_roundTrips.size());
			for(;i!=(int)v->m_timeSync.m_roundTrips.size();i++) {
				datax.push_back((double)v->m_timeSync.m_roundTrips[i].m_time/1000L);
				datay.push_back((double)v->m_timeSync.m_roundTrips[i].m_roundtripMicroseconds);
			}
		}
		dataset->SetTypedArray("datax",datax.data(),datax.size());
		dataset->SetTypedArray("datay",datay.data(),datay.size());
	}
	m_activeHostsLock.unlock();
}

bool SensorClientImpl::ActiveHost::SendRequestFrames(uint64_t delay) {
	uint64_t t=GetTimeEpochMicroseconds();
	if(m_lastRequestTime && t-m_lastRequestTime<delay)
		return false;
	//uprintf("request more frames\n");
	m_lastRequestTime=t;
	RequestFramesHeader rfh;
	rfh.m_timeSend=t;
	//uprintf("%s Received Packets:%d Frames:%d. Average round trip time %d. Request more frames!\n",client.m_name.c_str(),client.m_framesReceived,client.AveragePingTime());
	m_transfer->SendToHost(&rfh,(int)sizeof(rfh));
	return true;
}

bool SensorClientImpl::ActiveHost::SendPing(uint64_t delay) {
	uint64_t t=GetTimeEpochMicroseconds();
	if(m_lastPingTime && t-m_lastPingTime<delay)
		return false;
	m_lastPingTime=t;
	PingFrameHeader ping;
	ping.m_count=m_pingCount++;
	ping.m_timeSend=t;
	//uprintf("%s Received Packets:%d Frames:%d. Average round trip time %d. Request more frames!\n",client.m_name.c_str(),client.m_framesReceived,client.AveragePingTime());
	m_transfer->SendToHost(&ping,(int)sizeof(ping));
	return true;
}

bool SensorClientImpl::Begin(Dict* dict) {

	//dict->Dump();
	dict->Get("verbose",&m_verbose,false);
	dict->Get("id",&m_id,"NA");

	Dict* transfer=dict->Find("transfer");
	if(transfer) {
		NetTransfer* t=CreateNetTransfer(transfer);
		ActiveHost* ah=new ActiveHost;
		ah->m_transfer=t;
		ah->m_sourceName=t->Host();
		ah->m_index=0;
		m_activeHosts.insert(std::make_pair(t->Host(),ah));
		t->SetDataCallback([this](NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived){
			OnData(transfer,data,dataBytesize,sourceName,timeReceived);
		});
	}else{
		Dict* transfers=dict->Find("transfers");
		if(transfers) {
			uint8_t index=0;
			for(auto i=transfers->begin();i!=transfers->end();++i) {
				Dict* transfer=&*i;
				NetTransfer* t=CreateNetTransfer(transfer);
				ActiveHost* ah=new ActiveHost;
				ah->m_transfer=t;
				ah->m_sourceName=t->Host();
				ah->m_index=index++;
				m_activeHosts.insert(std::make_pair(t->Host(),ah));
				t->SetDataCallback([this](NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived){
					OnData(transfer,data,dataBytesize,sourceName,timeReceived);
				});
			}
			if(index>MAX_NUMBER_HOSTS)
				FATAL("Number transfers %d exceeds maximum %d",index,MAX_NUMBER_HOSTS);
		}
	}
	if(!m_activeHosts.size())
		FATAL("SensorClientImpl transfers not setup");
	std::atomic<bool> ready=false;
	m_close=false;
	m_threadSend=std::thread([&]{
#ifdef USE_THREAD_AFFINITY
		SetSelfAffinityMask(THREAD_AFFINITY_CLIENT_MAIN);
#endif
		ready=true;
		while(!m_close) {

			m_activeHostsLock.lock();
			for(auto const& [k,v]:m_activeHosts) {
				if(v->SendRequestFrames(20*1000000L)) {
					uprintf("client %s send request frames from client receive thread. Host index %d\n",Id().c_str(),v->m_index);
				}
			}
			m_activeHostsLock.unlock();
			Frame frameDecode;
			if(m_frames.Pop(1000,[&](Frame* frame){
				//uprintf("Client %s received frame %d size %d checksum %d\n",Id().c_str(),frame->m_index,frame->m_data.size(),CRC32(frame->m_data.data(),(int)frame->m_data.size()));
				if(frame)
					frameDecode=*frame;
			})){
				m_frameIndex=frameDecode.m_index;
				for(auto fc:m_frameCallbacks) {
					DebugData debugData=frameDecode.m_debugData;
					debugData.m_timers[1]+=(int)(GetTimeEpochMicroseconds()-frameDecode.m_firstTime);
					fc.m_callback(frameDecode.m_hostIndex,frameDecode.m_streamId,frameDecode.m_dataSourceType,frameDecode.m_data.data(),(int)frameDecode.m_data.size(),frameDecode.m_index,frameDecode.m_timeCapture,debugData,fc.m_arg);
				}
			}
		}
	});
	while(!ready);
	return true;
}
void SensorClientImpl::End() {
	m_close=true;
	if(m_threadSend.joinable())
		m_threadSend.join();
	for(auto const& [k,v]:m_activeHosts) {
		DestroyNetTransfer(v->m_transfer);
		delete v;
	}
	m_activeHosts.clear();
}
