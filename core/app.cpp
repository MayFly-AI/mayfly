#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<chrono>
#include<thread>
#include<assert.h>
#include<algorithm>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"
#include "shared/dict.h"
#include "shared/thread.h"
#include "memory/tensor.h"
#include "video/decoder.h"

#include "app.h"
#include "transfer.h"
#include "httpserver.h"

std::string Color2String(uint32_t color) {
	return stdx::format_string("#%02x%02x%02x",color&0xff,(color>>8)&0xff,(color>>16)&0xff);
}
uint32_t String2Color(const char* colorString) {
	uint32_t color=0;
	if(colorString[0]=='#') {
		const char* p=colorString+1;
		unsigned int byte;
		uint8_t bytes[4];
		int cnt=0;
		while(cnt<=(int)sizeof(bytes) && sscanf(p,"%2x",&byte)==1 ) {
			bytes[cnt++]=byte;
			//uprintf( "res=%d, byte=%d(%02x)\n", res, byte, byte );
			p+=2;
		}
		color=(bytes[2]<<16)|(bytes[1]<<8)|bytes[0];
	}
	return color;
}

//HistoryLog
void HistoryLog::Add(uint64_t time,const char* str) {
	std::scoped_lock sl(m_lock);
	if(m_strings.size() && time<m_strings.back().m_time) {
		::printf("Got log with time older than last. ignore log\n");			//recursive, cannot use uprint
		return;
	}
	m_strings.push_back({});
	Entry* e=&m_strings.back();
	e->m_time=time;
	e->m_str=str;
}
void HistoryLog::GetLastEntries(std::vector<HistoryLog::Entry>* strings,int count) {
	std::scoped_lock sl(m_lock);
	int i=MAX(0,(int)m_strings.size()-count);
	for(;i<(int)m_strings.size();i++) {
		strings->push_back(m_strings[i]);
	}
}
uint64_t HistoryLog::GetNewEntries(std::vector<HistoryLog::Entry>* strings,uint64_t time) {
	std::scoped_lock sl(m_lock);
	if(!m_strings.size())
		return time;
	for(int i=(int)m_strings.size()-1;i>=-1;--i) {
		if(i==-1 || m_strings[i].m_time<=time) {
			i++;
			for(;i!=(int)m_strings.size();i++) {
				strings->push_back(m_strings[i]);
				time=m_strings[i].m_time;
				if(strings->size()==100)					//Limit number lines per request
					break;
			}
			return time;
		}
	}
	return time;
}
//Timers

#define TIMERS_AGE 5000000

Timers::Timers() {
	m_timers.resize(NUMBER_MEASUREMENTS);
}
void Timers::SetTimerInfo(int index,const char* name,uint32_t color) {
	if(index>(int)m_timers.size())
		FATAL("out of range");
	m_timers[index].m_name=name;
	m_timers[index].m_color=color;
}
void Timers::PushTime(int v0,int v1,int v2,int v3) {
	m_timersLock.lock();
	uint64_t time=GetTimeEpochMicroseconds();
	while(m_measurements.size()>2 && time-m_measurements[1].m_time>TIMERS_AGE)
		m_measurements.pop_front();
	m_measurements.push_back({time,{v0,v1,v2,v3}});
	m_timersLock.unlock();
}
//ActiveService
ActiveService::~ActiveService(){
}
ActiveServer::~ActiveServer(){
	DestroySensorServer((SensorServer*)m_service);
}
ActiveClient::~ActiveClient(){
	DestroySensorClient((SensorClient*)m_service);
	for(auto&[k,v]:m_frameDecoders) {
		DestroyFrameDecoder(v);
	}
}

FrameDecoder* ActiveClient::GetOrCreateFrameDecoder(uint8_t streamId,const std::string& encoding) {
	auto found=m_frameDecoders.find(streamId);
	if(found==m_frameDecoders.end()) {
		FrameDecoder* dec=CreateFrameDecoder(encoding, &m_decoderDict);
		if(!dec) {
			uprintf("No frame decoder found for '%s' enconding\n", encoding.c_str());
		}
		m_frameDecoders[streamId]=dec;
		return dec;
	}
	return found->second;
}

struct DebugConnection {
	NetTransfer* m_transfer=0;
	std::string m_id;
	struct Service {
		std::string m_id;
		Dict m_status;
		Dict m_properties;
		bool m_statusEnabled=false;
	};
	void UpdatePeerTimeEstimate(uint64_t timeReceived,uint64_t timeSend,uint64_t timeLoopback,uint32_t timeSpendServer) {
		uint32_t rt=(uint32_t)(timeReceived-timeLoopback)-timeSpendServer;			//roundtrip=receivetime-sendtime
		uint64_t driftMax=((timeReceived-m_timeLastMeasure)*500)/(60*1000000);		//drift estimate since used difference. Max drift constant set to 500 us per minute
		if(!m_timeDifference || m_timeRoundtrip+driftMax>rt) {						//Is this meassurement better than the used one
			uint64_t estimateTimeServerNow=timeSend+(rt/2);							//time server now=time server send + roundtrip/2
			m_timeRoundtrip=rt;
			m_timeDifference=estimateTimeServerNow-timeReceived;
			m_timeLastMeasure=timeReceived;
			//uprintf("UpdatePeerTimeEstimate timeRoundtrip %d timeDifference %lldus\n",m_timeRoundtrip,m_timeDifference);
		}
	}
	bool GetDeltaTimeToPeer(int64_t* dt)const {
		if(!m_timeLastMeasure) {
			uprintf("WARNING: Unable get time difference before roundtrip packet is registered\n");
			return false;
		}
		//if(m_timeRoundtrip>5000) {
		//	uprintf("WARNING: Unable to register log time difference %d not within threshold\n",m_timeRoundtrip);
		//	return false;
		//}
		*dt=m_timeDifference;
		return true;
	}
	uint64_t m_lastLogDisplayTime=0;

	uint32_t m_timeRoundtrip=0;
	int64_t m_timeDifference=0;
	int64_t m_timeLastMeasure=0;

	uint64_t m_lastLogRequestTime=0;
	uint64_t m_logTime=0;
	HistoryLog m_log;
	File m_logFile;
	std::mutex m_componentLock;
	std::vector<Service> m_components;
};

FixedQueueMT<App::Event,16> App::m_queue;

App::App() {
}
App::~App() {
}

void App::PostEvent(eEventId eventId,const uint8_t* data,int dataBytesize) {
	m_queue.Push([&](Event* event) {
		event->m_eventId=eventId;
		event->m_data.assign(data,data+dataBytesize);
	});
}
void App::CollectAndSaveConfig() {
	bool success=true;
	Dict config;
	config.Copy(m_config);
	Dict* devices=config.Find("devices");
	if(devices && devices->Size()) {
		devices=devices->First();
		SaveConfig(devices);
	}
	if(success) {
		config.Set("lastAccess",(int64_t)GetTimeEpochMicroseconds());
		std::string json=config.WriteToJson(false,0,4);
		if(!SaveFile(m_configFileName,std::vector<char>(json.begin(),json.end()))) {
			uprintf("Save config failed\n");
		}else{
			uprintf("Save config success\n");
			m_config.Copy(config);
		}
	}
}
void App::OnEvent(App::eEventId eventId,const uint8_t* data,int dataBytesize) {
	//uprintf("App::OnEvent %d\n",eventId);
	if(eventId==App::EV_SAVE_RESTART) {
		CollectAndSaveConfig();
		m_restart=true;
		return;
	}
	if(eventId==App::EV_SAVE) {
		CollectAndSaveConfig();
		return;
	}
	if(eventId==App::EV_RESTART) {
		m_restart=true;
		return;
	}
	m_close=true;
}
void App::PollEvents(bool block) {
	Update(m_tick);
	m_tick++;
	if(block || !m_queue.Empty()) {
		//uprintf("App::PollEvents wait\n");
		m_queue.Pop(1000/10,[&](Event* event) {
			OnEvent(event->m_eventId,event->m_data.data(),(int)event->m_data.size());
		});
	}
}
bool App::ShouldExitMainLoop()const {
	return (m_close || m_restart);
}
void App::MainLoop() {
	while(true) {
		PollEvents(true);
		if(ShouldExitMainLoop())
			break;
	}
	uprintf("exit mainloop\n");
}

int App::Run(const std::string& configFileName) {
	m_configFileName=configFileName;
	uprintf("Load config file %s\n",GetFileNameRemap(m_configFileName).c_str());
	std::vector<char> data;
	if(!LoadFile(&data,m_configFileName)) {
		if(!m_config.ReadFromJson(m_defaultJson.c_str(),m_defaultJson.size())) {
			FATAL("Invalid default config file");
		}
		std::string json=m_config.WriteToJson(false,0,4);
		if(!SaveFile(m_configFileName,std::vector<char>(json.begin(),json.end()))) {
			uprintf("Unable to write config file\n");
		}
	}else{
		if(!m_config.ReadFromJson(data.data(),data.size())) {
			FATAL("Unable to parse config file");
		}
		std::string json=m_config.WriteToJson(false,0,4);
		if(!SaveFile(m_configFileName,std::vector<char>(json.begin(),json.end()))) {
			FATAL("Unable to write config file\n");
		}
	}
	//m_config.Dump();

	InitSockets();

	m_running=true;

	SetPrintCallback([this](const char* str){
		m_log.Add(GetTimeEpochMicroseconds(),str);
	});
	Begin(&m_config);

#ifdef USE_THREAD_AFFINITY
	SetSelfAffinityMask(THREAD_AFFINITY_MAIN);
#endif
	while(!m_close) {
		MainLoop();
		End();
		SetPrintCallback(0);

		if(m_close)
			break;
		m_running=false;
		for(int i=1;i>0;i--) {
			uprintf("Wait for socket time out %d second(s)\n",i);
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
		m_running=true;

		SetPrintCallback([this](const char* str){
			m_log.Add(GetTimeEpochMicroseconds(),str);
		});
		Begin(&m_config);

		m_restart=false;
	}

	EndSockets();
	std::string json=m_config.WriteToJson(false,0,4);
	if(!SaveFile(m_configFileName,std::vector<char>(json.begin(),json.end()))) {
		uprintf("Unable to write config file\n");
	}
	return 0;
}

bool App::SaveConfig(Dict* config) {
	Dict* vs=config->Find("services");
	if(vs && vs->Size()) {
		vs=vs->First();
		for(ActiveService* as:m_activeServices) {
			if(!as->m_service->SaveConfig(vs))
				return false;
			vs=vs->Next();
		}
	}
	return true;
}

std::vector<std::string> App::GetDisplayClients()const {
	std::vector<std::string> clients;
	for(ActiveService* ac:m_activeServices) {
		if(ac->Type()!=Service::VIDEO_CLIENT)
			continue;
		clients.push_back(ac->Id());
	}
	return clients;
}
void App::UpdateRemoteHttpServer() {
	uprintf("remote\n");
	Dict remoteConnections;
	Dict* connections=remoteConnections.AddArrayNode("connections");
	for(DebugConnection* dc:m_debugConnections) {
		uprintf("host %s\n",dc->m_transfer->Host().c_str());
		Dict* node=connections->PushBack();
		node->Set("host",dc->m_transfer->Host());
	}
	m_httpServer->Send(remoteConnections);
}
void App::UpdateDashboardHttpServer() {
	Dict dashboard;
	dashboard.Set("device",m_id);
	dashboard.Set("services",(int)m_activeServices.size());
	dashboard.Set("time",(int64_t)GetTimeEpochMicroseconds());
	m_httpServer->Send(dashboard);
}
void App::UpdateHttpServer() {
	Dict statusViewer;
	statusViewer.ReadFromJson("{\"status\":[]}");
	Dict* status=statusViewer.Find("status");
	Dict statusDict;
	GetStatus(&statusDict,true,true);
	status->PushBack(statusDict);
	Dict statusHttpServer;
	m_httpServer->GetStatus(&statusHttpServer);
	status->PushBack(statusHttpServer);
	for(ActiveService* as:m_activeServices) {
		Dict statusServer;
		as->m_service->GetStatus(&statusServer,true,true);
		status->PushBack(statusServer);
	}
	//uprintf("UpdateHttpServer ok\n");
	m_httpServer->Send(statusViewer);
}

void App::SendPropertiesToBrowser(const PropertiesHeader& propertiesHeader,const Dict& properties) {
	//uprintf("send properties to browser\n");
	Dict response;
	response.Set("queryId",propertiesHeader.m_loopback.m_queryId);
	Dict* content=response.AddObjectNode("content");
	//content->Set("queryType","Properties");
	//properties.Dump();

	const Dict* values=properties.Find("values");

	Dict* settings=content->AddObjectNode("settings");
	settings->Copy(*values);
	settings->SetName("settings");

	Dict* schema=content->AddArrayNode("schema");
	const Dict* schemaProperties=properties.Find("schema");

	schema->Copy(*schemaProperties);
	schema->SetName("schema");

	//Dict* param=schema->PushBack();
	//param->Set("param","type");
	//param->Set("type","int_range");
	//param->SetTypedArray("range",std::vector<double>({0,1}));
	//param->PushBack(Dict(R"({"param":"created","edit":false,"type":"date"})"));

	content->Set("success",true);
	m_httpServer->Send(response);
}
void App::SendStatusToBrowser(const StatusHeader& statusHeader,const Dict& status) {
	//uprintf("send status to browser\n");
	Dict response;
	response.Set("queryId",statusHeader.m_loopback.m_queryId);
	Dict* content=response.AddObjectNode("content");
	//content->Set("queryType","Status");

	const Dict* statusSchema=0;
	const Dict* statusGraps=0;
	const Dict* statusSettings=0;

	const Dict* p=status.First();
	while(p) {
		if(p->Name()=="schema") {
			statusSchema=p;
		}else
		if(p->Name()=="graphs") {
			statusGraps=p;
		}else{
			statusSettings=p;
		}
		p=p->Next();
	}

	Dict* settings=content->AddObjectNode("settings");
	if(statusSettings) {
		settings->Copy(*statusSettings);
		settings->SetName("settings");
	}else{
		settings->Set("hash","fgdfdg");
		settings->Set("fullName","david guld");
	}
	Dict* schema=content->AddArrayNode("schema");
	if(statusSchema) {
		schema->Copy(*statusSchema);
		schema->SetName("schema");
	}else{
		Dict* param=schema->PushBack();
		param->Set("param","type");
		param->Set("type","int_range");
		param->SetTypedArray("range",std::vector<double>({0,1}));
		param->PushBack(Dict(R"({"param":"created","edit":false,"type":"date"})"));
	}
	if(statusGraps) {
		Dict* graphs=content->AddArrayNode("graphs");
		graphs->Copy(*statusGraps);
		graphs->SetName("graphs");
	}
	content->Set("success",true);
	//response.Dump();

	m_httpServer->Send(response);
}
bool App::GetPropertiesFromBrowser(const Dict& request)const {
	const Dict* requestContent=request.Find("queryContent");
	std::string id;
	if(!requestContent || !requestContent->Get("id",&id))
		return false;
	int queryId;
	if(!request.Get("queryId",&queryId))
		FATAL("WTF!");
	if(!queryId)
		FATAL("WTF!");
	int index=0;
	for(const DebugConnection* dc:m_debugConnections) {
		if(dc->m_id==id) {
			GetDebugPropertiesHeader dph;
			dph.m_timeSend=GetTimeEpochMicroseconds();
			dph.m_loopback.m_debugConnectionIndex=index;
			dph.m_loopback.m_queryId=queryId;
			dc->m_transfer->SendToHost(&dph,(int)sizeof(dph));
			break;
		}
		index++;
	}
	return false;
}
bool App::GetStatusFromBrowser(const Dict& request)const {
	const Dict* requestContent=request.Find("queryContent");
	std::string id;
	if(!requestContent || !requestContent->Get("id",&id))
		return false;
	int queryId;
	if(!request.Get("queryId",&queryId))
		FATAL("WTF!");
	if(!queryId)
		FATAL("WTF!");
	int index=0;
	for(const DebugConnection* dc:m_debugConnections) {
		if(dc->m_id==id) {
			GetDebugStatusHeader dsh;
			dsh.m_timeSend=GetTimeEpochMicroseconds();
			dsh.m_loopback.m_debugConnectionIndex=index;
			dsh.m_loopback.m_queryId=queryId;
			dc->m_transfer->SendToHost(&dsh,(int)sizeof(dsh));
			break;
		}
		index++;
	}

	return false;
}
bool App::GetServices(Dict* response) {
	Dict* content=response->Find("content");
	if(!content)
		return false;
	Dict* services=content->AddArrayNode("services");
	for(DebugConnection* dc:m_debugConnections) {
		Dict* service=services->PushBack();
		service->Set("id",dc->m_id);
		Dict* settings=service->AddObjectNode("settings");
		settings->Set("created",(int64_t)GetTimeEpochMilliseconds()/1000L);
		settings->Set("host",dc->m_transfer->Host().c_str());
		settings->Set("type",1);
		settings->Set("edit",true);
	}
	return true;
}

void App::OnDataDebug(NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived) {
	PacketHeader* header=(PacketHeader*)data;
	header->SanityCheck(dataBytesize);
	if(!header->m_dataBytesize) {

		if(header->m_type==PacketHeader::DEBUG_TIMING_REPLY) {
			DebugTimingReplyHeader* dtReply=(DebugTimingReplyHeader*)data;
			int roundtripTime=(int)(timeReceived-dtReply->m_timeLoopback);
			int captureDelta=(int)(dtReply->m_timeSend-dtReply->m_captureTimeLoopback);
			if(captureDelta>roundtripTime) {		//Should always be the case
				m_debugTiming.m_timers.PushTime(captureDelta-roundtripTime,roundtripTime,0,0);
				//uprintf("captureDelta-roundtripTime %d\n",(int)(captureDelta-roundtripTime));
			}else{
				uprintf("Received debug timing reply from %s roundtrip time %dus capture delta %dus\n",sourceName,roundtripTime,captureDelta);
			}
			return;
		}
		if(header->m_type==PacketHeader::DEBUG_TIMING_REQUEST) {
			DebugTimingRequestHeader* dtRequest=(DebugTimingRequestHeader*)header;
			//uprintf("Received debug timing request from %s, sending pong\n",source.ToString().c_str());
			DebugTimingReplyHeader dtReply;
			dtReply.m_timeLoopback=dtRequest->m_timeSend;
			dtReply.m_captureTimeLoopback=dtRequest->m_captureTime;
			dtReply.m_timeSend=GetTimeEpochMicroseconds();
			m_debugTransfer->SendTo(sourceName,&dtReply,sizeof(dtReply));
			return;
		}

		if(header->m_type==PacketHeader::GET_IDS) {
			//uprintf("got ids\n");
			GetDebugIdsHeader* gih=(GetDebugIdsHeader*)header;
			std::vector<std::string> ids;
			ids.push_back(Id());
			for(ActiveService* ac:m_activeServices) {
				ids.push_back(ac->Id());
			}
			std::string idsString=stdx::FromStdArray(ids);
			uint8_t buf[512];
			DebugIdsHeader* dsh=(DebugIdsHeader*)buf;
			*dsh=DebugIdsHeader();
			dsh->m_loopback=gih->m_loopback;
			dsh->m_timeSend=GetTimeEpochMicroseconds();
			dsh->m_dataBytesize=(uint32_t)idsString.size();
			memcpy(dsh->Data(),idsString.c_str(),dsh->m_dataBytesize);
			transfer->SendTo(sourceName,dsh,dsh->PacketByteSize());
			return;
		}
		uprintf("App::OnDataDebug got debug packet without component id. Ignore packet\n");
		return;
	}
	switch(header->m_type) {
		case PacketHeaderBase::GET_STATUS: {
			std::string id((char*)header->Data(),header->m_dataBytesize);
			Service* component=FindService(id);
			if(!component) {
				uprintf("App::OnDataDebug GET_STATUS id %s not found\n",id.c_str());
				break;
			}
			Dict status;
			component->GetStatus(&status,true,true);
			GetDebugStatusHeader* gsh=(GetDebugStatusHeader*)data;
			std::string json=status.WriteToJson(true);
			StatusHeader sh;
			sh.m_timeSend=GetTimeEpochMicroseconds();
			sh.m_dataBytesize=(int)json.size();
			sh.m_loopback=gsh->m_loopback;
			std::vector<char> p;
			p.resize(sh.m_headerBytesize+sh.m_dataBytesize);
			memcpy(p.data(),&sh,sh.m_headerBytesize);
			memcpy(p.data()+sh.m_headerBytesize,json.c_str(),(int)json.size());
			transfer->SendTo(sourceName,p.data(),(int)p.size());
			break;
		}
		case PacketHeaderBase::GET_LOG: {
			Dict status;
			uint32_t requestLen=(uint32_t)strlen((char*)header->Data());
			Dict request;
			request.ReadFromJson((char*)header->Data(),requestLen);
			GetLog(&status,request);
			GetDebugLogHeader* glh=(GetDebugLogHeader*)data;
			std::string json=status.WriteToJson(true);
			LogHeader lh;
			lh.m_timeSend=GetTimeEpochMicroseconds();
			lh.m_dataBytesize=(int)json.size();
			lh.m_loopback=glh->m_loopback;
			lh.m_timeLoopback=glh->m_timeSend;
			std::vector<char> p;
			p.resize(lh.m_headerBytesize+lh.m_dataBytesize);
			lh.m_timeSpendServer=(uint32_t)(GetTimeEpochMicroseconds()-timeReceived);
			memcpy(p.data(),&lh,lh.m_headerBytesize);
			memcpy(p.data()+lh.m_headerBytesize,json.c_str(),(int)json.size());
			transfer->SendTo(sourceName,p.data(),(int)p.size());
			break;
		}
		case PacketHeaderBase::SET_PROPERTIES: {
			//uprintf("device received SET_PROPERTIES for id %s\n",id.c_str());
			uint32_t idLen=(uint32_t)strlen((char*)header->Data());
			std::string id((char*)header->Data(),idLen);
			Service* component=FindService(id);
			if(!component) {
				uprintf("App::OnDataDebug SET_PROPERTIES id %s not found\n",id.c_str());
				break;
			}
			SetDebugPropertiesHeader* sph=(SetDebugPropertiesHeader*)data;
			const char* pjson=(const char*)sph->Data()+idLen+1;
			const char* pjsonEnd=(const char*)sph->Data()+sph->m_dataBytesize;
			Dict properties;
			properties.ReadFromJson(pjson,pjsonEnd-pjson);
			component->SetProperties(properties);
			break;
		}
		case PacketHeaderBase::GET_PROPERTIES: {
			//uprintf("device received GET_PROPERTIES for id %s\n",id.c_str());
			std::string id((char*)header->Data(),header->m_dataBytesize);
			Service* component=FindService(id);
			if(!component) {
				uprintf("App::OnDataDebug GET_PROPERTIES id %s not found\n",id.c_str());
				break;
			}
			GetDebugPropertiesHeader* gph=(GetDebugPropertiesHeader*)data;
			//uprintf("client %s get properties\n",Id().c_str());
			Dict dict;
			component->GetProperties(&dict);
			std::string json=dict.WriteToJson(true);
			PropertiesHeader ph;
			ph.m_timeSend=GetTimeEpochMicroseconds();
			ph.m_dataBytesize=(int)json.size();
			ph.m_loopback=gph->m_loopback;
			std::vector<char> p;
			p.resize(ph.m_headerBytesize+ph.m_dataBytesize);
			memcpy(p.data(),&ph,ph.m_headerBytesize);
			memcpy(p.data()+ph.m_headerBytesize,json.c_str(),(int)json.size());
			m_debugTransfer->SendTo(sourceName,p.data(),(int)p.size());
			break;
		}
		default:
			uprintf("Received unhandled packet type %d\n",header->m_type);
			break;
	}
}

void App::Update(int tick) {
	if(!tick) {
		for(const DebugConnection* dc:m_debugConnections) {
			dc->m_transfer->SetDataCallback([this](NetTransfer* transfer,const void* packet,int packetBytesize,const SocketAddress& source,uint64_t timeReceived){
				PacketHeader* header=(PacketHeader*)packet;
				header->SanityCheck(packetBytesize);
				switch(header->m_type) {
					case PacketHeader::IDS: {
						DebugIdsHeader* dsh=(DebugIdsHeader*)header;
						std::string idsString((char*)dsh->Data(),dsh->m_dataBytesize);
						std::vector<std::string> ids=stdx::Split(idsString,',');
						DebugConnection* dc=m_debugConnections[dsh->m_loopback.m_debugConnectionIndex];
						dc->m_componentLock.lock();
						dc->m_components.clear();
						dc->m_components.reserve(ids.size());
						for(const auto& id:ids) {
							dc->m_components.push_back({id});
						}
						dc->m_componentLock.unlock();
						break;
					}
					case PacketHeader::LOG: {
						LogHeader* lh=(LogHeader*)header;
						DebugConnection* dc=m_debugConnections[lh->m_loopback.m_debugConnectionIndex];
						dc->UpdatePeerTimeEstimate(timeReceived,lh->m_timeSend,lh->m_timeLoopback,lh->m_timeSpendServer);
						int64_t deltaTimeToPeer;
						if(!dc->GetDeltaTimeToPeer(&deltaTimeToPeer)) {
							break;
						}
						Dict log;
						log.ReadFromJson((const char*)header->Data(),header->m_dataBytesize);
						//log.Dump();
						double dt;
						if(!log.Get("time",&dt)) {
							FATAL("Received log without time");
						}
						dc->m_lastLogRequestTime=(uint64_t)dt;
						const Dict* entries=log.Find("entries");
						//uint64_t localTime=GetTimeEpochMicroseconds();
						if(entries) {
							std::string str;
							for(auto it=entries->begin();it!=entries->end();++it) {
								const Dict& entry=*it;
								entry.Get("time",&dt);
								entry.Get("str",&str);
								uint64_t peerLogTime=(uint64_t)dt;
								uint64_t time=peerLogTime+deltaTimeToPeer;
								dc->m_log.Add(time,str.c_str());
							}
						}
						break;
					}
					case PacketHeader::STATUS: {
						StatusHeader* sh=(StatusHeader*)header;
						DebugConnection* dc=m_debugConnections[sh->m_loopback.m_debugConnectionIndex];
						Dict status;
						status.ReadFromJson((const char*)header->Data(),header->m_dataBytesize);
						dc->m_componentLock.lock();
						DebugConnection::Service* component=&dc->m_components[sh->m_loopback.m_debugServiceIndex];
						component->m_status=status;
						dc->m_componentLock.unlock();
						if(!dc->m_id.size()) {
							const Dict* p=status.First();
							while(p) {
								if(p->Name()!="schema" && p->Name()!="graphs") {
									p->Get("id",&dc->m_id);
									break;
								}
								p=p->Next();
							}
						}
						if(sh->m_loopback.m_queryId) {
							SendStatusToBrowser(*sh,status);
						}
						break;
					}
					case PacketHeader::PROPERTIES: {
						PropertiesHeader* ph=(PropertiesHeader*)header;
						DebugConnection* dc=m_debugConnections[ph->m_loopback.m_debugConnectionIndex];
						Dict properties;
						properties.ReadFromJson((const char*)header->Data(),header->m_dataBytesize);
						dc->m_componentLock.lock();
						DebugConnection::Service* component=&dc->m_components[ph->m_loopback.m_debugServiceIndex];
						component->m_properties=properties;
						dc->m_componentLock.unlock();
						if(ph->m_loopback.m_queryId) {
							SendPropertiesToBrowser(*ph,properties);
						}
						break;
					}
					default:
						uprintf("Received unknow packet type %d\n",header->m_type);
				}
			});
		}
	}
	if(tick%2)
		return;
	//m_log.Add(GetTimeEpochMicroseconds(),stdx::format_string("test %d\n",tick).c_str());

	if(!(tick%100)) {
		for(int i=0;i!=(int)m_debugConnections.size();i++) {
			DebugConnection* dc=m_debugConnections[i];
			Dict request;
			request.Set("time",(int64_t)dc->m_lastLogRequestTime);
			//request.Dump();
			std::string json=request.WriteToJson(true);
			GetDebugLogHeader dlh;
			dlh.m_timeSend=GetTimeEpochMicroseconds();
			dlh.m_dataBytesize=(int)json.size();
			dlh.m_loopback.m_debugConnectionIndex=i;
			std::vector<char> p;
			p.resize(dlh.m_headerBytesize+dlh.m_dataBytesize);
			memcpy(p.data(),&dlh,dlh.m_headerBytesize);
			memcpy(p.data()+dlh.m_headerBytesize,json.c_str(),(int)json.size());
			dc->m_transfer->SendToHost(p.data(),(int)p.size());
		}
	}
	for(int i=0;i!=(int)m_debugConnections.size();i++) {
		DebugConnection* dc=m_debugConnections[i];
		dc->m_componentLock.lock();
		if(!dc->m_components.size()) {
			GetDebugIdsHeader gih;
			gih.m_loopback.m_debugConnectionIndex=i;
			m_debugConnections[i]->m_transfer->SendToHost(&gih,(int)sizeof(gih));
		}else{
			for(int j=0;j!=(int)dc->m_components.size();j++) {
				if(!dc->m_components[j].m_statusEnabled)
					continue;
				uint8_t buf[512];
				GetDebugStatusHeader* dsh=(GetDebugStatusHeader*)buf;
				*dsh=GetDebugStatusHeader();
				dsh->m_timeSend=GetTimeEpochMicroseconds();
				dsh->m_loopback.m_debugConnectionIndex=i;
				dsh->m_loopback.m_debugServiceIndex=j;
				dsh->m_dataBytesize=(uint32_t)dc->m_components[j].m_id.size();
				memcpy(dsh->Data(),dc->m_components[j].m_id.c_str(),dsh->m_dataBytesize);
				dc->m_transfer->SendToHost(dsh,dsh->PacketByteSize());
			}
		}
		dc->m_componentLock.unlock();
	}
	for(DebugConnection* dc:m_debugConnections) {
		if(!dc->m_logFile.Valid()) {
			std::string host=dc->m_transfer->Host();
			if(!host.size())
				continue;
			stdx::ReplaceAll(&host,".","_");
			stdx::ReplaceAll(&host,":","_");				//Host name can be used as filename
			std::string fileName=m_sessionLogsPath+host+".log";
			dc->m_logFile.SetFileName(fileName.c_str());
		}
		std::vector<HistoryLog::Entry> entries;
		dc->m_logTime=dc->m_log.GetNewEntries(&entries,dc->m_logTime);
		for(const auto& e:entries) {
			std::string ts=stdx::format_string("[%s] %s",TimeEpochMicrosecondsToString(e.m_time).c_str(),e.m_str.c_str()," ");
			dc->m_logFile.AppendString(ts.c_str());
		}
	}
}


bool App::Begin(Dict* dict) {
	uint64_t time=GetTimeEpochMicroseconds();
	std::string timeString=TimeEpochMicrosecondsToString(time,"_","-");
	m_sessionLogsPath=stdx::format_string("$(HOME)/logs/%s/",timeString.c_str());
	CreateDirRecursive(m_sessionLogsPath.c_str());
	Dict* httpSettings=dict->Find("httpserver");

#ifdef OPEN_GL
	dict->Get("display",&m_display,false);
#endif
	dict->Get("id",&m_id);
	dict->Get("verbose",&m_verbose,false);

	const Dict* debugTiming=dict->Find("debugTiming");
	if(debugTiming) {
		int port;
		std::string ip;
		if(debugTiming->Get("port",&port) && debugTiming->Get("ip",&ip)) {
			m_debugTiming.m_host=stdx::format_string("%s:%d",ip.c_str(),port);
			m_debugTiming.m_timers.SetTimerInfo(0,"glasstodecode",0x5284dd);
			m_debugTiming.m_timers.SetTimerInfo(1,"roundtrip",0x84dd52);
		}
	}

	Dict* debugConnectionsDict=dict->Find("debugConnections");
	if(debugConnectionsDict) {
		for(auto it=debugConnectionsDict->begin();it!=debugConnectionsDict->end();++it) {
			Dict* debugConnectionDict=&*it;
			Dict* transfer=debugConnectionDict->Find("transfer");
			if(!transfer)
				continue;
			m_debugConnections.push_back(new DebugConnection);
			DebugConnection* dc=m_debugConnections.back();
			dc->m_transfer=CreateNetTransfer(transfer);
		}
	}

	Dict* debugTransfer=dict->Find("debugTransfer");
	if(debugTransfer) {
		m_debugTransfer=CreateNetTransfer(debugTransfer);
		m_debugTransfer->SetDataCallback([this](NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived){
			OnDataDebug(transfer,data,dataBytesize,sourceName,timeReceived);
		});
	}

	if(httpSettings) {
		m_httpServer=CreateHttpServer(*httpSettings);
		m_httpServer->SetWebsocketDataCallback(this,[](const char* json,int jsonBytesize,void* arg){
			App* device=(App*)arg;
			Dict request;
			request.ReadFromJson(json,jsonBytesize);
			std::string type;
			if(request.Get("type",&type)) {
				if(type=="dashboard") {
					device->UpdateDashboardHttpServer();
				}else
				if(type=="local") {
					device->UpdateHttpServer();
				}else
				if(type=="remote") {
					device->UpdateRemoteHttpServer();
				}else
				if(type=="query") {
					std::string queryType;
					int queryId;
					if(request.Get("queryId",&queryId) && request.Get("queryType",&queryType)) {
						if(queryType=="GetStatus") {
							device->GetStatusFromBrowser(request);
						}else
						if(queryType=="GetProperties") {
							device->GetPropertiesFromBrowser(request);
						}else{
							Dict response;
							response.Set("queryId",queryId);
							Dict* content=response.AddObjectNode("content");
							if(queryType=="SetService") {
								content->Set("success",true);
								device->m_httpServer->Send(response);
							}else
							if(queryType=="GetServices") {
								if(device->GetServices(&response)) {
									content->Set("success",true);
									device->m_httpServer->Send(response);
								}
							}else{
								uprintf("unsupported query %s\n",queryType.c_str());

							}
						}
					}else{
						uprintf("malformed query\n");
						request.Dump();
					}
					//device->UpdateQueryHttpServer();
				}else{
					uprintf("WebsocketDataCallback message type %s not supported\n",type.c_str());
				}
			}else{
				uprintf("WebsocketDataCallback message type not defined\n");
			}
		});
	}else{
		uprintf("tag \"httpserver\" not found. no http server started\n");
	}

	Dict* servicesSettings=dict->Find("services");
	//if(!servicesSettings)
	//	FATAL("ini file missing services, delete inifile and rerun to create default");
	if(servicesSettings) {
		for(auto it=servicesSettings->begin();it!=servicesSettings->end();++it) {
			Dict* dict=&*it;
			std::string type;
			dict->Get("type",&type);
			if(type=="sensorServer" || type=="videoServer") {
				SensorServer* sensorServer=CreateSensorServer(dict);
				sensorServer->m_owner=this;
				sensorServer->SetFrameEncodedCallback(this,[](const char* id,const void* frame,int frameBytesize,int index,uint64_t captureTime,void* arg){
					App* app=(App*)arg;
					if(app->m_frameEncodedCallback) {
						app->m_frameEncodedCallback(id,frame,frameBytesize,index,captureTime,app->m_frameEncodedCallbackArg);
					}
				});
				ActiveServer* as=new ActiveServer;
				as->m_service=sensorServer;
				AddActiveService(as);
			}else
			if(type=="sensorClient" || type=="videoClient") {
				SensorClient* vc=CreateSensorClient(dict);
				vc->m_owner=this;
				ActiveClient* ac=new ActiveClient;
				ac->m_service=vc;
				AddActiveService(ac);
				Dict* decoderDict=dict->Find("decoder");
				if(decoderDict)
					ac->m_decoderDict.Copy(*decoderDict);
				//ac->m_decoder=CreateH264Decoder(decoderDict);
				ac->SetTimerInfo(0,"encode",0x5284dd);
				ac->SetTimerInfo(1,"send",0x68a855);
				ac->SetTimerInfo(2,"decode1",0x524ec4);
				ac->SetTimerInfo(3,"decode2",0xb37281);

			}
		}
	}
	for(ActiveService* as:m_activeServices) {
		if(as->Type()!=Service::VIDEO_CLIENT)
			continue;
		ActiveClient* ac=(ActiveClient*)as;
		SensorClient* sensorClient=(SensorClient*)as->m_service;
		uprintf("set connection for client %s\n",sensorClient->Id().c_str());
		bool connected=false;
		for(ActiveService* as1:m_activeServices) {
			if(as1->Type()!=Service::VIDEO_SERVER)
				continue;
			SensorServer* sensorServer=(SensorServer*)as1->m_service;
			if(sensorServer->ConnectToClient(sensorClient)) {
				connected=true;
				break;
			}
		}
		if(connected) {
			sensorClient->AddFrameCallback(ac,[](uint8_t hostIndex,uint8_t streamId,uint8_t dataSourceType,const uint8_t* frame,int frameBytesize,int index,uint64_t captureTime,const DebugData& debugData,void* arg){
				//uprintf("connected frame callback frame %d streamId %d\n",index,streamId);
				ActiveClient* ac=(ActiveClient*)arg;
				ac->PushTime(debugData.m_timers[0],debugData.m_timers[1],0,0);
			});
			continue;
		}
		sensorClient->AddFrameCallback(ac,[this](uint8_t hostIndex,uint8_t streamId,uint8_t dataSourceType,const uint8_t* frame,int frameBytesize,int index,uint64_t captureTime,const DebugData& debugData,void* arg){
			ActiveClient* ac=(ActiveClient*)arg;
			//uprintf("client id %s frame %d streamId %d\n",ac->Id().c_str(),index,streamId);
				int offset=0;
				const uint8_t* data=frame;
				Dict streamInfo;
				streamInfo.ReadFromBinary(data,&offset);

				int totalTimers[2]={0};
				std::vector<Tensor> tensors;
				if(const Dict* binaryDataInfo=streamInfo.Find("binaryData")) {
					const uint8_t* binaryData=data+offset;
					for(auto&e:*binaryDataInfo) {
						std::string encoding;
						int streamId2,offset,size;
						e.Get("encoding",&encoding);
						e.Get("id",&streamId2);
						e.Get("offset",&offset);
						e.Get("size",&size);
						if(FrameDecoder* decoder=ac->GetOrCreateFrameDecoder(streamId2, encoding)) {
							tensors.emplace_back();
							decoder->DecodeFrame(&tensors.back(), binaryData+offset, size); // might leave tensor unchanged
							int timers[2]={0};
							decoder->GetTimers(timers, countof(timers));
							totalTimers[0]+=timers[0];
							totalTimers[1]+=timers[1];
						}else
						{
							tensors.push_back(CreateCPUTensor(1, &size, sizeof(uint8_t)));
							memcpy(tensors.back().m_data, binaryData+offset, size);
						}
					}
				}
				if(m_frameDecodedCallback.m_callbackTensor) {
					m_frameDecodedCallback.m_callbackTensor(ac->Id().c_str(),streamId,streamInfo,tensors,m_frameDecodedCallback.m_arg);
				}
				ac->PushTime(debugData.m_timers[0],debugData.m_timers[1],totalTimers[0],totalTimers[1]);
		});
	}
	return true;
}

void App::End() {
	if(m_httpServer) {
		m_httpServer->ClearWebsocketDataCallback();
	}
	for(ActiveService* as:m_activeServices) {
		delete as;
	}
	m_activeServices.clear();
	if(m_debugTransfer) {
		DestroyNetTransfer(m_debugTransfer);
		m_debugTransfer=0;
	}
	if(m_httpServer) {
		DestroyHttpServer(m_httpServer);
		m_httpServer=0;
	}
	for(DebugConnection* dc:m_debugConnections) {
		DestroyNetTransfer(dc->m_transfer);
		delete dc;
	}
	m_debugConnections.clear();
}

bool App::UpdateProperties(const std::string& id) {
	for(int i=0;i!=(int)m_debugConnections.size();i++) {
		DebugConnection* dc=m_debugConnections[i];
		dc->m_componentLock.lock();
		for(int j=0;j!=(int)dc->m_components.size();j++) {
			if(dc->m_components[j].m_id==id) {
				uint8_t buf[512];
				GetDebugPropertiesHeader* gph=(GetDebugPropertiesHeader*)buf;
				*gph=GetDebugPropertiesHeader();
				gph->m_loopback.m_debugConnectionIndex=i;
				gph->m_loopback.m_debugServiceIndex=j;
				gph->m_timeSend=GetTimeEpochMicroseconds();
				gph->m_dataBytesize=(uint32_t)id.size();
				memcpy(gph->Data(),id.c_str(),gph->m_dataBytesize);
				dc->m_transfer->SendToHost(gph,gph->PacketByteSize());
				dc->m_componentLock.unlock();
				return true;
			}
		}
		dc->m_componentLock.unlock();
	}
	return false;
}
bool App::AddActiveService(ActiveService* s) {
	std::string id=s->Id();
	if(id==Id()) {
			FATAL("A service attempted to use the already reserved device id: \"%s\"", id.c_str());
	}
	for(ActiveService* as:m_activeServices) {
		if(id==as->Id()||s==as) {
			FATAL("ActiveService with id: %s already registered", id.c_str());
			return false;
		}
	}
	m_activeServices.push_back(s);
	return true;
}
Service* App::FindService(const std::string& id) {
	if(id==Id()) {
		return this;
	}else{
		for(ActiveService* ac:m_activeServices) {
			if(id==ac->Id()) {
				return ac->m_service;
			}
		}
	}
	return 0;
}
bool App::FindConnection(const std::string& id,std::function<void(DebugConnection* dc,int connectionIndex,int componentIndex)> cb) const {
	for(int i=0;i!=(int)m_debugConnections.size();i++) {
		DebugConnection* dc=m_debugConnections[i];
		dc->m_componentLock.lock();
		for(int j=0;j!=(int)dc->m_components.size();j++) {
			if(dc->m_components[j].m_id==id) {
				cb(dc,i,j);
				dc->m_componentLock.unlock();
				return true;
			}
		}
		dc->m_componentLock.unlock();
	}
	return false;
}

bool App::HasConnection(const std::string& id) {
	return FindConnection(id,[](DebugConnection* d,int connectionIndex,int componentIndex){});
}

bool App::SetProperties(const Dict& properties,const std::string& id) {
	const std::string json=properties.WriteToJson(true);
	return FindConnection(id,[&](DebugConnection* dc,int connectionIndex,int componentIndex){
		dc->m_components[componentIndex].m_properties=properties;
		SetDebugPropertiesHeader dph;
		dph.m_timeSend=GetTimeEpochMicroseconds();
		dph.m_loopback.m_debugConnectionIndex=connectionIndex;
		dph.m_loopback.m_debugServiceIndex=componentIndex;
		dph.m_dataBytesize=(int)id.size()+1+(int)json.size();
		std::vector<char> p;
		p.resize(dph.m_headerBytesize+dph.m_dataBytesize);
		memcpy(p.data(),&dph,dph.m_headerBytesize);
		memcpy(p.data()+dph.m_headerBytesize,id.c_str(),(int)id.size()+1);
		memcpy(p.data()+dph.m_headerBytesize+id.size()+1,json.c_str(),(int)json.size());
		dc->m_transfer->SendToHost(p.data(),(int)p.size());
	});
}

bool App::GetProperties(Dict* properties,const std::string& id) {
	return FindConnection(id,[&](DebugConnection* dc,int connectionIndex,int componentIndex){
		properties->Copy(dc->m_components[componentIndex].m_properties);
	});
}

void App::SetStatusForConnection(const std::string& id,bool enabled) {
	for(DebugConnection* dc:m_debugConnections) {
		dc->m_componentLock.lock();
		for(int i=0;i!=(int)dc->m_components.size();i++) {
			if(dc->m_components[i].m_id==id) {
				dc->m_components[i].m_statusEnabled=enabled;
			}
		}
		dc->m_componentLock.unlock();
	}
}
void App::GetDebugConnections(Dict* dict)const {
	Dict* connections=dict->AddArrayNode("connections");
	for(DebugConnection* dc:m_debugConnections) {
		dc->m_componentLock.lock();
		Dict* connection=connections->PushBack();
		connection->Set("id",dc->m_transfer->Host());
		Dict* components=connection->AddArrayNode("components");
		for(int i=0;i!=(int)dc->m_components.size();i++) {
			Dict* component=components->PushBack();
			component->Set("id",dc->m_components[i].m_id);
			if(dc->m_components[i].m_status.First() && dc->m_components[i].m_statusEnabled) {
				Dict* status=component->AddObjectNode("status");
				status->Copy(dc->m_components[i].m_status);
				status->SetName("status");
			}else{
				component->AddObjectNode("status");
			}
		}
		dc->m_componentLock.unlock();
		Dict* log=connection->AddObjectNode("log");
		Dict* entriesDict=log->AddArrayNode("entries");
		std::vector<HistoryLog::Entry> entries;
		//dc->m_log.GetNewEntries(&entries,0);
		dc->m_log.GetLastEntries(&entries,30);
		for(const auto& e:entries) {
			Dict* entryDict=entriesDict->AddObjectNode();
			entryDict->Set("str",e.m_str);
		}
	}

}
void App::GetLog(Dict* log,const Dict& request) {
	//status->ReadFromJson("{\"schema\":[{\"param\":\"tal\",\"type\":\"int\"}],\"device\":{},\"graphs\":[]}");
	//request.Dump();
	double dt;
	if(!request.Get("time",&dt)) {
		uprintf("App::GetLog request has no time\n");
		return;
	}
	uint64_t time=(uint64_t)dt;
	std::vector<HistoryLog::Entry> entries;
	time=m_log.GetNewEntries(&entries,time);
	log->Set("time",(int64_t)time);
	Dict* entriesDict=log->AddArrayNode("entries");
	for(const auto& e:entries) {
		Dict* entryDict=entriesDict->AddObjectNode();
		entryDict->Set("time",(int64_t)e.m_time);
		entryDict->Set("str",e.m_str);
	}
}

void App::GetStatus(Dict* status,bool includeSchema,bool includeGraphs) {

std::string statusString=R"({
	"schema":[
		{
			"param":"tal",
			"type":"int"
		}
	],
	"graphs":[]
})";

	status->ReadFromJson(statusString);
	status->Set("id",m_id);

	if(m_httpServer) {
		std::string bindAddress=m_httpServer->GetBindAddress();
		status->Set("httpserver",bindAddress);
	}
	Dict* graphs=status->Find("graphs");
	if(!graphs)
		return;

	uint64_t time=GetTimeEpochMicroseconds();
	uint64_t timeMilliseconds=time/1000L;
    const char* colors[]={"#dd8452","#68a855","#524ec4","#b37281","#607893","#c38bda","#8c8c8c","#74b9cc","#cdb564","#b0724c"};



	int cnt=0;
	for(ActiveService* as:m_activeServices) {
		if(as->Type()!=Service::VIDEO_CLIENT)
			continue;
		ActiveClient* ac=(ActiveClient*)as;

		Dict* graph=graphs->PushBack();
		graph->Set("xmax",(int64_t)timeMilliseconds);
		graph->Set("xmin",(int64_t)(timeMilliseconds-(TIMERS_AGE/1000)));

		graph->Set("formatX","HH:MM:SS");
		graph->Set("formatY","us");
		Dict* legend=graph->AddObjectNode("legend");
		legend->Set("text",stdx::format_string("%s-timers",ac->Id().c_str()).c_str());
		legend->Set("color",colors[cnt]);
		legend->Set("timers",true);
		Dict* dataset=graph->AddObjectNode("dataset");
		Dict* infos=dataset->AddArrayNode("infos");
		for(int i=0;i!=(int)ac->m_timers.size();i++) {
			Dict* info=infos->PushBack();
			info->Set("name",ac->m_timers[i].m_name);
			info->Set("color",Color2String(ac->m_timers[i].m_color));
		}
		dataset->Set("stacked",(int)ac->m_timers.size());

		//for(int i=0;i!=4;i++) {
		//	info->Set("name","decode");
		//	info->Set("color",colors[i]);
		//}
		//dataset->Set("name","decode");
		//dataset->Set("color",colors[cnt]);
		std::vector<double> datax;
		std::vector<double> datay;

		ac->m_timersLock.lock();
		datax.reserve(ac->m_measurements.size());
		datay.reserve(ac->m_measurements.size()*4);
		for(int i=0;i!=(int)ac->m_measurements.size();i++) {
			datax.push_back((double)(ac->m_measurements[i].m_time/1000L));
			datay.push_back((double)ac->m_measurements[i].m_values[0]);
			datay.push_back((double)ac->m_measurements[i].m_values[1]);
			datay.push_back((double)ac->m_measurements[i].m_values[2]);
			datay.push_back((double)ac->m_measurements[i].m_values[3]);
		}
		ac->m_timersLock.unlock();
		dataset->SetTypedArray("datax",datax.data(),datax.size());
		dataset->SetTypedArray("datay",datay.data(),datay.size());
		cnt++;
	}

	if(m_debugTiming.m_host.size()) {
		Dict* graph=graphs->PushBack();
		graph->Set("xmax",(int64_t)timeMilliseconds);
		graph->Set("xmin",(int64_t)(timeMilliseconds-(TIMERS_AGE/1000)));

		graph->Set("formatX","HH:MM:SS");
		graph->Set("formatY","us");
		Dict* legend=graph->AddObjectNode("legend");
		legend->Set("text",stdx::format_string("debugTiming").c_str());
		legend->Set("color",colors[cnt]);
		legend->Set("timers",true);
		Dict* dataset=graph->AddObjectNode("dataset");
		Dict* infos=dataset->AddArrayNode("infos");
		for(int i=0;i!=(int)m_debugTiming.m_timers.m_timers.size();i++) {
			Dict* info=infos->PushBack();
			info->Set("name",m_debugTiming.m_timers.m_timers[i].m_name);
			info->Set("color",Color2String(m_debugTiming.m_timers.m_timers[i].m_color));
		}
		dataset->Set("stacked",(int)m_debugTiming.m_timers.m_timers.size());

		//for(int i=0;i!=4;i++) {
		//	info->Set("name","decode");
		//	info->Set("color",colors[i]);
		//}
		//dataset->Set("name","decode");
		//dataset->Set("color",colors[cnt]);
		std::vector<double> datax;
		std::vector<double> datay;

		m_debugTiming.m_timers.m_timersLock.lock();
		datax.reserve(m_debugTiming.m_timers.m_measurements.size());
		datay.reserve(m_debugTiming.m_timers.m_measurements.size()*4);
		for(int i=0;i!=(int)m_debugTiming.m_timers.m_measurements.size();i++) {
			datax.push_back((double)(m_debugTiming.m_timers.m_measurements[i].m_time/1000L));
			datay.push_back((double)m_debugTiming.m_timers.m_measurements[i].m_values[0]);
			datay.push_back((double)m_debugTiming.m_timers.m_measurements[i].m_values[1]);
			datay.push_back((double)m_debugTiming.m_timers.m_measurements[i].m_values[2]);
			datay.push_back((double)m_debugTiming.m_timers.m_measurements[i].m_values[3]);
		}
		m_debugTiming.m_timers.m_timersLock.unlock();
		dataset->SetTypedArray("datax",datax.data(),datax.size());
		dataset->SetTypedArray("datay",datay.data(),datay.size());
	}
}


