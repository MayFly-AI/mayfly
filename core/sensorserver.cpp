#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <map>
#include <algorithm>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"
#include "shared/dict.h"
#include "shared/thread.h"

#include "camera/synccameraencoder.h"
#include "camera/options.h"

#include "app.h"
#include "transfer.h"
#include "errorcorrection.h"
#include "packet.h"
#include "sensorserver.h"

#include "service.h"
#include "datasource.h"


//SensorServerImpl
class SensorServerImpl : public SensorServer {
	public:
		virtual ~SensorServerImpl(){}
		virtual bool SaveConfig(Dict* config);
		virtual bool Begin(Dict* dict);
		virtual void End();
		virtual void GetStatus(Dict* status,bool includeSchema,bool includeGraphs);
		virtual const std::string& Id()const{return m_id;}
		virtual bool ConnectToClient(SensorClient* sensorClient);
		virtual void SetFrameEncodedCallback(void* arg,TFrameEncodedCallbackFunc cb){m_frameEncodedCallbackArg=arg;m_frameEncodedCallback=cb;}
		virtual void GetProperties(Dict* properties);
		virtual void SetProperties(const Dict& properties);

		//void PushFrame(const void* frame,int frameBytesize,int index,uint64_t timeCapture);
	protected:
		void* m_frameEncodedCallbackArg;
		TFrameEncodedCallbackFunc m_frameEncodedCallback=0;

		DataSource* m_dataSourceClock=0;
		std::vector<DataSource*> m_dataSources;

		Dict m_settings;

		std::string m_clientSourceId;

		std::thread m_threadSend;
		std::string m_id;
		bool m_verbose=false;
		std::atomic<bool> m_close;

		void OnData(NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived);

		//Active connections
		struct ActiveClient {
			std::string m_sourceName;
			uint64_t m_timeStart=0;
			uint64_t m_timeLastData=0;
			uint64_t m_timeClientLastPacket=0;
			uint64_t m_timeServerLastPacket=0;
			uint64_t m_lastPingTimeReceived=0;
			uint64_t m_lastPingLoopback=0;
			uint32_t m_lastPingCount=0;
		};
		mutable std::mutex m_activeClientLock;
		std::map<std::string,ActiveClient> m_activeClients;

		int CountClients()const{std::scoped_lock sl(m_activeClientLock);return(int)m_activeClients.size();}
		void GetActiveClients(std::vector<ActiveClient>* activeClients)const;
		void TimeoutActiveClients();
		void SendToAll(uint8_t streamId,uint8_t dataSourceType,uint32_t index,const uint8_t* data,int dataBytesize,const DebugData& debugData,bool last);

		NetTransfer* m_transfer=0;
		ErrorCorrectionEncoder* m_encoder=0;

		//Sources
		void BeginSources(const Dict& sourcesDict);
		void EndSources();
};

SensorServer* CreateSensorServer(Dict* dict) {
	SensorServerImpl* sensorServer=new SensorServerImpl;
	sensorServer->Begin(dict);
	return sensorServer;
}
void DestroySensorServer(SensorServer* sensorServer) {
	sensorServer->End();
	delete sensorServer;
}

bool SensorServerImpl::Begin(Dict* dict) {
	m_settings.Copy(*dict);
	//uprintf("SensorServer settings:\n");
	dict->Get("verbose",&m_verbose,false);
	dict->Get("id", &m_id, "NA");
	m_close=false;

	Dict* encode=dict->Find("errorCorrection");
	m_encoder=CreateErrorCorrectionEncoder(encode);

	Dict* transfer=dict->Find("transfer");
	m_transfer=CreateNetTransfer(transfer);

	m_transfer->SetErrorCallback([&](NetTransfer* transfer,const char* sourceName,const char* errorName){
		m_activeClientLock.lock();
		auto r=m_activeClients.find(sourceName);
		if(r==m_activeClients.end()) {
			if(strcmp(errorName,"RESET"))
				uprintf("Transfer error callback unregistered source %s error %s\n",sourceName,errorName);
		}else{
			uprintf("Transfer error callback source %s error %s\n",sourceName,errorName);
			m_activeClients.erase(r);
		}
		m_activeClientLock.unlock();
	});

	m_transfer->SetDataCallback([&](NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived){
		uint64_t dt=GetTimeEpochMicroseconds()-timeReceived;
		if(dt>200) {
			uprintf("SensorServer receive data took %dus before OnData\n",(int)dt);
		}
		OnData(m_transfer,data,dataBytesize,sourceName,timeReceived);
		dt=GetTimeEpochMicroseconds()-timeReceived;
		if(dt>200) {
			uprintf("SensorServer receive data took %dus\n",(int)dt);
		}
	});
	Dict* sourcesDict=dict->Find("sources");
	if(sourcesDict) {
		BeginSources(*sourcesDict);
	}
	return true;
}
void SensorServerImpl::End() {
	m_close=true;
	EndSources();
	if(m_threadSend.joinable())
		m_threadSend.join();
	if(m_transfer) {
		DestroyNetTransfer(m_transfer);
		m_transfer=0;
	}
	DestroyErrorCorrectionEncoder(m_encoder);
	//uprintf("SensorServerImpl::End\n");
}
void SensorServerImpl::OnData(NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived) {
	//uprintf("SensorServerImpl::OnData len %d from %s\n",dataBytesize,source.ToString().c_str());
	PacketHeader* header=(PacketHeader*)data;
	header->SanityCheck(dataBytesize);
	if(header->IsDebugPacket()) {
		FATAL("received debug packet\n");
	}
	m_activeClientLock.lock();
	ActiveClient as={sourceName};
	auto r=m_activeClients.insert(std::make_pair(sourceName,as));
	ActiveClient* ac=&r.first->second;
	if(r.second) {
		ac->m_timeStart=timeReceived;
		uprintf("server %s client begin %s\n",Id().c_str(),sourceName);
	}
	ac->m_timeLastData=timeReceived;
	ac->m_timeClientLastPacket=header->m_timeSend;
	ac->m_timeServerLastPacket=timeReceived;
	if(header->m_type==PacketHeader::FROM_CLIENT_PING) {
		PingFrameHeader* ping=(PingFrameHeader*)data;
		ac->m_lastPingLoopback=ping->m_timeSend;
		ac->m_lastPingCount=ping->m_count;
		ac->m_lastPingTimeReceived=timeReceived;
	}
	m_activeClientLock.unlock();
	switch(header->m_type){
		case PacketHeader::FROM_CLIENT_REQUEST_MISSING_CHUNKS: {
			const RequestMissingChunksHeader* mch=(const RequestMissingChunksHeader*)header;
			std::vector<std::vector<uint8_t>> fragments;
			m_encoder->EncodeRetransmit(&fragments,((uint8_t*)data)+header->m_headerBytesize,dataBytesize-header->m_headerBytesize);
			char buf[4096];
			for(int i=0;i!=(int)fragments.size();i++) {
				if(fragments[i].size()+sizeof(PacketHeaderFrame)>sizeof(buf)) {
					FATAL("block size exceeds buffer");
				}
				PacketHeaderFrame* phb=(PacketHeaderFrame*)buf;
				*phb=PacketHeaderFrame();
				phb->m_type=PacketHeaderBase::FROM_SERVER_FRAME_CHUNK_RESEND;
				phb->m_dataSourceType=mch->m_dataSourceType;
				phb->m_errorCorrectionType=m_encoder->Type();
				phb->m_streamId=mch->m_streamId;
				memcpy(phb+1,fragments[i].data(),fragments[i].size());
				m_transfer->SendTo(sourceName,buf,(int)sizeof(PacketHeaderFrame)+(int)fragments[i].size());
			}
			break;
		}
		case PacketHeader::CLOCK_MASTER_TIME_SEND: {
			const ClockMasterTimeSendHeader* cm=(const ClockMasterTimeSendHeader*)header;
			int64_t deltaTime = cm->m_masterClockMicroseconds-timeReceived;
			if(m_dataSourceClock->Id() == cm->m_streamId && m_dataSourceClock->Type() == cm->m_dataSourceType) {
				m_dataSourceClock->SetTimeOffset(deltaTime);
				//uprintf("master: %lu,local: %lu,delta: %ld\n",cm->m_masterClockMicroseconds,timeReceived,deltaTime);
			}
			break;
		}
		case PacketHeader::FROM_CLIENT_PING:
		case PacketHeader::FROM_CLIENT_REQUEST_FRAMES:
			break;
		default:
			uprintf("SensorServerImpl::OnData received invalid packet type %d\n",header->m_type);
	}
}

int talval=30;
std::string mode="two";

void SensorServerImpl::GetProperties(Dict* properties) {
	std::string test=R"({
		"schema":{
			"properties": {
				"talval":{
					"type":"integer",
					"minimum":10,
					"maximum":120,
					"default":30,
					"step":1
				},
				"mode":{
					"type": "string",
					"items": {
						"type": "string",
						"enum": ["one", "two", "three"]
					},
					"default": "three"
				}
			}
		}
	})";
	Dict* server0Dict=properties->AddObjectNode(Id());
	Dict dict;
	dict.ReadFromJson(test);
	server0Dict->Set(dict);
	Dict* values=server0Dict->AddObjectNode("values");
	values->Set("talval",talval);
	values->Set("mode",mode);

	if(m_dataSourceClock->Type()!=DS_TYPE_NA)
		m_dataSourceClock->GetProperties(properties);
	for(DataSource* ds:m_dataSources) {
		ds->GetProperties(properties);
	}
}
void SensorServerImpl::SetProperties(const Dict& properties) {

	const Dict* server0Dict=properties.Find(Id());
	bool modified=false;
	if(server0Dict && server0Dict->Get("modified",&modified) && modified) {
		const Dict* values=server0Dict->Find("values");
		values->Get("talval",&talval);
		values->Get("mode",&mode);
		uprintf("talval %d mode %s\n",talval,mode.c_str());
	}
	if(m_dataSourceClock->Type()!=DS_TYPE_NA)
		m_dataSourceClock->SetProperties(properties);
	for(DataSource* ds:m_dataSources) {
		ds->SetProperties(properties);
	}
}

bool SensorServerImpl::SaveConfig(Dict* config){
	std::string id;
	config->Get("id",&id);
	uprintf("SensorServerImpl::SaveConfig %s %s\n",Id().c_str(),id.c_str());
	//Dict* source=config->Find("source");
	//if(source) {
	//	source->Set("mode",m_source.m_mode);
	//	source->Set("fps",m_source.m_fps);
	//}
	return true;
}

void SensorServerImpl::GetStatus(Dict* status,bool includeSchema,bool includeGraphs) {
	if(includeSchema) {
		status->ReadFromJson("{\"schema\":[{\"param\":\"tal\",\"type\":\"int\"}],\"server\":{},\"graphs\":[]}");
		Dict* schema=status->Find("schema");
		Dict* node=schema->PushBack();
		node->Set("param","packets received");
		node->Set("type","int");
	}else{
		status->ReadFromJson("{\"server\":{}}");
	}
	Dict* settings=status->Find("server");
	std::string id;
	m_settings.Get("id",&id,"NA");
	settings->Set("id",id);
	if(m_owner) {
		std::string deviceId=m_owner->Id();
		settings->Set("deviceId",deviceId);
	}
	std::vector<SensorServerImpl::ActiveClient> activeClients;
	GetActiveClients(&activeClients);

	Dict* dataSourcesDict=settings->AddArrayNode("sources");
	auto dataSources=m_dataSources;
	if(m_dataSourceClock->Type()!=DS_TYPE_NA)
		dataSources.push_back(m_dataSourceClock);
	for(DataSource* ds:dataSources) {
		Dict* dataSourceDict=dataSourcesDict->PushBack();
		dataSourceDict->Set("type",DataSourceTypeToName(ds->Type()));
		dataSourceDict->Set("id",ds->Id());
	}

	Dict* clients=settings->AddArrayNode("clients");
	for(auto ac:activeClients) {
		Dict* element=clients->PushBack();
		element->Set("ip",ac.m_sourceName);
		element->Set("start",(int)(ac.m_timeStart));
		element->Set("last",(int)(ac.m_timeLastData));
	}
	m_transfer->GetStatus(settings->AddObjectNode("transfer"));

	settings->Set("packets received",m_transfer->m_packetsReceived);
	settings->Set("packets send",m_transfer->m_packetsSend);
	settings->Set("bytes received",m_transfer->m_bytesReceived);
	settings->Set("bytes send",m_transfer->m_bytesSend);

	Dict* graphs=status->Find("graphs");
	if(!graphs)
		return;
	uint64_t time=GetTimeEpochMicroseconds();
	uint64_t timeMilliseconds=time/1000L;
	//Dict* graph=graphs->PushBack();
	m_transfer->GetGraphs(graphs,timeMilliseconds);
}

void SensorServerImpl::GetActiveClients(std::vector<ActiveClient>* activeClients)const{
	std::scoped_lock sl(m_activeClientLock);
	activeClients->clear();
	for(auto it=m_activeClients.begin();it!=m_activeClients.end();it++) {
		const ActiveClient* ac=&it->second;
		activeClients->push_back(*ac);
	}
}
void SensorServerImpl::TimeoutActiveClients() {
	std::scoped_lock sl(m_activeClientLock);
	uint64_t time=GetTimeEpochMicroseconds();
	for(auto it=m_activeClients.begin();it!=m_activeClients.end();) {
		const ActiveClient& ac=it->second;
		if(time-ac.m_timeLastData>(30*1000000)) {
			//uprintf("client timeout %s\n",ac.m_sourceName.c_str());
			it=m_activeClients.erase(it);
		}else{
			it++;
		}
	}
	m_encoder->Timeout();
}
/*
void ValidateDecode(uint8_t subType,const std::vector<std::vector<uint8_t>>& fragments,const std::vector<char>& frame) {
	Dict dict;
	dict.ReadFromJson(R"({"type":"fec"})");
	ErrorCorrectionDecoder* decoder=CreateErrorCorrectionDecoder(&dict);
	for(auto& fragment:fragments) {
		decoder->Decode(subType,fragment.data(),(int)fragment.size(),GetTimeEpochMicroseconds(),[&](int index,const uint8_t* data,int dataBytesize,const DebugData& debugData,uint64_t timeFirst){
			std::vector<char> frame1((char*)data,(char*)data+dataBytesize);
			if(frame!=frame1) {
				FATAL("not same");
			}else{
				uprintf("ok %d sz %d\n",index,dataBytesize);
			}
		});
	}
}
*/
void SensorServerImpl::SendToAll(uint8_t streamId,uint8_t dataSourceType,uint32_t index,const uint8_t* data,int dataBytesize,const DebugData& debugData,bool last) {
	if(!DataSourceTypeValid(dataSourceType))
		FATAL("SensorServerImpl::SendToAll dataSourceType %d not valid",dataSourceType);

	std::vector<std::vector<uint8_t>> fragments;
	m_encoder->Encode(&fragments,index,data,dataBytesize,debugData,streamId);
	//ValidateDecode(fragments,std::vector<char>((char*)data,(char*)data+dataBytesize));
	ActiveClient activeClients[64];
	m_activeClientLock.lock();
	if((int)m_activeClients.size()>countof(activeClients)) {
		FATAL("SensorServerImpl::SendToAll too many active clients %d",(int)m_activeClients.size());
	}
	int cnt=0;
	for(auto &[key,as]:m_activeClients) {
		activeClients[cnt++]=as;
		as.m_lastPingTimeReceived=0;
	}
	m_activeClientLock.unlock();
	char buf[4096];
	for(int i=0;i!=cnt;i++) {
		const ActiveClient& as=activeClients[i];
		uint64_t timeReceived=as.m_lastPingTimeReceived;
		if(timeReceived) {
			PongFrameHeader pong;
			pong.m_timeLoopback=as.m_lastPingLoopback;
			pong.m_timeSend=GetTimeEpochMicroseconds();
			pong.m_count=as.m_lastPingCount;
			pong.m_timeSpendServer=(uint32_t)(pong.m_timeSend-timeReceived);
			m_transfer->SendTo(as.m_sourceName.c_str(),&pong,sizeof(pong));
			//uprintf("Send pong! Time spend on server %d\n",pong.m_timeSpendServer);
		}
	}
	PacketHeaderFrame* phb=(PacketHeaderFrame*)buf;
	*phb=PacketHeaderFrame();
	phb->m_dataSourceType=dataSourceType;
	phb->m_errorCorrectionType=m_encoder->Type();
	phb->m_streamId=streamId;

	if(phb->m_dataSourceType!=dataSourceType)
		FATAL("SensorServerImpl::SendToAll dataSourceType %d!=%d does not match after header store",phb->m_dataSourceType,dataSourceType);
	if(phb->m_streamId!=streamId)
		FATAL("SensorServerImpl::SendToAll streamId %d!=%d does not match after header store",phb->m_streamId,streamId);

	if(m_transfer->IsDestBroadcast()) {
		for(int i=0;i!=(int)fragments.size();i++) {
			memcpy(phb+1,fragments[i].data(),fragments[i].size());
			phb->m_last=last && i==(int)fragments.size()-1 ? true:false;
			m_transfer->SendToHost(buf,(int)sizeof(PacketHeaderFrame)+(int)fragments[i].size());
		}
	}else{
		for(int i=0;i!=cnt;i++) {
			const ActiveClient& as=activeClients[i];
			for(int j=0;j!=(int)fragments.size();j++) {
				memcpy(phb+1,fragments[j].data(),fragments[j].size());
				phb->m_last=last && j==(int)fragments.size()-1 ? true:false;
				m_transfer->SendTo(as.m_sourceName.c_str(),buf,(int)sizeof(PacketHeaderFrame)+(int)fragments[j].size());
			}
		}
	}
}

void SensorServerImpl::EndSources() {
	if(m_dataSourceClock) {
		DestroyDataSource(m_dataSourceClock);
	}
	for(DataSource* ds:m_dataSources) {
		DestroyDataSource(ds);
	}
}
void SensorServerImpl::BeginSources(const Dict& sourcesDict) {
	for(auto it=sourcesDict.begin();it!=sourcesDict.end();++it) {
		const Dict& sourceDict=*it;
		std::string type;
		sourceDict.Get("type",&type);
		if(type=="client") {
			if(!m_clientSourceId.empty()) {
				FATAL("Source client already set to: %s", m_clientSourceId.c_str());
			}
			sourceDict.Get("client",&m_clientSourceId);
			if(m_clientSourceId.empty()) {
				FATAL("Sources type client has no client source");
			}
		}else
		{
			DataSource* ds=CreateDataSource(sourceDict);
			if(ds) {
				if(ds->HasClock()) {
					if(m_dataSourceClock) {
						FATAL("Multiple datasource with clock. Only one is possible");
					}
					m_dataSourceClock=ds;
				}else{
					m_dataSources.push_back(ds);
				}
			}
		}
	}
	if(!m_dataSourceClock) {
		Dict dict;
		dict.ReadFromJson(R"({"type":"defaultclock","clock":30})");
		m_dataSourceClock=CreateDataSource(dict);
		m_dataSourceClock->Begin(dict);
	}
	m_dataSourceClock->RunCaptureLoop([](DataSource* dataSource,uint64_t timeCapture,int clockIndex,void* arg){
		SensorServerImpl* vs=(SensorServerImpl*)arg;
		vs->TimeoutActiveClients();
		int timer0=(int)ElapsedMicroseconds(timeCapture);
		DebugData debugData={timeCapture,{timer0,0}};
		std::vector<DataSource*> dataSources;
		for(DataSource* ds:vs->m_dataSources) {
			if(ds->HasFrameData(clockIndex))
				dataSources.push_back(ds);
		}
		if(dataSource->HasFrameData(clockIndex))
			dataSources.push_back(dataSource);
		std::vector<uint8_t> data1;
		for(int i=0;i!=(int)dataSources.size();i++) {
			DataSource* ds=dataSources[i];
			bool last=i==(int)dataSources.size()-1;
			int dataIndex=0;
			data1.clear();
			ds->GetFrameData(&data1,&dataIndex,clockIndex);
			//vs->SendToAll(ds->Id(),ds->Format(),ds->Type(),dataIndex,data1.data(),(int)data1.size(),debugData,last);
			vs->SendToAll(ds->Id(),ds->Type(),dataIndex,data1.data(),(int)data1.size(),debugData,last);
		}
	},this);
}

bool SensorServerImpl::ConnectToClient(SensorClient* sensorClient) {
	if(!m_clientSourceId.size() || m_clientSourceId!=sensorClient->Id())
		return false;
	uprintf("connect server id %s to client id %s\n",Id().c_str(),m_clientSourceId.c_str());
	sensorClient->AddFrameCallback(this,[](uint8_t hostIndex,uint8_t streamId,uint8_t dataSourceType,const uint8_t* frame,int frameBytesize,int index,uint64_t timeCapture,const DebugData& debugData,void* arg){
		//uprintf("push frame %d to server\n",index);
		SensorServerImpl* SensorServer=(SensorServerImpl*)arg;
		//if(!(index%100))
		//	uprintf("server %s send frame %d size %d\n",SensorServer->Id().c_str(),index,frameBytesize);
		SensorServer->SendToAll(streamId,dataSourceType,index,frame,frameBytesize,debugData,true);
	});
	return true;
}
