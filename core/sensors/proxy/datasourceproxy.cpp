#include "core/datasource.h"
#include "core/sensorclient.h"
#include "shared/queue.h"
#include <atomic>
#include <thread>


class DataSourceProxy : public DataSource {
	public:
		DataSourceProxy();
		virtual ~DataSourceProxy();
		virtual uint8_t Type()const{return DS_TYPE_PROXY;}
		virtual uint8_t Id()const{return m_id;}

		virtual bool HasClock()const{return false;}
		virtual void RunCaptureLoop(TCaptureFunc cb,void* arg);
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);
	protected:
		SensorClient* m_client;
		uint8_t m_id;
		std::atomic<bool> m_close;
		std::thread m_thread;
		struct Frame {
			int m_index;
			std::vector<char> m_data;
		};
		FixedQueueMT<Frame,8> m_frames;
		static DataSource* Create() {return new DataSourceProxy();}
		static const bool s_registered;
};

const bool DataSourceProxy::s_registered=RegisterDataSource("proxy",DataSourceProxy::Create);

DataSourceProxy::DataSourceProxy() : DataSource() {
	m_close=false;
}
DataSourceProxy::~DataSourceProxy() {
}
bool DataSourceProxy::Begin(const Dict& dict) {
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceProxy no id");
	if(m_id >= STREAM_ID_MAX)
		FATAL("stream id must fit in %d bits", STREAM_ID_BITS);
	const Dict* transferDict=dict.Find("transfer");
	if(!transferDict)
		FATAL("DataSourceProxy no transfer");
	Dict clientDict;
	clientDict.ReadFromJson(R"({"id":"dummy"})");
	clientDict.InsertCopy(*transferDict);
	m_client=CreateSensorClient(&clientDict);
	m_client->AddFrameCallback(this,[](uint8_t hostIndex,uint8_t streamId,uint8_t dataSourceType,const uint8_t* data,int dataBytesize,int index,uint64_t captureTime,const DebugData& debugData,void* arg){
		DataSourceProxy* p=(DataSourceProxy*)arg;
		if(!p->m_frames.Push([&](Frame* frame){
			frame->m_index=index;
			frame->m_data.resize(dataBytesize);
			memcpy(frame->m_data.data(),data,dataBytesize);
		})){
			uprintf("DataSourceProxy frames queue full\n");
		}
		//uprintf("DataSourceProxy got frame %d\n",index);
	});
	return true;
}
void DataSourceProxy::End() {
	m_close=true;
	DestroySensorClient(m_client);
}
void DataSourceProxy::RunCaptureLoop(TCaptureFunc cb,void* arg) {
	FATAL("DataSourceProxy::RunCaptureLoop");
}
bool DataSourceProxy::HasFrameData(int clockIndex)const{
	if(m_frames.Size())
		return true;
	return false;
}
void DataSourceProxy::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	//FATAL("DataSourceProxy::GetFrameData");
	while(m_frames.Size()) {
		if(m_frames.Pop(1000,[&](Frame* frame){
			*dataIndex=frame->m_index;
			data->insert(data->begin(),frame->m_data.begin(),frame->m_data.end());
		})){
		}
	}
}

int datasource_proxy_static_link_use=0;
