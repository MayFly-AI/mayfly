#include "core/datasource.h"
#include "core/app.h"
#include "core/sensorclient.h"
#include "core/transfer.h"
#include "camera/synccameraencoder.h"
#include "camera/options.h"
#include <atomic>
#include <thread>

class DataSourceSyncCameraMaster : public DataSource {
public:
	DataSourceSyncCameraMaster() : DataSource() {}
	virtual ~DataSourceSyncCameraMaster() {}

	virtual uint8_t Type()const{return DS_TYPE_SYNCCAMERA_MASTER;}
	virtual uint8_t Id()const{return m_id;}
	virtual bool HasClock()const{return true;}
	virtual void RunCaptureLoop(TCaptureFunc cb,void* arg);
	virtual bool Begin(const Dict& dict);
	virtual void End();
	virtual bool HasFrameData(int clockIndex)const;
	virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);

	struct FrameInfo {
		int streamId;
		int frameIndex;
		uint64_t captureTime;
		uint8_t type;
		std::vector<char> data;
	};

	struct FrameCache {
		void Reserve(size_t n) {
			outgoing.reserve(n);
			count=n;
		}
		void Add(FrameInfo&& info,uint64_t timeout) {
			std::unique_lock lock(mtx);
			auto it=begin(outgoing);
			while(it!=end(outgoing)) {
				if(it->streamId==info.streamId) {
					//uprintf("new frame from stream %d - flushing all frames\n",(int)info.streamId);
					outgoing.clear();
					break;
				}
			  if(it->captureTime < info.captureTime - timeout) {
					uprintf("evicting old frame from %s stream %d\n",it->streamId==info.streamId?"same":"other",(int)it->streamId);
					it = outgoing.erase(it);
				}
				else {
					++it;
				}
			}
			outgoing.push_back(info);
		}
		bool Pop(std::function<void(FrameInfo&&)> pop_func) {
			std::unique_lock lock(mtx);
			if(outgoing.size() < count)
				return false;
			for(auto& e:outgoing) {
				pop_func(std::move(e));
			}
			outgoing.clear();
			return true;
		}
		std::vector<FrameInfo> outgoing;
		std::mutex mtx;
		size_t count{0};
	};

	FrameCache m_cache;
	std::vector<FrameInfo> m_send;
	std::atomic<bool> m_hasAllFrames{false};
	int m_frameIndex{0};

	TCaptureFunc m_callbackFrameReady;
	void* m_callbackFrameReadyArg;
	bool m_close{false};

	std::mutex m_send_mtx;
	std::condition_variable m_send_cond;

	uint8_t m_id{0};
	int m_clock{0};
	std::vector<SensorClient*> m_clients;
	std::thread m_master;
	static DataSource* Create() {return new DataSourceSyncCameraMaster();}
	static const bool s_registered;
};

const bool DataSourceSyncCameraMaster::s_registered=RegisterDataSource("synccameramaster",DataSourceSyncCameraMaster::Create);

bool DataSourceSyncCameraMaster::Begin(const Dict& dict) {
	if(!dict.Get("clock",&m_clock))
		FATAL("missing 'clock'");
	if(!dict.Get("id",(char*)&m_id))
		FATAL("Missing 'id'");
	if(m_id >= STREAM_ID_MAX)
		FATAL("stream id must fit in %d bits",STREAM_ID_BITS);
	const Dict* servers=dict.Find("servers");
	if(!servers)
		FATAL("missing 'servers'");
	m_cache.Reserve(servers->Children());
	uint64_t timeout = (uint64_t)(1e6f/m_clock * 0.6f);

	for(const Dict& d : *servers) {
		Dict transferDict(d);
		SensorClient* vc=CreateSensorClient(&transferDict);
		m_clients.push_back(vc);
		vc->AddFrameCallback(this,[this,timeout](uint8_t hostIndex,uint8_t streamId,uint8_t dataSourceType,const uint8_t* data,int dataBytesize,int index,uint64_t captureTime,const DebugData& debugData,void* arg){
			int dictOffset=0;
			Dict recv;
			recv.ReadFromBinary(data,&dictOffset);

			if(0) {
				const int64_t rate=33333;
				int64_t rem=captureTime%rate;
				rem=rem>rate/2?rem-rate:rem;
				auto now=GetTimeEpochMicroseconds();
				uprintf("got frame: %6d, time: %lu, drift: %8ld, age: %8lu)\n",index,captureTime,rem,(now-captureTime));
			}

			FrameInfo info;
			info.streamId=streamId;
			info.captureTime=captureTime;
			info.frameIndex=index;
			info.type=dataSourceType;
			info.data.assign((char*)data+dictOffset,(char*)data+dataBytesize);
			m_cache.Add(std::move(info),timeout);

			uint64_t tmin=UINT64_MAX;
			uint64_t tmax=0;
			std::vector<FrameInfo> send;
			bool hasAllFrames=m_cache.Pop([&](FrameInfo&& e){
				//auto now=GetTimeEpochMicroseconds();
				//uprintf("e.captureTime = %llu  now = %llu  delta = %llu\n",e.captureTime, now, now-e.captureTime);
				tmin=MIN(tmin,e.captureTime);
				tmax=MAX(tmax,e.captureTime);
				send.push_back(std::move(e));
			});
			if(hasAllFrames){
				m_frameIndex++;
				{
					std::lock_guard<std::mutex> lock(m_send_mtx);
					m_send=std::move(send);
				}
				m_hasAllFrames=true;
				m_send_cond.notify_one();
			}
		});
		vc->AddLastMessageCallback(this,[](int frameIndex,uint8_t streamId,uint8_t dataSourceType,NetTransfer* transfer,void* arg){
//			uprintf("got last message: %d from %s:%lu\n",frameIndex,transfer->Host().c_str());
			ClockMasterTimeSendHeader hd;
			hd.m_streamId=streamId;
			hd.m_dataSourceType=dataSourceType;
			hd.m_masterClockMicroseconds=GetTimeEpochMicroseconds();
			transfer->SendToHost(&hd,sizeof(hd));
		});
	}

	return true;
}
bool DataSourceSyncCameraMaster::HasFrameData(int clockIndex)const{
	return m_hasAllFrames;
}
void DataSourceSyncCameraMaster::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex){
	// We're assuming this method is called while m_send_mtx is held by RunCaptureLoop
	Dict info;
	Dict& frame=info.Add("source");
	auto now=GetTimeEpochMicroseconds();
	frame.Set("type",DataSourceTypeToName(Type()));
	frame.Set("id", m_id);
	frame.Set("time", (int64_t)now);
	frame.Set("index", m_frameIndex);

	Dict* sync=frame.AddArrayNode("frames");
	Dict* bdat=info.AddArrayNode("binaryData");

	size_t numBytes=0;
	int idx=0;
	for(auto& e:m_send) {
		int64_t size=e.data.end()-e.data.begin();

		Dict camInfo;
		camInfo.Set("type",DataSourceTypeToName(e.type));
		camInfo.Set("id",e.streamId);
		camInfo.Set("captureTime",(int64_t)e.captureTime);
		camInfo.Set("frameIndex",e.frameIndex);
		Dict& dataRef=camInfo.Add("$data");
		dataRef.Set("name", "image"); // <- TODO: 'name' should come from source
		dataRef.Set("idx", idx);
		sync->PushBack(camInfo);

		Dict binaryDataInfo;
		binaryDataInfo.Set("id",e.streamId);
		binaryDataInfo.Set("encoding","h264"); // <- TODO: 'encoding' should come from source
		binaryDataInfo.Set("offset", (int)numBytes);
		binaryDataInfo.Set("size", (int)size);
		bdat->PushBack(binaryDataInfo);

		numBytes+=size;
		idx++;
	}
	numBytes+=info.BinarySize();
	//info.Dump();

	data->reserve(numBytes);
	info.WriteToBinary(data);
	for(auto& e:m_send) {
		data->insert(data->end(),e.data.begin(),e.data.end());
	}
	m_send.clear();
	m_hasAllFrames=false;
	*dataIndex=m_frameIndex;
}

void DataSourceSyncCameraMaster::RunCaptureLoop(TCaptureFunc cb,void* arg) {
	using namespace std::chrono_literals;
	m_callbackFrameReady=cb;
	m_callbackFrameReadyArg=arg;
	m_master=std::thread([this]{
		while(!m_close) {
			std::unique_lock<std::mutex> lock(m_send_mtx);
			m_send_cond.wait_for(lock,200ms);
			if(m_hasAllFrames) {
				if(m_send.empty()) FATAL("Unexpected empty set of frames");
				auto now=GetTimeEpochMicroseconds();
				m_callbackFrameReady(this,now,m_frameIndex,m_callbackFrameReadyArg);
			}
		}
	});
}

void DataSourceSyncCameraMaster::End() {
	m_close=true;
	m_master.join();
	for(SensorClient* vc:m_clients) {
		DestroySensorClient(vc);
	}
}

int datasource_synccameramaster_static_link_use=0;
