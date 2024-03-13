#include "core/datasource.h"
#include "shared/queue.h"
#include "tof/arducam_tof.h"
#include <thread>

#ifdef PLATFORM_RPI
class DataSourceToF : public DataSource {
	public:
		DataSourceToF();
		virtual ~DataSourceToF();
		virtual uint8_t Type()const{return DS_TYPE_TOF;}
		virtual uint8_t Id()const{return m_id;}
		virtual bool HasClock()const{return m_clock?true:false;}
		virtual void RunCaptureLoop(TCaptureFunc cb,void* arg);
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);

		struct Frame {
			std::vector<uint8_t> m_data;
			uint64_t m_timeCapture;
			int m_frameIndex;
			uint8_t m_streamId;
		};

	protected:
		void* m_callbackFrameReadyArg;
		TCaptureFunc m_callbackFrameReady;
		int m_dataIndex = 0;
		uint8_t m_id;
		int m_clock;
		int m_clockDivider;
		std::thread m_thread;
		static const int N=10;
		FixedQueueMT<Frame,N> m_queue;
		int m_frameIndex=0;
		std::vector<uint8_t> m_data;
		float m_threshold;
		static DataSource* Create() {return new DataSourceToF();}
		static const bool s_registered;
};

const bool DataSourceToF::s_registered=RegisterDataSource("tof",DataSourceToF::Create);

DataSourceToF::DataSourceToF() : DataSource() {
}
DataSourceToF::~DataSourceToF() {
}
bool DataSourceToF::Begin(const Dict& dict) {
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceToF no id");
	dict.Get("clock",&m_clock,30);
	dict.Get("clockDivider",&m_clockDivider,1);
	dict.Get("threshold",&m_threshold,30.);

 	// start capture thread if not clock here, otherwise start capture thread in RunCaptureLoop()
	if(!HasClock()) {
		m_thread=std::thread([&]{
#ifdef USE_THREAD_AFFINITY
			SetSelfAffinityMask(THREAD_AFFINITY_SERVER_MAIN);
#endif
			auto tof_cb=[&](const uint8_t* data, size_t size, uint64_t timeCapture) {
				if(!m_queue.Push([&](Frame* element)->void {
					element->m_data.assign(data,data+size);
					element->m_timeCapture=timeCapture;
					element->m_frameIndex=m_frameIndex;
					element->m_streamId=m_id;
				})) {
					uprintf("ArducamToF queue full\n");
				}
				m_frameIndex++;
			};
			ArducamToF tof(tof_cb, true, m_threshold);
			tof.MainLoop();
		});
	}

	return true;
}

void DataSourceToF::End() {
	if(HasClock()) {
		m_thread.join();
	}
}

void DataSourceToF::RunCaptureLoop(TCaptureFunc cb,void* arg) {
	m_callbackFrameReady=cb;
	m_callbackFrameReadyArg=arg;
	m_thread=std::thread([&]{
#ifdef USE_THREAD_AFFINITY
		SetSelfAffinityMask(THREAD_AFFINITY_SERVER_MAIN);
#endif
		auto tof_cb=[&](const uint8_t* data, size_t size, uint64_t timeCapture) {
			if(!m_queue.Push([&](Frame* element)->void {
				element->m_data.assign(data,data+size);
				element->m_timeCapture=timeCapture;
				element->m_frameIndex=m_frameIndex;
				element->m_streamId=m_id;
			})) {
				uprintf("ArducamToF queue full\n");
			}
			m_callbackFrameReady(this,timeCapture,m_frameIndex,m_callbackFrameReadyArg);
			m_frameIndex++;
		};
		ArducamToF tof(tof_cb, true, m_threshold);
		tof.MainLoop();
	});
}

bool DataSourceToF::HasFrameData(int clockIndex)const{
	if(m_queue.Empty()) {
		return false;
	}
	return true;
}

void DataSourceToF::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	*dataIndex=m_dataIndex;
	m_dataIndex+=1;
	m_data.clear();
	m_data.reserve(10*(240*180*sizeof(uint16_t))); // prob way to big

	Dict info;
	Dict& frame=info.Add("source");
	auto now=GetTimeEpochMicroseconds();
	frame.Set("type",DataSourceTypeToName(Type()));
	frame.Set("id",m_id);
	frame.Set("time",(int64_t)now);
	frame.Set("index",m_frameIndex);

	Dict* depth=frame.AddArrayNode("frames");
	Dict* bdat=info.AddArrayNode("binaryData");

	int cnt=0;
	int bytes=0;
	while(!m_queue.Empty()) {
		m_queue.Pop([&](Frame* element)->void {
			Dict frameInfo;
			frameInfo.Set("type",DataSourceTypeToName(Type()));
			frameInfo.Set("id",element->m_streamId);
			frameInfo.Set("captureTime",(int64_t)element->m_timeCapture);
			frameInfo.Set("frameIndex",element->m_frameIndex);
			Dict& dataRef=frameInfo.Add("$data");
			dataRef.Set("name", "depth");
			dataRef.Set("idx", cnt);
			depth->PushBack(frameInfo);

			Dict binaryDataInfo;
			binaryDataInfo.Set("id",element->m_streamId);
			binaryDataInfo.Set("encoding","huff");
			binaryDataInfo.Set("offset", (int)bytes);
			binaryDataInfo.Set("size", (int)element->m_data.size());
			bdat->PushBack(binaryDataInfo);

			m_data.resize(data->size()+element->m_data.size());
			memcpy(m_data.data()+bytes, element->m_data.data(), sizeof(uint8_t)*element->m_data.size());
			bytes+=(int)element->m_data.size();
		});
		cnt+=1;
	}
	if(cnt>1) {
		uprintf("DataSouceToF::GetFrameData sending more than 1 frame\n");
	}
	bytes+=info.BinarySize();

	data->reserve(bytes);
	info.WriteToBinary(data);
	data->insert(data->end(),m_data.begin(),m_data.end());
}
#endif

int datasource_tof_static_link_use=0;
