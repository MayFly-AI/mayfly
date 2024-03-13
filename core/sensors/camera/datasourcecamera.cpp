#include "core/datasource.h"
#include "core/app.h"
#include "camera/synccameraencoder.h"
#include "camera/options.h"
#include "shared/thread.h"
#include <atomic>
#include <thread>

class DataSourceCamera : public DataSource {
	public:
		DataSourceCamera();
		virtual ~DataSourceCamera();
		virtual uint8_t Type()const{return DS_TYPE_CAMERA;}
		virtual uint8_t Id()const{return m_id;}
		virtual bool HasClock()const{return m_clock?true:false;}
		virtual void RunCaptureLoop(TCaptureFunc cb,void* arg);
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);
		virtual void GetProperties(Dict* properties);
		virtual void SetProperties(const Dict& properties);
		virtual void SetTimeOffset(int64_t offset){m_timeOffset=offset;}

	protected:
		void PrintStatistics();
		void* m_callbackFrameReadyArg;
		TCaptureFunc m_callbackFrameReady;
		int64_t m_timeOffset{0};
		uint64_t m_captureTimeCorrected;
		uint8_t m_id;
		int m_clock=0;					//ticks per second(fps)
		int m_clockDivider=1;
		std::atomic<bool> m_close;
		std::thread m_thread;
		std::string m_mode;
		int m_frameIndex=0;

		struct FrameInfo {
			uint64_t m_time;
			int m_size;
			int m_timeErrorcorrectionAndSend;
		};
		uint64_t m_lastPrintTime=0;
		FrameInfo m_frameSizesRing[128]={{}};

		const uint8_t* m_data=0;
		size_t m_dataBytesize=0;
		static DataSource* Create() {return new DataSourceCamera();}
		static const bool s_registered;
};

const bool DataSourceCamera::s_registered=RegisterDataSource("camera",DataSourceCamera::Create);

DataSourceCamera::DataSourceCamera() : DataSource() {
	m_close=false;
}
DataSourceCamera::~DataSourceCamera() {
}
bool DataSourceCamera::Begin(const Dict& dict) {
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceCamera no id");
	if(m_id >= STREAM_ID_MAX)
		FATAL("stream id must fit in %d bits", STREAM_ID_BITS);
	if(!dict.Get("clock",&m_clock) && !dict.Get("clockDivider",&m_clockDivider))
		FATAL("DataSourceCamera no clock or clockDivider");
	dict.Get("mode",&m_mode);
	return true;
}
void DataSourceCamera::End() {
	m_close=true;
	if(HasClock()) {
		m_thread.join();
	}
}

void DataSourceCamera::PrintStatistics() {
	uint64_t t=GetTimeEpochMicroseconds();
	if(m_frameIndex>countof(m_frameSizesRing)) {
		float fps=0;
		double ringTime=0;
		if(m_frameIndex>countof(m_frameSizesRing)) {
			uint64_t oldestTime=m_frameSizesRing[(m_frameIndex)%countof(m_frameSizesRing)].m_time;
			uint64_t newestTime=m_frameSizesRing[(m_frameIndex-1)%countof(m_frameSizesRing)].m_time;
			ringTime=(double)(newestTime-oldestTime);
			fps=(float)(1000000.0/((double)(t-oldestTime)/(double)countof(m_frameSizesRing)));
		}
		int maxSize=0;
		int sumSize=0;
		int cnt=MIN(countof(m_frameSizesRing),m_frameIndex);
		int maxTime=0;
		int sumTime=0;
		for(int i=0;i!=cnt;i++) {
			maxTime=MAX(m_frameSizesRing[i].m_timeErrorcorrectionAndSend,maxTime);
			sumTime+=m_frameSizesRing[i].m_timeErrorcorrectionAndSend;
			maxSize=MAX(m_frameSizesRing[i].m_size,maxSize);
			sumSize+=m_frameSizesRing[i].m_size;
		}
		double ringSeconds=ringTime/1000000.0;
		double bitsPerSecond=((double)sumSize/ringSeconds)*8;
		int aveTime=sumTime/countof(m_frameSizesRing);
		float aveSize=((float)sumSize/(float)countof(m_frameSizesRing))/1000.0f;
		uprintf("index %d %.1ffps time ave %dus max %dus size ave %.2fkB max %.2fkB %.1f Mbit/s\n",m_frameIndex,fps,aveTime,maxTime,aveSize,(float)maxSize/1000.0f,bitsPerSecond/1000000.0);
	}
	m_lastPrintTime=t;
}

static const char* ExtractNextInteger(uint32_t* res, const char* str) {
    if(!str) return 0;
    while(*str && (*str<'0' || *str>'9')) str++;
    if(!*str) return 0;
    char* end;
    *res=(uint32_t)strtol(str, &end, 10);
    return end;
}

static bool ParseResolution(uint32_t* w, uint32_t* h, const std::string& mode){
	std::string str=mode;
	size_t colonPos=mode.find_first_of(':');
	if(colonPos!=std::string::npos) {
		str=mode.substr(colonPos+1);
	}
  return ExtractNextInteger(h, ExtractNextInteger(w, str.c_str()));
}

void DataSourceCamera::RunCaptureLoop(TCaptureFunc cb,void* arg) {
	m_callbackFrameReadyArg=arg;
	m_callbackFrameReady=cb;
	std::atomic<bool> ready=false;
	m_thread=std::thread([&]{
#ifdef USE_THREAD_AFFINITY
		SetSelfAffinityMask(THREAD_AFFINITY_SERVER_MAIN);
#endif
		SyncCameraEncoder* app=CreateSyncCameraEncoder(0);
		if(!app){
			FATAL("could not create SyncCameraEncoder");
		}
		auto opt=app->GetOptions();

		if(ParseResolution(&opt->width, &opt->height, m_mode)) {
			uprintf("Requesting resolution: width=%d, height=%d\n", opt->width, opt->height);
		} else {
			FATAL("Could not parse camera mode %s",m_mode.c_str());
		}
		ready=true;
		auto enc_cb=[&](void* data,size_t size,uint64_t timeCapture) {

			uint64_t t=GetTimeEpochMicroseconds();
			//if(t-m_lastPrintTime>5000000) {
			//	PrintStatistics();
			//	t=GetTimeEpochMicroseconds();		//Don't include statistics time in next measurement
			//}
			m_data=(const uint8_t*)data;
			m_dataBytesize=size;
			m_callbackFrameReady(this,timeCapture+m_timeOffset,m_frameIndex,m_callbackFrameReadyArg);
			m_captureTimeCorrected=timeCapture+m_timeOffset;
			m_data=0;
			m_dataBytesize=0;

			m_frameSizesRing[m_frameIndex%countof(m_frameSizesRing)]={t,(int)size,(int)(GetTimeEpochMicroseconds()-t)};

			m_frameIndex++;
			opt->synchronize_offset=m_timeOffset;
			Options* opt=app->GetOptions();
			if(m_clock!=opt->framerate) {
				opt->framerate=(float)m_clock;
			}
			if(m_close) {
				opt->timeout=1;
			}
		};
		run(app,nullptr,enc_cb,nullptr);
		DestroySyncCameraEncoder(app);
	});
	while(!ready);

}
bool DataSourceCamera::HasFrameData(int clockIndex)const{
	if(!m_dataBytesize)
		return false;
	//if(clockIndex%m_clockDivider)
	//	return false;
	return true;
}
void DataSourceCamera::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	*dataIndex=m_frameIndex;
	if(!m_dataBytesize)
		FATAL("DataSourceCamera::GetFrameData no data!");

	Dict info;
	Dict& frame=info.Add("source");
	auto now=GetTimeEpochMicroseconds();
	frame.Set("type",DataSourceTypeToName(Type()));
	frame.Set("id", m_id);
	frame.Set("time", (int64_t)now);
	frame.Set("index", m_frameIndex);

	Dict* image=frame.AddArrayNode("frames");
	Dict* bdat=info.AddArrayNode("binaryData");

	size_t numBytes=m_dataBytesize;
	Dict camInfo;
	camInfo.Set("type",DataSourceTypeToName(Type()));
	camInfo.Set("id",m_id);
	camInfo.Set("captureTime",(int64_t)m_captureTimeCorrected);
	camInfo.Set("frameIndex",m_frameIndex);
	Dict& dataRef=camInfo.Add("$data");
	dataRef.Set("name", "image");
	dataRef.Set("idx", 0);
	image->PushBack(camInfo);

	Dict binaryDataInfo;
	binaryDataInfo.Set("id",m_id);
	binaryDataInfo.Set("encoding","h264");
	binaryDataInfo.Set("offset", 0);
	binaryDataInfo.Set("size", (int)numBytes);
	bdat->PushBack(binaryDataInfo);

	numBytes+=info.BinarySize();
	//info.Dump();

	data->reserve(numBytes);
	info.WriteToBinary(data);

	data->insert(data->end(),m_data,m_data+m_dataBytesize);
	ASSERT(data->size()==numBytes, "data size expected: %d, actual: %d\n", data->size(), numBytes);
}

void DataSourceCamera::GetProperties(Dict* properties) {
	std::string test=R"({
		"schema":{
			"properties":{
				"display":{
					"type":"boolean"
				},
				"clock":{
					"type":"integer",
					"minimum":10,
					"maximum":120,
					"default":30,
					"step":1
				},
				"mode": {
					"type":"string",
					"items":{
						"type":"string",
						"enum": ["vga: 640x480","720p: 1280x720","1080p: 1920x1080"]
					},
					"default": "720p: 1280x720"
				}
			}
		}
	})";
	Dict* clockDict=properties->AddObjectNode(PropertiesName());
	Dict dict;
	dict.ReadFromJson(test);
	clockDict->Set(dict);
	Dict* values=clockDict->AddObjectNode("values");
	values->Set("clock",m_clock);
	values->Set("mode",m_mode);
}

void DataSourceCamera::SetProperties(const Dict& properties) {
	const Dict* clockDict=properties.Find(PropertiesName());
	bool modified=false;
	if(clockDict && clockDict->Get("modified",&modified) && modified) {
		const Dict* values=clockDict->Find("values");
		std::string mode;
		int clock;
		values->Get("clock",&clock);
		values->Get("mode",&mode);
		if(mode!=m_mode) {
			m_mode=mode;
			m_clock=clock;
			App::PostEvent(App::EV_SAVE_RESTART);
		}else
		if(clock!=m_clock) {
			m_clock=clock;
			App::PostEvent(App::EV_SAVE);
		}
	}
}

int datasource_camera_static_link_use=0;
