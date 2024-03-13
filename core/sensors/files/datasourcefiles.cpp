#include "core/datasource.h"
#include "shared/file.h"
#include <atomic>
#include <thread>
#include <algorithm>

class DataSourceFiles : public DataSource {
	public:
		DataSourceFiles();
		virtual ~DataSourceFiles();
		virtual uint8_t Type()const{return DS_TYPE_VIDEO;}
		virtual uint8_t Id()const{return m_id;}
		virtual bool HasClock()const{return m_clock?true:false;}
		virtual void RunCaptureLoop(TCaptureFunc cb,void* arg);
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);

		virtual void GetProperties(Dict* properties);
		virtual void SetProperties(const Dict& properties);
	protected:
		void* m_callbackFrameReadyArg;
		TCaptureFunc m_callbackFrameReady;
		void InitLoadH264Packets(const char* path);
		uint8_t m_id;
		std::vector<std::vector<char>> m_loadedFramesH264;
		int m_clock=0;					//ticks per second(fps)
		int m_clockDivider=1;
		std::atomic<bool> m_close;
		std::thread m_thread;
		static DataSource* Create() {return new DataSourceFiles();}
		static const bool s_registered;
};

const bool DataSourceFiles::s_registered=RegisterDataSource("files",DataSourceFiles::Create);

DataSourceFiles::DataSourceFiles() : DataSource() {
	m_close=false;
}
DataSourceFiles::~DataSourceFiles() {
}
bool DataSourceFiles::Begin(const Dict& dict) {
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceFiles no id");
	if(m_id >= STREAM_ID_MAX)
		FATAL("stream id must fit in %d bits", STREAM_ID_BITS);
	if(!dict.Get("clock",&m_clock) && !dict.Get("clockDivider",&m_clockDivider))
		FATAL("DataSourceFiles no clock or clockDivider");
	std::string path;
	dict.Get("path",&path,"$(DATA)/video/exploded/office-17-10-22/");
	InitLoadH264Packets(path.c_str());
	return true;
}
void DataSourceFiles::End() {
	m_close=true;
	if(HasClock()) {
		m_thread.join();
	}
}
void DataSourceFiles::RunCaptureLoop(TCaptureFunc cb,void* arg) {
	m_callbackFrameReadyArg=arg;
	m_callbackFrameReady=cb;
	m_thread=std::thread([&]{
#ifdef USE_THREAD_AFFINITY
		SetSelfAffinityMask(THREAD_AFFINITY_SERVER_MAIN);
#endif
		int frameIndex=0;
		while(!m_close) {
			const uint64_t frameWaitTime=(1000000L/(int64_t)m_clock);
			uint64_t timeCapture=GetTimeEpochMicroseconds();
			m_callbackFrameReady(this,timeCapture,frameIndex,m_callbackFrameReadyArg);
			frameIndex++;
			uint64_t endTime=timeCapture+frameWaitTime;
			int sleepTime=(int)(endTime-GetTimeEpochMicroseconds());
			preciseSleep(sleepTime);
		}
	});
}
bool DataSourceFiles::HasFrameData(int clockIndex)const{
	if(!m_loadedFramesH264.size())
		return false;
	if(clockIndex%m_clockDivider)
		return false;
	return true;
}
void DataSourceFiles::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	*dataIndex=clockIndex/m_clockDivider;
	int fileIndex=*dataIndex%(int)m_loadedFramesH264.size();
	const std::vector<char>* frameH264=&m_loadedFramesH264[fileIndex];

	Dict info;
	Dict& frame=info.Add("source");
	auto now=GetTimeEpochMicroseconds();
	frame.Set("type",DataSourceTypeToName(Type()));
	frame.Set("id", m_id);
	frame.Set("time", (int64_t)now);
	frame.Set("index", fileIndex);

	Dict* image=frame.AddArrayNode("frames");
	Dict* bdat=info.AddArrayNode("binaryData");

	size_t numBytes=frameH264->size();
	Dict camInfo;
	camInfo.Set("type",DataSourceTypeToName(Type()));
	camInfo.Set("id",m_id);
	camInfo.Set("captureTime",(int64_t)GetTimeEpochMicroseconds());
	camInfo.Set("frameIndex",fileIndex);
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

	data->reserve(numBytes);
	info.WriteToBinary(data);
	//info.Dump();
	data->insert(data->end(),(uint8_t*)frameH264->data(),((uint8_t*)frameH264->data())+frameH264->size());
	ASSERT(data->size() == numBytes, "unexpected data size");
}
void DataSourceFiles::InitLoadH264Packets(const char* path) {
	std::vector<std::string> files;
	GetDirectoryFiles(&files,0,path);
	std::sort(files.begin(),files.end(),[](const std::string& a,const std::string& b)->bool{return a<b;});
	for(std::string& file:files) {
		std::vector<char> data;
		if(!LoadFile(&data,path+file))
			FATAL("Unable to load file %s",file.c_str());
		//uprintf("File %s size %d\n",file.c_str(),data.size());
		m_loadedFramesH264.push_back(data);
	}
	if(!m_loadedFramesH264.size())
		FATAL("Unable to load H264 frames from path %s\n",path);
	uprintf("Loaded %d H264 frames\n",(int)m_loadedFramesH264.size());
}

void DataSourceFiles::GetProperties(Dict* properties) {
	std::string test=R"({
		"schema":{
			"properties":{
				"clock":{
					"type":"integer",
					"minimum":1,
					"maximum":300,
					"default":30,
					"step":1
				}
			}
		}
	})";
	Dict* filesDict=properties->AddObjectNode(PropertiesName());
	Dict dict;
	dict.ReadFromJson(test);
	filesDict->Set(dict);
	Dict* values=filesDict->AddObjectNode("values");
	values->Set("clock",m_clock);
}

void DataSourceFiles::SetProperties(const Dict& properties) {
	const Dict* filesDict=properties.Find(PropertiesName());
	bool modified=false;
	if(filesDict && filesDict->Get("modified",&modified) && modified) {
		const Dict* values=filesDict->Find("values");
		std::string mode;
		int clock;
		values->Get("clock",&clock);
		if(clock!=m_clock) {
			m_clock=clock;
		}
	}
}

int datasource_files_static_link_use=0;
