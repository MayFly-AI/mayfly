#include "core/datasource.h"
#include "shared/queue.h"
#include "magnetometer/magnetometer.h"
#include <thread>

class DataSourceMAG : public DataSource {
	public:
		DataSourceMAG();
		virtual ~DataSourceMAG();
		virtual uint8_t Type()const{return DS_TYPE_MAG;}
		virtual uint8_t Id()const{return m_id;}
		virtual bool HasClock()const{return false;}
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);

	protected:
		void* m_callbackFrameReadyArg;
		TCaptureFunc m_callbackFrameReady;
		int m_dataIndex=0;
		uint8_t m_id;
		int m_clockDivider=1;
		int m_freq;
		std::thread m_thread;
		std::thread m_captureLoopThread;
		struct MAGDataPoint {
			V3 m_mag;
			float m_temp;
			uint64_t m_t;
			int m_index;
		};
		static const int N=10;
		FixedQueueMT<MAGDataPoint,N> m_queue;
		int m_index=0;
		static DataSource* Create() {return new DataSourceMAG();}
		static const bool s_registered;
};

const bool DataSourceMAG::s_registered=RegisterDataSource("mag",DataSourceMAG::Create);

DataSourceMAG::DataSourceMAG() : DataSource() {
}
DataSourceMAG::~DataSourceMAG() {
}
bool DataSourceMAG::Begin(const Dict& dict) {
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceMAG no id");
	if(!dict.Get("clockDivider",&m_clockDivider))
		FATAL("DataSourceMAG no clockDivider");
	if(dict.Get("frequency",&m_freq)) {
		if(!(m_freq==200 || m_freq==100 || m_freq==50 || m_freq==25)) {
			uprintf("MAG Requested frequency %d BMM350 only supports frequency values: 200, 100, 50, 25\n",m_freq);
		}
	} else {
		FATAL("DataSourceMAG no frequency");
	}
	m_thread=std::thread([&]{
		auto mag_cb=[&](const V3& mag,float temp, const uint64_t& t) {
			if(!m_queue.Push([&](MAGDataPoint* element)->void {
				element->m_mag=mag;
				element->m_temp=temp;
				element->m_t=t;
				element->m_index=m_index;
			})) {
				uprintf("DataSourceMAG :: queue full\n");
			}
			m_index++;
		};
		ReadMagnetometer(mag_cb,m_freq);
	});
	return true;
}

void DataSourceMAG::End() {
}

bool DataSourceMAG::HasFrameData(int clockIndex)const{
	if(m_queue.Empty()) {
		//printf("im here DataSourceMAG HasFrameData queue empty\n");
		return false;
	}
	return true;
}

void DataSourceMAG::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	*dataIndex=m_dataIndex++;
	float mag[3*N];
	float temp[N];
	int64_t time[N];
	int index[N];
	int writePos=0;
	while(!m_queue.Empty()) {
		m_queue.Pop([&](MAGDataPoint* element)->void {
			if(!element || writePos>=N) {
				FATAL("DataSourceMAG sanity check failed\n");
			}
			mag[writePos*3+0]=element->m_mag.x;
			mag[writePos*3+1]=element->m_mag.y;
			mag[writePos*3+2]=element->m_mag.z;
			temp[writePos]=element->m_temp;
			time[writePos]=element->m_t;
			index[writePos]=element->m_index;
			writePos++;
		});
	}
	Dict info;
	auto now=GetTimeEpochMicroseconds();
	Dict& frame=info.Add("source");
	frame.Set("type",DataSourceTypeToName(Type()));
	frame.Set("id",m_id);
	frame.Set("time",(int64_t)now);
	frame.Set("index",m_dataIndex);
	Dict* imu=frame.AddObjectNode("values");
	imu->SetTypedArray("mag",mag,writePos*3);
	imu->SetTypedArray("temp",temp,writePos);
	imu->SetTypedArray("time",time,writePos);
	imu->SetTypedArray("index",index,writePos);

	//uprintf("m_dataIndex %d mag %d\n",m_dataIndex,mag);
	//info.Dump();
	info.WriteToBinary(data);
	//std::string str=info.WriteToJson();
	//data->assign(str.begin(),str.end());
}

int datasource_mag_static_link_use=0;
