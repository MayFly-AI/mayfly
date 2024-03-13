#include "core/datasource.h"
#include "shared/queue.h"
#include "imu/imu.h"
#include <thread>

class DataSourceIMU : public DataSource {
	public:
		DataSourceIMU();
		virtual ~DataSourceIMU();
		virtual uint8_t Type()const{return DS_TYPE_IMU;}
		virtual uint8_t Id()const{return m_id;}
		virtual bool HasClock()const{return false;}
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);

	protected:
		void* m_callbackFrameReadyArg;
		TCaptureFunc m_callbackFrameReady;
		int m_dataIndex = 0;
		uint8_t m_id;
		int m_clockDivider=1;
		int m_freq;
		std::thread m_thread;
		std::thread m_captureLoopThread;

		std::vector<float> acc;
		std::vector<float> gyr;
		std::vector<float> temp;
		std::vector<int> dt;
		std::vector<int64_t> time;
		std::vector<int> index;
		struct IMUDataPoint {
			V3 m_acc;
			V3 m_gyr;
			float m_temp;
			uint32_t m_dt;
			uint64_t m_time;
			int m_index;
		};
		static const int N = 10;
		FixedQueueMT<IMUDataPoint,N> m_queue;
		int m_index=0;
		static DataSource* Create() {return new DataSourceIMU();}
		static const bool s_registered;
};

const bool DataSourceIMU::s_registered=RegisterDataSource("imu",DataSourceIMU::Create);

DataSourceIMU::DataSourceIMU() : DataSource() {
}
DataSourceIMU::~DataSourceIMU() {
}
bool DataSourceIMU::Begin(const Dict& dict) {
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceIMU no id");
	if(!dict.Get("clockDivider",&m_clockDivider))
		FATAL("DataSourceIMU no clockDivider");
	if(dict.Get("frequency",&m_freq)) {
		if(!(m_freq==100 || m_freq==50 || m_freq==25 || m_freq==12)) {
			uprintf("IMU only supports frequency values: 100, 50, 25, 12\n");
		}
	} else {
		FATAL("DataSourceIMU no frequency");
	}
	acc.reserve(N*3);
	gyr.reserve(N*3);
	temp.reserve(N);
	dt.reserve(N);
	time.reserve(N);
	index.reserve(N);
	m_thread=std::thread([&]{
		auto imu_cb=[&](const V3& acc, const V3& gyr, const float& temp, const uint32_t& dt, const uint64_t& time) {
			if(!m_queue.Push([&](IMUDataPoint* element)->void {
				element->m_acc = acc;
				element->m_gyr = gyr;
				element->m_temp = temp;
				element->m_dt = dt;
				element->m_time = time;
				element->m_index = m_index;
			})) {
				uprintf("DataSourceIMU :: queue full\n");
			}
			m_index++;
		};
		ReadIMU(imu_cb, m_freq);
	});
	return true;
}

void DataSourceIMU::End() {
}

bool DataSourceIMU::HasFrameData(int clockIndex)const{
	if(m_queue.Empty()) {
		//printf("im here DataSourceIMU HasFrameData queue empty\n");
		return false;
	}
	return true;
}

void DataSourceIMU::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	data->clear();
	acc.clear();
	gyr.clear();
	temp.clear();
	dt.clear();
	time.clear();
	index.clear();
	int cnt=0;
	while(!m_queue.Empty()) {
		m_queue.Pop([&](IMUDataPoint* element)->void {
			if(!element) {
				FATAL("no element\n");
			}
			acc.push_back(element->m_acc.x);
			acc.push_back(element->m_acc.y);
			acc.push_back(element->m_acc.z);
			gyr.push_back(element->m_gyr.x);
			gyr.push_back(element->m_gyr.y);
			gyr.push_back(element->m_gyr.z);
			temp.push_back(element->m_temp);
			dt.push_back((int)element->m_dt);
			time.push_back((int64_t)element->m_time);
			index.push_back(element->m_index);
		});
		cnt+=1;
	}


	Dict info;
	Dict& frame=info.Add("source");
	auto now=GetTimeEpochMicroseconds();
	frame.Set("type",DataSourceTypeToName(Type()));
	frame.Set("id",m_id);
	frame.Set("time",(int64_t)now);
	frame.Set("index",m_dataIndex);

	Dict* imu=frame.AddObjectNode("values");
	imu->SetTypedArray("acc", acc);
	imu->SetTypedArray("gyr", gyr);
	imu->SetTypedArray("temp", temp);
	imu->SetTypedArray("dt", dt.data(), (int)dt.size());
	imu->SetTypedArray("time", time.data(), (int)time.size());
	imu->SetTypedArray("index", index.data(), (int)index.size());

	//info.Dump();
	info.WriteToBinary(data);

	*dataIndex=m_dataIndex++;
}

int datasource_imu_static_link_use=0;
