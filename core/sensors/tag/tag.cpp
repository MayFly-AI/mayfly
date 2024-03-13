
#include <thread>
#include <atomic>
#include "tag.h"
#include <math.h>
#include <iostream>

#include "shared/queue.h"
#include "shared/math.h"

#include "core/datasource.h"

#include "uwb.h"

#if PLATFORM_RPI
#include "platform/gpio.h"
#include "platform/spi.h"

#include "uwb/dw_regs.h"
#include "uwb/platform_io.h"
#include "uwb/deca_spi.h"
#include "uwb/deca_dw_device_api.h"
#include "uwb/dw_device.h"

// connection pins (BROADCOM PIN NUMBERING)
const uint8_t PIN_RST = 23; // reset pin
const uint8_t PIN_IRQ = 24; // irq pin
const uint8_t PIN_CE0 = 8; // spi select pin

#endif

class DataSourceTag : public DataSource {
	public:
		DataSourceTag();
		virtual ~DataSourceTag();
		virtual uint8_t Type()const{return DS_TYPE_TAG;}
		virtual uint8_t Id()const{return m_id;}
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);
	protected:

		uint8_t m_id=0;
		int m_dataIndex=0;
		int m_clockDivider=1;
		std::string m_tagClass;
		std::atomic<bool> m_close;
		std::thread m_thread;
		struct Measurement {
			PingManager::Measurements m_data;
		};
		volatile int m_writeIndex;
		volatile int m_readIndex;
		Measurement m_measurements[10];

		//FixedQueueMT<int,128> m_queue;
		static DataSource* Create() {return new DataSourceTag();}
		static const bool s_registered;
};

const bool DataSourceTag::s_registered=RegisterDataSource("tag",DataSourceTag::Create);

DataSourceTag::DataSourceTag() : DataSource() {
}
DataSourceTag::~DataSourceTag() {
}
bool DataSourceTag::Begin(const Dict& dict) {
	//uprintf("DataSourceTag::Begin\n");
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceTag no id");
	if(m_id>=STREAM_ID_MAX)
		FATAL("stream id must fit in %d bits",STREAM_ID_BITS);
	if(!dict.Get("clockDivider",&m_clockDivider)) {
		FATAL("DataSourceTag no clockDivider defined");
	}
	std::atomic<bool> ready=false;
	m_writeIndex=0;
	m_readIndex=0;
	m_close=false;
#if PLATFORM_RPI
	RPI_GPIO::setup();
	//gpio_init();
	rpi_spi_init();
	RPI_GPIO::setup_gpio(PIN_CE0,RPI_GPIO::PIN_MODE::OUTPUT,0);
	RPI_GPIO::output_gpio(PIN_CE0,1);

	dw_irq_init(PIN_IRQ);
	deca_usleep(2000); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
	reset_DWIC(PIN_RST);
	m_thread=std::thread([&]{
		ready=true;
		PingManager pm;
		pm.Begin();
		while(!m_close) {
			uint64_t t64s0=GetTimeMicroseconds();
			uint32_t count=pm.BeginPing();
			for(uint32_t i=0;i!=count;i++) {
				pm.SendPing(i);
			}
			pm.EndPing();
			//if(pm.HasWarning()) {
			//	uprintf("PingManager status flags $%08x\n",pm.GetStatusMask());
			//}
			PingManager::Measurements data;
			pm.GetMeasurements(&data);
			if(m_writeIndex-m_readIndex>=countof(m_measurements)) {
				uprintf("DataSourceTag ring que full %d,%d\n",m_writeIndex,m_readIndex);
			}else{
				Measurement* m=&m_measurements[m_writeIndex%countof(m_measurements)];
				m->m_data=data;
				m_writeIndex++;
			}
			//pm.GetMeasurements(&stream);
			while(true){
				uint64_t t64=GetTimeMicroseconds();
				if(t64-t64s0>500000L)						//Ping frequency
					break;
				std::this_thread::sleep_for(std::chrono::microseconds(1000));
			}
		}
		//pm.End();
	});
#else
	m_thread=std::thread([&]{
		ready=true;
		while(!m_close) {
			std::this_thread::sleep_for(std::chrono::microseconds(100000));
		}
	});
#endif
	while(!ready);
	return true;
}
void DataSourceTag::End() {
	m_close=true;
}
bool DataSourceTag::HasFrameData(int clockIndex)const {
	if(clockIndex%m_clockDivider)
		return false;
	return true;
}

void DataSourceTag::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	Dict info;
	auto now=GetTimeEpochMicroseconds();
	Dict& frame=info.Add("source");
	frame.Set("type",DataSourceTypeToName(Type()));
	frame.Set("id",m_id);
	frame.Set("time",(int64_t)now);
	frame.Set("index",m_dataIndex);
	Dict* frames=frame.AddArrayNode("values");
	while(m_readIndex<m_writeIndex) {
		const Measurement& measurements=m_measurements[m_readIndex%countof(m_measurements)];
		PingManager::Measurements data=measurements.m_data;
		m_readIndex++;
		Dict dict;
		Dict* bases=dict.AddArrayNode("bases");
		dict.Set("status",data.m_status);
		dict.Set("frameCount",data.m_frameCount);
		for(int i=0;i!=(int)data.m_bases.size();i++) {
			Dict base;
			base.Set("seq",(int)data.m_bases[i].m_sequenceNr);
			base.Set("time",(int64_t)data.m_bases[i].m_time);
			base.Set("cnt",(int)data.m_bases[i].m_frameCount);
			base.Set("id",(int)data.m_bases[i].m_id);
			base.Set("dist",data.m_bases[i].m_distance);
			base.Set("nlos",data.m_bases[i].m_nlos);
			if(data.m_bases[i].m_neighbors.size()) {
				Dict* neighbors=base.AddArrayNode("nei");
				for(int j=0;j!=(int)data.m_bases[i].m_neighbors.size();j++) {
					Dict neighbor;
					neighbor.Set("id",(int)data.m_bases[i].m_neighbors[j].m_id);
					neighbor.Set("dist",data.m_bases[i].m_neighbors[j].m_dist/100.0f);
					neighbors->PushBack(neighbor);
				}
			}
			bases->PushBack(base);
		}
		frames->PushBack(dict);
	}
	//clientDict.Dump();
	info.WriteToBinary(data);
	//uprintf("DataSourceTag::GetFrameData index %d bytesize %d\n",m_dataIndex,(int)data->size());
	//std::string str=info.WriteToJson();
	//data->assign(str.begin(),str.end());
	*dataIndex=m_dataIndex++;
}

int datasource_tag_static_link_use=0;
