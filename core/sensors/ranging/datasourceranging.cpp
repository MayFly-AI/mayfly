#include "core/datasource.h"
#include "ranging.h"
#include <math.h>

class DataSourceRanging : public DataSource {
	public:
		DataSourceRanging();
		virtual ~DataSourceRanging();
		virtual uint8_t Type()const{return DS_TYPE_RANGING;}
		virtual uint8_t Id()const{return m_id;}
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);
	protected:
		uint8_t m_id=0;
		int m_dataIndex=0;
		int m_clockDivider=1;
		std::string m_rangingClass;
		Ranging* m_ranging;
		static DataSource* Create() {return new DataSourceRanging();}
		static const bool s_registered;
};

const bool DataSourceRanging::s_registered=RegisterDataSource("ranging",DataSourceRanging::Create);

DataSourceRanging::DataSourceRanging() : DataSource() {
}
DataSourceRanging::~DataSourceRanging() {
}
bool DataSourceRanging::Begin(const Dict& dict) {
	//uprintf("DataSourceRanging::Begin\n");
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceRanging no id");
	if(m_id >= STREAM_ID_MAX)
		FATAL("stream id must fit in %d bits", STREAM_ID_BITS);
	if(!dict.Get("clockDivider",&m_clockDivider)) {
		FATAL("DataSourceRanging no clockDivider defined");
	}

	if(!dict.Get("class",&m_rangingClass)) {
		FATAL("DataSourceRanging no class defined. Candidates are \"initiator\" or \"responder\"");
	}

	Dict rangingDict;
	if(m_rangingClass=="initiator") {
		rangingDict.ReadFromJson(R"({"type":"ping"})");
		m_ranging=CreateRanging(rangingDict);
	}else
	if(m_rangingClass=="responder") {
		rangingDict.ReadFromJson(R"({"type":"pong"})");
		m_ranging=CreateRanging(rangingDict);
	}else{
		FATAL("DataSourceRanging class %s not valid. Candidates are \"initiator\" or \"responder\"",m_rangingClass.c_str());
	}
	return true;
}
void DataSourceRanging::End() {
	if(m_ranging)
		DestroyRanging(m_ranging);
}
bool DataSourceRanging::HasFrameData(int clockIndex)const {
	if(clockIndex%m_clockDivider)
		return false;
	return true;
}

void DataSourceRanging::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	Dict info;
	Dict& frame=info.Add("source");
	auto now=GetTimeEpochMicroseconds();
	frame.Set("type",DataSourceTypeToName(Type()));
	frame.Set("id",m_id);
	frame.Set("time",(int64_t)now);
	frame.Set("index",m_dataIndex);

	Dict* ranging=frame.AddObjectNode("values");
	ranging->Set("class",m_rangingClass);
	ranging->Set("tx",m_ranging->m_packetsSend);
	ranging->Set("rx",m_ranging->m_packetsReceived);
	if(m_rangingClass=="initiator") {
		float distance=-1;
		uint16_t time;
		if(m_ranging->SendPing(&time,&distance)) {
			distance=roundf(distance*100.0f)/100.0f;
			ranging->Set("distance",distance);
			ranging->Set("time",time);
			//uprintf("DataSourceRanging::GetFrameData time %d distance %.2fm\n",time,distance);
		}
	}
	info.WriteToBinary(data);
	//info.Dump();
	*dataIndex=m_dataIndex++;
}

int datasource_ranging_static_link_use=0;
