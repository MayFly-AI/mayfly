#include "core/datasource.h"

class DataSourceClock : public DataSource {
	public:
		DataSourceClock();
		virtual ~DataSourceClock();
		virtual uint8_t Type()const{return DS_TYPE_CLOCK;}
		virtual uint8_t Id()const{return m_id;}
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);
		virtual void GetProperties(Dict* properties);
		virtual void SetProperties(const Dict& properties);
	protected:
		uint8_t m_id=0;
		int m_dataIndex=0;
		int m_clockDivider=1;
		static DataSource* Create() {return new DataSourceClock();}
		static const bool s_registered;
};

const bool DataSourceClock::s_registered=RegisterDataSource("clock",DataSourceClock::Create);

DataSourceClock::DataSourceClock() : DataSource() {
}
DataSourceClock::~DataSourceClock() {
}
bool DataSourceClock::Begin(const Dict& dict) {
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceClock no id");
	if(m_id>=STREAM_ID_MAX)
		FATAL("stream id must fit in %d bits", STREAM_ID_BITS);
	dict.Get("clockDivider",&m_clockDivider,m_clockDivider);
	return true;
}
void DataSourceClock::End() {
}
bool DataSourceClock::HasFrameData(int clockIndex)const {
	if(clockIndex%m_clockDivider)
		return false;
	return true;
}

void DataSourceClock::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	*dataIndex=m_dataIndex++;
	Dict info;
	uint64_t microsecondsUTC=GetTimeEpochMicroseconds();
	std::string clock=TimeEpochMicrosecondsToString(microsecondsUTC,true);
	info.Set("type",DataSourceTypeToName(Type()));
	info.Set("clock",clock);
	info.WriteToBinary(data);
}
void DataSourceClock::SetProperties(const Dict& properties) {
	const Dict* clockDict=properties.Find(PropertiesName());
	bool modified=false;
	if(clockDict && clockDict->Get("modified",&modified) && modified) {
		const Dict* values=clockDict->Find("values");
		values->Get("clockDivider",&m_clockDivider);
		//uprintf("m_clockDivider %d\n",m_clockDivider);
	}
}

void DataSourceClock::GetProperties(Dict* properties) {
	std::string test=R"({
		"schema":{
			"properties":{
				"clockDivider":{
					"type":"integer",
					"minimum":1,
					"maximum":20,
					"default":1,
					"step":1
				},
			}
		}
	})";
	Dict* clockDict=properties->AddObjectNode(PropertiesName());
	Dict dict;
	dict.ReadFromJson(test);
	clockDict->Set(dict);
	Dict* values=clockDict->AddObjectNode("values");
	values->Set("clockDivider",m_clockDivider);
}

int datasource_clock_static_link_use=0;
