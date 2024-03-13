#include "core/datasource.h"
#include "core/history.h"
#include "shared/net.h"
#include <atomic>
#include <thread>
#ifdef __linux__
#include <sys/sysinfo.h>
#include <dirent.h>
#endif

class DataSourceStatus : public DataSource {
	public:
		DataSourceStatus();
		virtual ~DataSourceStatus();
		virtual uint8_t Type()const{return DS_TYPE_STATUS;}
		virtual uint8_t Id()const{return m_id;}
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex);
	protected:
		void AddSystemInfo(Dict* root);
		uint8_t m_id=0;
		int m_dataIndex=0;
		int m_clockDivider=1;
		std::vector<AdaptorInfo*> m_adapters;
		static DataSource* Create() {return new DataSourceStatus();}
		static const bool s_registered;
};
const bool DataSourceStatus::s_registered=RegisterDataSource("status",DataSourceStatus::Create);

DataSourceStatus::DataSourceStatus() : DataSource() {
}
DataSourceStatus::~DataSourceStatus() {
}
bool DataSourceStatus::Begin(const Dict& dict) {
	if(!dict.Get("id",(char*)&m_id))
		FATAL("DataSourceStatus no id");
	if(m_id >= STREAM_ID_MAX)
		FATAL("stream id must fit in %d bits", STREAM_ID_BITS);
	dict.Get("clockDivider",&m_clockDivider,m_clockDivider);

#ifdef __linux__
	std::vector<std::string> interfaces;
	GetEthernetAdapterIPv4Adresses(&interfaces);
	for(auto& i:interfaces) {
		std::string interfaceName = GetInterfaceFromIP(i.c_str());
		if(IsWireless(interfaceName.c_str())) {
			uprintf("Interface %s is wireless\n",i.c_str());
			AdaptorInfo* ai=new AdaptorInfo();
			ai->m_rssi.SetMaxTime(20*1000000);
			ai->m_name=interfaceName;
			ai->m_ip=i;
			m_adapters.push_back(ai);
		}
	}
#endif
	return true;
}

void DataSourceStatus::AddSystemInfo(Dict* root) {
#ifdef __linux__
	struct sysinfo info;
	if(sysinfo(&info)) {
		FATAL("failed to get sysinfo");
	}
	Dict* obj=root->AddObjectNode("System");
	obj->Set("free",(int64_t)(info.freeram*info.mem_unit/1024/1024));
	obj->Set("total",(int64_t)(info.totalram*info.mem_unit/1024/1024));

	int fd_count = 0;
	DIR* dh = opendir("/proc/self/fd");
	if(dh) {
		struct dirent* entry;
		while((entry=readdir(dh))){
			fd_count++;
		}
		closedir(dh);
		obj->Set("fd", (int64_t)fd_count);
	}
#endif
}
void DataSourceStatus::End() {
	for(auto*ai:m_adapters){
		delete ai;
	}
	m_adapters.clear();
}
bool DataSourceStatus::HasFrameData(int clockIndex)const {
	if(clockIndex%m_clockDivider)
		return false;
	return true;
}

void DataSourceStatus::GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex) {
	Dict info;
	Dict& source=info.Add("source");
	auto now=GetTimeEpochMicroseconds();
	source.Set("type",DataSourceTypeToName(Type()));
	source.Set("id", m_id);
	source.Set("time", (int64_t)now);
	source.Set("index", m_dataIndex);

	//Dict info;
	Dict* rssiDict=0;
	for(AdaptorInfo *ai:m_adapters) {
		int rssi = 0;
		if(GetRSSI(&rssi, ai->m_name.c_str())) {
			if(rssiDict==0){
				rssiDict=source.AddArrayNode("RSSI");
			}
			Dict* entry=rssiDict->AddObjectNode();
			entry->Set("name",ai->m_name);
			entry->Set("rssi",rssi);
			entry->Set("ip",ai->m_ip);
		}
	}
	AddSystemInfo(&source);
	info.WriteToBinary(data);
	*dataIndex=m_dataIndex++;
}

int datasource_status_static_link_use=0;

