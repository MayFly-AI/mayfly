#include "datasource.h"
#include <thread>
#include <atomic>
#include "shared/queue.h"
#include "shared/std_ext.h"

std::string DataSource::PropertiesName()const {
	return stdx::format_string("%s %d",DataSourceTypeToName(Type()),Id());
}

class DataSourceFactory {
public:
	typedef DataSource* (*CreateDataSourceFunc)();
	typedef std::unordered_map<std::string,CreateDataSourceFunc> CreateMap;
	static bool Register(const char* name,CreateDataSourceFunc func) {
		uprintf("Registering datasource: %s\n",name);
		auto& reg=Get();
		auto str=std::string(name);
		auto found=reg.find(str);
		if(found!=reg.end()) {
			FATAL("A datasource with name \"%s\" is already registered",name);
			return false;
		}
		reg[str]=func;
		return true;
	}
	static DataSource* Create(const char* name) {
		uprintf("Creating datasource: %s\n",name);
		auto& reg=Get();
		auto str=std::string(name);
		auto found=reg.find(str);
		if(found==reg.end()) return nullptr;
		return found->second();
	}
	static CreateMap& Get() {
		static CreateMap reg;
		return reg;
	}
};

bool RegisterDataSource(const char* name,CreateDataSourceFunc func){
	return DataSourceFactory::Register(name,func);
}

DataSource* CreateDataSource(const Dict& dict) {
	std::string datasourceType;
	if(!dict.Get("type",&datasourceType)) {
		FATAL("Missing 'type' for creating datasource");
	}
	DataSource* ds=DataSourceFactory::Create(datasourceType.c_str());
	if(!ds) {
		uprintf("No datasource registered for type: %s\n",datasourceType.c_str());
		return nullptr;
	}
	ds->Begin(dict);
	return ds;
}

void DestroyDataSource(DataSource* ds) {
	ds->End();
	delete ds;
}

#ifdef STATIC_LINK_DATASOURCES
// Workaround for linker removing unused statically linked object
// Disable this when used as dynamic library.
void WorkaroundStaticLinkDataSources() {
	extern int datasource_camera_static_link_use;
	extern int datasource_clock_static_link_use;
	extern int datasource_defaultclock_static_link_use;
	extern int datasource_files_static_link_use;
	extern int datasource_imu_static_link_use;
	extern int datasource_proxy_static_link_use;
	extern int datasource_ranging_static_link_use;
	extern int datasource_status_static_link_use;
	extern int datasource_synccameramaster_static_link_use;
	extern int datasource_tof_static_link_use;
	extern int datasource_tag_static_link_use;
	extern int datasource_mag_static_link_use;

	datasource_camera_static_link_use=1;
	datasource_clock_static_link_use=1;
	datasource_defaultclock_static_link_use=1;
	datasource_files_static_link_use=1;
	datasource_imu_static_link_use=1;
	datasource_proxy_static_link_use=1;
	datasource_ranging_static_link_use=1;
	datasource_status_static_link_use=1;
	datasource_synccameramaster_static_link_use=1;
	datasource_tof_static_link_use=1;
	datasource_tag_static_link_use=1;
	datasource_mag_static_link_use=1;
}
#endif
