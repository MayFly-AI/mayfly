#pragma once

#include <stdint.h>
#include "shared/misc.h"
#include "shared/types.h"
#include "shared/dict.h"

#define DS_TYPE_NA 0
#define DS_TYPE_VIDEO 1
#define DS_TYPE_CLOCK 2
#define DS_TYPE_CAMERA 3
#define DS_TYPE_RANGING 4
#define DS_TYPE_PROXY 5
#define DS_TYPE_SYNCCAMERA_MASTER 6
#define DS_TYPE_STATUS 7
#define DS_TYPE_TOF 8
#define DS_TYPE_IMU 9
#define DS_TYPE_TAG 10
#define DS_TYPE_MAG 11

#define DS_TYPE_BITS 4

#define DS_TYPE_MAX (1<<DS_TYPE_BITS)

#define STREAM_ID_BITS 6

#define STREAM_ID_MAX (1<<STREAM_ID_BITS)

#define STREAM_FORMAT_NA 0
#define STREAM_FORMAT_CUSTOM 3


static bool DataSourceTypeValid(uint8_t type) {
	if(type==DS_TYPE_NA || type>=DS_TYPE_MAX)
		return false;
	return true;
}

static inline const char* DataSourceTypeToName(uint8_t type){
	switch(type) {
		case DS_TYPE_VIDEO:return "video";
		case DS_TYPE_CLOCK:return "clock";
		case DS_TYPE_CAMERA:return "camera";
		case DS_TYPE_RANGING:return "ranging";
		case DS_TYPE_PROXY:return "proxy";
		case DS_TYPE_SYNCCAMERA_MASTER:return"synccameramaster";
		case DS_TYPE_STATUS:return "status";
		case DS_TYPE_TOF:return "tof";
		case DS_TYPE_IMU:return "imu";
		case DS_TYPE_TAG:return "tag";
		case DS_TYPE_MAG:return "mag";
		default:
			break;
	}
	return "NA";
}
class DataSource;

typedef void(*TCaptureFunc)(DataSource* dataSource,uint64_t timeCapture,int index,void* arg);

class DataSource {
	public:
		virtual ~DataSource(){}

		virtual uint8_t Type()const{return DS_TYPE_NA;}
		virtual uint8_t Id()const{return 0;}
		virtual bool Begin(const Dict& settings)=0;
		virtual void End()=0;
		virtual bool HasFrameData(int clockIndex)const=0;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex)=0;
		virtual bool HasClock()const{return false;}
		virtual void RunCaptureLoop(TCaptureFunc cb,void* arg){}
		virtual std::string PropertiesName()const;
		virtual void GetProperties(Dict* properties){}
		virtual void SetProperties(const Dict& properties){};
		virtual void SetTimeOffset(int64_t offset){}
};


// Factory
typedef DataSource*(*CreateDataSourceFunc)();
bool RegisterDataSource(const char* name, CreateDataSourceFunc func);

DataSource* CreateDataSource(const Dict& dict);
void DestroyDataSource(DataSource* ds);
