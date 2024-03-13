#pragma once

#include <stdint.h>
#include "shared/types.h"
#include "shared/dict.h"

#include "core/service.h"
#include "core/datasource.h"

class Dict;

class Ranging {
	public:
		Ranging();
		virtual ~Ranging();
		virtual uint8_t Type()const{return DS_TYPE_RANGING;}
		virtual bool SendPing(uint16_t* time,float* distance)=0;
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex){}
		virtual bool HasFrameData(int clockIndex)const{return false;}
		int m_packetsSend=0;
		int m_packetsReceived=0;
};

Ranging* CreateRanging(const Dict& dict);
void DestroyRanging(Ranging*);
