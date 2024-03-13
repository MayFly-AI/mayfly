#pragma once

#include <stdio.h>
#include <stdint.h>
#include "shared/dict.h"

class Service {
	public:
		enum eType {
			NA=0,
			VIDEO_SERVER=1,
			VIDEO_CLIENT=2,
			APP=3
		};
		virtual eType Type(){return NA;}
		virtual bool SaveConfig(Dict* config)=0;
		virtual bool Begin(Dict* config)=0;
		virtual void End()=0;
		virtual const std::string& Id()const=0;
		virtual void GetStatus(Dict* status,bool includeSchema,bool includeGraphs)=0;
		virtual void GetProperties(Dict* properties)=0;
		virtual void SetProperties(const Dict& properties)=0;
		Service* m_owner=0;
};
