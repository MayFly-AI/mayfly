#pragma once

#include <string>
#include <set>
#include <atomic>
#include <unordered_map>

#include "shared/types.h"
#include "shared/queue.h"
#include "shared/dict.h"

#include "sensorserver.h"
#include "sensorclient.h"
#include "packet.h"
#include "service.h"
#include "datasource.h"
#include "history.h"

#if PLATFORM_RPI
#define USE_THREAD_AFFINITY			//Onlyuse thread affinity on pi

#define THREAD_AFFINITY_MAIN (1<<0)
#define THREAD_AFFINITY_PCAP_RECEIVE (1<<0)
#define THREAD_AFFINITY_UDP_RECEIVE (1<<1)
#define THREAD_AFFINITY_CLIENT_MAIN (1<<3)
#define THREAD_AFFINITY_SERVER_MAIN (1<<2)			//Camera read

#endif

class HttpServer;
class NetTransfer;
class H264Decoder;
class FrameDecoder;
struct DebugConnection;
struct DecodedFrameInfo;
struct Tensor;

std::string Color2String(uint32_t color);
uint32_t String2Color(const char* colorString);

typedef std::function<void(Dict& frame,const uint8_t* data,int dataBytesize,void* arg)> TFrameDecodedCallbackFunc;
typedef std::function<void(const char* clientId,uint8_t streamId,Dict& streamInfo,std::vector<Tensor>& tensors,void* arg)> TFrameTensorCallbackFunc;


#define NUMBER_MEASUREMENTS 4

struct DecodedFrameInfo {
	int streamId;
	int frameIndex;
	uint64_t captureTime;
};

class Timers {
	public:
		Timers();
		struct ReadTimer {
			uint64_t m_time;
			int m_values[NUMBER_MEASUREMENTS];
		};
		struct Timer {
			std::string m_name;
			uint32_t m_color;
		};
		std::vector<Timer> m_timers;
		void SetTimerInfo(int index,const char* name,uint32_t color);
		std::deque<ReadTimer> m_measurements;
		void PushTime(int v0,int v1,int v2,int v3);
		std::mutex m_timersLock;
};

class ActiveService {
	public:
		virtual ~ActiveService();
		Service::eType Type()const{return m_service->Type();}
		const std::string& Id()const{return m_service->Id();}
		Service* m_service=0;
};

class ActiveServer : public ActiveService{
	public:
		ActiveServer(){}
		virtual ~ActiveServer();
};

class ActiveClient : public Timers, public ActiveService {
	public:
		ActiveClient(){}
		virtual ~ActiveClient();
		FrameDecoder* GetOrCreateFrameDecoder(uint8_t streamId,const std::string& encoding);
		Dict m_decoderDict;
		int m_index=0;
		std::unordered_map<uint8_t, FrameDecoder*> m_frameDecoders;
};

class HistoryLog {
	public:
		struct Entry {
			uint64_t m_time;
			std::string m_str;
		};
		void Add(uint64_t time,const char* str);
		void GetLastEntries(std::vector<HistoryLog::Entry>* strings,int count);
		uint64_t GetNewEntries(std::vector<HistoryLog::Entry>* strings,uint64_t time);
		std::mutex m_lock;
		std::deque<Entry> m_strings;
};

class App : public Service {
	public:
		App();
		virtual ~App();
		virtual eType Type(){return APP;}
		enum eEventId {
			EV_SAVE,						//Save - Traverse all objects and update config and then save file
			EV_RESTART,						//Restart - Destroy all objects and start over without closing app
			EV_SAVE_RESTART,				//Save & Restart
			EV_EXIT_AND_RESTART,			//Exit app and restart
			EV_EXIT_AND_REBOOT,				//Exit app and reboot
			EV_EXIT							//Exit without restarting, this is fatal and should not happen ever
		};
		bool Running()const{return m_running;}
		static void PostEvent(eEventId eventId,const uint8_t* data=0,int dataBytesize=0);
		void PollEvents(bool block);
		bool ShouldExitMainLoop()const;
		std::atomic<bool> m_running={false};
		int Run(const std::string& configFileName);

		std::string m_defaultJson;
		virtual bool SaveConfig(Dict* config);
		virtual const std::string& Id()const{return m_id;}
		virtual void GetStatus(Dict* status,bool includeSchema,bool includeGraphs);
		virtual void GetLog(Dict* log,const Dict& request);

		virtual void GetProperties(Dict* properties){}
		virtual void SetProperties(const Dict& properties){}

		virtual bool SetProperties(const Dict& properties,const std::string& id);
		virtual void GetDebugConnections(Dict* dict)const;
		virtual void SetStatusForConnection(const std::string& id,bool enabled);
		virtual bool HasConnection(const std::string& id);
		virtual bool GetProperties(Dict* properties,const std::string& id);
		virtual bool UpdateProperties(const std::string& id);
		virtual void Update(int tick);
		virtual std::vector<std::string> GetDisplayClients()const;
		virtual void ClearFrameDecodedCallback(){m_frameDecodedCallback.m_callback=0;m_frameDecodedCallback.m_callbackTensor=0;}
		virtual void ClearFrameEncodedCallback(){m_frameEncodedCallback=0;}
		virtual void SetFrameEncodedCallback(void* arg,TFrameEncodedCallbackFunc cb) {m_frameEncodedCallbackArg=arg;m_frameEncodedCallback=cb;}
		virtual void SetFrameDecodedCallback(void* arg,TFrameDecodedCallbackFunc cb,TFrameTensorCallbackFunc cbt){
			m_frameDecodedCallback.m_callback=cb;
			m_frameDecodedCallback.m_callbackTensor=cbt;
			m_frameDecodedCallback.m_arg=arg;
		}

		struct FrameVideoCallback {
			TFrameDecodedCallbackFunc m_callback=0;
			TFrameTensorCallbackFunc m_callbackTensor=0;
			void* m_arg=0;
		};
		FrameVideoCallback m_frameDecodedCallback;

		void* m_frameEncodedCallbackArg;
		TFrameEncodedCallbackFunc m_frameEncodedCallback=0;

		bool Begin(Dict* dict);
		void End();

		HttpServer* m_httpServer=0;
		std::vector<ActiveService*> m_activeServices;

	protected:

		std::string m_configFileName;
		Dict m_config;
		struct Event {
			eEventId m_eventId;
			std::vector<uint8_t> m_data;
		};
		static FixedQueueMT<Event,16> m_queue;

		int m_tick=0;
		bool m_restart=false;
		bool m_close=false;

		void OnEvent(App::eEventId eventId,const uint8_t* data,int dataBytesize);
		virtual void MainLoop();
		void CollectAndSaveConfig();

		HistoryLog m_log;
		void UpdateHttpServer();
		void UpdateRemoteHttpServer();
		void UpdateDashboardHttpServer();
		bool GetServices(Dict* response);
		bool GetPropertiesFromBrowser(const Dict& request)const;
		bool GetStatusFromBrowser(const Dict& request)const;
		void SendStatusToBrowser(const StatusHeader& statusHeader,const Dict& status);
		void SendPropertiesToBrowser(const PropertiesHeader& propertiesHeader,const Dict& properties);

		NetTransfer* m_debugTransfer=0;
		void OnDataDebug(NetTransfer* transfer,const uint8_t* data,int dataBytesize,const char* sourceName,uint64_t timeReceived);

		struct DebugTiming {
			Timers m_timers;
			std::string m_host;
		};
		DebugTiming m_debugTiming;

		std::vector<DebugConnection*> m_debugConnections;
		bool FindConnection(const std::string& id,std::function<void(DebugConnection* dc,int connectionIndex,int componentIndex)> cb) const;
		Service* FindService(const std::string& id);
		bool AddActiveService(ActiveService* s);

		bool m_verbose=false;
		std::string m_id;
		std::string m_sessionLogsPath;
};
