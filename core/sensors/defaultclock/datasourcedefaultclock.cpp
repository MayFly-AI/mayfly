#include "core/datasource.h"
#include <atomic>
#include <thread>

class DataSourceDefaultClock : public DataSource {
	public:
		DataSourceDefaultClock();
		virtual ~DataSourceDefaultClock();
		virtual bool HasClock()const{return true;}
		virtual void RunCaptureLoop(TCaptureFunc cb,void* arg);
		virtual uint8_t Type()const{return 0;}
		virtual bool Begin(const Dict& dict);
		virtual void End();
		virtual bool HasFrameData(int clockIndex)const{return false;}
		virtual void GetFrameData(std::vector<uint8_t>* data,int* dataIndex,int clockIndex){}
	protected:
		void* m_callbackFrameReadyArg;
		TCaptureFunc m_callbackFrameReady;
		int m_clock=30;					//ticks per second(fps)
		std::atomic<bool> m_close;
		std::thread m_thread;
		static DataSource* Create() {return new DataSourceDefaultClock();}
		static const bool s_registered;
};

const bool DataSourceDefaultClock::s_registered=RegisterDataSource("defaultclock",DataSourceDefaultClock::Create);

DataSourceDefaultClock::DataSourceDefaultClock() : DataSource() {
	m_close=false;
}
DataSourceDefaultClock::~DataSourceDefaultClock() {
}
bool DataSourceDefaultClock::Begin(const Dict& dict) {
	dict.Get("clock",&m_clock,m_clock);
	return true;
}
void DataSourceDefaultClock::End() {
	m_close=true;
	m_thread.join();
}
void DataSourceDefaultClock::RunCaptureLoop(TCaptureFunc cb,void* arg) {
	m_callbackFrameReady=cb;
	m_callbackFrameReadyArg=arg;
	m_thread=std::thread([&]{
#ifdef USE_THREAD_AFFINITY
		SetSelfAffinityMask(THREAD_AFFINITY_SERVER_MAIN);
#endif
		int frameIndex=0;
		const uint64_t frameWaitTime=(1000000L/(int64_t)m_clock);
		std::vector<uint8_t> data;
		while(!m_close) {
			uint64_t timeCapture=GetTimeEpochMicroseconds();
			m_callbackFrameReady(this,timeCapture,frameIndex,m_callbackFrameReadyArg);
			frameIndex++;
			uint64_t endTime=timeCapture+frameWaitTime;
			int sleepTime=(int)(endTime-GetTimeEpochMicroseconds());
			preciseSleep(sleepTime);
		}
	});
}

int datasource_defaultclock_static_link_use=0;
