#ifdef _WIN32
#include <windows.h>
#include <malloc.h>
#endif

#ifdef __linux__
#include <string.h> // memcpy
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <chrono>
#include <thread>

#include "shared/types.h"
#include "shared/misc.h"
#include <ctime>

TPrintCallbackFunc g_printCallback=0;

void SetPrintCallback(TPrintCallbackFunc pcb) {
	g_printCallback=pcb;
}

uint64_t GetTimer() {
	//auto t=std::chrono::high_resolution_clock::now().time_since_epoch();
	uint64_t microsecondsUTC=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	return microsecondsUTC;
}

uint64_t GetTimeEpochMicroseconds() {
	return GetTimer();
}
uint64_t GetTimeEpochMilliseconds(){
	return GetTimer()/1000L;
}

uint64_t ElapsedMilliseconds(uint64_t t) {
	return ElapsedMicroseconds(t)/1000;
}

uint64_t ElapsedMicroseconds(uint64_t t) {
	return GetTimer()-t;
}

uint64_t GetTimeMicroseconds() {
	static uint64_t st=GetTimer();
	return GetTimer()-st;
}


std::string TimeEpochMicrosecondsToString(uint64_t microsecondsUTC,bool milliseconds,const char* sd,const char* st) {
	uint64_t secondsSince1970=microsecondsUTC/1000000;
	time_t rawtime=secondsSince1970;
	struct tm ts;
	//(void)gmtime_r(&rawtime,&ts);
#if _WIN32
	(void)gmtime_s(&ts,&rawtime);
#else
	(void)gmtime_r(&rawtime,&ts);
#endif
	char buf[512];
	if(milliseconds) {
		snprintf(buf,sizeof(buf),"%d%s%02d%s%02d%s%02d%s%02d%s%02d%s%03d",ts.tm_year+1900,sd,ts.tm_mon+1,sd,ts.tm_mday,sd,ts.tm_hour,st,ts.tm_min,st,ts.tm_sec,st,(int)((microsecondsUTC/1000)%1000));
	}else{
		snprintf(buf,sizeof(buf),"%d%s%02d%s%02d%s%02d%s%02d%s%02d",ts.tm_year+1900,sd,ts.tm_mon+1,sd,ts.tm_mday,sd,ts.tm_hour,st,ts.tm_min,st,ts.tm_sec);
	}
	return buf;
}

void preciseSleep(int sleepTime) {
#ifdef _WIN32
	if(sleepTime>20000) {			//Semi busywaiting due to windows lousy sleep precision
		std::this_thread::sleep_for(std::chrono::microseconds(10000));
	}
	uint64_t endTime=GetTimeEpochMicroseconds()+sleepTime;
	while(true) {
		if(endTime<GetTimeEpochMicroseconds())
			break;
	}
#else
	std::this_thread::sleep_for(std::chrono::microseconds(sleepTime));
#endif
}

int PrintPrepend(char* buf,size_t bufByteSize) {
	uint64_t microsecondsUTC=GetTimer();
	uint64_t secondsSince1970=microsecondsUTC/1000000;
	time_t rawtime=secondsSince1970;
	struct tm ts;
	//(void)gmtime_r(&rawtime,&ts);
#if _WIN32
	(void)gmtime_s(&ts,&rawtime);
#else
	(void)gmtime_r(&rawtime,&ts);
#endif

#if 0
	int tlen=snprintf(buf,bufByteSize,"[%d/%02d/%02d %02d:%02d:%02d:%03d] ",ts.tm_year+1900,ts.tm_mon+1,ts.tm_mday,ts.tm_hour,ts.tm_min,ts.tm_sec,(int)((microsecondsUTC/1000)%1000));
#else
	int tlen=snprintf(buf,bufByteSize,"[%d %02d %02d %02d:%02d:%02d] ",ts.tm_year+1900,ts.tm_mon+1,ts.tm_mday,ts.tm_hour,ts.tm_min,ts.tm_sec);
#endif
	return tlen;
}

extern "C" {
	void Print(const char* format,...){
		thread_local static bool newline=true;
		va_list v;
		va_start(v,format);
		int len=vsnprintf(NULL,0,format,v);
		va_end(v);
		char tstr[100];
		int tlen=newline?PrintPrepend(tstr,sizeof(tstr)):0;
		char* str=(char*)malloc(tlen+len+1);
		if(tlen)
			memcpy(str,tstr,tlen);
		va_start(v,format);
		vsnprintf(str+tlen,len+1,format,v);
		va_end(v);
		str[tlen+len]=0;
		newline=str[tlen+len-1]=='\n';
		::printf("%s",str);
#ifdef _WIN32
		OutputDebugString(str);
#endif
		if(g_printCallback)
			g_printCallback(str);
		free(str);
	}
	void Fatal(const char* file,int line,const char* func,const char* format,...){
		static const char* pformat = "FATAL %s\n\t%s:%d : ";
		int plen=snprintf(nullptr,0,pformat,func,file,line);
		va_list v;
		va_start(v,format);
		int len=vsnprintf(nullptr,0,format,v);
		va_end(v);
#ifdef _WIN32
		char* str=(char*)_malloca(len+plen+2);
#else
		char* str=(char*)malloc(len+plen+2);
#endif
		snprintf(str,plen+1,pformat,func,file,line);
		int offset = plen;
		va_start(v,format);
		vsnprintf(str+offset,len+1,format,v);
		va_end(v);
		offset += len;
		str[offset++]='\n';
		str[offset++]=0;
		uprintf("%s",str);
#ifdef _WIN32
		OutputDebugString(str);
#endif
		if(g_printCallback)
			g_printCallback(str);
#ifdef _WIN32
		_freea(str);
#else
		free(str);
		fflush(0);
#endif
	}
};
