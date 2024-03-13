#pragma once

#include <stdint.h>

#ifdef __cplusplus
#include <string>
#include <functional>
extern "C" {
#endif
void Print(const char* format,...);
void Fatal(const char* file,int line,const char* func,const char* format,...);
#ifdef __cplusplus
}
#endif

#define uprintf(...)Print(__VA_ARGS__)

#ifdef __cplusplus
typedef std::function<void(const char* str)> TPrintCallbackFunc;
void SetPrintCallback(TPrintCallbackFunc pcb);
std::string TimeEpochMicrosecondsToString(uint64_t microsecondsUTC,bool milliseconds=false,const char* sd=" ",const char* st=":");
#endif

uint64_t GetTimer();
uint64_t GetTimeEpochMicroseconds();
uint64_t GetTimeEpochMilliseconds();
uint64_t GetTimeMicroseconds();
uint64_t ElapsedMilliseconds(uint64_t t);
uint64_t ElapsedMicroseconds(uint64_t t);
void preciseSleep(int sleepTime);

#ifdef _WIN32
#define DEBUGBREAK __debugbreak
#else
#define DEBUGBREAK __builtin_trap
#endif

#define FATAL(...)do{Fatal(__FILE__,__LINE__,__func__,__VA_ARGS__);DEBUGBREAK();}while(0)
#define ASSERT(x,...)do{if(!(x)){Fatal(__FILE__,__LINE__,__func__,__VA_ARGS__);DEBUGBREAK();}}while(0)
