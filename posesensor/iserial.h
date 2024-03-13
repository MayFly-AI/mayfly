#pragma once
#include <stdint.h>
#include <functional>

typedef std::function<void(const uint8_t* data,int len,void* arg)> TDataReadyCallbackFunc;

class ISerialConnection {
public:
	virtual ~ISerialConnection(){};
	virtual void SetCallback(TDataReadyCallbackFunc f, void* arg) = 0;
	virtual void Run() = 0;
	virtual void Stop() = 0;
	virtual int Write(const uint8_t* data, int len) = 0;
};

ISerialConnection* CreateSerialConnection(const char* name);
void DestroySerialConnection(ISerialConnection* con);
