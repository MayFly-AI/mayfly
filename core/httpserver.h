#pragma once

typedef std::function<void(const char* json,int jsonBytesize,void* arg)> TWebDataCallbackFunc;

class HttpServer {
	public:
		virtual ~HttpServer(){};
		virtual void Begin()=0;
		virtual void End()=0;
		virtual void Send(const Dict& status)=0;
		virtual bool HasConnectedClients()const=0;
		virtual void GetStatus(Dict* status)=0;
		virtual std::string GetBindAddress()const=0;
		virtual void SetWebsocketDataCallback(void* arg,TWebDataCallbackFunc cb)=0;
		virtual void ClearWebsocketDataCallback()=0;
};

HttpServer* CreateHttpServer(const Dict& ini);
void DestroyHttpServer(HttpServer* s);
