#include <math.h>

#include "shared/types.h"
#include "shared/file.h"
#include "shared/std_ext.h"
#include "shared/misc.h"
#include "shared/dict.h"

#include "memory/tensor.h"
#include "core/app.h"
#include "video/decoder.h"

std::string g_defaultJson=R"(
{
    "id":"local-mayfly",
    "debugTransfer":{
        "protocol":"udp",
        "bindAddress":{
            "ip":"0.0.0.0",
            "port":7777
        }
    },
    "services":[
        {
            "type":"sensorServer",
            "id":"mayfly_server0",
            "errorCorrection":{
                "type":"fec"
            },
            "transfer":{
                "protocol":"udp",
                "bindAddress":{
                    "ip":"0.0.0.0",
                    "port":8999
                }
            },
            "sources":[
                {
                    "id":3,
                    "type":"files",
                    "path":"$(DATA)/video/exploded/office-17-10-22/",
                    "clock":30,
                    "mode":"720p: 1280x720"
                }
            ]
        }
    ]
}
)";

class AppFrames : public App {
	public:
		AppFrames();
		virtual ~AppFrames();
		virtual void MainLoop();

};
AppFrames::AppFrames() {
	m_defaultJson=g_defaultJson;
}
AppFrames::~AppFrames() {
}
void AppFrames::MainLoop() {
	//for(Device* device:m_devices) {
		SetFrameDecodedCallback(this,
			[](Dict& frame,const void* data,int dataBytesize,void* arg){
				frame.Dump();
			},
			[](const char* clientId,uint8_t streamId,Dict& streamInfo,std::vector<Tensor>& tensors,void* arg) {
				uprintf("Tensor callback\n");
				streamInfo.Dump();
			}
		);
	//}
	App::MainLoop();
}

void TestDict();
void TestTensor();
/*
#include "core/transfer.h"

void TestUDP() {
	InitSockets();
	static std::thread* tp=new std::thread([]{

		for(int j=0;j!=2;j++) {
			uprintf("create test transfer");
			Dict dict;
			dict.ReadFromJson(R"({"protocol":"udp","bindAddress":{"ip":"127.0.0.1","port":4003},"hostAddress":{"ip":"127.0.0.1","port":8999}})");
			NetTransfer* ntpeer0=CreateNetTransfer(&dict);
			std::this_thread::sleep_for(std::chrono::milliseconds(2000));
			for(int i=0;i!=5;i++) {
				uint64_t t=GetTimeEpochMicroseconds();
				uprintf("request more frames\n");
				RequestFramesHeader rfh;
				rfh.m_timeSend=t;
				ntpeer0->SendToHost(&rfh,(int)sizeof(rfh));
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}
			uprintf("close test transfer");
			DestroyNetTransfer(ntpeer0);
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		}
	});
}
*/

int main(int argc, char *argv[]) {
//	TestDict();
//	TestTensor();
	if(DirectoryExists(GetExecutableDir()+"/data")) {
		AddFilePathRemap("$(DATA)",GetExecutableDir()+"/data");
	}else{
#ifdef CMAKE_SOURCE_DIR
		AddFilePathRemap("$(DATA)",std::string(CMAKE_SOURCE_DIR)+"/data");
#else
		AddFilePathRemap("$(DATA)",GetExecutableDir()+"/data");
#endif
	}
	AddFilePathRemap("$(EXE)",GetExecutableDir());
#ifdef WIN32
	if(DirectoryExists("d:/")) {
		AddFilePathRemap("$(HOME)","d:/");
	}else{
		AddFilePathRemap("$(HOME)",getenv("HOMEPATH"));
	}
#else
	AddFilePathRemap("$(HOME)",getenv("HOME"));
#endif
	std::string configFileName;
	if(argc>1) {
		configFileName=argv[argc-1];
	}else{
		configFileName="$(HOME)/config.json";
	}
	uprintf("exe directory %s\n",GetFileNameRemap("$(EXE)").c_str());
	uprintf("data directory %s\n",GetFileNameRemap("$(DATA)").c_str());
	//TestUDP();

	AppFrames app;
	return app.Run(configFileName);
}

#ifdef _WIN32
#undef APIENTRY
#include<windows.h>
#include "debugapi.h"
#include<crtdbg.h>
int __stdcall WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, char*, int nShowCmd) {
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
	_CrtSetBreakAlloc(98);
    return main(__argc, __argv);
}
#endif


