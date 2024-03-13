#include "posesensor/iserial.h"
#include "posesensor/usb.h"
#include <stdio.h>
#include <thread>
#include <chrono>
#include <cstring>

int main(int argc,const char* argv[]) {
	const char* dev_name="/dev/ttyACM0";
	if(argc==2) {
		dev_name=argv[1];
	}
	printf("using connection device/port: %s\n",dev_name);

	ISerialConnection* con=CreateSerialConnection(dev_name);
	std::vector<uint8_t> parseBuffer;

	auto callback=[&](const uint8_t* new_data,int len,void* arg){
		parseBuffer.insert(parseBuffer.end(),new_data,new_data+len);
		if(parseBuffer[0]==PT_IMU1 && parseBuffer[1]==sizeof(IMU1Packet)) {
			if (parseBuffer.size() < sizeof(IMU1Packet)) return;
			parseBuffer.erase(parseBuffer.begin(),parseBuffer.begin()+sizeof(IMU1Packet));
		}
		else if(parseBuffer[0]==PT_SYS1 && parseBuffer[1]==sizeof(SYS1Packet)) {
			if (parseBuffer.size() < sizeof(SYS1Packet)) return;
			parseBuffer.erase(parseBuffer.begin(),parseBuffer.begin()+sizeof(SYS1Packet));
		}
		else if(parseBuffer[0]==PT_UWB1 && parseBuffer[1]==sizeof(UWB1Packet)) {
			if (parseBuffer.size() < sizeof(UWB1Packet)) return;
			parseBuffer.erase(parseBuffer.begin(),parseBuffer.begin()+sizeof(UWB1Packet));
		}
		else {
			// assuming newline terminated string message
			char sbuf[256];
			size_t i=0,k=0;
			for(;i<parseBuffer.size() && parseBuffer[i]!='\n'; ++i) {
				if(k<sizeof(sbuf)-1) {
					sbuf[k++]=parseBuffer[i];
				}
			}
			if(i==parseBuffer.size()) {
				return;
			}
			sbuf[k]=0;
			printf("GOT STRING MESSAGE: %s\n",sbuf);
			parseBuffer.erase(parseBuffer.begin(),parseBuffer.begin()+i+1);
		}
	};

	con->SetCallback(callback,nullptr);
	con->Run();

	int iter=0;
	char sbuf[1024];
	while(iter++ < 10) {
		sprintf(sbuf,"message %d from host",iter);
		con->Write((uint8_t*)sbuf,(int)strlen(sbuf));
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	printf("stopping\n");
	con->Stop();

	DestroySerialConnection(con);
	return 0;
}
