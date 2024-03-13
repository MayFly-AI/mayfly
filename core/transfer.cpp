
#include <stdio.h>
#include <chrono>
#include <map>
#include <deque>
#include <thread>
#include <algorithm>

#include "shared/file.h"
#include "shared/net.h"
#include "shared/crc32.h"
#include "shared/dict.h"
#include "shared/thread.h"
#include "app.h"
#include "transfer.h"

#ifdef __linux__
#include "pcap/pcap_rxtx.h"
#include <glob.h>
#include <unistd.h>
#endif

void NetTransfer::GetGraphs(Dict* graphs,uint64_t timeMilliseconds) {
	Dict* graph=graphs->PushBack();
	graph->Set("xmax",(int64_t)timeMilliseconds);
	graph->Set("xmin",(int64_t)(timeMilliseconds-m_send.GetMaxTime()/1000));
	graph->Set("ymin",0.0);
	graph->Set("formatX","HH:MM:SS");
	graph->Set("formatY","mbits");
	Dict* legend=graph->AddObjectNode("legend");
	legend->Set("text","Transfer");
	legend->Set("color","#0000ff");
	Dict* datasets=graph->AddArrayNode("datasets");
	Dict* dataset=datasets->AddObjectNode();
	dataset->Set("name","send");
	dataset->Set("color","#dd8452");
	std::vector<double> datax;
	std::vector<double> datay;
	m_send.GetMbps(&datax,&datay,timeMilliseconds*1000);
	dataset->SetTypedArray("datax",datax.data(),datax.size());
	dataset->SetTypedArray("datay",datay.data(),datay.size());
	dataset=datasets->AddObjectNode();
	dataset->Set("name","receive");
	dataset->Set("color","#68a855");
	m_received.GetMbps(&datax,&datay,timeMilliseconds*1000);
	dataset->SetTypedArray("datax",datax.data(),datax.size());
	dataset->SetTypedArray("datay",datay.data(),datay.size());
}


class NetTransferUDP : public NetTransfer {
	public:
		virtual ~NetTransferUDP(){}
		virtual std::string Host()const{return m_host;}
		virtual uint8_t HostIndex()const;
		virtual void Begin(const Dict& settings);
		virtual void End();
		virtual bool IsDestBroadcast()const{return false;}
		virtual int SendToHost(const void* data,int len);
		virtual int SendTo(const char* addressName,const void* data,int len);
		virtual void GetStatus(Dict* status);
		virtual int PreferedPacketBytesize()const{return 1448;}
		virtual void AddGraphs(Dict* graphs,uint64_t timeMilliseconds);

		SocketAddress m_bind;
		//std::string m_bindAddress;		//bind
		Socket m_bindSocket;

		uint16_t m_port=0;				//receive port
		std::string m_host;				//send host

		std::atomic<bool> m_close;
		bool m_receiveThreadRunning=false;
		std::thread m_receiveThread;
		int ReceiveFromTime(void* buf,int buflen,void* id,uint64_t* t);
};

void NetTransferUDP::AddGraphs(Dict* graphs,uint64_t timeMilliseconds) {
	NetTransfer::GetGraphs(graphs,timeMilliseconds);

}

#ifdef __linux__

class NetTransferPCAP : public NetTransfer {
	public:
		virtual ~NetTransferPCAP(){}
		virtual void Begin(const Dict& settings);
		virtual void End();
		virtual bool IsDestBroadcast()const{return true;}
		virtual std::string Host()const{return m_dest.ToString();}
		virtual uint8_t HostIndex()const;
		virtual int SendToHost(const void* data,int len);
		virtual int SendTo(const char* addressName,const void* data,int len);
		virtual void GetStatus(Dict* status);
		virtual int PreferedPacketBytesize()const{return 1430;}
		virtual void AddGraphs(Dict* graphs,uint64_t timeMilliseconds);

		struct IPv2 {
			uint8_t m_net=0;
			uint8_t m_host=0;
			bool Valid()const{return (!m_net && !m_host)?false:true;}
			bool Set(const char* address){
				int net,host;
				int c=std::sscanf(address,"%d.%d",&net,&host);
				if(c!=2)
					return false;
				m_net=(uint8_t)net;
				m_host=(uint8_t)host;
				return true;
			}
			std::string ToString()const{return stdx::format_string("%d.%d",m_net,m_host);}
			bool ToString(char* buf)const{
				sprintf(buf,"%d.%d",m_net,m_host);
				//if(bufBytesize<3+1+3+1)return false;
				//char* p=itoa((int)m_net,buf,10);
				//itoa((int)m_net,buf,10);
				return true;
			}
		};
		struct HeaderPCAP {
			IPv2 m_source;
			IPv2 m_dest;
			uint32_t m_crc32;
		};

		IPv2 m_bind;
		IPv2 m_dest;

		History m_rssi;

		//uint16_t m_port=0;
		//uint16_t m_hostPort=0;

		std::atomic<bool> m_close;
		std::thread m_receiveThread;

		class PCAPSender* m_sender=0;
		class PCAPReceiver* m_receiver=0;
		void DataReceived(const uint8_t* data,int dataBytesize,const ReceivedDataInfo& receiveInfo);
};

void NetTransferPCAP::AddGraphs(Dict* graphs,uint64_t timeMilliseconds) {
	NetTransfer::GetGraphs(graphs,timeMilliseconds);
	Dict* graph=graphs->PushBack();
	graph->Set("xmax",(int64_t)timeMilliseconds);
	graph->Set("xmin",(int64_t)(timeMilliseconds-m_rssi.GetMaxTime()/1000));
	graph->Set("ymin",0.0);
	graph->Set("formatX","HH:MM:SS");
	graph->Set("formatY","dBm");
	Dict* legend=graph->AddObjectNode("legend");
	legend->Set("text","RSSI");
	legend->Set("color","#0000ff");
	Dict* datasets=graph->AddArrayNode("datasets");
	Dict* dataset = datasets->AddObjectNode();
	dataset->Set("name","RSSI");
	dataset->Set("color","#dd8452");
	std::vector<double> datax;
	std::vector<double> datay;
	m_rssi.Get(&datax,&datay,timeMilliseconds*1000);
	dataset->SetTypedArray("datax",datax.data(),datax.size());
	dataset->SetTypedArray("datay",datay.data(),datay.size());
}

void NetTransferPCAP::GetStatus(Dict* status) {
	status->Set("bind",m_bind.ToString());
	status->Set("dest",m_dest.ToString());

	//status->Set("port",m_port);
	//status->Set("hostPort",m_hostPort);
}

static void glob_find(std::vector<std::string>* out, const char* match) {
	glob_t gstruct;
	int ret = glob(match, GLOB_ERR, NULL, &gstruct);
	if(ret == GLOB_NOMATCH) {
		return;
	}
	if(ret) {
		FATAL("Unknown glob error");
	}
	for(char** found=gstruct.gl_pathv;*found;++found) {
		out->push_back(*found);
	}
	globfree(&gstruct);
}

static std::string GetRTL88Interface() {
	std::vector<std::string> drivers;
	glob_find(&drivers, "/sys/class/ieee80211/*/device/driver/module/drivers/*");
	ASSERT(!drivers.empty(), "Found no ieee80211 drivers");

	std::vector<std::string> nets;
	glob_find(&nets, "/sys/class/ieee80211/*/device/net/*");
	ASSERT(!nets.empty(), "Found no ieee80211 net devices");

	std::string phy;
	for(auto& d:drivers) {
		std::size_t found = d.find("rtl88");
		if(found!=std::string::npos) {
			std::size_t found = d.find("phy");
			if(found!=std::string::npos) {
				std::string d_substr = d.substr(found, std::string::npos);
				std::size_t found = d_substr.find("/");
				if(found!=std::string::npos) {
					phy = d_substr.substr(0,found);
				} else {
					FATAL("/ not found even though phy has been found\n");
				}
			} else {
				FATAL("phy not found even though rtl88 has been found\n");
			}
		}
	}
	if(phy.empty()) {
		FATAL("rtl88 not found\n");
	}
	uprintf("Found rtl88 driver on physical device %s\n",phy.c_str());

	std::string interface;
	for(auto& n:nets) {
		std::size_t found = n.find(phy);
		if(found!=std::string::npos) {
			std::size_t found = n.find_last_of("/");
			if(found!=std::string::npos) {
				interface = n.substr(found+1, std::string::npos);
			} else {
				FATAL("/ not found even though phy has been found\n");
			}
		}
	}
	if(interface.empty()) {
		FATAL("interface not found\n");
	}
	uprintf("Found rtl88 on interface %s\n",interface.c_str());
	return interface;
}

void NetTransferPCAP::Begin(const Dict& settings) {
	m_send.SetMaxTime(60*1000000);
	m_received.SetMaxTime(60*1000000);
	m_rssi.SetMaxTime(60*1000000);
	std::string address;
	if(settings.Get("bind",&address)) {
		m_bind.Set(address.c_str());
	}
	if(settings.Get("dest",&address)) {
		m_dest.Set(address.c_str());
	}
	if(m_dest.Valid()) {
		if(m_bind.Valid()) {
			uprintf("Create PCAP transfer bind %s, dest %s\n",m_bind.ToString().c_str(),m_dest.ToString().c_str());
		}else{
			uprintf("Create PCAP transfer dest %s\n",m_dest.ToString().c_str());
		}
	}else{
		if(!m_bind.Valid()) {
			uprintf("WARNING: NetTransferPCAP::Begin dest and host not defined\n");
			return;
		}
	}

	std::string interfaceName = GetRTL88Interface();
	if(m_dest.Valid()) {
		m_sender=CreatePCAPSender(interfaceName.c_str(), m_dest.m_net);
		m_sender->SetId(m_bind.m_net);
	}
	if(m_bind.Valid()) {
		m_receiver=CreatePCAPReceiver(interfaceName.c_str(), m_bind.m_net);
		m_receiver->SetId(m_bind.m_net);
		m_receiveThread=std::thread([&]{
#ifdef USE_THREAD_AFFINITY
			SetSelfAffinityMask(THREAD_AFFINITY_PCAP_RECEIVE);
#endif
			m_receiver->WaitForData([](const uint8_t* data,int dataBytesize,const ReceivedDataInfo& receiveInfo,const void* arg){
				NetTransferPCAP* nt=(NetTransferPCAP*)arg;
				nt->DataReceived(data,dataBytesize,receiveInfo);
				nt->m_rssi.RegisterMax(receiveInfo.m_time, receiveInfo.m_rssi);
				uint64_t dt=GetTimeEpochMicroseconds()-receiveInfo.m_time;
				if(dt>500) {
					uprintf("NetTransferPCAP receive data took %dus\n",(int)dt);
				}
			},this);
			uprintf("close\n");
		});
	}
}

void NetTransferPCAP::End() {
	DestroyPCAPSender(m_sender);
	m_sender=0;
	m_receiver->End();
	m_receiveThread.join();
	DestroyPCAPReceiver(m_receiver);
	m_receiver=0;
}
void NetTransferPCAP::DataReceived(const uint8_t* data,int dataBytesize,const ReceivedDataInfo& receiveInfo) {
	//uint64_t time=GetTimeEpochMicroseconds();
	//uprintf("NetTransferPCAP::DataReceived\n");
	const uint8_t* buf=data;
	if(dataBytesize<=(int)sizeof(HeaderPCAP))
		FATAL("NetTransferPCAP::ReceiveFrom WTF");
	const HeaderPCAP* ech=(const HeaderPCAP*)data;
//	if(ech->m_srcPort==m_port) {	//Packet from myself?
//		FATAL("received packet from my self\n");
//		return;
//	}
	//uprintf("received %d bytes. packet src %d dst %d. Local listen %d\n",dataBytesize,ech->m_srcPort,ech->m_dstPort,m_port);

/*
	if(!m_server) {
		if(ech->m_srcPort!=m_hostPort) {
			uprintf("not for me1\n");
			return;
		}
	}
	if(ech->m_dstPort && ech->m_dstPort!=m_port) {		//Not broadcast and not for me
		uprintf("not for me2\n");
		return;
	}
*/
	if(ech->m_dest.m_net!=m_bind.m_net) {
		uprintf("Packet destnation different net. Ignore packet\n");
		return;
	}
	if(ech->m_dest.m_host!=0xff) {		//Is broadcast
		if(ech->m_dest.m_host!=m_bind.m_host) {
			//uprintf("Packet host part not broadcast and host part of address does not match bind host. Ignore packet\n");
			return;

		}
	}

	m_packetsReceived++;
	m_bytesReceived+=dataBytesize;
	m_received.RegisterAccumulate(receiveInfo.m_time,dataBytesize);
	dataBytesize-=(int)sizeof(HeaderPCAP);
	if(CRC32(buf+sizeof(HeaderPCAP),dataBytesize)!=ech->m_crc32) {
		uprintf("NetTransferPCAP::ReceiveFrom CRC error\n");
		return;
	}
	char pbuf[64];
	ech->m_source.ToString(pbuf);
	//sprintf(pbuf,"%s",ech->m_source.ToString());
	//uprintf("NetTransferPCAP::SeDataReceived source \"%s\" dataBytesize %d\n",pbuf,dataBytesize);
	m_dataCallback(this,buf+sizeof(HeaderPCAP),dataBytesize,pbuf,receiveInfo.m_time);
}
uint8_t NetTransferPCAP::HostIndex()const {
	return m_bind.m_host;
}
int NetTransferPCAP::SendToHost(const void* data,int dataBytesize) {
	if(!m_dest.Valid())
		return 0;
	char buf[64];
	m_dest.ToString(buf);
	return SendTo(buf,data,dataBytesize);
}
int NetTransferPCAP::SendTo(const char* addressName,const void* data,int dataBytesize) {
	char buf[2048];
	if(sizeof(HeaderPCAP)+dataBytesize>(int)sizeof(buf))
		FATAL("NetTransferPCAP::SendTo dataBytesize exceed buffer size");
	HeaderPCAP* ech=(HeaderPCAP*)buf;
	if(!ech->m_dest.Set(addressName))
		FATAL("NetTransferPCAP::SendTo unable to set address %s",addressName);

	//uprintf("NetTransferPCAP::SendTo src %d dst %d\n",m_port,dstPort);
	ech->m_source=m_bind;
	ech->m_crc32=CRC32(data,dataBytesize);
	memcpy(buf+(int)sizeof(HeaderPCAP),data,dataBytesize);
	data=buf;
	dataBytesize+=(int)sizeof(HeaderPCAP);

	//if(RandomUnitFloat()<0.01f)
	//	buf[sizeof(ErrorCorrectionHeader)]++;

	//uint64_t t0=GetTimeEpochMicroseconds();
	if(!m_sender->Send(data,dataBytesize)) {
		uprintf("SendPCAP returned false\n");
	}
	//uint64_t t1=GetTimeEpochMicroseconds();
	//uprintf("inject time %d\n",t1-t0);
	m_packetsSend++;
	m_bytesSend+=dataBytesize;
	m_send.RegisterAccumulate(GetTimeEpochMicroseconds(),dataBytesize);
	return dataBytesize;
}
#endif


void NetTransferUDP::Begin(const Dict& settings) {

	m_send.SetMaxTime(60*1000000);
	m_received.SetMaxTime(60*1000000);
	std::string hostName;
	const Dict* host=settings.Find("hostAddress");
	if(host) {
		std::string ip;
		host->Get("ip",&ip);
		int port;
		if(!host->Get("port",&port,0))
			FATAL("transfer hostAddress must define port");
		//uprintf("host %s %d\n",ip.c_str(),port);
		hostName=stdx::format_string("%s:%d",ip.c_str(),port);
	}else{
		settings.Get("host",&hostName);
	}
	const Dict* bind=settings.Find("bindAddress");
	bool portOk;
	if(bind) {
		std::string ip;
		bind->Get("ip",&ip,"0.0.0.0");
		int port;
		if(!bind->Get("port",&port,0))
			FATAL("transfer bindAddress must define port");
		//uprintf("host %s %d\n",ip.c_str(),port);
		//m_bindAddress=stdx::format_string("%s:%d",ip.c_str(),port);
		m_bind.Set(ip.c_str());
		m_bind.PortSet(port);
		portOk=true;
		m_port=port;
	}else{
		FATAL("NetTransferUDP::Begin bind address not found");
		int port;
		portOk=settings.Get("port",&port);
		if(!portOk)
			portOk=settings.Get("listenPort",&port,0);
		m_port=port;
	}
	//m_server=server;
	m_close=false;
	m_host=hostName;
	m_bindSocket=Socket(Socket::INET,Socket::DGRAM,Socket::UDP);
	m_bindSocket.SetBlockingMode(true);
	m_bindSocket.SetSendTimeout(10);
	m_bindSocket.EnableTimestampRX();
	if(!portOk) {
		m_receiveThreadRunning=false;
		return;
	}
	if(!m_bind.IsValid()) {
		m_bind.SetAny();
		if(m_port==0) {			//Find available port
			m_port=3000;
			while(true) {
				m_bind.PortSet(m_port);
				if(m_bindSocket.Bind(m_bind))
					break;
				if(m_port++==3100)
					FATAL("Unable to bind test client\n");
			}
			uprintf("NetTransferUDP::Begin listen port %d\n",m_port);
		}else{
			m_bind.PortSet(m_port);
		}
	}
	if(!m_bindSocket.Bind(m_bind))
		FATAL("NetTransferUDP::Begin Unable to bind address %s\n",m_bind.ToString().c_str());
	uprintf("NetTransferUDP::Begin bind address %s\n",m_bind.ToString().c_str());

	std::atomic<bool> ready=false;
	m_receiveThreadRunning=true;
	m_receiveThread=std::thread([&] {
#ifdef USE_THREAD_AFFINITY
		SetSelfAffinityMask(THREAD_AFFINITY_UDP_RECEIVE);
#endif
		ready=true;
		uint8_t buf[0x10000];
		while(!m_close) {
			SocketAddress source;
			uint64_t timeEpochMicroseconds;
			int ret=ReceiveFromTime(&buf,sizeof(buf)-1,&source,&timeEpochMicroseconds);
			if(ret==0)
				break;
			if(ret>0) {
				if(ret>=(int)sizeof(buf))
					FATAL("UDP received %d bytes, buffer size %d\n",ret,sizeof(buf));
				if(m_dataCallback)
					m_dataCallback(this,buf,ret,source.ToString().c_str(),timeEpochMicroseconds);
	 			//uprintf("UPDServer received %d bytes from %s\n",ret,source.ToString().c_str());
			}else{
				Socket::Error error=Socket::GetLastError();
				if(m_errorCallback) {
					m_errorCallback(this,source.ToString().c_str(),Socket::ErrorCodeToName(error));
				}
				//	uprintf("NetTransferUDP port %d host %s ret %d error \"%s\"\n",m_port,m_host.c_str(),ret,Socket::ErrorCodeToName(error));
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}
		}
		uprintf("NetTransferImpl end listen thread\n");
	});
	while(!ready);
}

void NetTransferUDP::End() {
	m_close=true;
	if(m_receiveThreadRunning) {
		if(m_bind.IsAny()) {
			m_bindSocket.SendTo(SocketAddress(127,0,0,1,m_port),"",0);
		}else{
			m_bindSocket.SendTo(m_bind,"",0);
		}
		//uprintf("NetTransferUDP::End %s\n",m_bind.ToString().c_str());
		m_receiveThread.join();
	}
	//uprintf("call destructor for socket port %d\n",m_port);
	//uprintf("NetTransferImpl::End\n");
}

void NetTransferUDP::GetStatus(Dict* status) {
	status->Set("port",m_port);
	status->Set("host",m_host);
}

uint8_t NetTransferUDP::HostIndex()const {
	return m_bind.HostIndex();
}
int NetTransferUDP::SendToHost(const void* data,int dataBytesize) {
	if(!m_host.size())
		return 0;
	return SendTo(m_host.c_str(),data,dataBytesize);
}
#if 1
int NetTransferUDP::SendTo(const char* addressName,const void* data,int dataBytesize) {
	SocketAddress address;
	address.Set(addressName);
	int ret=m_bindSocket.SendTo(address,data,dataBytesize);
	if(ret>0) {
		if(dataBytesize!=ret)
			uprintf("NetTransferUDP::SendTo %s should have send %d bytes but returned %d bytes\n",addressName,dataBytesize,ret);
		m_packetsSend++;
		m_bytesSend+=ret;
		m_send.RegisterAccumulate(GetTimeEpochMicroseconds(),ret);
	}else{
		uprintf("UDP send to returned %d\n",ret);
	}
	return ret;
}
int NetTransferUDP::ReceiveFromTime(void* buf,int buflen,void* id, uint64_t* t) {
	SocketAddress* address=(SocketAddress*)id;
	int ret=m_bindSocket.ReceiveFromTime(buf,buflen,address,t);
	if(ret>0) {
		m_packetsReceived++;
		m_bytesReceived+=ret;
		m_received.RegisterAccumulate(GetTimeEpochMicroseconds(),ret);
	}
	return ret;
}
#else
struct ErrorCorrectionHeader {
	uint32_t m_count;
	uint32_t m_crc32;
};
int NetTransferUDP::SendTo(const char* addressName,const void* data,int dataBytesize) {
	char buf[0x10000];
	if(dataBytesize>(int)sizeof(buf))
		FATAL("NetTransferImpl::SendTo dataBytesize exceed buffer size");
	static uint32_t count=1;
	((ErrorCorrectionHeader*)buf)->m_count=count++;
	((ErrorCorrectionHeader*)buf)->m_crc32=CRC32(data,dataBytesize);
	memcpy(buf+(int)sizeof(ErrorCorrectionHeader),data,dataBytesize);
	data=buf;
	dataBytesize+=(int)sizeof(ErrorCorrectionHeader);

	//static int wcount=200;
	//if(!wcount--) {
	//	wcount=200;
		if(RandomUnitFloat()<0.01f) {
			buf[sizeof(ErrorCorrectionHeader)]++;
		}
	//}
	SocketAddress address;
	address.Set(addressName);
	int ret=m_bindSocket.SendTo(address,data,dataBytesize);
	if(ret>0) {
		m_packetsSend++;
		m_bytesSend+=ret;
		m_send.RegisterAccumulate(GetTimeEpochMicroseconds(),ret);
	}else{
		uprintf("UDP send to returned %d\n",ret);
	}
	return ret;
}
int NetTransferUDP::ReceiveFrom(void* buf,int bufBytesize,void* id) {
	SocketAddress* address=(SocketAddress*)id;
	char rbuf[0x10000];
	int ret=0;
	while(true) {
		ret=m_bindSocket.ReceiveFrom(rbuf,(int)sizeof(rbuf),address);
		if(ret>0) {
			m_packetsReceived++;
			m_bytesReceived+=ret;
			m_received.RegisterAccumulate(GetTimeEpochMicroseconds(),ret);
			if(ret>bufBytesize)
				FATAL("NetTransferUDP::ReceiveFrom bufBytesize exceed buffer size");
			if(ret<=(int)sizeof(ErrorCorrectionHeader))
				FATAL("NetTransferUDP::ReceiveFrom WTF");
			ret-=(int)sizeof(ErrorCorrectionHeader);
			memcpy(buf,rbuf+(int)sizeof(ErrorCorrectionHeader),ret);
			if(CRC32(buf,ret)!=((ErrorCorrectionHeader*)rbuf)->m_crc32) {
				uprintf("NetTransferUDP::ReceiveFrom CRC error\n");
				continue;
			}
		}
		break;
	}
	return ret;
}
#endif

NetTransfer* CreateNetTransfer(const Dict* settings) {
	std::string type;
	if(!settings->Get("protocol",&type))
		settings->Get("type",&type);

	NetTransfer* nt=0;
	if(type=="udp") {
		nt=new NetTransferUDP();
	}else
	if(type=="pcap") {
#ifdef __linux__
		if(getuid()) {
			FATAL("Transfer mode 'pcap' is only available as root");
		}
		nt=new NetTransferPCAP();
#else
		FATAL("Transfer mode 'pcap' is only available on linux");
#endif
	}else{
		FATAL("Unknown transfer type '%s'",type.c_str());
	}
	nt->Begin(*settings);
	return nt;
}
void DestroyNetTransfer(NetTransfer* nt) {
	nt->End();
	delete nt;
}

