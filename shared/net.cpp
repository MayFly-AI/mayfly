#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>		//GetAdaptersInfo
#else

#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <ifaddrs.h>
#include <signal.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <limits.h>
#include <sys/time.h>
#include "unistd.h"
#include <assert.h>
#ifdef __linux__
#include <linux/wireless.h>
#include <linux/net_tstamp.h>
#endif

#define INVALID_SOCKET -1
#endif

#include <math.h>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include "shared/linkdefines.h"
#include "shared/misc.h"
#include "shared/net.h"

#ifdef _WIN32
void InitSockets() {
	WSADATA info;
	WSAStartup(MAKEWORD(2,0),&info);
}
void EndSockets() {
	WSACleanup();
}
#else

#define SOCKET_ERROR -1
#define SOCKET int

void InitSockets() {
	if(signal(SIGPIPE,SIG_IGN)==SIG_ERR) {
		FATAL("InitSockets failed");
	}
}
void EndSockets() {}
#endif

SocketAddress GetHostByName(const char* pszHostName) {
	SocketAddress address;
	struct hostent* host=::gethostbyname(pszHostName);
	if(host) {
		struct sockaddr_in sockaddr;
		for(int a=0;host->h_addr_list[a];a++) {
			memcpy(&sockaddr.sin_addr,host->h_addr_list[a],host->h_length);
			address.Set(inet_ntoa(sockaddr.sin_addr));
			break;
		}
	}
	return address;
}

class SocketHandle {
	int m_refCount=0;
public:
#ifdef _WIN32
	void* m_event=0;
#endif
	SOCKET m_socket;
	SocketHandle(SOCKET lSocket) {
		m_socket=lSocket;
		m_refCount=1;
	}
	~SocketHandle() {
		if(m_socket) {
#ifdef _WIN32
			//shutdown(m_socket,SD_SEND);
			if(closesocket(m_socket))
				FATAL("closesocket failed");
#else
		close(m_socket);
		//	::shutdown(m_socket,SHUT_RDWR);
//#ifndef WSL
//			::close(m_socket);
//#endif
#endif
		}
	}
	void AddRef() {
		m_refCount++;
	}
	void Release() {
		m_refCount--;
		if(!m_refCount) {
			delete this;
		}
	}
	bool IsValid()const {
		return m_socket!=-1;
	}
};

Socket::~Socket() {
	if(m_handle) m_handle->Release();
}
Socket::Socket(AddressFamily af,SocketType st,ProtocolType pt) {
	int domain;
	switch(af) {
		case INET:
			domain=PF_INET;
			break;
		case INET6:
			domain=PF_INET6;
			break;
		default:
			return;
	}
	int type;
	switch(st) {
		case STREAM:
			type=SOCK_STREAM;
			if(pt!=TCP)return;
			break;
		case DGRAM:
			type=SOCK_DGRAM;
			if(pt!=UDP)return;
			break;
		default:
			return;
	}
	SOCKET fd=socket(domain,type,0);
	if(fd==-1)return;
	m_handle=new SocketHandle(fd);
	m_family=af;
	m_socketType=st;
	m_protocolType=pt;
}
Socket::Socket(SocketHandle* sh) {
	m_handle=sh;
}
Socket::Socket(const Socket& tocopy) {
	m_handle=tocopy.m_handle;
	m_family=tocopy.m_family;
	m_protocolType=tocopy.m_protocolType;
	m_socketType=tocopy.m_socketType;
	if(m_handle) m_handle->AddRef();
}
Socket& Socket::operator=(const Socket& assign) {
	SocketHandle* sh=m_handle;
	m_handle=assign.m_handle;
	m_family=assign.m_family;
	m_protocolType=assign.m_protocolType;
	m_socketType=assign.m_socketType;
	if(m_handle) m_handle->AddRef();
	if(sh) sh->Release();
	return *this;
}
const char* Socket::ErrorCodeToName(Socket::Error error){
	switch(error) {
		case ER_WOULDBLOCK:
			return "WOULDBLOCK";
		case ER_TIMEOUT:
			return "TIMEOUT";
		case ER_AGAIN:
			return "AGAIN";
		case ER_RESET:
			return "RESET";
		case ER_NA:
			return "NA";
	}
	return "UNDEFINED ERROR CODE";
}

#ifdef _WIN32
Socket::Error Socket::GetLastError(){
	int errnoWin=WSAGetLastError();
	switch(errnoWin) {
		case WSAEWOULDBLOCK:
			return ER_WOULDBLOCK;
		case WSAETIMEDOUT:
			return ER_TIMEOUT;
		case WSAECONNRESET:
			return ER_RESET;
	}
	uprintf("Socket::GetLastError WSAGetLastError()=%d\n",errnoWin);
	return ER_NA;
}
#else
Socket::Error Socket::GetLastError() {
	switch(errno) {
		case EINTR:
			return ER_TIMEOUT;
		case EAGAIN:
			return ER_AGAIN;
	}
	uprintf("Socket::GetLastError errno=%d\n",errno);
	return ER_NA;
}
#endif

#ifdef _WIN32
SocketAddress Socket::GetPeerName() {
	SocketAddress s;
	int namelen=sizeof(sockaddr);
	int lres=getpeername(m_handle->m_socket,&s.m_handle,&namelen);
	if(!lres)return s;
	return SocketAddress();
}
#else
SocketAddress Socket::GetPeerName() {
	SocketAddress s;
	int namelen=sizeof(sockaddr);
	int lres=::getpeername(m_handle->m_socket,&s.m_handle,(socklen_t*)&namelen);
	if(!lres)return s;
	return SocketAddress();
}
#endif
bool Socket::IsValid()const {
	return m_handle && m_handle->IsValid();
}
#ifdef _WIN32
bool Socket::OptionNoError()const {
	int opt=0;
	int len=sizeof(opt);
	const int ret=getsockopt(m_handle->m_socket,SOL_SOCKET,SO_ERROR,(char*)&opt,&len);
	return (ret==0) && (opt==0);
}
bool Socket::HasPeerName()const {
	SocketAddress s;
	int namelen=sizeof(sockaddr);
	return getpeername(m_handle->m_socket,&s.m_handle,&namelen)==0;
}
bool Socket::EnableTimestampRX() {
	return false;
}
int Socket::SetSendTimeout(int milliseconds) {
	return setsockopt(m_handle->m_socket,SOL_SOCKET,SO_SNDTIMEO,(char*)&milliseconds,sizeof(int));
}
int Socket::SetRecvTimeout(int milliseconds) {
	return setsockopt(m_handle->m_socket,SOL_SOCKET,SO_RCVTIMEO,(char*)&milliseconds,sizeof(int));
}

#else
bool Socket::OptionNoError() const {
	int opt=0;
	unsigned int len=sizeof(opt);
	const int ret=getsockopt(m_handle->m_socket, SOL_SOCKET, SO_ERROR, &opt, &len);
	return (ret==0) && (opt==0);
}
bool Socket::HasPeerName() const {
	struct sockaddr_in addr;
	socklen_t addr_len=sizeof(addr);
	return getpeername(m_handle->m_socket, (struct sockaddr*)&addr, &addr_len)==0;
}
bool Socket::EnableTimestampRX() {
#if __linux__
	int flags=SOF_TIMESTAMPING_SOFTWARE|SOF_TIMESTAMPING_RX_SOFTWARE|SOF_TIMESTAMPING_RX_HARDWARE;
	int err=setsockopt(m_handle->m_socket, SOL_SOCKET, SO_TIMESTAMPING, &flags, sizeof(flags));
	return err==0;
#else
	return false;
#endif
}
int Socket::SetSendTimeout(int milliseconds) {
	timeval timeout;
	timeout.tv_sec = milliseconds / 1000;
	timeout.tv_usec = (milliseconds % 1000)*1000;
	return setsockopt(m_handle->m_socket,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
}
int Socket::SetRecvTimeout(int milliseconds) {
	timeval timeout;
	timeout.tv_sec = milliseconds / 1000;
	timeout.tv_usec = (milliseconds % 1000)*1000;
	return setsockopt(m_handle->m_socket,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
}
#endif

int Socket::SetBuffered(bool buffered) {
	int flag=buffered ? 0:1;
	return setsockopt(m_handle->m_socket,IPPROTO_TCP,TCP_NODELAY,(char*)&flag,sizeof(int));
}
bool Socket::Listen(int backLog) {
	return listen(m_handle->m_socket,backLog)!=SOCKET_ERROR;
}
bool Socket::Connect(const SocketAddress& address) {
	return connect(m_handle->m_socket,&address.m_handle,sizeof(sockaddr))!=SOCKET_ERROR;
}
Socket Socket::Accept() {
	SOCKET socket=::accept(m_handle->m_socket,0,0);
	if(socket==INVALID_SOCKET) return Socket();
	return Socket(new SocketHandle(socket));
}
#ifdef _WIN32
bool Socket::ConnectNonBlock(const SocketAddress& address) {
	SetBlockingMode(false);
	const int ret=connect(m_handle->m_socket,&address.m_handle,sizeof(sockaddr));
	if(ret==SOCKET_ERROR) {
		return (WSAGetLastError()==WSAEWOULDBLOCK);
	}
	return true;
}
#else
bool Socket::ConnectNonBlock(const SocketAddress& Address) {
	SetBlockingMode(false);
	int ret=::connect(m_handle->m_socket,&Address.m_handle,sizeof(sockaddr));
	if(ret==SOCKET_ERROR) {
		if(errno==EINPROGRESS) {
			return true;
        }
		return false;
	}
	return true;
}
#endif




#ifdef _WIN32
void Socket::SetBlockingMode(bool blocking)const {
	u_long mode=blocking ? 0:1;
	ioctlsocket(m_handle->m_socket,FIONBIO,&mode);
}
#else
void Socket::SetBlockingMode(bool blocking)const {
	int socket=m_handle->m_socket;
    const int flags=fcntl(socket,F_GETFL,0);
    fcntl(socket,F_SETFL,blocking ? flags&(~O_NONBLOCK):flags|O_NONBLOCK);
}
#endif

#ifdef _WIN32
bool Socket::Bind(const SocketAddress& Address) {
	return bind(m_handle->m_socket,&Address.m_handle,sizeof(sockaddr))!=SOCKET_ERROR;
}
#else
bool Socket::Bind(const SocketAddress& address) {
	//int optval=0;
	//setsockopt(m_handle->m_socket,SOL_SOCKET,SO_REUSEADDR,&optval,sizeof optval); //allow reuse of unconnected address,this is something else altogether on win32
	int ret=::bind(m_handle->m_socket,&address.m_handle,sizeof(sockaddr));
	return ret!=SOCKET_ERROR;
}
#endif


#ifdef _WIN32
bool IsWireless(const char* ifname) {
	FATAL("net.cpp: IsWireless not implemented for windows");
	return false;
}
#endif
#ifdef __linux__
bool IsWireless(const char* ifname) {
	struct iwreq pwrq;
	memset(&pwrq,0,sizeof(pwrq));
	int n=strnlen(ifname,IFNAMSIZ);
	if(n>=IFNAMSIZ) {
		return false;
	}
	memcpy(pwrq.ifr_name,ifname,n);
	pwrq.ifr_name[n]=0;

	int sock=-1;
	if((sock=socket(AF_INET,SOCK_STREAM,0))==-1) {
		perror("socket");
		return false;
	}
	if(ioctl(sock,SIOCGIWNAME,&pwrq)!=-1) {
		close(sock);
		return true;
	}
	close(sock);
	return false;
}
#endif

#ifdef __linux__
bool GetRSSI(int* rssi,const char* interfaceName) {
    if(!IsWireless(interfaceName)) return false;
    iw_statistics m_stats;
    iwreq m_req;
    m_req.u.data.pointer = &m_stats;
    m_req.u.data.length = sizeof(m_stats);
    m_req.u.data.flags = 1;
    strcpy(m_req.ifr_name, interfaceName);

    //have to use a socket for ioctl
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    //this will gather the signal strength
    if(ioctl(sockfd, SIOCGIWSTATS, &m_req) == -1){
        //die with error, invalid interface
        //printf("WiFiInfo::GetInfo Invalid interface.\n");
		close(sockfd);
        return false;
    }
    else if(((iw_statistics *)m_req.u.data.pointer)->qual.updated & IW_QUAL_DBM){
        //signal is measured in dBm and is valid for us to use
        *rssi = ((iw_statistics *)m_req.u.data.pointer)->qual.level - 256;
		close(sockfd);
		return true;
    }
	close(sockfd);
	return false;
#if 0
    //SIOCGIWESSID for ssid
    char buffer[32] = {0};
    m_req.u.essid.pointer = buffer;
    m_req.u.essid.length = 32;
    //this will gather the SSID of the connected network
    if(ioctl(sockfd, SIOCGIWESSID, &m_req) == -1){
        //die with error, invalid interface
		close(sockfd);
        return 0;
    }
    else{
        memcpy(&m_sigInfo.ssid, m_req.u.essid.pointer, m_req.u.essid.length);
        memset(&m_sigInfo.ssid[m_req.u.essid.length],0,1);
    }

    //SIOCGIWRATE for bits/sec (convert to mbit)
    int bitrate=-1;
    //this will get the bitrate of the link
    if(ioctl(sockfd, SIOCGIWRATE, &m_req) == -1){
        printf("WiFiInfo::GetInfo bitratefail\n");
		close(sockfd);
        return 0;
    }else{
        memcpy(&bitrate, &m_req.u.bitrate, sizeof(int));
        m_sigInfo.bitrate=bitrate/1000000;
    }

    //SIOCGIFHWADDR for mac addr
    ifreq req2;
    strcpy(req2.ifr_name, m_iwname.c_str());
    //this will get the mac address of the interface
    if(ioctl(sockfd, SIOCGIFHWADDR, &req2) == -1){
        printf("WiFiInfo::GetInfo mac error\n");
		close(sockfd);
        return 0;
    }
    else{
        sprintf(m_sigInfo.mac, "%.2X", (unsigned char)req2.ifr_hwaddr.sa_data[0]);
        for(int s=1; s<6; s++){
            sprintf(m_sigInfo.mac+strlen(m_sigInfo.mac), ":%.2X", (unsigned char)req2.ifr_hwaddr.sa_data[s]);
        }
    }
    close(sockfd);
    return &m_sigInfo;
#endif
}
#else
bool GetRSSI(int* rssi,const char* interfaceName) {
	return false;
}
#endif

#ifdef _WIN32
int Socket::SendTo(const SocketAddress& Destination,const void* pBuffer,int nLength,int nFlags)const {
	int lResult=sendto(m_handle->m_socket,(const char*)pBuffer,nLength,0,&Destination.m_handle,sizeof(sockaddr));
	if(lResult==SOCKET_ERROR && WSAGetLastError()==WSAENOBUFS) {
		return -2345;
	}
	return lResult;
}
int Socket::ReceiveFrom(void* pBuffer,int nLength,SocketAddress* source,int nFlags)const {
	int nNativeFlags=0;
	if(nFlags&PEEK) nNativeFlags|=MSG_PEEK;
	int nSize=sizeof(sockaddr);
	return recvfrom(m_handle->m_socket,(char*)pBuffer,nLength,nNativeFlags,&source->m_handle,&nSize);
}
int Socket::ReceiveFromTime(void* pBuffer,int nLength,SocketAddress* source,uint64_t* time_us, int nFlags)const {
	int nNativeFlags=0;
	if(nFlags&PEEK) nNativeFlags|=MSG_PEEK;
	int nSize=sizeof(sockaddr);
	int n=recvfrom(m_handle->m_socket,(char*)pBuffer,nLength,nNativeFlags,&source->m_handle,&nSize);
	*time_us=GetTimeEpochMicroseconds();
	return n;
}
#else
int Socket::SendTo(const SocketAddress& destination,const void* buffer,int length,int flags)const {
	int result=sendto(m_handle->m_socket,(const char*)buffer,(socklen_t)length,0,&destination.m_handle,sizeof(sockaddr));
	if(result==SOCKET_ERROR) {
		return -2345;
	}
	return result;
}
int Socket::ReceiveFrom(void* buffer,int length,SocketAddress* source,int flags)const {
	int nativeFlags=0;
	if(flags&PEEK) nativeFlags|=MSG_PEEK;
	int sz=sizeof(sockaddr);
	return ::recvfrom(m_handle->m_socket,(char*)buffer,length,nativeFlags,&source->m_handle,(socklen_t*)&sz);
}
int Socket::ReceiveFromTime(void* buffer,int length,SocketAddress* source,uint64_t* timeEpochMicroseconds,int flags)const {
#if __linux__
	int nativeFlags=0;
	if(flags&PEEK) nativeFlags|=MSG_PEEK;
	iovec iov={buffer,(size_t)length};
	union {
		char buffer[CMSG_SPACE(32)];
		struct cmsghdr align;
	} ctrl_data;
	msghdr msg{0};
	msg.msg_name=&source->m_handle;
	msg.msg_namelen=sizeof(source->m_handle);
	msg.msg_iov=&iov;
	msg.msg_iovlen=1;
	msg.msg_control=&ctrl_data;
	msg.msg_controllen=sizeof(ctrl_data);
	msg.msg_flags=nativeFlags;

	ssize_t res=::recvmsg(m_handle->m_socket,&msg,nativeFlags);
	uint64_t socketTime=GetTimeEpochMicroseconds();
	bool hasCMSG=false;
	for (cmsghdr* cmsg=CMSG_FIRSTHDR(&msg);cmsg;cmsg=CMSG_NXTHDR(&msg,cmsg)) {
		if (cmsg->cmsg_level==SOL_SOCKET && cmsg->cmsg_type==SO_TIMESTAMPING) {
			const uint8_t* cmsg_data=CMSG_DATA(cmsg);
			uint64_t tv_sec,tv_nsec;
			int off=0;
			while(off + sizeof(tv_sec) + sizeof(tv_nsec) < cmsg->cmsg_len) {
				memcpy(&tv_sec,cmsg_data+off,sizeof(tv_sec));
				off+=sizeof(tv_sec);
				memcpy(&tv_nsec,cmsg_data+off,sizeof(tv_nsec));
				off+=sizeof(tv_nsec);
				if(tv_sec) {
					hasCMSG=true;
					socketTime=tv_sec*1000000 + tv_nsec/1000;
					break;
				}
			}
		}
	}
	if(!hasCMSG) {
		uprintf("Warning: No valid timestamp on received message\n");
	}
	*timeEpochMicroseconds=socketTime;
	return (int)res;
#else
	int nativeFlags=0;
	if(flags&PEEK) nativeFlags|=MSG_PEEK;
	socklen_t size=(socklen_t)sizeof(sockaddr);
	int n=::recvfrom(m_handle->m_socket,(char*)buffer,length,nativeFlags,&source->m_handle,&size);
	*timeEpochMicroseconds=GetTimeEpochMicroseconds();
	return n;
#endif
}
#endif

//STREAM
int Socket::Send(const void* buffer,int length,int flags)const {
	return send(m_handle->m_socket,(const char*)buffer,length,0);
}

//SocketAddress
SocketAddress::SocketAddress() {
	memset(&m_handle,0,sizeof(m_handle));
	m_handle.sa_family=0xff;
}
SocketAddress::SocketAddress(uint8_t v0,uint8_t v1,uint8_t v2,uint8_t v3,uint16_t port) {
	Set(v0,v1,v2,v3,port);
}
void SocketAddress::Set(uint8_t v0,uint8_t v1,uint8_t v2,uint8_t v3,uint16_t port) {
	m_handle.sa_family=AF_INET;
	sockaddr_in* sa=(sockaddr_in*)&m_handle;
	sa->sin_port=htons(port);
	uint8_t* p=(uint8_t*)&sa->sin_addr;
	p[0]=v0;
	p[1]=v1;
	p[2]=v2;
	p[3]=v3;
}
void SocketAddress::Set(const char* address) {
	int v0,v1,v2,v3,lPort;
	int lNumberArgs=std::sscanf(address,"%d.%d.%d.%d:%d",&v0,&v1,&v2,&v3,&lPort);
	if(lNumberArgs<4) {
		m_handle.sa_family=0xff;
		return;
	}
	if(lNumberArgs==4) {
		lPort=Port();
	}
	Set(v0,v1,v2,v3,lPort);
}
uint8_t* SocketAddress::GetIPBytes()const {
	sockaddr_in* sa=(sockaddr_in*)&m_handle;
	return (uint8_t*)&sa->sin_addr;
}
std::string SocketAddress::ToString()const {
	sockaddr_in* sa=(sockaddr_in*)&m_handle;
	if(!sa->sin_port)
		return "0.0.0.0:0";
	uint8_t* p=GetIPBytes();
	return stdx::format_string("%d.%d.%d.%d:%d",p[0],p[1],p[2],p[3],ntohs(sa->sin_port));
}
bool SocketAddress::operator==(const SocketAddress& cmp)const {
	if(Port()!=cmp.Port())return false;
	uint8_t* pb1=GetIPBytes();
	uint8_t* pb2=cmp.GetIPBytes();
	for(int a=0;a!=4;a++) {
		if(pb1[a]!=pb2[a])return false;
	}
	return true;
}
bool SocketAddress::operator!=(const SocketAddress& cmp)const {
	return !(cmp==*this);
}
bool SocketAddress::IsValid()const {
	return m_handle.sa_family==AF_INET;
}
uint16_t SocketAddress::Port()const {
	sockaddr_in* sa=(sockaddr_in*)&m_handle;
	return ntohs(sa->sin_port);
}
void SocketAddress::PortSet(uint16_t port) {
	sockaddr_in* sa=(sockaddr_in*)&m_handle;
	sa->sin_port=htons(port);
}
void SocketAddress::SetAny() {
	sockaddr_in* sa=(sockaddr_in*)&m_handle;
	sa->sin_family=AF_INET;
	sa->sin_addr.s_addr=INADDR_ANY;
}
bool SocketAddress::IsAny()const {
	uint8_t* p=GetIPBytes();
	return (p[0]|p[1]|p[2]|p[3])==0;
}



BlockDataBuffer::~BlockDataBuffer() {
	Reset();
}
bool BlockDataBuffer::CmpBytes(const Data* link,int offset,const char* data,int dataByteSize)const {
	uint32_t remain=dataByteSize;
	uint32_t offsetCur=offset;
	while(link) {
		const char* p=(char*)link;
		uint32_t copyByteSize=MIN(remain,link->m_byteSize-(link->m_pos+offsetCur));
		if(memcmp(data,p+sizeof(Data)+link->m_pos+offsetCur,copyByteSize))
			return false;
		offsetCur=0;
		data+=copyByteSize;
		remain-=copyByteSize;
		link=link->m_next;
	}
	return true;
}


int32_t BlockDataBuffer::Find(const void* data,int32_t dataByteSize,int32_t offset)const {
	if(DataByteSize()<offset+dataByteSize) {
		return 0;
	}
	int32_t offsetCur=offset;
	const Data* link=m_first;
	while(offsetCur>link->m_byteSize-link->m_pos) {
		offsetCur-=link->m_byteSize-link->m_pos;
		link=link->m_next;
	}
	int32_t offsetTotal=offset;
	while(link) {
		for(int i=0;i!=link->m_byteSize-link->m_pos-offsetCur;i++) {
			if(CmpBytes(link,offsetCur+i,(const char*)data,dataByteSize)) {
				return offsetTotal+i;
			}
		}
		offsetTotal+=link->m_byteSize-link->m_pos-offsetCur;
		offsetCur=0;
		link=link->m_next;
	}
	return -1;
}

int32_t BlockDataBuffer::Find(const char* data,int32_t offset)const {
	return Find(data,(uint32_t)strlen((char*)data),offset);
}
void BlockDataBuffer::Push(const char* data) {
	Push(data,(int32_t)strlen((const char*)data));
}
void BlockDataBuffer::Push(const BlockDataBuffer* data) {
	Data* link=data->m_first;
	while(link) {
		const char* p=(char*)link;
		Push(p+sizeof(Data)+link->m_pos,link->m_byteSize-link->m_pos);
		link=link->m_next;
	}
}
void BlockDataBuffer::Push(const void* data,int32_t dataByteSize) {
	char* p=new char[sizeof(Data)+dataByteSize];
	Data* link=(Data*)p;
	memset(link,0,sizeof(Data));
	link->m_byteSize=dataByteSize;
	memcpy(p+sizeof(Data),data,dataByteSize);
	LINKINSERTENDCUSTOM(m_prev,m_next,link,m_first,m_last);
}
void BlockDataBuffer::Reset() {
	while(m_first) {
		char* first=(char*)m_first;
		m_first=m_first->m_next;
		delete [] first;
	}
	m_last=0;
}
int32_t BlockDataBuffer::DataByteSize()const {
	Data* link=m_first;
	int32_t sz=0;
	while(link) {
		sz+=link->m_byteSize-link->m_pos;
		link=link->m_next;
	}
	return sz;
}
int32_t BlockDataBuffer::Pop(void* data,int32_t dataByteSize) {
	if(ReadBytes(data,dataByteSize,0)!=dataByteSize)
		return 0;
	return PopBytes(dataByteSize);
}
int32_t BlockDataBuffer::PopBytes(int32_t dataByteSize) {
	if(DataByteSize()<dataByteSize) {
		return 0;
	}
	Data* link=m_first;
	int32_t remain=dataByteSize;
	while(link) {
		const char* p=(char*)link;
		int32_t copyByteSize=MIN(remain,link->m_byteSize-link->m_pos);
		link->m_pos+=copyByteSize;
		remain-=copyByteSize;
		if(link->m_pos!=link->m_byteSize) {
			link=link->m_next;
			continue;
		}
		Data* link1=link;
		link=link->m_next;
		LINKREMOVECUSTOM(m_prev,m_next,link1,m_first,m_last);
		delete [] p;
	}
	return dataByteSize;
}
int32_t BlockDataBuffer::ReadBytes(void* data,int32_t dataByteSize,int32_t offset)const {
	if(DataByteSize()<offset+dataByteSize) {
		return 0;
	}
	int32_t offsetCur=offset;
	const Data* link=m_first;
	while(offsetCur>link->m_byteSize-link->m_pos) {
		offsetCur-=link->m_byteSize-link->m_pos;
		link=link->m_next;
	}
	char* dst=(char*)data;
	int32_t remain=dataByteSize;
	while(remain) {
		const char* p=(char*)link;
		uint32_t copyByteSize=MIN(remain,link->m_byteSize-(link->m_pos+offsetCur));
		memcpy(dst,p+sizeof(Data)+link->m_pos+offsetCur,copyByteSize);
		offsetCur=0;
		dst+=copyByteSize;
		remain-=copyByteSize;
		link=link->m_next;
	}
	return dataByteSize;
}


struct internal_fd_set : fd_set {
};

#ifdef _WIN32
SocketSet::SocketSet()
{
	write_fds=new internal_fd_set;
	read_fds=new internal_fd_set;
	error_fds=new internal_fd_set;

	m_SignalSocket=Socket(Socket::INET,Socket::DGRAM,Socket::UDP);

	FD_ZERO(read_fds);
	FD_ZERO(write_fds);
	FD_ZERO(error_fds);

}
#else
#endif
SocketSet::~SocketSet() {
	delete write_fds;
	delete read_fds;
	delete error_fds;
}

#ifdef _WIN32
void SocketSet::SignalSelect() {
	uint8_t buf[1]={0xff};
	SocketAddress Destination;
	Destination.Set("127.0.0.1");
	m_SignalSocket.SendTo(Destination,buf,1);
}
void SocketSet::SetReadSockets(const Socket** pSocketList,uint32_t Count) {
	if(Count>=FD_SETSIZE)FATAL("INT3");
	//assert(Count<FD_SETSIZE);
	FD_ZERO(read_fds);
	uint32_t i=0;
	for(;i<Count;i++) {
		read_fds->fd_array[i]=pSocketList[i]->m_handle->m_socket;
	}
	if(m_SignalSocket.IsValid()) {
		read_fds->fd_array[i]=m_SignalSocket.m_handle->m_socket;
		read_fds->fd_count=Count+1;
	} else {
		read_fds->fd_count=Count;
	}
}
void SocketSet::SetWriteSockets(const Socket** pSocketList,uint32_t Count) {
	//assert(Count<FD_SETSIZE);
	FD_ZERO(write_fds);
	if(!Count)
		return;
	for(uint32_t i=0;i<Count;i++) {
		write_fds->fd_array[i]=pSocketList[i]->m_handle->m_socket;
	}
	write_fds->fd_count=Count;
}
void SocketSet::SetErrorSockets(const Socket** pSocketList,uint32_t Count) {
	//assert(Count<FD_SETSIZE);
	FD_ZERO(error_fds);
	for(uint32_t i=0;i<Count;i++) {
		error_fds->fd_array[i]=pSocketList[i]->m_handle->m_socket;
	}
	error_fds->fd_count=Count;
}
int SocketSet::WaitForDataOrSignal(int milliseconds) {
	int ret;
	if(milliseconds==-1) {
		ret=select(0,read_fds,write_fds,error_fds,NULL);
	}else{
		timeval timeout;
		timeout.tv_sec=milliseconds/1000;
		timeout.tv_usec=(milliseconds%1000)*1000;
		ret=select(0,read_fds,write_fds,error_fds,&timeout);
	}
	if(ret==SOCKET_ERROR) {
		//FATAL("INT3");
		return -1;
		//return WSAGetLastError();
	}
	if(IsRead(m_SignalSocket)) {
		uint8_t flush;
		m_SignalSocket.Receive(&flush,1);
		ret--;
	}
	return ret;
}
#else
SocketSet::SocketSet()
{
	write_fds=new internal_fd_set;
	read_fds=new internal_fd_set;
	error_fds=new internal_fd_set;
	m_lMaxSocketRead=0;
	m_lMaxSocketWrite=0;
	m_lMarSocketError=0;

	FD_ZERO(read_fds);
	FD_ZERO(write_fds);
	FD_ZERO(error_fds);
	if(pipe(m_lSignalPipe)==-1) {
		uprintf("SocketSet::SocketSet pipe failed errno %x",errno);
		FATAL("INT3");
	}
}
void SocketSet::SignalSelect() {
	if(write(m_lSignalPipe[1],"x",1)==-1) {
		uprintf("SocketSet::SignalSelect write failed errno %x",errno);
		FATAL("INT3");
	}
}

int g_readCount=0;
int g_writeCount=0;
int g_errorCount=0;

void SocketSet::SetReadSockets(const Socket** pSocketList,uint32_t Count)
{
	assert(Count<FD_SETSIZE);
	FD_ZERO(read_fds);

	g_readCount=Count;

	m_lMaxSocketRead=0;
	for(uint32_t i=0;i<Count;i++) {
		m_lMaxSocketRead=MAX(m_lMaxSocketRead,pSocketList[i]->m_handle->m_socket);
		//if(pSocketList[i]->m_handle->m_socket>m_lMaxSocket) {
		//	m_lMaxSocket=pSocketList[i]->m_handle->m_socket;
		//}
		FD_SET(pSocketList[i]->m_handle->m_socket,read_fds);
	}
	m_lMaxSocketRead=MAX(m_lMaxSocketRead,m_lSignalPipe[0]);
	FD_SET(m_lSignalPipe[0],read_fds);
}
void SocketSet::SetWriteSockets(const Socket** pSocketList,uint32_t Count)
{
	g_writeCount=Count;

	assert(Count<FD_SETSIZE);
	FD_ZERO(write_fds);
	m_lMaxSocketWrite=0;
	for(uint32_t i=0;i<Count;i++) {
		m_lMaxSocketWrite=MAX(m_lMaxSocketWrite, pSocketList[i]->m_handle->m_socket);
		FD_SET(pSocketList[i]->m_handle->m_socket,write_fds);
	}
}
void SocketSet::SetErrorSockets(const Socket** pSocketList,uint32_t Count)
{
	g_errorCount=Count;

	assert(Count<FD_SETSIZE);
	FD_ZERO(error_fds);
	m_lMarSocketError=0;
	for(uint32_t i=0;i<Count;i++) {
		m_lMarSocketError=MAX(m_lMarSocketError, pSocketList[i]->m_handle->m_socket);
		FD_SET(pSocketList[i]->m_handle->m_socket,error_fds);
	}
}
int SocketSet::WaitForDataOrSignal(int milliseconds) {
	int ret;
	int maxSocket=MAX(m_lMaxSocketRead, m_lMaxSocketWrite);
	maxSocket=MAX(maxSocket, m_lMarSocketError);
	if(milliseconds==-1) {
		ret=select(maxSocket+1,read_fds,write_fds,error_fds,NULL);
	}else{
		timeval timeout;
		timeout.tv_sec=(milliseconds/1000);
		timeout.tv_usec=(milliseconds%1000)*1000;
		ret=select(maxSocket+1,read_fds,write_fds,error_fds,&timeout);
	}
	if(ret==0) {
		//uprintf("Timeout %d,%d,%d",g_readCount,g_writeCount,g_errorCount);
		return 0;
	}
	if(FD_ISSET(m_lSignalPipe[0],read_fds)) {
		char ch;
		ret--;
		if(read(m_lSignalPipe[0],&ch,1)==-1) {
			if(errno!=EAGAIN) {
				FATAL("INT3");
			}
//			uprintf("signal consume");
		}
	}
	return ret;
}
#endif

int SocketSet::WaitForData(int milliseconds)
{
	if(milliseconds==-1) {
		return select(0,read_fds,write_fds,error_fds,NULL);
	}
	timeval timeout;
	timeout.tv_sec=milliseconds/1000;
	timeout.tv_usec=(milliseconds%1000)*1000;
	return select(0,read_fds,write_fds,error_fds,&timeout);
}
int SocketSet::IsRead(const Socket& Socket)
{
	if(!Socket.IsValid()) return false;
	return FD_ISSET(Socket.m_handle->m_socket, read_fds);
}
int SocketSet::IsWrite(const Socket& Socket)
{
	return FD_ISSET(Socket.m_handle->m_socket, write_fds);
}
int SocketSet::IsError(const Socket& Socket)
{
	return FD_ISSET(Socket.m_handle->m_socket, error_fds);
}



#define MAX_NUMBER_CONNECTIONS 1024

#define SOCKETSERVERERROR uprintf
#define SOCKETSERVERWARNING uprintf
#define SOCKETSERVERNOTIFY(...)

#define SOCKETCLIENTERROR uprintf
#define SOCKETCLIENTWARNING uprintf
#define SOCKETCLIENTNOTIFY(...)

#define TIMEGUARD(function,limit,msg) {uint64_t t=GetTimer();\
						function;\
						uint64_t dt=ElapsedMilliseconds(t);\
						if(dt>limit) {\
							uprintf("%s execution time %lluns\n",msg,dt);\
						}}\

//Socket Server
void SocketServer::SetEvent() {
	m_eventLock.lock();
	m_eventIndex++;
	m_eventLock.unlock();
	m_socketSet.SignalSelect();
}

SocketServer::SocketServer() {
	m_totalBytesSend=0;
	m_totalBytesReceived=0;
	m_bindAddress="0.0.0.0";
	m_bindPort=8889;
	m_socketIdCount=0x1000;
	m_close=false;
	m_threadRunning=false;
	m_eventIndex=0;
	m_verbose=true;
	m_numConnectedSockets=0;
}
SocketServer::~SocketServer() {
	if(m_threadRunning)		//End not called?
		FATAL("INT3");
}
int SocketServer::TimeoutSockets(const ConnectedSocket** sockets,int socketsCount) {
	return 0;
}
void SocketServer::Start() {
	m_listenThread=std::thread([&](){Run();});
	while(!m_threadRunning) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void SocketServer::End() {
	m_close=true;
	SetEvent();
	m_listenThread.join();
	//while(m_threadRunning) {
	//	std::this_thread::sleep_for(std::chrono::milliseconds(10));
		//Thread::Sleep(10);
	//}
}

void SocketServer::SetBindAddress(const char* bindAddress,uint16_t bindPort) {
	m_bindAddress=bindAddress;
	m_bindPort=bindPort;
}


void SocketServer::SendToSocket(int socketId,const char* data,int dataByteSize) {
	m_sendLock.lock();
	m_sendList.emplace_back(socketId,data,dataByteSize);
	m_sendTime=GetTimer();
	m_sendLock.unlock();
	m_socketSet.SignalSelect();
}

void SocketServer::SendToSocket(int socketId,const void** dataEntries,const int* dataByteSizes,const int numEntries) {
	m_sendLock.lock();
	m_sendList.emplace_back(socketId,dataEntries,dataByteSizes,numEntries);
	m_sendTime=GetTimer();
	m_sendLock.unlock();
	m_socketSet.SignalSelect();
}

void SocketServer::EncodeAndSend(ConnectedSocket* socket,const void* data,int byteSize)const {
	socket->m_send.Push(data,byteSize);
}


//SocketServer
int SocketServer::Run() {
	//InitRandom();
	Socket socketMaster=Socket(Socket::INET,Socket::STREAM,Socket::TCP);
	SocketAddress address;
	//address.SetAny();
	address.Set(m_bindAddress.c_str());
	address.PortSet(m_bindPort);

	socketMaster.SetBlockingMode(false);
	socketMaster.SetBuffered(false);
	socketMaster.SetSendTimeout(5000);
	socketMaster.SetRecvTimeout(5000);
	while(1) {
		if(socketMaster.Bind(address)) break;
		SOCKETSERVERWARNING("WARNING: %s::Run Unable to bind '%s'. Retrying in 5 seconds.\n",ClassName(),address.ToString().c_str());
		//Thread::Sleep(5000);
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		if(m_close) return 0;
	}
	if(!socketMaster.Listen(3)) {
		FATAL("INT3");
	}
	m_threadRunning=true;
	int numSockets=0;
	ConnectedSocket* sockets[MAX_NUMBER_CONNECTIONS];
	const Socket* writeSockets[MAX_NUMBER_CONNECTIONS];
	memset(sockets,0,sizeof(sockets));
	sockets[numSockets++]=new ConnectedSocket(socketMaster,m_socketIdCount++);
	//char data[0x10000];
	char data[0x10000];
	SOCKETSERVERNOTIFY("NOTIFY: %s::Run enter listen loop port %d\n",ClassName(),m_port);
	int event_index_read=0;
	while(!m_close) {
		m_socketSet.SetReadSockets((const Socket**)sockets,numSockets);
		m_socketSet.SetErrorSockets((const Socket**)sockets,numSockets);

		if(m_sendList.size()) {
			m_sendLock.lock();
			uint64_t dt=ElapsedMicroseconds(m_sendTime);
			for(int i=0;i!=(int)m_sendList.size();i++) {				//implementation scales poorly
				bool found=false;
				for(int j=1;j<numSockets;j++) {							//do not include master
					if(sockets[j]->m_id==m_sendList[i].m_socketId) {
						EncodeAndSend(sockets[j],m_sendList[i].m_data.data(),(int)m_sendList[i].m_data.size());
						found=true;
						break;
					}
				}
				if(!found) {
					SOCKETSERVERWARNING("WARNING: %s::Run Unable to find socket %d\n",ClassName(),m_sendList[i].m_socketId);
				}
			}
			m_sendList.clear();
			m_sendLock.unlock();
			if(dt>10*1000)
				SOCKETSERVERWARNING("WARNING: %s::Run SendToSocketTime %llu\n",ClassName(),dt);
		}

		int numWriteSockets=0;
		for(int i=1;i<numSockets;i++) {							//do not include master
			if(sockets[i]->m_send.DataByteSize()) {
				writeSockets[numWriteSockets++]=sockets[i];
			}
		}
		m_socketSet.SetWriteSockets(writeSockets,numWriteSockets);
		int numSocketsReady=m_socketSet.WaitForDataOrSignal(10000);
		if(m_close)
			break;

		//uprintf("Test %f %d",Time::GetTime(),numSocketsReady);
		m_eventLock.lock();
		if(event_index_read!=m_eventIndex) {
			event_index_read=m_eventIndex;
			TIMEGUARD(OnEvent(((ConnectedSocket**)sockets)+1,numSockets-1),10*1000,stdx::format_string("WARNING: %s::Run OnEvent\n",ClassName()).c_str());
			//OnEvent(((ConnectedSocket**)sockets)+1,numSockets-1);
		}
		m_eventLock.unlock();

		//uprintf("select end count %d",num_sockets_ready);
		if(numSocketsReady>0) {
			for(int i=0;i<numSockets;i++) {
				if(m_socketSet.IsError(*sockets[i])) {
					SOCKETSERVERERROR("ERROR: %s::Run socket %d\n",ClassName(),i);
				}
			}
			for(int i=1;i<numSockets;i++) {
				if(m_socketSet.IsWrite(*sockets[i])) {
					ConnectedSocket* socket=sockets[i];
					int maxSendSize=MIN(socket->m_send.DataByteSize(),(int)sizeof(data));
					int sendByteSize=socket->m_send.ReadBytes(data,maxSendSize,0);
					bool connectionLost=false;
					if(sendByteSize) {
						int wasSentByteSize=socket->Send(data,sendByteSize);
						if(m_verbose) {
							SOCKETSERVERNOTIFY("NOTIFY: %s::Run port %d send data %d remain %d\n",ClassName(),m_port,wasSentByteSize,maxSendSize);
						}
						if(wasSentByteSize>0) {
							socket->m_send.PopBytes(wasSentByteSize);
							m_totalBytesSend+=(int64_t)wasSentByteSize;
						} else {
							connectionLost=true;
						}
					}else{
						SOCKETSERVERWARNING("WARNING: %s::Run send data size zero for socket id %d\n",ClassName(),socket->m_id);
					}
					if(connectionLost || socket->m_send.DataByteSize()==0) {
						if(connectionLost || socket->m_closeAfterSend) {
							//uprintf("data end");
							TIMEGUARD(OnClose(socket),2*1000,stdx::format_string("WARNING: %s::Run OnClose\n",ClassName()).c_str());
							//OnClose(socket);
							sockets[i--]=sockets[--numSockets];
							sockets[numSockets]=0;		//Debug niceness only
							m_numConnectedSockets=numSockets-1;
							delete socket;
						}
					}
				}
			}
			//uprintf("read %d",num_sockets);
			for(int i=0;i<numSockets;i++) {
				if(m_socketSet.IsRead(*sockets[i])) {
					//uprintf("Read socket %d",i);
					if(i==0) {
						if(numSockets==countof(sockets)) {
							SOCKETSERVERWARNING("WARNING: %s::Run OnData unable to accept new connection sockets list full,max %d\n",ClassName(),numSockets);
						}else{
							Socket newSocket=socketMaster.Accept();
							if(newSocket.IsValid()) {
								newSocket.SetSendTimeout(5000);
								newSocket.SetRecvTimeout(5000);
								ConnectedSocket* socket=new ConnectedSocket(newSocket,m_socketIdCount++);
								socket->SetBuffered(false);
								sockets[numSockets++]=socket;
								SOCKETSERVERNOTIFY("NOTIFY: %s::Run connected socket %d\n",ClassName(),socket->m_id);
								if(!OnConnected(socket)) {
									sockets[--numSockets]=0;
									delete socket;
								}
								m_numConnectedSockets=numSockets-1;
							}
						}
					}else{
						ConnectedSocket* socket=sockets[i];
						int bytesReceived=socket->Receive(data,sizeof(data));
						bool close=false;
						if(bytesReceived<=0) {
							//uprintf("Remove socket %d closed by host %d",i,bytesReceived);
							close=true;
						}else{
							SOCKETSERVERNOTIFY("NOTIFY: %s::Run data socket %d bytes received %d\n",ClassName(),socket->m_id,bytesReceived);
						    socket->m_timeLastByteReceived=std::chrono::high_resolution_clock::now();//Time::GetTimeReal();
							//uprintf("socket data %p %d",socket,bytesReceived);
							m_totalBytesReceived+=(int64_t)bytesReceived;
							TIMEGUARD(close=!OnData(socket,data,bytesReceived),100*1000,stdx::format_string("WARNING: %s::Run OnData\n",ClassName()).c_str());
							//close=!OnData(socket,data,bytesReceived);
						}
						if(close) {
							SOCKETSERVERNOTIFY("NOTIFY: %s::Run close socket %d\n",ClassName(),socket->m_id);
							TIMEGUARD(OnClose(socket),2*1000,stdx::format_string("WARNING: %s::Run OnClose\n",ClassName()).c_str());
							//OnClose(socket);
							sockets[i--]=sockets[--numSockets];
							sockets[numSockets]=0;		//Debug niceness only
							delete socket;
							m_numConnectedSockets=numSockets-1;
						}
					}
				}
			}
		}else{
			//uprintf("ERROR %d",num_sockets_ready);
		}
	}
	//uprintf("server destroy sockets\n");
	for(int i=0;i<MAX_NUMBER_CONNECTIONS;++i) {
		if(sockets[i]) {
			if(i) {		//Skip master socket
				OnDestroy(sockets[i]);
			}
			delete sockets[i];
		}
	}
	m_numConnectedSockets=0;
	SOCKETSERVERNOTIFY("NOTIFY: %s::Run leave listen loop\n",ClassName());
	m_threadRunning=false;
	return 0;
}

//SocketClient
SocketClient::SocketClient() {
	m_connected=false;
	m_timerFrequency=0;
	m_socket=0;
	m_close=false;
	m_running=false;
	m_nonBlockConnect=false;
	m_host="";
	m_port=0;
}
SocketClient::~SocketClient() {
	if(m_socket || m_running)
		FATAL("ERROR: SocketClient destructor thread not closed before destructor is called");
}

void SocketClient::Begin(const char* host,int port,float timerFrequency) {
	m_host=host;
	m_port=port;
	m_running=true;
	m_timerFrequency=timerFrequency;

	m_socket=0;
	m_keepAliveThread=std::thread([&](){KeepAliveThreadFunc();});
	//while(!m_socket) {
	//	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	//}
}
void SocketClient::End() {
	m_close=true;
	m_socketSet.SignalSelect();
	//while(m_running.load()) {
	//	m_waitClose.notify_all();
	//	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	//}
	m_keepAliveThread.join();
	if(m_socket)
		FATAL("ERROR: SocketClient::End socket not deleted\n");
	m_close=false;
}
bool SocketClient::Send(const void* data,int dataByteSize) {
	if(!m_socket) {
		//uprintf("WARNING: SocketClient::Send called outside of thread,data not send\n");
		uprintf("WARNING: SocketClient::Send called but socket is not connected,data not send\n");
		return false;
	}
	m_socket->m_sendLock.lock();
	m_socket->m_send.Push(data,dataByteSize);
	m_socket->m_sendLock.unlock();
	m_socketSet.SignalSelect();
	return true;
}
int Socket::Receive(void* buffer,int length,int flags)const {
	int nativeFlags=0;
	if(flags&PEEK) nativeFlags|=MSG_PEEK;
#ifdef MSG_WAITALL
	if(flags&WAITALL) nativeFlags|=MSG_WAITALL;
#endif
	return recv(m_handle->m_socket,(char*)buffer,length,nativeFlags);
}

int SocketClient::KeepAliveThreadFunc() {
	//SOCKETCLIENTNOTIFY("NOTIFY: SocketClient::KeepAliveThreadFunc");
	m_socket=0;
	uint8_t data[64*1000];
	uint64_t startTime=GetTimer();
	uint64_t prevTime=-1;
	while(!m_close) {
		if(!m_connected) {
			if(m_host.length()) {
				m_address=GetHostByName(m_host.c_str());
				m_address.PortSet(m_port);
			}
			if(m_nonBlockConnect) {
				if(!m_socket) {
					Socket socket=Socket(Socket::INET,Socket::STREAM,Socket::TCP);
					socket.SetBuffered(false);
					socket.SetSendTimeout(5000);
					socket.SetRecvTimeout(5000);
					if(!socket.ConnectNonBlock(m_address)) {
						continue;
					}
					m_socket=new ConnectedSocket(socket);
					m_socketSet.SetWriteSockets((const Socket**)&m_socket,1);
					m_socketSet.SetReadSockets(nullptr,0); // For signal:
				}
				int ret=m_socketSet.WaitForDataOrSignal(5000);
				if(ret < 1) {
					if(ret==-1) { // Select error here:
						m_socketSet.SetWriteSockets(0,0);
					} else {
						if(m_host.length()) {
							SOCKETCLIENTWARNING("WARNING: %s::KeepAliveThreadFunc Unable to connect to \"%s:%d\" (%s). Retrying in 5 seconds\n",InstanceName(),m_host.c_str(),m_port,m_address.ToString().c_str());
						}
					}
					delete m_socket;
					m_socket=nullptr;
					continue; // Signal or time out here:
				}
				else {
					if(m_socketSet.IsWrite(*m_socket) && m_socket->OptionNoError() && m_socket->HasPeerName())
					{
						m_socketSet.SetWriteSockets(0,0); // Connected
						m_socket->SetBlockingMode(true);
					}
					else {
						m_socketSet.SetWriteSockets(0,0); // Error or no peer name.
						delete m_socket;
						m_socket=nullptr;
						continue;
					}
				}
			}
			else {
				if(!m_socket) {
					Socket socket=Socket(Socket::INET,Socket::STREAM,Socket::TCP);
					m_socket=new ConnectedSocket(socket);
					m_socket->SetBuffered(false);
					m_socket->SetSendTimeout(5000);
					m_socket->SetRecvTimeout(5000);
				}
				if(!m_socket->Connect(m_address)) {
					SOCKETCLIENTWARNING("WARNING: %s::KeepAliveThreadFunc Unable to connect to %s. Retrying in 5 seconds\n",InstanceName(),m_address.ToString().c_str());
					//m_waitClose.Wait(5000);
					FATAL("impl std cond");
					continue;
				}
			}
			SOCKETCLIENTNOTIFY("NOTIFY: %s::KeepAliveThreadFunc Connected to %s\n",InstanceName(),m_address.ToString().c_str());
			OnConnected();
			m_connected=true;
		}
		int waitNextTimer=-1;
		if(m_timerFrequency) {
			uint64_t time=GetTimer()-startTime;
			if(floorf((float)prevTime/m_timerFrequency)!=floorf((float)time/m_timerFrequency)) {
				OnTimer();
			}
			double timeNextTimer=(floorf((float)time/m_timerFrequency)+1.0f)*m_timerFrequency;
			waitNextTimer=(int)(timeNextTimer-time);
			prevTime=time;
		}
		m_socket->m_sendLock.lock();
		if(m_socket->m_send.DataByteSize()) {
			m_socketSet.SetWriteSockets((const Socket**)&m_socket,1);
		}else{
			m_socketSet.SetWriteSockets(0,0);
		}
		m_socket->m_sendLock.unlock();
		m_socketSet.SetReadSockets((const Socket**)&m_socket,1);
		m_socketSet.SetErrorSockets((const Socket**)&m_socket,1);
		int numSocketsReady;
		if(waitNextTimer==-1) {
			numSocketsReady=m_socketSet.WaitForDataOrSignal();
		}else{
			numSocketsReady=m_socketSet.WaitForDataOrSignal(waitNextTimer);
		}
		if(m_close)
			break;
		if(numSocketsReady>0) {
			if(m_socketSet.IsError(*m_socket)) {
				SOCKETCLIENTERROR("ERROR: %s::KeepAliveThreadFunc Socket error\n",InstanceName());
			}
			if(m_socketSet.IsWrite(*m_socket)) {
				m_socket->m_sendLock.lock();
				int maxSendSize=MIN(m_socket->m_send.DataByteSize(),(int)sizeof(data));
				int sendByteSize=m_socket->m_send.ReadBytes(data,maxSendSize,0);
				if(!sendByteSize)
					FATAL("sendByteSize=0");
				if(m_socket->m_send.IsStartOfData()) {
					OnSendInjection(data,sendByteSize);
				}
				int wasSentByteSize=m_socket->Send(data,sendByteSize);
				if(wasSentByteSize>0) {
					m_socket->m_send.PopBytes(wasSentByteSize);
					m_totalBytesSend+=(int64_t)wasSentByteSize;
				}
				m_socket->m_sendLock.unlock();
				SOCKETCLIENTNOTIFY("NOTIFY: %s::KeepAliveThreadFunc send data %d remain %d\n",InstanceName(),sendByteSize,m_socket->m_send.DataByteSize());
				if(wasSentByteSize<=0 || m_socket->m_send.DataByteSize()==0) {
					if(wasSentByteSize<=0|| m_socket->m_closeAfterSend) {
						SOCKETCLIENTNOTIFY("NOTIFY: %s::KeepAliveThreadFunc close after send\n",InstanceName());
						delete m_socket;
						m_socket=0;
						TIMEGUARD(OnClose(),20*1000,"WARNING: KeepAliveThreadFunc OnClose\n");
						m_connected=false;
						//Thread::Sleep(1000);
						std::this_thread::sleep_for(std::chrono::milliseconds(1000));
						continue;
					}
				}
			}
			if(m_socketSet.IsRead(*m_socket)) {
				int bytesReceived=m_socket->Receive(data,sizeof(data));
				bool close=false;
				if(bytesReceived<=0) {
					//uprintf("SocketClient socket closed by host. Receive returned %d",bytesReceived);
					close=true;
				}else{
					m_totalBytesReceived+=(int64_t)bytesReceived;
					TIMEGUARD(close=!OnData(data,bytesReceived),40*100,"WARNING: SocketClient::KeepAliveThreadFun OnData\n");
				}
				if(close) {
					SOCKETCLIENTNOTIFY("NOTIFY: %s::KeepAliveThreadFunc socket closed\n",InstanceName());
					delete m_socket;
					m_socket=0;
					TIMEGUARD(OnClose(),20*1000,"WARNING: KeepAliveThreadFunc::Run OnClose\n");
					m_connected=false;
					//Thread::Sleep(1000);
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
					continue;
				}
			}
		}else{
			if(numSocketsReady<0) {
				//SOCKETCLIENTERROR("ERROR: %sSocketClient WaitForDataOrSignal returned %d,last select error 0x%08x\n",InstanceName(),numSocketsReady,WSAGetLastError());
				delete m_socket;
				m_socket=0;
				TIMEGUARD(OnClose(),20*1000,"WARNING: SocketClient::KeepAliveThreadFun OnClose\n");
				m_connected=false;
				std::this_thread::sleep_for(std::chrono::milliseconds(60*1000));
				//Thread::Sleep(60*1000);
				continue;
			}else{
				//uprintf("ERROR: SocketClient WaitForDataOrSignal zero last select error 0x%08x",Net::GetLastError());
			}
		}
	}
	SOCKETCLIENTNOTIFY("NOTIFY: %sSocketClient::KeepAliveThreadFunc end thread",InstanceName());
	delete m_socket;
	m_socket=0;
	if(m_connected) {
		OnClose();
	}
	m_connected=false;
	m_socketSet.SetReadSockets(nullptr,0);
	m_socketSet.SetWriteSockets(nullptr,0);
	m_socketSet.SetErrorSockets(nullptr,0);
	m_socketSet.WaitForDataOrSignal(0); // Clear any signal set.
	m_running.store(false);
	return 0;
}


#ifdef WIN32

void GetEthernetAdapterIPv4Adresses(std::vector<std::string>* interfaces, std::vector<std::string>* names) {
	if(names) {
		FATAL("GetEthernetAdapterIPv4Adresses: names argument not implemented");
	}
	interfaces->clear();
	PIP_ADAPTER_INFO adapterInfo;
	PIP_ADAPTER_INFO adapter=NULL;
	ULONG ulOutBufLen=sizeof (IP_ADAPTER_INFO);
	adapterInfo=(IP_ADAPTER_INFO *)malloc(sizeof (IP_ADAPTER_INFO));
	if(adapterInfo==NULL) {
		uprintf("Error allocating memory needed to call GetAdaptersinfo\n");
		return;
	}
	// Make an initial call to GetAdaptersInfo to get the necessary size into the ulOutBufLen variable
	if(GetAdaptersInfo(adapterInfo, &ulOutBufLen)==ERROR_BUFFER_OVERFLOW) {
		free(adapterInfo);
		adapterInfo=(IP_ADAPTER_INFO *)malloc(ulOutBufLen);
		if(adapterInfo==NULL) {
			uprintf("Error allocating memory needed to call GetAdaptersinfo\n");
			return;
		}
	}
	if((GetAdaptersInfo(adapterInfo, &ulOutBufLen))==NO_ERROR) {
		adapter=adapterInfo;
		while(adapter) {
			if(adapter->Type==MIB_IF_TYPE_ETHERNET) {
				interfaces->push_back(adapter->IpAddressList.IpAddress.String);
			}
			adapter=adapter->Next;
		}
	}
	if(adapterInfo)
		free(adapterInfo);
}

#else

void GetEthernetAdapterIPv4Adresses(std::vector<std::string>* interfaces, std::vector<std::string>* names) {
	interfaces->clear();
	if(names)
		names->clear();
	struct ifaddrs* ptr_ifaddrs=nullptr;
	auto result=getifaddrs(&ptr_ifaddrs);
	if(result){
		uprintf("error %s\n",strerror(errno));
		return;
	}
	for(struct ifaddrs* ptr_entry=ptr_ifaddrs;ptr_entry !=nullptr;ptr_entry=ptr_entry->ifa_next){
		std::string ipaddress_human_readable_form;
		std::string netmask_human_readable_form;
		std::string interface_name=std::string(ptr_entry->ifa_name);
		if(!ptr_entry->ifa_addr) continue;
		sa_family_t address_family=ptr_entry->ifa_addr->sa_family;
		if(address_family==AF_INET){
			// IPv4
			// Be aware that the `ifa_addr`, `ifa_netmask` and `ifa_data` fields might contain nullptr.
			// Dereferencing nullptr causes "Undefined behavior" problems.
			// So it is need to check these fields before dereferencing.
			if( ptr_entry->ifa_addr !=nullptr ){
				char buffer[INET_ADDRSTRLEN]={0,};
				inet_ntop(address_family,&((struct sockaddr_in*)(ptr_entry->ifa_addr))->sin_addr,buffer,INET_ADDRSTRLEN);
				ipaddress_human_readable_form=std::string(buffer);
			}
			if(ptr_entry->ifa_netmask){
				char buffer[INET_ADDRSTRLEN]={0,};
				inet_ntop(address_family,&((struct sockaddr_in*)(ptr_entry->ifa_netmask))->sin_addr,buffer,INET_ADDRSTRLEN);
				netmask_human_readable_form=std::string(buffer);
			}
			interfaces->push_back(ipaddress_human_readable_form);
			if(names)
				names->push_back(interface_name);
			//uprintf("interface_name %s\n",ipaddress_human_readable_form.c_str());
		}else
		if(address_family==AF_INET6){
		}else{
			// AF_UNIX, AF_UNSPEC, AF_PACKET etc.
			// If ignored, delete this section.
		}
	}

	freeifaddrs(ptr_ifaddrs);
}

bool printfifinfo(void) {
    struct ifaddrs* ptr_ifaddrs=nullptr;
    auto result=getifaddrs(&ptr_ifaddrs);
    if( result !=0 ){
        std::cout << "`getifaddrs()` failed: " << strerror(errno) << std::endl;
        return false;
    }
    for(
        struct ifaddrs* ptr_entry=ptr_ifaddrs;
        ptr_entry !=nullptr;
        ptr_entry=ptr_entry->ifa_next
    ){
        std::string ipaddress_human_readable_form;
        std::string netmask_human_readable_form;

        std::string interface_name=std::string(ptr_entry->ifa_name);
        sa_family_t address_family=ptr_entry->ifa_addr->sa_family;
        if( address_family==AF_INET ){
            // IPv4

            // Be aware that the `ifa_addr`, `ifa_netmask` and `ifa_data` fields might contain nullptr.
            // Dereferencing nullptr causes "Undefined behavior" problems.
            // So it is need to check these fields before dereferencing.
            if( ptr_entry->ifa_addr !=nullptr ){
                char buffer[INET_ADDRSTRLEN]={0, };
                inet_ntop(
                    address_family,
                    &((struct sockaddr_in*)(ptr_entry->ifa_addr))->sin_addr,
                    buffer,
                    INET_ADDRSTRLEN
                );

                ipaddress_human_readable_form=std::string(buffer);
            }

            if( ptr_entry->ifa_netmask !=nullptr ){
                char buffer[INET_ADDRSTRLEN]={0, };
                inet_ntop(
                    address_family,
                    &((struct sockaddr_in*)(ptr_entry->ifa_netmask))->sin_addr,
                    buffer,
                    INET_ADDRSTRLEN
                );

                netmask_human_readable_form=std::string(buffer);
            }

            std::cout << interface_name << ": IP address=" << ipaddress_human_readable_form << ", netmask=" << netmask_human_readable_form << std::endl;
        }
        else if( address_family==AF_INET6 ){
            // IPv6
            uint32_t scope_id=0;
            if( ptr_entry->ifa_addr !=nullptr ){
                char buffer[INET6_ADDRSTRLEN]={0, };
                inet_ntop(
                    address_family,
                    &((struct sockaddr_in6*)(ptr_entry->ifa_addr))->sin6_addr,
                    buffer,
                    INET6_ADDRSTRLEN
                );

                ipaddress_human_readable_form=std::string(buffer);
                scope_id=((struct sockaddr_in6*)(ptr_entry->ifa_addr))->sin6_scope_id;
            }

            if( ptr_entry->ifa_netmask !=nullptr ){
                char buffer[INET6_ADDRSTRLEN]={0, };
                inet_ntop(
                    address_family,
                    &((struct sockaddr_in6*)(ptr_entry->ifa_netmask))->sin6_addr,
                    buffer,
                    INET6_ADDRSTRLEN
                );

                netmask_human_readable_form=std::string(buffer);
            }

            std::cout << interface_name << ": IP address=" << ipaddress_human_readable_form << ", netmask=" << netmask_human_readable_form << ", Scope-ID=" << scope_id << std::endl;
        }
        else {
            // AF_UNIX, AF_UNSPEC, AF_PACKET etc.
            // If ignored, delete this section.
        }
    }

    freeifaddrs(ptr_ifaddrs);
    return true;
}

std::string GetInterfaceFromIP(const std::string& ip) {
	std::vector<std::string> interfaces;
	std::vector<std::string> names;
	GetEthernetAdapterIPv4Adresses(&interfaces,&names);
	for(unsigned int i=0; i<interfaces.size(); ++i) {
		if(ip==interfaces[i]) {
			return std::string(names[i]);
		}
	}
	return std::string();
}

void GetInterfaces(std::vector<std::string>* interfaces) {
	interfaces->clear();
    struct if_nameindex *if_nidxs, *intf;
    if_nidxs = if_nameindex();
    if ( if_nidxs != NULL ) {
        for (intf = if_nidxs; intf->if_index != 0 || intf->if_name != NULL; intf++) {
            printf("%s\n", intf->if_name);
			std::string interfaceName = intf->if_name;
			interfaces->push_back(interfaceName);
        }
        if_freenameindex(if_nidxs);
    }
}

#endif



#ifdef WIN32
std::string GetMACAdress() {
	PIP_ADAPTER_INFO AdapterInfo;
	DWORD dwBufLen=sizeof(AdapterInfo);
	AdapterInfo=(IP_ADAPTER_INFO*)malloc(sizeof(IP_ADAPTER_INFO));
	if(AdapterInfo==NULL) {
		uprintf("Error allocating memory needed to call GetAdaptersinfo\n");
	}
// Make an initial call to GetAdaptersInfo to get the necessary size into the dwBufLen     variable
	if(GetAdaptersInfo(AdapterInfo, &dwBufLen)==ERROR_BUFFER_OVERFLOW) {
		AdapterInfo=(IP_ADAPTER_INFO*)malloc(dwBufLen);
		if(AdapterInfo==NULL) {
			uprintf("Error allocating memory needed to call GetAdaptersinfo\n");
		}
		if(GetAdaptersInfo(AdapterInfo, &dwBufLen)==NO_ERROR) {
			PIP_ADAPTER_INFO pAdapterInfo = AdapterInfo;// Contains pointer to current adapter info
			do{
				std::string strMac=stdx::format_string("%02X:%02X:%02X:%02X:%02X:%02X",pAdapterInfo->Address[0], pAdapterInfo->Address[1],pAdapterInfo->Address[2], pAdapterInfo->Address[3],pAdapterInfo->Address[4], pAdapterInfo->Address[5]);
				return strMac;
	//			uprintf("Address: %s, mac: %s\n", pAdapterInfo->IpAddressList.IpAddress.String, mac_addr);
	//			return mac_addr;
	//			uprintf("\n");
				pAdapterInfo = pAdapterInfo->Next;
			}while(pAdapterInfo);
			free(AdapterInfo);
		}
	}
	return std::string("00:00:00:00:00:00");
}
#endif

#if __linux__
#include <linux/if_packet.h>
#include <net/ethernet.h> /* the L2 protocols */

std::string GetMACAdress() {
	struct ifaddrs *ifaddr=NULL;
	struct ifaddrs *ifa = NULL;
	int i=0;
	if(getifaddrs(&ifaddr)==-1) {
		perror("getifaddrs");
	}else{
		for(ifa=ifaddr;ifa!=NULL;ifa=ifa->ifa_next) {
			if((ifa->ifa_addr) && (ifa->ifa_addr->sa_family == AF_PACKET) ) {
				struct sockaddr_ll *s = (struct sockaddr_ll*)ifa->ifa_addr;
				printf("%-8s ", ifa->ifa_name);
				for (i=0; i <(int)s->sll_halen; i++) {
					printf("%02x%c", (s->sll_addr[i]), (i+1!=s->sll_halen)?':':'\n');
				}
			}
		}
		freeifaddrs(ifaddr);
	}
	return std::string("00:00:00:00:00:00");
}
#endif









