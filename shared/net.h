#pragma once

#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>

#include "shared/misc.h"
#include "shared/std_ext.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif

class SocketHandle;

void InitSockets();
void EndSockets();
void GetEthernetAdapterIPv4Adresses(std::vector<std::string>* interfaces, std::vector<std::string>* names=NULL);
std::string GetMACAdress();


bool IsWireless(const char* ifname);
bool GetRSSI(int* rssi,const char* interfaceName);


#ifdef __linux__
std::string GetInterfaceFromIP(const std::string& ip);
void GetInterfaces(std::vector<std::string>* interfaces); // returns all interfaces, also ones without address
#endif

class SocketAddress {
		friend class Socket;
		union {
			char reserve[sizeof(sockaddr)]; //sizeof(sockaddr) is 16 bytes on windows and linux
			sockaddr m_handle;
		};
	public:
		SocketAddress();
		SocketAddress(uint8_t v0,uint8_t v1,uint8_t v2,uint8_t v3,uint16_t port);
		SocketAddress(const char* address){Set(address);}
		uint8_t HostIndex()const{
			const uint8_t* p=GetIPBytes();
			return p[3];
		}
		void SetAny();
		bool IsAny()const;
		bool IsValid()const;
		uint16_t Port()const;
		void PortSet(uint16_t port);
		void Set(const char* address);
		void Set(uint8_t v0,uint8_t v1,uint8_t v2,uint8_t v3,uint16_t port);
		bool operator==(const SocketAddress& cmp)const;
		bool operator!=(const SocketAddress& cmp)const;
		uint8_t* GetIPBytes()const;
		std::string ToString()const;
};

class Socket {
	public:
		enum Error {
			ER_NA=0,
			ER_WOULDBLOCK=1,
			ER_TIMEOUT=2,
			ER_AGAIN=3,
			ER_RESET=4
		};
		union {
			SocketHandle* m_handle=0;
		};
		enum AddressFamily {
			INET,
			INET6
		};
		enum SocketType {
			STREAM,
			DGRAM
		};
		enum ProtocolType {
			TCP,
			UDP
		};
		enum Flags {
			DONTWAIT=1,
			PEEK=2,
			WAITALL=4,
		};
		AddressFamily m_family:2;
		SocketType m_socketType:2;
		ProtocolType m_protocolType:2;

		Socket(AddressFamily af,SocketType st,ProtocolType pt);
		Socket(SocketHandle* sh);

		Socket(){}
		Socket(const Socket& tocopy);
		~Socket();
		Socket& operator=(const Socket& assign);
		bool IsValid()const;

		static Socket::Error GetLastError();
		static const char* ErrorCodeToName(Socket::Error error);

		bool Bind(const SocketAddress& address);
		void SetBlockingMode(bool blocking)const;
		int SetBuffered(bool bBuffered);
		int SetSendTimeout(int milliseconds);
		int SetRecvTimeout(int milliseconds);
		bool EnableTimestampRX();
		bool OptionNoError()const;
		bool HasPeerName()const;

		SocketAddress GetPeerName();

		//DGRAM
		int SendTo(const SocketAddress& Destination,const void* pBuffer,int nLength,int nFlags=0)const;
		int ReceiveFrom(void* pBuffer,int nLength,SocketAddress* source,int nFlags=0)const;
		int ReceiveFromTime(void* buffer,int length,SocketAddress* source,uint64_t* timeEpochMicroseconds,int flags=0)const;

		//STREAM
		bool Listen(int backLog);
		bool Connect(const SocketAddress& address);
		bool ConnectNonBlock(const SocketAddress& address);
		Socket Accept();
		int Send(const void* buffer,int length,int flags=0)const;
		int Receive(void* buffer,int length,int flags=0)const;
};

struct internal_fd_set;

class SocketSet {
	public:
		SocketSet();
		~SocketSet();
		void SetReadSockets(const Socket** pSocketList, uint32_t Count);
		void SetWriteSockets(const Socket** pSocketList, uint32_t Count);
		void SetErrorSockets(const Socket** pSocketList, uint32_t Count);
		int WaitForDataOrSignal(int milliseconds=-1);
		int WaitForData(int milliseconds=-1);
		int IsRead(const Socket& Socket);
		int IsWrite(const Socket& Socket);
		int IsError(const Socket& Socket);
		void SignalSelect();
	protected:
#ifndef _WIN32
		int m_lSignalPipe[2];
		int m_lMaxSocketRead;
		int m_lMaxSocketWrite;
		int m_lMarSocketError;
#else
		Socket m_SignalSocket;
#endif
		internal_fd_set* write_fds;
		internal_fd_set* read_fds;
		internal_fd_set* error_fds;
};

class BlockDataBuffer {
	public:
		~BlockDataBuffer();
		int32_t DataByteSize()const;
		void Push(const char* data);
		void Push(const BlockDataBuffer* data);
		void Push(const void* data,int32_t dataByteSize);
		void Reset();
		int32_t Pop(void* data,int32_t dataByteSize);
		int32_t PopBytes(int32_t dataByteSize);
		int32_t ReadBytes(void* data,int32_t dataByteSize,int32_t offset)const;
		bool IsStartOfData()const{return m_first->m_pos ? false:true;}
		int32_t Find(const void* data,int32_t dataByteSize,int32_t offset)const;
		int32_t Find(const char* data,int32_t offset)const;
	protected:
		struct Data {
			int32_t m_byteSize;
			int32_t m_pos;
			Data* m_next;
			Data* m_prev;
		};
		Data* m_first=0;
		Data* m_last=0;
		bool CmpBytes(const Data* link,int offset,const char* data,int dataByteSize)const;
};

class SocketServer {
	public:
		virtual ~SocketServer();
		class ConnectedSocket : public Socket {
			public:
				enum Status {
					ST_NEW=1,
					ST_RECEIVING_HEADER=2,
					ST_WS_RECEIVING_CONTENT=3
				};
				ConnectedSocket(Socket& socket,int id) : Socket(socket) {
					m_userData=0;
					m_id=id;
					m_status=ST_NEW;
					m_closeAfterSend=false;
				}
				void EncodeAndPush(const void* data,uint32_t dataByteSize);
				//void EncodeAndPush(const MemoryBuffer& mb) {
				//	EncodeAndPush(mb.Data(),mb.ByteSize());
				//}
				void SetUserData(void* data) {
					m_userData=data;
				}
				void* GetUserData(){return m_userData;}
				int m_id;
				BlockDataBuffer m_send;
				BlockDataBuffer m_received;
				Status m_status;
				bool m_closeAfterSend;
                std::chrono::high_resolution_clock::time_point m_timeLastByteReceived;
			protected:
				void* m_userData;
		};

		virtual int NumConnectedSockets()const{return m_numConnectedSockets;}

		virtual int64_t TotalBytesSend()const{return m_totalBytesSend;}
		virtual int64_t TotalBytesReceived()const{return m_totalBytesReceived;}

		virtual bool IsRunning()const{return m_threadRunning;}

		virtual bool OnConnected(ConnectedSocket* socket)=0;
		virtual bool OnData(ConnectedSocket* socket,char* data,int dataByteSize)=0;
		virtual void OnClose(ConnectedSocket* socket)=0;
		virtual void OnEvent(ConnectedSocket** sockets,int num_sockets)=0;
		virtual void OnDestroy(ConnectedSocket* socket){}

		void SetBindAddress(const char* bindAddress,uint16_t bindPort);

		//virtual void SetPort(int port){m_port=port;}
		//int Port()const{return m_port;}

		virtual void Start();
		virtual void End();
		int Run();
		void SendToSocket(int socketId,const char* data,int dataByteSize);
		void SendToSocket(int socketId,const void** dataEntries,const int* dataByteSizes, const int numEntries);
		SocketSet m_socketSet;
	protected:
		virtual void EncodeAndSend(ConnectedSocket* socket,const void* data,int byteSize)const;

		virtual const char* ClassName()const{return "SocketServer";}
		SocketServer();
		int TimeoutSockets(const ConnectedSocket* *sockets,int socketsCount);

		void SetEvent();

		class ThreadSend {
			public:
				ThreadSend(int socketId,const char* data,int dataByteSize) {
					m_socketId=socketId;
					m_data.assign(data,data+dataByteSize);
					//FATAL("Disabled");
					//m_mb.Add(data,dataByteSize);
				}
				ThreadSend(int socketId,const void** dataEntries, const int* dataByteSizes, const int numEntries) {
					m_socketId=socketId;
					//m_data.assign(dataEntries,dataByteSizes,numEntries);
					FATAL("Disabled");
					//m_mb.Add(dataEntries,dataByteSizes, numEntries);
				}
				std::vector<char> m_data;
				//MemoryBuffer m_mb;
				int m_socketId;
		};

		std::mutex m_sendLock;
		std::vector<ThreadSend> m_sendList;
		uint64_t m_sendTime;

		std::atomic<int> m_numConnectedSockets;

		std::string m_bindAddress;
		uint16_t m_bindPort;
		int m_socketIdCount;
		int64_t m_totalBytesSend;
		int64_t m_totalBytesReceived;
		volatile int m_eventIndex;
		std::mutex m_eventLock;
		std::thread m_listenThread;
		volatile bool m_close;
		volatile bool m_threadRunning;
		bool m_verbose;
};

class SocketClient {
	public:
		SocketClient();
		virtual ~SocketClient();
		virtual const char* InstanceName()const{return "SocketClient";}
		int KeepAliveThreadFunc();
		void EnableNonBlockingConnect() { m_nonBlockConnect=true; } //Must be called before begin
		void Begin(const char* host,int port,float timerFrequency);			//Frequency is in milliseconds
		bool IsConnected()const{return m_socket ? true:false;}
		void End();
		class ConnectedSocket : public Socket {
			public:
				ConnectedSocket(Socket& socket) : Socket(socket) {
					m_closeAfterSend=false;
				}
				bool m_closeAfterSend;
				BlockDataBuffer m_send;
				BlockDataBuffer m_received;
				std::mutex m_sendLock;
		};
		virtual void OnConnected()=0;
		virtual bool OnData(const void* data,int dataByteSize)=0;
		virtual void OnClose()=0;
		virtual void OnTimer()=0;
		virtual void OnSendInjection(void* data,int dataByteSize){}
		virtual bool Running(){
			return m_running || !m_close.load();
		}
	protected:
		bool Send(const void* data,int dataByteSize);			//If this should be accessable from outside, implement wrapper func from derived class
		float m_timerFrequency;
		int m_totalBytesSend;
		int m_totalBytesReceived;
		std::thread m_keepAliveThread;
		std::atomic<bool> m_connected;
		std::atomic<bool> m_close;
		std::atomic<bool> m_running;
		bool m_nonBlockConnect;
		std::condition_variable m_waitClose;
		ConnectedSocket* m_socket;
		SocketSet m_socketSet;
		SocketAddress m_address;
		std::string m_host;
		int m_port;
};
