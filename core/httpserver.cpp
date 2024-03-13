#include <stdio.h>
#include <chrono>
#include <map>
#include <thread>
#include <assert.h>

#include "shared/misc.h"
#include "shared/file.h"
#include "shared/net.h"
#include "shared/sha1.h"
#include "shared/dict.h"
#include "httpserver.h"

typedef uint32_t MimeType;

static const char* base64_chars=
"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"abcdefghijklmnopqrstuvwxyz"
"0123456789+/";

std::string Base64Encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
	int out_len=(((in_len+2)/3)*4);
	std::string ret;
	ret.resize(out_len+1);
	char* buf=&ret[0];
	int lPos=0;
	int i=0;
	int j=0;
	unsigned char char_array_3[3];
	unsigned char char_array_4[4];
	while(in_len--) {
		char_array_3[i++]=*(bytes_to_encode++);
		if(i==3) {
			char_array_4[0]=(char_array_3[0]&0xfc)>>2;
			char_array_4[1]=((char_array_3[0]&0x03)<<4)+((char_array_3[1]&0xf0)>>4);
			char_array_4[2]=((char_array_3[1]&0x0f)<<2)+((char_array_3[2]&0xc0)>>6);
			char_array_4[3]=char_array_3[2]&0x3f;
			for(i=0; (i<4); i++) {
				buf[lPos++]=base64_chars[char_array_4[i]];
			}
			i=0;
		}
	}
	if(i) {
		for(j=i; j<3; j++)
			char_array_3[j]='\0';

		char_array_4[0]=(char_array_3[0]&0xfc)>>2;
		char_array_4[1]=((char_array_3[0]&0x03)<<4)+((char_array_3[1]&0xf0)>>4);
		char_array_4[2]=((char_array_3[1]&0x0f)<<2)+((char_array_3[2]&0xc0)>>6);
		char_array_4[3]=char_array_3[2]&0x3f;

		for(j=0; (j<i+1); j++)
			buf[lPos++]=base64_chars[char_array_4[j]];

		while((i++<3))
			buf[lPos++]='=';
	}
	if(lPos!=out_len)
		FATAL("Base64Encode");
	buf[lPos]=0;
	ret.resize(out_len);
	char* buf1=&ret[0];
	if(buf1!=buf)		//sanity check to validate that last resize did not reallocate buffer
		FATAL("Base64Encode");
	if(buf1[lPos])		//trailing zero still inplace after buffer
		FATAL("Base64Encode");
	return ret;
}
static inline bool is_base64(unsigned char c) {
	return true;//(isalnum(c) || (c == '+') || (c == '/'));
}
int Base64Decode(char* pOutBuffer,int lOutBufferByteSize,unsigned char const* encoded_string,unsigned int in_len) {

	static uint8_t CharacterToOffsetLookup[256];
	static bool bFirst=true;
	if(bFirst) {
		bFirst=false;
		memset(CharacterToOffsetLookup,0,sizeof(CharacterToOffsetLookup));
		int i=0;
		while(base64_chars[i]) {
			CharacterToOffsetLookup[(unsigned char)base64_chars[i]]=i;
			i++;
		}
	}
	int i=0;
	int j=0;
	int in_=0;
	unsigned char char_array_4[4],char_array_3[3];
	int lCount=0;
	while(in_len--&&(encoded_string[in_]!='=')&&is_base64(encoded_string[in_])) {
		char_array_4[i++]=encoded_string[in_]; in_++;
		if(i==4) {
			for(i=0; i<4; i++) {
				char_array_4[i]=CharacterToOffsetLookup[char_array_4[i]];
			}

			char_array_3[0]=(char_array_4[0]<<2)+((char_array_4[1]&0x30)>>4);
			char_array_3[1]=((char_array_4[1]&0xf)<<4)+((char_array_4[2]&0x3c)>>2);
			char_array_3[2]=((char_array_4[2]&0x3)<<6)+char_array_4[3];

			for(i=0; (i<3); i++) {
				if(lOutBufferByteSize<=lCount)return -1;
				pOutBuffer[lCount++]=char_array_3[i];
			}
			i=0;
		}
	}
	if(i) {
		for(j=i; j<4; j++)
			char_array_4[j]=0;

		for(j=0; j<4; j++) {
			char_array_4[j]=CharacterToOffsetLookup[char_array_4[j]];
			//char_array_4[j] = base64_chars1.find(char_array_4[j]);
		}

		char_array_3[0]=(char_array_4[0]<<2)+((char_array_4[1]&0x30)>>4);
		char_array_3[1]=((char_array_4[1]&0xf)<<4)+((char_array_4[2]&0x3c)>>2);
		char_array_3[2]=((char_array_4[2]&0x3)<<6)+char_array_4[3];

		for(j=0; (j<i-1); j++) {
			if(lOutBufferByteSize<=lCount)return -1;
			pOutBuffer[lCount++]=char_array_3[j];
		}
	}

	return lCount;
}

//FileCache
class FileCache {
	public:
		void SetRootPath(const char* path);
		std::string RootPath()const{return m_rootPath;}
		bool GetResource(BlockDataBuffer *data,const char *name)const;
	protected:
		std::string m_rootPath;
};

void FileCache::SetRootPath(const char* path) {
	uprintf("FileCache::SetRootPath %s\n",path);
	uprintf("FileCache::SetRootPath remap %s\n",GetFileNameRemap(path).c_str());
	m_rootPath=path;
}
bool FileCache::GetResource(BlockDataBuffer* data,const char* name)const {
	bool found=false;
	char path[1024];
	path[m_rootPath.copy(path,sizeof(path),0)]=0;

	//strcpy(path,sizeof(path),m_rootPath.c_str());
	strcat(path,name);
	FILE* fp=fopen(GetFileNameRemap(path).c_str(),"rb");
	if(!fp) {
		uprintf("FileCache::GetResource unable to load resource %s filename %s\n",name,GetFileNameRemap(path).c_str());
		return false;
	}
	fseek(fp, 0L, SEEK_END);//21571665
	int sz=(int)ftell(fp);
	if(sz>0) {
		char* p=new char[sz];
		rewind(fp);
		if((int)fread(p,1,sz,fp)==sz) {
			//uprintf("push send data file %s %d\n",name,sz);
			data->Push(p,sz);
			found=true;
		}
		delete[] p;
	}
	fclose(fp);
	return found;
}

typedef MimeType (*GETRESOURCECALLBACK)(BlockDataBuffer* db,const FileCache* fileCache,const char* resourceName,void* argument,const uint8_t* content,uint32_t contentLength,const char* cookie);
typedef void (*WEBSOCKETDATACALLBACK)(SocketServer::ConnectedSocket* socket,BlockDataBuffer* db,void* argument);
typedef void (*WEBSOCKETCONNECTCALLBACK)(SocketServer::ConnectedSocket* socket,void* argument);
typedef void (*WEBSOCKETCLOSECALLBACK)(SocketServer::ConnectedSocket* socket,void* argument);

class WebServer : public SocketServer {
	public:
		enum MimeType_ {
			MT_NA=0,
			MT_FORBIDDEN=-1,
			MT_TEXT_HTML=1,
			MT_JAVASCRIPT=2,
			MT_OCTET_STREAM=3,
			MT_SVG=4,
			MT_ICO=5,
			MT_CSS=6,
			MT_PNG=7,
			MT_GIF=8,
			MT_JPG=9,
			MT_JSON=10,
			MT_WOFF2=11,
			MT_TTF=12
		};
		WebServer();
		~WebServer();
		void SetRootPath(const char* path);
		std::string RootPath()const;
		void SetDefaultHtml(const char* defaultHtml);
		void SetResourceCallback(GETRESOURCECALLBACK getResourceCallback,void* argument);
		void SetWebSocketDataCallback(WEBSOCKETDATACALLBACK webSocketDataCallback,WEBSOCKETCONNECTCALLBACK webSocketConnectCallback,WEBSOCKETCLOSECALLBACK webSocketCloseCallback,void* argument);
		static MimeType GetMimeType(const char* path);
		bool Send(const std::string& data);
        virtual bool OnConnected(ConnectedSocket* socket);
		virtual bool OnData(ConnectedSocket* socket,char* data,int byteSize);
		virtual void OnClose(ConnectedSocket* socket);
		virtual void OnEvent(ConnectedSocket** sockets,int numSockets);
	protected:
		virtual const char* ClassName()const{return "WebServer";}
		void SendResponse(ConnectedSocket* socket,char* uri,uint8_t* content,uint32_t clength,const char* cookie);
		int m_indexCount;
		std::mutex m_lock;
		BlockDataBuffer m_data;
		GETRESOURCECALLBACK m_getResourceCallback;
		void* m_getResourceArgument;
		WEBSOCKETDATACALLBACK m_webSocketDataCallback;
		WEBSOCKETCONNECTCALLBACK m_webSocketConnectCallback;
		WEBSOCKETCLOSECALLBACK m_webSocketCloseCallback;
		void* m_webSocketArgument;
		std::string m_defaultHtml;
		FileCache*  m_fileCache;
};

#define SOCKETSERVERERROR uprintf
#define SOCKETSERVERWARNING uprintf
#define SOCKETSERVERNOTIFY uprintf
//#define SOCKETSERVERNOTIFY(...)

//WebServer
WebServer::WebServer() : m_defaultHtml("/index.html") {
	m_indexCount=0;
	m_getResourceArgument=0;
	m_getResourceCallback=0;
	m_webSocketDataCallback=0;
	m_webSocketConnectCallback=0;
	m_webSocketCloseCallback=0;
	m_webSocketArgument=0;
	m_fileCache = new FileCache;
}
WebServer::~WebServer(){
	delete m_fileCache;
}

void WebServer::SetRootPath(const char* path) {
	m_fileCache->SetRootPath(path);
}
std::string WebServer::RootPath()const {
	return m_fileCache->RootPath();
}

void WebServer::SetResourceCallback(GETRESOURCECALLBACK getResourceCallback,void* argument) {
	m_getResourceCallback=getResourceCallback;
	m_getResourceArgument=argument;
}

void WebServer::SetWebSocketDataCallback(WEBSOCKETDATACALLBACK webSocketDataCallback,WEBSOCKETCONNECTCALLBACK webSocketConnectCallback,WEBSOCKETCLOSECALLBACK webSocketCloseCallback,void* argument) {
	m_webSocketDataCallback=webSocketDataCallback;
	m_webSocketConnectCallback=webSocketConnectCallback;
	m_webSocketCloseCallback=webSocketCloseCallback;
	m_webSocketArgument=argument;
}
MimeType WebServer::GetMimeType(const char* path) {
	const char* p=strrchr(path,'.');
	//uprintf("GetMimeType %s\b",path);
	if(p) {
		if(!strcmp(p,".html") || !strcmp(p,".wasm"))
			return MT_TEXT_HTML;
		if(!strcmp(p,".js"))
			return MT_JAVASCRIPT;
		if(!strcmp(p,".mem") || !strcmp(p,".bin"))
			return MT_OCTET_STREAM;
		if(!strcmp(p,".svg"))
			return MT_SVG;
		if(!strcmp(p,".ico"))
			return MT_ICO;
		if(!strcmp(p,".css"))
			return MT_CSS;
		if(!strcmp(p,".jpg"))
			return MT_JPG;
		if(!strcmp(p,".png"))
			return MT_PNG;
		if(!strcmp(p,".gif"))
			return MT_GIF;
		if(!strcmp(p,".map") || !strcmp(p,".json") || !strcmp(p,".gltf"))
			return MT_JSON;
		if(!strcmp(p,".woff2"))
			return MT_WOFF2;
		if(!strcmp(p,".ttf"))
			return MT_TTF;
	}
	return MT_NA;
}
bool WebServer::OnConnected(ConnectedSocket* socket) {
	//uprintf("Connect socket %p\b",socket);
	socket->m_status=ConnectedSocket::ST_RECEIVING_HEADER;
	if(m_webSocketConnectCallback) {
		m_webSocketConnectCallback(socket,m_webSocketArgument);
	}
	return true;
}

int64_t ReadBigEndianI64(const void* m) {
	uint8_t* d=(uint8_t*)m;
	return ((int64_t)d[0]<<56)|((int64_t)d[1]<<48)|((int64_t)d[2]<<40)|((int64_t)d[3]<<32)|((int64_t)d[4]<<24)|((int64_t)d[5]<<16)|((int64_t)d[6]<<8)|((int64_t)d[7]);
}

int GetWebSocketPacketByteSize(const char* data,int dataByteSize,uint8_t* opcode,bool* fin) {
	if(dataByteSize!=4 && dataByteSize!=10)
		FATAL("GetWebSocketPacketByteSize");
	if(!(data[1]&0x80)) {
		uprintf("GetWebSocketPacketByteSize unable to decode format with mask=0\n");
		return -1;
	}
	*opcode=data[0]&0x0f;
	int payloadLength=data[1]&0x7f;
	*fin=data[0]&0x80 ? true:false;
	int maskStart=2;
	if(payloadLength==126) {
		maskStart=4;
		payloadLength=((uint8_t)data[2]<<8)|((uint8_t)data[3]);
	}else
	if(payloadLength==127) {
		maskStart = 10;
		int64_t payloadLength64=ReadBigEndianI64(data+2);
		if(payloadLength64>(int64_t)INT32_MAX){
			uprintf("GetWebSocketPacketByteSize unable to decode format with payloadlen larger than %d\n", INT32_MAX);
			uprintf("GetWebSocketPacketByteSize payloadlen is %llu\n", payloadLength64);
			return -1;
		}
		payloadLength=static_cast<int>(payloadLength64);
	}
	//uprintf("GetWebSocketPacketByteSize opcode %x payload size %d\b",*opcode,payloadLength);
	return maskStart+4+payloadLength;
}

int EncodeWebSocketPacketFromServer(char* encodeData,int lEncodeMaxByteSize,const char* data,int dataByteSize,bool binary) {
	struct WebSocketData {
		uint8_t m_flags;
		uint8_t m_payloadLength;
		uint8_t m_extendedLength[8];
	};
	WebSocketData* webSocketData=(WebSocketData*)encodeData;
	memset(webSocketData,0,sizeof(WebSocketData));
	if(binary) {
		webSocketData->m_flags=2|0x80;
	}else{
		webSocketData->m_flags=1|0x80;
	}
	int headerByteSize=2;
	if(dataByteSize>125) {
		if(dataByteSize>0xffff) {
			int64_t byteSize=dataByteSize;
			webSocketData->m_payloadLength=127;
			webSocketData->m_extendedLength[0]=(byteSize>>56)&0xff;
			webSocketData->m_extendedLength[1]=(byteSize>>48)&0xff;
			webSocketData->m_extendedLength[2]=(byteSize>>40)&0xff;
			webSocketData->m_extendedLength[3]=(byteSize>>32)&0xff;
			webSocketData->m_extendedLength[4]=(byteSize>>24)&0xff;
			webSocketData->m_extendedLength[5]=(byteSize>>16)&0xff;
			webSocketData->m_extendedLength[6]=(byteSize>>8)&0xff;
			webSocketData->m_extendedLength[7]=(byteSize)&0xff;
			headerByteSize=10;
		}else{
			webSocketData->m_payloadLength=126;
			webSocketData->m_extendedLength[0]=(dataByteSize>>8)&0xff;
			webSocketData->m_extendedLength[1]=dataByteSize&0xff;
			headerByteSize=4;
		}
	}else{
		webSocketData->m_payloadLength=dataByteSize;
	}
	int i=0;
	for(;i<dataByteSize;i++) {
		encodeData[headerByteSize+i]=data[i];
	}
	encodeData[headerByteSize+i]=0;
	return headerByteSize+dataByteSize;
}
int DecodeWebSocketPacket(char* data,int dataByteSize,int* remainByteSize) {
	if(dataByteSize<4) {
		uprintf("DecodeWebSocketPacket data byte size <4\n");
		return -1;
	}
	if(!(data[1]&0x80)) {
		uprintf("DecodeWebSocketPacket unable to decode format with mask=0\n");
		return -1;
	}
	int payloadLength=data[1]&0x7f;
	int maskStart=2;
	if(payloadLength==126) {
		maskStart=4;
		payloadLength=((uint8_t)data[2]<<8)|((uint8_t)data[3]);
	}else
	if(payloadLength==127) {
		maskStart=10;
		int64_t payloadLength64=ReadBigEndianI64(data+2);
		if(payloadLength64>INT32_MAX){
			uprintf("DecodeWebSocketPacket unable to decode format with payloadlen larger than %d\n",INT32_MAX);
			uprintf("DecodeWebSocketPacket payloadlen is %llu\n", payloadLength64);
			return -1;
		}
		payloadLength=(int)payloadLength64;
	}
	if(payloadLength>dataByteSize) {
		uprintf("DecodeWebSocketPacket payloadLength (%d)>dataByteSize (%d)\n",payloadLength,dataByteSize);
		return -1;
	}
	//uprintf("WebSocket data length %d opcode=%d Payload len %d",dataByteSize,lOpcode,payloadLength);
	unsigned char mask[4];
	mask[0]=data[maskStart+0];
	mask[1]=data[maskStart+1];
	mask[2]=data[maskStart+2];
	mask[3]=data[maskStart+3];
	int i=0;
	for(;i<payloadLength;i++) {
		data[i]=data[i+maskStart+4]^mask[i&3];
	}
	data[i]=0;
	if(remainByteSize)
		*remainByteSize=dataByteSize-(maskStart+4+payloadLength);

	return payloadLength;
}
std::string CreateWebSocketResponse(const char* key) {
	std::string strKey;
	strKey=key;
	strKey+="258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
	const unsigned char* string0=(const unsigned char*)strKey.c_str();
	ISha1* pSha1=ISha1::Create();
	pSha1->Update(string0,(uint32_t)strlen((const char*)string0));
	pSha1->Final();
	unsigned char binaryArray[20];
	pSha1->GetHash(binaryArray);
	delete pSha1;
	std::string strAccept=Base64Encode(binaryArray,sizeof(binaryArray));
	const char*  handshakeFormat = "HTTP/1.1 101 Switching Protocols\r\n"
	"Upgrade: websocket\r\n"
	"Connection: Upgrade\r\n"
	"Sec-WebSocket-Accept: %s\r\n"
	"Sec-WebSocket-Version: 13\r\n\r\n";
	std::string strresponse=stdx::format_string(handshakeFormat,strAccept.c_str());
	return strresponse;
}
void SocketServer::ConnectedSocket::EncodeAndPush(const void* data,uint32_t dataByteSize) {
	SOCKETSERVERNOTIFY("NOTIFY: SocketServer::ConnectedSocket::EncodeAndPush byte size %d",dataByteSize);
	int encodeBufferByteSize=dataByteSize*2+1000;		//Should be enough!
	if(m_status!=ConnectedSocket::ST_WS_RECEIVING_CONTENT)
		SOCKETSERVERWARNING("WARNING: HTTP socket encoding packet for websocket");
	char* encodedData=new char[encodeBufferByteSize];
	int encodedDataByteSize=EncodeWebSocketPacketFromServer(encodedData,encodeBufferByteSize,(char* )data,dataByteSize,true);
	m_send.Push(encodedData,encodedDataByteSize);
	delete [] encodedData;
}
void WebServer::OnEvent(ConnectedSocket** sockets,int num_sockets) {
	//uprintf("OnEvent %d\n",num_sockets);
	m_lock.lock();
	for(int i=0;i!=num_sockets;i++) {
		ConnectedSocket* socket=sockets[i];
		if(socket->m_status==ConnectedSocket::ST_WS_RECEIVING_CONTENT) {
			uint32_t dataByteSize=m_data.DataByteSize();
			uint8_t* data=new uint8_t[dataByteSize];
			m_data.ReadBytes(data,dataByteSize,0);
			char* encoded_data=new char[dataByteSize*2];
			int encodedDataByteSize=EncodeWebSocketPacketFromServer(encoded_data,dataByteSize*2,(char*)data,dataByteSize,true);
			socket->m_send.Push(encoded_data,encodedDataByteSize);
			delete [] encoded_data;
			delete [] data;
		}
	}
	m_lock.unlock();
}

bool WebServer::Send(const std::string& data) {
	m_lock.lock();
	m_data.Reset();
	m_data.Push(data.c_str(),(int)data.size());
	m_lock.unlock();
	SetEvent();
	return true;
}

void WebServer::SetDefaultHtml(const char* defaultHtml) {
	if(!defaultHtml) m_defaultHtml="/"; else m_defaultHtml=defaultHtml;
}

void WebServer::SendResponse(ConnectedSocket* socket,char* uri,uint8_t* content,uint32_t clength,const char* cookie) {
	char path[1024];
	strcpy(path,uri);
	char* p=strchr(path,'?');
	if(p)p[0]=0;
	if(!strcmp(path,"/")) {
		strcpy(path,m_defaultHtml.c_str());
	}
	//uprintf("WebServer::SendResponse path %s\n",path);
	BlockDataBuffer resource;
	MimeType mt=MT_NA;
	if(m_getResourceCallback)
		mt=m_getResourceCallback(&resource,m_fileCache,path,m_getResourceArgument,content,clength,cookie);
	if(mt==MT_NA) {
		if(m_fileCache->GetResource(&resource,path)) {
			mt=GetMimeType(path);
		}else{
			uprintf("WebServer::SendResponse GetResource %s failed for peer ip %s\n",path,socket->GetPeerName().ToString().c_str());
		}
	}
	switch((int)mt) {
		case MT_NA:
			socket->m_send.Push("HTTP/1.1 404 Not Found\r\n");
			socket->m_send.Push("Content-Type: text/html; charset=UTF-8\r\n\r\n");
			break;
		case MT_FORBIDDEN:
			socket->m_send.Push("HTTP/1.1 403 Forbidden\r\n");
			socket->m_send.Push("Content-Type: text/html; charset=UTF-8\r\n\r\n");
			break;
		case MT_TEXT_HTML: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: text/html; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_JAVASCRIPT: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: application/javascript; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_OCTET_STREAM: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: application/octet-stream; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_ICO: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type:image/x-icon; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		};
		case MT_SVG: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type:image/svg+xml; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_CSS: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: text/css; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_JPG: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: image/jpeg; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_PNG: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: image/png; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_GIF: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: image/gif; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_JSON: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: application/json; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_WOFF2: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: font/woff2; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		case MT_TTF: {
			socket->m_send.Push("HTTP/1.1 200 OK\r\n");
			socket->m_send.Push("Content-Type: font/ttf; charset=UTF-8\r\n\r\n");
			socket->m_send.Push(&resource);
			break;
		}
		default: {
			uprintf("ERROR: undefined mime type %d for resource %s\n",mt,path);
			socket->m_send.Push("HTTP/1.1 404 Not Found\r\n");
			socket->m_send.Push("Content-Type: text/html; charset=UTF-8\r\n\r\n");
		}
	}
	socket->m_closeAfterSend=true;
}
bool WebServer::OnData(ConnectedSocket* socket,char* data,int byteSize) {
	socket->m_received.Push(data,byteSize);
	//uprintf("Data socket %s byte size %d\n",data,ByteSize);
	if(socket->m_status==ConnectedSocket::ST_WS_RECEIVING_CONTENT) {
		while(true) {
			bool fin=false;
			int offset=0;
			int packetByteSizeMax=0;
			char header[10];
			do {
				if(socket->m_received.ReadBytes(header,sizeof(header),offset)!=sizeof(header)) {
					return true;
				}
				uint8_t opcode=0;
				int packetByteSize=GetWebSocketPacketByteSize(header,sizeof(header),&opcode,&fin);
				if(opcode==8) {		//Close socket
					return false;
				}
				offset+=packetByteSize;
				if(packetByteSize<0 || offset>(int)socket->m_received.DataByteSize()) {		//No valid packet data return and wait for more data
					return true;
				}
				packetByteSizeMax=MAX(packetByteSize,packetByteSizeMax);
			}while(!fin);
			if(m_webSocketDataCallback) {
				offset=0;
				fin=false;
				char* buf=new char[packetByteSizeMax];
				BlockDataBuffer wsd;
				do {
					if(socket->m_received.ReadBytes(header,sizeof(header),offset)!=sizeof(header)) {
						FATAL("WebServer::OnData");
					}
					uint8_t opcode=0;
					int packetByteSize=GetWebSocketPacketByteSize(header,sizeof(header),&opcode,&fin);
					socket->m_received.ReadBytes(buf,packetByteSize,offset);
					int decodedBytes=DecodeWebSocketPacket(buf,packetByteSize,0);
					if(decodedBytes<0)
						FATAL("Should not happen. GetWebSocketPacketByteSize should have returned -1");
					wsd.Push(buf,decodedBytes);
					offset+=packetByteSize;
				}while(!fin);
				delete [] buf;
				m_webSocketDataCallback(socket,&wsd,m_webSocketArgument);
			}

			socket->m_received.PopBytes(offset);
		}
		return true;
	}
	if(socket->m_received.DataByteSize()<4)
		return true;
	char buf[4];
	char uri[1024];
	char prop[1024];
	if(socket->m_received.ReadBytes(buf,4,0)==4 && !memcmp(buf,"POST",4)) {
		int headerByteSize=socket->m_received.Find("\r\n\r\n",0);
		uri[0]=0;
		if(headerByteSize!=-1) {
			int offset=socket->m_received.Find(" HTTP/1.1\r\n",0);
			int clength=0;
			std::string cookie;
			if(offset!=-1 && offset<(int)sizeof(uri)) {
				int uriByteSize=offset-5;
				if((int)socket->m_received.ReadBytes(uri,uriByteSize,5)!=uriByteSize)
					return false;
				uri[uriByteSize]=0;
				offset+=(int)strlen(" HTTP/1.1\r\n");
				while(offset<headerByteSize) {
					int end=socket->m_received.Find("\r\n",offset);
					if(end==-1)
						return false;
					int b=socket->m_received.ReadBytes(prop,end-offset,offset);
					if(b==-1)
						return false;
					prop[b]=0;
					char* value=strchr(prop,':');
					if(value) {
						value[0]=0;
						while(*++value==' ');
						if(!strcmp(prop,"Content-Length")) {
							clength=std::atoi(value);
							//uprintf("clength %d\n",clength);
						}else
						if(!strcmp(prop,"Cookie")) {
							cookie=value;
							uprintf("Cookie in http POST header. %s\n",value);
						}
						//uprintf("http value %s=%s\n",prop,value);
					}
					offset=end+2;
				}
			}
			uint8_t* content=new uint8_t[clength+1];
			if(socket->m_received.ReadBytes(content,clength,headerByteSize+4)!=clength) {
				delete [] content;
				return true;
			}
			content[clength]=0;
			SendResponse(socket,uri,content,clength,cookie.c_str());
			delete [] content;
		}
	}else
	if(socket->m_received.ReadBytes(buf,3,0)==3 && !memcmp(buf,"GET",3)) {
		int headerByteSize=socket->m_received.Find("\r\n\r\n",0);
		if(headerByteSize!=-1) {
			int offset=socket->m_received.Find(" HTTP/1.1\r\n",0);
			if(offset!=-1 && offset<(int)sizeof(uri)) {
				int uriByteSize=offset-4;
				if(socket->m_received.ReadBytes(uri,uriByteSize,4)!=uriByteSize)
					return false;
				uri[uriByteSize]=0;
				offset+=(int)strlen(" HTTP/1.1\r\n");
				std::string cookie;
				while(offset<headerByteSize) {
					int end=socket->m_received.Find("\r\n",offset);
					if(end==-1)
						return false;
					int b=socket->m_received.ReadBytes(prop,end-offset,offset);
					if(b==-1)
						return false;
					prop[b]=0;
					char* value=strchr(prop,':');
					if(value) {
						value[0]=0;
						while(*++value==' ');
						if(!strcmp(prop,"Sec-WebSocket-Key")) {
							//uprintf("received %d websocket key:%s\n",socket->m_received.DataByteSize(),value);
							std::string strresponse=CreateWebSocketResponse(value);
							socket->m_send.Push(strresponse.c_str(),(int32_t)strresponse.size());
							socket->m_status=ConnectedSocket::ST_WS_RECEIVING_CONTENT;
							socket->m_received.PopBytes(socket->m_received.DataByteSize());
							return true;
						}
						if(!strcmp(prop,"Cookie")) {
							cookie=value;
							//uprintf("Cookie in http header. %s\n",value);
						}
						//uprintf("http value %s=%s\n",prop,value);
					}
					offset=end+2;
				}
				SendResponse(socket,uri,0,0,cookie.c_str());
				//uprintf("HTTP get request path %s\n",path);
				//uprintf("Got full http packet %d\n",socket->m_received.DataByteSize());
			}else{
				uprintf("Malformed HTTP\n");
				return false;
			}
		}
	}
	return true;
}
void WebServer::OnClose(ConnectedSocket* socket) {
	//uprintf("WebServer::OnClose %p\n",socket);
	if(m_webSocketCloseCallback) {
		m_webSocketCloseCallback(socket,m_webSocketArgument);
	}
}

class HttpServerImpl : public HttpServer, WebServer {
	public:
		explicit HttpServerImpl(const Dict& httpSettings);
		virtual ~HttpServerImpl();
		void Begin();
		void End();
		void Send(const Dict& status);
		bool HasConnectedClients()const;
		virtual std::string GetBindAddress()const{return stdx::format_string("%s:%d",m_bindAddress.c_str(),m_bindPort);}
		virtual void GetStatus(Dict* status);
		virtual void SetWebsocketDataCallback(void* arg,TWebDataCallbackFunc cb){
			std::scoped_lock sl(m_websocketDataLock);
			m_arg=arg;
			m_cb=cb;
		}
		virtual void ClearWebsocketDataCallback(){
			std::scoped_lock sl(m_websocketDataLock);
			m_cb=0;
			m_arg=0;
		}
		std::mutex m_websocketDataLock;
		void* m_arg=0;
		TWebDataCallbackFunc m_cb=0;
};

HttpServer* CreateHttpServer(const Dict& httpSettings) {
	HttpServer* s=new HttpServerImpl(httpSettings);
	s->Begin();
	return s;
}
void DestroyHttpServer(HttpServer* s) {
	s->End();
	delete s;
}

HttpServerImpl::HttpServerImpl(const Dict& httpSettings) {
	int port;
	std::string ip;
	const Dict* bindAddress=httpSettings.Find("bindAddress");
	if(bindAddress) {
		bindAddress->Get("ip",&ip);
		bindAddress->Get("port",&port,8888);
	}else{
		httpSettings.Get("port",&port,8888);
		ip="0.0.0.0";
	}
	uprintf("starting http server ip %s port %d\n",ip.c_str(),port);
	SetBindAddress(ip.c_str(),port);
}
HttpServerImpl::~HttpServerImpl(){
	uprintf("~HttpServerImpl\n");
}

void HttpServerImpl::GetStatus(Dict* status) {
	status->ReadFromJson("{\"schema\":[{\"param\":\"tal\",\"type\":\"int\"}],\"httpserver\":{},\"graphs\":[]");
	Dict* settings=status->Find("httpserver");
	Dict* bindAddress=settings->AddObjectNode("bindAddress");
	bindAddress->Set("ip",m_bindAddress);
	bindAddress->Set("port",(int)m_bindPort);
}



void HttpServerImpl::Begin() {
	//uprintf("HttpServerImpl::Begin\n");
	std::string publicPath="$(DATA)/public";
	SetRootPath(publicPath.c_str());
	struct _{
		static MimeType GetResourceCallback(BlockDataBuffer* db,const FileCache* fileCache,const char* resourceName,void* argument,const uint8_t* content,uint32_t contentLength,const char* cookie) {
			//return fileCache->GetResource(db,"/index.html");
			if(!fileCache->GetResource(db,resourceName))
				return WebServer::MT_NA;
			return WebServer::GetMimeType(resourceName);
		}
		static void WebsocketEndCallback(SocketServer::ConnectedSocket* socket,void* argument) {
			//uprintf("End socket %d\n",socket->m_id);
		}
		static void WebsocketDataCallback(SocketServer::ConnectedSocket* socket,BlockDataBuffer* db,void* argument) {
			HttpServerImpl* hs=(HttpServerImpl*)argument;
			int sz=db->DataByteSize();
			std::vector<char> data(sz+1);
			int bytes=db->Pop(data.data(),sz);
			data[bytes]=0;
			if(!bytes || bytes!=sz || data[0]!='{' || data[bytes-1]!='}') {
				uprintf("ERROR: WebsocketDataCallback invalid json, sz %d\n",sz);
				return;
			}
			//uprintf("Data socket %d %s\n",socket->m_id,data.data());
			std::scoped_lock sl(hs->m_websocketDataLock);
			if(hs->m_cb)
				hs->m_cb(data.data(),bytes,hs->m_arg);
		}
		static void WebsocketBeginCallback(SocketServer::ConnectedSocket* socket,void* argument) {
			//uprintf("Begin socket %d\n",socket->m_id);
		}
	};
	SetResourceCallback(_::GetResourceCallback,this);
	SetWebSocketDataCallback(_::WebsocketDataCallback,_::WebsocketBeginCallback,_::WebsocketEndCallback,this);
	SetDefaultHtml("/index.html");
	Start();
}
void HttpServerImpl::End() {
	uprintf("HttpServerImpl::End\n");
	WebServer::End();
}
void HttpServerImpl::Send(const Dict& status) {
	std::string jsonSend=status.WriteToJson(false);
	//std::string jsonSend="{\"fromc\":\"hello\"}";
	//uprintf("jsonSend %s",jsonSend.c_str());
	//uprintf("HttpServerImpl::Update send %d bytes\n",(int)jsonSend.size());
	WebServer::Send(jsonSend);
}
bool HttpServerImpl::HasConnectedClients()const {
	int nc=NumConnectedSockets();
	return nc ? true:false;
}
