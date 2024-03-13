
#include <stdio.h>
#include <chrono>
#include <map>
#include <deque>
#include <set>
#include <string>
#include <vector>

#include "shared/dict.h"
#include "errorcorrection.h"
#include "packet.h"
#include "history.h"

#include "3rdparty/fec/fecbuffer.h"

void TestErrorCorrection() {
	Dict dict;
	dict.ReadFromJson(R"({"type":"fec"})");
	ErrorCorrectionEncoder* encoder=CreateErrorCorrectionEncoder(&dict);
	ErrorCorrectionDecoder* decoder=CreateErrorCorrectionDecoder(&dict);
	for(int i=0;i!=100;i++) {
		DebugData debugData;
		std::vector<std::vector<uint8_t>> fragments;
		std::vector<uint8_t> frame;
		frame.resize((1+i)*211);
		for(int j=0;j!=(int)frame.size();j++)
			frame[j]=i&0xff;
		encoder->Encode(&fragments,i,frame.data(),(int)frame.size(),debugData,1);
		for(auto& fragment:fragments) {
			decoder->Decode(fragment.data(),(int)fragment.size(),GetTimeEpochMicroseconds(),[&](int index,const uint8_t* data,int dataBytesize,const DebugData& debugData,uint64_t timeFirst){
				std::vector<uint8_t> frame1((uint8_t*)data,(uint8_t*)data+dataBytesize);
				if(frame!=frame1) {
					FATAL("not same");
				}else{
					uprintf("ok %d sz %d\n",index,dataBytesize);
				}
			});
		}
	}
	uprintf("ok\n");
	exit(0);
}

class ErrorCorrectionEncoderFec : public ErrorCorrectionEncoder {
	public:
		ErrorCorrectionEncoderFec();
		virtual ~ErrorCorrectionEncoderFec();
		virtual uint8_t Type()const{return EC_TYPE_FEC;}
		virtual void Timeout(){}
		virtual void EncodeRetransmit(std::vector<std::vector<uint8_t>>* fragments,const uint8_t* data,int dataBytesize){
			FATAL("Retransmit not supported for FEC encode");
		}
		virtual void Encode(std::vector<std::vector<uint8_t>>* fragments,uint32_t index,const uint8_t* data,int dataBytesize,const DebugData& debugData,uint8_t streamId);
	protected:
		class FecBuffer* m_fec=0;
		uint32_t m_packetIndex=0;
};

ErrorCorrectionEncoderFec::ErrorCorrectionEncoderFec() {
	m_fec=CreateFecBuffer(8,12);
}
ErrorCorrectionEncoderFec::~ErrorCorrectionEncoderFec() {
	DestroyFecBuffer(m_fec);
}
void ErrorCorrectionEncoderFec::Encode(std::vector<std::vector<uint8_t>>* fragments,uint32_t index,const uint8_t* data,int dataBytesize,const DebugData& debugData,uint8_t streamId) {

	FecFrameHeader header;
	header.m_frameIndex=index;
	header.m_debugData=debugData;
	header.m_frameBytesize=dataBytesize;

	int sendFrameBytesize=sizeof(header)+dataBytesize;

	char* sendData=new char[sendFrameBytesize];
	memcpy(sendData,&header,sizeof(FecFrameHeader));
	memcpy(sendData+sizeof(header),data,dataBytesize);

	int fragmentSizeMax=1400;

	int blockSizeMax=fragmentSizeMax*m_fec->GetK();

	int numberBlocks=(sendFrameBytesize+(blockSizeMax-1))/blockSizeMax;
	int remain=sendFrameBytesize;
	char* block=sendData;

	fragments->clear();

	uint8_t fragment[2048];
	for(int i=0;i!=numberBlocks;i++) {
		if(remain<=0)
			FATAL("WTF!");
		int blockSize=MIN(remain,blockSizeMax);
		//uprintf("frame %d block %d blocksize %d\n",index,i,blockSize);
		int fragmentSize=(blockSize+(m_fec->GetK()-1))/m_fec->GetK();
		std::vector<std::vector<uint8_t>> encoded;
		m_fec->Encode(&encoded,block,blockSize,fragmentSize);
		for(int j=0;j!=(int)encoded.size();j++) {
			FECHeader* h=(FECHeader*)fragment;
			h->m_blockSize=blockSize;
			h->m_blockCount=numberBlocks;
			h->m_blokIndex=i;
			h->m_k=m_fec->GetK();
			h->m_n=m_fec->GetN();
			h->m_fragmentSize=fragmentSize;
			h->m_fragmentIndex=j;
			h->m_bufferIndex=index;
			h->m_packetIndex=m_packetIndex++;
			memcpy(h+1,encoded[j].data(),encoded[j].size());
			fragments->emplace_back(fragment,fragment+sizeof(FECHeader)+encoded[j].size());
		}

		remain-=blockSize;
		block+=blockSize;
	}
	delete[] sendData;

}

class ErrorCorrectionDecoderFec : public ErrorCorrectionDecoder {
	public:
		ErrorCorrectionDecoderFec();
		virtual ~ErrorCorrectionDecoderFec();
		virtual int GetPacketDataIndex(const void* packet,int packetBytesize)const{
			FECHeader* h=(FECHeader*)packet;
			return h->m_bufferIndex;
		}
		virtual void Reset() {
			m_blocks.clear();
			DestroyFecBuffer(m_fec);
			m_firstToLast.SetMaxTime(60*1000000);
			m_lostPackets.SetMaxTime(60*1000000);
			m_lostFrames.SetMaxTime(60*1000000);
			m_fec=CreateFecBuffer(8,12);

			m_firstTime=0;
			m_lastTime=0;
			m_packetIndex=0;
			m_packetsLost=0;
			m_packetsReceived=0;
			m_bytesReceived=0;
			m_framesLost=0;
			m_framesDecoded=0;
			m_frameIndex=0;

			m_lastTimeLost=0;
			m_lastPacketsReceived=0;
			m_lastPacketsLost=0;

			uprintf("ErrorCorrectionDecoderFec::Reset\n");

			//FATAL("ErrorCorrectionDecoderFec::Reset not implemented");
		}
		virtual void Decode(const void* packet,int packetBytesize,uint64_t receivedTime,TBlocksDecodedCallbackFunc decodedCallback);
		virtual void GetGraphs(Dict* graphs,uint64_t timeMilliseconds,uint8_t streamId);
		virtual std::string GetStatistics(Statistics* s)const;
	protected:
		struct Block {
			std::vector<FECHeader> m_headers;
			std::vector<std::vector<uint8_t>> m_encoded;
			std::vector<uint8_t> m_data;
			bool m_decoded=false;
			bool Decode(FecBuffer* fec,const void* packet,int packetBytesize);
		};

		FecBuffer* m_fec=0;
		uint64_t m_firstTime=0;
		uint64_t m_lastTime=0;
		uint32_t m_packetIndex=0;
		uint32_t m_packetsLost=0;
		uint32_t m_packetsReceived=0;
		uint32_t m_bytesReceived=0;
		uint32_t m_framesLost=0;
		uint32_t m_framesDecoded=0;
		int m_frameIndex=0;

		uint64_t m_lastTimeLost=0;
		uint32_t m_lastPacketsReceived=0;
		uint32_t m_lastPacketsLost=0;

		std::vector<Block> m_blocks;
		History m_firstToLast;
		History m_lostPackets;
		History m_lostFrames;
};
bool ErrorCorrectionDecoderFec::Block::Decode(FecBuffer* fec,const void* packet,int packetBytesize) {
	FECHeader* h=(FECHeader*)packet;
	if(m_decoded)
		return false;
	m_encoded.emplace_back((char*)(h+1),((char*)packet)+packetBytesize);
	m_headers.push_back(*h);
	if((int)m_headers.size()<h->m_k)
		return false;
	std::vector<bool> received(h->m_n,false);
	std::vector<std::vector<uint8_t>> encoded;
	std::vector<uint8_t> block(h->m_fragmentSize);
	for(int i=0;i!=h->m_n;i++) {
		encoded.push_back(block);
	}
	for(int i=0;i!=(int)m_headers.size();i++) {
		encoded[m_headers[i].m_fragmentIndex]=m_encoded[i];
		received[m_headers[i].m_fragmentIndex]=true;
	}
	m_data.resize(h->m_k*h->m_fragmentSize);
	if(fec->Decode(m_data.data(),(int)m_data.size(),received,encoded,h->m_fragmentSize)) {
		m_decoded=true;
		m_encoded.clear();
		return true;
	}else{
		FATAL("block decode failed");
	}
	return false;
}

ErrorCorrectionDecoderFec::ErrorCorrectionDecoderFec() {
	m_firstToLast.SetMaxTime(60*1000000);
	m_lostPackets.SetMaxTime(60*1000000);
	m_lostFrames.SetMaxTime(60*1000000);
	m_fec=CreateFecBuffer(8,12);
}
ErrorCorrectionDecoderFec::~ErrorCorrectionDecoderFec() {
	m_blocks.clear();
	DestroyFecBuffer(m_fec);
}

void ErrorCorrectionDecoderFec::GetGraphs(Dict* graphs,uint64_t timeMilliseconds,uint8_t streamId) {
	{
		Dict* graph=graphs->PushBack();
		graph->Set("xmax",(int64_t)timeMilliseconds);
		graph->Set("xmin",(int64_t)(timeMilliseconds-(m_firstToLast.GetMaxTime()/1000)));
		graph->Set("ymin",0.0);
		graph->Set("formatX","HH:MM:SS");
		graph->Set("formatY","us");
		Dict* legend=graph->AddObjectNode("legend");
		legend->Set("text","Timers");
		legend->Set("color","#0000ff");
		Dict* datasets=graph->AddArrayNode("datasets");

		Dict* dataset=datasets->AddObjectNode();
		dataset->Set("name","first to last");
		dataset->Set("color","#dd8452");

		std::vector<double> datax;
		std::vector<double> datay;
		m_firstToLast.Get(&datax,&datay,timeMilliseconds*1000);
		dataset->SetTypedArray("datax",datax.data(),datax.size());
		dataset->SetTypedArray("datay",datay.data(),datay.size());
	}

	{
		Dict* graph=graphs->PushBack();
		graph->Set("columns",true);
		graph->Set("xmax",(int64_t)timeMilliseconds);
		graph->Set("xmin",(int64_t)(timeMilliseconds-(m_lostPackets.GetMaxTime()/1000)));
		graph->Set("ymin",0.0);
		graph->Set("formatX","HH:MM:SS");
		graph->Set("formatY","int");
		Dict* legend=graph->AddObjectNode("legend");
		legend->Set("text","Quality");
		legend->Set("color","#0000ff");
		Dict* datasets=graph->AddArrayNode("datasets");
		Dict* dataset=datasets->AddObjectNode();
		dataset->Set("name","lost packets");
		dataset->Set("color","#dd4232");
		std::vector<double> datax;
		std::vector<double> datay;
		m_lostPackets.Get(&datax,&datay,timeMilliseconds*1000);
		dataset->SetTypedArray("datax",datax.data(),datax.size());
		dataset->SetTypedArray("datay",datay.data(),datay.size());
	}

	{
		Dict* graph=graphs->PushBack();
		graph->Set("columns",true);
		graph->Set("xmax",(int64_t)timeMilliseconds);
		graph->Set("xmin",(int64_t)(timeMilliseconds-(m_lostFrames.GetMaxTime()/1000)));
		graph->Set("ymin",0.0);
		graph->Set("formatX","HH:MM:SS");
		graph->Set("formatY","int");
		Dict* legend=graph->AddObjectNode("legend");
		legend->Set("text","Stream");
		legend->Set("color","#0000ff");
		Dict* datasets=graph->AddArrayNode("datasets");
		Dict* dataset=datasets->AddObjectNode();
		dataset->Set("name","lost frames");
		dataset->Set("color","#68a855");
		std::vector<double> datax;
		std::vector<double> datay;
		m_lostFrames.Get(&datax,&datay,timeMilliseconds*1000);
		dataset->SetTypedArray("datax",datax.data(),datax.size());
		dataset->SetTypedArray("datay",datay.data(),datay.size());
	}
}

std::string ErrorCorrectionDecoderFec::GetStatistics(Statistics* s)const {
	s->m_bytesReceived=m_bytesReceived-s->m_lastBytesReceived;
	s->m_packetsReceived=m_packetsReceived-s->m_lastPacketsReceived;
	s->m_packetsLost=m_packetsLost-s->m_lastPacketsLost;
	s->m_framesLost=m_framesLost-s->m_lastFramesLost;
	s->m_framesDecoded=m_framesDecoded-s->m_lastFramesDecoded;

	s->m_lastBytesReceived=m_bytesReceived;
	s->m_lastPacketsReceived=m_packetsReceived;
	s->m_lastPacketsLost=m_packetsLost;
	s->m_lastFramesLost=m_framesLost;
	s->m_lastFramesDecoded=m_framesDecoded;
	uint64_t t=GetTimeEpochMicroseconds();
	double bitsPerSecond=0;
	if(s->m_lastTime) {
		double deltaSeconds=((double)t-s->m_lastTime)/1000000.0;
		bitsPerSecond=(s->m_bytesReceived*8)/deltaSeconds;
	}
	s->m_lastTime=t;
	float prc=0;
	if(s->m_packetsReceived) {
		prc=((float)s->m_packetsLost*100.0f)/(float)(s->m_packetsReceived+s->m_packetsLost);
	}
	return stdx::format_string("frames %d lost %d packets %d lost %d / %.4f%% Mbps %.2f",s->m_framesDecoded,s->m_framesLost,s->m_packetsReceived,s->m_packetsLost,prc,bitsPerSecond/1000000.0);
}

void ErrorCorrectionDecoderFec::Decode(const void* packet,int packetBytesize,uint64_t receivedTime,TBlocksDecodedCallbackFunc decodedCallback) {
	FECHeader* h=(FECHeader*)packet;
	//uprintf("receive frame %d block %d fragment %d\n",h->m_bufferIndex,h->m_blokIndex,h->m_fragmentIndex);
	if(h->m_k!=8 || h->m_n!=12)
		FATAL("forkert pakke");

	if((int)m_blocks.size()<h->m_blockCount) {
		m_blocks.resize(10);//h->m_blockCount);
	}
	Block* rd=&m_blocks[h->m_blokIndex];
	if(m_packetIndex && m_packetIndex+1!=h->m_packetIndex) {
		int lost=h->m_packetIndex-(m_packetIndex+1);
		if(lost<0)
			FATAL("WTF!");
		m_packetsLost+=lost;
	}
	m_packetIndex=h->m_packetIndex;
	m_packetsReceived++;
	m_bytesReceived+=packetBytesize;

	if(receivedTime-m_lastTimeLost>1000000) {
		uint32_t packetsLost=m_packetsLost-m_lastPacketsLost;
		m_lostPackets.Register(receivedTime, packetsLost);
		m_lastTimeLost=receivedTime;
		m_lastPacketsReceived=m_packetsReceived;
		m_lastPacketsLost=m_packetsLost;
	}
	if(!m_fec || m_fec->GetK()!=h->m_k || m_fec->GetN()!=h->m_n) {
		FATAL("received fec packet with k=%d and n=%d, unable to find decoder\n",h->m_k,h->m_n);
	}
	if(rd->m_headers.size() && rd->m_headers[0].m_bufferIndex!=h->m_bufferIndex) {		//New packet?
		//uprintf("next buffer index %d!=%d\n",rd->m_headers[0].m_bufferIndex,h->m_bufferIndex);
		if(m_firstTime && m_lastTime)
			m_firstToLast.RegisterMax(receivedTime,(int)(m_lastTime-m_firstTime));
		m_firstTime=receivedTime;
		m_lastTime=0;
		for(int i=0;i!=(int)m_blocks.size();i++) {
			m_blocks[i].m_decoded=false;
			m_blocks[i].m_data.clear();
			m_blocks[i].m_headers.clear();
			m_blocks[i].m_encoded.clear();
		}
		rd->m_encoded.emplace_back((char*)(h+1),((char*)packet)+packetBytesize);
		rd->m_headers.push_back(*h);
	}else{
		m_lastTime=receivedTime;
		if(rd->Decode(m_fec,packet,packetBytesize)) {
			if(h->m_blockCount>m_blocks.size())
				FATAL("underligt");

			bool allBlocksDecoded=true;
			for(int i=0;i!=h->m_blockCount;i++) {
				allBlocksDecoded&=m_blocks[i].m_decoded;
			}
			if(allBlocksDecoded) {
				std::vector<uint8_t> sendData;
				for(int i=0;i!=h->m_blockCount;i++) {
					sendData.insert(sendData.end(),m_blocks[i].m_data.data(),m_blocks[i].m_data.data()+m_blocks[i].m_headers[0].m_blockSize);
				}
				const FecFrameHeader* fecFrameHeader=(const FecFrameHeader*)sendData.data();
				if(fecFrameHeader->m_frameBytesize!=(int)sendData.size()-(int)sizeof(FecFrameHeader)) {
					uprintf("frame size %d!=%d\n",fecFrameHeader->m_frameBytesize,(int)sendData.size()-(int)sizeof(FecFrameHeader));
				}else{
					//uprintf("frame %d blocks %d\n",fecFrameHeader->m_frameIndex,h->m_blockCount);
					if(m_frameIndex && fecFrameHeader->m_frameIndex!=m_frameIndex+1) {
						uprintf("frame lost %d\n",m_frameIndex+1);
						m_framesLost++;
						m_lostFrames.RegisterAccumulate(receivedTime,1);
					}else{
						m_lostFrames.RegisterAccumulate(receivedTime,0);
					}
					m_framesDecoded+=1;
					m_frameIndex=fecFrameHeader->m_frameIndex;
					decodedCallback(m_frameIndex,(const uint8_t*)(fecFrameHeader+1),fecFrameHeader->m_frameBytesize,fecFrameHeader->m_debugData,m_firstTime);
				}
			}
		}
	}
}














//DataChunks
struct MissingBlockSendHeader {
	int m_numberBlocks;
	uint8_t m_streamId;
	uint32_t m_frameIndex;
};
struct MissingBlockSend {
	int m_offset;
	int m_size;
};

class DataChunks {
	public:
		struct Chunk {
			int m_chunkChecksum;
			//int m_chunkIndex;			//Debug counter for chunks
			uint16_t m_dataIndex;		//Data index, Same for all chunks
			uint16_t m_chunkByteSize;	//Chunk byte size without header
			int m_chunkOffset;			//Chunk position in databuffer
			int m_dataBytesize;			//Data byte size. Same for all chunks
			uint64_t m_timeSendServer;
			uint64_t m_loopback;
			int m_timeSpendServer;
			DebugData m_debugData;
			int CheckSum()const{
				const char* st=(char*)&m_dataIndex;
				const char* en=((char*)(this+1))+m_chunkByteSize;
				return CRC32(st,(int)(en-st));
			}
		};
		bool Add(const Chunk* chunk);
		bool GotAllChunks()const;
		void Reset() {
			m_chunkCount=0;
			m_dataIndex=0;
			m_retryCount=0;
			m_data.clear();
			m_links.clear();
		}
		void GetMissingBlocks(MissingBlockSendHeader* MissingBlocksHeader,std::vector<MissingBlockSend>* MissingBlocks);
		int m_dataIndex;
		int m_retryCount;
		int m_chunkCount;
		DebugData m_debugData;
		uint64_t m_firstChunkTime;
		uint64_t m_lastChunkTime;
		std::vector<uint8_t> m_data;
	protected:
		struct Link {
			int m_offset;
			int m_size;
		};
		std::deque<Link> m_links;
};


bool DataChunks::GotAllChunks()const {
	return m_links.size()==0;
}
void DataChunks::GetMissingBlocks(MissingBlockSendHeader* MissingBlocksHeader,std::vector<MissingBlockSend>* MissingBlocks) {
	MissingBlocksHeader->m_numberBlocks=(int)m_links.size();
	MissingBlocksHeader->m_frameIndex=m_dataIndex;
	for(auto it=m_links.begin();it!=m_links.end();++it) {
		MissingBlocks->push_back({it->m_offset,it->m_size});
	}
	m_retryCount++;
	//uprintf("DataChunks::GetMissingBlocks %d count %d\n",MissingBlocksHeader->m_numberBlocks,m_retryCount);
}
bool DataChunks::Add(const DataChunks::Chunk* chunk) {
	uint64_t time=GetTimeEpochMicroseconds();
	if(!m_data.size()) {
		m_firstChunkTime=time;
		m_data.resize(chunk->m_dataBytesize);
		m_debugData=chunk->m_debugData;
		m_links.clear();
		m_links.push_back({0,chunk->m_dataBytesize});		//Mark entire block as unreceived
		m_dataIndex=chunk->m_dataIndex;
		m_retryCount=0;
		m_chunkCount=0;
	}else{
		if(chunk->m_dataIndex!=m_dataIndex) {
			return false;
		}
		if((int)m_data.size()!=chunk->m_dataBytesize)
			FATAL("DataChunks::Add invalid chunks data size %d!=%d",(int)m_data.size(),chunk->m_dataBytesize);
	}
	m_lastChunkTime=time;
	m_chunkCount++;
	if(chunk->m_chunkOffset<0)
		FATAL("DataChunks::Add chunk->m_chunkOffset less than zero\n");
	if(chunk->m_chunkOffset+chunk->m_chunkByteSize>(int)m_data.size())
		FATAL("DataChunks::Add chunk size outside buffer\n");
	memcpy(m_data.data()+chunk->m_chunkOffset,chunk+1,chunk->m_chunkByteSize);
	for(auto it=m_links.begin();it!=m_links.end();++it) {	//Cut received block from free space links
		if(it->m_offset<=chunk->m_chunkOffset && it->m_offset+it->m_size>=chunk->m_chunkOffset+chunk->m_chunkByteSize) {
			if(it->m_offset==chunk->m_chunkOffset && it->m_size==chunk->m_chunkByteSize) {
				m_links.erase(it);
				return true;
			}
			if(it->m_offset==chunk->m_chunkOffset) {
				it->m_offset+=chunk->m_chunkByteSize;
				it->m_size-=chunk->m_chunkByteSize;
				return true;
			}
			if(it->m_offset+it->m_size==chunk->m_chunkOffset+chunk->m_chunkByteSize) {
				it->m_size-=chunk->m_chunkByteSize;
				return true;
			}
			int o=it->m_offset;
			it->m_size=(it->m_offset+it->m_size)-(chunk->m_chunkOffset+chunk->m_chunkByteSize);
			it->m_offset=chunk->m_chunkOffset+chunk->m_chunkByteSize;
			m_links.insert(it,{o,chunk->m_chunkOffset-o});
			return true;
		}
	}
	//uprintf("DataChunks::Add received same chunk. Can happen if chunk is requested again\n");
	return false;
}


class ErrorCorrectionEncoderARQ : public ErrorCorrectionEncoder {
	public:
		ErrorCorrectionEncoderARQ();
		virtual ~ErrorCorrectionEncoderARQ();
		virtual uint8_t Type()const{return EC_TYPE_ARQ;}
		virtual void Timeout();
		virtual void Encode(std::vector<std::vector<uint8_t>>* fragments,uint32_t index,const uint8_t* data,int dataBytesize,const DebugData& debugData,uint8_t streamId);
		virtual void EncodeRetransmit(std::vector<std::vector<uint8_t>>* blocks,const uint8_t* data,int dataBytesize);
	protected:
		struct SendData {
			uint32_t m_index;
			uint8_t m_streamId;
			std::vector<uint8_t> m_data;
			DebugData m_debugData;
		};
		mutable std::mutex m_sendDataLock;
		std::deque<SendData> m_sendDatas;
};

ErrorCorrectionEncoderARQ::ErrorCorrectionEncoderARQ(){

}
ErrorCorrectionEncoderARQ::~ErrorCorrectionEncoderARQ() {
}

void ErrorCorrectionEncoderARQ::Timeout() {
	m_sendDataLock.lock();
	while(m_sendDatas.size()) {
		if(m_sendDatas.size()<8)
			break;
		m_sendDatas.pop_front();
	}
	m_sendDataLock.unlock();
}

void ErrorCorrectionEncoderARQ::EncodeRetransmit(std::vector<std::vector<uint8_t>>* blocks,const uint8_t* data,int dataBytesize) {
	const MissingBlockSendHeader* missingBlocksHeader=(const MissingBlockSendHeader*)data;
	const MissingBlockSend* MissingBlocks=(const MissingBlockSend*)(missingBlocksHeader+1);
	//uprintf("Blocks %d frame number %d\n",missingBlocksHeader->m_numberBlocks,missingBlocksHeader->m_frameIndex);
	for(int i=0;i!=missingBlocksHeader->m_numberBlocks;i++) {
		//uprintf("Block %d %d,%d\n",i,MissingBlocks[i].m_offset,MissingBlocks[i].m_size);
		std::scoped_lock sl(m_sendDataLock);
		for(SendData& sd:m_sendDatas) {
			if(sd.m_streamId==missingBlocksHeader->m_streamId && sd.m_index==missingBlocksHeader->m_frameIndex) {
				int offset=MissingBlocks[i].m_offset;
				int size=MissingBlocks[i].m_size;
				uint64_t timeClientLastPacket=GetTimeEpochMicroseconds();
				char chunk[4096];
				int chunkSize=1400;
				if(chunkSize>(int)sizeof(chunk))
					FATAL("VideoServerImpl::SplitAndSendChunks chunk size too small");
				memset(chunk,0x5a,chunkSize);
				DataChunks::Chunk* c=(DataChunks::Chunk*)chunk;
				int dataOffsetEnd=offset+size;
				uint64_t time=GetTimeEpochMicroseconds();
				int cnt=0;
				int sz=0;
				while(offset<dataOffsetEnd) {
					int chunkByteSize=MIN(dataOffsetEnd-offset,chunkSize-(int)sizeof(DataChunks::Chunk));
					if(!chunkByteSize)
						break;
					c->m_debugData=sd.m_debugData;
					c->m_timeSpendServer=0;
					c->m_timeSendServer=time;
					c->m_loopback=timeClientLastPacket;
					c->m_dataIndex=sd.m_index;
					c->m_chunkOffset=offset;
					c->m_chunkByteSize=chunkByteSize;
					c->m_dataBytesize=(int)sd.m_data.size();

					if((char*)(c+1)+c->m_chunkByteSize>chunk+chunkSize)
						FATAL("RO1");
					if(c->m_chunkOffset>c->m_dataBytesize)
						FATAL("RO2");
					memcpy(c+1,sd.m_data.data()+c->m_chunkOffset,c->m_chunkByteSize);
					c->m_chunkChecksum=c->CheckSum();
					int packetByteSize=sizeof(DataChunks::Chunk)+c->m_chunkByteSize;

					blocks->emplace_back(chunk,chunk+packetByteSize);

					offset+=chunkByteSize;
					cnt++;
					sz+=packetByteSize;
				}
				return;
			}
		}
		//uprintf("VideoServerImpl::OnData Frame %d not found\n",MissingBlocksHeader->m_frameIndex);
	}
}

void ErrorCorrectionEncoderARQ::Encode(std::vector<std::vector<uint8_t>>* fragments,uint32_t index,const uint8_t* data,int dataBytesize,const DebugData& debugData,uint8_t streamId) {
	int offset=0;
	int size=dataBytesize;

	uint64_t timeClientLastPacket=GetTimeEpochMicroseconds();

	m_sendDataLock.lock();
	m_sendDatas.push_back({index,streamId,std::vector<uint8_t>(data,data+dataBytesize),debugData});
	m_sendDataLock.unlock();

	char chunk[4096];
	int chunkSize=1400;
	if(chunkSize>(int)sizeof(chunk))
		FATAL("VideoServerImpl::SplitAndSendChunks chunk size too small");
	memset(chunk,0x5a,chunkSize);
	DataChunks::Chunk* c=(DataChunks::Chunk*)chunk;
	int dataOffsetEnd=offset+size;
	uint64_t time=GetTimeEpochMicroseconds();
	//int timeSpendServer=(int)(time-timeServerLastPacket);
	int cnt=0;
	int sz=0;
	while(offset<dataOffsetEnd) {
		int chunkByteSize=MIN(dataOffsetEnd-offset,chunkSize-(int)sizeof(DataChunks::Chunk));
		if(!chunkByteSize)
			break;
		//c->m_chunkIndex=m_chunkIndex++;
		c->m_debugData=debugData;
		c->m_timeSpendServer=0;
		c->m_timeSendServer=time;
		c->m_loopback=timeClientLastPacket;
		c->m_dataIndex=index;
		c->m_chunkOffset=offset;
		c->m_chunkByteSize=chunkByteSize;
		c->m_dataBytesize=dataBytesize;
		if((char*)(c+1)+c->m_chunkByteSize>chunk+chunkSize)
			FATAL("RO");
		if(c->m_chunkOffset>dataBytesize)
			FATAL("RO");
		memcpy(c+1,data+c->m_chunkOffset,c->m_chunkByteSize);
		c->m_chunkChecksum=c->CheckSum();
		int packetByteSize=sizeof(DataChunks::Chunk)+c->m_chunkByteSize;
		fragments->emplace_back(chunk,chunk+packetByteSize);
		offset+=chunkByteSize;
		cnt++;
		sz+=packetByteSize;
	}
}

class ErrorCorrectionDecoderARQ : public ErrorCorrectionDecoder {
	public:
		ErrorCorrectionDecoderARQ();
		virtual ~ErrorCorrectionDecoderARQ();
		virtual void Reset();
		virtual int GetPacketDataIndex(const void* packet,int packetBytesize)const{
			DataChunks::Chunk* c=(DataChunks::Chunk*)packet;
			return c->m_dataIndex;
		}
		virtual void RequestRetransmit(uint8_t streamId,uint8_t dataSourceType,const void* packet,int packetBytesize,uint64_t receivedTime,TBlocksRetransmitRequestCallbackFunc retransmitCallback);
		virtual void Decode(const void* packet,int packetBytesize,uint64_t receivedTime,TBlocksDecodedCallbackFunc decodedCallback);
		virtual void GetGraphs(Dict* graphs,uint64_t timeMilliseconds,uint8_t streamId);
	protected:
		uint8_t m_receivedFrames[256];
		mutable std::mutex m_dataChunksLock;
		std::deque<DataChunks> m_dataChunks;
		History m_rerequestFrames;
		History m_lostFrames;
};
ErrorCorrectionDecoderARQ::ErrorCorrectionDecoderARQ() {
	m_rerequestFrames.SetMaxTime(20*1000000);
	m_lostFrames.SetMaxTime(20*1000000);
	memset(m_receivedFrames,0,sizeof(m_receivedFrames));
}
ErrorCorrectionDecoderARQ::~ErrorCorrectionDecoderARQ() {
}
void ErrorCorrectionDecoderARQ::Reset() {
	m_dataChunks.clear();
	memset(m_receivedFrames,0,sizeof(m_receivedFrames));
}
void ErrorCorrectionDecoderARQ::GetGraphs(Dict* graphs,uint64_t timeMilliseconds,uint8_t streamId) {
	Dict* graph=graphs->PushBack();
	graph->Set("columns",true);
	graph->Set("xmax",(int64_t)timeMilliseconds);
	graph->Set("xmin",(int64_t)(timeMilliseconds-(m_rerequestFrames.GetMaxTime()/1000)));
	graph->Set("ymin",0.0);
	graph->Set("formatX","HH:MM:SS");
	graph->Set("formatY","int");
	Dict* legend=graph->AddObjectNode("legend");
	legend->Set("text",stdx::format_string("Errors streamid %d",streamId).c_str());
	legend->Set("color","#0000ff");
	Dict* datasets=graph->AddArrayNode("datasets");
	Dict* dataset=datasets->AddObjectNode();
	std::vector<double> datax;
	std::vector<double> datay;
	m_rerequestFrames.Get(&datax,&datay,timeMilliseconds*1000);
	dataset->Set("name","retransmit");
	dataset->Set("color","#dd8452");
	dataset->SetTypedArray("datax",datax.data(),datax.size());
	dataset->SetTypedArray("datay",datay.data(),datay.size());

	dataset=datasets->AddObjectNode();
	m_lostFrames.Get(&datax,&datay,timeMilliseconds*1000);
	dataset->Set("name","lost");
	dataset->Set("color","#68a855");
	dataset->SetTypedArray("datax",datax.data(),datax.size());
	dataset->SetTypedArray("datay",datay.data(),datay.size());
}

void ErrorCorrectionDecoderARQ::RequestRetransmit(uint8_t streamId,uint8_t dataSourceType,const void* packet,int packetBytesize,uint64_t receivedTime,TBlocksRetransmitRequestCallbackFunc retransmitCallback) {
	DataChunks::Chunk* c=(DataChunks::Chunk*)packet;
	m_dataChunksLock.lock();
	while(m_dataChunks.size() && m_dataChunks.front().m_dataIndex+2<=c->m_dataIndex) {
		m_lostFrames.RegisterAccumulate(receivedTime,1);
		uprintf("did not receive all chunks for frame %d. ignore frame.\n",m_dataChunks.front().m_dataIndex);
		m_dataChunks.pop_front();
	}
	for(auto it=m_dataChunks.begin();it!=m_dataChunks.end();) {
		DataChunks* dc=&*it;
		if(dc->GotAllChunks()) {
			++it;
			continue;
		}
		if(dc->m_dataIndex+1==c->m_dataIndex) {
			if(dc->m_retryCount) {
				++it;
				continue;
			}
			m_rerequestFrames.RegisterAccumulate(receivedTime,1);
			//uprintf("received chunk from new frame %d before all packets from previous frame was received\n",c->m_dataIndex);
			//uprintf("p %p stream id %d lost packet\n",this,streamId);
			std::vector<MissingBlockSend> missingBlocks;
			MissingBlockSendHeader missingBlocksHeader;
			missingBlocksHeader.m_streamId=streamId;
			dc->GetMissingBlocks(&missingBlocksHeader,&missingBlocks);
			if(!missingBlocks.size()) {
				uprintf("Incomplete datachunks %d but no missing blocks. Should not be possible got all chunks ? %d\n",dc->m_dataIndex,dc->GotAllChunks());
				it=m_dataChunks.erase(it);
				continue;
			}
			RequestMissingChunksHeader rmch;
			rmch.m_timeSend=GetTimeEpochMicroseconds();
			rmch.m_dataBytesize=(int)sizeof(missingBlocksHeader)+(int)missingBlocks.size()*sizeof(MissingBlockSend);
			rmch.m_streamId=streamId;
			rmch.m_dataSourceType=dataSourceType;
			std::vector<uint8_t> p;
			p.resize(rmch.m_headerBytesize+rmch.m_dataBytesize);
			memcpy(p.data(),&rmch,rmch.m_headerBytesize);
			memcpy(p.data()+rmch.m_headerBytesize,&missingBlocksHeader,sizeof(missingBlocksHeader));
			memcpy(p.data()+rmch.m_headerBytesize+sizeof(missingBlocksHeader),missingBlocks.data(),missingBlocks.size()*sizeof(MissingBlockSend));

			retransmitCallback(p.data(),(int)p.size());
			//ah->m_transfer->SendToHost(p.data(),(int)p.size());
			uprintf("request missing chunks %d for frame %d\n",(int)missingBlocks.size(),dc->m_dataIndex);
		}
		++it;
	}
	m_dataChunksLock.unlock();
}

void ErrorCorrectionDecoderARQ::Decode(const void* packet,int packetBytesize,uint64_t receivedTime,TBlocksDecodedCallbackFunc decodedCallback) {
	DataChunks::Chunk* c=(DataChunks::Chunk*)packet;
	if(packetBytesize!=(int)sizeof(DataChunks::Chunk)+c->m_chunkByteSize)
		FATAL("VideoClientImpl::OnData invalid packet size %d!=%d+%d+%d\n",packetBytesize,(int)sizeof(PacketHeaderBase),(int)sizeof(DataChunks::Chunk),c->m_chunkByteSize);
	if(c->m_chunkChecksum!=c->CheckSum())
		FATAL("chunk checksum failed");
	bool found=false;
	m_dataChunksLock.lock();

	{
		int idx=(c->m_dataIndex)&0xff;        // [0-255]
		int gen=((c->m_dataIndex>>8)&0x7f)+1; // [1-128], 0 means unused.
		if(m_receivedFrames[idx]==gen){
			uprintf("frame %d allready received: idx=%d, gen=%d\n",c->m_dataIndex,idx,gen);
			m_dataChunksLock.unlock();
			return;
		}
	}

	for(auto& dc:m_dataChunks) {
		if(dc.m_dataIndex==c->m_dataIndex) {
			dc.Add(c);
			found=true;
			break;
		}
	}
	if(!found) {
		m_dataChunks.emplace_back();
		m_dataChunks.back().Add(c);
	}
	while(m_dataChunks.size() && m_dataChunks.front().GotAllChunks()) {
		const DataChunks& dataChunks=m_dataChunks.front();
		int idx=dataChunks.m_dataIndex&0xff;
		int gen=((dataChunks.m_dataIndex>>8)&0x7f)+1;
		//uprintf("frame %d complete: idx=%d, gen=%d, old gen=%d\n", dataChunks.m_dataIndex, idx, gen, m_receivedFrames[idx]);
		m_receivedFrames[idx]=gen;

		decodedCallback(dataChunks.m_dataIndex,dataChunks.m_data.data(),(int)dataChunks.m_data.size(),dataChunks.m_debugData,dataChunks.m_firstChunkTime);
		m_dataChunks.pop_front();
	}
	m_dataChunksLock.unlock();
}

ErrorCorrectionEncoder* CreateErrorCorrectionEncoder(const Dict* settings) {
	std::string type="fec";
	if(settings) {
		settings->Get("type",&type);
	}
	if(type=="fec") {
		return new ErrorCorrectionEncoderFec;
	}
	if(type=="arq") {
		return new ErrorCorrectionEncoderARQ;
	}
	FATAL("Unknow frame encode type %s. Valid types \"fec\" and \"arq\"",type.c_str());
	return 0;
}
void DestroyErrorCorrectionEncoder(ErrorCorrectionEncoder* be) {
	delete be;
}

//Forward error correction "fec"
//Backward error correction (Automatic Repeat Request) "arq"

ErrorCorrectionDecoder* CreateErrorCorrectionDecoder(uint8_t errorCorrectionType) {
	switch(errorCorrectionType) {
		case EC_TYPE_FEC:
			return new ErrorCorrectionDecoderFec;
		case EC_TYPE_ARQ:
			return new ErrorCorrectionDecoderARQ;
	}
	return 0;
}

ErrorCorrectionDecoder* CreateErrorCorrectionDecoder(const Dict* settings) {
	std::string type="fec";
	if(settings) {
		settings->Get("type",&type);
	}
	if(type=="fec") {
		return new ErrorCorrectionDecoderFec;
	}
	if(type=="arq") {
		return new ErrorCorrectionDecoderARQ;
	}
	FATAL("Unknow frame decode type %s. Valid types \"fec\" and \"arq\"",type.c_str());
	return 0;
}
void DestroyErrorCorrectionDecoder(ErrorCorrectionDecoder* bd) {
	delete bd;
}
