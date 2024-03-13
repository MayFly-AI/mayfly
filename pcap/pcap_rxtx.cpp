#include "pcap_rxtx.h"
#include "shared/misc.h"

#if ENABLE_LIBPCAP
#include "pcap.h"
#include <climits>
#include <cstring>
#include <atomic>
#include <chrono>
#include "shared/std_ext.h"
#include "3rdparty/radiotap/radiotap_iter.h"

static pcap_t* init_tx(const char* interface){
	char err[PCAP_ERRBUF_SIZE];
	pcap_t* ppcap=pcap_create(interface,err);
	if(!ppcap){
		uprintf("pcap_create %s failed: %s\n",interface,err);
		return 0;
	}
	if(pcap_set_snaplen(ppcap,4096)!=0){
		uprintf("pcap_set_snaplen failed\n");
		pcap_close(ppcap);
		return 0;
	}
	if(pcap_set_immediate_mode(ppcap,1)!=0){
		uprintf("pcap_set_immediate_mode failed: %s\n",pcap_geterr(ppcap));
		pcap_close(ppcap);
		return 0;
	}
	if(pcap_activate(ppcap)!=0){
		uprintf("pcap_activate failed: %s\n",pcap_geterr(ppcap));
		pcap_close(ppcap);
		return 0;
	}
	if(DLT_IEEE802_11_RADIO!=pcap_datalink(ppcap)){
		uprintf("unexpected datalink\n");
		pcap_close(ppcap);
		return 0;
	}
	return ppcap;
}

static pcap_t* init_rx(const char* interface,const char* bpf_str){
	char err[PCAP_ERRBUF_SIZE];
	struct bpf_program prog;

	pcap_t* ppcap=pcap_create(interface,err);
	if(!ppcap){
		uprintf("pcap_create %s failed: %s\n",interface,err);
		return 0;
	}
	if(pcap_set_snaplen(ppcap,4096)!=0){
		uprintf("pcap_set_snaplen failed\n");
		pcap_close(ppcap);
		return 0;
	}
	if(pcap_set_immediate_mode(ppcap,1)!=0){
		uprintf("pcap_set_immediate_mode failed: %s\n",pcap_geterr(ppcap));
		pcap_close(ppcap);
		return 0;
	}
	if(pcap_activate(ppcap)!=0){
		uprintf("pcap_activate failed: %s\n",pcap_geterr(ppcap));
		pcap_close(ppcap);
		return 0;
	}
	if(DLT_IEEE802_11_RADIO!=pcap_datalink(ppcap)){
		uprintf("unexpected datalink\n");
		pcap_close(ppcap);
		return 0;
	}
	if(pcap_compile(ppcap,&prog,bpf_str,1,0)!=0){
		uprintf("pcap_compile \"%s\" failed: %s\n",bpf_str,pcap_geterr(ppcap));
		pcap_close(ppcap);
		return 0;
	}
	if(pcap_setfilter(ppcap,&prog)!=0){
		uprintf("pcap_setfilter failed: %s\n",pcap_geterr(ppcap));
		pcap_freecode(&prog);
		pcap_close(ppcap);
		return 0;
	}
	pcap_freecode(&prog);
	return ppcap;
}

// Sender
class PCAPSenderImpl : public PCAPSender {
public:
	PCAPSenderImpl(const char* interface,uint8_t id);
	virtual ~PCAPSenderImpl() override;
	virtual bool Send(const void* data,int dataByteSize) override;
	virtual void SetId(uint8_t id) override {}
protected:
	pcap_t* m_pcap{nullptr};
	uint8_t m_id{0};
	size_t m_sent{0};
};

PCAPSenderImpl::PCAPSenderImpl(const char* interface,uint8_t id){
	m_pcap=init_tx(interface);
	if(!m_pcap){
		FATAL("Failed to initialize pcap sender");
	}
	m_id=id;
}
PCAPSenderImpl::~PCAPSenderImpl(){
	if(m_pcap){
		pcap_close(m_pcap);
	}
}
bool PCAPSenderImpl::Send(const void* data,int dataByteSize){
	if(!m_pcap){
		FATAL("Packet capture handle is not valid");
	}
	const uint8_t MCS_INDEX=3;
	const uint8_t MCS_KNOWN=IEEE80211_RADIOTAP_MCS_HAVE_MCS|IEEE80211_RADIOTAP_MCS_HAVE_BW|IEEE80211_RADIOTAP_MCS_HAVE_GI|IEEE80211_RADIOTAP_MCS_HAVE_FEC;
	const uint8_t MCS_FLAGS=IEEE80211_RADIOTAP_MCS_BW_40|IEEE80211_RADIOTAP_MCS_SGI|IEEE80211_RADIOTAP_MCS_FEC_LDPC;
	const uint8_t radiotap[]={
		0x00, 0x00,                     // version 0
		0x0d, 0x00,                     // length 13
		0x00, 0x80, 0x08, 0x00,         // added : 1<<15 TX_FLAGS | 1<<19 MCS
		0x08, 0x00,                     // TX_FLAGS data: NOACK
		MCS_KNOWN, MCS_FLAGS, MCS_INDEX // MCS data
	};

	uint8_t ieee80211[]={
		0x08, 0x01, 0x00, 0x00,              // data frame, not protected, from STA to DS via AP
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  // Addr 1 : broadcast
		0x5b, 0x4d, 0x46, 0x4c, 0x00, 0x00,  // Addr 2 : 0x?[37bf] is local multicast
		0x5b, 0x4d, 0x46, 0x4c, 0x00, 0x00,  // Addr 3 : --
		0x00, 0x00,                          // frag (4bit) seq (12bit)
	};

	uint8_t buffer[2048];
	ieee80211[10+5]=m_id;
	ieee80211[16+5]=m_id;

	int seq=m_sent<<4;
	ieee80211[22]=seq&0xff;
	ieee80211[23]=(seq>>8)&0xff;

	uint8_t* out=buffer;
	memcpy(out,radiotap,sizeof(radiotap));
	out+=sizeof(radiotap);
	memcpy(out,ieee80211,sizeof(ieee80211));
	out+=sizeof(ieee80211);
	memcpy(out,data,dataByteSize);
	out+=dataByteSize;

	int size=out-buffer;
	int count=pcap_inject(m_pcap,buffer,size);
	if(count!=size){
		pcap_perror(m_pcap,"inject failure");
		pcap_close(m_pcap);
		return false;
	}
	m_sent++;
	return true;
}

// Receiver
class PCAPReceiverImpl : public PCAPReceiver {
public:
	PCAPReceiverImpl(const char* interface,uint8_t id);
	virtual ~PCAPReceiverImpl() override;
	virtual void End() override;
	virtual bool WaitForData(recvcallback cb,void* arg) override;
	virtual void SetId(uint8_t id) override {}
protected:
	struct PCAPDispatchCallbackWrapper {
		recvcallback* cb;
		const void* arg;
		static void Callback(unsigned char* user,const struct pcap_pkthdr* hdr,const unsigned char* data);
	};
	pcap_t* m_pcap{nullptr};
	uint8_t m_id{0};
	std::atomic<bool> m_close{false};
};
PCAPReceiverImpl::PCAPReceiverImpl(const char* interface,uint8_t id){
	std::string program=stdx::format_string("ether[10:4]==0x5b4d464c && ether[14:2]==0x00%02x",id);
	m_pcap=init_rx(interface,program.c_str());
	if(!m_pcap){
		FATAL("Failed to initialize pcap receiver");
	}
	m_id=id;
}
PCAPReceiverImpl::~PCAPReceiverImpl(){
	if(m_pcap){
		pcap_close(m_pcap);
	}
}
void PCAPReceiverImpl::End(){
	if(m_pcap){
		pcap_breakloop(m_pcap);
	}
}
bool PCAPReceiverImpl::WaitForData(recvcallback cb,void* arg){
	if(!m_pcap){
		FATAL("Packet capture handle is not valid");
	}
	m_close=false;
	PCAPDispatchCallbackWrapper cb_wrap={&cb,arg};
	while(!m_close){
		int count=pcap_dispatch(m_pcap,1,PCAPDispatchCallbackWrapper::Callback,(unsigned char*)&cb_wrap);
		if(count==0){
			continue;
		}
		if(count<0){
			if(count==PCAP_ERROR_BREAK){
				uprintf("PCAPReceiverImpl terminating\n");
			} else
			if(count==PCAP_ERROR){
				FATAL("Failed reading next packet: %s",pcap_geterr(m_pcap));
			}
			m_close=1;
			continue;
		}
	}
	return false;
}

void PCAPReceiverImpl::PCAPDispatchCallbackWrapper::Callback(unsigned char* user,const struct pcap_pkthdr* h,const unsigned char* data){
	uint64_t cap_time=1000000UL*h->ts.tv_sec+h->ts.tv_usec;

	int ant_idx=0;
	const int MAX_ANTENNA_COUNT=4;
	//uint8_t ant_num[MAX_ANTENNA_COUNT]{UCHAR_MAX};
	int8_t ant_dBm[MAX_ANTENNA_COUNT]{SCHAR_MIN};
	uint8_t flags=0;

	struct ieee80211_radiotap_iterator it;
	int err=ieee80211_radiotap_iterator_init(&it,(ieee80211_radiotap_header*)data,h->caplen,0);
	if(err){
		FATAL("Radiotap iterator failed: %d\n",err);
	}

	while(!(err=ieee80211_radiotap_iterator_next(&it))){
		switch(it.this_arg_index){
			case IEEE80211_RADIOTAP_ANTENNA: {
				if(ant_idx<MAX_ANTENNA_COUNT){
					//ant_num[ant_idx]=*(uint8_t*)(it.this_arg);
					ant_idx+=1;
				}
			} break;
			case IEEE80211_RADIOTAP_DBM_ANTSIGNAL: {
				if(ant_idx<MAX_ANTENNA_COUNT){
					ant_dBm[ant_idx]=*(int8_t*)(it.this_arg);
				}
			} break;
			case IEEE80211_RADIOTAP_FLAGS: {
				flags=*(uint8_t*)(it.this_arg);
			} break;
			default: {
				//uprintf("unknown type: %d\n",it.this_arg_index);
			} break;
		}
	}
	if (err!=-ENOENT){
		FATAL("Bad radiotap data: %d",err);
	}
	if(flags&IEEE80211_RADIOTAP_F_BADFCS){
		uprintf("WARNING: packet with bad FSC\n");
	}

	// The size of the 3 MAC address ieee80211 header including sequence control
	const int ieee80211_size=24;
	const uint8_t* payload=data+it._max_length+ieee80211_size;
	int payload_size=h->caplen-it._max_length-ieee80211_size;

	if(flags&IEEE80211_RADIOTAP_F_FCS){
		payload_size-=4;
	}
	//for(int i=0;i<ant_idx;++i){
	//	uprintf("ant %d,dBm=%d\n",ant_num[i],ant_dBm[i]);
	//}

	ReceivedDataInfo ri;
	memset(&ri,0,sizeof(ri));
	ri.m_time=cap_time;
	ri.m_rssi=ant_dBm[0];

	PCAPDispatchCallbackWrapper* cb_wrap=(PCAPDispatchCallbackWrapper*)user;
	(*cb_wrap->cb)(payload,payload_size,ri,cb_wrap->arg);
}

// Create/Destroy
PCAPSender* CreatePCAPSender(const char* interfaceName,uint8_t id){
	return new PCAPSenderImpl(interfaceName,id);
}
void DestroyPCAPSender(PCAPSender* sender){
	delete sender;
}
PCAPReceiver* CreatePCAPReceiver(const char* interfaceName,uint8_t id){
	return new PCAPReceiverImpl(interfaceName,id);
}
void DestroyPCAPReceiver(PCAPReceiver* receiver){
	if(receiver){
		receiver->End();
		delete receiver;
	}
}

#else // ENABLE_LIBPCAP

PCAPSender* CreatePCAPSender(const char* interfaceName,uint8_t id){
	FATAL("PCAPSender not available: missing pcap");
	return 0;
}
void DestroyPCAPSender(PCAPSender* sender){}
PCAPReceiver* CreatePCAPReceiver(const char* interfaceName,uint8_t id){
	FATAL("PCAPReceiver not available: missing pcap");
	return 0;
}
void DestroyPCAPReceiver(PCAPReceiver* receiver){}
#endif
