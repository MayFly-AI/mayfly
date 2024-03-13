#include <stdint.h>
#include <stdio.h>
#include "usb.h"

uint8_t GetSeqIndex() {
	static uint32_t index=0;
	return (uint8_t)(index++);
}

uint32_t SYS1Packet::ToString(char* buf,uint32_t bufBytesize,uint64_t time)const {
	if(bufBytesize<100)		// avoid overrun
		return 0;
	uint32_t pos=sprintf(buf,"SYS1Packet %d %d s=$%08x t=",m_byteSize,m_seqIndex,(int)m_status);
	int thi=(int)(m_time>>32);
	int tlo=(int)m_time;
	if(thi) {
		pos+=sprintf(buf+pos,"$%x%08x",thi,tlo);
	}else{
		pos+=sprintf(buf+pos,"$%x",tlo);
	}
	pos+=sprintf(buf+pos," data send=%d received=%d mesh=%d\r\n",(int)m_dataSendBytesize,(int)m_dataReceivedBytesize,(int)m_meshCount);
	if(m_status&(1<<PACKET_MAGIC))
		pos+=sprintf(buf+pos,"WARNING: Validate magic failed\r\n");
	if(m_status&(1<<PACKET_LEN))
		pos+=sprintf(buf+pos,"WARNING: Received packet size out of bound\r\n");
	if(m_status&(1<<RECEIVE_FAIL))
		pos+=sprintf(buf+pos,"WARNING: Receive failed\r\n");
	if(m_status&(1<<RECEIVE_TIMEOUT))
		pos+=sprintf(buf+pos,"WARNING: Timeout while waiting for response\r\n");
	if(m_status&(1<<TX_FAIL))
		pos+=sprintf(buf+pos,"WARNING: Transmit failed\r\n");
	if(m_status&(1<<IMU_RING_FULL))
		pos+=sprintf(buf+pos,"WARNING: IMU ring buffer full\r\n");
	if(m_status&(1<<RECEIVE_FAIL_DATA))
		pos+=sprintf(buf+pos,"WARNING: base reading data failed\r\n");

	return pos;
}
uint32_t IMU1Packet::ToString(char* buf,uint32_t bufBytesize,uint64_t time)const {
	if(bufBytesize<200)		// avoid overrun
		return 0;
	int t=(int)((int64_t)m_time-(int64_t)time);
	uint32_t pos=sprintf(buf,"IMU1Packet %d %d ",m_byteSize,m_seqIndex);
	pos+=sprintf(buf+pos,"t=%d d=%d ",t,(int)m_dt);
	pos+=sprintf(buf+pos,"a=%.2f,%.2f,%.2f ",m_acc[0],m_acc[1],m_acc[2]);
	pos+=sprintf(buf+pos,"r=%.2f,%.2f,%.2f ",m_rads[0],m_rads[1],m_rads[2]);
	pos+=sprintf(buf+pos,"m=%.2f,%.2f,%.2f ",m_mags[0],m_mags[1],m_mags[2]);
	pos+=sprintf(buf+pos,"c=%.2f,%.2f\r\n",m_temp[0],m_temp[1]);
	return pos;
}

uint32_t UWB1Packet::ToString(char* buf,uint32_t bufBytesize,uint64_t time)const {
	if(bufBytesize<100)		// avoid overrun
		return 0;
	int t=(int)((int64_t)m_time-(int64_t)time);
	uint32_t pos=sprintf(buf,"UWB1Packet %d %d ",m_byteSize,m_seqIndex);
	pos+=sprintf(buf+pos,"t=%d id=$%08x d=%f irq=%d neighbor ix=%d id=$%08x dist=%d\r\n",t,(int)m_id,m_distance,(int)m_irqCount,(int)m_neighborIndex,(int)m_neighborId,(int)m_neighborDist);
	return pos;
}

uint32_t Packet::ToString(char* buf,uint32_t bufBytesize,uint64_t time)const {
	uint32_t pos=0;
	switch(m_type) {
		case PT_IMU1: {
			pos=((IMU1Packet*)this)->ToString(buf,bufBytesize,time);
			break;
		}
		case PT_SYS1: {
			pos=((SYS1Packet*)this)->ToString(buf,bufBytesize,time);
			break;
		}
		case PT_UWB1: {
			pos=((UWB1Packet*)this)->ToString(buf,bufBytesize,time);
			break;
		}
		default: {
			if(m_byteSize-pos>128) {														// avoid overrun
				const char* name=PacketTypeToName((ePacketType)m_type);
				pos=sprintf(buf+pos,"unknown packet %s size %d seq %d\r\n",name,m_byteSize,m_seqIndex);
			}
			break;
		}
	};
	return pos;
}
