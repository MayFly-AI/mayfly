
#include <stdio.h>
#include <chrono>
#include <map>
#include <deque>
#include <set>
#include <thread>
#include <algorithm>

#include "shared/file.h"
#include "shared/net.h"
#include "shared/crc32.h"
#include "shared/dict.h"
#include "packet.h"

bool PacketHeaderBase::SanityCheck(int packetBytesize)const {
	if(m_magic!=PacketHeader::Magic()) {
		FATAL("PacketHeaderBase::SanityCheck packet type %d has invalid magic.",m_type);
		return false;
	}
	if(packetBytesize<(int)sizeof(PacketHeaderBase)) {
		uprintf("PacketHeaderBase::SanityCheck Packet type %d size less than header. Ignoring packet\n",m_type);
		return false;
	}
	if(m_type >= PacketHeaderBase::LAST_TYPE) {
		uprintf("PacketHeaderBase::SanityCheck packet type %d out of range. Ignoring packet\n",m_type);
	}
	return true;
}

bool PacketHeader::SanityCheck(int packetBytesize)const {
	if(m_magic!=PacketHeader::Magic()) {
		FATAL("PacketHeader::SanityCheck packet type %d has invalid magic.",m_type);
		return false;
	}
	if(packetBytesize!=(int)m_headerBytesize+(int)m_dataBytesize) {
		FATAL("PacketHeader::SanityCheck packet type %d invalid. packetBytesize %d!=%d",m_type,packetBytesize,(int)m_headerBytesize+(int)m_dataBytesize);
		return false;
	}
	if(m_type >= PacketHeaderBase::LAST_TYPE) {
		uprintf("PacketHeader::SanityCheck packet type %d out of range. Ignoring packet\n",m_type);
	}
	return true;
}
