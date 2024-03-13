#pragma once

#include <stdio.h>
#include <stdint.h>
#include <mutex>
#include <deque>

#undef max
#undef min

#include <algorithm>

//#include "shared/dict.h"

#define HISTORY_AGE 20000000		//microseconds, to save memory - must be more than status time window

class History {
	public:
		struct TimeEntry {
			uint64_t m_timeRound;
			double m_value;
		};
		void SetMaxTime(uint32_t maxTime){m_maxTime=maxTime;}
		uint32_t GetMaxTime()const{return m_maxTime;}
		std::mutex m_lock;
		std::deque<TimeEntry> m_queue;
		uint32_t m_maxTime=0;
		void Get(std::vector<double>* datax,std::vector<double>* datay,double scalex,double scaley,uint64_t timeMicrosecond) {
			std::scoped_lock sl(m_lock);
			while(m_maxTime && m_queue.size()) {
				if(m_queue.front().m_timeRound>timeMicrosecond-m_maxTime)
					break;
				m_queue.pop_front();
			}
			datax->clear();
			datay->clear();
			datax->reserve(m_queue.size());
			datay->reserve(m_queue.size());
			for(int i=0;i<(int)m_queue.size();i++) {
				datax->push_back((double)m_queue[i].m_timeRound*scalex);
				datay->push_back(m_queue[i].m_value*scaley);
			}
		}
		void GetMbps(std::vector<double>* datax,std::vector<double>* datay,uint64_t timeMicrosecond) {
			Get(datax,datay,1.0/1000.0,8.0/1000000.0,timeMicrosecond);
		}
		void Get(std::vector<double>* datax,std::vector<double>* datay,uint64_t timeMicrosecond) {
			Get(datax,datay,1.0/1000.0,1,timeMicrosecond);
		}
		void Register(uint64_t timeMicrosecond,double value) {
			std::scoped_lock sl(m_lock);
			while(m_maxTime && m_queue.size()) {
				if(m_queue.front().m_timeRound>timeMicrosecond-m_maxTime)
					break;
				m_queue.pop_front();
			}
			while(m_queue.size()>256)
				m_queue.pop_front();
			m_queue.push_back({timeMicrosecond,value});
		}
		void RegisterAccumulate(uint64_t timeMicrosecond,double value) {
			std::scoped_lock sl(m_lock);
			while(m_maxTime && m_queue.size()) {
				if(m_queue.front().m_timeRound>timeMicrosecond-m_maxTime)
					break;
				m_queue.pop_front();
			}
			while(m_queue.size()>256)
				m_queue.pop_front();
			uint64_t timeRound=timeMicrosecond-(timeMicrosecond%1000000L);
			if(m_queue.size() && m_queue.at(m_queue.size()-1).m_timeRound==timeRound) {
				m_queue.at(m_queue.size()-1).m_value+=value;
			}else{
				m_queue.push_back({timeRound,value});
			}
		}
		void RegisterMax(uint64_t timeMicrosecond,double value) {
			std::scoped_lock sl(m_lock);
			while(m_maxTime && m_queue.size()) {
				if(m_queue.front().m_timeRound>timeMicrosecond-m_maxTime)
					break;
				m_queue.pop_front();
			}
			while(m_queue.size()>256)
				m_queue.pop_front();
			uint64_t timeRound=timeMicrosecond-(timeMicrosecond%1000000L);
			if(m_queue.size() && m_queue.at(m_queue.size()-1).m_timeRound==timeRound) {
				m_queue.at(m_queue.size()-1).m_value=std::max(m_queue.at(m_queue.size()-1).m_value,value);
			}else{
				m_queue.push_back({timeRound,value});
			}
		}
};

struct AdaptorInfo {
	std::string m_ip;
	std::string m_name;
	History m_rssi;
};
