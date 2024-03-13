#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include "../posesensor/iserial.h"
#include "../posesensor/usb.h"

namespace py=pybind11;


bool VerifyPacket(const uint8_t* ptr,size_t len) {
	if(len<sizeof(Packet))
		return false;
	const Packet& p=*reinterpret_cast<const Packet*>(ptr);
	if(p.m_type==PT_IMU1 && p.m_byteSize==sizeof(IMU1Packet))
		return true;
	if(p.m_type==PT_SYS1 && p.m_byteSize==sizeof(SYS1Packet))
		return true;
	if(p.m_type==PT_UWB1 && p.m_byteSize==sizeof(UWB1Packet))
		return true;
	return false;
}

py::object ToPython(const Packet& p) {
	if(p.m_type==PT_UWB1 && p.m_byteSize==sizeof(UWB1Packet)) {
		const UWB1Packet* uwb=reinterpret_cast<const UWB1Packet*>(&p);
		py::dict dct;
		dct["type"]="uwb";
		dct["time"]=uwb->m_time;
		dct["id"]=uwb->m_id;
		dct["dist"]=uwb->m_distance;

		if(uwb->m_neighborId) {
			py::dict neighborDistInfo;
			neighborDistInfo["id"]=uwb->m_neighborId;
			neighborDistInfo["dist"]=0.01f*uwb->m_neighborDist; // cm -> m
			dct["neighbor"]=neighborDistInfo;
		}
		return dct;
	}
	if(p.m_type==PT_IMU1 && p.m_byteSize==sizeof(IMU1Packet)) {
		const IMU1Packet* imu=reinterpret_cast<const IMU1Packet*>(&p);
		py::dict dct;
		dct["type"]="imu";
		dct["time"]=imu->m_time;
		dct["dt"]=imu->m_dt;
		dct["acc"]=std::vector({imu->m_acc[0],imu->m_acc[1],imu->m_acc[2]});
		dct["gyr"]=std::vector({imu->m_rads[0],imu->m_rads[1],imu->m_rads[2]});
		dct["mag"]=std::vector({imu->m_mags[0],imu->m_mags[1],imu->m_mags[2]});
		dct["tmp"]=std::vector({imu->m_temp[0],imu->m_temp[1]});
		return dct;
	}
	if(p.m_type==PT_SYS1 && p.m_byteSize==sizeof(SYS1Packet)) {
		const SYS1Packet* sys=reinterpret_cast<const SYS1Packet*>(&p);

		py::dict dct;
		dct["type"]="sys";
		dct["time"]=sys->m_time;
		if(sys->m_status) {
			char sbuf[4096];
			int slen=sys->ToString(sbuf,sizeof(sbuf),0);
			dct["message"]=std::string(sbuf,slen-2);
		}
		return dct;
	}
	return py::none();
}

class SensorDataQueue {
public:
		SensorDataQueue()=default;
		virtual ~SensorDataQueue()=default;
		void Push(const uint8_t* data,int len) {
			if(!data||!len) {
				return;
			}
			int overflow=0;
			{
				std::lock_guard<std::mutex> lock(m_mtx);
				while(m_queue.size()>=m_maxSize) {
					m_queue.pop();
					++overflow;
				}
				m_queue.push({data,data+len});
			}
			if(overflow) {
				printf("queue overlow (%d element%s)\n",overflow,overflow>1?"s":"");
			}
		}
		py::object Wait(int timeout_us) {
			std::unique_lock<std::mutex> lock(m_mtx);
			if(m_queue.empty()) {
				if(timeout_us>0) {
					m_cond.wait_for(lock,std::chrono::microseconds(timeout_us));
				} else
				{
					m_cond.wait(lock);
				}
			}
			if(m_queue.empty()) {
				return py::none();
			}

			std::vector<uint8_t>& front=m_queue.front();
			if(front.size() < sizeof(Packet)) {
				printf("bad size: %zu\n",front.size());
				m_queue.pop();
				return py::none();
			}
			const Packet* pkt=(const Packet*)front.data();
			py::object obj=ToPython(*pkt);
			m_queue.pop();
			return obj;
		}
		const size_t m_maxSize=64*1024;
		std::queue<std::vector<uint8_t>> m_queue;
		std::condition_variable m_cond;
		std::mutex m_mtx;
};

class Connection {
public:
	Connection() {
	}
	virtual ~Connection() {
		DestroySerialConnection(m_serial);
	}

	py::object Wait(int timeout_ms) {
		return m_sensorDataQueue.Wait(timeout_ms);
	}

	int Write(const char* str) {
		return m_serial->Write((const uint8_t*)str,strlen(str));
	}

	void Open(const char* device) {
		m_serial=CreateSerialConnection(device);
		m_serial->SetCallback([this](const uint8_t* new_data,int len,void* arg){
			m_buffer.insert(m_buffer.end(),new_data,new_data+len);
			// at least 2 bytes necessary to determine type and size
			while(m_buffer.size()>1) {
				if(m_buffer[0]==PT_IMU1 && m_buffer[1]==sizeof(IMU1Packet)) {
					if (m_buffer.size() < sizeof(IMU1Packet)) return;
					m_sensorDataQueue.Push(m_buffer.data(),sizeof(IMU1Packet));
					m_buffer.erase(m_buffer.begin(),m_buffer.begin()+sizeof(IMU1Packet));
				}
				else if(m_buffer[0]==PT_SYS1 && m_buffer[1]==sizeof(SYS1Packet)) {
					if (m_buffer.size() < sizeof(SYS1Packet)) return;
					m_sensorDataQueue.Push(m_buffer.data(),sizeof(SYS1Packet));
					m_buffer.erase(m_buffer.begin(),m_buffer.begin()+sizeof(SYS1Packet));
				}
				else if(m_buffer[0]==PT_UWB1 && m_buffer[1]==sizeof(UWB1Packet)) {
					if (m_buffer.size() < sizeof(UWB1Packet)) return;
					m_sensorDataQueue.Push(m_buffer.data(),sizeof(UWB1Packet));
					m_buffer.erase(m_buffer.begin(),m_buffer.begin()+sizeof(UWB1Packet));
				}
				else {
					// assuming newline terminated string message
					char sbuf[256];
					size_t i=0,k=0;
					for(;i<m_buffer.size() && m_buffer[i]!='\n'; ++i) {
						if(k<sizeof(sbuf)-1) {
							sbuf[k++]=m_buffer[i];
						}
					}
					if(i==m_buffer.size()) {
						return;
					}
					sbuf[k]=0;
					printf("GOT STRING MESSAGE: %s\n",sbuf);
					m_buffer.erase(m_buffer.begin(),m_buffer.begin()+i+1);
				}
			}
		},nullptr);
		m_serial->Run();
	}
	void Close() {
		if(m_serial) {
			m_serial->Stop();
		}
	}
	SensorDataQueue m_sensorDataQueue;
	ISerialConnection* m_serial{nullptr};
	std::vector<uint8_t> m_buffer;
};

PYBIND11_MODULE(posesensor,m) {
	py::class_<Connection>(m,"Connection")
		.def(py::init<>())
		.def("__repr__",[](const Connection& c) {
			const char* buf="[[Connection]]";
			return std::string(buf);
		})
		.def("open",
			&Connection::Open,
			py::arg("device")=""
		)
		.def("wait",
			&Connection::Wait,
			py::arg("timeout")=100
		)
		.def("write",
			&Connection::Write
		)
		.def("close",[](Connection& c) {
			c.Close();
		})
	;
}
