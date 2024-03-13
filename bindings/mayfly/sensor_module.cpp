#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <thread>
#include <queue>
#include <set>
#include <unordered_map>
#include "shared/misc.h"
#include "shared/file.h"
#include "dlpack/dlpack.h"
#include "memory/tensor.h"
#include "core/app.h"
#include "core/transfer.h"
#include "video/decoder.h"
#include "dicttopython.h"
#include <exception>

//#define ENABLE_SENSOR_COMMAND_QUEUE_API

namespace py=pybind11;
using namespace std::chrono_literals;
using namespace pybind11::literals;
const char* dltensor_name="dltensor";

struct ExpTensor {
	typedef std::shared_ptr<ExpTensor> Ptr;

	Tensor m_decoded;
	bool m_exported{false};
	static void delete_dltensor(DLManagedTensor* o);
	static std::unordered_map<DLManagedTensor*,ExpTensor::Ptr> s_exported;
};
std::unordered_map<DLManagedTensor*,ExpTensor::Ptr> ExpTensor::s_exported;


ExpTensor::Ptr CreateExpTensor(Tensor* move_from_buf){
	ExpTensor* t=new ExpTensor();
	t->m_decoded=std::move(*move_from_buf);
	t->m_exported=false;
	return std::shared_ptr<ExpTensor>(t);
}

void destruct_cap(PyObject* self){
	if(PyCapsule_IsValid(self,"used_dltensor")){
		//uprintf("used dl_tensor skip\n");
		return;
	}
	DLManagedTensor* dlmt=(DLManagedTensor*)PyCapsule_GetPointer(self,dltensor_name);
	if(dlmt && dlmt->deleter){
		//uprintf("delete dangling __from_dlpack__\n");
		dlmt->deleter(dlmt);
	}
}

void ExpTensor::delete_dltensor(DLManagedTensor* o){
	s_exported.erase(o);
	//uprintf("delete_dltensor: tensor allocations: %zu\n",ExpTensor::s_exported.size());
	delete[] o->dl_tensor.shape;
	delete[] o->dl_tensor.strides;
	delete o;
}

py::capsule* export_dltensor(ExpTensor::Ptr tensor,py::object* stream){
	if(tensor->m_exported){
		throw std::invalid_argument("dlmt can only be exported once");
	}
	tensor->m_exported=true;
	DLManagedTensor* dlmt=new DLManagedTensor();
	dlmt->dl_tensor.data=tensor->m_decoded.m_data;

	bool cuda=tensor->m_decoded.m_type==Tensor::AllocatorType::ALLOC_CUDA;
	if(cuda){
		dlmt->dl_tensor.device={kDLCUDA,0};
	}else{
		dlmt->dl_tensor.device={kDLCPU,0};
	}

	if(tensor->m_decoded.m_elementSize==1){
		dlmt->dl_tensor.dtype={kDLUInt,8,1};
	}else
	if(tensor->m_decoded.m_elementSize==4){
		dlmt->dl_tensor.dtype={kDLFloat,32,1};
	}else{
		FATAL("Unhandled element size: %d",tensor->m_decoded.m_elementSize);
	}


	int dim=tensor->m_decoded.m_dimensions;
	dlmt->dl_tensor.ndim=dim;
	dlmt->dl_tensor.shape=new int64_t[dim];
	for(int i=0;i<dim;++i)
	dlmt->dl_tensor.shape[i]=tensor->m_decoded.m_shape[i];
	dlmt->dl_tensor.strides=nullptr;
	dlmt->dl_tensor.byte_offset=0; // hack ?
	dlmt->deleter=ExpTensor::delete_dltensor;
	dlmt->manager_ctx=0;
	ExpTensor::s_exported[dlmt]=tensor;
	//uprintf("export_dltensor: tensor allocations: %zu\n",ExpTensor::s_exported.size());

	py::capsule* cap=new py::capsule(dlmt,dltensor_name,destruct_cap);
	return cap;
}

std::tuple<int,int> dlpack_device(const ExpTensor& tensor){
	if(tensor.m_decoded.m_type==Tensor::AllocatorType::ALLOC_CUDA)
		return {kDLCUDA,0};
	if(tensor.m_decoded.m_type==Tensor::AllocatorType::ALLOC_CPU)
		return {kDLCPU,0};
	FATAL("No data to export");
	return {0,0};
}

struct Resource {
	typedef std::shared_ptr<Resource> Ptr;
	Resource(){}
	virtual ~Resource(){}
	virtual py::object ToPython()=0;
	uint8_t m_streamId{255};
};

struct DataAugmentedResource : Resource {
	DataAugmentedResource(){}
	virtual ~DataAugmentedResource(){}
	py::object ToPython() override {
		py::dict d=DictToPython(m_dict);
		size_t count=m_tensors.size();
		if(count){
			py::list tensorData;
			for(size_t i=0;i<count;++i){
				py::dict frmdict("frame"_a=m_tensors[i]);
				tensorData.append(m_tensors[i]);
			}
			d["$data"]=tensorData;
		}
		return d;
	}
	Dict m_dict;
	std::vector<ExpTensor::Ptr> m_tensors;
};

struct ResourceQueue;
struct CommandQueue;

class AppSensor : public App {
	public:
		AppSensor();
		virtual ~AppSensor();
		void MainLoop() override;
		void Start(const std::string& configFilename);

		ResourceQueue* CreateResourceQueue(const std::vector<int>& streamIds,size_t size);
#ifdef ENABLE_SENSOR_COMMAND_QUEUE_API
		CommandQueue* CreateCommandQueue();
		void DetachCommandQueue(CommandQueue* q);
#endif

		std::thread m_runThread;
		std::mutex m_mtx;
		std::condition_variable m_cond_var;
		std::vector<ResourceQueue*> m_listeners;
#ifdef ENABLE_SENSOR_COMMAND_QUEUE_API
		std::vector<CommandQueue*> m_commanders;
#endif
};

struct ResourceQueue {
	ResourceQueue(const std::vector<int>& streamIds,size_t maxSize,AppSensor* app)
	: m_maxSize(maxSize)
	,m_streamIds(streamIds.begin(),streamIds.end())
	,m_owner(app){
	}

	bool Push(Resource::Ptr& r){
		// m_streamIds are not mutable - no lock necessary
		if(m_streamIds.find(r->m_streamId)==m_streamIds.end()){
			return false;
		}
		{
			std::lock_guard<std::mutex> lock(m_mtx);
			if(m_queue.size()>=m_maxSize)
				m_queue.pop();
			m_queue.push(r);
		}
		m_cond_var.notify_one();
		return true;
	}
	py::object Poll(){
		if(!m_owner){
			throw std::runtime_error("ResourceQueue is not attached to application");
		}
		std::lock_guard<std::mutex> lock(m_mtx);
		if(m_queue.empty()){
			return py::none();
		}
		Resource::Ptr res=m_queue.front();
		auto pythonResult=res->ToPython();
		m_queue.pop();
		return pythonResult;
	}
	py::object Wait(int timeout){
		if(!m_owner){
			throw std::runtime_error("ResourceQueue is not attached to application");
		}
		std::unique_lock<std::mutex> lock(m_mtx);
		if(m_queue.empty()){
			if(timeout>0){
				m_cond_var.wait_for(lock,std::chrono::microseconds(timeout));
			}else{
				m_cond_var.wait(lock);
			}
		}
		if(m_queue.empty()) return py::none();
		Resource::Ptr r=m_queue.front();
		auto pythonResult=r->ToPython();
		m_queue.pop();
		return pythonResult;
	}

	void Flush(){
		uprintf("Flushing ResourceQueue\n");
		std::lock_guard<std::mutex> lock(m_mtx);
		m_queue={};
	}

	std::queue<Resource::Ptr> m_queue;

	size_t m_maxSize;
	const std::set<uint8_t> m_streamIds;
	std::mutex m_mtx;
	std::condition_variable m_cond_var;
	AppSensor* m_owner;
};


#ifdef ENABLE_SENSOR_COMMAND_QUEUE_API
struct CommandQueue {
	~CommandQueue(){
		if(m_owner){
			m_owner->DetachCommandQueue(this);
		}
	}
	void Push(const char* msg){
		if(!m_owner){
			throw std::runtime_error("CommandQueue is not attached to application");
		}
		m_commands.emplace_back(msg);
	}
	std::vector<std::string> m_commands;
	AppSensor* m_owner;
};
#endif

AppSensor::AppSensor(){
	uprintf("AppSensor::AppSensor\n");
}

AppSensor::~AppSensor(){
	uprintf("AppSensor::~AppSensor\n");
	m_runThread.join();
	uprintf("joined\n");

	for(auto l:m_listeners){
		l->m_owner=nullptr;
	}
#ifdef ENABLE_SENSOR_COMMAND_QUEUE_API
	for(auto c:m_commanders){
		c->m_owner=nullptr;
	}
#endif
}

Resource::Ptr CreateDataAugmentedResource(uint8_t streamId,const Dict& dict,std::vector<Tensor>& tensors){
	auto res=new DataAugmentedResource();
	res->m_streamId=streamId;
	res->m_dict=dict;
	for(auto& t:tensors){
		res->m_tensors.push_back(CreateExpTensor(&t));
	}
	return std::shared_ptr<DataAugmentedResource>(res);
}

void AppSensor::MainLoop(){
	SetFrameDecodedCallback(this,
		[](Dict& frame,const void* data,int dataBytesize,void* arg){
			std::string s((const char*)data,(const char*)data+dataBytesize);
			uprintf("JSON dataBytesize: %d\n%s\n",dataBytesize,s.c_str());
		},
		[this](const char* clientId,uint8_t streamId,Dict& streamInfo,std::vector<Tensor>& tensors,void* arg){
			//uprintf("message from \"%s\",stream: %d,# tensors: %zu\n",clientId,streamId,tensors.size());
			//streamInfo.Dump();
			size_t validTensors=std::count_if(tensors.begin(),tensors.end(),[](auto&t){return t.m_data;});
			if(validTensors!=tensors.size()){
				//uprintf("Received invalid tensors! (%zu/%zu)\n",tensors.size()-validTensors,tensors.size());
				return;
			}

			Resource::Ptr resource=CreateDataAugmentedResource(streamId,streamInfo,tensors);
			for(auto&e:m_listeners){
				e->Push(resource);
			}
		}
	);

#ifdef ENABLE_SENSOR_COMMAND_QUEUE_API
	for(ActiveService* as:m_activeServices){
		if(as->Type()!=Service::VIDEO_CLIENT)
			continue;
		SensorClient* vc=(SensorClient*)as->m_service;
		uprintf("set AddLastMessageCallback for client %s\n",vc->Id().c_str());
		printf("MESSAGE: client id= %s\n", vc->Id().c_str());
		vc->AddLastMessageCallback(this,
			[this](int frameIndex,uint8_t streamId,uint8_t dataSourceType,NetTransfer* transfer,void* arg){
			//uprintf("got last message: %d from %s:%lu\n",frameIndex,transfer->Host().c_str());
				for(auto c:m_commanders){
					for(auto& cmd: c->m_commands){
						if(cmd!="master_clock")
							continue;
						ClockMasterTimeSendHeader hd;
						hd.m_streamId=streamId;
						hd.m_dataSourceType=dataSourceType;
						hd.m_masterClockMicroseconds=GetTimeEpochMicroseconds();
						transfer->SendToHost(&hd,sizeof(hd));
					}
					c->m_commands.clear();
				}
			}
		);
	}
#endif

	App::MainLoop();

	ClearFrameDecodedCallback();
	for(auto l:m_listeners){
		l->Flush();
	}
}

void AppSensor::Start(const std::string& configFilename){
	if(DirectoryExists(GetExecutableDir()+"/data")){
		AddFilePathRemap("$(DATA)",GetExecutableDir()+"/data");
	}else{
#ifdef CMAKE_SOURCE_DIR
		AddFilePathRemap("$(DATA)",std::string(CMAKE_SOURCE_DIR)+"/data");
#else
		AddFilePathRemap("$(DATA)",GetExecutableDir()+"/data");
#endif
	}
	AddFilePathRemap("$(EXE)",GetExecutableDir());
	AddFilePathRemap("$(HOME)",getenv("HOME"));
	std::string cfgFile=configFilename.empty()?std::string("$(HOME)/config.json"):configFilename;
	m_runThread=std::thread([this,cfgFile]{
		Run(cfgFile);
	});
}

ResourceQueue* AppSensor::CreateResourceQueue(const std::vector<int>& streamIds,size_t size){
	auto q=new ResourceQueue(streamIds,size,this);
	m_listeners.push_back(q);
	return q;
}

#ifdef ENABLE_SENSOR_COMMAND_QUEUE_API
CommandQueue* AppSensor::CreateCommandQueue(){
	auto q=new CommandQueue;
	q->m_owner=this;
	m_commanders.push_back(q);
	return q;
}

void AppSensor::DetachCommandQueue(CommandQueue* q){
	auto found=std::find(m_commanders.begin(),m_commanders.end(),q);
	ASSERT(found!=m_commanders.end(),"CommandQueue not found");
	m_commanders.erase(found);
}
#endif

PYBIND11_MODULE(sensor,m){
	py::class_<ExpTensor,std::shared_ptr<ExpTensor>>(m,"Tensor")
		.def("__repr__",[](const ExpTensor& t){
			char buf[512];
			bool cuda=t.m_decoded.m_type==Tensor::AllocatorType::ALLOC_CUDA;
			snprintf(buf,sizeof(buf),"Tensor([%dx%dx%d], exported: %s, cuda: %s)",
					t.m_decoded.m_shape[0],t.m_decoded.m_shape[1],t.m_decoded.m_shape[2],
					t.m_exported?"True":"False",cuda?"True":"False");
			return std::string(buf);
		})
		.def("__dlpack_device__",&dlpack_device)
		.def("__dlpack__",&export_dltensor,
			py::call_guard<py::gil_scoped_release>(),
			py::return_value_policy::take_ownership,
			py::arg("stream")=nullptr)
	;

	py::class_<ResourceQueue>(m,"ResourceQueue")
		.def("poll",&ResourceQueue::Poll)
		.def("wait",&ResourceQueue::Wait,
			py::arg("timeout")=0)
	;

#ifdef ENABLE_SENSOR_COMMAND_QUEUE_API
	py::class_<CommandQueue>(m,"CommandQueue")
		.def("push",&CommandQueue::Push)
	;
#endif

	py::class_<AppSensor>(m,"AppSensor")
		.def(py::init<>())
		.def("__repr__",[](const AppSensor& c){
			char buf[512];
			snprintf(buf,sizeof(buf),"size: %zu\n",c.m_listeners.size());
			return std::string(buf);
		})
		.def("start",&AppSensor::Start,
			py::arg("config")=""
		)
		.def("stop",[](AppSensor& c){
			uprintf("stop: exported tensors %zu\n",ExpTensor::s_exported.size());
			App::PostEvent(App::EV_EXIT);
		})
		.def("create_event_queue",
			&AppSensor::CreateResourceQueue
		)
#ifdef ENABLE_SENSOR_COMMAND_QUEUE_API
		.def("create_command_queue",
			&AppSensor::CreateCommandQueue
		)
#endif
	;
}
