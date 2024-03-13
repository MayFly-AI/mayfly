#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>

#include "shared/misc.h"
#include "shared/std_ext.h"

#ifdef _WIN32
void SetSelfAffinityMask(uint8_t cpuMask) {
	FATAL("AffinityMask not supported on windows");
}

#endif

#ifdef __linux__
#include <pthread.h>

void SetSelfAffinityMask(uint8_t cpuMask){
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	for(int i=0;i!=4;i++) {
		if(cpuMask&(1<<i))
			CPU_SET(i,&cpuset);
	}
	pthread_t thread=pthread_self();
	int s=pthread_setaffinity_np(thread,sizeof(cpuset),&cpuset);
	if(s!=0)
		FATAL("pthread_setaffinity_np %d",s);
}

//Test from https://bytefreaks.net/programming-2/cc-set-affinity-to-threads-example-code

struct thread_info {
	pthread_t thread_id;// ID returned by pthread_create()
	int core_id;// Core ID we want this pthread to set its affinity to
};

#define SUCCESS_MSG "Successfully set thread %lu to affinity to CPU %d\n"
#define FAILURE_MSG "Failed to set thread %lu to affinity to CPU %d\n"

void* thread_camper(void *arg) {
 	struct thread_info *thread_info=(struct thread_info *)arg;

	const pthread_t pid=pthread_self();
	const int core_id=thread_info->core_id;

	// cpu_set_t: This data set is a bitset where each bit represents a CPU.
	cpu_set_t cpuset;
	// CPU_ZERO: This macro initializes the CPU set set to be the empty set.
	CPU_ZERO(&cpuset);
	// CPU_SET: This macro adds cpu to the CPU set set.
	CPU_SET(core_id,&cpuset);

	// pthread_setaffinity_np: The pthread_setaffinity_np() function sets the CPU affinity mask of the thread thread to the CPU set pointed to by cpuset. If the call is successful, and the thread is not currently running on one of the CPUs in cpuset, then it is migrated to one of those CPUs.
	const int set_result=pthread_setaffinity_np(pid,sizeof(cpu_set_t),&cpuset);
	if(set_result!=0) {
		FATAL("pthread_setaffinity_np %d",set_result);
	}

	// Check what is the actual affinity mask that was assigned to the thread.
	// pthread_getaffinity_np: The pthread_getaffinity_np() function returns the CPU affinity mask of the thread thread in the buffer pointed to by cpuset.
	const int get_affinity=pthread_getaffinity_np(pid,sizeof(cpu_set_t),&cpuset);
	if(get_affinity!=0) {
		FATAL("pthread_getaffinity_np %d",get_affinity);
	}
	char *buffer;
	// CPU_ISSET: This macro returns a nonzero value (true) if cpu is a member of the CPU set set, and zero (false) otherwise.
	if(CPU_ISSET(core_id,&cpuset)) {
		const size_t needed=snprintf(NULL,0,SUCCESS_MSG,pid,core_id);
		buffer=(char *)malloc(needed);
		snprintf(buffer,needed,SUCCESS_MSG,pid,core_id);
	}else{
		const size_t needed=snprintf(NULL,0,FAILURE_MSG,pid,core_id);
		buffer=(char *)malloc(needed);
		snprintf(buffer,needed,FAILURE_MSG,pid,core_id);
	}
	return buffer;
}

// #define handle_error_en(en, msg) do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

void TestPThread() {
	// Initialize thread creation attributes
	pthread_attr_t attr;
	const int attr_init_result=pthread_attr_init(&attr);
	if(attr_init_result!=0) {
		FATAL("pthread_attr_init %d",attr_init_result);
	}
 	// We will set the stack size limit to is 1 MB (0x100000 bytes)
	const int stack_size=0x100000;
	const int setstacksize_result=pthread_attr_setstacksize(&attr,stack_size);
	if(setstacksize_result!=0) {
		FATAL("pthread_attr_setstacksize %d",setstacksize_result);
	}
 	const int num_threads=4;
	// Allocate memory for pthread_create() arguments
	struct thread_info* thread_info=(struct thread_info*)calloc(num_threads,sizeof(struct thread_info));
	if(thread_info==NULL) {
		FATAL("calloc");
	}
	// Create the threads and initialize the core_id argument,which will be used to set the thread to the specific CPU core.
	// For example, we want the first pthread to camp on the first CPU core which has the ID 0. So we pass the value 0 to its core_id.
	int tnum;
	for(tnum=0;tnum<num_threads;tnum++) {
 		thread_info[tnum].core_id=tnum;
		// The pthread_create() call stores the thread ID into corresponding element of thread_info[]
		const int create_result=pthread_create(&thread_info[tnum].thread_id,&attr,&thread_camper,&thread_info[tnum]);
		if(create_result!=0) {
			FATAL("pthread_create %d",create_result);
		}
	}
 	// Destroy the thread attributes object, since it is no longer needed
	const int destroy_result=pthread_attr_destroy(&attr);
	if(destroy_result!=0) {
		FATAL("pthread_attr_destroy %d",destroy_result);
	}
 	// Now join with each thread,and display its returned value
	for (tnum=0;tnum<num_threads;tnum++) {
		void *res;
		const int join_result=pthread_join(thread_info[tnum].thread_id,&res);
		if(join_result!=0) {
			FATAL("pthread_join %d",join_result);
		}
		uprintf("Joined with thread %d; returned value was %s\n",thread_info[tnum].core_id,(char *) res);
		free(res);// Free memory allocated by thread
	}
	free(thread_info);

	int s;
	cpu_set_t cpuset;
	pthread_t thread=pthread_self();
	// Set affinity mask to include CPUs 0 to 7.
	CPU_ZERO(&cpuset);
	for(int j=0;j<8;j++)
		CPU_SET(j,&cpuset);
	s=pthread_setaffinity_np(thread,sizeof(cpuset),&cpuset);
	if(s!=0)
		FATAL("pthread_setaffinity_np %d",s);
	// Check the actual affinity mask assigned to the thread.
	s=pthread_getaffinity_np(thread,sizeof(cpuset),&cpuset);
	if(s!=0)
		FATAL("pthread_getaffinity_np %d",s);
	uprintf("Set returned by pthread_getaffinity_np() contained:\n");
	for(int j=0;j<CPU_SETSIZE;j++)
		if(CPU_ISSET(j,&cpuset))
			uprintf("CPU %d\n",j);
}



#include <sched.h>

#include "shared/queue.h"
#include "shared/crc32.h"

struct TimeTest {
	uint32_t m_index;
	uint64_t m_time;
};

FixedQueueMT<TimeTest,128> g_queue;
std::atomic<bool> g_close={false};

void* busyThreadFunc(void *arg) {
	SetSelfAffinityMask((1<<1)|(1<<2)|(1<<3));
	char buf[1024];
	memset(buf,0xff,sizeof(buf));
	uint32_t crcSum=0;
	uint32_t count=0;
	while(!g_close) {
		uint32_t crc=CRC32(buf,sizeof(buf));
		crcSum+=crc;
		count++;
	}
	uprintf("count %d crcSum %08x\n",count,crcSum);
	return 0;
}

void* threadFunc(void *arg) {
	SetSelfAffinityMask(1<<0);
	std::atomic<bool> close={false};
	while(!close) {
		g_queue.Pop([&](TimeTest* p){
			if(!p) {
				uprintf("pop close\n");
				close=true;
				return;
			}
			int dt=(int)(GetTimeEpochMicroseconds()-p->m_time);
			if(dt>100)
				uprintf("pop %d %dus\n",p->m_index,dt);
		});
	}
	return 0;
}

void TestThreadPriority() {
	pthread_attr_t attr;
	const int attr_init_result=pthread_attr_init(&attr);
	if(attr_init_result!=0) {
		FATAL("pthread_attr_init %d",attr_init_result);
	}

	pthread_t thread_id;
	const int create_result=pthread_create(&thread_id,&attr,&threadFunc,NULL);
	if(create_result!=0) {
		FATAL("pthread_create %d",create_result);
	}
	pthread_t busy_thread_ids[4];
	for(int i=0;i!=countof(busy_thread_ids);i++) {
		const int create_result=pthread_create(&busy_thread_ids[i],&attr,&busyThreadFunc,NULL);
		if(create_result!=0) {
			FATAL("busy pthread_create %d",create_result);
		}
	}

	std::this_thread::sleep_for(std::chrono::microseconds(10000));

	SetSelfAffinityMask(1<<0);

	uprintf("enter main loop\n");
	int count=0;
	uint64_t timeStart=GetTimeEpochMicroseconds();
	while(true){
		auto time=std::chrono::system_clock::now();
		uint64_t t=GetTimeEpochMicroseconds();
		g_queue.Push([&](TimeTest* p){
			p->m_time=t;
			p->m_index=count++;
		});
		if(t-timeStart>10*1000000L)
			break;
		std::this_thread::sleep_until(time+std::chrono::microseconds(1000));
	}
	uprintf("leave main loop %d\n",count);

	g_queue.Close();
	void* res;
	const int join_result=pthread_join(thread_id,&res);
	if(join_result) {
		FATAL("pthread_join %d",join_result);
	}

	g_close=true;
	for(int i=0;i!=countof(busy_thread_ids);i++) {
		const int join_result=pthread_join(busy_thread_ids[i],&res);
		if(join_result) {
			FATAL("busy pthread_join %d",join_result);
		}
	}


	const int destroy_result=pthread_attr_destroy(&attr);
	if(destroy_result!=0) {
		FATAL("pthread_attr_destroy %d",destroy_result);
	}
}

#endif//_WIN32
