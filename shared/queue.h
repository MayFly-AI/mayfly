#pragma once

#include "shared/misc.h"

#include <atomic>
#include <mutex>
#include <functional>
#include <condition_variable>

/*
Usage:
	struct Test {
		int m_a;
	};
	int a=2;
	FixedQueueMT<Test,8> q;
	q.Push([&](Test* element)->void {
		element->m_a=a;
	});
	q.Pop([&](Test* element)->void {
		if(!element)
			return;		//Closing
		a=element->m_a;
	});
	q.Close();			//Force Pop wait to be interrupted


Multi workers example

FixedQueueMT<int,128> g_queue;

int WorkerFunc(int i) {
	uprintf("begin %d",i);
	while(true) {
		int val=-1;
		if(!g_queue.Pop(500,[&](int* element)->void {	//Get job from queue or time out
			if(!element)
				return;		//Closing					//Close signal
			val=*element;
		})){
			uprintf("timeout %d",i);					//Timeout reached
			continue;
		};
		if(val==-1)										//Close or job
			break;
		uprintf("pop %d val %d",i,val);					//Deal with it!
		Thread::Sleep((mRandomU32()&0xff)+200);			//For a variable amount of time
	}
	uprintf("end %d",i);
	return 0;
}

void ManageWorkers() {
	std::vector<std::thread> threads;
	for(int i=0;i!=10;i++) {
		threads.emplace_back(WorkerFunc,i);				//Start worker
	}
	for(int i=0;i!=100;i++) {
		if(!g_queue.Push([&](int* element)->void {		//Add job
			*element=10000+i;//(int)mRandomU32()&0xffff;
		})) {
			uprintf("queue full");						//Queue full
		}
		Thread::Sleep(mRandomU32()&0xf);
	}

	Thread::Sleep(1000);
	uprintf("wait begin");
	while(!g_queue.Empty()) {							//Wait for queue to empty if needed
		Thread::Sleep(10);
	}
	uprintf("wait end");
	g_queue.Close();									//Signal all workers

	for(int i=0;i!=10;i++) {
		threads[i].join();								//Wait for worker to finish
	}
	INT3;
}

*/

template <typename T,uint64_t _count> class FixedQueueMT {
public:
	FixedQueueMT() {
		Reset();
	}
	explicit FixedQueueMT(std::function<void(T* element)> init) {
		Reset();
		Init(init);
	}
	void Init(std::function<void(T* element)> init) {
		std::unique_lock<std::mutex> mlock(m_mutex);
		for(T& obj : m_array) {
			init(&obj);
		}
	}
	void Reset() {
		std::unique_lock<std::mutex> mlock(m_mutex);
		m_read=0;
		m_write=0;
		m_closed=false;
		m_closeInstant=true;
	}
	bool IsClosed()const { return m_closed; }
	bool Empty()const { return m_write==m_read; }
	bool Full()const { return m_write==m_read+_count; }
	int Size()const { return (int)(m_write-m_read); }
	int Max()const { return _count; }
	void Close(const bool closeInstant=true) {
		std::unique_lock<std::mutex> mlock(m_mutex);
		m_closed = true;
		m_closeInstant = closeInstant;
		mlock.unlock();
		m_cond.notify_all();
	}
	void Pop(std::function<void(T* element)> lf) {
		std::unique_lock<std::mutex> mlock(m_mutex);
		while(m_write==m_read && !m_closed) {
			m_cond.wait(mlock);
		}
		if(m_write==m_read || (m_closed && m_closeInstant)) {
			lf(0);
		} else {
			lf(m_array+m_read++%_count);
		}
	}
	void Flush(std::function<void(T* element)> lf) {
		std::unique_lock<std::mutex> mlock(m_mutex);
		while(m_write!=m_read) {
			lf(m_array+m_read++%_count);
		}
	}
	bool Pop(int milliseconds,std::function<void(T*element)> lf) {
		std::unique_lock<std::mutex> mlock(m_mutex);
		while(m_write==m_read && !m_closed) {
			if(milliseconds!=-1) {
				if(m_cond.wait_for(mlock,std::chrono::milliseconds(milliseconds))==std::cv_status::timeout)
					return false;
			} else {
				m_cond.wait(mlock);
			}
		}
		if(m_write==m_read || (m_closed && m_closeInstant)) {
			lf(0);
		} else {
			lf(m_array+m_read++%_count);
		}
		return true;
	}
	bool Push(std::function<void(T*element)> lf) {
		if(m_closed)
			FATAL("FixedQueueMT::Push called after close");
		std::unique_lock<std::mutex> mlock(m_mutex);
		if(m_write==m_read+_count)
			return false;
		lf(m_array+m_write++%_count);
		mlock.unlock();
		m_cond.notify_one();
		return true;
	}
private:
#if SIZE_MAX==0xFFFFFFFF
	std::atomic<uint32_t> m_read;
	std::atomic<uint32_t> m_write;
#else
	std::atomic<uint64_t> m_read;
	std::atomic<uint64_t> m_write;
#endif
	std::atomic<bool> m_closed;
	std::atomic<bool> m_closeInstant;
	std::mutex m_mutex;
	std::condition_variable m_cond;
	T m_array[_count];
};
