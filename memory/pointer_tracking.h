#pragma once
#include <unordered_set>
#include <mutex>
#include "shared/misc.h"

struct PointerTracking {
	typedef std::unordered_set<void*> Container;
	PointerTracking(const char* name) : m_name(name) {}
	void Unregister(void* p) {
		size_t count;
		{
			std::lock_guard<std::mutex> lock(m_mtx);
			count=m_allocations.erase(p);
		}
		ASSERT(count==1,"%s: Failed to unregister %p", m_name.c_str(), p);
	}
	void Register(void* p) {
		std::pair<Container::const_iterator, bool> ret;
		{
			std::lock_guard<std::mutex> lock(m_mtx);
			ret=m_allocations.insert(p);
		}
		ASSERT(ret.second,"%s: Failed to register %p", m_name.c_str(), p);
	}
	size_t Size() {
		std::lock_guard<std::mutex> lock(m_mtx);
		return m_allocations.size();
	}
	Container m_allocations;
	std::mutex m_mtx;
	std::string m_name;
};
