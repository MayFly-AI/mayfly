#pragma once
#include "iserial.h"
#include <atomic>
#include <thread>

class TTYSerialConnection : public ISerialConnection {
public:
	TTYSerialConnection(const char* dev);
	virtual ~TTYSerialConnection() override;
	void Run() override;
	void SetCallback(TDataReadyCallbackFunc f, void* arg) override {
		m_callback=f;
		m_callbackArg=arg;
	}
	void Stop() override {
		m_stop=true;
	}
	int Write(const uint8_t* data, int len) override;

protected:
	void Loop();
	int m_fd{-1};
	TDataReadyCallbackFunc m_callback;
	void* m_callbackArg{nullptr};
	std::atomic<bool> m_stop{false};
	std::thread m_run_thread;
	bool m_running{false};
};
