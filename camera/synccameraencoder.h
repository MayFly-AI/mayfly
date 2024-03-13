#pragma once
#include <stdint.h>

class Dict;
struct Options;

class SyncCameraEncoder {
public:
	virtual ~SyncCameraEncoder(){}
	virtual Options* GetOptions() const = 0;
};

SyncCameraEncoder* CreateSyncCameraEncoder(const Dict* settings);
void DestroySyncCameraEncoder(SyncCameraEncoder* p);

#include <functional>
namespace libcamera {
	class ControlList;
}
typedef std::function<void(void*, size_t, int64_t)> EncodeOutputReadyCallback;
typedef std::function<void(libcamera::ControlList&)> MetadataReadyCallback;
typedef std::function<void(const uint8_t*, size_t, int64_t)> BufferdataReadyCallback;
void run(SyncCameraEncoder* a, BufferdataReadyCallback buf_cb, EncodeOutputReadyCallback enc_cb, MetadataReadyCallback meta_cb);
