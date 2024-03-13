#pragma once
#include "shared/queue.h"
#include "shared/dctquant16.h"
#include <vector>

typedef std::function<void(const uint8_t*, size_t, int64_t)> ToFOutputReadyCallback;

class ArducamToF {
public:
	ArducamToF(ToFOutputReadyCallback cb, bool encode, const float& thresh=30.);
	~ArducamToF();

	void MainLoop();

	const int m_width = 240;
	const int m_height = 180;
	float m_amplitudeThresh;

private:
	void GetPreview(uint8_t* preview_ptr, const float* phase_image_ptr, const float* amplitude_image_ptr);
	void GetMaskedDepth(uint16_t* depth_masked, const float* phase_image_ptr, const float* amplitude_image_ptr);
	void SetupDevice();
	void StartCapturing();
	void StopCapturing();
	void CleanupDevice();
	void ProcessImage(const void *p, int size, int subFrame);
	int ReadFrame();
	void InitMMAP(void);

	struct Buffer {
		void* m_start;
		size_t  m_length;
	};

	int16_t* m_frameBuf=0;
	const char* m_devName = "/dev/video0";
	int m_fd = -1;
	struct Buffer* m_buffers;
	unsigned int m_nBuffers;
	float* m_phase;
	float* m_ampli;
	std::vector<uint16_t> m_phaseMasked;
	std::vector<uint16_t> m_filtered;
	int64_t m_clockOffset;
	bool m_encode;
	EncoderDCT16* m_encoder;
	std::vector<uint8_t> m_encoded;

	ToFOutputReadyCallback m_callback;
};
