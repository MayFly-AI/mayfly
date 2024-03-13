/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * h264_encoder.h - h264 video encoder.
 */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <functional>
#include "options.h"
#include "stream_info.h"

typedef std::function<void(void*, size_t, int64_t)> OutputReadyCallback;

class H264Encoder
{
public:
	H264Encoder(const Options* options, const StreamInfo& info);
	~H264Encoder();

	// This callback is how the application is told that an encoded buffer is
	// available. The application may not hang on to the memory once it returns
	// (but the callback is already running in its own thread).
	void SetOutputReadyCallback(OutputReadyCallback callback) { m_output_cb = callback; }

	// Encode the given buffer. The buffer is specified both by an fd and size
	// describing a DMABUF, and by a mmapped userland pointer.
	void EncodeBuffer(int fd, const uint8_t* buffer_data, size_t buffer_size, int64_t timestamp_us);

protected:
	OutputReadyCallback m_output_cb;
	const Options* m_options;

private:
	// We want at least as many output buffers as there are in the camera queue
	// (we always want to be able to queue them when they arrive). Make loads
	// of capture buffers, as this is our buffering mechanism in case of delays
	// dealing with the output bitstream.
	static constexpr int NUM_OUTPUT_BUFFERS = 6;
	static constexpr int NUM_CAPTURE_BUFFERS = 12;

	// This thread just sits waiting for the encoder to finish stuff. It will either:
	// * receive "output" buffers (codec inputs), which we must return to the caller
	// * receive encoded buffers, which we pass to the application.
	void pollThread();

	// Handle the output buffers in another thread so as not to block the encoder. The
	// application can take its time, after which we return this buffer to the encoder for
	// re-use.
	void outputThread();

	bool m_abortPoll;
	bool m_abortOutput;
	int m_fd;
	struct BufferDescription {
		void* mem;
		size_t size;
	};
	BufferDescription m_buffers[NUM_CAPTURE_BUFFERS];
	int m_num_capture_buffers;
	std::thread m_poll_thread;
	std::mutex m_input_buffers_available_mutex;
	std::queue<int> m_input_buffers_available;
	struct OutputItem {
		void* mem;
		size_t bytes_used;
		size_t length;
		unsigned int index;
		int64_t timestamp_us;
	};
	std::queue<OutputItem> m_output_queue;
	std::mutex m_output_mutex;
	std::condition_variable m_output_cond_var;
	std::thread m_output_thread;
};
