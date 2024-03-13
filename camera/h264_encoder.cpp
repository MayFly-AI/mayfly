/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * h264_encoder.cpp - h264 video encoder.
 */

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <string.h>
#include <map>

#include <chrono>
#include <iostream>

#include "h264_encoder.h"

static volatile bool gPrintEncoderTimings = false;

static int xioctl(int fd, unsigned long ctl, void *arg) {
	int ret, num_tries = 10;
	do {
		ret = ioctl(fd, ctl, arg);
	} while (ret == -1 && errno == EINTR && num_tries-- > 0);
	return ret;
}

static int get_v4l2_colorspace(const std::optional<libcamera::ColorSpace>& cs) {
	if (cs == libcamera::ColorSpace::Rec709)
		return V4L2_COLORSPACE_REC709;
	else if (cs == libcamera::ColorSpace::Smpte170m)
		return V4L2_COLORSPACE_SMPTE170M;

	return V4L2_COLORSPACE_SMPTE170M;
}

H264Encoder::H264Encoder(const Options* options, const StreamInfo& info)
	: m_options(options), m_abortPoll(false), m_abortOutput(false) {
	// First open the encoder device. Maybe we should double-check its "caps".

	const char device_name[] = "/dev/video11";
	m_fd = open(device_name, O_RDWR, 0);
	if (m_fd < 0)
		throw std::runtime_error("failed to open V4L2 H264 encoder");

	// Apply any options->

	v4l2_control ctrl = {};
	if (options->bitrate) {
		ctrl.id = V4L2_CID_MPEG_VIDEO_BITRATE;
		ctrl.value = options->bitrate;
		if (xioctl(m_fd, VIDIOC_S_CTRL, &ctrl) < 0)
			throw std::runtime_error("failed to set bitrate");
	}
	if (!options->profile.empty()) {
		static const std::map<std::string, int> profile_map =
			{ { "baseline", V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE },
			  { "main", V4L2_MPEG_VIDEO_H264_PROFILE_MAIN },
			  { "high", V4L2_MPEG_VIDEO_H264_PROFILE_HIGH } };
		auto it = profile_map.find(options->profile);
		if (it == profile_map.end())
			throw std::runtime_error("no such profile " + options->profile);
		ctrl.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
		ctrl.value = it->second;
		if (xioctl(m_fd, VIDIOC_S_CTRL, &ctrl) < 0)
			throw std::runtime_error("failed to set profile");
	}

	if (!options->level.empty()) {
		static const std::map<std::string, int> level_map =
			{ { "4", V4L2_MPEG_VIDEO_H264_LEVEL_4_0 },
			  { "4.1", V4L2_MPEG_VIDEO_H264_LEVEL_4_1 },
			  { "4.2", V4L2_MPEG_VIDEO_H264_LEVEL_4_2 } };
		auto it = level_map.find(options->level);
		if (it == level_map.end())
			throw std::runtime_error("no such level " + options->level);
		ctrl.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
		ctrl.value = it->second;
		if (xioctl(m_fd, VIDIOC_S_CTRL, &ctrl) < 0)
			throw std::runtime_error("failed to set level");
	}

	/*
	if (options->intra) {
		ctrl.id = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD;
		ctrl.value = options->intra;
		if (xioctl(m_fd, VIDIOC_S_CTRL, &ctrl) < 0)
			throw std::runtime_error("failed to set intra period");
	}
	*/

	if (options->inline_headers) {
		ctrl.id = V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER;
		ctrl.value = 1;
		if (xioctl(m_fd, VIDIOC_S_CTRL, &ctrl) < 0)
			throw std::runtime_error("failed to set inline headers");
	}
	if (false) {
		ctrl.id = V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME;
		ctrl.value = 1;
		if (xioctl(m_fd, VIDIOC_S_CTRL, &ctrl) < 0)
			throw std::runtime_error("failed to set force keyframe");
	}

	// Set the output and capture formats. We know exactly what they will be.
	if(V4L2_PIX_FMT_YUV420 != info.pixel_format.fourcc()) {
		throw std::runtime_error("unexpected pixel format");
	}

	v4l2_format fmt = {};
	fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	fmt.fmt.pix_mp.width = info.width;
	fmt.fmt.pix_mp.height = info.height;
	fmt.fmt.pix_mp.pixelformat = info.pixel_format.fourcc();
	fmt.fmt.pix_mp.plane_fmt[0].bytesperline = info.stride;
	fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
	fmt.fmt.pix_mp.colorspace = get_v4l2_colorspace(info.colour_space);
	fmt.fmt.pix_mp.num_planes = 1;
	if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0)
		throw std::runtime_error("failed to set output format");

	fmt = {};
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	fmt.fmt.pix_mp.width = options->width;
	fmt.fmt.pix_mp.height = options->height;
	fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
	fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_DEFAULT;
	fmt.fmt.pix_mp.num_planes = 1;
	fmt.fmt.pix_mp.plane_fmt[0].bytesperline = 0;
	fmt.fmt.pix_mp.plane_fmt[0].sizeimage = 512 << 10;
	if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) < 0)
		throw std::runtime_error("failed to set capture format");

	struct v4l2_streamparm parm = {};
	parm.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	parm.parm.output.timeperframe.numerator = 1000 / options->framerate;
	parm.parm.output.timeperframe.denominator = 1000;
	if (xioctl(m_fd, VIDIOC_S_PARM, &parm) < 0)
		throw std::runtime_error("failed to set streamparm");

	// Request that the necessary buffers are allocated. The output queue
	// (input to the encoder) shares buffers from our caller, these must be
	// DMABUFs. Buffers for the encoded bitstream must be allocated and
	// m-mapped.

	v4l2_requestbuffers reqbufs = {};
	reqbufs.count = NUM_OUTPUT_BUFFERS;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	reqbufs.memory = V4L2_MEMORY_DMABUF;
	if (xioctl(m_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		throw std::runtime_error("request for output buffers failed");

	// We have to maintain a list of the buffers we can use when our caller gives
	// us another frame to encode.
	for (unsigned int i = 0; i < reqbufs.count; i++)
		m_input_buffers_available.push(i);

	reqbufs = {};
	reqbufs.count = NUM_CAPTURE_BUFFERS;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	if (xioctl(m_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		throw std::runtime_error("request for capture buffers failed");
	m_num_capture_buffers = reqbufs.count;

	for (unsigned int i = 0; i < reqbufs.count; i++) {
		v4l2_plane planes[VIDEO_MAX_PLANES];
		v4l2_buffer buffer = {};
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		buffer.memory = V4L2_MEMORY_MMAP;
		buffer.index = i;
		buffer.length = 1;
		buffer.m.planes = planes;
		if (xioctl(m_fd, VIDIOC_QUERYBUF, &buffer) < 0)
			throw std::runtime_error("failed to capture query buffer " + std::to_string(i));
		m_buffers[i].mem = mmap(0, buffer.m.planes[0].length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd,
							   buffer.m.planes[0].m.mem_offset);
		if (m_buffers[i].mem == MAP_FAILED)
			throw std::runtime_error("failed to mmap capture buffer " + std::to_string(i));
		m_buffers[i].size = buffer.m.planes[0].length;
		// Whilst we're going through all the capture buffers, we may as well queue
		// them ready for the encoder to write into.
		if (xioctl(m_fd, VIDIOC_QBUF, &buffer) < 0)
			throw std::runtime_error("failed to queue capture buffer " + std::to_string(i));
	}

	// Enable streaming and we're done.

	v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (xioctl(m_fd, VIDIOC_STREAMON, &type) < 0)
		throw std::runtime_error("failed to start output streaming");

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (xioctl(m_fd, VIDIOC_STREAMON, &type) < 0)
		throw std::runtime_error("failed to start capture streaming");

	m_output_thread = std::thread(&H264Encoder::outputThread, this);
	m_poll_thread = std::thread(&H264Encoder::pollThread, this);
}

H264Encoder::~H264Encoder() {
	m_abortPoll = true;
	m_poll_thread.join();
	m_abortOutput = true;
	m_output_thread.join();

	// Turn off streaming on both the output and capture queues, and "free" the
	// buffers that we requested. The capture ones need to be "munmapped" first.

	v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (xioctl(m_fd, VIDIOC_STREAMOFF, &type) < 0)
		printf("Failed\n");
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (xioctl(m_fd, VIDIOC_STREAMOFF, &type) < 0)
		printf("Failed\n");

	v4l2_requestbuffers reqbufs = {};
	reqbufs.count = 0;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	reqbufs.memory = V4L2_MEMORY_DMABUF;
	if (xioctl(m_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		printf("failed\n");

	for (int i = 0; i < m_num_capture_buffers; i++)
		if (munmap(m_buffers[i].mem, m_buffers[i].size) < 0)
			printf("failed\n");
	reqbufs = {};
	reqbufs.count = 0;
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	reqbufs.memory = V4L2_MEMORY_MMAP;
	if (xioctl(m_fd, VIDIOC_REQBUFS, &reqbufs) < 0)
		printf("failed\n");

	close(m_fd);
}

void H264Encoder::EncodeBuffer(int fd, const uint8_t* buffer_data, size_t buffer_size, int64_t timestamp_us) {

	if(gPrintEncoderTimings) {
		auto sys_time = std::chrono::system_clock::now();
		int64_t now = std::chrono::time_point_cast<std::chrono::microseconds>(sys_time).time_since_epoch().count();
		printf("Encode In at:  %18li %18li\n  delta: %18lius\n", now, timestamp_us, (now-timestamp_us));
	}

	int index;
	{
		// We need to find an available output buffer (input to the codec) to
		// "wrap" the DMABUF.
		std::lock_guard<std::mutex> lock(m_input_buffers_available_mutex);
		if (m_input_buffers_available.empty())
			throw std::runtime_error("no buffers available to queue codec input");
		index = m_input_buffers_available.front();
		m_input_buffers_available.pop();
	}
	v4l2_buffer buf = {};
	v4l2_plane planes[VIDEO_MAX_PLANES] = {};
	buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	buf.index = index;
	buf.field = V4L2_FIELD_NONE;
	buf.memory = V4L2_MEMORY_DMABUF;
	buf.length = 1;
	buf.timestamp.tv_sec = timestamp_us / 1000000;
	buf.timestamp.tv_usec = timestamp_us % 1000000;
	buf.m.planes = planes;
	buf.m.planes[0].m.fd = fd;
	buf.m.planes[0].bytesused = buffer_size;
	buf.m.planes[0].length = buffer_size;
	if (xioctl(m_fd, VIDIOC_QBUF, &buf) < 0)
		throw std::runtime_error("failed to queue input to codec");
}

void H264Encoder::pollThread() {
	while (true) {
		{
			std::lock_guard<std::mutex> lock(m_input_buffers_available_mutex);
			if (m_abortPoll && m_input_buffers_available.size() == NUM_OUTPUT_BUFFERS) {
				//printf("exiting pollThread\n");
				break;
			}
		}

		pollfd p = { m_fd, POLLIN, 0 };
		int ret = poll(&p, 1, 200);
		if (ret == -1) {
			if (errno == EINTR || errno == EAGAIN)
				continue;
			throw std::runtime_error("unexpected errno " + std::to_string(errno) + " from poll");
		}
		if (p.revents & POLLIN) {
			v4l2_buffer buf;
			v4l2_plane planes[VIDEO_MAX_PLANES];

			memset(&buf, 0, sizeof(buf));
			memset(planes, 0, sizeof(planes));
			buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
			buf.memory = V4L2_MEMORY_DMABUF;
			buf.length = 1;
			buf.m.planes = planes;
			if(!xioctl(m_fd, VIDIOC_DQBUF, &buf)) {
				// Return this to the caller, first noting that this buffer, identified
				// by its index, is available for queueing up another frame.
				{
					std::lock_guard<std::mutex> lock(m_input_buffers_available_mutex);
					m_input_buffers_available.push(buf.index);
				}
			}

			memset(&buf, 0, sizeof(buf));
			memset(planes, 0, sizeof(planes));
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.length = 1;
			buf.m.planes = planes;
			if(!xioctl(m_fd, VIDIOC_DQBUF, &buf)) {
				// We push this encoded buffer to another thread so that our
				// application can take its time with the data without blocking the
				// encode process.

				int64_t timestamp_us = (buf.timestamp.tv_sec * (int64_t)1000000) + buf.timestamp.tv_usec;

				OutputItem item = { m_buffers[buf.index].mem,
									buf.m.planes[0].bytesused,
									buf.m.planes[0].length,
									buf.index,
									timestamp_us };
				{
					std::lock_guard<std::mutex> lock(m_output_mutex);
					m_output_queue.push(std::move(item));
				}
				m_output_cond_var.notify_one();
			}
		}
	}
}

void H264Encoder::outputThread() {
	OutputItem item;
	while (true) {

		// proceed if aborted or on successful deque
		{
			std::unique_lock<std::mutex> lock(m_output_mutex);
			while(true) {
				if(m_abortOutput && m_output_queue.empty()) {
					//printf("exiting outputThread\n");
					return;
				}
				if(!m_output_queue.empty()) {
					item = m_output_queue.front();
					m_output_queue.pop();
					break;
				} else {
					using namespace std::chrono_literals;
					m_output_cond_var.wait_for(lock, 200ms);
				}
			}
		}

		if(gPrintEncoderTimings) {
			auto sys_time = std::chrono::system_clock::now();
			int64_t now = std::chrono::time_point_cast<std::chrono::microseconds>(sys_time).time_since_epoch().count();
			printf("Encode Out at: %18li %18li\n  delta: %18lius\n", now, item.timestamp_us, (now-item.timestamp_us));
		}

		m_output_cb(item.mem, item.bytes_used, item.timestamp_us);
		v4l2_buffer buf = {};
		v4l2_plane planes[VIDEO_MAX_PLANES] = {};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = item.index;
		buf.length = 1;
		buf.m.planes = planes;
		buf.m.planes[0].bytesused = 0;
		buf.m.planes[0].length = item.length;
		if (xioctl(m_fd, VIDIOC_QBUF, &buf) < 0)
			throw std::runtime_error("failed to re-queue encoded buffer");
	}
}
