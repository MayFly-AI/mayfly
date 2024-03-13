/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_app.cpp - base class for libcamera apps.
 */

#include "synccameraencoder.h"

#if ENABLE_LIBCAMERA
#include <sys/mman.h>

#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>

#include <libcamera/base/span.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/logging.h>
#include <libcamera/property_ids.h>
#include "stream_info.h"
#include "h264_encoder.h"
#include "options.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <string.h>
#include "shared/misc.h"

struct CompletedRequest {

	CompletedRequest(unsigned int seq, libcamera::Request* r)
		: sequence(seq), buffers(r->buffers()), metadata(r->metadata()), request(r) {
		r->reuse();
	}
	unsigned int sequence;
	libcamera::Request::BufferMap buffers;
	libcamera::ControlList metadata;
	libcamera::Request* request;
};
using CompletedRequestPtr = std::shared_ptr<CompletedRequest>;


class SyncCameraEncoderImpl : public SyncCameraEncoder {
public:
	using Stream = libcamera::Stream;
	using FrameBuffer = libcamera::FrameBuffer;
	using ControlList = libcamera::ControlList;
	using Request = libcamera::Request;
	using CameraManager = libcamera::CameraManager;
	using Camera = libcamera::Camera;
	using CameraConfiguration = libcamera::CameraConfiguration;
	using FrameBufferAllocator = libcamera::FrameBufferAllocator;
	using StreamRole = libcamera::StreamRole;
	using StreamRoles = std::vector<libcamera::StreamRole>;
	using StreamConfiguration = libcamera::StreamConfiguration;
	using BufferMap = Request::BufferMap;

	enum class MsgType {
		RequestComplete,
		Timeout
	};
	struct Msg {
		explicit Msg(const MsgType& t) : type(t) {}
		Msg(const MsgType& t, CompletedRequestPtr&& p) : type(t), payload(p) {}
		MsgType type;
		CompletedRequestPtr payload;
	};

	SyncCameraEncoderImpl();
	virtual ~SyncCameraEncoderImpl();

	Options* GetOptions() const override {
		return m_options.get();
	};

	void OpenCamera();
	void CloseCamera();

	void ConfigureVideo();

	void Teardown();
	void StartCamera();
	void StopCamera();

	Msg Wait();

	Stream* VideoStream() const { return m_video_stream;}
	void SetControls(ControlList& controls);
	StreamInfo GetStreamInfo(Stream const* stream) const;

	void StartEncoder();
	void EncodeBuffer(const CompletedRequestPtr& completed_request, Stream* stream);

	// This is callback when the encoder gives you the encoded output data.
	void SetEncodeOutputReadyCallback(EncodeOutputReadyCallback callback) { m_encode_output_ready_callback = callback; }
	void SetMetadataReadyCallback(MetadataReadyCallback callback) { m_metadata_ready_callback = callback; }
	void SetBufferdataReadyCallback(BufferdataReadyCallback callback) { m_bufferdata_ready_callback = callback; }
	void StopEncoder() { delete m_encoder; m_encoder=nullptr;}

protected:
	std::unique_ptr<Options> m_options;

private:
	class MessageQueue {
	public:
		void Post(Msg&& msg) {
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				m_queue.push(msg);
			}
			m_cond.notify_one();
		}
		Msg Wait() {
			std::unique_lock<std::mutex> lock(m_mutex);
			m_cond.wait(lock, [this] { return !m_queue.empty(); });
			Msg msg = std::move(m_queue.front());
			m_queue.pop();
			return msg;
		}
		void Clear() {
			std::unique_lock<std::mutex> lock(m_mutex);
			m_queue = {};
		}

	private:
		std::queue<Msg> m_queue;
		std::mutex m_mutex;
		std::condition_variable m_cond;
	};

	void setupCapture();
	void makeRequests();
	void queueRequest(CompletedRequest* completed_request);
	void requestComplete(Request* request);

	std::unique_ptr<CameraManager> m_camera_manager;
	std::shared_ptr<Camera> m_camera;
	bool m_camera_acquired = false;
	std::unique_ptr<CameraConfiguration> m_configuration;
	std::map<const FrameBuffer*, std::vector<libcamera::Span<uint8_t>>> m_mapped_buffers;

	Stream* m_video_stream;
	FrameBufferAllocator* m_allocator = nullptr;
	std::map<Stream*, std::queue<FrameBuffer*>> m_frame_buffers;
	std::vector<std::unique_ptr<Request>> m_requests;
	std::mutex m_completed_requests_mutex;
	std::set<CompletedRequest*> m_completed_requests;
	bool m_camera_started = false;
	std::mutex m_camera_stop_mutex;
	MessageQueue m_msg_queue;

	// For setting camera controls.
	std::mutex m_control_mutex;
	ControlList m_controls;
	// Other:
	uint64_t m_sequence = 0;
	int64_t m_time_base = 0;

private:
	EncodeOutputReadyCallback m_encode_output_ready_callback;
	MetadataReadyCallback m_metadata_ready_callback;
	BufferdataReadyCallback m_bufferdata_ready_callback;
	H264Encoder* m_encoder;
};



SyncCameraEncoderImpl::SyncCameraEncoderImpl()
	: m_controls(libcamera::controls::controls) {
	m_options = std::make_unique<Options>();
	m_camera_started = true;
	m_encoder = nullptr;
	m_encode_output_ready_callback= nullptr;
	m_metadata_ready_callback = nullptr;
	m_bufferdata_ready_callback = nullptr;
}

SyncCameraEncoderImpl::~SyncCameraEncoderImpl() {
	StopEncoder();
	StopCamera();
	Teardown();
	CloseCamera();
}

void SyncCameraEncoderImpl::OpenCamera() {
	uprintf("Opening camera...\n");

	m_camera_manager = std::make_unique<CameraManager>();
	int ret = m_camera_manager->start();
	if (ret)
		throw std::runtime_error("camera manager failed to start, code " + std::to_string(-ret));

	std::vector<std::shared_ptr<libcamera::Camera>> cameras = m_camera_manager->cameras();
	// Do not show USB webcams as these are not supported in libcamera-apps!
	auto rem = std::remove_if(cameras.begin(), cameras.end(),
							  [](auto& cam) { return cam->id().find("/usb") != std::string::npos; });
	cameras.erase(rem, cameras.end());

	if (cameras.size() == 0)
		throw std::runtime_error("no cameras available");
	if (m_options->camera >= cameras.size())
		throw std::runtime_error("selected camera is not available");

	std::string const& cam_id = cameras[m_options->camera]->id();
	m_camera = m_camera_manager->get(cam_id);
	if (!m_camera)
		throw std::runtime_error("failed to find camera " + cam_id);

	if (m_camera->acquire())
		throw std::runtime_error("failed to acquire camera " + cam_id);
	m_camera_acquired = true;

	uprintf("Acquired camera %s\n", cam_id.c_str());

}

void SyncCameraEncoderImpl::CloseCamera() {
	if (m_camera_acquired)
		m_camera->release();
	m_camera_acquired = false;
	m_camera.reset();
	m_camera_manager.reset();
	//uprintf("Camera closed\n");
}

void SyncCameraEncoderImpl::ConfigureVideo() {
	uprintf("Configuring video...\n");

	StreamRoles stream_roles = { StreamRole::VideoRecording };
	m_configuration = m_camera->generateConfiguration(stream_roles);
	if (!m_configuration)
		throw std::runtime_error("failed to generate video configuration");

	// Now we get to override any of the default settings from the m_options->
	StreamConfiguration& cfg = m_configuration->at(0);
	cfg.pixelFormat = libcamera::formats::YUV420;
	cfg.bufferCount = 6; // 6 buffers is better than 4
	cfg.size.width = m_options->width;
	cfg.size.height = m_options->height;
	cfg.colorSpace = libcamera::ColorSpace::Smpte170m;
	m_configuration->transform = libcamera::Transform::Identity;
	// NoiseReductionModeMinimal, NoiseReductionModeFast, NoiseReductionModeOff, NoiseReductionModeHighQuality                  // isp    size
	//m_controls.set(libcamera::controls::draft::NoiseReductionMode, libcamera::controls::draft::NoiseReductionModeOff);         // 30ms 25000B
	//m_controls.set(libcamera::controls::draft::NoiseReductionMode, libcamera::controls::draft::NoiseReductionModeMinimal);     // 31ms  9400B
	m_controls.set(libcamera::controls::draft::NoiseReductionMode, libcamera::controls::draft::NoiseReductionModeFast);          // 34ms  7600B
	//m_controls.set(libcamera::controls::draft::NoiseReductionMode, libcamera::controls::draft::NoiseReductionModeHighQuality); // 44ms  7600B

	setupCapture();
	m_video_stream = m_configuration->at(0).stream();
	uprintf("Video setup complete\n");
}

void SyncCameraEncoderImpl::Teardown() {
	//uprintf("Tearing down requests, buffers and configuration\n");

	for (auto& iter : m_mapped_buffers) {
		// assert(iter.first->planes().size() == iter.second.size());
		// for (unsigned i = 0; i < iter.first->planes().size(); i++)
		for (auto& span : iter.second)
			munmap(span.data(), span.size());
	}
	m_mapped_buffers.clear();

	delete m_allocator;
	m_allocator = nullptr;
	m_configuration.reset();
	m_frame_buffers.clear();
}

void SyncCameraEncoderImpl::StartCamera() {
	// This makes all the Request objects that we shall need.
	makeRequests();

	if (m_options->framerate > 0) {
		int64_t frame_time = 1000000 / m_options->framerate; // in us
		uprintf("setting frame time: %lld\n", frame_time);
		m_controls.set(libcamera::controls::FrameDurationLimits,
					  libcamera::Span<const int64_t, 2>({ frame_time, frame_time }));
	}

	if(m_options->exposure != 0) {
		uprintf("setting exposure time: %lld\n", m_options->exposure);
		m_controls.set(libcamera::controls::ExposureTime, m_options->exposure);
		m_controls.set(libcamera::controls::AeEnable, false);
	} else {
		m_controls.set(libcamera::controls::AeEnable, true);
	}

	m_controls.set(libcamera::controls::AwbEnable, false);
	m_controls.set(libcamera::controls::AnalogueGain, 1.0);
	//m_controls.set(libcamera::controls::DigitalGain, 1);


	int metering_index{libcamera::controls::MeteringCentreWeighted};
	if (!m_controls.get(libcamera::controls::AeMeteringMode))
		m_controls.set(libcamera::controls::AeMeteringMode, metering_index);
	int exposure_index{libcamera::controls::ExposureNormal};
	if (!m_controls.get(libcamera::controls::AeExposureMode))
		m_controls.set(libcamera::controls::AeExposureMode, exposure_index);
	int awb_index{libcamera::controls::AwbAuto};
	if (!m_controls.get(libcamera::controls::AwbMode))
		m_controls.set(libcamera::controls::AwbMode, awb_index);
	if (!m_controls.get(libcamera::controls::ExposureValue))
		m_controls.set(libcamera::controls::ExposureValue, m_options->ev);
	if (!m_controls.get(libcamera::controls::Brightness))
		m_controls.set(libcamera::controls::Brightness, m_options->brightness);
	if (!m_controls.get(libcamera::controls::Contrast))
		m_controls.set(libcamera::controls::Contrast, m_options->contrast);
	if (!m_controls.get(libcamera::controls::Saturation))
		m_controls.set(libcamera::controls::Saturation, m_options->saturation);
	if (!m_controls.get(libcamera::controls::Sharpness))
		m_controls.set(libcamera::controls::Sharpness, m_options->sharpness);

	if (m_camera->start(&m_controls))
		throw std::runtime_error("failed to start camera");
	m_controls.clear();
	m_camera_started = true;

	m_camera->requestCompleted.connect(this, &SyncCameraEncoderImpl::requestComplete);

	for (std::unique_ptr<Request>& request : m_requests) {
		if (m_camera->queueRequest(request.get()) < 0)
			throw std::runtime_error("Failed to queue request");
	}

	auto sys_time = std::chrono::system_clock::now();
	auto std_time = std::chrono::steady_clock::now();
	int64_t sys = std::chrono::time_point_cast<std::chrono::microseconds>(sys_time).time_since_epoch().count();
	int64_t now = std::chrono::time_point_cast<std::chrono::microseconds>(std_time).time_since_epoch().count();
	m_time_base = sys - now;

	uprintf("Camera started!\n");
}

void SyncCameraEncoderImpl::StopCamera() {
	{
		// We don't want QueueRequest to run asynchronously while we stop the camera.
		std::lock_guard<std::mutex> lock(m_camera_stop_mutex);
		if (m_camera_started) {
			if (m_camera->stop())
				throw std::runtime_error("failed to stop camera");
			m_camera_started = false;
		}
	}

	if (m_camera)
		m_camera->requestCompleted.disconnect(this, &SyncCameraEncoderImpl::requestComplete);

	// An application might be holding a CompletedRequest, so queueRequest will get
	// called to delete it later, but we need to know not to try and re-queue it.
	m_completed_requests.clear();
	m_msg_queue.Clear();
	m_requests.clear();
	m_controls.clear(); // no need for mutex here
}

SyncCameraEncoderImpl::Msg SyncCameraEncoderImpl::Wait() {
	return m_msg_queue.Wait();
}

void SyncCameraEncoderImpl::queueRequest(CompletedRequest* completed_request) {
	BufferMap buffers(std::move(completed_request->buffers));

	// This function may run asynchronously so needs protection from the
	// camera stopping at the same time.
	std::lock_guard<std::mutex> stop_lock(m_camera_stop_mutex);

	// An application could be holding a CompletedRequest while it stops and re-starts
	// the camera, after which we don't want to queue another request now.
	bool request_found;
	{
		std::lock_guard<std::mutex> lock(m_completed_requests_mutex);
		auto it = m_completed_requests.find(completed_request);
		if (it != m_completed_requests.end()) {
			request_found = true;
			m_completed_requests.erase(it);
		}
		else
			request_found = false;
	}

	Request* request = completed_request->request;
	delete completed_request;
	assert(request);

	if (!m_camera_started || !request_found)
		return;

	for (auto const& p : buffers) {
		if (request->addBuffer(p.first, p.second) < 0)
			throw std::runtime_error("failed to add buffer to request in QueueRequest");
	}

	{
		std::lock_guard<std::mutex> lock(m_control_mutex);
		request->controls() = std::move(m_controls);
	}

	if (m_camera->queueRequest(request) < 0)
		throw std::runtime_error("failed to queue request");
}

void SyncCameraEncoderImpl::SetControls(ControlList& controls) {
	std::lock_guard<std::mutex> lock(m_control_mutex);
	m_controls = std::move(controls);
}

StreamInfo SyncCameraEncoderImpl::GetStreamInfo(Stream const* stream) const {
	StreamConfiguration const& cfg = stream->configuration();
	StreamInfo info;
	info.width = cfg.size.width;
	info.height = cfg.size.height;
	info.stride = cfg.stride;
	info.pixel_format = stream->configuration().pixelFormat;
	info.colour_space = stream->configuration().colorSpace;
	return info;
}

void SyncCameraEncoderImpl::setupCapture() {
	// First finish setting up the configuration.

	CameraConfiguration::Status validation = m_configuration->validate();
	if (validation == CameraConfiguration::Invalid)
		throw std::runtime_error("failed to valid stream configurations");
	else if (validation == CameraConfiguration::Adjusted)
		uprintf("Stream configuration adjusted\n");

	if (m_camera->configure(m_configuration.get()) < 0)
		throw std::runtime_error("failed to configure streams");
	uprintf("Camera streams configured\n");

	// Next allocate all the buffers we need, mmap them and store them on a free list.

	m_allocator = new FrameBufferAllocator(m_camera);
	for (StreamConfiguration& config : *m_configuration) {
		Stream* stream = config.stream();

		if (m_allocator->allocate(stream) < 0)
			throw std::runtime_error("failed to allocate capture buffers");

		for (const std::unique_ptr<FrameBuffer>& buffer : m_allocator->buffers(stream)) {
			// "Single plane" buffers appear as multi-plane here, but we can spot them because then
			// planes all share the same fd. We accumulate them so as to mmap the buffer only once.
			size_t buffer_size = 0;
			for (unsigned i = 0; i < buffer->planes().size(); i++) {
				const FrameBuffer::Plane& plane = buffer->planes()[i];
				buffer_size += plane.length;
				if (i == buffer->planes().size() - 1 || plane.fd.get() != buffer->planes()[i + 1].fd.get()) {
					void* memory = mmap(NULL, buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0);
					m_mapped_buffers[buffer.get()].push_back(
						libcamera::Span<uint8_t>(static_cast<uint8_t*>(memory), buffer_size));
					buffer_size = 0;
				}
			}
			m_frame_buffers[stream].push(buffer.get());
		}
	}
	uprintf("Buffers allocated and mapped\n");

	// The requests will be made when StartCamera() is called.
}

void SyncCameraEncoderImpl::makeRequests() {
	auto free_buffers(m_frame_buffers);
	while (true) {
		for (StreamConfiguration& config : *m_configuration) {
			Stream* stream = config.stream();
			if (stream == m_configuration->at(0).stream()) {
				if (free_buffers[stream].empty()) {
					uprintf("Requests created\n");
					return;
				}
				std::unique_ptr<Request> request = m_camera->createRequest();
				if (!request)
					throw std::runtime_error("failed to make request");
				m_requests.push_back(std::move(request));
			}
			else if (free_buffers[stream].empty())
				throw std::runtime_error("concurrent streams need matching numbers of buffers");

			FrameBuffer* buffer = free_buffers[stream].front();
			free_buffers[stream].pop();
			if (m_requests.back()->addBuffer(stream, buffer) < 0)
				throw std::runtime_error("failed to add buffer to request");
		}
	}
}

void SyncCameraEncoderImpl::requestComplete(Request* request) {
	if (request->status() == Request::RequestCancelled) {
		// If the request is cancelled while the camera is still running, it indicates
		// a hardware timeout. Let the application handle this error.
		if (m_camera_started)
			m_msg_queue.Post(Msg(MsgType::Timeout));
		return;
	}

	CompletedRequest* r = new CompletedRequest(m_sequence++, request);
	CompletedRequestPtr payload(r, [this](CompletedRequest* cr) { this->queueRequest(cr); });
	{
		std::lock_guard<std::mutex> lock(m_completed_requests_mutex);
		m_completed_requests.insert(r);
	}

	m_msg_queue.Post(Msg(MsgType::RequestComplete, std::move(payload)));
}
void SyncCameraEncoderImpl::StartEncoder() {
	assert(!m_encoder);
	StreamInfo info = GetStreamInfo(m_video_stream);
	if (!info.width || !info.height || !info.stride)
		throw std::runtime_error("video steam is not configured");

	m_encoder = new H264Encoder(GetOptions(), info);
	m_encoder->SetOutputReadyCallback(m_encode_output_ready_callback);
}

void SyncCameraEncoderImpl::EncodeBuffer(const CompletedRequestPtr& completed_request, Stream* stream) {
	assert(m_encoder);
	const FrameBuffer* buffer = completed_request->buffers[stream];

	auto found = m_mapped_buffers.find(buffer);
	assert(found != m_mapped_buffers.end());
	libcamera::Span span = found->second[0];
	const uint8_t* buffer_data = reinterpret_cast<const uint8_t*>(span.data());
	size_t buffer_size = span.size();

	int64_t timestamp_ns = *completed_request->metadata.get(libcamera::controls::SensorTimestamp);
	int64_t timestamp_us = timestamp_ns/1000 + m_time_base;

	m_encoder->EncodeBuffer(buffer->planes()[0].fd.get(), buffer_data, buffer_size, timestamp_us);

	if(m_metadata_ready_callback)
		m_metadata_ready_callback(completed_request->metadata);

	if(m_bufferdata_ready_callback)
		m_bufferdata_ready_callback(buffer_data, buffer_size, timestamp_us);
}

void run(SyncCameraEncoder* a, BufferdataReadyCallback buf_cb, EncodeOutputReadyCallback enc_cb, MetadataReadyCallback meta_cb) {
	auto& app = *reinterpret_cast<SyncCameraEncoderImpl*>(a);
	Options const *options = app.GetOptions();
	app.SetBufferdataReadyCallback(buf_cb);
	app.SetEncodeOutputReadyCallback(enc_cb);
	app.SetMetadataReadyCallback(meta_cb);

	app.OpenCamera();
	uprintf("codec = %s\n", options->codec.c_str());
	app.ConfigureVideo();
	app.StartEncoder();
	app.StartCamera();
	auto start_time = std::chrono::high_resolution_clock::now();

	int64_t time_base = 0;
	libcamera::ControlList lst;

	for (unsigned int count = 0; ; count++) {
		auto now = std::chrono::high_resolution_clock::now();
		bool timeout = options->timeout && (now - start_time > std::chrono::milliseconds(options->timeout));
		//timeout = false;
		if (timeout) {
			app.StopEncoder();
			app.StopCamera(); // stop complains if encoder very slow to close
			break;
		}

		const int64_t vblank = 1000000 / options->framerate;
		SyncCameraEncoderImpl::Msg msg = app.Wait();
		if (msg.type == SyncCameraEncoderImpl::MsgType::Timeout) {
			app.StopCamera();
			app.StartCamera();
			continue;
		}
		if (msg.type != SyncCameraEncoderImpl::MsgType::RequestComplete) {
			throw std::runtime_error("unrecognised message!");
		}

		const CompletedRequestPtr& completed_request = msg.payload;
		app.EncodeBuffer(completed_request, app.VideoStream());

		if(options->synchronize) {
			if(count == 0) {
				auto sys_time = std::chrono::system_clock::now();
				auto std_time = std::chrono::steady_clock::now();
				int64_t sys = std::chrono::time_point_cast<std::chrono::microseconds>(sys_time).time_since_epoch().count();
				int64_t now = std::chrono::time_point_cast<std::chrono::microseconds>(std_time).time_since_epoch().count();
				time_base = sys - now;

				lst.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({vblank, vblank}));
				app.SetControls(lst);
			}

			int64_t ts = *completed_request->metadata.get(libcamera::controls::SensorTimestamp);
			int64_t shutter_time_since_epoch_us = ts/1000LL + time_base + options->synchronize_offset;
			int64_t dtime = count * vblank;
			//dtime += vblank/2;

			int64_t drift = shutter_time_since_epoch_us - dtime;
			drift = drift % vblank;
			if(drift > vblank/2)
				drift -= vblank;

			int64_t ablank = vblank;
			if(drift < -10 || drift > 10)
				ablank = vblank - drift/10;
			if(drift > -10 && drift < 10)
				ablank = vblank;
			//uprintf("%10u %25lld %lld %lld\n", count, shutter_time_since_epoch_us, drift, ablank);
			//uprintf("dtime: %lld. tcam: %lld, drift : %lld, ablank: %lld\n", dtime, shutter_time_since_epoch_us, drift, ablank);

			lst.set(libcamera::controls::FrameDurationLimits, libcamera::Span<const int64_t, 2>({ablank, ablank}));
			app.SetControls(lst);
		}
	}
}

SyncCameraEncoder* CreateSyncCameraEncoder(const Dict* dict) {
	return new SyncCameraEncoderImpl;
}
void DestroySyncCameraEncoder(SyncCameraEncoder* p) {
	delete p;
}

#else

SyncCameraEncoder* CreateSyncCameraEncoder(const Dict* dict) { return nullptr; }
void DestroySyncCameraEncoder(SyncCameraEncoder* p) { }
void run(SyncCameraEncoder* a, BufferdataReadyCallback buf_cb, EncodeOutputReadyCallback enc_cb, MetadataReadyCallback meta_cb) {}

#endif
