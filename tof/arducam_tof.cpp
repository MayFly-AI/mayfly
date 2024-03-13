#include "arducam_tof.h"

#ifdef PLATFORM_RPI
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <queue>
#include <mutex>
#include <cstring>
#include <math.h>
#include <linux/videodev2.h>
#include "shared/math.h"
#include "shared/medianfilter.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

ArducamToF::ArducamToF(ToFOutputReadyCallback cb, bool encode, const float& thresh) {
	m_encode = encode;
	m_amplitudeThresh = thresh;
	m_encoder = new EncoderDCT16();
	m_callback = cb;
	m_phase = new float[m_width*m_height];
	m_ampli = new float[m_width*m_height];
	m_frameBuf = new int16_t[4*m_width*m_height];
	m_phaseMasked.resize(m_width*m_height);
	m_filtered.resize(m_width*m_height);
	auto sys_time = std::chrono::system_clock::now();
	auto std_time = std::chrono::steady_clock::now();
	int64_t sys = std::chrono::time_point_cast<std::chrono::microseconds>(sys_time).time_since_epoch().count();
	int64_t now = std::chrono::time_point_cast<std::chrono::microseconds>(std_time).time_since_epoch().count();
	m_clockOffset = sys-now;
	SetupDevice();
	StartCapturing();
}

ArducamToF::~ArducamToF() {
	StopCapturing();
	CleanupDevice();
	delete[] m_phase;
	delete[] m_ampli;
	delete[] m_frameBuf;
	delete m_encoder;
}


static void errno_exit(const char *s) {
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

static int xioctl(int fh, unsigned long request, void *arg) {
	int r;
	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);
	return r;
}

inline float atan2_approx(float y, float x) {
	float ax=fabsf(x);
	float ay=fabsf(y);
	float den=std::max(ax,ay);
	if(den==0.0f) return 0.0f;
	float a=std::min(ax,ay)/den;
	float s=a*a;
	float r=((-0.0464964749f*s+0.15931422f)*s-0.327622764f)*s*a+a;
	if(ay>ax){r=1.57079637f-r;}
	if(x<0) {r=3.14159274f-r;}
	if(y<0) {r=-r;}
	return r;
}


void GetPhaseAndAmplitude(float f_mod, int size, int16_t* input, float* phase_ptr, float* amplitude_ptr) {
	int16_t* plane0 = input + 0*size;
	int16_t* plane1 = input + 1*size;
	int16_t* plane2 = input + 2*size;
	int16_t* plane3 = input + 3*size;

	const float tau=2.0f*M_PI;
	const float c1=3e8f/(2.0f*tau*f_mod);
	for(int i=0;i<size;++i) {
		float y = (float)(  ((short)(((int)plane2[i]) << 5) >> 1)  -  ((short)(((int)plane0[i]) << 5) >> 1)  );
		float x = (float)(  ((short)(((int)plane3[i]) << 5) >> 1)  -  ((short)(((int)plane1[i]) << 5) >> 1)  );
		float angle = atan2_approx(y,x);
		float l2 = y*y + x*x;
		if (angle < 0.0f) {
			angle += tau;
		}
		phase_ptr[i] = c1*angle;
		amplitude_ptr[i] = 0.5f*sqrtf(l2);
	}
}

void ArducamToF::GetMaskedDepth(uint16_t* depth_masked, const float* phase_image_ptr, const float* amplitude_image_ptr) {
	for (auto i = 0; i < m_width*m_height; i++) {
		if(amplitude_image_ptr[i] > m_amplitudeThresh) {
			if(m_encode) {
				uint16_t val = (phase_image_ptr[i]*(4095./4.)); // 12 bit
				uint16_t val2=CLAMP(0, val, 4095);
				depth_masked[i] = val2;
			} else {
				uint16_t val = (phase_image_ptr[i]*(65535./4.)); // 16 bit
				uint16_t val2=CLAMP(0, val, 65535);
				depth_masked[i] = val2;
			}
		}
		else {
			depth_masked[i] = 0;
		}
	}
}

void ArducamToF::ProcessImage(const void* p, int size, int subFrame) {
	int pixel_count = m_width*m_height;
	assert(size == pixel_count*(int)sizeof(int16_t));

	// flip image vertically (using v4l2 control V4L2_CID_VFLIP has no effect)
	const int16_t* rrow =(const int16_t*)p;
	int16_t* wrow = m_frameBuf + (subFrame+1)*pixel_count - m_width;
	for(int i=0;i<m_height;++i) {
		memcpy(wrow, rrow, m_width*sizeof(int16_t));
		wrow -= m_width;
		rrow += m_width;
	}

	if(subFrame==3) {
		GetPhaseAndAmplitude(3.75e7, pixel_count, m_frameBuf, m_phase, m_ampli);
		// ^ time: 2300us
		GetMaskedDepth(m_phaseMasked.data(), m_phase, m_ampli);
		// ^ time:  345us
		MedianFilter3x3(m_filtered.data(),m_phaseMasked.data(),m_width,m_height);
		// ^ time:  202us
	}
}

int ArducamToF::ReadFrame() {
	struct v4l2_buffer buf;
	CLEAR(buf);

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	if (-1 == xioctl(m_fd, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
			case EAGAIN:
				return 0;
			case EIO:
				/* Could ignore EIO, see spec. */
				/* fall through */
			default:
				errno_exit("VIDIOC_DQBUF");
		}
	}

	assert(buf.index < m_nBuffers);
	//uprintf("buffer time: %ld %ld\n", buf.timestamp.tv_sec, buf.timestamp.tv_usec);

	struct timespec now;
	static struct timespec before;
	static struct timeval before_buf;
	clock_gettime(CLOCK_MONOTONIC, &now);
	double frameDelta = ((now.tv_sec - before.tv_sec)*1e6 + (now.tv_nsec - before.tv_nsec)/1000.0)/1000.0;
	//double capDelta = ((now.tv_sec - buf.timestamp.tv_sec)*1e6 + (now.tv_nsec/1000 - buf.timestamp.tv_usec))/1000.0;
	double bufDelta = ((buf.timestamp.tv_sec-before_buf.tv_sec)*1e6 + (buf.timestamp.tv_usec - before_buf.tv_usec))/1000.;
	//uprintf("frame delta %8.2fms capt. delta %8.2fms bufDelta %8.2fms\n", frameDelta, capDelta, bufDelta);
	before = now;
	before_buf = buf.timestamp;

	static int subFrame = 0;
	if(bufDelta > 20. && subFrame!=3){
		uprintf("Resetting subFrame to 0 even though subFrame is not 3. subFrame %d - bufDelta %f\n",subFrame,bufDelta);
	}
	if(bufDelta > 20.0 || subFrame == 3) {
		subFrame = 0;
	} else {
		subFrame++;
	}
	if(subFrame==0) {
		if(frameDelta < 20 || frameDelta > 30) {
			uprintf("ToF maybe not running properly: subFrame %d frameDelta %f should be approx 22\n",subFrame,frameDelta);
		}
	} else {
		if(frameDelta > 5) {
			uprintf("ToF maybe not running properly: subFrame %d frameDelta %f should be approx 4\n",subFrame,frameDelta);
		}
	}
	uint64_t tstamp = buf.timestamp.tv_sec*1e6 + buf.timestamp.tv_usec;
	tstamp += m_clockOffset;

	ProcessImage(m_buffers[buf.index].m_start, buf.bytesused, subFrame); // Maybe dont do too much work here: We need QBUF to be called fast enough?
	// ^ time: 212us (subFrame 0-2), 3100us (subFrame 3)

	if (-1 == xioctl(m_fd, VIDIOC_QBUF, &buf))
		errno_exit("VIDIOC_QBUF");

	if(subFrame == 3) {
		struct timespec now;
		static struct timespec prev;
		clock_gettime(CLOCK_MONOTONIC, &now);
		prev = now;
		if(m_encode) {
			m_encoded.clear(); // important to clear
			m_encoder->Encode(&m_encoded, m_filtered, m_width, m_height);
			// ^ time: 5100us (not filtered) 3600us (filtered)
			m_callback(m_encoded.data(), sizeof(uint8_t)*m_encoded.size(), tstamp);
		} else {
			m_callback((const uint8_t*)m_filtered.data(), sizeof(uint16_t)*m_height*m_width, tstamp);
		}
	}
	return 1;
}

void ArducamToF::MainLoop() {
	for(;;) {
		for (;;) {
			fd_set fds;
			FD_ZERO(&fds);
			FD_SET(m_fd, &fds);

			struct timeval tv;
			tv.tv_sec = 0;
			tv.tv_usec = 200000;

			int r = select(m_fd + 1, &fds, NULL, NULL, &tv);
			if(-1 == r) {
				if (EINTR == errno)
					continue;
				errno_exit("select");
			}
			if(0 == r) {
				fprintf(stderr, "select timeout\n");
				exit(EXIT_FAILURE);
			}

			if(ReadFrame()) {
				break;
			}
			/* EAGAIN - continue select loop. */
		}
	}
}

void ArducamToF::StartCapturing(void) {
	unsigned int i;
	enum v4l2_buf_type type;

	for (i = 0; i < m_nBuffers; ++i) {
		struct v4l2_buffer buf;
		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if (-1 == xioctl(m_fd, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(m_fd, VIDIOC_STREAMON, &type))
		errno_exit("VIDIOC_STREAMON");

	unsigned int CTRL_RANGE = V4L2_CTRL_CLASS_USER+0x1901;
	//unsigned int CTRL_EXPOS = V4L2_CTRL_CLASS_USER+0x0911;
	v4l2_control ctrl = {CTRL_RANGE, 0};
	if (-1 == xioctl(m_fd, VIDIOC_S_CTRL, &ctrl))
	  errno_exit("TOF magic - set control failed");
}

void ArducamToF::StopCapturing(void) {
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(m_fd, VIDIOC_STREAMOFF, &type))
		errno_exit("VIDIOC_STREAMOFF");
}

void ArducamToF::InitMMAP(void) {
	struct v4l2_requestbuffers req;
	CLEAR(req);
	req.count = 8;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(m_fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
					"memory mapping\n", m_devName);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}
	//uprintf("needs %i buffers\n", req.count);
	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n",
				m_devName);
		exit(EXIT_FAILURE);
	}

	m_buffers = (struct Buffer*)calloc(req.count, sizeof(*m_buffers));
	if (!m_buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}
	for (m_nBuffers = 0; m_nBuffers < req.count; ++m_nBuffers) {
		struct v4l2_buffer buf;
		CLEAR(buf);
		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = m_nBuffers;

		if (-1 == xioctl(m_fd, VIDIOC_QUERYBUF, &buf))
			errno_exit("VIDIOC_QUERYBUF");

		m_buffers[m_nBuffers].m_length = buf.length;
		m_buffers[m_nBuffers].m_start =
			mmap(NULL /* start anywhere */,
					buf.length,
					PROT_READ | PROT_WRITE /* required */,
					MAP_SHARED /* recommended */,
					m_fd, buf.m.offset);

		if (MAP_FAILED == m_buffers[m_nBuffers].m_start)
			errno_exit("mmap");
	}
}

void ArducamToF::SetupDevice(void) {
	struct stat st;
	if (-1 == stat(m_devName, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n",
				m_devName, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", m_devName);
		exit(EXIT_FAILURE);
	}
	m_fd = open(m_devName, O_RDWR /* required */ | O_NONBLOCK, 0);
	if (-1 == m_fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
				m_devName, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

	struct v4l2_format fmt;
	int inp = 0;
	if (-1 == xioctl(m_fd, VIDIOC_S_INPUT, &inp)) {
		fprintf(stderr, "inp not working\n");
		exit(EXIT_FAILURE);
	}

	CLEAR(fmt);
	fprintf(stderr, "Setting format\r\n");
	fmt.type                 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.pixelformat  = V4L2_PIX_FMT_Y12;
	fmt.fmt.pix.width        = m_width;
	fmt.fmt.pix.height       = m_height;
	fmt.fmt.pix.field        = V4L2_FIELD_NONE;
	fmt.fmt.pix.bytesperline = 0;//480;
	fmt.fmt.pix.sizeimage    = 0;//fmt.fmt.pix.bytesperline*fmt.fmt.pix.height;

	if (-1 == xioctl(m_fd, VIDIOC_TRY_FMT, &fmt))
		errno_exit("VIDIOC_TRY_FMT");
	/* Note VIDIOC_S_FMT may change width and height. */
	//uprintf("bytesperline: %d, sizeimage: %d\n", fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);
	if (-1 == xioctl(m_fd, VIDIOC_S_FMT, &fmt))
		errno_exit("VIDIOC_S_FMT");
	InitMMAP();
}

void ArducamToF::CleanupDevice(void) {
	for (unsigned int i = 0; i < m_nBuffers; ++i)
		if (-1 == munmap(m_buffers[i].m_start, m_buffers[i].m_length))
			errno_exit("munmap");
	free(m_buffers);
	if (-1 == close(m_fd))
		errno_exit("close");
	m_fd = -1;
}

#else

// empty implementations for non RPi
ArducamToF::ArducamToF(ToFOutputReadyCallback cb, bool encode, const float& thresh) { FATAL("ArducamToF only works in RPi\n"); }
ArducamToF::~ArducamToF() { }
void ArducamToF::MainLoop() {}

#endif
