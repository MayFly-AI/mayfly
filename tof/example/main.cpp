#include "arducam_tof.h"
#include <thread>
#include "shared/queue.h"
#include "shared/thread.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//#define USE_THREAD_AFFINITY
//#define THREAD_AFFINITY_MAIN (1<<0)

cv::Rect selectRect(240/2-4, 180/2-4, 8, 8);

struct Frame {
	std::vector<uint8_t> m_data;
	uint64_t m_timeCapture;
	int m_frameIndex;
};

// Needs to be compiled with optimization to work properly

int main(int argc, char **argv) {
	bool encode=true;
	int frameIndex=0;
	FixedQueueMT<Frame,10> q;

	std::thread thread=std::thread([&]{
#ifdef USE_THREAD_AFFINITY
		SetSelfAffinityMask(THREAD_AFFINITY_MAIN);
#endif
		auto tof_cb=[&](const uint8_t* data, size_t size, uint64_t timeCapture) {
			if(!q.Push([&](Frame* element)->void {
				element->m_data.assign(data, data+size);
				element->m_timeCapture = timeCapture;
				element->m_frameIndex = frameIndex;
			})) {
				uprintf("ArducamToF queue full\n");
			}
			frameIndex++;
		};
		ArducamToF tof(tof_cb, encode);
		tof.MainLoop();
	});

	while(true) {
		int cnt=0;
		while (!q.Empty()) {
			q.Pop([&](Frame *element) -> void {
				if(!element) {
					FATAL("no element\n");
				}
				void* data = element->m_data.data();
				std::vector<uint16_t> decoded;
				if(encode) {
					DecoderDCT16 decoder;
					int widthOut;
					int heightOut;
					if(!decoder.DecodeQuant16(&decoded,&widthOut,&heightOut,element->m_data)) {
						FATAL("Decoding failed\n");
					}
					data = decoded.data();
				}

				cv::Mat phase_frame(180, 240, CV_16U, data);
//				printf("select Rect distance: %f\n", cv::mean(phase_frame(selectRect)).val[0]);
				double minVal, maxVal;
				cv::minMaxLoc(phase_frame, &minVal, &maxVal);
//				printf("min=%f max=%f\n",minVal,maxVal);
				if(encode) {
					phase_frame.convertTo(phase_frame, CV_8U, 255.0 / 4095., 0);
				} else {
					phase_frame.convertTo(phase_frame, CV_8U, 255.0 / 65535., 0);
				}
//				cv::imshow("phase", phase_frame);
//				if (cv::waitKey(1) == 27)
//					exit(1);
				});
			cnt += 1;
		}
		if (cnt > 1) {
			uprintf("DataSouceToF::GetFrameData more than 1 frame popped");
		}
	}
}
