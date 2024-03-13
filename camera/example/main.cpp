#include <string>
#include <chrono>
#include "synccameraencoder.h"
#include "shared/misc.h"
#include "shared/dict.h"
#include "shared/std_ext.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdio.h>
#include <vector>
#include <netdb.h>

// receive eg. by running this on remove address:
// ffplay -fast -framedrop -fflags nobuffer -i udp://0.0.0.0:8888
static int main_send(int argc, const char* argv[]) {
	bool nalu_hack = true;
	const char* remote_addr = "10.10.10.40";
	if(argc>1) remote_addr = argv[1];

	int port = 8888;
	if(argc>2) port = atoi(argv[2]);

	uprintf("sending to %s at port %d\n", remote_addr, port);

	struct sockaddr_in saddr_ = {};
	saddr_.sin_family = AF_INET;
	saddr_.sin_port = htons(port);
	socklen_t sockaddr_in_size_ = sizeof(sockaddr_in);

	struct hostent* h = gethostbyname(remote_addr);
	//uprintf("host name: %s\n", h->h_name);
	memcpy( (char*)&saddr_.sin_addr.s_addr, h->h_addr_list[0], h->h_length);
	saddr_.sin_family = h->h_addrtype;

	int fd =socket(AF_INET, SOCK_DGRAM, 0);
	const struct sockaddr* saddr_ptr_ = (const sockaddr*)&saddr_;

	const int MAX_UDP_SIZE = 1448;
	//const int MAX_UDP_SIZE = 508;
	std::vector<char> data;

	int num_frames = 0;
	SyncCameraEncoder* app = CreateSyncCameraEncoder(nullptr);
	auto enc_cb = [&](void* data, size_t size, uint64_t captureTime) {
		if (num_frames > 6000) {
			return;
		}
		int remain = size;
		char* start = (char*)data;
		int chunks = 0;
		bool nalu_sent = false;
		while(remain) {
			int send_size = remain > MAX_UDP_SIZE ? MAX_UDP_SIZE : remain;

			if(nalu_hack && nalu_sent) {
				start += 4;
				send_size -= 4;
				nalu_sent = false;
			}
			sendto(fd, start, send_size, 0, saddr_ptr_, sockaddr_in_size_);
			remain -= send_size;
			start += send_size;
			if (nalu_hack && remain == 0) {
				const char* nalu = "\00\00\00\01";
				sendto(fd, nalu, 4, 0, saddr_ptr_, sockaddr_in_size_);
				nalu_sent = true;
			}
			chunks++;
		}
		uprintf("frame %i, size %i, udp pkgs: %i\n", num_frames, size, chunks);

		auto name = stdx::format_string("/dev/shm/pkg_%08d.bin", num_frames);
		FILE* fp = fopen(name.c_str(), "wb");
		fwrite(data, size,1,fp);
		fclose(fp);

		num_frames++;
	};

	run(app, nullptr, enc_cb, nullptr);
	DestroySyncCameraEncoder(app);
	return 0;
}

#include <libcamera/controls.h>
#include "options.h"
static int main_test(int argc, const char* argv[]) {
	FILE* fp_frame = fopen("/dev/shm/recording.h264", "wb");
	FILE* fp_meta = fopen("/dev/shm/recording.txt", "w");

	int max_frames = 2*60*30;
	int num_frames = 0;
	SyncCameraEncoder* app = CreateSyncCameraEncoder(nullptr);
	Options* opt = app->GetOptions();

	printf("# %16s%18s\n", "callback", "sensor");

	auto enc_cb = [&](void* data, size_t size, uint64_t time) {
		auto sys_time = std::chrono::system_clock::now();
		int64_t now = std::chrono::time_point_cast<std::chrono::microseconds>(sys_time).time_since_epoch().count();
		//printf("encoder callback time: %lld , %llu (%5.2f)\n", now, time, (now-time)/1000.0);
		//printf("%18lld%18llu\n", now, time);
		printf("%12.2f ms - %zu bytes\n", (now - time)/1000.0, size);
		//printf("%18" PRId64 "%18" PRIu64 "\n", now, time);

		if(num_frames < max_frames) {
			fwrite(data, size, 1, fp_frame);
		} else {
			opt->timeout = 1;
		}
		num_frames++;
	};

	auto meta_cb = [&](const libcamera::ControlList& metadata) {
		const libcamera::ControlIdMap* id_map = metadata.idMap();
		for (const auto&[id, val] : metadata) {
			fprintf(fp_meta, "%s = %s\n", id_map->at(id)->name().c_str(), val.toString().c_str());
		}
		fprintf(fp_meta,"\n");
	};
	run(app, nullptr, enc_cb, meta_cb);
	DestroySyncCameraEncoder(app);

	fclose(fp_frame);
	fclose(fp_meta);
	return 0;
}

int main(int argc, const char* argv[]) {
	return main_test(argc, argv);
	//return main_send(argc, argv);
}
