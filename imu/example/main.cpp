#include "shared/misc.h"
#include "shared/file.h"
#include "shared/std_ext.h"

#include "bmi088.h"
#include "imu.h"

int main(int argc,char *argv[]) {
	auto cb=[&](const V3& acc, const V3& rads, const float& temp, const uint32_t& dt, const uint64_t& t) {
		uprintf("acc  %f %f %f\n", acc[0], acc[1], acc[2]);
		uprintf("rads %f %f %f\n", rads[0], rads[1], rads[2]);
		uprintf("temperature %f\n", temp);
		uprintf("dt %lu\n", dt);
		uprintf("time %llu\n", t);
	};
	ReadIMU(cb,50);
}
