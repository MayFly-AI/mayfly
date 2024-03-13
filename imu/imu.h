#pragma once
#include "shared/math.h"
#include <stdint.h>
#include <functional>

typedef std::function<void(const V3&, const V3&, const float&,
			   const uint32_t&, const uint64_t&)> IMUOutputReadyCallback;

void ReadIMU(IMUOutputReadyCallback, int freq);
