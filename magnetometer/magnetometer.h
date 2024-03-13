#pragma once
#include "shared/math.h"
#include <stdint.h>
#include <functional>

#if ENABLE_LIBGPIOD
#include "bmm350.h"
#include "bmm350_oor.h"
#include "coines.h"
#include "common.h"
#endif

typedef std::function<void(const V3&,float temp,const uint64_t&)> MagnetometerOutputReadyCallback;

void ReadMagnetometer(MagnetometerOutputReadyCallback, int freq);
