// Copyright (C) 2022 Storj Labs, Inc.
// See LICENSE for copying information.
//
// Vector add-multiply with no hardware acceleration.
//
// (C) 1997-1998 Luigi Rizzo (luigi@iet.unipi.it)
// (C) 2009,2010,2021 Jack Lloyd
// (C) 2011 Billy Brumley (billy.brumley@aalto.fi)
// (C) 2016-2017 Vivint, Inc.
// (C) 2022 Storj Labs, Inc.
//
// Based on the addmul() implementation in Botan. Botan is released under the
// Simplified BSD License (see LICENSE).
//
// Portions of this code are also subject to the terms of the MIT License:
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "fec.h"
#include "tables.h"

//namespace infectious {

// a lot of constants are needed here, and the 'magic numbers' all make sense
// in context.
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

//
// addmul() computes z[] = z[] + x[] * y
//
void FEC::addmul(
	uint8_t* z_begin, const uint8_t* z_end,
	const uint8_t* x, uint8_t y
) {
	if (y == 0) {
		return;
	}

	long size = (long)(z_end - z_begin);
	auto* z = z_begin;
	const auto& gf_mul_y = gf_mul_table[y];

	// first align z to 16 bytes
	while (size > 0 && (reinterpret_cast<uintptr_t>(z) % 16) != 0) {
		z[0] ^= gf_mul_y[x[0]];
		++z;
		++x;
		size--;
	}

#if defined(INFECTIOUS_HAS_VPERM)
	if (size >= 16 && CPUID::has_vperm()) {
		const size_t consumed = addmul_vperm(z, x, y, static_cast<int>(size));
		z += consumed;
		x += consumed;
		size -= static_cast<long>(consumed);
	}
#endif
#if defined(INFECTIOUS_HAS_SSE2)
	if (size >= 64 && CPUID::has_sse2()) {
		const size_t consumed = addmul_sse2(z, x, y, static_cast<int>(size));
		z += consumed;
		x += consumed;
		size -= consumed;
	}
#endif

	while (size >= 16) {
		z[0] ^= gf_mul_y[x[0]];
		z[1] ^= gf_mul_y[x[1]];
		z[2] ^= gf_mul_y[x[2]];
		z[3] ^= gf_mul_y[x[3]];
		z[4] ^= gf_mul_y[x[4]];
		z[5] ^= gf_mul_y[x[5]];
		z[6] ^= gf_mul_y[x[6]];
		z[7] ^= gf_mul_y[x[7]];
		z[8] ^= gf_mul_y[x[8]];
		z[9] ^= gf_mul_y[x[9]];
		z[10] ^= gf_mul_y[x[10]];
		z[11] ^= gf_mul_y[x[11]];
		z[12] ^= gf_mul_y[x[12]];
		z[13] ^= gf_mul_y[x[13]];
		z[14] ^= gf_mul_y[x[14]];
		z[15] ^= gf_mul_y[x[15]];

		x += 16;
		z += 16;
		size -= 16;
	}

	// Clean up the trailing pieces
	for (long i = 0; i < size; ++i) {
		z[i] ^= gf_mul_y[x[i]];
	}
}

auto addmul_provider() -> std::string {
#if defined(INFECTIOUS_HAS_VPERM)
	if (CPUID::has_vperm()) {
#if defined(INFECTIOUS_TARGET_CPU_IS_X86_FAMILY)
		return "ssse3";
#elif defined(INFECTIOUS_TARGET_CPU_IS_ARM_FAMILY)
		return "neon";
#else
		return "vperm/unknown";
#endif
	}
#endif
#if defined(INFECTIOUS_HAS_SSE2)
	if (CPUID::has_sse2()) {
		return "sse2";
	}
#endif
	return "none";
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

//} // namespace infectious
