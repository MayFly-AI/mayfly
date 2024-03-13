#include <stdint.h>
#include <string.h>
#include "shared/misc.h"
#include "shared/crc32.h"

int main() {
	const char* buffer = "Hello World!";
	int n = (int)strlen(buffer);
	uint32_t crc = CRC32(buffer, n, 0);

	uprintf("0x%8x\n", crc);
}
