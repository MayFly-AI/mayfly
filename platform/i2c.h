#pragma once

#include <stdint.h>

int InitializeI2C(const char* idname);
void BeginDeviceI2C(int file,uint8_t deviceId);
void CloseI2C(int file);
void WriteToAddressI2C(int file,uint8_t address,uint8_t value);
int ReadFromAddressI2C(int file,uint8_t* buffer,uint8_t address,uint8_t byteSize);
void LockI2C();
void UnlockI2C();
void WriteToAddressI2C(int file,uint8_t deviceId,uint8_t address,uint8_t value);
int ReadFromAddressI2C(int file,uint8_t deviceId,uint8_t* buffer,uint8_t address,uint8_t byteSize);
