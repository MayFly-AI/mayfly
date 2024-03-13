#pragma once

#include <vector>
#include <stdint.h>

bool WriteJPGToMemory(std::vector<char>* data,uint8_t* pixels,int m_width,int m_height,int channels);
