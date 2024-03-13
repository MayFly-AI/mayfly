#pragma once

#include "shared/misc.h"
#include <vector>
//#include "jpg.h"

class Huffman;

class EncoderDCT16 {
	public:
		void ForwardDCTBlockComponent(int* component);
		void EncodeBlockComponent(class BitWriter* bitWriter,Huffman* huffmanDC,Huffman* huffmanAC,int* component,int* previousDC);
		void RegisterBlockComponent(Huffman* huffmanDC,Huffman* huffmanAC,int* component,int* previousDC);
		int Encode(std::vector<uint8_t>* outStream,const std::vector<uint16_t>& pixels,int width,int height);
		
};

class BitReader;

class DecoderDCT16 {
	public:
		void DecodeBlockComponent(int* previousDC,BitReader* bitReader,int* component,Huffman* huffmanDC,Huffman* huffmanAC);
		void InverseDCTBlockComponent(int* const component);
		bool DecodeQuant16(std::vector<uint16_t>* data,int* widthOut,int* heightOut,const std::vector<uint8_t>& encoded);
};
