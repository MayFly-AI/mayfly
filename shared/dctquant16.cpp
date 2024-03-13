#include <stdio.h>
#include <unordered_map>
#include <vector>
#include <queue>
#include <cmath>
#include <string.h>

//#define _USE_MATH_DEFINES

#include "shared/types.h"
#include "shared/misc.h"
#include "shared/math.h"
#include "dctquant16.h"

#define MAXCOEFFLENGTH 16

#define MAX_COEFF_LENGTH_BITS 6

struct QuantizationTable {
	uint32_t table[64]={0};
};

struct HuffmanTable {
	uint16_t offsets[30]={0};
	uint16_t symbols[400]={0};
	uint32_t codes[400]={0};
};

struct Block {
	int y[64]={0};
};

const uint8_t zigZagMap[]={
	0, 1,8,16,9,2,3,10,
	17,24,32,25,18,11,4,5,
	12,19,26,33,40,48,41,34,
	27,20,13,6,7,14,21,28,
	35,42,49,56,57,50,43,36,
	29,22,15,23,30,37,44,51,
	58,59,52,45,38,31,39,46,
	53,60,61,54,47,55,62,63
};

// IDCT scaling factors
const float m0=2.0f*cosf(1.0f/16.0f*2.0f*PI);
const float m1=2.0f*cosf(2.0f/16.0f*2.0f*PI);
const float m3=2.0f*cosf(2.0f/16.0f*2.0f*PI);
const float m5=2.0f*cosf(3.0f/16.0f*2.0f*PI);
const float m2=m0-m5;
const float m4=m0+m5;

const float s0=cosf(0.0f/16.0f*PI)/(float)std::sqrt(8);
const float s1=cosf(1.0f/16.0f*PI)/2.0f;
const float s2=cosf(2.0f/16.0f*PI)/2.0f;
const float s3=cosf(3.0f/16.0f*PI)/2.0f;
const float s4=cosf(4.0f/16.0f*PI)/2.0f;
const float s5=cosf(5.0f/16.0f*PI)/2.0f;
const float s6=cosf(6.0f/16.0f*PI)/2.0f;
const float s7=cosf(7.0f/16.0f*PI)/2.0f;

const QuantizationTable quant50={
	{
		16, 11, 10, 16, 24, 40, 51, 61,
		12, 12, 14, 19, 26, 58, 60, 55,
		14, 13, 16, 24, 40, 57, 69, 56,
		14, 17, 22, 29, 51, 87, 80, 62,
		18, 22, 37, 56, 68,109,103, 77,
		24, 35, 55, 64, 81,104,113, 92,
		49, 64, 78, 87,103,121,120,101,
		72, 92, 95, 98,112,100,103, 99
	}
};
const QuantizationTable quant75={
	{
		16/2, 11/2, 10/2, 16/2, 24/2, 40/2, 51/2, 61/2,
		12/2, 12/2, 14/2, 19/2, 26/2, 58/2, 60/2, 55/2,
		14/2, 13/2, 16/2, 24/2, 40/2, 57/2, 69/2, 56/2,
		14/2, 17/2, 22/2, 29/2, 51/2, 87/2, 80/2, 62/2,
		18/2, 22/2, 37/2, 56/2, 68/2,109/2,103/2, 77/2,
		24/2, 35/2, 55/2, 64/2, 81/2,104/2,113/2, 92/2,
		49/2, 64/2, 78/2, 87/2,103/2,121/2,120/2,101/2,
		72/2, 92/2, 95/2, 98/2,112/2,100/2,103/2, 99/2
	}
};
const QuantizationTable quant100={
	{
		1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1
	}
};

//#define QTABLE quant100
//#define QTABLE quant75
#define QTABLE quant50

class BitReader {
	public:
		uint8_t nextByte=0;
		uint8_t nextBit=0;
		std::vector<uint8_t> m_data;
		int m_dataPos=0;
		int m_dataNumberBits=0;
		BitReader(const std::vector<uint8_t>& data,int numberBits) : m_data(data) {
			m_dataNumberBits=numberBits;
		}
		~BitReader() {}
		bool hasBits() {
			return BitPos()<m_dataNumberBits;
		}
		int BitPos()const{
			if(!m_dataPos)
				return 0;
			if(!nextBit)
				return m_dataPos<<3;
			return ((m_dataPos-1)<<3)+nextBit;
		}
		uint8_t readByte() {
			nextBit=0;
			return m_data[m_dataPos++];
		}
		uint32_t readWord() {
			nextBit=0;
			uint8_t byte0=m_data[m_dataPos++];
			uint8_t byte1=m_data[m_dataPos++];
			return (byte0<<8)|byte1;
		}
		uint32_t readBit() {
			if(nextBit==0) {
				if(!hasBits()) {
					return-1;
				}
				nextByte= m_data[m_dataPos++];
				while(nextByte==0xFF) {
					uint8_t marker= m_data[m_dataPos];
					// ignore multiple 0xFF's in a row
					while(marker==0xFF) {
						m_dataPos++;
						marker= m_data[m_dataPos];
					}
					// literal 0xFF's are encoded in the bitstream as 0xFF00
					if(marker==0x00) {
							m_dataPos++;
						break;
					}else{
						FATAL("Error-Invalid marker: 0x02x",(uint32_t)marker);
					}
				}
			}
			uint32_t bit=(nextByte>>(7-nextBit))&1;
			nextBit=(nextBit+1)&7;
			return bit;
		}
		// read a variable number of bits,first read bit is most significant bit return-1 if at any point all bits have already been read
		uint32_t readBits(const uint32_t length) {
			uint32_t bits=0;
			for(uint32_t i=0;i<length;++i) {
				uint32_t bit=readBit();
				if(bit==(uint32_t)-1) {
					bits=(uint32_t)-1;
					break;
				}
				bits=(bits<<1)|bit;
			}
			return bits;
		}
		// advance to the 0th bit of the next uint8_t
		void align() {
			nextBit=0;
		}
};

static std::string CodeToBinaryString(uint32_t code,int codeLength) {
	std::string str;
	for(int i=0;i!=codeLength;i++) {
		uint32_t mask=(1<<(codeLength-i-1));
		str+=code&mask ? "1":"0";
	}
	return str;
}

#define MAX_SYMBOL_LENGTH_BITS 10
#define MAX_SYMBOL_COUNT (1<<MAX_SYMBOL_LENGTH_BITS)

#define VALIDATE_SYMBOL(symbol) (!(symbol&(~(MAX_SYMBOL_COUNT-1))))

class Huffman {
	public:
		~Huffman();
		void RegisterSymbol(uint16_t symbol);
		void AllSymbolsRegistered();
		const HuffmanTable* GetTable()const{return &m_table;}
		bool CodeToSymbol(uint16_t* symbol,uint32_t code)const;
		bool SymbolToCode(uint32_t* code,uint32_t* codeLength,uint16_t symbol)const;
		void ReadTable(BitReader* bitReader);
		uint16_t GetNextCodeToSymbol(BitReader* bitReader)const;
		void Validate();
	protected:

		struct Node {
			uint16_t m_symbol;
			int m_freq;
			Node* m_left;
			Node* m_right;
		};
		struct HuffmanCode {
			uint32_t m_code;
			uint32_t m_length;
		};

		const Node* GetNextCodeToNodeRecursive(uint32_t codeCurrent,BitReader* bitReader,const Node* node)const;

		Node* CreateNode(uint16_t ch,int freq,Node* left,Node* right);
		void GenerateCodes(HuffmanTable* table)const;
		void DumpNode(int indent,const Huffman::Node* node,uint32_t code)const;
		void CreateTreeFromFrequencies();
		void GenerateTableCodes();
		void CreateTreeFromTable();
		bool CodeToSymbolRecursive(uint16_t* symbol,uint32_t codeFind,uint32_t codeCurrent,const Node* node)const;
#if MAX_SYMBOL_LENGTH_BITS<=12
		HuffmanCode m_symbolToCode[MAX_SYMBOL_COUNT]={{}};
		int m_freq[MAX_SYMBOL_COUNT]={};
#else
		std::unordered_map<uint16_t,HuffmanCode> m_symbolToCode;
		std::unordered_map<uint16_t,int> m_freq;
#endif
		uint32_t m_numberSymbols=0;
		HuffmanTable m_table;
		Node* m_root=0;
		Node* m_nodes=0;
		uint32_t m_nodeMaxCount=0;
		uint32_t m_nodeFreeIndex=0;
};
Huffman::~Huffman() {
	if(m_nodes)
		delete [] m_nodes;
}

// Function to allocate a new tree node
Huffman::Node* Huffman::CreateNode(uint16_t symbol,int freq,Huffman::Node* left,Huffman::Node* right) {
	if(m_nodeFreeIndex>=m_nodeMaxCount)
		FATAL("Huffman::CreateNode nodes buffer too small");
	Node* node=m_nodes+m_nodeFreeIndex++;
	node->m_symbol=symbol;
	node->m_freq=freq;
	node->m_left=left;
	node->m_right=right;
	return node;
}

void Huffman::GenerateCodes(HuffmanTable* table)const {
	uint32_t code=0;
	for(uint32_t i=0;i<countof(table->offsets)-1;++i) {
		for(uint32_t j=table->offsets[i];j<table->offsets[i+1];++j) {
			table->codes[j]=code;
			code++;
		}
		code<<=1;
	}
}

void Huffman::ReadTable(BitReader* bitReader) {
	m_table.offsets[0]=0;
	uint32_t allSymbols=0;
	for(uint32_t i=1;i<=countof(m_table.offsets)-1;++i) {
		allSymbols+=bitReader->readByte();
		m_table.offsets[i]=allSymbols;
	}
	for(uint32_t i=0;i<allSymbols;++i) {
		m_table.symbols[i]=bitReader->readWord();
	}
	m_numberSymbols=m_table.offsets[countof(m_table.offsets)-1];
	GenerateTableCodes();
	CreateTreeFromTable();
}

void Huffman::RegisterSymbol(uint16_t symbol) {
#if MAX_SYMBOL_LENGTH_BITS<=12
	if(!VALIDATE_SYMBOL(symbol))
		FATAL("Huffman::RegisterSymbol symbol 0x%04x out of bit range",symbol);
	if(!m_freq[symbol])
		m_numberSymbols++;
#endif
	m_freq[symbol]++;
}

void Huffman::GenerateTableCodes() {
	uint32_t code=0;
	for(uint32_t i=0;i<countof(m_table.offsets)-1;++i) {
		for(uint32_t j=m_table.offsets[i];j<m_table.offsets[i+1];++j) {
			m_table.codes[j]=code;
			code+=1;
		}
		code<<=1;
	}
}

void Huffman::CreateTreeFromTable() {
	if(m_nodeMaxCount<m_numberSymbols*2) {
		m_nodeMaxCount=m_numberSymbols*2;
		m_nodes=new Node[m_nodeMaxCount];
	}
	m_nodeFreeIndex=0;
	memset(m_nodes,0,sizeof(Node)*m_nodeMaxCount);

	uint32_t code=0;
	memset(m_nodes,0,sizeof(Node)*m_nodeMaxCount);
	std::unordered_map<uint32_t,Node*> codeToNode;
	for(uint32_t i=0;i<countof(m_table.offsets)-1;++i) {
		for(uint32_t j=m_table.offsets[i];j<m_table.offsets[i+1];++j) {
			//uprintf("insert symbol %d code %s codeLength %d\n",m_table.symbols[j],CodeToBinaryString(code,i+1).c_str(),i+1);
			if(!VALIDATE_SYMBOL(m_table.symbols[j]))
				FATAL("Symbol 0x%04x use out of bit range, adjust define MAX_SYMBOL_LENGTH_BITS",m_table.symbols[j]);
			m_symbolToCode[m_table.symbols[j]].m_code=code;
			m_symbolToCode[m_table.symbols[j]].m_length=i+1;
			Node* node=CreateNode(0,0,0,0);
			node->m_symbol=m_table.symbols[j];
			uint32_t codeParent=code;
			for(int k=i;k>=0;k--) {
				uint32_t key=(k<<24)|(codeParent>>1);
			//	uprintf("code %s\n",CodeToBinaryString(codeParent,k).c_str());
			//	uprintf("key 0x%08x\n",key);
			//	uprintf("mask 0x%x\n",1<<k);
				auto it=codeToNode.find(key);
				if(it==codeToNode.end()) {
					Node* nodeParent=CreateNode(0,0,0,0);
					if(codeParent&1) {
						nodeParent->m_right=node;
					}else{
						nodeParent->m_left=node;
					}
					codeToNode.insert(std::make_pair(key,nodeParent));
					node=nodeParent;
				}else{
					Node* nodeParent=&*it->second;
					if(codeParent&1) {
						if(nodeParent->m_right)
							FATAL("Existing right node");
						nodeParent->m_right=node;
					}else{
						if(nodeParent->m_left)
							FATAL("Existing left node");
						nodeParent->m_left=node;
					}
					break;
				}
				codeParent>>=1;
			}
			code+=1;
		}
		code<<=1;
	}
	auto it=codeToNode.find(0);
	if(it==codeToNode.end()) {
		for(auto [key,value]:codeToNode) {
			delete value;
		}
		return;
	}
	m_root=&*it->second;
}

void Huffman::CreateTreeFromFrequencies() {

#if MAX_SYMBOL_LENGTH_BITS>12
	m_numberSymbols=(int)m_freq.size();
#endif
	m_nodeMaxCount=m_numberSymbols*2;
	m_nodes=new Node[m_nodeMaxCount];
	m_nodeFreeIndex=0;
	memset(m_nodes,0,sizeof(Node)*m_nodeMaxCount);

	struct comp {
		bool operator()(Node* l,Node* r) {
			// highest priority item has lowest frequency
			return l->m_freq>r->m_freq;
		}
	};
	std::priority_queue<Node*,std::vector<Node*>,comp> tree;
#if MAX_SYMBOL_LENGTH_BITS<=12
	for(int i=0;i!=(1<<MAX_SYMBOL_LENGTH_BITS);i++) {
		if(m_freq[i])
			tree.push(CreateNode(i,m_freq[i],0,0));
	}
#else
	for(auto pair:m_freq) {
		tree.push(CreateNode(pair.first,pair.second,0,0));
	}
#endif
	if(!tree.size())
		FATAL("Huffman::CreateTreeFromFrequencies no symbols registered");
	while(tree.size()!=1) {
		Node* left=tree.top();
		tree.pop();
		Node* right=tree.top();
		tree.pop();
		int sum=left->m_freq+right->m_freq;
		tree.push(CreateNode(0,sum,left,right));
	}
	m_table={{}};
	std::vector<Node*> nodes;
	nodes.push_back(tree.top());
	int current=0;
	int pos=0;
	int symPos=0;
	while(current!=(int)nodes.size()) {
		int currentEnd=(int)nodes.size();
		for(int i=current;i!=currentEnd;i++) {
			Node* node=nodes[i];
			if(!node->m_left && !node->m_right) {
				m_table.symbols[symPos++]=node->m_symbol;
				//uprintf("node %d length %d\n",node->m_symbol,m_huffmanCode[node->m_symbol].m_length);
			}else{
				if(node->m_left)
					nodes.push_back(node->m_left);
				if(node->m_right)
					nodes.push_back(node->m_right);
			}
			current++;
		}
		m_table.offsets[pos++]=symPos;
	}
	while(pos<countof(m_table.offsets))
		m_table.offsets[pos++]=symPos;
	GenerateTableCodes();
	CreateTreeFromTable();
}

void Huffman::Validate() {
	DumpNode(0,m_root,0);
	for(uint32_t i=0;i<countof(m_table.offsets)-1;++i) {
		for(uint32_t j=m_table.offsets[i];j<m_table.offsets[i+1];++j) {
			uint32_t code=0;
			uint32_t codeLength=0;
			if(!SymbolToCode(&code,&codeLength,m_table.symbols[j]))
				FATAL("TF");
			if(code!=m_table.codes[j])
				FATAL("Huffman code validation failed %d!=%d",code!=m_table.codes[j]);
			uint16_t symbol=0;
			if(!CodeToSymbol(&symbol,code))
				FATAL("Huffman unable to find code %d",code);
			if(symbol!=m_table.symbols[j])
				FATAL("Huffman symbol validation failed %d!=%d",symbol,m_table.symbols[j]);
			uprintf("Validated symbol %d code 0x%04x %s length %d\n",m_table.symbols[j],code,CodeToBinaryString(code,codeLength).c_str(),codeLength);

		}
	}
}

void Huffman::AllSymbolsRegistered() {
	CreateTreeFromFrequencies();
}

void Huffman::DumpNode(int indent,const Huffman::Node* node,uint32_t code)const {
	if(node->m_left || node->m_right) {
		if(node->m_left)
			DumpNode(indent+1,node->m_left,code<<1);
		if(node->m_right)
			DumpNode(indent+1,node->m_right,(code<<1)|1);
		//uprintf("%*s left %s,right %s\n",indent," ",node->m_left?"true":"false",node->m_right?"true":"false");
	}else{
		uprintf("%*s node symbol %d freq %d code 0x%04x %s codeLength %d\n",indent," ",node->m_symbol,node->m_freq,code,CodeToBinaryString(code,indent).c_str(),indent);
	}
}

const Huffman::Node* Huffman::GetNextCodeToNodeRecursive(uint32_t codeCurrent,BitReader* bitReader,const Node* node)const {
	if(!node->m_left && !node->m_right) {
		return node;
	}
	bool bit=bitReader->readBit();
	if(!bit) {
		if(node->m_left) {
			node=GetNextCodeToNodeRecursive(codeCurrent<<1,bitReader,node->m_left);
			if(node)
				return node;
		}
	}else{
		if(node->m_right) {
			node=GetNextCodeToNodeRecursive((codeCurrent<<1)|1,bitReader,node->m_right);
			if(node)
				return node;
		}
	}
	return 0;
}
uint16_t Huffman::GetNextCodeToSymbol(BitReader* bitReader)const {
	if(m_numberSymbols>100) {		//Recursive or brute force?
		const Huffman::Node* node=GetNextCodeToNodeRecursive(0,bitReader,m_root);
		return node ? node->m_symbol:(uint16_t)-1;
	}
//	return bitReader->readBits(16);
	uint32_t currentCode=0;
	for(uint32_t i=0;i<countof(m_table.offsets)-1;++i) {
		int bit=bitReader->readBit();
		if(bit==-1) {
			return-1;
		}
		currentCode=(currentCode<<1)|bit;
		for(uint32_t j=m_table.offsets[i];j<m_table.offsets[i+1];++j) {
			if(currentCode==m_table.codes[j]) {
				return m_table.symbols[j];
			}
		}
	}
	return -1;
}

bool Huffman::CodeToSymbolRecursive(uint16_t* symbol,uint32_t codeFind,uint32_t codeCurrent,const Node* node)const {
	if(!node->m_left && !node->m_right) {
		if(codeCurrent==codeFind) {
			*symbol=node->m_symbol;
			return true;
		}
		return false;
	}
	if(node->m_left)
		if(CodeToSymbolRecursive(symbol,codeFind,codeCurrent<<1,node->m_left))
			return true;
	if(node->m_right)
		if(CodeToSymbolRecursive(symbol,codeFind,(codeCurrent<<1)|1,node->m_right))
			return true;
	return false;
}

bool Huffman::CodeToSymbol(uint16_t* symbol,uint32_t code)const {
	return CodeToSymbolRecursive(symbol,code,0,m_root);
}

bool Huffman::SymbolToCode(uint32_t* code,uint32_t* codeLength,uint16_t symbol)const {
#if MAX_SYMBOL_LENGTH_BITS<=12
	if(!m_symbolToCode[symbol].m_length)
		return false;
	*code=m_symbolToCode[symbol].m_code;
	*codeLength=m_symbolToCode[symbol].m_length;
	return true;

#else
	auto it=m_symbolToCode.find(symbol);
	if(it==m_symbolToCode.end())
		return false;
	*code=it->second.m_code;
	*codeLength=it->second.m_length;
	return true;
/*
	for(uint32_t i=0;i<countof(m_table.offsets)-1;++i) {		//Original slow search impl
		for(uint32_t j=m_table.offsets[i];j<m_table.offsets[i+1];++j) {
			if(symbol==m_table.symbols[j]) {
				*code=m_table.codes[j];
				*codeLength=i+1;
				return true;
			}
		}
	}
	return false;
*/
#endif
}

class BitWriter {
	public:
		uint8_t nextBit=0;
		std::vector<uint8_t> m_data;
		BitWriter(int reserveSize) {
			m_data.reserve(reserveSize);
		}
		void writeBit(uint32_t bit) {
			if(nextBit==0) {
				m_data.push_back(0);
			}
			m_data.back()|=(bit&1)<<(7-nextBit);
			nextBit=(nextBit+1)&7;
			if(nextBit==0 && m_data.back()==0xFF) {
				m_data.push_back(0);
			}
		}
		void writeBits(uint32_t bits,uint32_t length) {
			for(uint32_t i=1;i<=length;++i) {
				writeBit(bits>>(length-i));
			}
		}
};

// perform 1-D FDCT on all columns and rows of a block component, resulting in 2-D FDCT
void EncoderDCT16::ForwardDCTBlockComponent(int* component) {
	for(uint32_t i=0;i<8;++i) {
		const float a0=(float)component[0*8+i];
		const float a1=(float)component[1*8+i];
		const float a2=(float)component[2*8+i];
		const float a3=(float)component[3*8+i];
		const float a4=(float)component[4*8+i];
		const float a5=(float)component[5*8+i];
		const float a6=(float)component[6*8+i];
		const float a7=(float)component[7*8+i];

		const float b0=a0+a7;
		const float b1=a1+a6;
		const float b2=a2+a5;
		const float b3=a3+a4;
		const float b4=a3-a4;
		const float b5=a2-a5;
		const float b6=a1-a6;
		const float b7=a0-a7;

		const float c0=b0+b3;
		const float c1=b1+b2;
		const float c2=b1-b2;
		const float c3=b0-b3;
		const float c4=b4;
		const float c5=b5-b4;
		const float c6=b6-c5;
		const float c7=b7-b6;

		const float d0=c0+c1;
		const float d1=c0-c1;
		const float d2=c2;
		const float d3=c3-c2;
		const float d4=c4;
		const float d5=c5;
		const float d6=c6;
		const float d7=c5+c7;
		const float d8=c4-c6;

		const float e0=d0;
		const float e1=d1;
		const float e2=d2*m1;
		const float e3=d3;
		const float e4=d4*m2;
		const float e5=d5*m3;
		const float e6=d6*m4;
		const float e7=d7;
		const float e8=d8*m5;

		const float f0=e0;
		const float f1=e1;
		const float f2=e2+e3;
		const float f3=e3-e2;
		const float f4=e4+e8;
		const float f5=e5+e7;
		const float f6=e6+e8;
		const float f7=e7-e5;

		const float g0=f0;
		const float g1=f1;
		const float g2=f2;
		const float g3=f3;
		const float g4=f4+f7;
		const float g5=f5+f6;
		const float g6=f5-f6;
		const float g7=f7-f4;

		component[0*8+i]=(int)(g0*s0);
		component[4*8+i]=(int)(g1*s4);
		component[2*8+i]=(int)(g2*s2);
		component[6*8+i]=(int)(g3*s6);
		component[5*8+i]=(int)(g4*s5);
		component[1*8+i]=(int)(g5*s1);
		component[7*8+i]=(int)(g6*s7);
		component[3*8+i]=(int)(g7*s3);
	}
	for(uint32_t i=0;i<8;++i) {
		const float a0=(float)component[i*8+0];
		const float a1=(float)component[i*8+1];
		const float a2=(float)component[i*8+2];
		const float a3=(float)component[i*8+3];
		const float a4=(float)component[i*8+4];
		const float a5=(float)component[i*8+5];
		const float a6=(float)component[i*8+6];
		const float a7=(float)component[i*8+7];

		const float b0=a0+a7;
		const float b1=a1+a6;
		const float b2=a2+a5;
		const float b3=a3+a4;
		const float b4=a3-a4;
		const float b5=a2-a5;
		const float b6=a1-a6;
		const float b7=a0-a7;

		const float c0=b0+b3;
		const float c1=b1+b2;
		const float c2=b1-b2;
		const float c3=b0-b3;
		const float c4=b4;
		const float c5=b5-b4;
		const float c6=b6-c5;
		const float c7=b7-b6;

		const float d0=c0+c1;
		const float d1=c0-c1;
		const float d2=c2;
		const float d3=c3-c2;
		const float d4=c4;
		const float d5=c5;
		const float d6=c6;
		const float d7=c5+c7;
		const float d8=c4-c6;

		const float e0=d0;
		const float e1=d1;
		const float e2=d2*m1;
		const float e3=d3;
		const float e4=d4*m2;
		const float e5=d5*m3;
		const float e6=d6*m4;
		const float e7=d7;
		const float e8=d8*m5;

		const float f0=e0;
		const float f1=e1;
		const float f2=e2+e3;
		const float f3=e3-e2;
		const float f4=e4+e8;
		const float f5=e5+e7;
		const float f6=e6+e8;
		const float f7=e7-e5;

		const float g0=f0;
		const float g1=f1;
		const float g2=f2;
		const float g3=f3;
		const float g4=f4+f7;
		const float g5=f5+f6;
		const float g6=f5-f6;
		const float g7=f7-f4;

		component[i*8+0]=(int)(g0*s0);
		component[i*8+4]=(int)(g1*s4);
		component[i*8+2]=(int)(g2*s2);
		component[i*8+6]=(int)(g3*s6);
		component[i*8+5]=(int)(g4*s5);
		component[i*8+1]=(int)(g5*s1);
		component[i*8+7]=(int)(g6*s7);
		component[i*8+3]=(int)(g7*s3);
	}
}

static uint32_t BitLength(int v) {
	uint32_t length=0;
	while(v>0) {
		v>>=1;
		length++;
	}
	return length;
}
static void PushU16(std::vector<uint8_t>* vec,uint16_t val) {
	vec->push_back(val>>8);
	vec->push_back(val&0xff);
}

static uint16_t ReadU16(const std::vector<uint8_t>& vec,int pos) {
	return (vec[pos]<<8)|vec[pos+1];
}


void EncoderDCT16::RegisterBlockComponent(Huffman* huffmanDC,Huffman* huffmanAC,int* component,int* previousDC) {
	int coeff=component[0]-*previousDC;
	*previousDC=component[0];
	uint32_t coeffLength=BitLength(std::abs(coeff));
	if(coeffLength>MAXCOEFFLENGTH) {
		FATAL("RegisterBlockComponent Error-DC coefficient length %d",coeffLength);
	}
	if(coeff<0) {
		coeff+=(1<<coeffLength)-1;
	}
	huffmanDC->RegisterSymbol(coeffLength);
	for(uint32_t i=1;i<64;++i) {
		uint8_t numZeroes=0;
		while(i<64 && component[zigZagMap[i]]==0) {
			numZeroes+=1;
			i+=1;
		}
		if(i==64) {
			huffmanAC->RegisterSymbol(0x00);
			return;
		}
		while(numZeroes>=16) {
			huffmanAC->RegisterSymbol(0xf<<MAX_COEFF_LENGTH_BITS);
			numZeroes-=16;
		}
		coeff=component[zigZagMap[i]];
		coeffLength=BitLength(std::abs(coeff));
		if(coeffLength>MAXCOEFFLENGTH) {
			FATAL("RegisterBlockComponent Error-AC coefficient length  %d",coeffLength);
		}
		if(coeff<0) {
			coeff+=(1<<coeffLength)-1;
		}
		uint16_t symbol=(numZeroes<<MAX_COEFF_LENGTH_BITS)|coeffLength;
		huffmanAC->RegisterSymbol(symbol);
	}
}

void EncoderDCT16::EncodeBlockComponent(BitWriter* bitWriter,Huffman* huffmanDC,Huffman* huffmanAC,int* component,int* previousDC) {
	int coeff=component[0]-*previousDC;
	*previousDC=component[0];
	uint32_t coeffLength=BitLength(std::abs(coeff));
	if(coeffLength>MAXCOEFFLENGTH) {
		FATAL("Error-DC coefficient length  %d",coeffLength);
	}
	if(coeff<0) {
		coeff+=(1<<coeffLength)-1;
	}
	uint32_t code=0;
	uint32_t codeLength=0;
	if(!huffmanDC->SymbolToCode(&code,&codeLength,coeffLength)) {
		FATAL("Encode Invalid DC value");
	}
	bitWriter->writeBits(code,codeLength);
	bitWriter->writeBits(coeff,coeffLength);
	for(uint32_t i=1;i<64;++i) {
		uint8_t numZeroes=0;
		while(i<64 && component[zigZagMap[i]]==0) {
			numZeroes+=1;
			i+=1;
		}
		if(i==64) {
			if(!huffmanAC->SymbolToCode(&code,&codeLength,0x00)) {
				FATAL("Encode Invalid AC value");
			}
			bitWriter->writeBits(code,codeLength);
			return;
		}
		while(numZeroes>=16) {
			if(!huffmanAC->SymbolToCode(&code,&codeLength,0xf<<MAX_COEFF_LENGTH_BITS)) {
				FATAL("Encode Invalid AC value");
			}
			bitWriter->writeBits(code,codeLength);
			numZeroes-=16;
		}
		coeff=component[zigZagMap[i]];
		coeffLength=BitLength(std::abs(coeff));
		if(coeffLength>MAXCOEFFLENGTH) {
			FATAL("Error-AC coefficient length %d",coeffLength);
		}
		if(coeff<0) {
			coeff+=(1<<coeffLength)-1;
		}
		uint16_t symbol=(numZeroes<<MAX_COEFF_LENGTH_BITS)|coeffLength;
		if(!huffmanAC->SymbolToCode(&code,&codeLength,symbol)) {
			FATAL("Encode Invalid AC value");
		}
		bitWriter->writeBits(code,codeLength);
		bitWriter->writeBits(coeff,coeffLength);
	}
}

int EncoderDCT16::Encode(std::vector<uint8_t>* outStream,const std::vector<uint16_t>& data,int width,int height) {
//	uint64_t t0=GetTimeEpochMicroseconds();
	uint32_t blockHeight=(height+7)>>3;
	uint32_t blockWidth=(width+7)>>3;
	std::vector<Block> blocks(blockHeight*blockWidth);
	int cnt=0;
	for(int y=height-1;y>=0;--y) {
		const uint32_t blockRow=y>>3;
		const uint32_t pixelRow=y&7;
		for(int x=0;x<width;++x) {
			const uint32_t blockColumn=x>>3;
			const uint32_t pixelColumn=x&7;
			const uint32_t blockIndex=blockRow*blockWidth+blockColumn;
			const uint32_t pixelIndex=pixelRow*8+pixelColumn;
			blocks[blockIndex].y[pixelIndex]=data[cnt++];
		}
	}
	for(int i=0;i!=(int)blocks.size();i++) {
		ForwardDCTBlockComponent(blocks[i].y);
	}
	for(int i=0;i!=(int)blocks.size();i++) {
		for(uint32_t j=0;j<64;++j) {
			blocks[i].y[j]/=(int32_t)QTABLE.table[j];
		}
	}
//	uint64_t t1=GetTimeEpochMicroseconds();
	Huffman huffmanDC;
	Huffman huffmanAC;
	int previousDCs=0;
	for(int i=0;i!=(int)blocks.size();i++) {
		RegisterBlockComponent(&huffmanDC,&huffmanAC,blocks[i].y,&previousDCs);
	}
	previousDCs=0;
//	uint64_t t2=GetTimeEpochMicroseconds();
	huffmanDC.AllSymbolsRegistered();
	huffmanAC.AllSymbolsRegistered();

	//huffmanDC.Validate();			//Check all codes and symbols remap correctly
	//huffmanAC.Validate();

//	uint64_t t3=GetTimeEpochMicroseconds();
	BitWriter bitWriter((width*height)>>3);			//To avoid vector expansion
	for(int i=0;i!=(int)blocks.size();i++) {
		EncodeBlockComponent(&bitWriter,&huffmanDC,&huffmanAC,blocks[i].y,&previousDCs);
	}
//	uint64_t t4=GetTimeEpochMicroseconds();
	for(uint32_t i=0;i<64;++i) {
		outStream->push_back(QTABLE.table[zigZagMap[i]]);
	}
	PushU16(outStream,height);
	PushU16(outStream,width);
	const HuffmanTable* table=huffmanDC.GetTable();

//	int sz=(int)outStream->size();
	for(uint32_t i=0;i<countof(table->offsets)-1;++i) {
		outStream->push_back(table->offsets[i+1]-table->offsets[i]);
	}
	for(uint32_t i=0;i<countof(table->offsets)-1;++i) {
		for(uint32_t j=table->offsets[i];j<table->offsets[i+1];++j) {
			PushU16(outStream,table->symbols[j]);
		}
	}
	table=huffmanAC.GetTable();
	for(uint32_t i=0;i<countof(table->offsets)-1;++i) {
		outStream->push_back(table->offsets[i+1]-table->offsets[i]);
	}
	for(uint32_t i=0;i<countof(table->offsets)-1;++i) {
		for(uint32_t j=table->offsets[i];j<table->offsets[i+1];++j) {
			PushU16(outStream,table->symbols[j]);
		}
	}
//	int tablesSize=(int)outStream->size()-sz;
	outStream->insert(outStream->end(),bitWriter.m_data.begin(),bitWriter.m_data.end());
//	uint64_t t5=GetTimeEpochMicroseconds();

#if 0
	uprintf("time DCT & Quant %dus\n",(uint32_t)(t1-t0));
	uprintf("time Register all symbols %dus\n",(uint32_t)(t2-t1));
	uprintf("time Create tree and table %dus\n",(uint32_t)(t3-t2));
	uprintf("time Encode all symbols %dus\n",(uint32_t)(t4-t3));
	uprintf("time Write output %dus\n",(uint32_t)(t5-t4));
	uprintf("time total %dus\n",(uint32_t)(t5-t0));
#endif

//	int packedSize=(int)outStream->size();
//	float mbps=((float)packedSize*8.0f*30.0f)/1000000.0f;
//	uprintf("huffman tables size %d\n",tablesSize);
//	uprintf("raw %d packed %d ratio %d%% Mbps %.2f\n",width*height*sizeof(data[0]),outStream->size(),(outStream->size()*100)/(width*height*sizeof(data[0])),mbps);
	return 1;
}


void DecoderDCT16::DecodeBlockComponent(int* previousDC,BitReader* bitReader,int* component,Huffman* huffmanDC,Huffman* huffmanAC) {
	uint16_t length=huffmanDC->GetNextCodeToSymbol(bitReader);
	if(length==(uint8_t)-1) {
		FATAL("Decode Invalid DC value\n");
	}
	if(length>MAXCOEFFLENGTH) {
		FATAL("Decode DC coefficient length %d\n",length);
	}
	int coeff=bitReader->readBits(length);
	if(coeff==-1) {
		FATAL("Decode Invalid DC value\n");
	}
	if(length!=0 && coeff<(1<<(length-1))) {
		coeff-=(1<<length)-1;
	}
	component[0]=coeff+*previousDC;
	*previousDC=component[0];
	for(uint32_t i=1;i<64;++i) {
		uint16_t symbol=huffmanAC->GetNextCodeToSymbol(bitReader);
		if(symbol==(uint16_t)-1) {
			FATAL("Error-Invalid AC value\n");
		}
		if(symbol==0x00) {
			return;
		}
		if(symbol==0xf<<MAX_COEFF_LENGTH_BITS) {
			i+=15;
			component[zigZagMap[i]]=0;
			continue;
		}
		uint16_t numZeroes=(symbol>>MAX_COEFF_LENGTH_BITS)&0xf;
		uint16_t coeffLength=symbol&((1<<MAX_COEFF_LENGTH_BITS)-1);
		if(i+numZeroes>=64) {
			FATAL("Decode Zero run-length exceeded block component\n");
		}
		i+=numZeroes;
		if(coeffLength>MAXCOEFFLENGTH) {
			FATAL("Decode AC coefficient length %d\n",coeffLength);
		}
		coeff=bitReader->readBits(coeffLength);
		if(coeff==-1) {
			FATAL("Decode Invalid AC value\n");
		}
		if(coeff<(1<<(coeffLength-1))) {
			coeff-=(1<<coeffLength)-1;
		}
		component[zigZagMap[i]]=coeff;
	}
}

// perform 1-D IDCT on all columns and rows of a block component resulting in 2-D IDCT
void DecoderDCT16::InverseDCTBlockComponent(int* const component) {
	for(uint32_t i=0;i<8;++i) {
		const float g0=component[0*8+i]*s0;
		const float g1=component[4*8+i]*s4;
		const float g2=component[2*8+i]*s2;
		const float g3=component[6*8+i]*s6;
		const float g4=component[5*8+i]*s5;
		const float g5=component[1*8+i]*s1;
		const float g6=component[7*8+i]*s7;
		const float g7=component[3*8+i]*s3;

		const float f0=g0;
		const float f1=g1;
		const float f2=g2;
		const float f3=g3;
		const float f4=g4-g7;
		const float f5=g5+g6;
		const float f6=g5-g6;
		const float f7=g4+g7;

		const float e0=f0;
		const float e1=f1;
		const float e2=f2-f3;
		const float e3=f2+f3;
		const float e4=f4;
		const float e5=f5-f7;
		const float e6=f6;
		const float e7=f5+f7;
		const float e8=f4+f6;

		const float d0=e0;
		const float d1=e1;
		const float d2=e2*m1;
		const float d3=e3;
		const float d4=e4*m2;
		const float d5=e5*m3;
		const float d6=e6*m4;
		const float d7=e7;
		const float d8=e8*m5;

		const float c0=d0+d1;
		const float c1=d0-d1;
		const float c2=d2-d3;
		const float c3=d3;
		const float c4=d4+d8;
		const float c5=d5+d7;
		const float c6=d6-d8;
		const float c7=d7;
		const float c8=c5-c6;

		const float b0=c0+c3;
		const float b1=c1+c2;
		const float b2=c1-c2;
		const float b3=c0-c3;
		const float b4=c4-c8;
		const float b5=c8;
		const float b6=c6-c7;
		const float b7=c7;

		component[0*8+i]=(int)(b0+b7);
		component[1*8+i]=(int)(b1+b6);
		component[2*8+i]=(int)(b2+b5);
		component[3*8+i]=(int)(b3+b4);
		component[4*8+i]=(int)(b3-b4);
		component[5*8+i]=(int)(b2-b5);
		component[6*8+i]=(int)(b1-b6);
		component[7*8+i]=(int)(b0-b7);
	}
	for(uint32_t i=0;i<8;++i) {
		const float g0=component[i*8+0]*s0;
		const float g1=component[i*8+4]*s4;
		const float g2=component[i*8+2]*s2;
		const float g3=component[i*8+6]*s6;
		const float g4=component[i*8+5]*s5;
		const float g5=component[i*8+1]*s1;
		const float g6=component[i*8+7]*s7;
		const float g7=component[i*8+3]*s3;

		const float f0=g0;
		const float f1=g1;
		const float f2=g2;
		const float f3=g3;
		const float f4=g4-g7;
		const float f5=g5+g6;
		const float f6=g5-g6;
		const float f7=g4+g7;

		const float e0=f0;
		const float e1=f1;
		const float e2=f2-f3;
		const float e3=f2+f3;
		const float e4=f4;
		const float e5=f5-f7;
		const float e6=f6;
		const float e7=f5+f7;
		const float e8=f4+f6;

		const float d0=e0;
		const float d1=e1;
		const float d2=e2*m1;
		const float d3=e3;
		const float d4=e4*m2;
		const float d5=e5*m3;
		const float d6=e6*m4;
		const float d7=e7;
		const float d8=e8*m5;

		const float c0=d0+d1;
		const float c1=d0-d1;
		const float c2=d2-d3;
		const float c3=d3;
		const float c4=d4+d8;
		const float c5=d5+d7;
		const float c6=d6-d8;
		const float c7=d7;
		const float c8=c5-c6;

		const float b0=c0+c3;
		const float b1=c1+c2;
		const float b2=c1-c2;
		const float b3=c0-c3;
		const float b4=c4-c8;
		const float b5=c8;
		const float b6=c6-c7;
		const float b7=c7;

		component[i*8+0]=(int)(b0+b7);
		component[i*8+1]=(int)(b1+b6);
		component[i*8+2]=(int)(b2+b5);
		component[i*8+3]=(int)(b3+b4);
		component[i*8+4]=(int)(b3-b4);
		component[i*8+5]=(int)(b2-b5);
		component[i*8+6]=(int)(b1-b6);
		component[i*8+7]=(int)(b0-b7);
	}
}

bool DecoderDCT16::DecodeQuant16(std::vector<uint16_t>* data,int* widthOut,int* heightOut,const std::vector<uint8_t>& encoded) {
//	uint64_t t0=GetTimeEpochMicroseconds();
	BitReader bitReader(encoded,(int)encoded.size()*8);
	QuantizationTable quantizationTable;
	for(uint32_t i=0;i<64;++i) {
		quantizationTable.table[zigZagMap[i]]=bitReader.readByte();
	}
	uint32_t height=bitReader.readWord();
	uint32_t width=bitReader.readWord();
	if(!height || !width) {
		FATAL("Invalid dimensions\n");
	}
	Huffman huffmanDC;
	Huffman huffmanAC;
	huffmanDC.ReadTable(&bitReader);
	huffmanAC.ReadTable(&bitReader);
//	uint64_t t1=GetTimeEpochMicroseconds();
	uint32_t blockHeight=(height+7)>>3;
	uint32_t blockWidth=(width+7)>>3;
	std::vector<Block> blocks(blockHeight*blockWidth);
	int previousDCs=0;
	for(uint32_t i=0;i!=blocks.size();i++) {
		DecodeBlockComponent(&previousDCs,&bitReader,blocks[i].y,&huffmanDC,&huffmanAC);
	}
//	uint64_t t2=GetTimeEpochMicroseconds();
	for(uint32_t i=0;i!=blocks.size();i++) {
		for(uint32_t j=0;j<64;++j) {
			blocks[i].y[j]*=quantizationTable.table[j];
		}
	}
//	uint64_t t3=GetTimeEpochMicroseconds();
	for(uint32_t i=0;i!=blocks.size();i++) {
		InverseDCTBlockComponent(blocks[i].y);
	}
//	uint64_t t4=GetTimeEpochMicroseconds();
	*widthOut=width;
	*heightOut=height;
	data->resize(width*height);
	int pos=0;
	uint16_t* p=data->data();
	for(uint32_t y=0;y<height;y++) {
		uint32_t blockOffset=(y>>3)*blockWidth;
		uint32_t pixelRow=(y&7)<<3;
		for(uint32_t x=0;x<width;++x) {
			uint32_t blockIndex=blockOffset+(x>>3);
			uint32_t pixelIndex=pixelRow+(x&7);
			if(blocks[blockIndex].y[pixelIndex]>0) {
				p[pos++]=blocks[blockIndex].y[pixelIndex];
			}else{
				p[pos++]=0;
			}
		}
	}
//	uint64_t t5=GetTimeEpochMicroseconds();

#if 0
	uprintf("time Load tables %dus\n",(uint32_t)(t1-t0));
	uprintf("time Decode blocks %dus\n",(uint32_t)(t2-t1));
	uprintf("time Quant %dus\n",(uint32_t)(t3-t2));
	uprintf("time Inverse DCT %dus\n",(uint32_t)(t4-t3));
	uprintf("time Copy blocks to output %dus\n",(uint32_t)(t5-t4));
	uprintf("time total %dus\n",(uint32_t)(t5-t0));
#endif
	return true;
}
