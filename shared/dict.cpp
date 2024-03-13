
//#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string>
#include <vector>
#include "shared/misc.h"
#include "shared/dict.h"
#include "shared/std_ext.h"

#ifndef countof
#define countof(array) ((int)(sizeof(array)/sizeof(*array)))
#endif//countof

#ifndef ASSERT
#define ASSERT(x,...)do{if(!(x)){Fatal(__FILE__,__LINE__,__func__,__VA_ARGS__);}}while(0)
#endif

#ifndef LINKINSERTEND_A
#define LINKINSERTEND_A(node,first,last)	{ \
												(node)->m_prev=last; \
												(node)->m_next=NULL; \
												if(last) \
													last->m_next=node; \
												else \
													first=node; \
												last=node; \
											}

#define LINKREMOVE_A(node,first,last)			{ \
												if((node)->m_prev) \
													(node)->m_prev->m_next=(node)->m_next; \
												else \
													first=(node)->m_next; \
												if((node)->m_next) \
													(node)->m_next->m_prev=(node)->m_prev; \
												else \
													last=(node)->m_prev; \
												(node)->m_next=NULL; \
												(node)->m_prev=NULL; \
											}
#endif

#define NUMBER_TO_ASCII_MAX_LEN 64

inline int DoubleToAscii(char* buf,double value,const char* format=0) {
	if(!format)format="%.15f";
	int len=sprintf(buf,format,value);
	if(len) {
		int l=len-1;
		while(l && buf[l]=='0') {
			l--;
		}
		if(buf[l]!='.')
			l++;
		int l1=l;
		while(l1 && buf[l1]!='.') {
			l1--;
		}
		if(l1) {
			len=l;
			buf[l]=0;
		}
	}
	return len;
}
inline int IntToAscii(char* buf,int value) {
	return sprintf(buf,"%d",value);
}
std::string DoubleToString(double value) {
	char buf[NUMBER_TO_ASCII_MAX_LEN];
	DoubleToAscii(buf,value);
	return std::string(buf);
}

inline int FloatToAscii(char* buf,float value,const char* format=0) {
	if(!format)format="%.6f";
	int len=sprintf(buf,format,value);
	if(len) {
		int l=len-1;
		while(buf[l]=='0') {
			l--;
		}
		if(buf[l]=='.') {
			len=l;
			buf[l]=0;
		}
	}
	return len;
}
std::string FloatToString(float value) {
	char buf[NUMBER_TO_ASCII_MAX_LEN];
	FloatToAscii(buf,value);
	return std::string(buf);
}


uint32_t TypeBytesize(Dict::ValueType type) {
	switch (type) {
		case Dict::Byte:
			return sizeof(char);
		case Dict::Int:
			return sizeof(int);
		case Dict::Int64:
			return sizeof(int64_t);
		case Dict::Bool:
			return sizeof(bool);
		case Dict::Double:
			return sizeof(double);
		case Dict::Float:
			return sizeof(float);
		default:
			FATAL(" TypeBytesize failed for type %d",type);
			return 0;// Suppress compiler warning
	}
}
Dict::ValueType StringToType(const char* typeName) {
	if(!strcmp(typeName,"Byte"))return Dict::Byte;
	if(!strcmp(typeName,"Int"))return Dict::Int;
	if(!strcmp(typeName,"Int64"))return Dict::Int64;
	if(!strcmp(typeName,"Float"))return Dict::Float;
	if(!strcmp(typeName,"Double"))return Dict::Double;
	if(!strcmp(typeName,"Bool"))return Dict::Bool;
	if(!strcmp(typeName,"String"))return Dict::String;
	if(!strcmp(typeName,"Object"))return Dict::Object;
	if(!strcmp(typeName,"Array"))return Dict::Array;
	return Dict::NA;
}
const char* TypeToString(const Dict::ValueType& type){
	switch(type) {
		case Dict::Byte:	return "Byte";
		case Dict::Int:	return "Int";
		case Dict::Int64:	return "Int64";
		case Dict::Float:	return "Float";
		case Dict::Double:return "Double";
		case Dict::Bool:	return "Bool";
		case Dict::String:return "String";
		case Dict::Object:return "Object";
		case Dict::Array:	return "Array";
		default:
			break;
	}
	return "NA";
}
enum class Token {
	Unknown=0,
	Array=1,
	Object=2,
	ArrayEnd=3,
	ObjectEnd=4,
	String=5,
	Number=6,
	Bool=7,
	Null=8,
	Key=9,//has always other Type after it,which is not of Type Key
};

class DictJsonParser {
	public:
		DictJsonParser();
		void Parse(Dict* dict,const char* json,size_t size);
		static double AsciiToFloat(const char* x);
	private:
		size_t ParseChars(size_t pos,const char* chars);
		void ParseNumber(size_t pos);
		size_t ParseString(size_t pos,bool isTag=false);
		bool ParseValue(size_t pos);
		void ParseArray(size_t pos);
		void ParseObject(size_t pos);

		void HandleToken(Token token,const char* begin,const char* end);

		Dict* m_currentNode=0;
		std::string m_currentKey;
		std::vector<bool> m_currentBoolArray;
		std::vector<double> m_currentNumberArray;

		const char* m_json;
		size_t m_jsonSize;
		size_t m_jsonPos;
};

Dict::iterator Dict::m_end{0};
Dict::const_iterator Dict::m_cend{0};

Dict::Dict() {
	m_type=ValueType::Object;
	int nodeSize=sizeof(Dict);
	int expextedSize=sizeof(void*)+(sizeof(void*)*6)+(sizeof(uint16_t)*4)+sizeof(ValueUnion);

	expextedSize=(expextedSize+sizeof(double)-1)&-(int)sizeof(double);			//struct aligment to largest element

	if(nodeSize!=expextedSize)
		uprintf("Unexpected Dict size %d!=%d\n",nodeSize,expextedSize);
	memset(&m_value,0,sizeof(m_value));
}

Dict::Dict(const Dict& dict) {
	m_type=ValueType::Object;
	memset(&m_value,0,sizeof(m_value));
	Copy(dict);
}
Dict::Dict(const char* json) : Dict() {
	ReadFromJson(json);
}

void Dict::Clear() {
	if(m_name)
		free((void*)m_name);
	m_name=0;
	m_nameLength=0;
	ClearContent();
}

Dict::~Dict() {
	if(m_parent) {
		m_parent->UnlinkChild(this);
	}
	Clear();
//	ClearContent();
//	if(m_name) {
//		free((void*)m_name);
//		m_name=0;
//	}
}
int Dict::CalculateArrayIndex()const{
	if(!m_parent)
		return -1;
	Dict* node=m_parent->m_first;
	int count=0;
	while(node) {
		if(node==this)
			return count;
		count++;
		node=node->m_next;
	}
	return -1;
}
void Dict::UnlinkChild(Dict* child) {
	if(child->CalculateArrayIndex()==-1)
		FATAL("Tried to unlink nonexisting child");
	m_children--;
	LINKREMOVE_A(child,m_first,m_last);
}
void Dict::ClearValue() {
	if(HasAllocatedValue())
		free(m_value.m_arrayval);
	m_valueBytesize=0;
	m_type=ValueType::Object;
	memset(&m_value,0,sizeof(m_value));
	if(!m_first)
		m_children=0;
}

void Dict::ClearContent() {
	ClearValue();
	while(m_first) {
		Dict* expectedNext = m_first->m_next;
		delete m_first;
		ASSERT(m_first==expectedNext, "unlink problem");
	}
	ASSERT(m_children==0, "orphan dicts");
}

Dict* Dict::InsertCopy(const Dict& source) {
	Dict* node=NewChild();
	node->Copy(source);
	SetName(source.Name().c_str());
	return node;
}

void Dict::Copy(const Dict& source,bool copyName) {
	ClearContent();
	if(copyName) {
		if(m_name)
			free((void*)m_name);
		m_name=0;
		m_nameLength=source.m_nameLength;
		if(source.m_nameLength) {
			m_name=(char*)malloc(source.m_nameLength);
			memcpy((char*)m_name,source.m_name,m_nameLength);
		}
	}
	m_type=source.m_type;
	m_valueBytesize=source.m_valueBytesize;
	if(m_valueBytesize) {
		// First,assume primitive value
		char* dest=(char*)&m_value;
		const char* srcBuffer=(char*)&source.m_value;
		if((m_type==ValueType::String && m_valueBytesize > sizeof(ValueUnion)) || source.m_children) {
			// String or array
			dest=(char*)malloc(m_valueBytesize);
			srcBuffer=(char*)source.m_value.m_arrayval;
			m_value.m_arrayval=dest;
		}
		memcpy(dest,srcBuffer,m_valueBytesize);
		m_children=source.m_children;
	}else{
		m_value=source.m_value;
	}
	auto child=source.m_first;
	while(child) {
		Dict* node=NewChild();
		node->Copy(*child);
		child=child->m_next;
	}
}
const Dict* Dict::Find(const char* p,int len)const {
	Dict* node=m_first;
	while(node) {
		if(node->m_name && len==(int)node->m_nameLength && !memcmp(node->m_name,p,node->m_nameLength))
			return node;
		node=node->m_next;
	}
	return 0;
}

Dict::iterator Dict::begin() {
	if(!m_first)
		return m_end;
	return {m_first};
}

Dict::const_iterator Dict::begin()const {
	if(!m_first)
		return m_cend;
	return {m_first};
}



Dict& Dict::At(size_t index)const {
	Dict* node=m_first;
	while(node && index-- > 0)
		node=node->m_next;
	if(!node)
		FATAL("Dict::At index %d failed",index);
	return *node;
}

Dict* Dict::NewChild() {
	Dict* node=new Dict;
	LINKINSERTEND_A(node,m_first,m_last);
	node->m_parent=this;
	m_children++;
	return node;
}
Dict* Dict::FindOrAdd(const char* p,int len) {
	if(Dict* node=Find(p,len))
		return node;
	Dict* node=NewChild();
	node->SetName(p,len);
	return node;
}
Dict& Dict::Add(const char* name) {
	return *FindOrAdd(name);
}
Dict& Dict::Add(const std::string& name) {
	return *FindOrAdd(name.c_str(),(int)name.size());
}
Dict& Dict::Add(const char* p,int len) {
	return *FindOrAdd(p,len);
}
const void* Dict::GetTypedArrayData()const {
	if(!IsTypedArray())
		return 0;
	return m_value.m_arrayval;
}

void Dict::AllocatValueTypedArray(ValueType type,int n) {
	int valueBytesize=n*TypeBytesize(type);
	if(n && valueBytesize<=m_valueBytesize) {
		m_type=type;
		m_children=(uint16_t)n;
		return;
	}
	ClearContent();
	m_type=type;
	m_children=(uint16_t)n;
	m_valueBytesize=valueBytesize;
	if(!n) {
		m_value.m_arrayval=0;
		m_type=Array;
		return;
	}
	m_value.m_arrayval=n?malloc(m_valueBytesize):0;
}
void Dict::AllocateAndCopyTypedArray(ValueType type,const void* vals,int n) {
	AllocatValueTypedArray(type,n);
	if(n)
		memcpy(m_value.m_arrayval,vals,n*TypeBytesize(type));
}

bool Dict::HasAllocatedValue()const {
	if(IsTypedArray()) {
		return m_value.m_arrayval ? true:false;
	}
	if(m_type==ValueType::String && m_valueBytesize>sizeof(ValueUnion))
		return true;
	return false;
}
bool Dict::IsTypedArray()const {
	if(m_children>0 && !m_first)
		return true;
	return false;
}
Dict& Dict::operator=(const Dict& value) {
	ClearContent();
	Copy(value);
	return *this;
}

bool Dict::GetDouble(double* v)const {
	if(m_type!=Double)
		return false;
	*v=m_value.m_doubleval;
	return true;
}
bool Dict::GetFloat(float* v)const {
	if(m_type!=Float) {
		if(m_type==Double) {
			*v=(float)m_value.m_doubleval;
			return true;
		}
		return false;
	}
	*v=m_value.m_floatval;
	return true;
}
bool Dict::GetBool(bool* v)const {
	if(m_type!=Bool)
		return false;
	*v=m_value.m_boolval;
	return true;
}
bool Dict::GetInt(int* v)const {
	if(m_type!=Int) {
		if(m_type==Double) {
			*v=(int)m_value.m_doubleval;
			return true;
		}
		return false;
	}
	*v=m_value.m_intval;
	return true;
}
bool Dict::GetInt64(int64_t* v)const {
	if(m_type!=Int64)
		return false;
	*v=m_value.m_int64val;
	return true;
}
bool Dict::GetByte(char* v)const {
	if(m_type!=Byte) {
		if(m_type==Double) {
			*v=(char)m_value.m_doubleval;
			return true;
		}
		return false;
	}
	*v=m_value.m_byteval;
	return true;
}
bool Dict::GetString(std::string* v)const {
	if(m_type!=String)
		return false;
	if(m_valueBytesize>sizeof(ValueUnion)) {
		v->assign((char*)m_value.m_arrayval,m_valueBytesize);
	}else{
		v->assign(m_value.m_stringval,m_valueBytesize);
	}
	return true;
}

bool Dict::GetString(const char** v,int* len)const {
	if(m_type!=String)
		return false;
	if(m_valueBytesize>sizeof(ValueUnion)) {
		*v=(char*)m_value.m_arrayval;
	}else{
		*v=(char*)m_value.m_stringval;
	}
	*len=m_valueBytesize;
	return true;

}
bool Dict::Get(const char* name,bool* value,bool default_value)const {
	if(const Dict* node=Find(name))
		if(node->GetBool(value))
			return true;
	*value=default_value;
	return false;
}
bool Dict::Get(const char* name,char* value,char default_value)const {
	if(const Dict* node=Find(name))
		if(node->GetByte(value))
			return true;
	*value=default_value;
	return false;
}
bool Dict::Get(const char* name,int* value,int default_value)const {
	if(const Dict* node=Find(name))

		if(node->GetInt(value))
			return true;
	*value=default_value;
	return false;
}
bool Dict::Get(const char* name,int64_t* value,int64_t default_value)const {
	if(const Dict* node=Find(name))
		if(node->GetInt64(value))
			return true;
	*value=default_value;
	return false;
}
bool Dict::Get(const char* name,float* value,float default_value)const {
	if(const Dict* node=Find(name))
		if(node->GetFloat(value))
			return true;
	*value=default_value;
	return false;
}
bool Dict::Get(const char* name,double* value,double default_value)const {
	if(const Dict* node=Find(name))
		if(node->GetDouble(value))
			return true;
	*value=default_value;
	return false;
}
bool Dict::Get(const char* name,std::string* value,std::string default_value)const {
	if(const Dict* node=Find(name))
		if(node->GetString(value))
			return true;
	*value=default_value;
	return false;
}
bool Dict::GetTypedArray(ValueType type,void* vals,int n)const {
	if(IsArray() && !m_children)
		return true;
	if(!IsTypedArray() || m_children<n)
		return false;
	memcpy(vals,m_value.m_arrayval,TypeBytesize(type)*n);
	return true;
}

bool Dict::GetNumber(double* v)const {
	switch(m_type) {
		case Double:
			*v=m_value.m_doubleval;
			return true;
		case Float:
			*v=m_value.m_floatval;
			return true;
		case Int:
			*v=m_value.m_intval;
			return true;
		case Int64:
			*v=(double)m_value.m_int64val;
			return true;
		case Byte:
			*v=m_value.m_byteval;
			return true;
		case Bool:
			*v=m_value.m_boolval ? 1:0;
			return true;
		default:
			break;
	}
	return false;
}
bool Dict::GetNumberArray(const char* name,std::vector<double>* vec)const {
	const Dict* node=Find(name);
	return node ? node->GetNumberArray(vec):false;
}
bool Dict::GetNumberArray(std::vector<double>* vec)const {
	if(IsArray()) {
		Dict* d=m_first;
		vec->clear();
		vec->reserve(m_children);
		while(d) {
			double v;
			if(!d->GetNumber(&v)) {
				vec->clear();
				return false;
			}
			vec->push_back(v);
			d=d->m_next;
		}
		return true;
	}
	return GetTypedArray(vec);
}
bool Dict::GetTypedArray(std::vector<double>* vec)const {
	vec->clear();
	if(!IsArray() && !m_children)
		return true;
	if(!IsTypedArray())
		return false;
	const char* p=(const char*)m_value.m_arrayval;
	if(m_type==Double) {
		vec->resize(m_children);
		memcpy(&*vec->data(),p,sizeof(double)*m_children);
		return true;
	}
	vec->reserve(m_children);
	for(uint32_t i=0;i<m_children;i++,p+=TypeBytesize(m_type)) {
		switch(m_type) {
			case ValueType::Bool:
				vec->push_back(*p?1.0:0.0);
				break;
			case ValueType::Double:
				vec->push_back(*(const double*)p);
				break;
			case ValueType::Float:
				vec->push_back(*(const float*)p);
				break;
			case ValueType::Byte:
				vec->push_back(*p);
				break;
			case ValueType::Int:
				vec->push_back(*(const int*)p);
				break;
			case ValueType::Int64:
				vec->push_back((double)*(const int64_t*)p);
				break;
			default:
				FATAL("Dict::GetTypedArray Unsupported type %d",m_type);
				break;
		}
	}
	return true;
}
bool Dict::GetTypedArray(const char* name,ValueType type,void* vals,int n)const {
	const Dict* node=Find(name);
	return node ? node->GetTypedArray(type,vals,n):false;
}

bool Dict::GetTypedArray(const char* name,std::vector<float>* p)const {
	const Dict* node=Find(name);
	if(!node || (!node->IsArray() && !node->IsTypedArray()))
		return false;
	p->resize(node->m_children);
	return node->GetTypedArray(Float,(float*)p->data(),(int)p->size());
}
bool Dict::GetTypedArray(const char* name,std::vector<double>* p)const {
	const Dict* node=Find(name);
	if(!node || (!node->IsArray() && !node->IsTypedArray()))
		return false;
	p->resize(node->m_children);
	return node->GetTypedArray(Double,(float*)p->data(),(int)p->size());
}
bool Dict::GetTypedArray(const char* name,bool* value,size_t items)const {
	return GetTypedArray(name,ValueType::Bool,value,(int)items);
}
bool Dict::GetTypedArray(const char* name,char* value,size_t items)const {
	return GetTypedArray(name,ValueType::Byte,value,(int)items);
}
bool Dict::GetTypedArray(const char* name,int* value,size_t items)const {
	return GetTypedArray(name,ValueType::Int,value,(int)items);
}
bool Dict::GetTypedArray(const char* name,int64_t* value,size_t items)const {
	return GetTypedArray(name,ValueType::Int64,value,(int)items);
}
bool Dict::GetTypedArray(const char* name,double* value,size_t items)const {
	return GetTypedArray(name,ValueType::Double,value,(int)items);
}
bool Dict::GetTypedArray(const char* name,float* value,size_t items)const {
	return GetTypedArray(name,ValueType::Float,value,(int)items);
}

Dict* Dict::Set(const char* name,bool value) {
	Dict* node=FindOrAdd(name);
	node->SetBool(value);
	return node;
}
Dict* Dict::Set(const char* name,char value) {
	Dict* node=FindOrAdd(name);
	node->SetByte(value);
	return node;
}
Dict* Dict::Set(const char* name,int value) {
	Dict* node=FindOrAdd(name);
	node->SetInt(value);
	return node;
}
Dict* Dict::Set(const char* name,int64_t value) {
	Dict* node=FindOrAdd(name);
	node->SetInt64(value);
	return node;
}
Dict* Dict::Set(const char* name,float value) {
	Dict* node=FindOrAdd(name);
	node->SetFloat(value);
	return node;
}
Dict* Dict::Set(const char* name,double value) {
	Dict* node=FindOrAdd(name);
	node->SetDouble(value);
	return node;
}

Dict* Dict::Set(const char* name,const char* value,int len) {
	Dict* node=FindOrAdd(name);
	node->SetString(value,len);
	return node;
}
Dict* Dict::SetTypedArray(const char* name,ValueType type,const void* vals,int n) {
//	if(n<1)
//		FATAL("Dict::SetTypedArray with %d elements, must have at least 1 element.",n);
	Dict* node=FindOrAdd(name);
	node->AllocateAndCopyTypedArray(type,vals,n);
	return node;
}

Dict* Dict::SetTypedArray(const char* name,const bool* value,size_t items) {
	return SetTypedArray(name,ValueType::Bool,value,(int)items);
}
Dict* Dict::SetTypedArray(const char* name,const char* value,size_t items) {
	return SetTypedArray(name,ValueType::Byte,value,(int)items);
}
Dict* Dict::SetTypedArray(const char* name,const int* value,size_t items) {
	return SetTypedArray(name,ValueType::Int,value,(int)items);
}
Dict* Dict::SetTypedArray(const char* name,const int64_t* value,size_t items) {
	return SetTypedArray(name,ValueType::Int64,value,(int)items);
}
Dict* Dict::SetTypedArray(const char* name,const float* value,size_t items) {
	return SetTypedArray(name,ValueType::Float,value,(int)items);
}
Dict* Dict::SetTypedArray(const char* name,const double* value,size_t items) {
	return SetTypedArray(name,ValueType::Double,value,(int)items);
}

//Set values on this node
bool Dict::SetArray(float* value,size_t items) {
	AllocateAndCopyTypedArray(ValueType::Float,value,(int)items);
	return true;
}
bool Dict::SetArray(double* value,size_t items) {
	AllocateAndCopyTypedArray(ValueType::Double,value,(int)items);
	return true;
}
Dict* Dict::PushBack() {
	if(m_type==ValueType::Object && m_children==0)
		m_type=ValueType::Array;
	if(m_type!=ValueType::Array)
		FATAL("Dict::PushBack failed type %d",m_type);
	return NewChild();
}


bool Dict::PushBack(ValueType type,const void* value) {
	//If node is normal array insert child with new value
	if(m_type==ValueType::Array) {
		switch(type) {
			case Double: {
				NewChild()->SetDouble(*(double*)value);
				break;
			}
			default:
				FATAL("Dict::PushBack unsupported type %d",m_type);
		}
		return true;
	}
	if(!IsTypedArray() || m_type!=type)
		return false;
	void* data=m_value.m_arrayval;
	int length=m_valueBytesize;
	m_valueBytesize=m_valueBytesize+TypeBytesize(type);
	m_value.m_arrayval=(char*)malloc(m_valueBytesize);
	m_children++;
	memcpy(m_value.m_arrayval,data,length);
	memcpy((char*)m_value.m_arrayval+length,value,TypeBytesize(type));
	free(data);
	return true;
}
bool Dict::PushBack(double value) {
	return PushBack(ValueType::Double,&value);
}
bool Dict::PushBack(float value) {
	return PushBack(ValueType::Float,&value);
}
bool Dict::PushBack(int value) {
	return PushBack(ValueType::Int,&value);
}
bool Dict::PushBack(int64_t value) {
	return PushBack(ValueType::Int64,&value);
}
bool Dict::PushBack(bool value) {
	return PushBack(ValueType::Bool,&value);
}
bool Dict::PushBack(char value) {
	return PushBack(ValueType::Byte,&value);
}
bool Dict::PushBack(const Dict& value) {
	if(m_type!=ValueType::Array)
		return false;
	Dict* object=NewChild();
	object->Copy(value);
	return true;
}

size_t RawToPrimArrayDouble(double* buffer,size_t items,const char* data) {
	size_t retval=std::min(items,(size_t)items);
	for(size_t i=0;i<retval;i++) {
		*buffer++=DictJsonParser::AsciiToFloat(data);
		if(i==retval-1)
			break;
		while(*data !=',')
			data+=1;
		data+=1;
		while(*data==' ' || *data=='\n' || *data=='\t' || *data=='\r')
			data+=1;
	}
	return retval;
}
size_t RawToPrimArrayBool(bool* buffer,size_t items,const char* data) {
	size_t retval=std::min(items,(size_t)items);
	for(size_t i=0;i<retval;i++) {
		*buffer++=!strncmp(data,"true",4);
		if(i==retval-1)
			break;
		while(*data !=',')
			data+=1;
		data+=1;
		while(*data==' ' || *data=='\n' || *data=='\t' || *data=='\r')
			data+=1;
	}
	return retval;
}

Dict* Dict::AddObjectNode(const char* name,int len) {
	Dict* node=NewChild();
	node->SetName(name,len);
	node->m_type=ValueType::Object;
	return node;
}
Dict* Dict::AddArrayNode(const char* name,int len) {
	Dict* node=NewChild();
	node->SetName(name,len);
	node->m_type=ValueType::Array;
	return node;
}
void Dict::Set(ValueType type,const void* value) {
	ClearContent();
	memcpy(&m_value,value,TypeBytesize(type));
	m_type=type;
	m_valueBytesize=TypeBytesize(type);
}
void Dict::Set(const Dict& source) {
	ClearContent();
	Copy(source,false);
}
void Dict::SetDouble(double value) {
	Set(ValueType::Double,&value);
}
void Dict::SetFloat(float value) {
	Set(ValueType::Float,&value);
}
void Dict::SetInt(int value) {
	Set(ValueType::Int,&value);
}
void Dict::SetInt64(int64_t value) {
	Set(ValueType::Int64,&value);
}
void Dict::SetByte(char value) {
	Set(ValueType::Byte,&value);
}
void Dict::SetBool(bool value) {
	Set(ValueType::Bool,&value);
}
void Dict::SetString(const char* string,size_t length) {
	ClearContent();
	m_valueBytesize=(uint32_t)length;
	m_type=String;
	if(m_valueBytesize<=sizeof(ValueUnion)) {
#ifdef _WIN32
		strncpy(m_value.m_stringval,string,sizeof(ValueUnion));
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
		strncpy(m_value.m_stringval,string,sizeof(ValueUnion));
#pragma GCC diagnostic pop
#endif
	}else{
		m_value.m_arrayval=(char*)malloc(m_valueBytesize);
		memcpy(m_value.m_arrayval,string,m_valueBytesize);
	}
}
void Dict::SetString(const char* value) {
	SetString(value,strlen(value));
}
void Dict::SetString(const std::string& value) {
	SetString(value.data(),value.size());
}

void Dict::SetType(const ValueType& type) {
	if(type!=Object) {
		ClearContent();
	}
	if(HasAllocatedValue())
		free(m_value.m_arrayval);
	m_valueBytesize=0;
	memset(&m_value,0,sizeof(m_value));
	if(!m_first)
		m_children=0;
	m_type=type;
}

void Dict::SetName(const char* name,int len) {
	if(m_name)
		free((void*)m_name);
	m_nameLength=(uint16_t)len;
	if(!len) {
		m_name=0;
		return;
	}
	if(!name[len])			//allocate and copy trailing zero for debug purporses
		len++;
	m_name=(char*)malloc(len);
	memcpy((char*)m_name,name,len);
}

const char* Dict::TypeName()const {
	switch(m_type) {
		case Byte: return "byte";
		case Int: return "int";
		case Int64: return "int64";
		case Float: return "float";
		case Double: return "double";
		case Bool: return "bool";
		case String: return "string";
		case Object: return "object";
		case Array: return "array";
		default:
			break;
	}
	return "NA";
}

std::string Dict::FullName()const {
	if(!m_parent)
		return "root";
	if(!m_nameLength) {
		int ix=CalculateArrayIndex();
		return m_parent->FullName()+"["+std::to_string(ix)+"]";
	}
	return m_parent->FullName()+"."+Name();
}

std::string Dict::Name()const {
	return std::string(m_name,m_nameLength);
}
/*
std::string Dict::TypedArrayValue()const {
	if(m_first)
		FATAL("Dict::TypedArrayValue node is array");
	std::string value;
	const char* prim=(const char*)m_value.m_arrayval;
	for(uint32_t i=0;i<m_children;i++,prim+=TypeBytesize(m_type)) {
		switch(m_type) {
			case ValueType::Bool:
				value.append(*((const bool*)prim) ? "true" : "false");
				break;
			case ValueType::Double:
				value.append(DoubleToString(*(const double*)prim));
				//char buf[64];
				//sprintf(buf,"%f",*(const double*)prim);
				break;
			case ValueType::Float:
				//value.append(std::to_string(*(const float*)prim));
				value.append(FloatToString(*(const float*)prim));
				break;
			case ValueType::Byte:
				value.append(std::to_string(*prim));
				break;
			case ValueType::Int:
				value.append(std::to_string(*(const int*)prim));
				break;
			case ValueType::Int64:
				value.append(std::to_string(*(const int64_t*)prim));
				break;
			default:
				FATAL("Dict::TypedArrayValue Unsupported type %d",m_type);
				break;
		}
		if(i<(uint32_t)m_children-1)
			value.append(",");
	}
	return value;
}
*/
std::string Dict::Value()const {
	switch(m_type) {
		case ValueType::String: {
			std::string str;
			GetString(&str);
			return str;
		}
		case ValueType::Bool:
			return m_value.m_boolval ? "true" : "false";
		case ValueType::Double: {
			char buf[NUMBER_TO_ASCII_MAX_LEN];
			int len=DoubleToAscii(buf,m_value.m_doubleval);
			return std::string(buf,buf+len);
		}
		case ValueType::Float: {
			char buf[NUMBER_TO_ASCII_MAX_LEN];
			int len=FloatToAscii(buf,m_value.m_floatval);
			return std::string(buf,buf+len);
		}
		case ValueType::Byte:
			return std::to_string(m_value.m_byteval);
		case ValueType::Int:
			return std::to_string(m_value.m_intval);
		case ValueType::Int64:
			return std::to_string(m_value.m_int64val);
		default:
			break;
	}
	return "";
}

int Dict::BinarySize(const Dict* node)const {
	int size=0;
	if(node->m_type<16 && node->m_children<8 && node->m_nameLength<16 && node->m_valueBytesize<16) {
		size=2+node->m_nameLength+node->m_valueBytesize;
	}else{
		size=sizeof(ValueType)+3*sizeof(uint16_t)+node->m_nameLength+node->m_valueBytesize;
	}
	Dict* child=node->m_first;
	while(child) {
		size+=BinarySize(child);
		child=child->m_next;
	}
	return size;
}

void Dict::ReadFromBinary(const uint8_t** data,Dict* node) {
	uint16_t children;
	memcpy(&node->m_type,*data,sizeof(ValueType));
	*data+=sizeof(ValueType);
	if(node->m_type&(1<<15)) {		//Is packed
		uint16_t pack=node->m_type;
		node->m_type=(ValueType)((pack>>11)&0xf);
		children=(pack>>8)&0x7;
		node->m_nameLength=(pack>>4)&0xf;
		node->m_valueBytesize=pack&0xf;
	}else{
		memcpy(&children,*data,sizeof(uint16_t));
		*data+=sizeof(uint16_t);
		memcpy(&node->m_nameLength,*data,sizeof(uint16_t));
		*data+=sizeof(uint16_t);
		memcpy(&node->m_valueBytesize,*data,sizeof(uint16_t));
		*data+=sizeof(uint16_t);
	}
	if(node->m_nameLength>0) {
		node->m_name=(char*)malloc(node->m_nameLength+1);
		memcpy((char*)node->m_name,*data,node->m_nameLength);
		((char*)node->m_name)[node->m_nameLength]=0;
		*data+=node->m_nameLength;
	}
	if(node->m_type==ValueType::Bool || node->m_type==ValueType::Byte || node->m_type==ValueType::Int || node->m_type==ValueType::Int64 || node->m_type==ValueType::Float || node->m_type==ValueType::Double || node->m_type==ValueType::String) {
		if(children==0 && node->m_valueBytesize<=sizeof(ValueUnion))
			memcpy(&node->m_value,*data,node->m_valueBytesize);
		else {
			node->m_value.m_arrayval=(char*)malloc(node->m_valueBytesize);
			memcpy(node->m_value.m_arrayval,*data,node->m_valueBytesize);
		}
		node->m_children=children;
		*data+=node->m_valueBytesize;
	} else {
		*data+=node->m_valueBytesize;
		for(uint32_t i=0;i<children;i++)
			ReadFromBinary(data,node->AddArrayNode());
	}
}
void Dict::ReadFromBinary(const std::vector<uint8_t>& data) {
	Dict::Clear();
	const uint8_t* p=data.data();
	int pos=0;
	ReadFromBinary(p,&pos);
}

void Dict::ReadFromBinary(const uint8_t* buffer,int* pos) {
	Dict::Clear();
	buffer+=*pos;
	const uint8_t* start=buffer;
	ReadFromBinary(&buffer,this);
	*pos+=(int)(buffer-start);
}

void Dict::WriteToBinary(void* context,void(*write)(void* context,const void* data,int size),const Dict* node)const {
	if(node->m_type<16 && node->m_children<8 && node->m_nameLength<16 && node->m_valueBytesize<16) {
		uint64_t pack=(1<<15)|(node->m_type<<11)|(node->m_children<<8)|(node->m_nameLength<<4)|node->m_valueBytesize;
		write(context,&pack,sizeof(uint16_t));
	}else{
		write(context,&node->m_type,sizeof(Dict::ValueType));
		write(context,&node->m_children,sizeof(uint16_t));
		write(context,&node->m_nameLength,sizeof(uint16_t));
		write(context,&node->m_valueBytesize,sizeof(uint16_t));
	}
	write(context,node->m_name,node->m_nameLength);
	if(const void* tarray=node->GetTypedArrayData()) {
		write(context,tarray,node->m_valueBytesize);
	}else
	if(node->IsString()) {
		const char* p;
		int len;
		node->GetString(&p,&len);
		write(context,p,len);
	}else{
		write(context,&node->m_value,node->m_valueBytesize);
	}
	Dict* child=node->m_first;
	while(child) {
		WriteToBinary(context,write,child);
		child=child->m_next;
	}
}
void Dict::WriteToBinary(std::vector<uint8_t>* mb)const {
	mb->reserve(BinarySize());
	auto write=[](void* user_data,const void* data,int size) {
		std::vector<char>* mb=((std::vector<char>*)user_data);
		mb->insert(mb->end(),(char*)data,(char*)data+size);
	};
	WriteToBinary(mb,write,this);
}

void Dict::WriteToBinary(uint8_t* buffer,int* pos)const {
	buffer+=*pos;
	uint8_t* start=buffer;
	auto write=[](void* user_data,const void* data,int size) {memcpy (*(uint8_t**)user_data,data,size);*(uint8_t**)user_data+=size;};
	WriteToBinary(&buffer,write,this);
	*pos+=(int)(buffer-start);
}

std::string Dict::WriteToJson(bool appendZero,int indent,int spacingLevel,bool echo,bool compact,const char* floatFormat,const char* doubleFormat)const {
	std::vector<char> vec;
	vec.reserve(10000);									//avoid small resizes
	auto appendstring=[&](int indent,const char* s){
		int len=(int)strlen(s);
		size_t pos=vec.size();
		vec.resize(vec.size()+indent+len);
		memset(vec.data()+pos,' ',indent);
		memcpy(vec.data()+pos+indent,s,len);
		if(echo)
			uprintf("%.*s",indent+len,vec.data()+pos);		//use print with length since vec has no trailing zero
	};
	auto appendDouble=[&](double v){
		char buf[NUMBER_TO_ASCII_MAX_LEN];
		DoubleToAscii(buf,v,doubleFormat);
		appendstring(0,buf);
	};
	auto appendFloat=[&](float v){
		char buf[NUMBER_TO_ASCII_MAX_LEN];
		FloatToAscii(buf,v,floatFormat);
		appendstring(0,buf);
	};
	auto appendInt=[&](int v){
		char buf[NUMBER_TO_ASCII_MAX_LEN];
		IntToAscii(buf,v);
		appendstring(0,buf);
	};
	auto appendprint=[&](int indent,const char* format,...){
		va_list v;
		va_start(v,format);
		va_list v2;
		va_copy(v2,v);
		int len=vsnprintf(NULL,0,format,v2);
		size_t pos=vec.size();
		vec.resize(vec.size()+indent+len+1);			//ensure room for vsnprintf trailing zero
		memset(vec.data()+pos,' ',indent);
		if(vsnprintf(vec.data()+pos+indent,len+1,format,v)!=len)
			FATAL("fappendprint");
		vec.pop_back(); // remove trailing zero from vsnprintf
		va_end(v);
		va_end(v2);
		if(echo)
			uprintf("%.*s",indent+len,vec.data()+pos);
	};
	std::function<bool(const Dict& dict,int indent)> recurse=[&](const Dict& dict,int indent)->bool{
		//uprintf("%stype \"%s\" name \"%s\" value \"%s\"\n",spaces.c_str(),dict.TypeName(),dict.Name().c_str(),dict.Value().c_str());
		if(compact)
			indent=0;
		if(dict.Name().size()) {
			appendprint(indent,"\"%s\":",dict.Name().c_str());
		}else{
			appendstring(indent,"");
		}
		switch(dict.Type()) {
			case Object: {
				if(compact) {
					appendstring(0,"{");
				}else{
					appendstring(0,"{\n");
				}
				break;
			}
			case Array: {
				if(compact) {
					appendstring(0,"[");
				}else{
					appendstring(0,"[\n");
				}
				break;
			}
			case String: {
				const char* p;
				int len;
				dict.GetString(&p,&len);
				appendprint(0,"\"%.*s\"",len,p);
				break;
			}
			case Byte:
			case Int:
			case Int64:
			case Float:
			case Double:
			case Bool: {
				if(dict.IsTypedArray()) {
					appendstring(0,"[");
					const char* data=(char*)dict.GetTypedArrayData();
					int step=TypeBytesize(dict.Type());
					int pos=(int)vec.size();
					for(uint32_t i=0;i<dict.Size();i++) {
						if(i) {
							appendstring(0,",");
							if((int)vec.size()-pos>200) {				//Break extreme lines
								appendstring(0,"\n");
								pos=(int)vec.size();
								appendstring(indent+spacingLevel,"");
							}
						}
						switch(dict.Type()) {
							case Double: {
								appendDouble(*(double*)data);
								break;
							}
							case Float: {
								appendFloat(*(float*)data);
								break;
							}
							case Byte: {
								appendInt(*(char*)data);
								break;
							}
							case Int: {
								appendInt(*(int*)data);
								break;
							}
							case Int64: {
								appendprint(0,"%lld",*(int64_t*)data);
								break;
							}
							case Bool: {
								appendprint(0,"%s",*(bool*)data?"true":"false");
								break;
							}
							default: {
								uprintf("Unhandled type %d\n",dict.Type());
								break;
							}
						}
						data+=step;
					}
					appendstring(0,"]");
				}else{
					switch(dict.Type()) {
						case Byte: {
							appendInt(dict.m_value.m_byteval);
							break;
						}
						case Int: {
							appendInt(dict.m_value.m_intval);
							break;
						}
						case Int64: {
							appendprint(0,"%lld",dict.m_value.m_int64val);
							break;
						}
						case Float: {
							appendFloat(dict.m_value.m_floatval);
							break;
						}
						case Double: {
							appendDouble(dict.m_value.m_doubleval);
							break;
						}
						case Bool: {
							appendprint(0,"%s",dict.m_value.m_boolval?"true":"false");
							break;
						}
						default:
							FATAL("Unhandled type %d",dict.Type());
					}
				}
				break;
			}
			default:
				FATAL("Unhandled type %d",dict.Type());
		}
		const Dict* child=dict.First();
		while(child) {
			recurse(*child,indent+spacingLevel);
			child=child->Next();
			if(child) {
				if(compact) {
					appendstring(0,",");
				}else{
					appendstring(0,",\n");
				}
			}else{
				if(!compact) {
					appendstring(0,"\n");
				}
			}
		}
		if(dict.Type()==Dict::Object) {
			appendstring(indent,"}");
		}else
		if(dict.Type()==Dict::Array) {
			appendstring(indent,"]");
		}
		return true;
	};
	recurse(*this,indent);
	appendstring(0,"\n");
	if(appendZero)
		vec.push_back(0);
	std::string json;
	json.assign((char*)vec.data(),vec.size());
	return json;
}

//void PrintNodesRecursive(const Dict* node,int depth);

void Dict::Dump(int indent)const {
	WriteToJson(false,indent,4,true);
}



bool Dict::ReadFromJson(const char* buffer,size_t length) {
	Clear();
	DictJsonParser parser;
	parser.Parse(this,buffer,length);
	//Dump();
	//PrintNodesRecursive(this,0);
	return true;
}
bool Dict::AddFromJson(const char* buffer,size_t length) {
	//Clear();
	DictJsonParser parser;
	parser.Parse(this,buffer,length);
	//Dump();
	//PrintNodesRecursive(this,0);
	return true;
}
static const double decimals[]={
	pow(10,0),pow(10,-1),pow(10,-2),pow(10,-3),pow(10,-4),pow(10,-5),pow(10,-6),pow(10,-7),pow(10,-8),pow(10,-9),
	pow(10,-10),pow(10,-11),pow(10,-12),pow(10,-13),pow(10,-14),pow(10,-15),pow(10,-16),pow(10,-17),pow(10,-18),pow(10,-19),
	pow(10,-20),pow(10,-21),pow(10,-22),pow(10,-23),pow(10,-24),pow(10,-25),pow(10,-26),pow(10,-27),pow(10,-28),pow(10,-29),
	pow(10,-30),pow(10,-31)
};

// This function reads as many characters as can usefully form a floating-point number including exponent.
// The input string does not have to be 0-terminated.
double DictJsonParser::AsciiToFloat(const char* x) {
	uint64_t val=0;
	int sign=0;
	int decimal=-1;
	if(*x=='-') { sign=1; x++; }

	int exp=0;

	auto readExp=[&]() {
		bool esign=false;
		if(*x=='-') { esign=true; x++; }
		if(*x=='+') { esign=false; x++; }

		while(char ch=*x++) {
			if(ch<'0'||ch>'9') break;
			exp=exp*10+(ch-='0');
		}
		if(esign) exp=-exp;
	};
	auto getPow10=[](int exp) -> double {
		if(!exp) return 1.0;
		if(exp<0&&-exp<(int)countof(decimals)) {
			return decimals[-exp];
		}
		return pow(10,exp);
	};

	while(char ch=*x++) {
		if(ch=='e'||ch=='E') {
			readExp();
			goto done;
		}
		if(!((ch>='0'&&ch<='9')||ch=='.')) {
			goto done;
		}
		if(ch=='.') {
			if(decimal>=0) goto done;
			decimal++;
			continue;
		}
		uint64_t nval=val*10;
		if(nval>=(((1ULL)<<52)-1)) {
			if(decimal>=0) goto done;
			decimal=1;
			while(char ch=*x++) {
				if(ch=='e'||ch=='E') {
					readExp();
					goto fin;
				}
				if(ch<'0'||ch>'9'||ch=='.') goto fin;
				decimal++;
			}
fin:		double dval=val*getPow10(decimal+exp);
			return sign ? -dval : dval;
		}
		val=nval+ch-'0';
		if(decimal>=0) decimal++;
	}
done:
	if(decimal<0) decimal=0;
	double dval=val*getPow10(-decimal+exp);
	return sign ? -dval : dval;
}

DictJsonParser::DictJsonParser() {
}

int Dict::Depth()const {
	auto c=m_parent;
	int cnt=0;
	while(c) {
		c=c->m_parent;
		cnt++;
	}
	return cnt;
}
void DictJsonParser::HandleToken(Token token,const char* begin,const char* end) {
	switch(token) {
		case Token::Key: {
			m_currentKey.assign(begin,end);
			//uprintf("%d token Key %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			break;
		}
		case Token::Bool: {
			bool value=begin[0]=='t';
			if(m_currentNode->Type()==Dict::ValueType::Array) {
				m_currentBoolArray.push_back(value);
			}else{
				m_currentNode->Set(m_currentKey.c_str(),value);
				m_currentKey="";
			}
			//uprintf("%d token Bool %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			break;
		}
		case Token::Number: {
			while(std::isspace(*(end-1)) && end>begin)
				--end;// Drop trailing whitespace
			double value=DictJsonParser::AsciiToFloat(begin);
			if(m_currentNode->Type()==Dict::ValueType::Array) {
				m_currentNumberArray.push_back(value);
			}else{
				m_currentNode->Set(m_currentKey.c_str(),value);
				m_currentKey="";
			}
			//uprintf("%d token Number %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			break;
		}
		case Token::String: {
			m_currentNode->Set(m_currentKey.c_str(),begin,(int)(end-begin));
			m_currentKey="";
			//uprintf("%d token String %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			break;
		}
		case Token::Array: {
			//uprintf("%d token Array %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			m_currentNode=m_currentNode->AddArrayNode(m_currentKey.c_str());
			m_currentKey="";
			break;
		}
		case Token::ArrayEnd: {
			//uprintf("%d token ArrayEnd %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			if(!m_currentNode->HasChildNodes()) {
				if(m_currentNumberArray.size()) {
					if(m_currentBoolArray.size())
						FATAL("mixed types in typed array not allowed");
					m_currentNode->AllocatValueTypedArray(Dict::ValueType::Double,(int)m_currentNumberArray.size());
					double* p=(double*)m_currentNode->GetTypedArrayData();
					memcpy(p,&m_currentNumberArray[0],m_currentNumberArray.size()*sizeof(double));
					m_currentNumberArray.clear();
				}else
				if(m_currentBoolArray.size()) {
					m_currentNode->AllocatValueTypedArray(Dict::ValueType::Bool,(int)m_currentBoolArray.size());
					bool* p=(bool*)m_currentNode->GetTypedArrayData();
					for(int i=0;i!=(int)m_currentBoolArray.size();i++)
						p[i]=m_currentBoolArray[i];
					m_currentBoolArray.clear();
				}
			}else{
				if(m_currentNumberArray.size())
					FATAL("typed array number has children");
				if(m_currentBoolArray.size())
					FATAL("typed array bool has children");
			}
			m_currentNode=m_currentNode->Parent();
			break;
		}
		case Token::Object: {
			//uprintf("%d token Object %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			if(!m_currentNode->Parent() && !m_currentKey.size())	//ensure single root
				break;
			m_currentNode=m_currentNode->AddObjectNode(m_currentKey.c_str());
			m_currentKey="";
			break;
		}
		case Token::ObjectEnd: {
			if(!m_currentNode->Parent())	//ensure single root
				break;
			//uprintf("%d token ObjectEnd %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			m_currentNode=m_currentNode->Parent();
			break;
		}
		case Token::Unknown: {
			uprintf("%d token Unknown %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			break;
		}
		case Token::Null: {
			uprintf("%d token Null %s\n",m_currentNode->Depth(),std::string(begin,end).c_str());
			break;
		}
		default: {
			FATAL("unsupported token %d. invalid json\n",token);
			break;
		}
	}
}

void DictJsonParser::Parse(Dict* dict,const char* json,size_t size) {
	m_currentNode=dict;
	m_json=json;
	m_jsonSize=size;
	m_jsonPos=0;
	while(1) {
		size_t pos=m_jsonPos++;
		if(pos>=m_jsonSize) return;
		int ch=m_json[pos];
		switch(ch) {
			case EOF:
				return;
			case '[':
				ParseArray(pos);
				return;
			case '{':
				ParseObject(pos);
				return;
		}
	}
}

void DictJsonParser::ParseArray(size_t pos) {
	HandleToken(Token::Array,m_json+pos,m_json+pos+1);
	while(1) {
		ParseValue(pos);
		pos=m_jsonPos++;
		if(pos>=m_jsonSize) return;
		int ch=m_json[pos];
		switch(ch) {
			case EOF:
				return;
			case ']':
				HandleToken(Token::ArrayEnd,m_json+pos,m_json+pos+1);
				return;
			case ',':
				break;
		}
	}
}

bool DictJsonParser::ParseValue(size_t pos) {
	bool gotVal=false;
	while(1) {
		pos=m_jsonPos++;
		if(pos>=m_jsonSize) return gotVal;
		int ch=m_json[pos];
		switch(ch) {
			case EOF:
				return gotVal;
			case '}':
			case ',':
			case ']':
				--m_jsonPos;
				return gotVal;
			case '[':
				gotVal=true;
				ParseArray(pos);
				break;
			case '{':
				gotVal=true;
				ParseObject(pos);
				break;
			case 'n': {
				size_t i=ParseChars(pos,"ull");
				if(i) {
					gotVal=true;
					HandleToken(Token::Null,m_json+pos,m_json+pos+4);
				} else {
					//find next comma
					return true;
				}
				break;
			}
			case 't': {
				size_t i=ParseChars(pos,"rue");
				if(i) {
					gotVal=true;
					HandleToken(Token::Bool,m_json+pos,m_json+pos+4);
				} else {
					//find next comma
					return true;
				}
				break;
			}
			case 'f': {
				size_t i=ParseChars(pos,"alse");
				if(i) {
					gotVal=true;
					HandleToken(Token::Bool,m_json+pos,m_json+pos+5);
				} else {
					//find next comma
					return true;
				}
				break;
			}
			case '-':
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				gotVal=true;
				ParseNumber(pos);
				break;
			case '"':
				gotVal=true;
				ParseString(pos);
				break;
		}
	}
}

size_t DictJsonParser::ParseChars(size_t pos,const char* chars) {
	int p=0;
	while(1) {
		if(!chars[p]) break;
		pos=m_jsonPos++;
		if(pos>=m_jsonSize) break;
		int ch=m_json[pos];
		if(ch!=chars[p++])
			return 0;
	}
	return p;
}

void DictJsonParser::ParseNumber(size_t pos) {
	const char* begin=m_json+pos;
	while(1) {
		pos=m_jsonPos++;
		if(pos>=m_jsonSize) goto fin;
		int ch=m_json[pos];
		switch(ch) {
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case '.':
			case 'e':
			case '-':
				break;
			case ',':
			case ']':
			case '}':
				--m_jsonPos;
				goto fin;
			default:
				goto fin;
		}
	}
fin:
	HandleToken(Token::Number,begin,m_json+pos+1);
}

size_t DictJsonParser::ParseString(size_t pos,bool isKey) {
	const char* begin=m_json+pos+1;
	while(1) {
		pos=m_jsonPos++;
		if(pos>=m_jsonSize) return 0;
		int ch=m_json[pos];
		switch(ch) {
			case '\\': //escape
				pos=m_jsonPos++;
				if(pos>=m_jsonSize) return 0;
				ch=m_json[pos];
				switch(m_json[pos]) {
					case 'u':
						m_jsonPos+=4;
						if(m_jsonPos>=m_jsonSize) return 0;
						break;
					default:
						break;
				}
				break;
			case '"':
				HandleToken(isKey ? Token::Key : Token::String,begin,m_json+pos);
				return 0;
		}
	}
}

void DictJsonParser::ParseObject(size_t pos) {
	HandleToken(Token::Object,m_json+pos,m_json+pos+1);
	while(1) {
		pos=m_jsonPos++;
		if(pos>=m_jsonSize) return;
		int ch=m_json[pos];
		switch(ch) {
			case EOF:
				return;
			case ':':
				ParseValue(pos);
				break;
			case '"':
				ParseString(pos,true);
				break;
			case '}':
				HandleToken(Token::ObjectEnd,m_json+pos,m_json+pos+1);
				return;
			case ',': //next field
				break;
		}
	}
}





static std::string defaultjson=R"({
	"str":"superlongstringnotfittinginsidebuffer",
	"sarray":["foo0foo1foo2foo3foo4foo5foo6foo7foo8foo9fooafoobfoocfood","bar"],
	"earray":[],
	"array1":[4.00123456789,20000000000,3.888888888888],
	"array2":[{"tst1":1},{"tst2":2},{"tst3":3}],
	"barray":[true,true,false],
	"tal":123,
	"bit":true,
	"proxy":"127.0.0.1:8990",
	"server":"127.0.0.1:8999"
})";

/*
std::string tstjson=R"({
	"dataset":[
		[
			[{"txt11":"john1"},
			{"txt12":"sus1"}]
		],
		[
			[{"txt21":"john2"},
			{"txt22":"sus2"}]
		]
	],
	"array2":[{"tst1":1},{"tst2":2},{"tst3":3}],
	"foo0":4,
	"dataset":[
		1,2,3
	],
	"foo1":{
		"txt":"nano"
	}
})";

std::string tstjson1=R"({
	"settings":[]
})";

	Dict dict;
	if(!dict.ReadFromJson(tstjson1))
		FATAL("ARRGHH");
	dict.Dump();
*/

void TestDictBinary() {
	Dict info;
	{
		info.Set("type","camera");
		Dict* cams=info.AddArrayNode("cameras");
		Dict camInfo;
		int size = 1337;
		camInfo.Set("size",size);
		uint8_t id=4;
		camInfo.Set("id",id);
		int64_t captureTime=1683318966741847;
		camInfo.Set("captureTime",captureTime);
		int frameIndex=67181;
		camInfo.Set("frameIndex",frameIndex);
		cams->PushBack(camInfo);
	}
	info.Dump();
	std::vector<uint8_t> bin;
	info.WriteToBinary(&bin);

	Dict rinfo;
	rinfo.ReadFromBinary(bin);
	rinfo.Dump();


	bool getOk;
	std::string s;
	getOk=rinfo.Get("type", &s);
	ASSERT(getOk,"'type' not found");
	ASSERT(s=="camera", "not matching 'camera'");

	Dict* arr=rinfo.Find("cameras");
	ASSERT(arr, "'cameras' array not found");
	ASSERT(arr->Size()==1, "'cameras' array size mismatch");

	int64_t i64;
	getOk=arr->At(0).Get("captureTime", &i64);
	ASSERT(getOk, "int64_t 'captureTime' not found");
	ASSERT(i64==1683318966741847, "int64_t 'captureTime' value not matching 1683318966741847");

	int i;
	getOk=arr->At(0).Get("size", &i);
	ASSERT(getOk, "int 'size' not found");
	ASSERT(i==1337, "int 'size' value not matching 1337");

	getOk=arr->At(0).Get("frameIndex", &i);
	ASSERT(getOk, "int 'frameIndex' not found");
	ASSERT(i==67181, "int 'frameIndex' value not matching 4");

	getOk=arr->At(0).Get("id", &i);
	ASSERT(getOk, "int 'id' not found");
	ASSERT(i==4, "int 'id' value not matching 4");
}


void TestDict() {
	TestDictBinary();
	char buf[NUMBER_TO_ASCII_MAX_LEN];
	double testNumbers[]={8880,0.5,123450000,123456};
	for(int i=0;i!=(int)countof(testNumbers);i++) {
		DoubleToAscii(buf,testNumbers[i]);
		double val=DictJsonParser::AsciiToFloat(buf);
		if(val!=testNumbers[i])
			FATAL("double parsing failed");

	}
	Dict dict;
	std::vector<char> data;
	if (1) {//!LoadFile(&data,"ini.json")) {
		if (!dict.ReadFromJson(defaultjson.c_str(),defaultjson.size())) {
			FATAL("Invalid default ini file");
		}
		dict.Dump();
		std::string json=dict.WriteToJson(false);
		dict.ReadFromJson(json.c_str(),json.size());

		//if (!SaveFile("ini.json",std::vector<char>(json.begin(),json.end()))) {
		//	FATAL("Error writing default ini file");
		//}

		std::vector<double> ttal({1.0,2.0});
		dict.SetTypedArray("ttal",ttal);
		std::vector<double> ttal1({1.0});
		dict.SetTypedArray("ttal",ttal1);
		dict.GetTypedArray("ttal",&ttal);
		if(ttal!=ttal1)
			FATAL("should be euqal");


		Dict* darray=dict.AddArrayNode("darray");
		darray->PushBack(1.0);
		darray->PushBack(2.0);
		darray->PushBack(3.0);

		std::vector<double> dtal;
		darray->GetNumberArray(&dtal);

		if(!dict.SetTypedArray("darray",dtal))
			FATAL("should return true");

		std::vector<double> dtal1;
		dict.GetNumberArray("darray",&dtal1);

		std::vector<double> etal;
		if(!dict.SetTypedArray("etal",etal))
			FATAL("should return true");

		std::vector<double> etal1;
		if(!dict.GetTypedArray("etal",&etal1))
			FATAL("should return true");

		if(etal!=etal1)
			FATAL("should be euqal");

		std::string json1=dict.WriteToJson();
		Dict dict1;
		dict1.ReadFromJson(json1);

		if(!dict.GetTypedArray("etal",&etal1))
			FATAL("should return true");

		if(etal!=etal1)
			FATAL("should be euqal");

		etal={1.0};
		if(!dict.SetTypedArray("etal",etal))
			FATAL("should return true");

		if(!dict.GetTypedArray("etal",&etal1))
			FATAL("should return true");

		if(etal!=etal1)
			FATAL("should be euqal");

		json1=dict.WriteToJson();
		dict1.ReadFromJson(json1);

		if(!dict.GetTypedArray("etal",&etal1))
			FATAL("should return true");

		if(etal!=etal1)
			FATAL("should be euqal");

		std::vector<double> tal({ 1,2,3 });
		dict.SetTypedArray("lurt",tal.data(),3);
		{
			//V2 v(1,2);
			//dict.SetTypedArray("vec2",v,2);

			double val;
			dict.Get("tal",&val,456.0);
			Dict* array1=dict.Find("array1");
			if (array1 && array1->IsTypedArray() && array1->Type() == Dict::Double) {
				double da[5];
				int sz=(int)array1->Size();
				if (dict.GetTypedArray("array1",da,sz)) {
					for (int i=0; i != sz; i++) {
						uprintf("array1 %d %f\n",i,da[i]);
					}
				}
			}

			Dict* array2=dict.Find("array2");
			if (array2) {
				for (auto it=array2->begin(); it != array2->end(); ++it) {
					Dict* child=&*it;
					child->Dump();
				}
			}
		}

	}
	else {
		dict.ReadFromJson(data.data(),data.size());
	}
	dict.Dump();
	uint8_t buffer[1024];
	int pos=0;
	dict.WriteToBinary(buffer,&pos);

	std::vector<uint8_t> vec;
	dict.WriteToBinary(&vec);

	std::string str=dict.WriteToJson();
	uprintf("binary data size %d json size %d BinarySize returned %d\n",pos,str.size(),dict.BinarySize());

	Dict dict1;
	int pos1=0;
	dict1.ReadFromBinary(buffer,&pos1);
	dict1.Dump();
	//exit(0);
}









