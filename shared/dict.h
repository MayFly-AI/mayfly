#pragma once

#include <cstdint>
#include <cstring>
#include <stdlib.h>
#include <stack>
#include <string>
#include <assert.h>
#include <functional>

class Dict {
	public:
		friend class DictJsonParser;
		enum ValueType : uint16_t {
			NA=0,
			Byte=2,
			Int=3,
			Int64=4,
			Float=5,
			Double=6,
			Bool=7,
			String=8,
			Object=9,
			Array=10
		};
		Dict();
		Dict(const Dict&);
		explicit Dict(const char* json);
		virtual ~Dict();

		//std inspired iterator
		class iterator {
			friend class Dict;
			public:
				iterator operator++(){
					if (m_current)
						m_current=m_current->m_next;
					return *this;
				}
				iterator operator++(int){
					iterator retval=*this;
					if (m_current!=nullptr)
						m_current=m_current->m_next;
					return retval;
				}
				bool operator==(const iterator& other)const{return m_current==other.m_current;}
				bool operator!=(const iterator& other)const{return m_current!=other.m_current;}
				Dict& operator*(){return *m_current;}
				Dict* operator->(){return m_current;}
			protected:
				Dict* m_current;
				iterator(Dict* node) : m_current(node){}
		};
		class const_iterator {
				friend class Dict;
			public:
				const_iterator operator++(){
					if (m_current!=nullptr)
						m_current=m_current->m_next;
					return *this;
				}
				const_iterator operator++(int){
					const_iterator retval=*this;
					if (m_current!=nullptr)
						m_current=m_current->m_next;
					return retval;
				}
				bool operator==(const const_iterator& other)const{return m_current==other.m_current;}
				bool operator!=(const const_iterator& other)const{return m_current!=other.m_current;}
				const Dict& operator*(){return *m_current;}
				const Dict* operator->()const{return m_current;}
			protected:
				Dict* m_current;
				const_iterator(Dict* node) : m_current(node){}
		};
		iterator begin();
		const_iterator begin()const;
		iterator end(){return m_end;}
		const_iterator end()const {return m_cend;}
		static iterator m_end;
		static const_iterator m_cend;

		//Access
		Dict* Parent(){return m_parent;}
		const Dict* Parent()const{return m_parent;}
		ValueType Type()const{return m_type;}
		const char* TypeName()const;
		std::string FullName()const;			//recursive prepend name with parent names
		std::string Name()const;
		std::string Value()const;
		//std::string TypedArrayValue()const;
		bool IsString()const{return m_type==String;}
		bool IsObject()const{return m_type==Object;}
		bool IsArray()const{return m_type==Array;}
		bool IsTypedArray()const;
		bool IsNumber()const{return m_type!=String && m_type!=Object && m_type!=Array && !IsTypedArray();}
		bool HasChildNodes()const{return m_first ? true:false;}

		virtual void Clear();
		Dict& At(size_t index)const;
		size_t Size()const{return m_children;}
		int Children()const{return m_children;}

		union ValueUnion {
			char m_byteval;
			int m_intval;
			int64_t m_int64val;
			bool m_boolval;
			double m_doubleval;
			float m_floatval;
			char m_stringval[sizeof(double)];	//For short strings
			void* m_arrayval;					//Pointer to long strings and typed arrays
		};

		bool Exists(const char* name)const{return Find(name) ? true:false;}
		bool Exists(const std::string& name)const{return Exists(name.c_str());}

		const Dict* Find(const char* p,int len)const;
		Dict* Find(const char* p,int len) {return (Dict*)((const Dict*)this)->Find(p,len);}
		Dict* Find(const char* p){return Find(p,(int)strlen(p));}
		const Dict* Find(const char* p)const{return Find(p,(int)strlen(p));}
		const Dict* Find(const std::string& str)const{return Find(str.c_str(),(int)str.size());}

		Dict& operator=(const Dict& value);

		//Child access
		bool Get(const char* name,bool* value,bool default_value=false)const;
		bool Get(const char* name,char* value,char default_value=0)const;
		bool Get(const char* name,int* value,int default_value=0)const;
		bool Get(const char* name,int64_t* value,int64_t default_value=0)const;
		bool Get(const char* name,double* value,double default_value=0.0)const;
		bool Get(const char* name,float* value,float default_value=0.0f)const;
		bool Get(const char* name,std::string* value,std::string default_value=std::string())const;

		bool GetNumberArray(const char* name,std::vector<double>* vec)const;	//both on array and typed array, converts if type is different, returns false if conversion is not possible ie. objects and strings

		bool GetTypedArray(const char* name,std::vector<float>* p)const;
		bool GetTypedArray(const char* name,std::vector<double>* p)const;
		bool GetTypedArray(const char* name,bool* value,size_t items)const;
		bool GetTypedArray(const char* name,char* value,size_t items)const;
		bool GetTypedArray(const char* name,int* value,size_t items)const;
		bool GetTypedArray(const char* name,int64_t* value,size_t items)const;
		bool GetTypedArray(const char* name,double* value,size_t items)const;
		bool GetTypedArray(const char* name,float* value,size_t items)const;

		Dict* Set(const char* name,bool value);
		Dict* Set(const char* name,char value);
		Dict* Set(const char* name,int value);
		Dict* Set(const char* name,int64_t value);
		Dict* Set(const char* name,float value);
		Dict* Set(const char* name,double value);
		Dict* Set(const char* name,const char* value,int len);
		Dict* Set(const char* name,const char* value) {
			return Set(name,value,(int)strlen(value));
		}
		Dict* Set(const char* name,const std::string& value) {
			return Set(name,value.c_str(),(int)value.size());
		}

		Dict* SetTypedArray(const char* name,const bool* value,size_t items);
		Dict* SetTypedArray(const char* name,const char* value,size_t items);
		Dict* SetTypedArray(const char* name,const int* value,size_t items);
		Dict* SetTypedArray(const char* name,const int64_t* value,size_t items);
		Dict* SetTypedArray(const char* name,const float* value,size_t items);
		Dict* SetTypedArray(const char* name,const double* value,size_t items);

		Dict* SetTypedArray(const char* name,const std::vector<double>& p){return SetTypedArray(name,p.data(),(int)p.size());}
		Dict* SetTypedArray(const char* name,const std::vector<float>& p){return SetTypedArray(name,p.data(),(int)p.size());}

		//Node access
		void SetName(const char* name,int len);
		void SetName(const char* name) {
			SetName(name,(int)strlen(name));
		}
		void SetType(const ValueType& type);

		bool SetArray(float* value,size_t items);
		bool SetArray(double* value,size_t items);

		bool GetNumber(double* v)const;											//get any format number, returns false if conversion is not possible ie. objects and strings

		bool GetDouble(double* v)const;
		bool GetFloat(float* v)const;
		bool GetBool(bool* v)const;
		bool GetInt(int* v)const;
		bool GetInt64(int64_t* v)const;
		bool GetByte(char* v)const;
		bool GetString(const char** v,int* len)const;
		bool GetString(std::string* v)const;

		void Set(const Dict& source);
		void SetDouble(double value);
		void SetFloat(float value);
		void SetInt(int value);
		void SetInt64(int64_t value);
		void SetByte(char value);
		void SetBool(bool value);
		void SetString(const char* value);
		void SetString(const std::string& value);
		void SetString(const char* string,size_t length);

		bool GetNumberArray(std::vector<double>* p)const;						//both on array and typed array, converts if type is different, returns false if conversion is not possible ie. objects and strings
		bool GetTypedArray(std::vector<double>* p)const;

		//Construct/destroy
		Dict& Add(const char* name);
		Dict& Add(const std::string& name);
		Dict& Add(const char* p,int len);

		void Copy(const Dict& source,bool copyName=true);
		Dict* InsertCopy(const Dict& source);

		// Add an object to the end of array or typed array
		Dict* PushBack();
		bool PushBack(double value);
		bool PushBack(float value);
		bool PushBack(int value);
		bool PushBack(int64_t value);
		bool PushBack(char value);
		bool PushBack(bool value);
		bool PushBack(const Dict& value);

		Dict* AddObjectNode(const char* name,int len);
		Dict* AddArrayNode(const char* name,int len);

		//wrappers
		Dict* AddObjectNode() {
			return AddObjectNode(0,0);
		}
		Dict* AddObjectNode(const std::string& name) {
			return AddObjectNode(name.c_str(),(int)name.size());
		}
		Dict* AddArrayNode(const std::string& name) {
			return AddArrayNode(name.c_str(),(int)name.size());
		}
		Dict* AddArrayNode() {
			return AddArrayNode(0,0);
		}

		void AllocatValueTypedArray(ValueType type,int n);

		void Dump(int indent=0)const;

		int CalculateArrayIndex()const;			//Really slow, bad idea, but sometimes..

		//Import/export
		bool AddFromJson(const char* buffer,size_t length);
		bool AddFromJson(const std::string& json) {
			return AddFromJson(json.c_str(),json.size());
		}

		bool ReadFromJson(const char* buffer,size_t length);
		bool ReadFromJson(const std::string& json) {
			return ReadFromJson(json.c_str(),json.size());
		}
		//std::string WriteToJson(bool appendZero=false)const;
		std::string WriteToJson(bool appendZero=false,int indent=0,int spacingLevel=0,bool echo=false,bool compact=false,const char* floatFormat=0,const char* doubleFormat=0)const;

		//Advanced access
		void ForceSetType(ValueType type){m_type=type;}
		const ValueUnion& GetValueUnion()const{return m_value;}
		const void* GetTypedArrayData()const;
		Dict* First(){return m_first;}
		Dict* Last(){return m_last;}
		Dict* Next(){return m_next;}
		Dict* Prev(){return m_prev;}

		const Dict* First()const{return m_first;}
		const Dict* Last()const{return m_last;}
		const Dict* Next()const{return m_next;}
		const Dict* Prev()const{return m_prev;}
	protected:
		//Access helpers
		void AllocateAndCopyTypedArray(ValueType type,const void* vals,int n);
		Dict* SetTypedArray(const char* name,ValueType type,const void* vals,int n);
		bool GetTypedArray(ValueType type,void* vals,int n)const;
		bool GetTypedArray(const char* name,ValueType type,void* vals,int n)const;
		void Set(ValueType type,const void* value);
		bool PushBack(ValueType type,const void* value);
		bool HasAllocatedValue()const;	//Node has allocated value,can be string or typed array

		//Construct/destroy
		Dict* NewChild();
		void UnlinkChild(Dict* child);
		void ClearContent();
		void ClearValue();
		Dict* FindOrAdd(const char* p,int len);
		Dict* FindOrAdd(const char* p) {
			return FindOrAdd(p,(int)strlen(p));
		}

		Dict* m_parent=0;
		Dict* m_prev=0;					//Siblings
		Dict* m_next=0;
		Dict* m_first=0;					//Children
		Dict* m_last=0;
		ValueUnion m_value;
		const char* m_name=0;
		ValueType m_type=NA;
		uint16_t m_nameLength=0;
		uint16_t m_valueBytesize=0;		//String or TypedArray bytesize, zero for other types, allocated if larger than sizeof(ValueUnion)
		uint16_t m_children=0;			//Number of child elements or array length

		//Debug
		//void DumpValue(std::string& buffer)const;
		//void DumpTypedArray(std::string& buffer)const;
		//void DumpRecursive(int indent)const;
		int Depth()const;


	public:
		int BinarySize()const{return BinarySize(this);}

		void ReadFromBinary(const std::vector<uint8_t>& mb);
		void ReadFromBinary(const uint8_t* buffer,int* pos);

		void WriteToBinary(std::vector<uint8_t>* mb)const;
		void WriteToBinary(uint8_t* buffer,int* pos)const;

	protected:
		int BinarySize(const Dict* node)const;
		void WriteToBinary(void* context,void(*write)(void* context,const void* data,int size),const Dict* node)const;
		void ReadFromBinary(const uint8_t** data,Dict* node);
};

Dict::ValueType StringToType(const char* typeName);
uint32_t TypeBytesize(Dict::ValueType type);
const char* TypeToString(const Dict::ValueType& type);
