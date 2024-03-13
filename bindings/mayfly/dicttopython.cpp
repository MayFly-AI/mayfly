#include "dicttopython.h"
#include "shared/misc.h"
#include "shared/types.h"
#include "shared/dict.h"
#include <string>

namespace py = pybind11;

#define ASSIGN_TO_DICT_OR_LIST(x) \
	if(out_dict){(*out_dict)[node->Name().c_str()]=x;} \
	if(out_list){out_list->append(x);}

void DictToPython(py::dict* out_dict, py::list* out_list, const Dict* const node) {
	ASSERT((!out_list&&out_dict) || (!out_dict&&out_list), "lhs not unique");
	if(node->IsTypedArray()){
		py::list lst;
		switch (node->Type()) {
			case Dict::Float: {
				const float* arr = reinterpret_cast<const float*>(node->GetTypedArrayData());
				for(size_t i=0;i<node->Size();++i) lst.append(arr[i]);
			} break;
			case Dict::Double: {
				const double* arr = reinterpret_cast<const double*>(node->GetTypedArrayData());
				for(size_t i=0;i<node->Size();++i) lst.append(arr[i]);
			} break;
			case Dict::Int: {
				const int* arr = reinterpret_cast<const int*>(node->GetTypedArrayData());
				for(size_t i=0;i<node->Size();++i) lst.append(arr[i]);
			} break;
			case Dict::Int64: {
				const int64_t* arr = reinterpret_cast<const int64_t*>(node->GetTypedArrayData());
				for(size_t i=0;i<node->Size();++i) lst.append(arr[i]);
			} break;
			case Dict::Byte: {
				const char* arr = reinterpret_cast<const char*>(node->GetTypedArrayData());
				for(size_t i=0;i<node->Size();++i) lst.append(arr[i]);
			} break;
			case Dict::Bool: {
				const bool* arr = reinterpret_cast<const bool*>(node->GetTypedArrayData());
				for(size_t i=0;i<node->Size();++i) lst.append(arr[i]);
			} break;
			default:
				uprintf("Unknown typed array type");
		}
		ASSIGN_TO_DICT_OR_LIST(lst);
	} else
	if(node->IsString()) {
		std::string str;
		node->GetString(&str);
		ASSIGN_TO_DICT_OR_LIST(str);
	} else
	if(node->IsObject()) {
		py::dict dct;
		const Dict* child=node->First();
		while(child) {
			DictToPython(&dct,nullptr,child);
			child=child->Next();
		}
		ASSIGN_TO_DICT_OR_LIST(dct);
	} else
	if(node->IsArray()) {
		py::list lst;
		const Dict* child=node->First();
		while(child) {
			DictToPython(nullptr,&lst,child);
			child=child->Next();
		}
		ASSIGN_TO_DICT_OR_LIST(lst);
	} else
	if(node->IsNumber()) {
		switch(node->Type()) {
			case Dict::Double: {
				double val;
				node->GetDouble(&val);
				ASSIGN_TO_DICT_OR_LIST(val);
			} break;
			case Dict::Float: {
				float val;
				node->GetFloat(&val);
				ASSIGN_TO_DICT_OR_LIST(val);
			} break;
			case Dict::Bool: {
				bool val;
				node->GetBool(&val);
				ASSIGN_TO_DICT_OR_LIST(val);
			} break;
			case Dict::Int: {
				int val;
				node->GetInt(&val);
				ASSIGN_TO_DICT_OR_LIST(val);
			} break;
			case Dict::Int64: {
				int64_t val;
				node->GetInt64(&val);
				ASSIGN_TO_DICT_OR_LIST(val);
			} break;
			case Dict::Byte: {
				char val;
				node->GetByte(&val);
				ASSIGN_TO_DICT_OR_LIST(val);
			} break;
			default:
				uprintf("Unknown number type");
		}
	} else
	{
		uprintf("Unknown node type");
	}
}
#undef ASSIGN_TO_DICT_OR_LIST

py::dict DictToPython(const Dict& node) {
	ASSERT(!node.Parent(), "Input dict is not a root node");
	ASSERT(node.IsObject(), "Root node is not an object");

	py::dict res;
	const Dict* child=node.First();
	while(child) {
		DictToPython(&res,nullptr,child);
		child=child->Next();
	}
	return res;
}
