#ifndef STD_EXT_H
#define STD_EXT_H

#include <string>
#include <vector>
#include "shared/math.h"

#ifdef _WIN32
#define FORMATSTRING(buf,format) va_list args;va_start(args,format);vsprintf_s(buf,sizeof(buf),format,args);va_end(args);
#else
#define FORMATSTRING(buf,format) va_list args;va_start(args,format);vsnprintf(buf,sizeof(buf),format,args);va_end(args);
#endif

namespace stdx {
	std::vector<std::string> Split(const std::string& s,char seperator);
	std::string format_string(const char* format,...);
	std::string spaces(int count);
	std::string FromStdArray(const std::vector<std::string>& lst);
	std::string format_vector(const std::vector<int>& s,const char* format="%d",const char* seperator=",");
	std::string format_vector(const std::vector<uint8_t>& s,const char* format="%d",const char* seperator=",");
	std::string format_vector(const std::vector<float>& s,const char* format="%g",const char* seperator=",");
	void tolower(std::string* str);
	std::string tolower(const std::string& str);
	bool ends_with(const std::string& value,const std::string& ending);
	bool EndsWith(const std::string& value,const std::string& ending);
	bool exists(const std::string& str,const std::vector<std::string>& strings);
	void ReplaceAll(std::string* str, const std::string& from, const std::string& to);
	int hash32(const std::string& str);
	std::string to_string(const M44& m);
	int memicmp(const void* vs1,const void* vs2,size_t n);
}

#endif//STD_EXT_H
