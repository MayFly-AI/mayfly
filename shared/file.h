#pragma once

#include <vector>
#include <string>

bool LoadFile(std::vector<char>* data,const std::string& filename);
bool SaveFile(const std::string& filename,const void* p,size_t len);
bool SaveFile(const std::string& filename,const std::vector<char>& data);
bool DirectoryExists(const std::string& path);
std::string LoadFile(const std::string& filename,bool binary);
std::string GetResourcePath(const char* path,const char* filename);

void AddFilePathRemap(const char* alias,const char* path);
void AddFilePathRemap(const char* alias,const std::string& path);
std::string GetFileNameRemap(const char* fileName);
std::string GetFileNameRemap(const std::string& fileName);

std::string GetExecutablePath();
std::string GetExecutableDir();

bool CreateDirRecursive(const char* strDirectoryName);

bool GetDirectoryFiles(std::vector<std::string>* files,std::vector<std::string>* directories,const char* dir);

class File {
	public:
		File();
		explicit File(const char* pszFileName);
		explicit File(const std::string& pszFileName);
		~File();
		bool Valid()const{return m_strFileName.size() ? true:false;}
		//bool Exists();
		//bool Delete();
		//bool Copy(const char* pszDestinationFileName);
		//bool Move(const char* pszDestinationFileName);
		void SetFileName(const char* pszFileName);
		void SetFileName(const std::string& pszFileName);
		const char* GetFileName()const{return m_strFileName.c_str();}
		//FileTime GetWriteTime();
		//bool SetWriteTime(const FileTime& FileTime);
		int GetSize();
		bool Read(std::vector<char>* buf)const;
		//bool Read(MemoryBuffer& MemoryBuffer);
		//bool Read(MemoryBuffer* mb);
		bool Write(const void* p,int len);
		//bool Write(const MemoryBuffer& mb);
		bool Write(const std::vector<char>& buf);
		//template<class T> bool Write(const Span<T>& s) { return Write(s.Data(),(int)s.ByteSize()); }
		bool Append(const void* p,int len);
		bool AppendString(const char* p);
	protected:
		std::string m_strFileName;
};
