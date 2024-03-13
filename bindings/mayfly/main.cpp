#include "shared/types.h"
#include "shared/dict.h"
#include "dicttopython.h"
#include <pybind11/embed.h>

namespace py = pybind11;
void TestDictToPython();

int main() {
    py::scoped_interpreter guard;
	TestDictToPython();
	return 0;
}

void TestDictToPython() {
	const char* str = 1+ R"(
{
    "type":"imu",
    "imu":{
        "acc":[0.122109,0.646458,-9.754326,0.114926,0.653640,-9.775875,0.093377,0.682372,-9.790240,0.114926,0.632092,-9.783057],
        "rads":[0.002131,0.002131,-0.001065,0.002131,0.001065,-0.001065,0.002131,0.001065,-0.001065,0.002131,0.001065,-0.002131],
        "temp":[45,45,45,45],
        "dt":[256,256,256,258],
        "t":[1687249870902495,1687249870912422,1687249870922357,1687249870932364],
        "index":[18091,18092,18093,18094]
    }
})";

	Dict dict;

	dict.ReadFromJson(str);
	dict.Dump();
	py::print(DictToPython(dict));

	Dict* c = dict.AddObjectNode("child");
	std::vector<float> acc = {0.122109,0.646458,-9.754326,0.114926,0.653640,-9.775875,0.093377,0.682372,-9.790240,0.114926,0.632092,-9.783057};
	bool val[] = {true,false,false,false};
	char bytes[] = {17,18,19,20};
	std::vector<int> dt = {256,256,256,258};
	std::vector<int64_t> t = {1687249870902495,1687249870912422,1687249870922357,1687249870932364};
	c->SetTypedArray("acc", acc);
	c->SetTypedArray("val", val, countof(val));
	c->SetTypedArray("bytes", bytes, countof(bytes));
	c->SetTypedArray("dt", dt.data(), (int)dt.size());
	c->SetTypedArray("t", t.data(), (int)t.size());
	dict.Dump();
	py::print(DictToPython(dict));

	const char* str2 = "{\"arr\":[[1,2,3],[\"one\",\"two\"]], \"val\":32.2}";
	dict.ReadFromJson(str2);
	dict.Dump();
	py::print(DictToPython(dict));

	const char* str3 = "[ [1,2,3], [\"one\",\"two\"] ]";
	dict.ReadFromJson(str3);
	dict.Dump();
	py::print(DictToPython(dict));
}
