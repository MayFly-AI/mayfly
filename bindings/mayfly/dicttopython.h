#pragma once
#include <pybind11/pybind11.h>

class Dict;
pybind11::dict DictToPython(const Dict& node);

