#include <pybind11/pybind11.h>

#include "ICartesianControl.h"

namespace py = pybind11;
using namespace roboticslab;

void init_ICartesianControl(py::module & m)
{
    py::module::import("yarp");
}
