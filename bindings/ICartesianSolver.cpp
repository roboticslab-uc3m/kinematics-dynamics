#include <pybind11/pybind11.h>

#include <yarp/dev/PolyDriver.h>

#include "ICartesianSolver.h"

namespace py = pybind11;
using namespace roboticslab;

void init_ICartesianSolver(py::module & m)
{
    py::module::import("yarp");

    py::class_<ICartesianSolver, std::shared_ptr<ICartesianSolver>>(m, "ICartesianSolver")
        .def("getNumJoints", &ICartesianSolver::getNumJoints)
        .def("getNumTcps", &ICartesianSolver::getNumTcps)
        .def("appendLink", &ICartesianSolver::appendLink)
        .def("restoreOriginalChain", &ICartesianSolver::restoreOriginalChain)
        .def("inverseKinematics", &ICartesianSolver::inverseKinematics)
        .def("diffInverseKinematics", &ICartesianSolver::diffInverseKinematics)
        .def("inverseDynamics", py::overload_cast<const std::vector<double> &, std::vector<double> &>(&ICartesianSolver::inverseDynamics))
        .def("inverseDynamics", py::overload_cast<const std::vector<double> &, const std::vector<double> &, const std::vector<double> &, const std::vector<double> &, std::vector<double> &, ICartesianSolver::Frame>(&ICartesianSolver::inverseDynamics));

    m.def("viewICartesianSolver", [](yarp::dev::PolyDriver & driver) {
        ICartesianSolver * result;
        driver.view(result);
        return result;
    });
}
