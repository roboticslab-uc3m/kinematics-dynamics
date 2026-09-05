#include <pybind11/pybind11.h>

void init_ICartesianSolver(pybind11::module & m);
void init_ICartesianControl(pybind11::module & m);

PYBIND11_MODULE(roboticslab_kinematics_dynamics, m)
{
    m.doc() = "RoboticsLab UC3M kinematics-dynamics Python wrapper";
    m.attr("__version__") = "1.0.0";
    init_ICartesianSolver(m);
    init_ICartesianControl(m);
}
