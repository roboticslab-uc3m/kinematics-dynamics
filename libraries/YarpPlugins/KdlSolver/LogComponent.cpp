#include "LogComponent.hpp"

YARP_LOG_COMPONENT(KDLS, "rl.KdlSolver")

YARP_LOG_COMPONENT(KDLS_QUIET, "rl.KdlSolver",
                               yarp::os::Log::minimumPrintLevel(),
                               yarp::os::Log::minimumForwardLevel(),
                               nullptr, // disable print
                               nullptr) // disable forward
