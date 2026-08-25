//////////////////////////////////////////////////////////////////////////
//
// This is a configuration file to explain kinematics_dynamics to SWIG
//
// SWIG, for the most part, understands kinematics_dynamics auto-magically.
// There are a few things that need to be explained:
//  + use of multiple inheritance
//  + use of names that clash with special names in Java/Python/Perl/...
//  + use of templates

%module "roboticslab_kinematics_dynamics"

%include "std_vector.i"
%include "std_map.i"
%include "typemaps.i"

%define SWIG_PREPROCESSOR_SHOULD_SKIP_THIS %enddef

%{
/* Includes the header in the wrapper code */
#include "ICartesianSolver.h"
#include "ICartesianControl.h"
%}

%template(DVector) std::vector<double>;

%typemap(in, numinputs=0) roboticslab::ICartesianControl::Mode & (roboticslab::ICartesianControl::Mode temp) {
    $1 = &temp;
}

%typemap(argout) roboticslab::ICartesianControl::Mode & {
    %append_output(PyLong_FromLong(static_cast<long>(*$1)));
}

%apply std::vector<double> & OUTPUT { std::vector<double> & x };
%apply std::vector<double> & OUTPUT { std::vector<double> & q };
%apply double & OUTPUT { double & timestamp };
%apply double * OUTPUT { double * value };

/* Parse the header file to generate wrappers */
%include "ICartesianSolver.h"
%include "ICartesianControl.h"

%template(ConfigMap) std::map<roboticslab::ICartesianControl::Config, double>;

%{
#include <yarp/dev/PolyDriver.h>

roboticslab::ICartesianSolver * viewICartesianSolver(yarp::dev::PolyDriver & d)
{
    roboticslab::ICartesianSolver * result;
    d.view(result);
    return result;
}
%}

extern roboticslab::ICartesianSolver * viewICartesianSolver(yarp::dev::PolyDriver & d);

%{
#include <yarp/dev/PolyDriver.h>

roboticslab::ICartesianControl * viewICartesianControl(yarp::dev::PolyDriver & d)
{
    roboticslab::ICartesianControl * result;
    d.view(result);
    return result;
}
%}

extern roboticslab::ICartesianControl * viewICartesianControl(yarp::dev::PolyDriver & d);
