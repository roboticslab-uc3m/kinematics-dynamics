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

%feature("flatnested");

%include "std_vector.i"
%include "std_map.i"
%include "typemaps.i"
%include "variant.i"

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

%typemap(in, numinputs=0) roboticslab::ICartesianControl::config_map_t &
    (roboticslab::ICartesianControl::config_map_t temp) {
    $1 = &temp;
}

%typemap(in) const roboticslab::ICartesianControl::config_map_t & (int res) {
    res = SWIG_ConvertPtr(
        $input,
        reinterpret_cast<void **>(&$1),
        $descriptor(std::map<
            roboticslab::ICartesianControl::Config,
            std::variant<double, yarp::conf::vocab32_t>
        > *),
        0
    );

    if (!SWIG_IsOK(res)) {
        SWIG_exception_fail(
            SWIG_TypeError,
            "in method '" "$symname" "', argument " "$argnum"
            " of type '" "$type" "'"
        );
    }
}

/* SWIG_ConvertPtr above borrows the Python ConfigMap pointer. */
%typemap(freearg) const roboticslab::ICartesianControl::config_map_t & {
}

%std_variant(ConfigValue, double, yarp::conf::vocab32_t)

%typemap(in, numinputs=0) roboticslab::ICartesianControl::config_value_t *
    (roboticslab::ICartesianControl::config_value_t temp) {
    $1 = &temp;
}

%typemap(argout) roboticslab::ICartesianControl::config_value_t * {
    PyObject *result = SWIG_NewPointerObj(
        new roboticslab::ICartesianControl::config_value_t(*$1),
        $descriptor(std::variant<double, yarp::conf::vocab32_t> *),
        SWIG_POINTER_OWN
    );

    if (!result) {
        SWIG_fail;
    }

    %append_output(result);
}

%typemap(argout) roboticslab::ICartesianControl::config_map_t & {
    PyObject *result = SWIG_NewPointerObj(
        new roboticslab::ICartesianControl::config_map_t(*$1),
        $descriptor(std::map<roboticslab::ICartesianControl::Config,
                             std::variant<double, yarp::conf::vocab32_t> > *),
        SWIG_POINTER_OWN
    );

    if (!result) {
        SWIG_fail;
    }

    %append_output(result);
}

%typemap(in) std::variant<double, yarp::conf::vocab32_t>
    (std::variant<double, yarp::conf::vocab32_t> temp) {
    if (PyFloat_Check($input)) {
        temp = PyFloat_AsDouble($input);
    } else if (PyLong_Check($input)) {
        temp = static_cast<yarp::conf::vocab32_t>(PyLong_AsLong($input));

        if (PyErr_Occurred()) {
            SWIG_fail;
        }
    } else {
        SWIG_exception_fail(
            SWIG_TypeError,
            "expected a float or integer vocabulary value"
        );
    }

    $1 = temp;
}

%typemap(typecheck, precedence=SWIG_TYPECHECK_DOUBLE)
std::variant<double, yarp::conf::vocab32_t> {
    $1 = (PyFloat_Check($input) || PyLong_Check($input))
        ? SWIG_OK
        : SWIG_ERROR;
}

%typemap(in, numinputs=0) roboticslab::ICartesianControl::ControllerState &
    (roboticslab::ICartesianControl::ControllerState temp) {
    $1 = &temp;
}

%typemap(argout) roboticslab::ICartesianControl::ControllerState & {
    PyObject *result = SWIG_NewPointerObj(
        new roboticslab::ICartesianControl::ControllerState(*$1),
        $descriptor(roboticslab::ICartesianControl::ControllerState *),
        SWIG_POINTER_OWN
    );

    if (!result) {
        SWIG_fail;
    }

    %append_output(result);
}

/* Parse the header file to generate wrappers */
%include "ICartesianSolver.h"
%include "ICartesianControl.h"

%extend std::map<roboticslab::ICartesianControl::Config,
                 std::variant<double, yarp::conf::vocab32_t> > {
    void setitem(const roboticslab::ICartesianControl::Config & key,
                 const std::variant<double, yarp::conf::vocab32_t> & value) {
        (*$self)[key] = value;
    }
}

%template(ConfigMap)
    std::map<roboticslab::ICartesianControl::Config,
             std::variant<double, yarp::conf::vocab32_t> >;

%pythoncode %{
def _config_map_setitem(self, key, value):
    if isinstance(value, float):
        value = ConfigValue(value)
    elif isinstance(value, int):
        value = ConfigValue(value)

    return self.setitem(key, value)

ConfigMap.__setitem__ = _config_map_setitem
%}

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
