// Copyright: 
// Author: 
// CopyPolicy: 

//////////////////////////////////////////////////////////////////////////
// 
// This is a configuration file to explain kinematics_dynamics to SWIG
//
// SWIG, for the most part, understands kinematics_dynamics auto-magically.
// There are a few things that need to be explained:
//  + use of multiple inheritance
//  + use of names that clash with special names in Java/Python/Perl/...
//  + use of templates

%module "kinematics_dynamics"

%include "std_vector.i"  /* Do not doubt about the importance of this line */
%include "typemaps.i"

%apply int *OUTPUT { int *state };
%apply double *OUTPUT { double *timestamp };

//%import "yarp.i"

%define SWIG_PREPROCESSOR_SHOULD_SKIP_THIS %enddef

%{
/* Includes the header in the wrapper code */
#include "ICartesianSolver.h"
#include "ICartesianControl.h"
%}

/* Parse the header file to generate wrappers */
%include "ICartesianSolver.h"
%include "ICartesianControl.h"

%{
#include <yarp/dev/PolyDriver.h>
roboticslab::ICartesianSolver *viewICartesianSolver(yarp::dev::PolyDriver& d)
{
    roboticslab::ICartesianSolver *result;
    d.view(result);
    return result;
}
%}
extern roboticslab::ICartesianSolver *viewICartesianSolver(yarp::dev::PolyDriver& d);

%{
#include <yarp/dev/PolyDriver.h>
roboticslab::ICartesianControl *viewICartesianControl(yarp::dev::PolyDriver& d)
{
    roboticslab::ICartesianControl *result;
    d.view(result);
    return result;
}
%}
extern roboticslab::ICartesianControl *viewICartesianControl(yarp::dev::PolyDriver& d);

