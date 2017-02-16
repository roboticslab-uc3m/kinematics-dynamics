// Copyright: 
// Author: 
// CopyPolicy: 

//////////////////////////////////////////////////////////////////////////
// 
// This is a configuration file to explain TEO to SWIG
//
// SWIG, for the most part, understands TEO auto-magically.
// There are a few things that need to be explained:
//  + use of multiple inheritance
//  + use of names that clash with special names in Java/Python/Perl/...
//  + use of templates

%module teo

%include "std_vector.i"  /* Do not doubt about the importance of this line */

//%import "yarp.i"

%{
/* Includes the header in the wrapper code */
#include "ICartesianControl.h"
%}

/* Parse the header file to generate wrappers */
%include "ICartesianControl.h"

%extend teo::ICartesianControl
{
    int stat(std::vector<double> &x)
    {
        int buffer;
        bool ok = self->stat(buffer, x);
        if (!ok) return 0;
        return buffer;
    }
}

%{
#include <yarp/dev/all.h>
teo::ICartesianControl *viewICartesianControl(yarp::dev::PolyDriver& d)
{
    teo::ICartesianControl *result;
    d.view(result);
    return result;
}
%}
extern teo::ICartesianControl *viewICartesianControl(yarp::dev::PolyDriver& d);


