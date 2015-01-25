// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_SOLVER__
#define __I_CARTESIAN_SOLVER__

namespace teo
{

class ICartesianSolver
{
public:
    /**
     * Destructor.
     */
    virtual ~ICartesianSolver() {}

    virtual bool fwdKin(const yarp::sig::Vector &inUnits, yarp::sig::Vector &x, yarp::sig::Vector &o) = 0;

};

}  // namespace teo

#endif  //  __I_CARTESIAN_SOLVER__
