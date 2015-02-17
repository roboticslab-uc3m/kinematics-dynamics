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

    /** Perform forward kinematics. */
    virtual bool fwdKin(const std::vector<double> &q, std::vector<double> &x, std::vector<double> &o) = 0;

    /** Perform inverse kinematics. */
    virtual bool invKin(const std::vector<double> &x, const std::vector<double> &o, std::vector<double> &q) = 0;

};

}  // namespace teo

#endif  //  __I_CARTESIAN_SOLVER__
