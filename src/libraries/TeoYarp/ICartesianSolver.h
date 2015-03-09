// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_SOLVER__
#define __I_CARTESIAN_SOLVER__

#include <vector>

namespace teo
{

class ICartesianSolver
{
    public:
        /**
         * Destructor.
         */
        virtual ~ICartesianSolver() {}

        /** Get number of links for which the solver has been configured. */
        virtual bool getNumLinks(int* numLinks) = 0;

        /** Perform forward kinematics. */
        virtual bool fwdKin(const std::vector<double> &q, std::vector<double> &x, std::vector<double> &o) = 0;

        /** Perform inverse kinematics. */
        virtual bool invKin(const std::vector<double> &xd, const std::vector<double> &od, const std::vector<double> &qGuess, std::vector<double> &q) = 0;

        /** Perform inverse dynamics. */
        virtual bool invDyn(const std::vector<double> &q, std::vector<double> &t) = 0;
        virtual bool invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t) = 0;

};

}  // namespace teo

#endif  //  __I_CARTESIAN_SOLVER__
