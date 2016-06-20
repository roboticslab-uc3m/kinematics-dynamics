// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_CONTROL__
#define __I_CARTESIAN_CONTROL__

#include <vector>

#define VOCAB_STAT VOCAB4('s','t','a','t')
#define VOCAB_INV VOCAB3('i','n','v')

#define VOCAB_CCS_STOP VOCAB4('s','t','o','p')

namespace teo
{

/**
 *
 * @brief Abstract base for a cartesian control.
 *
 */
class ICartesianControl
{
    public:
        /**
         * Destructor.
         */
        virtual ~ICartesianControl() {}

        /** Get robot position and perform forward kinematics. */
        virtual bool stat(int &status, std::vector<double> &x) = 0;

        /** Perform inverse kinematics (using robot position as initial guess) but do not move. */
        virtual bool inv(const std::vector<double> &xd, std::vector<double> &q) = 0;
};

}  // namespace teo

#endif  //  __I_CARTESIAN_CONTROL__

