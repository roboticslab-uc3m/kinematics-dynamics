// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_CONTROL__
#define __I_CARTESIAN_CONTROL__

#include <vector>

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
        virtual bool stat(std::vector<double> &x) = 0;

};

}  // namespace teo

#endif  //  __I_CARTESIAN_CONTROL__

