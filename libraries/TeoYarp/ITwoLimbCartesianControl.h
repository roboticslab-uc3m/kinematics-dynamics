// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_TWO_LIMB_CARTESIAN_CONTROL__
#define __I_TWO_LIMB_CARTESIAN_CONTROL__

#include <vector>

#include <yarp/os/Vocab.h>

#define VOCAB_CC_STAT VOCAB4('s','t','a','t')
#define VOCAB_CC_STOP VOCAB4('s','t','o','p')
#define VOCAB_CC_STEP VOCAB4('s','t','e','p')

#define VOCAB_CC_NOT_CONTROLLING VOCAB4('c','c','n','c')
#define VOCAB_CC_MOVS_CONTROLLING VOCAB4('c','c','s','c')

#define DEFAULT_DURATION 15

namespace roboticslab
{

/**
 *
 * @brief Abstract base for a two limb cartesian control.
 *
 */
class ITwoLimbCartesianControl
{
    public:
        /**
         * Destructor.
         */
        virtual ~ITwoLimbCartesianControl() {}

        /** Inform on control state, and get robot position and perform forward kinematics. */
        virtual bool stat(int &state, std::vector<double> &x) = 0;

        /** Step. */
        virtual bool step() = 0;

        /** stop */
        virtual bool stopControl() = 0;

};

}  // namespace roboticslab

#endif  //  __I_TWO_LIMB_CARTESIAN_CONTROL__

