// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_TWO_LIMB_CARTESIAN_CONTROL__
#define __I_TWO_LIMB_CARTESIAN_CONTROL__

#include <vector>

#include <yarp/os/Vocab.h>

/**
 * @file
 * @brief Contains roboticslab::ITwoLimbCartesianControl and related vocabs.
 * @ingroup TeoYarp
 * @{
 */

/**
 * @name RPC vocabs
 * @{
 */

#define VOCAB_CC_STAT VOCAB4('s','t','a','t') ///< Current state and position
#define VOCAB_CC_STOP VOCAB4('s','t','o','p') ///< Stop control
#define VOCAB_CC_STEP VOCAB4('s','t','e','p') ///< Step

/** @} */

/**
 * @name Control state vocabs
 * @{
 */

#define VOCAB_CC_NOT_CONTROLLING VOCAB4('c','c','n','c')  ///< Not controlling
#define VOCAB_CC_MOVS_CONTROLLING VOCAB4('c','c','s','c') ///< Controlling step commands

/** @} */

#define DEFAULT_DURATION 15

namespace roboticslab
{

/**
 * @brief Abstract base class for a two limb cartesian controller.
 */
class ITwoLimbCartesianControl
{
    public:

        //! Destructor
        virtual ~ITwoLimbCartesianControl() {}

        /**
         * @brief Current state and position
         *
         * Inform on control state, get robot position and perform forward kinematics.
         *
         * @param state Identifier for a cartesian control vocab.
         * @param x 12-element vector describing current position in cartesian space, in two
         * sets of 6 elements for each limb; first three elements of the set denote translation
         * (meters), last three denote rotation in scaled axis-angle representation (radians).
         *
         * @return true on success, false otherwise
         */
        virtual bool stat(int &state, std::vector<double> &x) = 0;

        /**
         * @brief Step
         *
         * Create new gait trajectory and start control.
         *
         * @return true on success, false otherwise
         */
        virtual bool step() = 0;

        /**
         * @brief Stop control
         *
         * Halt current control loop if any and cease movement.
         *
         * @return true on success, false otherwise
         */
        virtual bool stopControl() = 0;

};

}  // namespace roboticslab

/** @} */

#endif  //  __I_TWO_LIMB_CARTESIAN_CONTROL__
