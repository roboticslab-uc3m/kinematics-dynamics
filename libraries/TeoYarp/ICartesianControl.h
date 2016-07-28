// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __I_CARTESIAN_CONTROL__
#define __I_CARTESIAN_CONTROL__

#include <vector>

#define VOCAB_CC_STAT VOCAB4('s','t','a','t')
#define VOCAB_CC_INV VOCAB3('i','n','v')
#define VOCAB_CC_MOVJ VOCAB4('m','o','v','j')
#define VOCAB_CC_RELJ VOCAB4('r','e','l','j')
#define VOCAB_CC_MOVL VOCAB4('m','o','v','l')
#define VOCAB_CC_MOVV VOCAB4('m','o','v','v')
#define VOCAB_CC_GCMP VOCAB4('g','c','m','p')
#define VOCAB_CC_FORC VOCAB4('f','o','r','c')
#define VOCAB_CC_STOP VOCAB4('s','t','o','p')

#define VOCAB_CC_NOT_CONTROLLING VOCAB4('c','c','n','c')
#define VOCAB_CC_MOVJ_CONTROLLING VOCAB4('c','c','j','c')
#define VOCAB_CC_MOVL_CONTROLLING VOCAB4('c','c','l','c')
#define VOCAB_CC_MOVV_CONTROLLING VOCAB4('c','c','v','c')
#define VOCAB_CC_GCMP_CONTROLLING VOCAB4('c','c','g','c')
#define VOCAB_CC_FORC_CONTROLLING VOCAB4('c','c','f','c')

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

        /** Inform on control state, and get robot position and perform forward kinematics. */
        virtual bool stat(int &state, std::vector<double> &x) = 0;

        /** Perform inverse kinematics (using robot position as initial guess) but do not move. */
        virtual bool inv(const std::vector<double> &xd, std::vector<double> &q) = 0;

        /** movj */
        virtual bool movj(const std::vector<double> &xd) = 0;

        /** relj */
        virtual bool relj(const std::vector<double> &xd) = 0;

        /** movl */
        virtual bool movl(const std::vector<double> &xd) = 0;

        /** movv */
        virtual bool movv(const std::vector<double> &xdotd) = 0;

        /** gcmp */
        virtual bool gcmp() = 0;

        /** forc */
        virtual bool forc(const std::vector<double> &td) = 0;

        /** stop */
        virtual bool stopControl() = 0;

};

}  // namespace teo

#endif  //  __I_CARTESIAN_CONTROL__

