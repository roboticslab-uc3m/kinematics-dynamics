// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __PRODUCT_OF_EXPONENTIALS_HPP__
#define __PRODUCT_OF_EXPONENTIALS_HPP__

#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "MatrixExponential.hpp"

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PoeExpression
{
    public:

        explicit PoeExpression(const KDL::Frame & H_S_T = KDL::Frame::Identity()) : H_S_T(H_S_T) {}

        void append(const MatrixExponential & exp, const KDL::Frame & H_new_old = KDL::Frame::Identity())
        { exps.push_back(exp.cloneWithBase(H_new_old)); }

        void append(const PoeExpression & poe, const KDL::Frame & H_new_old = KDL::Frame::Identity());

        KDL::Frame getTransform() const
        { return H_S_T; }

        int size() const
        { return exps.size(); }

        const MatrixExponential & exponentialAtJoint(int i) const
        { return exps.at(i); }

        void changeBaseFrame(const KDL::Frame & H_new_old);

        void changeToolFrame(const KDL::Frame & H_new_old)
        { H_S_T = H_S_T * H_new_old; }

        bool evaluate(const KDL::JntArray & q, KDL::Frame & H) const;

        void reverseSelf();

        PoeExpression makeReverse() const;

        KDL::Chain toChain() const;

        static PoeExpression fromChain(const KDL::Chain & chain);

    private:

        std::vector<MatrixExponential> exps;
        KDL::Frame H_S_T;
};

}  // namespace roboticslab

#endif  // __PRODUCT_OF_EXPONENTIALS_HPP__
