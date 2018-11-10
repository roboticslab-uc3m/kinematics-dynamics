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

        PoeExpression(const std::vector<MatrixExponential> & exps, const KDL::Frame & H_S_T);

        KDL::Frame getTransform() const
        { return H_S_T; }

        int size() const
        { return exps.size(); }

        const MatrixExponential & exponentialAtJoint(int i) const
        { return exps.at(i); }

        bool evaluate(const KDL::JntArray & q, KDL::Frame & H);

        KDL::Chain toChain() const;

        static PoeExpression fromChain(const KDL::Chain & chain);

    private:

        PoeExpression();

        std::vector<MatrixExponential> exps;
        KDL::Frame H_S_T;
};

}  // namespace roboticslab

#endif  // __PRODUCT_OF_EXPONENTIALS_HPP__
