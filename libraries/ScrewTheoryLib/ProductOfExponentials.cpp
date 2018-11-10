// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ProductOfExponentials.hpp"

#include <ColorDebug.h>

using namespace roboticslab;

// -----------------------------------------------------------------------------

PoeExpression::PoeExpression(const std::vector<MatrixExponential> & _exps, const KDL::Frame & _H_ST)
    : exps(_exps),
      H_ST(_H_ST)
{}

// -----------------------------------------------------------------------------

bool PoeExpression::evaluate(const KDL::JntArray & q, KDL::Frame & H)
{
    if (exps.size() != q.rows())
    {
        CD_WARNING("Size mismatch: %d (terms of PoE) != %d (joint array size).\n", exps.size(), q.rows());
        return false;
    }

    H = KDL::Frame::Identity();

    for (int i = 0; i < exps.size(); i++)
    {
        H = H * exps[i].asFrame(q(i));
    }

    H = H * H_ST;

    return true;
}

// -----------------------------------------------------------------------------

KDL::Chain PoeExpression::toChain() const
{
    // TODO
    KDL::Chain chain;
    return chain;
}

// -----------------------------------------------------------------------------

PoeExpression PoeExpression::fromChain(const KDL::Chain & chain)
{
    // TODO
    PoeExpression poe;
    return poe;
}

// -----------------------------------------------------------------------------
