// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCREW_THEORY_TOOLS_HPP__
#define __SCREW_THEORY_TOOLS_HPP__

#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-libraries
 * \defgroup ScrewTheoryLib
 *
 * @brief Contains classes related to Screw Theory solvers and tools.
 */

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class MatrixExponential
{
public:
    enum motion
    {
        ROTATION,
        TRANSLATION
    };

    MatrixExponential(motion motionType, const KDL::Vector & axis, const KDL::Vector & origin);
    KDL::Frame asFrame() const;
    motion getMotionType() const;
    bool liesOnAxis(const KDL::Vector & point) const;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PoeExpression
{
    public:
        PoeExpression(const std::vector<MatrixExponential> & exps, const KDL::Frame & H_ST);
        KDL::Frame getTransform() const;
        int size() const;
        const MatrixExponential & exponentialAtJoint(int i) const;
        KDL::Frame evaluate(const KDL::JntArray & q);
        KDL::Chain toChain() const;
        static PoeExpression fromChain(const KDL::Chain & chain);

    private:
        PoeExpression();
        std::vector<MatrixExponential> exps;
        KDL::Frame H_ST;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ScrewTheorySubproblem
{};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahan1 : public ScrewTheorySubproblem
{
public:
    PadenKahan1(const MatrixExponential & exp1, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahan2 : public ScrewTheorySubproblem
{
public:
    PadenKahan2(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahan3 : public ScrewTheorySubproblem
{
public:
    PadenKahan3(const MatrixExponential & exp1, const KDL::Vector & p, const KDL::Vector & k);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PardosGotor4 : public ScrewTheorySubproblem
{
public:
    PardosGotor4(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ScrewTheoryProblem
{
public:
    bool solve(const KDL::Frame & H_ST, std::vector<KDL::JntArray> & solutions);

private:
    ScrewTheoryProblem();
    ScrewTheoryProblem(const ScrewTheoryProblem &); // dynamic alloc?
    ScrewTheoryProblem & operator=(const ScrewTheoryProblem &); // dynamic alloc?
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ScrewTheoryProblemBuilder
{
public:
    ScrewTheoryProblemBuilder(const KDL::Frame & H_ST_0);
    void addStep(ScrewTheorySubproblem * subproblem);
    ScrewTheoryProblem build();
};

}  // namespace roboticslab

#endif  // __SCREW_THEORY_TOOLS_HPP__
