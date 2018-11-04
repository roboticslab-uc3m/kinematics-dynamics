// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCREW_THEORY_IK_SOLVER_HPP__
#define __SCREW_THEORY_IK_SOLVER_HPP__

#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-libraries
 * \defgroup ScrewTheoryIkSolverLib
 *
 * @brief Contains classes related to Screw Theory IK solvers.
 */

/**
 * @ingroup ScrewTheoryIkSolverLib
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
    KDL::Frame expToH();
    motion getMotionType() const;
    bool liesOnAxis(const KDL::Vector & point) const;
};

/**
 * @ingroup ScrewTheoryIkSolverLib
 * @brief ...
 */
class ScrewTheorySubproblem
{};

/**
 * @ingroup ScrewTheoryIkSolverLib
 * @brief ...
 */
class PadenKahan1 : public ScrewTheorySubproblem
{
public:
    PadenKahan1(const MatrixExponential & exp1, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryIkSolverLib
 * @brief ...
 */
class PadenKahan2 : public ScrewTheorySubproblem
{
public:
    PadenKahan2(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryIkSolverLib
 * @brief ...
 */
class PadenKahan3 : public ScrewTheorySubproblem
{
public:
    PadenKahan3(const MatrixExponential & exp1, const KDL::Vector & p, const KDL::Vector & k);
};

/**
 * @ingroup ScrewTheoryIkSolverLib
 * @brief ...
 */
class PardosGotor4 : public ScrewTheorySubproblem
{
public:
    PardosGotor4(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryIkSolverLib
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
 * @ingroup ScrewTheoryIkSolverLib
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

#endif  // __SCREW_THEORY_IK_SOLVER_HPP__
