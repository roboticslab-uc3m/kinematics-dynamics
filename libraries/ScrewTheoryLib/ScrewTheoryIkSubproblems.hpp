// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCREW_THEORY_IK_SUBPROBLEMS_HPP__
#define __SCREW_THEORY_IK_SUBPROBLEMS_HPP__

#include <kdl/frames.hpp>

#include "ScrewTheoryIkProblem.hpp"
#include "MatrixExponential.hpp"

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahan1 : public ScrewTheoryIkSubproblem
{
public:
    PadenKahan1(const MatrixExponential & exp, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahan2 : public ScrewTheoryIkSubproblem
{
public:
    PadenKahan2(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahan3 : public ScrewTheoryIkSubproblem
{
public:
    PadenKahan3(const MatrixExponential & exp, const KDL::Vector & p, const KDL::Vector & k);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosGotor1 : public ScrewTheoryIkSubproblem
{
public:
    PardosGotor1(const MatrixExponential & exp, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosGotor2 : public ScrewTheoryIkSubproblem
{
public:
    PardosGotor2(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosGotor3 : public ScrewTheoryIkSubproblem
{
public:
    PardosGotor3(const MatrixExponential & exp, const KDL::Vector & p, const KDL::Vector & k);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosGotor4 : public ScrewTheoryIkSubproblem
{
public:
    PardosGotor4(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

}  // namespace roboticslab

#endif  // __SCREW_THEORY_IK_SUBPROBLEMS_HPP__
