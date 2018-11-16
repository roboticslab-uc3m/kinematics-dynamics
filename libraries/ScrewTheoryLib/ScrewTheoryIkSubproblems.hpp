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
class PadenKahanOne : public ScrewTheoryIkSubproblem
{
public:
    PadenKahanOne(const MatrixExponential & exp, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahanTwo : public ScrewTheoryIkSubproblem
{
public:
    PadenKahanTwo(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahanThree : public ScrewTheoryIkSubproblem
{
public:
    PadenKahanThree(const MatrixExponential & exp, const KDL::Vector & p, const KDL::Vector & k);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosOne : public ScrewTheoryIkSubproblem
{
public:
    PardosOne(const MatrixExponential & exp, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosTwo : public ScrewTheoryIkSubproblem
{
public:
    PardosTwo(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosThree : public ScrewTheoryIkSubproblem
{
public:
    PardosThree(const MatrixExponential & exp, const KDL::Vector & p, const KDL::Vector & k);
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosFour : public ScrewTheoryIkSubproblem
{
public:
    PardosFour(const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);
};

}  // namespace roboticslab

#endif  // __SCREW_THEORY_IK_SUBPROBLEMS_HPP__
