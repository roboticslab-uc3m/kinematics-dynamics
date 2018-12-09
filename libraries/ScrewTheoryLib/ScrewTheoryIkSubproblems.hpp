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

    PadenKahanOne(int id, const MatrixExponential & exp, const KDL::Vector & p);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 1; }

private:

    int id;
    MatrixExponential exp;
    KDL::Vector p;
    KDL::Rotation axisPow;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahanTwo : public ScrewTheoryIkSubproblem
{
public:

    PadenKahanTwo(int id1, int id2, const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p, const KDL::Vector & r);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 2; }

private:

    int id1, id2;
    MatrixExponential exp1, exp2;
    KDL::Vector p, r, axesCross;
    KDL::Rotation axisPow1, axisPow2;
    double axesDot;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class PadenKahanThree : public ScrewTheoryIkSubproblem
{
public:

    PadenKahanThree(int id, const MatrixExponential & exp, const KDL::Vector & p, const KDL::Vector & k);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 2; }

private:

    int id;
    MatrixExponential exp;
    KDL::Vector p, k;
    KDL::Rotation axisPow;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosOne : public ScrewTheoryIkSubproblem
{
public:

    PardosOne(int id, const MatrixExponential & exp, const KDL::Vector & p);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 1; }

private:

    int id;
    MatrixExponential exp;
    KDL::Vector p;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosTwo : public ScrewTheoryIkSubproblem
{
public:

    PardosTwo(int id1, int id2, const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 1; }

private:

    int id1, id2;
    MatrixExponential exp1, exp2;
    KDL::Vector p, crossPr2;
    double crossPr2Norm;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosThree : public ScrewTheoryIkSubproblem
{
public:

    PardosThree(int id, const MatrixExponential & exp, const KDL::Vector & p, const KDL::Vector & k);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 2; }

private:

    int id;
    MatrixExponential exp;
    KDL::Vector p, k;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 * @cite pardosgotor2018str_handbook
 */
class PardosFour : public ScrewTheoryIkSubproblem
{
public:

    PardosFour(int id1, int id2, const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 2; }

private:

    int id1, id2;
    MatrixExponential exp1, exp2;
    KDL::Vector p, n;
    KDL::Rotation axisPow;
};

}  // namespace roboticslab

#endif  // __SCREW_THEORY_IK_SUBPROBLEMS_HPP__
