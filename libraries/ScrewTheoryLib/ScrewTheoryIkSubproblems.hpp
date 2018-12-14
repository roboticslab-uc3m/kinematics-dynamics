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
 *
 * @brief First Paden-Kahan subproblem
 *
 * Single solution, single revolute joint geometric IK subproblem given by
 * @f$ e\,^{\hat{\xi}\,{\theta}} \cdot p = k @f$
 * (rotation screw applied to a point).
 */
class PadenKahanOne : public ScrewTheoryIkSubproblem
{
public:

    /**
     * @brief Constructor
     *
     * @param id Zero-based joint id of the product of exponentials (POE) term.
     * @param exp POE term.
     * @param p Characteristic point.
     */
    PadenKahanOne(int id, const MatrixExponential & exp, const KDL::Vector & p);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 1; }

private:

    const int id;
    const MatrixExponential exp;
    const KDL::Vector p;
    const KDL::Rotation axisPow;
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Second Paden-Kahan subproblem
 *
 * Dual solution, double revolute joint geometric IK subproblem given by
 * @f$ e\,^{\hat{\xi_1}\,{\theta_1}} \cdot e\,^{\hat{\xi_2}\,{\theta_2}} \cdot p = k @f$
 * (consecutive crossing rotation screws to a point).
 */
class PadenKahanTwo : public ScrewTheoryIkSubproblem
{
public:

    /**
     * @brief Constructor
     *
     * @param id1 Zero-based joint id of the first product of exponentials (POE) term.
     * @param id2 Zero-based joint id of the second POE term.
     * @param exp1 First POE term.
     * @param exp2 Second POE term.
     * @param p Characteristic point.
     * @param r Point of intersection between both screw axes.
     */
    PadenKahanTwo(int id1, int id2, const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p, const KDL::Vector & r);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 2; }

private:

    const int id1, id2;
    const MatrixExponential exp1, exp2;
    const KDL::Vector p, r, axesCross;
    const KDL::Rotation axisPow1, axisPow2;
    const double axesDot;
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Third Paden-Kahan subproblem
 *
 * Dual solution, single revolute joint geometric IK subproblem given by
 * @f$ \left \| e\,^{\hat{\xi}\,{\theta}} \cdot p - k \right \| = \delta @f$
 * (rotation screw for moving @f$ p @f$ to a distance @f$ \delta @f$ from @f$ k @f$).
 */
class PadenKahanThree : public ScrewTheoryIkSubproblem
{
public:

    /**
     * @brief Constructor
     *
     * @param id Zero-based joint id of the product of exponentials (POE) term.
     * @param exp POE term.
     * @param p First characteristic point.
     * @param k Second characteristic point.
     */
    PadenKahanThree(int id, const MatrixExponential & exp, const KDL::Vector & p, const KDL::Vector & k);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 2; }

private:

    const int id;
    const MatrixExponential exp;
    const KDL::Vector p, k;
    const KDL::Rotation axisPow;
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief First Pardos-Gotor subproblem
 *
 * Single solution, single prismatic joint geometric IK subproblem given by
 * @f$ e\,^{\hat{\xi}\,{\theta}} \cdot p = k @f$
 * (translation screw applied to a point, see @cite pardosgotor2018str_handbook).
 */
class PardosGotorOne : public ScrewTheoryIkSubproblem
{
public:

    /**
     * @brief Constructor
     *
     * @param id Zero-based joint id of the product of exponentials (POE) term.
     * @param exp POE term.
     * @param p Characteristic point.
     */
    PardosGotorOne(int id, const MatrixExponential & exp, const KDL::Vector & p);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 1; }

private:

    const int id;
    const MatrixExponential exp;
    const KDL::Vector p;
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Second Pardos-Gotor subproblem
 *
 * Single solution, double prismatic joint geometric IK subproblem given by
 * @f$ e\,^{\hat{\xi_1}\,{\theta_1}} \cdot e\,^{\hat{\xi_2}\,{\theta_2}} \cdot p = k @f$
 * (consecutive translation screws to a point, see @cite pardosgotor2018str_handbook).
 */
class PardosGotorTwo : public ScrewTheoryIkSubproblem
{
public:

    /**
     * @brief Constructor
     *
     * @param id1 Zero-based joint id of the first product of exponentials (POE) term.
     * @param id2 Zero-based joint id of the second POE term.
     * @param exp1 First POE term.
     * @param exp2 Second POE term.
     * @param p Characteristic point.
     */
    PardosGotorTwo(int id1, int id2, const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 1; }

private:

    const int id1, id2;
    const MatrixExponential exp1, exp2;
    const KDL::Vector p, crossPr2;
    const double crossPr2Norm;
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Third Pardos-Gotor subproblem
 *
 * Dual solution, single prismatic joint geometric IK subproblem given by
 * @f$ \left \| e\,^{\hat{\xi}\,{\theta}} \cdot p - k \right \| = \delta @f$
 * (translation screw for moving @f$ p @f$ to a distance @f$ \delta @f$ from @f$ k @f$,
 * see @cite pardosgotor2018str_handbook).
 */
class PardosGotorThree : public ScrewTheoryIkSubproblem
{
public:

    /**
     * @brief Constructor
     *
     * @param id Zero-based joint id of the product of exponentials (POE) term.
     * @param exp POE term.
     * @param p First characteristic point.
     * @param k Second characteristic point.
     */
    PardosGotorThree(int id, const MatrixExponential & exp, const KDL::Vector & p, const KDL::Vector & k);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 2; }

private:

    const int id;
    const MatrixExponential exp;
    const KDL::Vector p, k;
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Fourth Pardos-Gotor subproblem
 *
 * Dual solution, double revolute joint geometric IK subproblem given by
 * @f$ e\,^{\hat{\xi_1}\,{\theta_1}} \cdot e\,^{\hat{\xi_2}\,{\theta_2}} \cdot p = k @f$
 * (two consecutive parallel rotation screws applied to a point,
 * see @cite pardosgotor2018str_handbook).
 */
class PardosGotorFour : public ScrewTheoryIkSubproblem
{
public:

    /**
     * @brief Constructor
     *
     * @param id1 Zero-based joint id of the first product of exponentials (POE) term.
     * @param id2 Zero-based joint id of the second POE term.
     * @param exp1 First POE term.
     * @param exp2 Second POE term.
     * @param p Characteristic point.
     */
    PardosGotorFour(int id1, int id2, const MatrixExponential & exp1, const MatrixExponential & exp2, const KDL::Vector & p);

    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & solutions) const;

    virtual int solutions() const
    { return 2; }

private:

    const int id1, id2;
    const MatrixExponential exp1, exp2;
    const KDL::Vector p, n;
    const KDL::Rotation axisPow;
};

}  // namespace roboticslab

#endif  // __SCREW_THEORY_IK_SUBPROBLEMS_HPP__
