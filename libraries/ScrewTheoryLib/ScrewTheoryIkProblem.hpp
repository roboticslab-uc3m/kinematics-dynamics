// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SCREW_THEORY_IK_PROBLEM_HPP__
#define __SCREW_THEORY_IK_PROBLEM_HPP__

#include <utility>
#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "ProductOfExponentials.hpp"

namespace roboticslab
{

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ScrewTheoryIkSubproblem
{
public:

    typedef std::pair<int, double> JointIdToSolution;
    typedef std::vector<JointIdToSolution> JointIdsToSolutions;
    typedef std::vector<JointIdsToSolutions> Solutions;

    virtual ~ScrewTheoryIkSubproblem() {}

    virtual Solutions solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform = KDL::Frame::Identity()) const = 0;

    virtual int solutions() const = 0;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ScrewTheoryIkProblem
{
public:

    typedef std::vector<const ScrewTheoryIkSubproblem *> Steps;
    typedef std::vector<KDL::JntArray> Solutions;

    ~ScrewTheoryIkProblem();

    bool solve(const KDL::Frame & H_S_T, Solutions & solutions);

    int solutions() const
    { return soln; }

    static ScrewTheoryIkProblem * create(const PoeExpression & poe, const Steps & steps, bool reversed = false);

private:

    enum poe_term
    {
        EXP_KNOWN,
        EXP_COMPUTED,
        EXP_UNKNOWN
    };

    typedef std::vector<KDL::Frame> Frames;
    typedef std::vector<poe_term> PoeTerms;

    // disable instantiation, force users to call builder class
    ScrewTheoryIkProblem(const PoeExpression & poe, const Steps & steps, bool reversed);

    // disable these too, avoid issues related to dynamic alloc
    ScrewTheoryIkProblem(const ScrewTheoryIkProblem &);
    ScrewTheoryIkProblem & operator=(const ScrewTheoryIkProblem &);

    void recalculateFrames(const Solutions & solutions, Frames & frames, PoeTerms & poeTerms);
    bool recalculateFrames(const Solutions & solutions, Frames & frames, PoeTerms & poeTerms, bool backwards);

    KDL::Frame transformPoint(const KDL::JntArray & jointValues, const PoeTerms & poeTerms);

    const PoeExpression poe;

    // we own these, resources freed in destructor
    const Steps steps;

    const bool reversed;

    const int soln;
};

/**
 * @ingroup ScrewTheoryLib
 * @brief ...
 */
class ScrewTheoryIkProblemBuilder
{
public:

    struct PoeTerm
    {
        PoeTerm() : known(false), simplified(false) {}
        bool known, simplified;
    };

    ScrewTheoryIkProblemBuilder(const PoeExpression & poe);

    ScrewTheoryIkProblem * build();

private:

    static std::vector<KDL::Vector> searchPoints(const PoeExpression & poe);

    ScrewTheoryIkProblem::Steps searchSolutions();

    void simplify(int depth);
    void simplifyWithPadenKahanOne(const KDL::Vector & point);
    void simplifyWithPadenKahanThree(const KDL::Vector & point);
    void simplifyWithPardosOne();

    ScrewTheoryIkSubproblem * trySolve(int depth);

    PoeExpression poe;

    std::vector<KDL::Vector> points;
    std::vector<KDL::Vector> testPoints;

    std::vector<PoeTerm> poeTerms;

    static const int MAX_SIMPLIFICATION_DEPTH = 2;
};

}  // namespace roboticslab

#endif  // __SCREW_THEORY_IK_PROBLEM_HPP__
