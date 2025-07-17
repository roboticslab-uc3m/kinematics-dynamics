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
 *
 * @brief Interface shared by all IK subproblems found in Screw Theory applied
 * to Robotics.
 *
 * Derived classes are considered to be immutable.
 */
class ScrewTheoryIkSubproblem
{
public:
    //! Joint configurations
    using JointConfig = std::vector<double>;

    //! Collection of local IK solutions
    using Solutions = std::vector<JointConfig>;

    //! Destructor
    virtual ~ScrewTheoryIkSubproblem() = default;

    /**
     * @brief Finds a closed geometric solution for this IK subproblem
     *
     * Given the product of exponentials (POE) formula
     * @f$ \prod_i e\,^{\hat{\xi}_i\,{\theta_i}} \cdot H_{ST}(0) = H_{ST}(\theta) @f$,
     * invariant and known terms are rearranged to the right side (\p rhs) as follows:
     *
     * @f[
     * \prod_{i=j}^{j+k} e\,^{\hat{\xi}_i\,{\theta_i}} =
     * \left [ \prod_{i=1}^{j-1} e\,^{\hat{\xi}_i\,{\theta_i}} \right ]^{-1} \cdot
     * H_{ST}(\theta) \cdot \left [ H_{ST}(0) \right ]^{-1} \cdot
     * \left [ \prod_{i=j+k+1}^{N} e\,^{\hat{\xi}_i\,{\theta_i}} \right ]^{-1}
     * @f]
     *
     * where @f$ j = \{1, 2, ..., N\} @f$, @f$ k = \{1, 2, ..., N-1\} @f$, @f$ 1 <= j+k <= N @f$.
     *
     * Given @f$ N @f$ terms in the POE formula, @f$ j @f$ of which are unknowns, any
     * characteristic point @f$ p @f$ postmultiplying this expression could be rewritten
     * as @f$ p' @f$ per:
     *
     * @f[
     * \prod_{i=1}^j e\,^{\hat{\xi}_i\,{\theta_i}} \cdot \prod_{i=j+1}^N e\,^{\hat{\xi}_i\,{\theta_i}} \cdot p =
     * \prod_{i=1}^j e\,^{\hat{\xi}_i\,{\theta_i}} \cdot p'
     * @f]
     *
     * where \p pointTransform is the transformation matrix that produces @f$ p' @f$
     * from @f$ p @f$.
     *
     * @param rhs Right-hand side of the POE formula prior to being applied to the
     * right-hand side of this subproblem.
     * @param pointTransform Transformation frame applied to the first (and perhaps
     * only) characteristic point of this subproblem.
     * @param reference Known nearby solutions to be used as reference in case a
     * singularity is found.
     * @param solutions Output vector of local solutions.
     *
     * @return True if all solutions are reachable, false otherwise.
     */
    virtual bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, const JointConfig & reference, Solutions & solutions) const = 0;

    bool solve(const KDL::Frame & rhs, const KDL::Frame & pointTransform, Solutions & _solutions) const
    {
        return solve(rhs, pointTransform, JointConfig(solutions()), _solutions);
    }

    //! Number of local IK solutions
    virtual int solutions() const = 0;

    //! Return a human-readable description of this IK subproblem
    virtual const char * describe() const = 0;
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Proxy IK problem solver class that iterates over a sequence of subproblems
 *
 * This class is immutable. Instantiation is allowed by means of a static builder method.
 *
 * @see ScrewTheoryIkProblemBuilder
 */
class ScrewTheoryIkProblem final
{
public:
    //! Pair of joint ids and an their associated local IK subproblem
    using JointIdsToSubproblem = std::pair<std::vector<int>, const ScrewTheoryIkSubproblem *>;

    //! Ordered sequence of IK subproblems that solve a global IK problem
    using Steps = std::vector<JointIdsToSubproblem>;

    //! Collection of global IK solutions
    using Solutions = std::vector<KDL::JntArray>;

    //! Destructor
    ~ScrewTheoryIkProblem();

    // disable these, avoid issues related to dynamic alloc
    ScrewTheoryIkProblem(const ScrewTheoryIkProblem &) = delete;
    ScrewTheoryIkProblem & operator=(const ScrewTheoryIkProblem &) = delete;

    /**
     * @brief Find all available solutions
     *
     * @param H_S_T Target pose in cartesian space.
     * @param reference Known nearby solutions to be used as reference in case
     * a singularity is found.
     * @param solutions Output vector of solutions stored as joint arrays.
     *
     * @return Vector of Booleans indicating whether the associated solution is reachable.
     */
    std::vector<bool> solve(const KDL::Frame & H_S_T, const KDL::JntArray & reference, Solutions & solutions);

    std::vector<bool> solve(const KDL::Frame & H_S_T, Solutions & solutions)
    {
        return solve(H_S_T, KDL::JntArray(poe.size()), solutions);
    }

    //! Number of global IK solutions
    int solutions() const
    { return soln; }

    //! Solution of the IK problem (if available)
    const Steps & getSteps() const
    { return steps; }

    //! Whether the computed solution is reversed
    bool isReversed() const
    { return reversed; }

    /**
     * @brief Creates an IK solver instance given a sequence of known subproblems
     *
     * @param poe A product of exponentials (POE) formula.
     * @param steps Collection of subproblems that solve this particular IK problem.
     * @param reversed True if the POE has been reversed (in order to find a valid solution).
     *
     * @return An instance of an IK problem solver.
     */
    static ScrewTheoryIkProblem * create(const PoeExpression & poe, const Steps & steps, bool reversed = false);

private:
    enum poe_term
    {
        EXP_KNOWN,
        EXP_COMPUTED,
        EXP_UNKNOWN
    };

    using Frames = std::vector<KDL::Frame>;
    using PoeTerms = std::vector<poe_term>;

    // disable instantiation, force users to call builder class
    ScrewTheoryIkProblem(const PoeExpression & poe, const Steps & steps, bool reversed);

    void recalculateFrames(const Solutions & solutions, Frames & frames, PoeTerms & poeTerms);
    bool recalculateFrames(const Solutions & solutions, Frames & frames, PoeTerms & poeTerms, bool backwards);

    KDL::Frame transformPoint(const KDL::JntArray & jointValues, const PoeTerms & poeTerms);

    const PoeExpression poe;

    // we own these, resources freed in destructor
    const Steps steps;

    PoeTerms poeTerms;
    Frames rhsFrames;
    std::vector<bool> reachability;

    const bool reversed;
    const int soln;
};

/**
 * @ingroup ScrewTheoryLib
 *
 * @brief Automated IK solution finder
 *
 * This class helps to automate the process for configuring a valid IK problem given
 * its geometric data only. It is intended to take care of the generation and
 * configuration of known subproblems, whereas particular joint-space solutions are
 * meant to be computed on runtime by \ref ScrewTheoryIkProblem.
 */
class ScrewTheoryIkProblemBuilder
{
public:
    //! Helper structure that holds the state of a POE term
    struct PoeTerm
    {
        PoeTerm() : known(false), simplified(false) {}
        bool known, simplified;
    };

    /**
     * @brief Constructor
     *
     * @param poe Product of exponentials (POE) formula.
     */
    ScrewTheoryIkProblemBuilder(const PoeExpression & poe);

    /**
     * @brief Finds a valid sequence of geometric subproblems that solve a global IK problem
     *
     * @return An instance of an IK problem solver if valid, null otherwise.
     */
    ScrewTheoryIkProblem * build();

private:
    static std::vector<KDL::Vector> searchPoints(const PoeExpression & poe);

    ScrewTheoryIkProblem::Steps searchSolutions();

    void refreshSimplificationState();

    void simplify(int depth);
    void simplifyWithPadenKahanOne(const KDL::Vector & point);
    void simplifyWithPadenKahanThree(const KDL::Vector & point);
    void simplifyWithPardosOne();

    ScrewTheoryIkProblem::JointIdsToSubproblem trySolve(int depth);

    PoeExpression poe;

    std::vector<KDL::Vector> points;
    std::vector<KDL::Vector> testPoints;

    std::vector<PoeTerm> poeTerms;

    static const int MAX_SIMPLIFICATION_DEPTH = 2;
};

} // namespace roboticslab

#endif // __SCREW_THEORY_IK_PROBLEM_HPP__
