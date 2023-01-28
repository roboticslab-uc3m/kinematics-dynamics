#include "gtest/gtest.h"

#include <cmath>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/PolyDriver.h>

#include "ICartesianSolver.h"

namespace roboticslab::test
{

/**
 * @ingroup kinematics-dynamics-tests
 * @brief Tests \ref KdlSolver ikin and idyn on a simple mechanism.
 */
class KdlSolverTest : public testing::Test
{
public:
    void SetUp() override
    {
        yarp::os::Bottle solverOptions;
        solverOptions.add(yarp::os::Value::makeList("device KdlSolver"));
        solverOptions.add(yarp::os::Value::makeList("gravity (0.0 -10.0 0.0)")); // note gravity is applied on the Y axis
        solverOptions.add(yarp::os::Value::makeList("numLinks 1"));
        // the CoG is referred to the segment's tip frame: https://github.com/orocos/orocos_kinematics_dynamics/issues/170
        solverOptions.add(yarp::os::Value::makeList("link_0 (A 1.0) (mass 1.0) (cog -0.5 0.0 0.0) (inertia 1.0 1.0 1.0)"));
        solverOptions.add(yarp::os::Value::makeList("mins (-180.0)"));
        solverOptions.add(yarp::os::Value::makeList("maxs (180.0)"));

        if (!solverDevice.open(solverOptions))
        {
            yError() << "solverDevice not valid:" << solverOptions.find("device").asString();
            return;
        }

        if (!solverDevice.view(iCartesianSolver))
        {
            yError() << "Could not view ICartesianSolver in" << solverOptions.find("device").asString();
            return;
        }
    }

    void TearDown() override
    {
        solverDevice.close();
    }

protected:
    yarp::dev::PolyDriver solverDevice;
    roboticslab::ICartesianSolver * iCartesianSolver;
};

TEST_F(KdlSolverTest, KdlSolverFwdKin1)
{
    std::vector<double> q(1), x;
    q[0] = 0.0;
    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));
    ASSERT_NEAR(x[0], 1.0, 1e-9);
    ASSERT_NEAR(x[1], 0.0, 1e-9);
    ASSERT_NEAR(x[2], 0.0, 1e-9);
}

TEST_F(KdlSolverTest, KdlSolverFwdKin2)
{
    std::vector<double> q(1), x;
    q[0] = 90.0;
    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));
    ASSERT_NEAR(x[0], 0.0, 1e-9);
    ASSERT_NEAR(x[1], 1.0, 1e-9);
    ASSERT_NEAR(x[2], 0.0, 1e-9);
}

TEST_F(KdlSolverTest, KdlSolverInvKin1)
{
    std::vector<double> xd(6), qGuess(1), q;
    xd[0] = 1.0; // x
    xd[1] = 0.0; // y
    xd[2] = 0.0; // z
    xd[3] = 0.0; // o(x)
    xd[4] = 0.0; // o(y)
    xd[5] = 0.0; // o(z)
    qGuess[0] = 0.0;
    ASSERT_TRUE(iCartesianSolver->invKin(xd, qGuess, q));
    ASSERT_EQ(q.size(), 1);
    ASSERT_NEAR(q[0], 0.0, 1e-3);
}

TEST_F(KdlSolverTest, KdlSolverInvKin2)
{
    std::vector<double> xd(6), qGuess(1), q;
    xd[0] = 0.0; // x
    xd[1] = 1.0; // y
    xd[2] = 0.0; // z
    xd[3] = 0.0; // o(x)
    xd[4] = 0.0; // o(y)
    xd[5] = M_PI / 2;  // o(z)
    qGuess[0] = 90.0;
    ASSERT_TRUE(iCartesianSolver->invKin(xd, qGuess, q));
    ASSERT_EQ(q.size(), 1);
    ASSERT_NEAR(q[0], 90.0, 1e-3);
}

TEST_F(KdlSolverTest, KdlSolverInvDyn1)
{
    std::vector<double> q(1), t;
    q[0] = -90.0;
    ASSERT_TRUE(iCartesianSolver->invDyn(q, t));
    ASSERT_EQ(t.size(), 1);
    ASSERT_NEAR(t[0], 0.0, 1e-9); //-- T = F*d = 1kg * 10m/s^2 * 0m = 0 N*m
}

TEST_F(KdlSolverTest, KdlSolverInvDyn2)
{
    std::vector<double> q(1), t;
    q[0] = 0.0;
    ASSERT_TRUE(iCartesianSolver->invDyn(q, t));
    ASSERT_EQ(t.size(), 1);
    ASSERT_NEAR(t[0], 5.0, 1e-9); //-- T = F*d = 1kg * 10m/s^2 * 0.5m = 5 N*m
}

TEST_F(KdlSolverTest, KdlSolverInvDyn3)
{
    std::vector<double> q(1), qdot(1, 0.0), qdotdot(1, 0.0), ftip(3, 0.0), t;
    q[0] = 0.0;
    ASSERT_TRUE(iCartesianSolver->invDyn(q, qdot, qdotdot, ftip, t));
    ASSERT_EQ(t.size(), 1);
    ASSERT_NEAR(t[0], 5.0, 1e-9); //-- T = F*d = 1kg * 10m/s^2 * 0.5m = 5 N*m
}

} // namespace roboticslab::test
