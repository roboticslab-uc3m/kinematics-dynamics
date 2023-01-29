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
        using yarp::os::Value;

        yarp::os::Bottle opts;
        opts.add(Value::makeList("device KdlSolver"));
        opts.add(Value::makeList("gravity (0.0 -10.0 0.0)")); // note gravity is applied on the Y axis
        opts.add(Value::makeList("numLinks 2"));
        // the CoG is referred to the segment's tip frame: https://github.com/orocos/orocos_kinematics_dynamics/issues/170
        opts.add(Value::makeList("link_0 (offset  0.0) (D 0.0) (A 1.0) (alpha 0.0) (mass 1.0) (cog -0.5 0.0 0.0) (inertia 1.0 1.0 1.0)"));
        opts.add(Value::makeList("link_1 (offset 90.0) (D 0.0) (A 1.0) (alpha 0.0) (mass 1.0) (cog -0.5 0.0 0.0) (inertia 1.0 1.0 1.0)"));
        opts.add(Value::makeList("mins (-180.0 -180.0)"));
        opts.add(Value::makeList("maxs (180.0 180.0)"));

        if (!solverDevice.open(opts))
        {
            yError() << "Unable to open solver device";
            return;
        }

        if (!solverDevice.view(iCartesianSolver))
        {
            yError() << "Unable to view ICartesianSolver";
            return;
        }
    }

    void TearDown() override
    {
        solverDevice.close();
    }

protected:
    static constexpr double eps = 1e-9;
    yarp::dev::PolyDriver solverDevice;
    roboticslab::ICartesianSolver * iCartesianSolver;
};

TEST_F(KdlSolverTest, ChainSize)
{
    ASSERT_EQ(iCartesianSolver->getNumJoints(), 2);
    ASSERT_EQ(iCartesianSolver->getNumTcps(), 1);
}

TEST_F(KdlSolverTest, FwdKin1)
{
    std::vector<double> q {0.0, 0.0};
    std::vector<double> x;
    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));
    ASSERT_EQ(x.size(), 6);
    ASSERT_NEAR(x[0], 1.0, eps); // x
    ASSERT_NEAR(x[1], 1.0, eps); // y
    ASSERT_NEAR(x[2], 0.0, eps); // z
    ASSERT_NEAR(x[3], 0.0, eps); // o(x)
    ASSERT_NEAR(x[4], 0.0, eps); // o(y)
    ASSERT_NEAR(x[5], M_PI / 2, eps); // o(z)
}

TEST_F(KdlSolverTest, FwdKin2)
{
    std::vector<double> q {-90.0, 0.0};
    std::vector<double> x;
    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));
    ASSERT_EQ(x.size(), 6);
    ASSERT_NEAR(x[0], 1.0, eps); // x
    ASSERT_NEAR(x[1], -1.0, eps); // y
    ASSERT_NEAR(x[2], 0.0, eps); // z
    ASSERT_NEAR(x[3], 0.0, eps); // o(x)
    ASSERT_NEAR(x[4], 0.0, eps); // o(y)
    ASSERT_NEAR(x[5], 0.0, eps); // o(z)
}

TEST_F(KdlSolverTest, InvKin1)
{
    std::vector<double> xd {1.0, 1.0, 0.0, 0.0, 0.0, M_PI / 2}; // x, y, z, o(x), o(y), o(z)
    std::vector<double> qGuess {0.0, 0.0};
    std::vector<double> q;
    ASSERT_TRUE(iCartesianSolver->invKin(xd, qGuess, q));
    ASSERT_EQ(q.size(), 2);
    ASSERT_NEAR(q[0], 90.0, eps);
    ASSERT_NEAR(q[1], 0.0, eps);
}

TEST_F(KdlSolverTest, InvKin2)
{
    std::vector<double> xd {1.0, -1.0, 0.0, 0.0, 0.0, 0.0}; // x, y, z, o(x), o(y), o(z)
    std::vector<double> qGuess {-90.0, 0.0};
    std::vector<double> q;
    ASSERT_TRUE(iCartesianSolver->invKin(xd, qGuess, q));
    ASSERT_EQ(q.size(), 2);
    ASSERT_NEAR(q[0], -90.0, eps);
    ASSERT_NEAR(q[1], 0.0, eps);
}

TEST_F(KdlSolverTest, InvDyn1)
{
    std::vector<double> q {90.0, -90.0};
    std::vector<double> t;
    ASSERT_TRUE(iCartesianSolver->invDyn(q, t));
    ASSERT_EQ(t.size(), 2);
    ASSERT_NEAR(t[0], 0.0, eps); //-- T = F * d = (1 kg * 10 m/s^2 * 0 m) + (1 kg * 10 m/s^2 * 0 m) = 0 N*m
    ASSERT_NEAR(t[1], 0.0, eps); //-- T = F * d = 1 kg * 10 m/s^2 * 0 m = 0 N*m
}

TEST_F(KdlSolverTest, InvDyn2)
{
    std::vector<double> q {0.0, 0.0};
    std::vector<double> t;
    ASSERT_TRUE(iCartesianSolver->invDyn(q, t));
    ASSERT_EQ(t.size(), 2);
    ASSERT_NEAR(t[0], 15.0, eps); //-- T = F * d = (1 kg * 10 m/s^2 * 0.5 m) + (1 kg * 10 m/s^2 * 1 m) = 15 N*m
    ASSERT_NEAR(t[1], 0.0, eps); //-- T = F * d = 1 kg * 10 m/s^2 * 0 m = 0 N*m
}

TEST_F(KdlSolverTest, InvDyn3)
{
    std::vector<double> q {0.0, 0.0};
    std::vector<double> qdot {0.0, 0.0};
    std::vector<double> qdotdot {0.0, 0.0};
    std::vector<double> ftip {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // fx, fy, fz, tx, ty, tz
    std::vector<double> t;
    ASSERT_TRUE(iCartesianSolver->invDyn(q, qdot, qdotdot, ftip, t));
    ASSERT_EQ(t.size(), 2);
    ASSERT_NEAR(t[0], 15.0, eps); //-- T = F * d = (1 kg * 10 m/s^2 * 0.5 m) + (1 kg * 10 m/s^2 * 1 m) = 15 N*m
    ASSERT_NEAR(t[1], 0.0, eps); //-- T = F * d = 1 kg * 10 m/s^2 * 0 m = 0 N*m
}

TEST_F(KdlSolverTest, InvDyn4)
{
    std::vector<double> q {0.0, 0.0};
    std::vector<double> qdot {0.0, 0.0};
    std::vector<double> qdotdot {0.0, 0.0};
    std::vector<double> ftip {0.0, 1.0, 0.0, 0.0, 0.0, 0.0}; // Fx, Fy, Fz, Tx, Ty, Tz
    std::vector<double> t;
    ASSERT_TRUE(iCartesianSolver->invDyn(q, qdot, qdotdot, ftip, t));
    ASSERT_EQ(t.size(), 2);
    ASSERT_NEAR(t[0], 14.0, eps); //-- T = F * d = (1 kg * 10 m/s^2 * 0.5 m) + (1 kg * 10 m/s^2 * 1 m) - (1 N * 1 m) = 14 N*m
    ASSERT_NEAR(t[1], 0.0, eps); //-- T = F * d = (1 kg * 10 m/s^2 * 0 m) - (1 N * 0 m) = 0 N*m
}

TEST_F(KdlSolverTest, InvDyn5)
{
    std::vector<double> q {0.0, -90.0};
    std::vector<double> qdot {0.0, 0.0};
    std::vector<double> qdotdot {0.0, 0.0};
    std::vector<double> ftip {0.0, 1.0, 0.0, 0.0, 0.0, 0.0}; // Fx, Fy, Fz, Tx, Ty, Tz
    std::vector<double> t;
    ASSERT_TRUE(iCartesianSolver->invDyn(q, qdot, qdotdot, ftip, t));
    ASSERT_EQ(t.size(), 2);
    ASSERT_NEAR(t[0], 18.0, eps); //-- T = F * d = (1 kg * 10 m/s^2 * 0.5 m) + (1 kg * 10 m/s^2 * 1.5 m) - (1 N * 2 m) = 18 N*m
    ASSERT_NEAR(t[1], 4.0, eps); //-- T = F * d = (1 kg * 10 m/s^2 * 0.5 m) - (1 N * 1 m) = 4 N*m
}

} // namespace roboticslab::test
