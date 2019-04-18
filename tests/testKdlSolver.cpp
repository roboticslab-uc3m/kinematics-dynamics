#include "gtest/gtest.h"

#include <cmath>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <ColorDebug.h>

#include "ICartesianSolver.h"

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-tests
 * @brief Tests \ref KdlSolver ikin and idyn on a simple mechanism.
 */
class KdlSolverTest : public testing::Test
{

    public:
        virtual void SetUp() {
            yarp::os::Property solverOptions("(device KdlSolver) (gravity (0 -10 0)) (numLinks 1) (link_0 (A 1) (mass 1) (cog -0.5 0 0) (inertia 1 1 1)) (mins (-180)) (maxs (180))");

            solverDevice.open(solverOptions);
            if( ! solverDevice.isValid() ) {
                CD_ERROR("solverDevice not valid: %s.\n",solverOptions.find("device").asString().c_str());
                return;
            }
            if( ! solverDevice.view(iCartesianSolver) ) {
                CD_ERROR("Could not view ICartesianSolver in %s.\n",solverOptions.find("device").asString().c_str());
                return;
            }
        }

        virtual void TearDown()
        {
            solverDevice.close();
        }

    protected:
        yarp::dev::PolyDriver solverDevice;
        roboticslab::ICartesianSolver *iCartesianSolver;
};

TEST_F( KdlSolverTest, KdlSolverFwdKin1)
{
    std::vector<double> q(1),x;
    q[0]=0.0;
    iCartesianSolver->fwdKin(q,x);
    ASSERT_NEAR(x[0], 1, 1e-9);
    ASSERT_NEAR(x[1], 0, 1e-9);
    ASSERT_NEAR(x[2], 0, 1e-9);
}

TEST_F( KdlSolverTest, KdlSolverFwdKin2)
{
    std::vector<double> q(1),x;
    q[0]=90.0;
    iCartesianSolver->fwdKin(q,x);
    ASSERT_NEAR(x[0], 0, 1e-9);
    ASSERT_NEAR(x[1], 1, 1e-9);
    ASSERT_NEAR(x[2], 0, 1e-9);
}

TEST_F( KdlSolverTest, KdlSolverInvKin1)
{
    std::vector<double> xd(6),qGuess(1),q;
    xd[0] = 1;  // x
    xd[1] = 0;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = 0;  // o(z)
    qGuess[0] = 0;
    iCartesianSolver->invKin(xd,qGuess,q);
    ASSERT_EQ(q.size(), 1 );
    ASSERT_NEAR(q[0], 0, 1e-3);
}

TEST_F( KdlSolverTest, KdlSolverInvKin2)
{
    std::vector<double> xd(6),qGuess(1),q;
    xd[0] = 0;  // x
    xd[1] = 1;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = M_PI / 2;  // o(z)
    qGuess[0] = 90;
    iCartesianSolver->invKin(xd,qGuess,q);
    ASSERT_EQ(q.size(), 1 );
    ASSERT_NEAR(q[0], 90, 1e-3);
}

TEST_F( KdlSolverTest, KdlSolverInvDyn1)
{
    std::vector<double> q(1),t;
    q[0] = -90.0;
    iCartesianSolver->invDyn(q,t);
    ASSERT_EQ(t.size(), 1 );
    ASSERT_NEAR(t[0], 0, 1e-9);  //-- T = F*d = 1kg * 10m/s^2 * 0m = 0 N*m
}

TEST_F( KdlSolverTest, KdlSolverInvDyn2)
{
    std::vector<double> q(1),t;
    q[0] = 0.0;
    iCartesianSolver->invDyn(q,t);
    ASSERT_EQ(t.size(), 1 );
    ASSERT_NEAR(t[0], 5, 1e-9);  //-- T = F*d = 1kg * 10m/s^2 * 0.5m = 5 N*m
}

TEST_F( KdlSolverTest, KdlSolverInvDyn3)
{
    std::vector<double> q(1),qdot(1,0.0),qdotdot(1,0.0),fext(6,0.0),t;
    q[0] = 0.0;
    std::vector< std::vector<double> > fexts;
    fexts.push_back(fext);
    iCartesianSolver->invDyn(q,qdot,qdotdot,fexts,t);
    ASSERT_EQ(t.size(), 1 );
    ASSERT_NEAR(t[0], 5, 1e-9);  //-- T = F*d = 1kg * 10m/s^2 * 0.5m = 5 N*m
}

}  // namespace roboticslab

