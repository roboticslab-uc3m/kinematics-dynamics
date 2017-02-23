#include "gtest/gtest.h"

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianSolver.h"
#include "ColorDebug.hpp"

//YARP_DECLARE_PLUGINS(TeoYarp)

namespace teo
{

/**
 * @brief Tests \ref KdlSolver ikin and idyn on a simple mechanism.
 */
class KdlSolverTest : public testing::Test
{

    public:
        virtual void SetUp() {
            //YARP_REGISTER_PLUGINS(TeoYarp);

            yarp::os::Property solverOptions("(device KdlSolver) (angleRepr axisAngle) (gravity 0 -10 0) (numLinks 1) (link_0 (A 1) (mass 1) (cog -0.5 0 0) (inertia 1 1 1))");

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
        teo::ICartesianSolver *iCartesianSolver;
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
    std::vector<double> xd(7),qGuess(1),q;
    xd[0] = 1;  // x
    xd[1] = 0;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = 1;  // o(z)
    xd[6] = 0;  // o(angle)
    qGuess[0] = 0;
    iCartesianSolver->invKin(xd,qGuess,q);
    ASSERT_EQ(q.size(), 1 );
    ASSERT_NEAR(q[0], 0, 1e-3);
}

TEST_F( KdlSolverTest, KdlSolverInvKin2)
{
    std::vector<double> xd(7),qGuess(1),q;
    xd[0] = 0;  // x
    xd[1] = 1;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = 1;  // o(z)
    xd[6] = 90;  // o(angle)
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

TEST_F( KdlSolverTest, KdlSolverSetLimits)
{
    std::vector<double> qMin(1), qMax(1);
    qMin[0] = 340.0;
    qMax[0] = 380.0;
    iCartesianSolver->setLimits(qMin,qMax);
    std::vector<double> xd(7),qGuess(1),q;
    xd[0] = 1;  // x
    xd[1] = 0;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = 1;  // o(z)
    xd[6] = 0;  // o(angle)
    qGuess[0] = 350;
    iCartesianSolver->invKin(xd,qGuess,q);
    ASSERT_EQ(q.size(), 1 );
    ASSERT_NEAR(q[0], 360, 1e-3);
    //--Restore default limits
    qMin[0] = -180.0;
    qMax[0] = 180.0;
    iCartesianSolver->setLimits(qMin,qMax);
}

}  // namespace teo

