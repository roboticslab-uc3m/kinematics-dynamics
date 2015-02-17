#include "gtest/gtest.h"

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianSolver.h"
#include "ColorDebug.hpp"

YARP_DECLARE_PLUGINS(TeoYarp)

class KdlSolverTest : public testing::Test
{

    public:
        virtual void SetUp() {
            YARP_REGISTER_PLUGINS(TeoYarp);

            yarp::os::Property options;
            options.put("device","kdlsolver");
            yarp::os::Property& psub = options.addGroup("link_0");  //-- A nested Property, easier syntax from file.
            psub.put("A",1);
            dd.open(options);
            if( ! dd.isValid() ) {
                CD_ERROR("\n");
                return;
            }
            if( ! dd.view(iCartesianSolver) ) {
                CD_ERROR("\n");
                return;
            }
        }

        virtual void TearDown()
        {
            dd.close();
        }

    protected:
        yarp::dev::PolyDriver dd;
        teo::ICartesianSolver *iCartesianSolver;
};

TEST_F( KdlSolverTest, KdlSolverFwdKin1)
{
    std::vector<double> q(1),x,o;
    q[0]=0.0;
    iCartesianSolver->fwdKin(q,x,o);
    ASSERT_EQ(x.size(), 3 );
    ASSERT_NEAR(x[0], 1, 1e-9);
    ASSERT_NEAR(x[1], 0, 1e-9);
    ASSERT_NEAR(x[2], 0, 1e-9);
}

TEST_F( KdlSolverTest, KdlSolverFwdKin2)
{
    std::vector<double> q(1),x,o;
    q[0]=90.0;
    iCartesianSolver->fwdKin(q,x,o);
    ASSERT_EQ(x.size(), 3 );
    ASSERT_NEAR(x[0], 0, 1e-9);
    ASSERT_NEAR(x[1], 1, 1e-9);
    ASSERT_NEAR(x[2], 0, 1e-9);
}

TEST_F( KdlSolverTest, KdlSolverInvKin1)
{
    std::vector<double> xd(3),od(4),qGuess(1),q;
    xd[0] = 1;
    xd[1] = 0;
    xd[2] = 0;
    qGuess[0] = 0;
    iCartesianSolver->invKin(xd,od,qGuess,q);
    ASSERT_NEAR(q[0], 0, 1e-3);
}

TEST_F( KdlSolverTest, KdlSolverInvKin2)
{
    std::vector<double> xd(3),od(4),qGuess(1),q;
    xd[0] = 0;
    xd[1] = 1;
    xd[2] = 0;
    qGuess[0] = 0;
    iCartesianSolver->invKin(xd,od,qGuess,q);
    ASSERT_NEAR(q[0], 90, 1e-3);
}
