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

            //-- Compact definition
            yarp::os::Property p("(device kdlsolver) (link_0 (A 1) (mass 1) (cog -0.5 0 0) (inertia 1 1 1))");

            //-- Commented out lengthly definition
            //yarp::os::Property p;
            //p.put("device","kdlsolver");
            //yarp::os::Property& psub = p.addGroup("link_0");  //-- A nested Property, easier syntax from file.
            //psub.put("A",1);

            dd.open(p);
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
    ASSERT_EQ(q.size(), 1 );
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
    ASSERT_EQ(q.size(), 1 );
    ASSERT_NEAR(q[0], 90, 1e-3);
}

TEST_F( KdlSolverTest, KdlSolverInvDyn1)
{
    std::vector<double> q(1),qdot(1,0.0),qdotdot(1,0.0),fext(6,0.0),t;
    q[0] = -90.0;
    std::vector< std::vector<double> > fexts;
    fexts.push_back(fext);
    iCartesianSolver->invDyn(q,qdot,qdotdot,fexts,t);
    ASSERT_EQ(t.size(), 1 );
    ASSERT_NEAR(t[0], 0, 1e-9);  //-- T = F*d = 1kg * 10m/s^2 * 0m = 0 N*m
}

TEST_F( KdlSolverTest, KdlSolverInvDyn2)
{
    std::vector<double> q(1),qdot(1,0.0),qdotdot(1,0.0),fext(6,0.0),t;
    q[0] = 0.0;
    std::vector< std::vector<double> > fexts;
    fexts.push_back(fext);
    iCartesianSolver->invDyn(q,qdot,qdotdot,fexts,t);
    ASSERT_EQ(t.size(), 1 );
    ASSERT_NEAR(t[0], 5, 1e-9);  //-- T = F*d = 1kg * 10m/s^2 * 0.5m = 5 N*m
}
