#include "gtest/gtest.h"

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianSolver.h"

YARP_DECLARE_PLUGINS(TeoYarp)

class KdlSolverTest : public testing::Test
{

    public:
        virtual void SetUp() {
            iCartesianSolver = NULL;
        }

        virtual void TearDown()
        {
        }

    protected:
        yarp::dev::PolyDriver dd;
        teo::ICartesianSolver *iCartesianSolver;
};

TEST_F( KdlSolverTest, KdlSolverFwdKin)
{
    YARP_REGISTER_PLUGINS(TeoYarp);

    yarp::os::Property options;
    options.put("device","kdlsolver");
    yarp::os::Property& psub = options.addGroup("link_0");  //-- A nested Property, easier syntax from file.
    psub.put("A",1);
    dd.open(options);

    ASSERT_EQ(true, dd.isValid() );
    dd.view(iCartesianSolver);
    ASSERT_NE((teo::ICartesianSolver*)NULL, iCartesianSolver );
    std::vector<double> q(1),x,o;

    //-- Cast to int to allow e-16 deviations
    q[0]=0.0;
    iCartesianSolver->fwdKin(q,x,o);
    ASSERT_EQ(x.size(), 3 );
    ASSERT_EQ((int)x[0], 1 );
    ASSERT_EQ((int)x[1], 0 );
    ASSERT_EQ((int)x[2], 0 );

    q[0]=90.0;
    iCartesianSolver->fwdKin(q,x,o);
    ASSERT_EQ(x.size(), 3 );
    ASSERT_EQ((int)x[0], 0  );
    ASSERT_EQ((int)x[1], 1 );
    ASSERT_EQ((int)x[2], 0 );
}


