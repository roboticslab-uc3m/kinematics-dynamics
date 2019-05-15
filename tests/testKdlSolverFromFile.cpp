#include "gtest/gtest.h"

#include <string>
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
class KdlSolverTestFromFile : public testing::Test
{

    public:
        virtual void SetUp() {
            yarp::os::ResourceFinder rf;
            rf.setVerbose(false);
            rf.setDefaultContext("testKdlSolverFromFile");
            rf.setDefaultConfigFile("testKdlSolverFromFile.ini");
            std::string kinematicsFileFullPath = rf.findFileByName( "testKdlSolverFromFile.ini" );

            yarp::os::Property solverOptions;
            if (! solverOptions.fromConfigFile(kinematicsFileFullPath) ) {  //-- Put first because defaults to wiping out.
                CD_ERROR("Could not configure from \"%s\".\n",kinematicsFileFullPath.c_str());
                return;
            }
            solverOptions.put("device","KdlSolver");
            solverOptions.fromString("(mins (-70 -15 -10 -100 -90 -100)) (maxs (45 70 75 10 90 10))", false);

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

TEST_F( KdlSolverTestFromFile, KdlSolverFwdKin1)
{
    std::vector<double> q(6),x;
    q[0]=0.0;
    q[1]=0.0;
    q[2]=0.0;
    q[3]=0.0;
    q[4]=0.0;
    q[5]=0.0;
    iCartesianSolver->fwdKin(q,x);
    ASSERT_EQ(x.size(), 6);  //-- axisAngle (scaled)
    ASSERT_NEAR(x[0], 0, 1e-9);  //-- x
    ASSERT_NEAR(x[1], 0.34692, 1e-9);  //-- y
    ASSERT_NEAR(x[2], -0.221806, 1e-9);  //-- z
    //-- Not checking orientation for now
}

TEST_F( KdlSolverTestFromFile, KdlSolverFwdKin2)
{
    std::vector<double> q(6),x;
    q[0]=-90.0;
    q[1]=0.0;
    q[2]=0.0;
    q[3]=0.0;
    q[4]=0.0;
    q[5]=0.0;
    iCartesianSolver->fwdKin(q,x);
    ASSERT_EQ(x.size(), 6);  //-- axisAngle (scaled)
    ASSERT_NEAR(x[0], 0.718506, 1e-9);  //-- x
    ASSERT_NEAR(x[1], 0.34692, 1e-9);  //-- y
    ASSERT_NEAR(x[2], 0.4967, 1e-9);  //-- z
    //-- Not checking orientation for now
}

}  // namespace roboticslab

