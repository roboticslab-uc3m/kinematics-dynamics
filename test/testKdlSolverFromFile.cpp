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
class KdlSolverTestFromFile : public testing::Test
{

    public:
        virtual void SetUp() {
            //YARP_REGISTER_PLUGINS(TeoYarp);

            yarp::os::ResourceFinder rf;
            rf.setVerbose(false);
            //-- Next two lines could use an .ini from any valid program (teoSim, teoGravityCompensator...)
            rf.setDefaultContext("teoSim");
            rf.setDefaultConfigFile("teoSim.ini");
            std::string kinematicsFileFullPath = rf.findFileByName( "../kinematics/leftArmKinematics.ini" );

            yarp::os::Property solverOptions;
            if (! solverOptions.fromConfigFile(kinematicsFileFullPath) ) {  //-- Put first because defaults to wiping out.
                CD_ERROR("Could not configure from \"%s\".\n",kinematicsFileFullPath.c_str());
                return;
            }
            solverOptions.put("device","KdlSolver");
            solverOptions.put("angleRepr","axisAngle");

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
    ASSERT_EQ(x.size(), 7);  //-- axisAngle
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
    ASSERT_EQ(x.size(), 7);  //-- axisAngle
    ASSERT_NEAR(x[0], 0.718506, 1e-9);  //-- x
    ASSERT_NEAR(x[1], 0.34692, 1e-9);  //-- y
    ASSERT_NEAR(x[2], 0.4967, 1e-9);  //-- z
    //-- Not checking orientation for now
}

}  // namespace teo

