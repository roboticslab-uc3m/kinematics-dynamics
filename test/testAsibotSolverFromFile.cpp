#include "gtest/gtest.h"

#include <string>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <ColorDebug.hpp>

#include "ICartesianSolver.h"

namespace roboticslab
{

/**
 * @brief Tests AsibotSolver ikin from loaded configuration file.
 */
class AsibotSolverTestFromFile : public testing::Test
{
public:
    virtual void SetUp()
    {
        yarp::os::ResourceFinder rf;
        rf.setVerbose(false);
        rf.setDefaultContext("testAsibotSolverFromFile");
        rf.setDefaultConfigFile("testAsibotSolverFromFile.ini");

        std::string kinematicsFileFullPath = rf.findFileByName("testAsibotSolverFromFile.ini");
        yarp::os::Property solverOptions;

        if (!solverOptions.fromConfigFile(kinematicsFileFullPath))
        {
            CD_ERROR("Could not configure from \"%s\".\n", kinematicsFileFullPath.c_str());
            return;
        }

        solverOptions.put("device", "AsibotSolver");
        solverDevice.open(solverOptions);

        if (!solverDevice.isValid())
        {
            CD_ERROR("solverDevice not valid: %s.\n", solverOptions.find("device").asString().c_str());
            return;
        }

        if (!solverDevice.view(iCartesianSolver))
        {
            CD_ERROR("Could not view ICartesianSolver in %s.\n", solverOptions.find("device").asString().c_str());
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

TEST_F(AsibotSolverTestFromFile, AsibotSolverFwdKin1)
{
    std::vector<double> q(5, 0.0), x;

    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));

    ASSERT_EQ(x.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(x[0], 0.0, 1e-9);  //-- x
    ASSERT_NEAR(x[1], 0.0, 1e-9);  //-- y
    ASSERT_NEAR(x[2], 1.4, 1e-9);  //-- z
    ASSERT_NEAR(x[3], 0.0, 1e-9);  //-- oyP
    ASSERT_NEAR(x[4], 0.0, 1e-9);  //-- ozPP
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverFwdKin2)
{
    std::vector<double> q(5), x;

    q[0] = 90.0;
    q[1] = 45.0;
    q[2] = 90.0;
    q[3] = -45.0;
    q[4] = 90.0;

    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));

    ASSERT_EQ(x.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(x[0], 0.0, 1e-9);  //-- x
    ASSERT_NEAR(x[1], 0.865685425, 1e-9);  //-- y
    ASSERT_NEAR(x[2], 0.3, 1e-9);  //-- z
    ASSERT_NEAR(x[3], 90.0, 1e-9);  //-- oyP
    ASSERT_NEAR(x[4], 90.0, 1e-9);  //-- ozPP
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverInvKin1)
{
    std::vector<double> xd(5), qGuess(5, 0.0), q;

    xd[0] = 0.494974746;  //-- x
    xd[1] = 0.0;  //-- y
    xd[2] = 1.194974747;  //-- z
    xd[3] = 45.0;  //-- oyP
    xd[4] = 0.0;  //-- ozPP

    qGuess[2] = 90.0;

    ASSERT_TRUE(iCartesianSolver->invKin(xd, qGuess, q));

    ASSERT_EQ(q.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(q[0], 0.0, 1e-3);
    ASSERT_NEAR(q[1], 0.0, 1e-3);
    ASSERT_NEAR(q[2], 45.0, 1e-3);
    ASSERT_NEAR(q[3], 0.0, 1e-3);
    ASSERT_NEAR(q[4], 0.0, 1e-3);
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverSetLimits)
{
    // enable joints q1, q3, q4, q5 on all configs
    std::vector<double> qMin(5, -180.0), qMax(5, 180.0);

    // restrict movement on joint q2, force FORWARD DOWN
    qMin[1] = 45.0;
    qMax[1] = 90.0;

    ASSERT_TRUE(iCartesianSolver->setLimits(qMin, qMax));

    std::vector<double> xd(5), qGuess(5, 0.0), q;

    xd[0] = 0.494974746;  //-- x
    xd[1] = 0.0;  //-- y
    xd[2] = 1.194974747;  //-- z
    xd[3] = 45.0;  //-- oyP
    xd[4] = 0.0;  //-- ozPP

    qGuess[2] = 90.0;

    ASSERT_TRUE(iCartesianSolver->invKin(xd, qGuess, q));

    ASSERT_EQ(q.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(q[0], 0.0, 1e-3);
    ASSERT_NEAR(q[1], 45.0, 1e-3);
    ASSERT_NEAR(q[2], -45.0, 1e-3);
    ASSERT_NEAR(q[3], 45.0, 1e-3);
    ASSERT_NEAR(q[4], 0.0, 1e-3);
}

}  // namespace roboticslab

