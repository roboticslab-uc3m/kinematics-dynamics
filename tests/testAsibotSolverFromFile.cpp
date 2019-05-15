#include "gtest/gtest.h"

#include <string>
#include <vector>

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <ColorDebug.h>

#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-tests
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
        solverOptions.fromString("(mins (-180.0 -135.0 -135.0 -135.0 -180.0)) (maxs ( 180.0  135.0  135.0  135.0  180.0))", false);
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

    static const double EPS_CART;  //-- cartesian space
    static const double EPS_JOINT;  //-- joint space
};

const double AsibotSolverTestFromFile::EPS_CART = 1e-6;
const double AsibotSolverTestFromFile::EPS_JOINT = 1e-3;

TEST_F(AsibotSolverTestFromFile, AsibotSolverAppendLink)
{
    std::vector<double> tool1(6), tool2(6), q(6), x;

    tool1[0] = 0.05;
    tool1[1] = 0.2;
    tool1[2] = 0.1;
    tool1[5] = 1.5707963267948966192313216916398;  //-- pi/2

    tool2[2] = 0.15;

    ASSERT_TRUE(iCartesianSolver->appendLink(tool1));

    q[2] = 90.0;

    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));

    ASSERT_TRUE(KinRepresentation::decodePose(x, x, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(x[0], 0.8, EPS_CART);  //-- x
    ASSERT_NEAR(x[1], 0.2, EPS_CART);  //-- y
    ASSERT_NEAR(x[2], 0.65, EPS_CART);  //-- z
    ASSERT_NEAR(x[3], 90.0, EPS_CART);  //-- oyP
    ASSERT_NEAR(x[4], 90.0, EPS_CART);  //-- ozPP

    ASSERT_TRUE(iCartesianSolver->appendLink(tool2));

    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));

    ASSERT_TRUE(KinRepresentation::decodePose(x, x, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(x[0], 0.95, EPS_CART);  //-- x
    ASSERT_NEAR(x[1], 0.2, EPS_CART);  //-- y
    ASSERT_NEAR(x[2], 0.65, EPS_CART);  //-- z
    ASSERT_NEAR(x[3], 90.0, EPS_CART);  //-- oyP
    ASSERT_NEAR(x[4], 90.0, EPS_CART);  //-- ozPP
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverRestoreOriginalChain)
{
    std::vector<double> tool(6), q(6), x;

    tool[0] = 0.05;
    tool[1] = 0.2;
    tool[2] = 0.1;
    tool[5] = 1.5707963267948966192313216916398;  //-- pi/2

    ASSERT_TRUE(iCartesianSolver->appendLink(tool));

    q[2] = 90.0;

    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));

    ASSERT_TRUE(KinRepresentation::decodePose(x, x, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(x[0], 0.8, EPS_CART);  //-- x
    ASSERT_NEAR(x[1], 0.2, EPS_CART);  //-- y
    ASSERT_NEAR(x[2], 0.65, EPS_CART);  //-- z
    ASSERT_NEAR(x[3], 90.0, EPS_CART);  //-- oyP
    ASSERT_NEAR(x[4], 90.0, EPS_CART);  //-- ozPP

    ASSERT_TRUE(iCartesianSolver->restoreOriginalChain());

    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));

    ASSERT_TRUE(KinRepresentation::decodePose(x, x, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(x[0], 0.7, EPS_CART);  //-- x
    ASSERT_NEAR(x[1], 0.0, EPS_CART);  //-- y
    ASSERT_NEAR(x[2], 0.7, EPS_CART);  //-- z
    ASSERT_NEAR(x[3], 90.0, EPS_CART);  //-- oyP
    ASSERT_NEAR(x[4], 0.0, EPS_CART);  //-- ozPP
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverFwdKin1)
{
    std::vector<double> q(5, 0.0), x;

    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));

    ASSERT_TRUE(KinRepresentation::decodePose(x, x, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(x[0], 0.0, EPS_CART);  //-- x
    ASSERT_NEAR(x[1], 0.0, EPS_CART);  //-- y
    ASSERT_NEAR(x[2], 1.4, EPS_CART);  //-- z
    ASSERT_NEAR(x[3], 0.0, EPS_CART);  //-- oyP
    ASSERT_NEAR(x[4], 0.0, EPS_CART);  //-- ozPP
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

    ASSERT_TRUE(KinRepresentation::decodePose(x, x, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(x[0], 0.0, EPS_CART);  //-- x
    ASSERT_NEAR(x[1], 0.865685425, EPS_CART);  //-- y
    ASSERT_NEAR(x[2], 0.3, EPS_CART);  //-- z
    ASSERT_NEAR(x[3], 90.0, EPS_CART);  //-- oyP
    ASSERT_NEAR(x[4], 90.0, EPS_CART);  //-- ozPP
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverFwdKinTool)
{
    std::vector<double> q(5), tool(6), x;

    q[0] = 90.0;
    q[1] = 45.0;
    q[2] = 90.0;
    q[3] = -45.0;
    q[4] = 90.0;

    tool[2] = 0.1;  //-- z

    ASSERT_TRUE(iCartesianSolver->appendLink(tool));

    ASSERT_TRUE(iCartesianSolver->fwdKin(q, x));

    ASSERT_TRUE(KinRepresentation::decodePose(x, x, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x.size(), 5);  //-- eulerYZ

    ASSERT_NEAR(x[0], 0.0, EPS_CART);  //-- x
    ASSERT_NEAR(x[1], 0.965685425, EPS_CART);  //-- y
    ASSERT_NEAR(x[2], 0.3, EPS_CART);  //-- z
    ASSERT_NEAR(x[3], 90.0, EPS_CART);  //-- oyP
    ASSERT_NEAR(x[4], 90.0, EPS_CART);  //-- ozPP
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverPoseDiff)
{
    std::vector<double> xd(6), xc(6), x;

    xd[0] = 0.5;
    xd[1] = 0.6;
    xd[2] = 0.7;
    xd[3] = 0.2;
    xd[4] = 0.0;
    xd[5] = 0.0;

    xc[0] = 0.1;
    xc[1] = 0.2;
    xc[2] = 0.3;
    xc[3] = 0.1;
    xc[4] = 0.0;
    xc[5] = 0.0;

    ASSERT_TRUE(iCartesianSolver->poseDiff(xd, xc, x));

    ASSERT_EQ(x.size(), 6);  //-- twist

    ASSERT_NEAR(x[0], 0.4, EPS_CART);  //-- x
    ASSERT_NEAR(x[1], 0.4, EPS_CART);  //-- y
    ASSERT_NEAR(x[2], 0.4, EPS_CART);  //-- z
    ASSERT_NEAR(x[3], 0.1, EPS_CART);  //-- rot_x
    ASSERT_NEAR(x[4], 0.0, EPS_CART);  //-- rot_y
    ASSERT_NEAR(x[5], 0.0, EPS_CART);  //-- rot_z
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverInvKin1)
{
    std::vector<double> xd(5), qGuess(5, 0.0), q;

    xd[0] = 0.494974746;  //-- x
    xd[1] = 0.0;  //-- y
    xd[2] = 1.194974747;  //-- z
    xd[3] = 45.0;  //-- oyP
    xd[4] = 0.0;  //-- ozPP

    // forces FORWARD UP, compare with AsibotSolverSetLimits
    qGuess[2] = 90.0;

    ASSERT_TRUE(KinRepresentation::encodePose(xd, xd, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_TRUE(iCartesianSolver->invKin(xd, qGuess, q));

    ASSERT_EQ(q.size(), 5);  //-- NUM_MOTORS

    ASSERT_NEAR(q[0], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[1], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[2], 45.0, EPS_JOINT);
    ASSERT_NEAR(q[3], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[4], 0.0, EPS_JOINT);
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverInvKin2)
{
    std::vector<double> xd(5), qGuess(5, 0.0), q;

    xd[0] = -0.777817459;  //-- x
    xd[1] = 0.0;  //-- y
    xd[2] = 1.077817459;  //-- z
    xd[3] = 45.0;  //-- oyP
    xd[4] = 180.0;  //-- ozPP

    // force REVERSED
    qGuess[1] = 45.0;

    ASSERT_TRUE(KinRepresentation::encodePose(xd, xd, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_TRUE(iCartesianSolver->invKin(xd, qGuess, q));

    ASSERT_EQ(q.size(), 5);  //-- NUM_MOTORS

    // increasing eps at q2-4
    ASSERT_NEAR(q[0], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[1], -45.0, EPS_JOINT * 10);
    ASSERT_NEAR(q[2], 0.0, EPS_JOINT * 10);
    ASSERT_NEAR(q[3], 0.0, EPS_JOINT * 10);
    ASSERT_NEAR(q[4], 0.0, EPS_JOINT);
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverInvKin3)
{
    std::vector<double> xd(5), xd_encoded, qGuess(5, 0.0), q;

    xd[0] = -0.318163634;  //-- x
    xd[1] = 0.379172654;  //-- y
    xd[2] = 1.194974747;  //-- z
    xd[3] = 45.0;  //-- oyP
    xd[4] = 0.0;  //-- ozPP

    // force FORWARD
    qGuess[1] = 45.0;

    ASSERT_TRUE(KinRepresentation::encodePose(xd, xd_encoded, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_TRUE(iCartesianSolver->invKin(xd_encoded, qGuess, q));

    ASSERT_EQ(q.size(), 5);  //-- NUM_MOTORS

    // selects elbow-down on LeastOverallAngularDisplacement strategy
    ASSERT_NEAR(q[0], 130.0, EPS_JOINT);
    ASSERT_NEAR(q[1], 45.0, EPS_JOINT);
    ASSERT_NEAR(q[2], -45.0, EPS_JOINT);
    ASSERT_NEAR(q[3], 45.0, EPS_JOINT);
    ASSERT_NEAR(q[4], 0.0, EPS_JOINT);

    // force REVERSED
    xd[4] = 180.0;

    ASSERT_TRUE(KinRepresentation::encodePose(xd, xd_encoded, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_TRUE(iCartesianSolver->invKin(xd_encoded, qGuess, q));

    ASSERT_NEAR(q[0], -50.0, EPS_JOINT);
    ASSERT_NEAR(q[1], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[2], -45.0, EPS_JOINT);
    ASSERT_NEAR(q[3], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[4], 0.0, EPS_JOINT);
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverInvKin4)
{
    std::vector<double> xd(5), xd_encoded, qGuess(5, 0.0), q;

    xd[0] = -0.379172654;  //-- x
    xd[1] = 0.318163634;  //-- y
    xd[2] = 1.194974747;  //-- z
    xd[3] = 45.0;  //-- oyP
    xd[4] = 180.0;  //-- ozPP

    // force REVERSED
    qGuess[1] = 45.0;

    ASSERT_TRUE(KinRepresentation::encodePose(xd, xd_encoded, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_TRUE(iCartesianSolver->invKin(xd_encoded, qGuess, q));

    ASSERT_EQ(q.size(), 5);  //-- NUM_MOTORS

    ASSERT_NEAR(q[0], -40.0, EPS_JOINT);
    ASSERT_NEAR(q[1], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[2], -45.0, EPS_JOINT);
    ASSERT_NEAR(q[3], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[4], 0.0, EPS_JOINT);

    // force FORWARD
    xd[4] = 0.0;

    ASSERT_TRUE(KinRepresentation::encodePose(xd, xd_encoded, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_TRUE(iCartesianSolver->invKin(xd_encoded, qGuess, q));

    // selects elbow-down on LeastOverallAngularDisplacement strategy
    ASSERT_NEAR(q[0], 140.0, EPS_JOINT);
    ASSERT_NEAR(q[1], 45.0, EPS_JOINT);
    ASSERT_NEAR(q[2], -45.0, EPS_JOINT);
    ASSERT_NEAR(q[3], 45.0, EPS_JOINT);
    ASSERT_NEAR(q[4], 0.0, EPS_JOINT);
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverInvKin5)
{
    std::vector<double> xd(5), xd_encoded, qGuess(5, 0.0), q;

    xd[0] = -0.4;  //-- x
    xd[1] = 0.0;  //-- y
    xd[2] = 1.1;  //-- z
    xd[3] = 26.56505118;  //-- oyP
    xd[4] = 180.0;  //-- ozPP

    // force REVERSED UP
    qGuess[1] = 15.0;
    qGuess[2] = 30.0;

    ASSERT_TRUE(KinRepresentation::encodePose(xd, xd_encoded, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_TRUE(iCartesianSolver->invKin(xd_encoded, qGuess, q));

    ASSERT_EQ(q.size(), 5);  //-- NUM_MOTORS

    ASSERT_NEAR(q[0], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[1], 15.44443859, EPS_JOINT);
    ASSERT_NEAR(q[2], -84.01897954, EPS_JOINT);
    ASSERT_NEAR(q[3], 42.00948977, EPS_JOINT);
    ASSERT_NEAR(q[4], 0.0, EPS_JOINT);

    // force REVERSED DOWN
    qGuess[1] = -30.0;
    qGuess[2] = 90.0;

    ASSERT_TRUE(iCartesianSolver->invKin(xd_encoded, qGuess, q));

    ASSERT_NEAR(q[0], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[1], -68.57454095, EPS_JOINT);
    ASSERT_NEAR(q[2], 84.01897954, EPS_JOINT);
    ASSERT_NEAR(q[3], -42.00948977, EPS_JOINT);
    ASSERT_NEAR(q[4], 0.0, EPS_JOINT);
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverInvKinTool)
{
    std::vector<double> xd(5), qGuess(5, 0.0), tool(6), q;

    xd[0] = 0.565685425;  //-- x
    xd[1] = 0.0;  //-- y
    xd[2] = 1.265685425;  //-- z
    xd[3] = 45.0;  //-- oyP
    xd[4] = 0.0;  //-- ozPP

    // forces FORWARD UP, compare with AsibotSolverSetLimits
    qGuess[2] = 90.0;

    tool[2] = 0.1;  //-- z

    ASSERT_TRUE(iCartesianSolver->appendLink(tool));

    ASSERT_TRUE(KinRepresentation::encodePose(xd, xd, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_TRUE(iCartesianSolver->invKin(xd, qGuess, q));

    ASSERT_EQ(q.size(), 5);  //-- NUM_MOTORS

    ASSERT_NEAR(q[0], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[1], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[2], 45.0, EPS_JOINT);
    ASSERT_NEAR(q[3], 0.0, EPS_JOINT);
    ASSERT_NEAR(q[4], 0.0, EPS_JOINT);
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverDiffInvKin)
{
    std::vector<double> q(5, 0.0), xdot(6, 0.0), qdot;

    q[1] = -45.0;
    q[2] = 90.0;
    q[3] = 45.0;

    xdot[2] = 0.005;  //- m/step
    xdot[3] = 0.017453292;  //-- 1º/step

    ASSERT_TRUE(iCartesianSolver->diffInvKin(q, xdot, qdot));

    ASSERT_EQ(qdot.size(), 5);  //-- NUM_MOTORS

    // increasing eps at q2-4
    ASSERT_NEAR(qdot[0], 0.0, EPS_JOINT);
    ASSERT_NEAR(qdot[1], 0.50869278, EPS_JOINT * 10);
    ASSERT_NEAR(qdot[2], -1.017385551, EPS_JOINT * 10);
    ASSERT_NEAR(qdot[3], 0.50869278, EPS_JOINT * 10);
    ASSERT_NEAR(qdot[4], 1.0, EPS_JOINT);
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverDiffInvKinEE)
{
    std::vector<double> q(5, 0.0), xdotee(6, 0.0), qdot;

    q[0] = 45.0;
    q[1] = -45.0;
    q[2] = 90.0;
    q[3] = 45.0;

    xdotee[0] = -0.005;  //- m/step
    xdotee[5] = 0.017453292;  //-- 1º/step

    ASSERT_TRUE(iCartesianSolver->diffInvKin(q, xdotee, qdot, ICartesianSolver::TCP_FRAME));

    ASSERT_EQ(qdot.size(), 5);  //-- NUM_MOTORS

    // increasing eps at q2-4
    ASSERT_NEAR(qdot[0], 0.0, EPS_JOINT);
    ASSERT_NEAR(qdot[1], 0.50869278, EPS_JOINT * 10);
    ASSERT_NEAR(qdot[2], -1.017385551, EPS_JOINT * 10);
    ASSERT_NEAR(qdot[3], 0.50869278, EPS_JOINT * 10);
    ASSERT_NEAR(qdot[4], 1.0, EPS_JOINT);
}

TEST_F(AsibotSolverTestFromFile, AsibotSolverDiffInvKinTool)
{
    std::vector<double> q(5, 0.0), xdot(6, 0.0), tool(6, 0.1), qdot;

    q[1] = -45.0;
    q[2] = 90.0;
    q[3] = 45.0;

    xdot[2] = 0.005;  //- m/step

    // no rotation in 'xdot', so 'tool' may take whatever value
    ASSERT_TRUE(iCartesianSolver->appendLink(tool));

    ASSERT_TRUE(iCartesianSolver->diffInvKin(q, xdot, qdot));

    ASSERT_EQ(qdot.size(), 5);  //-- NUM_MOTORS

    // increasing eps at q2-4
    ASSERT_NEAR(qdot[0], 0.0, EPS_JOINT);
    ASSERT_NEAR(qdot[1], 0.50869278, EPS_JOINT * 10);
    ASSERT_NEAR(qdot[2], -1.017385551, EPS_JOINT * 10);
    ASSERT_NEAR(qdot[3], 0.50869278, EPS_JOINT * 10);
    ASSERT_NEAR(qdot[4], 0.0, EPS_JOINT);
}

}  // namespace roboticslab

