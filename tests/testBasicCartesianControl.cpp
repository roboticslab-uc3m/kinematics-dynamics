#include "gtest/gtest.h"

#include <cmath>
#include <vector>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <ColorDebug.h>

#include "ICartesianControl.h"

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-tests
 * @brief Tests \ref BasicCartesianControl ikin and idyn on a simple mechanism.
 */
class BasicCartesianControlTest : public testing::Test
{

    public:
        virtual void SetUp() {
            yarp::os::Property cartesianControlOptions("(device BasicCartesianControl) (robot FakeControlboard) (axes 1) (solver KdlSolver) (gravity (0 -10 0)) (numLinks 1) (link_0 (A 1) (mass 1) (cog -0.5 0 0) (inertia 1 1 1))");

            cartesianControlDevice.open(cartesianControlOptions);
            if( ! cartesianControlDevice.isValid() ) {
                CD_ERROR("CartesianControl device not valid: %s.\n",cartesianControlOptions.find("device").asString().c_str());
                return;
            }
            if( ! cartesianControlDevice.view(iCartesianControl) ) {
                CD_ERROR("Could not view iCartesianControl in: %s.\n",cartesianControlOptions.find("device").asString().c_str());
                return;
            }
            yarp::os::Time::delay(1);
        }

        virtual void TearDown()
        {
            cartesianControlDevice.close();
        }

    protected:
        yarp::dev::PolyDriver cartesianControlDevice;
        roboticslab::ICartesianControl *iCartesianControl;
};

TEST_F( BasicCartesianControlTest, BasicCartesianControlStat)
{
    std::vector<double> x;
    int state;
    iCartesianControl->stat(x,&state);
    ASSERT_EQ(state,VOCAB_CC_NOT_CONTROLLING);
    ASSERT_NEAR(x[0], 1, 1e-9);
    ASSERT_NEAR(x[1], 0, 1e-9);
    ASSERT_NEAR(x[2], 0, 1e-9);
}

TEST_F( BasicCartesianControlTest, BasicCartesianControlInv1)
{
    std::vector<double> xd(6),q;
    xd[0] = 1;  // x
    xd[1] = 0;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = 0;  // o(z)
    iCartesianControl->inv(xd,q);
    ASSERT_EQ(q.size(), 1 );
    ASSERT_NEAR(q[0], 0, 1e-3);
}

TEST_F( BasicCartesianControlTest, BasicCartesianControlInv2)
{
    std::vector<double> xd(6),q;
    xd[0] = 0;  // x
    xd[1] = 1;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = M_PI / 2;  // o(z)
    iCartesianControl->inv(xd,q);
    ASSERT_EQ(q.size(), 1 );
    ASSERT_NEAR(q[0], 90, 1e-3);
}

TEST_F( BasicCartesianControlTest, BasicCartesianControlTool)
{
    std::vector<double> x(6),xToolA,xToolB,xNoTool;

    // add tool ('A')
    x[0] = 0;  // x
    x[1] = 0;  // y
    x[2] = 1;  // z
    x[3] = M_PI / 4;  // o(x)
    x[4] = 0;  // o(y)
    x[5] = 0;  // o(z)
    ASSERT_TRUE(iCartesianControl->tool(x));
    ASSERT_TRUE(iCartesianControl->stat(xToolA));
    ASSERT_NEAR(xToolA[0], 1, 1e-9);
    ASSERT_NEAR(xToolA[1], 0, 1e-9);
    ASSERT_NEAR(xToolA[2], 1, 1e-9);
    ASSERT_NEAR(xToolA[3], M_PI / 4, 1e-9);
    ASSERT_NEAR(xToolA[4], 0, 1e-9);
    ASSERT_NEAR(xToolA[5], 0, 1e-9);

    // change tool ('b')
    std::fill(x.begin(), x.end(), 0);
    x[0] = 1;
    x[4] = M_PI / 4;
    ASSERT_TRUE(iCartesianControl->tool(x));
    ASSERT_TRUE(iCartesianControl->stat(xToolB));
    ASSERT_NEAR(xToolB[0], 2, 1e-9);
    ASSERT_NEAR(xToolB[1], 0, 1e-9);
    ASSERT_NEAR(xToolB[2], 0, 1e-9);
    ASSERT_NEAR(xToolB[3], 0, 1e-9);
    ASSERT_NEAR(xToolB[4], M_PI / 4, 1e-9);
    ASSERT_NEAR(xToolB[5], 0, 1e-9);

    // remove tool
    std::fill(x.begin(), x.end(), 0);
    ASSERT_TRUE(iCartesianControl->tool(x));
    ASSERT_TRUE(iCartesianControl->stat(xNoTool));
    ASSERT_NEAR(xNoTool[0], 1, 1e-9);
    ASSERT_NEAR(xNoTool[1], 0, 1e-9);
    ASSERT_NEAR(xNoTool[2], 0, 1e-9);
    ASSERT_NEAR(xNoTool[3], 0, 1e-9);
    ASSERT_NEAR(xNoTool[4], 0, 1e-9);
    ASSERT_NEAR(xNoTool[5], 0, 1e-9);
}

}  // namespace roboticslab
