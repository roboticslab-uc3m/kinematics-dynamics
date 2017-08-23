#include "gtest/gtest.h"

#include <vector>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <ColorDebug.hpp>

#include "ICartesianControl.h"

namespace roboticslab
{

/**
 * @brief Tests \ref BasicCartesianControl ikin and idyn on a simple mechanism.
 */
class BasicCartesianControlTest : public testing::Test
{

    public:
        virtual void SetUp() {
            yarp::os::Property cartesianControlOptions("(device BasicCartesianControl) (robot FakeControlboard) (axes 1) (solver KdlSolver) (angleRepr axisAngle) (gravity 0 -10 0) (numLinks 1) (link_0 (A 1) (mass 1) (cog -0.5 0 0) (inertia 1 1 1))");

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
    iCartesianControl->stat(state,x);
    ASSERT_EQ(state,VOCAB_CC_NOT_CONTROLLING);
    ASSERT_NEAR(x[0], 1, 1e-9);
    ASSERT_NEAR(x[1], 0, 1e-9);
    ASSERT_NEAR(x[2], 0, 1e-9);
}

TEST_F( BasicCartesianControlTest, BasicCartesianControlInv1)
{
    std::vector<double> xd(7),q;
    xd[0] = 1;  // x
    xd[1] = 0;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = 1;  // o(z)
    xd[6] = 0;  // o(angle)
    iCartesianControl->inv(xd,q);
    ASSERT_EQ(q.size(), 1 );
    ASSERT_NEAR(q[0], 0, 1e-3);
}

TEST_F( BasicCartesianControlTest, BasicCartesianControlInv2)
{
    std::vector<double> xd(7),q;
    xd[0] = 0;  // x
    xd[1] = 1;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = 1;  // o(z)
    xd[6] = 90;  // o(angle)
    iCartesianControl->inv(xd,q);
    ASSERT_EQ(q.size(), 1 );
    ASSERT_NEAR(q[0], 90, 1e-3);
}

TEST_F( BasicCartesianControlTest, BasicCartesianControlTool)
{
    std::vector<double> x(7),xToolA,xToolB,xNoTool;
    int state;

    // add tool ('A')
    x[0] = 0;  // x
    x[1] = 0;  // y
    x[2] = 1;  // z
    x[3] = 1;  // o(x)
    x[4] = 0;  // o(y)
    x[5] = 0;  // o(z)
    x[6] = 90;  // o(angle)
    ASSERT_TRUE(iCartesianControl->tool(x));
    ASSERT_TRUE(iCartesianControl->stat(state, xToolA));
    ASSERT_NEAR(xToolA[0], 1, 1e-9);
    ASSERT_NEAR(xToolA[1], 0, 1e-9);
    ASSERT_NEAR(xToolA[2], 1, 1e-9);
    ASSERT_NEAR(xToolA[3], 1, 1e-9);
    ASSERT_NEAR(xToolA[4], 0, 1e-9);
    ASSERT_NEAR(xToolA[5], 0, 1e-9);
    ASSERT_NEAR(xToolA[6], 90, 1e-9);

    // change tool ('b')
    std::fill(x.begin(), x.end(), 0);
    x[0] = 1;
    x[4] = 1;
    x[6] = 90;
    ASSERT_TRUE(iCartesianControl->tool(x));
    ASSERT_TRUE(iCartesianControl->stat(state, xToolB));
    ASSERT_NEAR(xToolB[0], 2, 1e-9);
    ASSERT_NEAR(xToolB[1], 0, 1e-9);
    ASSERT_NEAR(xToolB[2], 0, 1e-9);
    ASSERT_NEAR(xToolB[3], 0, 1e-9);
    ASSERT_NEAR(xToolB[4], 1, 1e-9);
    ASSERT_NEAR(xToolB[5], 0, 1e-9);
    ASSERT_NEAR(xToolB[6], 90, 1e-9);

    // remote tool
    std::fill(x.begin(), x.end(), 0);
    CD_DEBUG("%f %f %f %f %f %f %f\n", x[0], x[1], x[2], x[3], x[4], x[5], x[6]);
    ASSERT_TRUE(iCartesianControl->tool(x));
    ASSERT_TRUE(iCartesianControl->stat(state, xNoTool));
    ASSERT_NEAR(xNoTool[0], 1, 1e-9);
    ASSERT_NEAR(xNoTool[1], 0, 1e-9);
    ASSERT_NEAR(xNoTool[2], 0, 1e-9);
    // we don't care about the axis, because it's implementation-dependent;
    // the rotation angle will be equal to 0º anyway
    ASSERT_NEAR(xNoTool[6], 0, 1e-9);
}

}  // namespace roboticslab

