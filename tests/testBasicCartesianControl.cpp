#include "gtest/gtest.h"

#include <cmath>
#include <vector>
#include <algorithm> // std::fill

#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"

namespace roboticslab::test
{

/**
 * @ingroup kinematics-dynamics-tests
 * @brief Tests \ref BasicCartesianControl ikin and idyn on a simple mechanism.
 */
class BasicCartesianControlTest : public testing::Test
{
public:
    void SetUp() override
    {
        yarp::os::Property cartesianControlOptions {
            {"device", yarp::os::Value("BasicCartesianControl")},
            {"robot", yarp::os::Value("fakeMotionControl")},
            {"solver", yarp::os::Value("KdlSolver")},
            {"numLinks", yarp::os::Value(1)}
        };

        cartesianControlOptions.addGroup("link_0").put("A", yarp::os::Value(1));
        cartesianControlOptions.put("mins", yarp::os::Value::makeList("-100.0"));
        cartesianControlOptions.put("maxs", yarp::os::Value::makeList("100.0"));
        cartesianControlOptions.put("maxvels", yarp::os::Value::makeList("100.0"));

        cartesianControlDevice.open(cartesianControlOptions);

        if (!cartesianControlDevice.isValid())
        {
            yError() << "CartesianControl device not valid:" << cartesianControlOptions.find("device").asString();
            return;
        }

        if (!cartesianControlDevice.view(iCartesianControl))
        {
            yError() << "Could not view iCartesianControl in:" << cartesianControlOptions.find("device").asString();
            return;
        }
    }

    void TearDown() override
    {
        cartesianControlDevice.close();
    }

protected:
    yarp::dev::PolyDriver cartesianControlDevice;
    roboticslab::ICartesianControl * iCartesianControl;
};

TEST_F(BasicCartesianControlTest, BasicCartesianControlStat)
{
    std::vector<double> x;
    int state;
    iCartesianControl->stat(x, &state);
    ASSERT_EQ(state, VOCAB_CC_NOT_CONTROLLING);
    ASSERT_NEAR(x[0], 1, 1e-9);
    ASSERT_NEAR(x[1], 0, 1e-9);
    ASSERT_NEAR(x[2], 0, 1e-9);
}

TEST_F(BasicCartesianControlTest, BasicCartesianControlInv1)
{
    std::vector<double> xd = {1, 0, 0, 0, 0, 0};
    std::vector<double> q;
    ASSERT_TRUE(iCartesianControl->inv(xd, q));
    ASSERT_EQ(q.size(), 1);
    ASSERT_NEAR(q[0], 0, 1e-3);
}

TEST_F(BasicCartesianControlTest, BasicCartesianControlInv2)
{
    std::vector<double> xd = {0, 1, 0, 0, 0, M_PI / 2};
    std::vector<double> q;
    ASSERT_TRUE(iCartesianControl->inv(xd, q));
    ASSERT_EQ(q.size(), 1);
    ASSERT_NEAR(q[0], 90, 1e-3);
}

TEST_F(BasicCartesianControlTest, BasicCartesianControlTool)
{
    std::vector<double> xToolA, xToolB, xNoTool;

    std::vector<double> x = {0, 0, 1, M_PI / 4, 0, 0};

    // add tool ('A')
    ASSERT_TRUE(iCartesianControl->tool(x));
    ASSERT_TRUE(iCartesianControl->stat(xToolA));
    ASSERT_NEAR(xToolA[0], 1, 1e-9);
    ASSERT_NEAR(xToolA[1], 0, 1e-9);
    ASSERT_NEAR(xToolA[2], 1, 1e-9);
    ASSERT_NEAR(xToolA[3], M_PI / 4, 1e-9);
    ASSERT_NEAR(xToolA[4], 0, 1e-9);
    ASSERT_NEAR(xToolA[5], 0, 1e-9);

    // change tool ('b')
    x = {1, 0, 0, 0, M_PI / 4, 0};
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

} // namespace roboticslab::test
