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
 * @brief Tests @ref BasicCartesianControl ikin and idyn on a simple mechanism.
 */
class BasicCartesianControlTest : public testing::Test
{
public:
    void SetUp() override
    {
        yarp::os::Property cartesianControlOptions {
            {"device", yarp::os::Value("BasicCartesianControl")},
            {"robot", yarp::os::Value("fakeMotionControl")},
            {"solver", yarp::os::Value("KdlSolver")}
        };

        auto & limits = cartesianControlOptions.addGroup("LIMITS");
        limits.put("mins", yarp::os::Value::makeList("-100.0"));
        limits.put("maxs", yarp::os::Value::makeList("100.0"));
        // default max joint velocity is hardcoded in fakeMotionControl

        auto & kinematics = cartesianControlOptions.addGroup("KINEMATICS");
        kinematics.put("numLinks", 1);
        kinematics.addGroup("link_0").put("A", yarp::os::Value(1));

        if (!cartesianControlDevice.open(cartesianControlOptions))
        {
            yError() << "Unable to open cartesian control device" << cartesianControlOptions.find("device").asString();
            return;
        }

        if (!cartesianControlDevice.view(iCartesianControl))
        {
            yError() << "Could not view iCartesianControl in" << cartesianControlOptions.find("device").asString();
            return;
        }
    }

    void TearDown() override
    {
        cartesianControlDevice.close();
    }

protected:
    static constexpr double eps = 1e-9;
    yarp::dev::PolyDriver cartesianControlDevice;
    roboticslab::ICartesianControl * iCartesianControl;
};

TEST_F(BasicCartesianControlTest, BasicCartesianControlStat)
{
    ICartesianControl::ControllerState state;
    ASSERT_TRUE(iCartesianControl->getState(state));
    ASSERT_EQ(state.mode, ICartesianControl::Mode::NONE);
    ASSERT_NEAR(state.x[0], 1, eps);
    ASSERT_NEAR(state.x[1], 0, eps);
    ASSERT_NEAR(state.x[2], 0, eps);
}

TEST_F(BasicCartesianControlTest, BasicCartesianControlInv1)
{
    std::vector<double> xd = {1, 0, 0, 0, 0, 0};
    std::vector<double> q;
    ASSERT_TRUE(iCartesianControl->solvePose(xd, q));
    ASSERT_EQ(q.size(), 1);
    ASSERT_NEAR(q[0], 0, eps);
}

TEST_F(BasicCartesianControlTest, BasicCartesianControlInv2)
{
    std::vector<double> xd = {0, 1, 0, 0, 0, M_PI / 2};
    std::vector<double> q;
    ASSERT_TRUE(iCartesianControl->solvePose(xd, q));
    ASSERT_EQ(q.size(), 1);
    ASSERT_NEAR(q[0], 90, eps);
}

TEST_F(BasicCartesianControlTest, BasicCartesianControlTool)
{
    ICartesianControl::ControllerState state;

    std::vector<double> x = {0, 0, 1, M_PI / 4, 0, 0};

    // add tool ('A')
    ASSERT_TRUE(iCartesianControl->changeTool(x));
    ASSERT_TRUE(iCartesianControl->getState(state));
    ASSERT_NEAR(state.x[0], 1, eps);
    ASSERT_NEAR(state.x[1], 0, eps);
    ASSERT_NEAR(state.x[2], 1, eps);
    ASSERT_NEAR(state.x[3], M_PI / 4, eps);
    ASSERT_NEAR(state.x[4], 0, eps);
    ASSERT_NEAR(state.x[5], 0, eps);

    // change tool ('b')
    x = {1, 0, 0, 0, M_PI / 4, 0};
    ASSERT_TRUE(iCartesianControl->changeTool(x));
    ASSERT_TRUE(iCartesianControl->getState(state));
    ASSERT_NEAR(state.x[0], 2, eps);
    ASSERT_NEAR(state.x[1], 0, eps);
    ASSERT_NEAR(state.x[2], 0, eps);
    ASSERT_NEAR(state.x[3], 0, eps);
    ASSERT_NEAR(state.x[4], M_PI / 4, eps);
    ASSERT_NEAR(state.x[5], 0, eps);

    // remove tool
    std::fill(x.begin(), x.end(), 0);
    ASSERT_TRUE(iCartesianControl->changeTool(x));
    ASSERT_TRUE(iCartesianControl->getState(state));
    ASSERT_NEAR(state.x[0], 1, eps);
    ASSERT_NEAR(state.x[1], 0, eps);
    ASSERT_NEAR(state.x[2], 0, eps);
    ASSERT_NEAR(state.x[3], 0, eps);
    ASSERT_NEAR(state.x[4], 0, eps);
    ASSERT_NEAR(state.x[5], 0, eps);
}

} // namespace roboticslab::test
