#include "gtest/gtest.h"

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"

#include "ColorDebug.hpp"

//YARP_DECLARE_PLUGINS(TeoYarp)

namespace teo
{

/**
 * @brief Tests \ref BasicCartesianControl ikin and idyn on a simple mechanism.
 */
class BasicCartesianControlTest : public testing::Test
{

    public:
        virtual void SetUp() {
            //YARP_REGISTER_PLUGINS(TeoYarp);

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
        teo::ICartesianControl *iCartesianControl;
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

}  // namespace teo

