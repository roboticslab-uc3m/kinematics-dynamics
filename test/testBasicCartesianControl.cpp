#include "gtest/gtest.h"

#include <yarp/os/all.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include "ICartesianControl.h"

#include "ColorDebug.hpp"

YARP_DECLARE_PLUGINS(TeoYarp)

namespace teo
{

/**
 * @brief Tests \ref BasicCartesianControl ikin and idyn on a simple mechanism.
 */
class BasicCartesianControlTest : public testing::Test
{

    public:
        virtual void SetUp() {
            YARP_REGISTER_PLUGINS(TeoYarp);

            yarp::os::Property p("(device BasicCartesianControl) (robot FakeControlboard) (axes 1) (solver KdlSolver) (angleRepr axisAngle) (gravity 0 -10 0) (numLinks 1) (link_0 (A 1) (mass 1) (cog -0.5 0 0) (inertia 1 1 1))");

            dd.open(p);
            if( ! dd.isValid() ) {
                CD_ERROR("BasicCartesianControl device not valid.\n");
                return;
            }
            if( ! dd.view(iCartesianControl) ) {
                CD_ERROR("Could not view ICartesianControl.\n");
                return;
            }
            yarp::os::Time::delay(1);
        }

        virtual void TearDown()
        {
            dd.close();
        }

    protected:
        yarp::dev::PolyDriver dd;
        teo::ICartesianControl *iCartesianControl;
};

TEST_F( BasicCartesianControlTest, BasicCartesianControlStat)
{
    std::vector<double> x;
    iCartesianControl->stat(x);
    ASSERT_NEAR(x[0], 1, 1e-9);
    ASSERT_NEAR(x[1], 0, 1e-9);
    ASSERT_NEAR(x[2], 0, 1e-9);
}

}  // namespace teo

