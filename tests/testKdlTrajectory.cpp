#include "gtest/gtest.h"

#include <cmath>
#include <vector>

#include "KdlTrajectory.hpp"

namespace roboticslab
{

/**
 * @brief Tests \ref KdlTrajectory.
 */
class KdlTrajectoryTest : public testing::Test
{
public:
    virtual void SetUp()
    {
        iCartesianTrajectory = new KdlTrajectory;
    }

    virtual void TearDown()
    {
        delete iCartesianTrajectory;
        iCartesianTrajectory = 0;
    }

protected:
    ICartesianTrajectory* iCartesianTrajectory;
};

TEST_F(KdlTrajectoryTest, KdlTrajectoryLine)
{
    //-- Create line trajectory
    ASSERT_TRUE( iCartesianTrajectory->setDuration(20.0) );  // Must be high, or may be expanded due to defaults
    std::vector<double> x(6);
    x[0] = 0;  // x
    x[1] = 0;  // y
    x[2] = 0;  // z
    x[3] = 0;  // o(x)
    x[4] = 0;  // o(y)
    x[5] = 0;  // o(z)
    std::vector<double> xd(6);
    xd[0] = 1;  // x
    xd[1] = 0;  // y
    xd[2] = 0;  // z
    xd[3] = 0;  // o(x)
    xd[4] = 0;  // o(y)
    xd[5] = 0;  // o(z)
    ASSERT_TRUE( iCartesianTrajectory->addWaypoint(x) );
    ASSERT_TRUE( iCartesianTrajectory->addWaypoint(xd) );
    ASSERT_TRUE( iCartesianTrajectory->configurePath( ICartesianTrajectory::LINE ) );
    ASSERT_TRUE( iCartesianTrajectory->configureVelocityProfile( ICartesianTrajectory::TRAPEZOIDAL ) );
    ASSERT_TRUE( iCartesianTrajectory->create() );

    //-- Use line
    double duration;
    ASSERT_TRUE( iCartesianTrajectory->getDuration(&duration) );
    ASSERT_EQ(duration, 20.0);

    //-- Destroy line
    ASSERT_TRUE( iCartesianTrajectory->destroy() );
}

}  // namespace roboticslab
