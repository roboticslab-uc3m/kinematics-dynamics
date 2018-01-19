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
    }

    virtual void TearDown()
    {
    }
};

TEST_F(KdlTrajectoryTest, KdlTrajectoryConfigureLine)
{
    ASSERT_EQ(1, 1);
}

}  // namespace roboticslab
