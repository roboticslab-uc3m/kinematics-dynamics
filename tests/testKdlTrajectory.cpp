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
        iCartesianTrajectory = new KdlTrajectory();

        x1.resize(6);
        x2.resize(6);
        v1.resize(6);
        v1_alt.resize(6);

        x1[0] = 1.0;
        x2[0] = 2.0;
        v1[0] = 0.1;
        v1_alt[0] = 0.5;
    }

    virtual void TearDown()
    {
        delete iCartesianTrajectory;
        iCartesianTrajectory = 0;
    }

protected:
    ICartesianTrajectory* iCartesianTrajectory;

    std::vector<double> x1, x2, v1, v1_alt;

    static const double DURATION;
    static const double MAX_VEL;
    static const double MAX_ACC;

    static const double EPS;
};

const double KdlTrajectoryTest::DURATION = 10.0;
const double KdlTrajectoryTest::MAX_VEL = 0.2;
const double KdlTrajectoryTest::MAX_ACC = 0.05;

const double KdlTrajectoryTest::EPS = 1e-9;

TEST_F(KdlTrajectoryTest, KdlTrajectoryCreate)
{
    //-- Create dummy trajectory, no path nor velocity profile set
    ASSERT_FALSE(iCartesianTrajectory->create());

    //-- Destroy line
    ASSERT_TRUE(iCartesianTrajectory->destroy());
}

TEST_F(KdlTrajectoryTest, KdlTrajectoryLineTrap)
{
    //-- Create line trajectory
    ASSERT_TRUE(iCartesianTrajectory->setDuration(DURATION)); // Must be higher than 9.0 due to defaults
    ASSERT_TRUE(iCartesianTrajectory->setMaxVelocity(MAX_VEL));
    ASSERT_TRUE(iCartesianTrajectory->setMaxAcceleration(MAX_ACC));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x1));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x2));
    ASSERT_TRUE(iCartesianTrajectory->configurePath(ICartesianTrajectory::LINE));
    ASSERT_TRUE(iCartesianTrajectory->configureVelocityProfile(ICartesianTrajectory::TRAPEZOIDAL));
    ASSERT_TRUE(iCartesianTrajectory->create());

    //-- Query duration, should be the same as set before
    double duration;
    ASSERT_TRUE(iCartesianTrajectory->getDuration(&duration));
    ASSERT_EQ(duration, DURATION);

    //-- Destroy line
    ASSERT_TRUE(iCartesianTrajectory->destroy());
}

TEST_F(KdlTrajectoryTest, KdlTrajectoryLineTrapNoDuration)
{
    //-- Create line trajectory
    ASSERT_TRUE(iCartesianTrajectory->setMaxVelocity(MAX_VEL));
    ASSERT_TRUE(iCartesianTrajectory->setMaxAcceleration(MAX_ACC));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x1));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x2));
    ASSERT_TRUE(iCartesianTrajectory->configurePath(ICartesianTrajectory::LINE));
    ASSERT_TRUE(iCartesianTrajectory->configureVelocityProfile(ICartesianTrajectory::TRAPEZOIDAL));
    ASSERT_TRUE(iCartesianTrajectory->create());

    //-- Query duration according to a trapezoidal velocity profile
    double duration;
    ASSERT_TRUE(iCartesianTrajectory->getDuration(&duration));
    ASSERT_NEAR(duration, 9.0, EPS);

    //-- Use line
    double movementTime;
    std::vector<double> position, velocity, acceleration;

    // ramp up
    movementTime = 2.0;
    ASSERT_TRUE(iCartesianTrajectory->getPosition(movementTime, position));
    ASSERT_NEAR(position[0], x1[0] + 0.1, EPS);
    ASSERT_TRUE(iCartesianTrajectory->getVelocity(movementTime, velocity));
    ASSERT_NEAR(velocity[0], MAX_ACC * movementTime, EPS);
    ASSERT_TRUE(iCartesianTrajectory->getAcceleration(movementTime, acceleration));
    ASSERT_NEAR(acceleration[0], MAX_ACC, EPS);

    // steady
    movementTime = 4.5;
    ASSERT_TRUE(iCartesianTrajectory->getPosition(movementTime, position));
    ASSERT_NEAR(position[0], x1[0] + 0.5, EPS);
    ASSERT_TRUE(iCartesianTrajectory->getVelocity(movementTime, velocity));
    ASSERT_NEAR(velocity[0], MAX_VEL, EPS);
    ASSERT_TRUE(iCartesianTrajectory->getAcceleration(movementTime, acceleration));
    ASSERT_NEAR(acceleration[0], 0.0, EPS);

    // ramp down
    movementTime = 7.0;
    ASSERT_TRUE(iCartesianTrajectory->getPosition(movementTime, position));
    ASSERT_NEAR(position[0], x1[0] + 0.9, EPS);
    ASSERT_TRUE(iCartesianTrajectory->getVelocity(movementTime, velocity));
    ASSERT_NEAR(velocity[0], MAX_ACC * (duration - movementTime), EPS);
    ASSERT_TRUE(iCartesianTrajectory->getAcceleration(movementTime, acceleration));
    ASSERT_NEAR(acceleration[0], -MAX_ACC, EPS);

    //-- Destroy line
    ASSERT_TRUE(iCartesianTrajectory->destroy());
}

TEST_F(KdlTrajectoryTest, KdlTrajectoryLineRect)
{
    //-- Create line trajectory
    ASSERT_TRUE(iCartesianTrajectory->setDuration(DURATION)); // Short duration means higher vel, beware of default limit
    ASSERT_TRUE(iCartesianTrajectory->setMaxVelocity(MAX_VEL));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x1));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x2));
    ASSERT_TRUE(iCartesianTrajectory->configurePath(ICartesianTrajectory::LINE));
    ASSERT_TRUE(iCartesianTrajectory->configureVelocityProfile(ICartesianTrajectory::RECTANGULAR));
    ASSERT_TRUE(iCartesianTrajectory->create());

    //-- Query duration, should be the same as set before
    double duration;
    ASSERT_TRUE(iCartesianTrajectory->getDuration(&duration));
    ASSERT_NEAR(duration, DURATION, EPS);

    //-- Use line
    double movementTime = 5.0;
    std::vector<double> position, velocity, acceleration;

    ASSERT_TRUE(iCartesianTrajectory->getPosition(movementTime, position));
    ASSERT_NEAR(position[0], x1[0] + (x2[0] - x1[0]) * (movementTime / duration), EPS);
    ASSERT_TRUE(iCartesianTrajectory->getVelocity(movementTime, velocity));
    ASSERT_NEAR(velocity[0], (x2[0] - x1[0]) / duration, EPS);
    ASSERT_FALSE(iCartesianTrajectory->getAcceleration(movementTime, acceleration));

    //-- Destroy line
    ASSERT_TRUE(iCartesianTrajectory->destroy());
}

TEST_F(KdlTrajectoryTest, KdlTrajectoryLineRectNoDuration)
{
    //-- Create line trajectory
    ASSERT_TRUE(iCartesianTrajectory->setMaxVelocity(MAX_VEL));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x1));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x2));
    ASSERT_TRUE(iCartesianTrajectory->configurePath(ICartesianTrajectory::LINE));
    ASSERT_TRUE(iCartesianTrajectory->configureVelocityProfile(ICartesianTrajectory::RECTANGULAR));
    ASSERT_TRUE(iCartesianTrajectory->create());

    //-- Query duration according to a rectangular velocity profile
    double duration;
    ASSERT_TRUE(iCartesianTrajectory->getDuration(&duration));
    ASSERT_NEAR(duration, 5.0, EPS);

    //-- Use line
    double movementTime = 2.0;
    std::vector<double> position, velocity, acceleration;

    ASSERT_TRUE(iCartesianTrajectory->getPosition(movementTime, position));
    ASSERT_NEAR(position[0], x1[0] + MAX_VEL * movementTime, EPS);
    ASSERT_TRUE(iCartesianTrajectory->getVelocity(movementTime, velocity));
    ASSERT_NEAR(velocity[0], MAX_VEL, EPS);
    ASSERT_FALSE(iCartesianTrajectory->getAcceleration(movementTime, acceleration));

    //-- Destroy line
    ASSERT_TRUE(iCartesianTrajectory->destroy());
}

TEST_F(KdlTrajectoryTest, KdlTrajectoryLineRectInitialTwist)
{
    //-- Create line trajectory
    ASSERT_TRUE(iCartesianTrajectory->setDuration(DURATION)); // Short duration means higher vel, beware of default limit
    ASSERT_TRUE(iCartesianTrajectory->setMaxVelocity(MAX_VEL));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x1, v1));
    ASSERT_TRUE(iCartesianTrajectory->configurePath(ICartesianTrajectory::LINE));
    ASSERT_TRUE(iCartesianTrajectory->configureVelocityProfile(ICartesianTrajectory::RECTANGULAR));
    ASSERT_TRUE(iCartesianTrajectory->create());

    //-- Query duration, should be the same as set before
    double duration;
    ASSERT_TRUE(iCartesianTrajectory->getDuration(&duration));
    ASSERT_NEAR(duration, DURATION, EPS);

    //-- Use line
    double movementTime = 5.0;
    std::vector<double> position, velocity, acceleration;

    ASSERT_TRUE(iCartesianTrajectory->getPosition(movementTime, position));
    ASSERT_NEAR(position[0], x1[0] + v1[0] * movementTime, EPS);
    ASSERT_TRUE(iCartesianTrajectory->getVelocity(movementTime, velocity));
    ASSERT_NEAR(velocity[0], v1[0], EPS);
    ASSERT_FALSE(iCartesianTrajectory->getAcceleration(movementTime, acceleration));

    //-- Destroy line
    ASSERT_TRUE(iCartesianTrajectory->destroy());
}

TEST_F(KdlTrajectoryTest, KdlTrajectoryLineRectInitialTwistNoDuration)
{
    //-- Create line trajectory
    ASSERT_TRUE(iCartesianTrajectory->setMaxVelocity(MAX_VEL));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x1, v1));
    ASSERT_TRUE(iCartesianTrajectory->configurePath(ICartesianTrajectory::LINE));
    ASSERT_TRUE(iCartesianTrajectory->configureVelocityProfile(ICartesianTrajectory::RECTANGULAR));
    ASSERT_TRUE(iCartesianTrajectory->create());

    //-- Use line
    double movementTime = 5.0;
    std::vector<double> position, velocity, acceleration;

    ASSERT_TRUE(iCartesianTrajectory->getPosition(movementTime, position));
    ASSERT_NEAR(position[0], x1[0] + v1[0] * movementTime, EPS);
    ASSERT_TRUE(iCartesianTrajectory->getVelocity(movementTime, velocity));
    ASSERT_NEAR(velocity[0], v1[0], EPS);
    ASSERT_FALSE(iCartesianTrajectory->getAcceleration(movementTime, acceleration));

    //-- Destroy line
    ASSERT_TRUE(iCartesianTrajectory->destroy());
}

TEST_F(KdlTrajectoryTest, KdlTrajectoryLineRectInitialTwistNoDurationCapped)
{
    //-- Create line trajectory
    ASSERT_TRUE(iCartesianTrajectory->setMaxVelocity(MAX_VEL));
    ASSERT_TRUE(iCartesianTrajectory->addWaypoint(x1, v1_alt));
    ASSERT_TRUE(iCartesianTrajectory->configurePath(ICartesianTrajectory::LINE));
    ASSERT_TRUE(iCartesianTrajectory->configureVelocityProfile(ICartesianTrajectory::RECTANGULAR));
    ASSERT_TRUE(iCartesianTrajectory->create());

    //-- Use line
    double movementTime = 5.0;
    std::vector<double> position, velocity, acceleration;

    ASSERT_TRUE(iCartesianTrajectory->getPosition(movementTime, position));
    ASSERT_NEAR(position[0], x1[0] + MAX_VEL * movementTime, EPS);
    ASSERT_TRUE(iCartesianTrajectory->getVelocity(movementTime, velocity));
    ASSERT_NEAR(velocity[0], MAX_VEL, EPS);
    ASSERT_FALSE(iCartesianTrajectory->getAcceleration(movementTime, acceleration));

    //-- Destroy line
    ASSERT_TRUE(iCartesianTrajectory->destroy());
}

}  // namespace roboticslab
