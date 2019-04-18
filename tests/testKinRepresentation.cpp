#include "gtest/gtest.h"

#include <cmath>
#include <vector>

#include "KinematicRepresentation.hpp"

namespace roboticslab
{

/**
 * @ingroup kinematics-dynamics-tests
 * @brief Tests \ref KinRepresentation.
 */
class KinRepresentationTest : public testing::Test
{
public:
    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {
    }

protected:
    static const double EPS;
};

const double KinRepresentationTest::EPS = 1e-9;

TEST_F(KinRepresentationTest, KinRepresentationEncodePoseAxisAngle)
{
    std::vector<double> x_in(7), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = 1.0;
    x_in[6] = 45.0;

    ASSERT_TRUE(KinRepresentation::encodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], M_PI / 4, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationEncodePoseAxisAngleScaled)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = 45.0;

    ASSERT_TRUE(KinRepresentation::encodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE_SCALED, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], M_PI / 4, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationEncodePoseRPY)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = 45.0;

    ASSERT_TRUE(KinRepresentation::encodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::RPY, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], M_PI / 4, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationEncodePoseEulerYZ)
{
    std::vector<double> x_in(5), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;

    ASSERT_TRUE(KinRepresentation::encodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], M_PI / 4, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationEncodePoseEulerZYZ)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 45.0;
    x_in[4] = 0.0;
    x_in[5] = 0.0;

    ASSERT_TRUE(KinRepresentation::encodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::EULER_ZYZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0, EPS);
    ASSERT_NEAR(x_out[5], M_PI / 4, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationEncodePoseRadians)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = M_PI / 4;

    ASSERT_TRUE(KinRepresentation::encodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE_SCALED, KinRepresentation::RADIANS));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], M_PI / 4, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationDecodePoseAxisAngle)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = M_PI / 4;

    ASSERT_TRUE(KinRepresentation::decodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 7);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], 1.0, EPS);
    ASSERT_NEAR(x_out[6], 45.0, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationDecodePoseAxisAngleScaled)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = M_PI / 4;

    ASSERT_TRUE(KinRepresentation::decodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE_SCALED, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], 45.0, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationDecodePoseRPY)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = M_PI / 4;

    ASSERT_TRUE(KinRepresentation::decodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::RPY, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], 45.0, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationDecodePoseEulerYZ)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = M_PI / 4;

    ASSERT_TRUE(KinRepresentation::decodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::EULER_YZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 5);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationDecodePoseEulerZYZ)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = M_PI / 4;

    ASSERT_TRUE(KinRepresentation::decodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::EULER_ZYZ, KinRepresentation::DEGREES));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 45.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], 0.0, EPS);
}

TEST_F(KinRepresentationTest, KinRepresentationDecodePoseRadians)
{
    std::vector<double> x_in(6), x_out;

    x_in[0] = 1.0;
    x_in[1] = 1.0;
    x_in[2] = 2.0;
    x_in[3] = 0.0;
    x_in[4] = 0.0;
    x_in[5] = M_PI / 4;

    ASSERT_TRUE(KinRepresentation::decodePose(x_in, x_out, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE_SCALED, KinRepresentation::RADIANS));

    ASSERT_EQ(x_out.size(), 6);

    ASSERT_NEAR(x_out[0], 1.0, EPS);
    ASSERT_NEAR(x_out[1], 1.0, EPS);
    ASSERT_NEAR(x_out[2], 2.0, EPS);
    ASSERT_NEAR(x_out[3], 0.0, EPS);
    ASSERT_NEAR(x_out[4], 0.0, EPS);
    ASSERT_NEAR(x_out[5], M_PI / 4, EPS);
}

}  // namespace roboticslab
