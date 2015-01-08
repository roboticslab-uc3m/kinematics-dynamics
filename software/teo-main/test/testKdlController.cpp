#include "gtest/gtest.h"

#include "KdlController.hpp"


using namespace teo;

YARP_DECLARE_PLUGINS(TeoYarp)

class KdlControllerTest : public testing::Test
{

    public:
        virtual void SetUp() {
            icart = NULL;
        }

        virtual void TearDown()
        {
        }

    protected:
        PolyDriver dd;
        ICartesianControl *icart;
        Network yarp;
};

TEST_F( KdlControllerTest, KdlControllerConnect)
{
    YARP_REGISTER_PLUGINS(TeoYarp);

    ASSERT_EQ(true, yarp.checkNetwork() );

    Property options;
    options.put("device","kdlcontroller");
    options.put("robotDevice","ravepart");  //-- because test_motor lacks joint limit interface
    dd.open(options);

    ASSERT_EQ(true, dd.isValid() );
    dd.view(icart);
    ASSERT_NE((ICartesianControl*)NULL, icart );

}


