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
        //Network yarp;  //-- This test can be performed without the network.
};

TEST_F( KdlControllerTest, KdlControllerFwdKin)
{
    YARP_REGISTER_PLUGINS(TeoYarp);

    //ASSERT_EQ(true, yarp.checkNetwork() );  //-- This test can be performed without the network.

    Property options;
    options.put("device","kdlcontroller");
    options.put("robotDevice","ravepart");  //-- ravepart because test_motor lacks joint limit interface.
    options.put("axes",1);  //-- Gets passed on to ravepart.
    Property& psub = options.addGroup("link_0");  //-- A nested Property, easier syntax from file.
    psub.put("A",1);
    dd.open(options);

    ASSERT_EQ(true, dd.isValid() );
    dd.view(icart);
    ASSERT_NE((ICartesianControl*)NULL, icart );
    yarp::sig::Vector x,o;
    icart->getPose(x,o);
    ASSERT_EQ(x[0], 1 );
    ASSERT_EQ(x[1], 0 );
    ASSERT_EQ(x[2], 0 );

}


