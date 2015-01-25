#include "gtest/gtest.h"

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>

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
        yarp::dev::PolyDriver dd;
        yarp::dev::ICartesianControl *icart;
};

TEST_F( KdlControllerTest, KdlSolverFwdKin)
{
    YARP_REGISTER_PLUGINS(TeoYarp);

    yarp::os::Property options;
    options.put("device","kdlsolver");
    yarp::os::Property& psub = options.addGroup("link_0");  //-- A nested Property, easier syntax from file.
    psub.put("A",1);
    dd.open(options);

    ASSERT_EQ(true, dd.isValid() );
    dd.view(icart);
    ASSERT_NE((yarp::dev::ICartesianControl*)NULL, icart );
    yarp::sig::Vector x,o;
    icart->getPose(x,o);
    ASSERT_EQ(x[0], 1 );
    ASSERT_EQ(x[1], 0 );
    ASSERT_EQ(x[2], 0 );

}


