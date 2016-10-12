// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TEO_SIM_EXAMPLE_HPP__
#define __TEO_SIM_EXAMPLE_HPP__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

namespace teo
{

class TeoSimExample
{
public:
    bool run();

private:
    yarp::os::Network yarp; // connect to YARP network
    yarp::dev::PolyDriver dd; //create a YARP multi-use driver
    yarp::dev::IPositionControl *pos; //make a position controller object we call 'pos'
    yarp::dev::IEncoders *enc; //make an encoder controller object we call 'enc'
    yarp::dev::IVelocityControl *vel; //make a velocity controller object we call 'vel'
};

}  // namespace teo

#endif // __TEO_SIM_EXAMPLE_HPP__

