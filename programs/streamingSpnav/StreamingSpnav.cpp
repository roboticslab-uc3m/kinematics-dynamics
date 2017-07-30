#include "StreamingSpnav.hpp"

#include <string>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <yarp/sig/Vector.h>

#include <ColorDebug.hpp>

namespace roboticslab
{

bool StreamingSpnav::configure(yarp::os::ResourceFinder &rf)
{
    CD_DEBUG("StreamingSpnav config: %s.\n", rf.toString().c_str());

    std::string localSpnav = rf.check("localSpnav", yarp::os::Value(DEFAULT_SPNAV_LOCAL), "local spnav port").asString();
    std::string remoteSpnav = rf.check("remoteSpnav", yarp::os::Value(DEFAULT_SPNAV_REMOTE), "remote spnav port").asString();

    std::string localCartesian = rf.check("localCartesian", yarp::os::Value(DEFAULT_CARTESIAN_LOCAL), "local cartesian port").asString();
    std::string remoteCartesian = rf.check("remoteCartesian", yarp::os::Value(DEFAULT_CARTESIAN_REMOTE), "remote cartesian port").asString();

    scaling = rf.check("scaling", yarp::os::Value(DEFAULT_SCALING), "scaling factor").asDouble();

    yarp::os::Value axesValue = rf.check("fixedAxes", yarp::os::Value(DEFAULT_FIXED_AXES), "axes with restricted movement");

    fixedAxes.resize(6, false);

    if (axesValue.isList())
    {
        yarp::os::Bottle * axesList = axesValue.asList();

        for (int i = 0; i < axesList->size(); i++)
        {
            std::string str = axesList->get(i).asString();

            if (str == "x")
            {
                fixedAxes[0] = true;
            }
            else if (str == "y")
            {
                fixedAxes[1] = true;
            }
            else if (str == "z")
            {
                fixedAxes[2] = true;
            }
            else if (str == "rotx")
            {
                fixedAxes[3] = true;
            }
            else if (str == "roty")
            {
                fixedAxes[4] = true;
            }
            else if (str == "rotz")
            {
                fixedAxes[5] = true;
            }
            else
            {
                CD_WARNING("Unrecognized fixed axis label: %s. Ignoring...\n", str.c_str());
            }
        }
    }

    if (rf.check("help"))
    {
        printf("StreamingSpnav options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        return false;
    }

    yarp::os::Property spnavClientOptions;
    spnavClientOptions.put("device", "analogsensorclient");
    spnavClientOptions.put("local", localSpnav);
    spnavClientOptions.put("remote", remoteSpnav);

    spnavClientDevice.open(spnavClientOptions);

    if (!spnavClientDevice.isValid())
    {
        CD_ERROR("spnav client device not valid.\n");
        return false;
    }

    if (!spnavClientDevice.view(iAnalogSensor))
    {
        CD_ERROR("Could not view iAnalogSensor.\n");
        return false;
    }

    yarp::os::Property cartesianControlClientOptions;
    cartesianControlClientOptions.put("device", "CartesianControlClient");
    cartesianControlClientOptions.put("cartesianLocal", localCartesian);
    cartesianControlClientOptions.put("cartesianRemote", remoteCartesian);

    cartesianControlClientDevice.open(cartesianControlClientOptions);

    if (!cartesianControlClientDevice.isValid())
    {
        CD_ERROR("cartesian control client device not valid.\n");
        return false;
    }

    if (!cartesianControlClientDevice.view(iCartesianControl))
    {
        CD_ERROR("Could not view iCartesianControl.\n");
        return false;
    }

    return true;
}

bool StreamingSpnav::updateModule()
{
    yarp::sig::Vector data;
    iAnalogSensor->read(data);

    CD_DEBUG("%s\n", data.toString(4, 1).c_str());

    if (data.size() != 6)
    {
        CD_ERROR("Invalid data size: %d.\n", data.size());
        return false;
    }

    std::vector<double> xdot(6, 0.0);

    for (int i = 0; i < data.size(); i++)
    {
        if (!fixedAxes[i])
        {
            xdot[i] = data[i] / scaling;
        }
    }

    if (!iCartesianControl->vmos(xdot))
    {
        CD_WARNING("vmos failed.\n");
    }

    return true;
}

bool StreamingSpnav::interruptModule()
{
    bool ok = true;
    ok &= iCartesianControl->stopControl();
    ok &= cartesianControlClientDevice.close();
    ok &= spnavClientDevice.close();
    return ok;
}

double StreamingSpnav::getPeriod()
{
    return 0.02;  // [s]
}

}  // namespace roboticslab
