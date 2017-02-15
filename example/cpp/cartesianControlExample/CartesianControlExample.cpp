// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup teo_examples_cpp
 * \defgroup cartesianControlExample cartesianControlExample
 *
 * @brief Creates an instance of teo::CartesianControlExample.
 *
 * <b>Legal</b>
 *
 * Copyright: (C) 2017 Universidad Carlos III de Madrid;
 *
 * Authors: rsantos88, jgvictores
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd $TEO_MAIN_ROOT/example/cpp/cartesianControlExample/
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * <b>Running</b>
\verbatim
./cartesianControlExample
\endverbatim
 *
 */
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include "ICartesianControl.h" // incluimos este interfaz, puesto que vamos a usar las funciones de la librería compartida (device CartesianControlClient)

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;

    if ( ! yarp::os::Network::checkNetwork() )
    {
        printf("Please start a yarp name server first\n");
        return(1);
    }
    yarp::os::Property options;
    options.put("device","CartesianControlClient");
    options.put("cartesianRemote","/teoSim/rightArm/CartesianControlServer"); // puerto al que nos vamos a conectar
    options.put("cartesianLocal","/CartesianControlExample");

    yarp::dev::PolyDriver dd(options);
    if(!dd.isValid()) {
      printf("Device not available.\n");
      dd.close();
      yarp::os::Network::fini();
      return 1;
    }


    teo::ICartesianControl *iCartesianControl;       

    if ( ! dd.view(iCartesianControl) )
    {
        printf("[error] Problems acquiring interface\n");
        return 1;
    }
    printf("[success] acquired interface\n");

    std::vector<double> vector;
    int status;

    iCartesianControl->stat( status, vector );
    printf("Controller status (forward kinematics): ");
    for (int i=0; i < vector.size(); i++)
        printf("%f ", vector[i]);
    printf("\n");

    dd.close();

    return 0;
}
