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
 * Authors: Raul de Santos Rico
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Building</b>
\verbatim
cd $TEO_MAIN_ROOT/example/cpp/cartesianControlExample/
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 * <b>Running example with teoSim</b>
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * What moslty changes is the library command line invocation. We also change the server port name. The following is an example for the simulated robot's right arm.
\verbatim
[on terminal 2] launchTeoYarp --device BasicCartesianControl --name /teoSim/rightArm/CartesianControlServer --from /usr/local/share/teo/contexts/kinematics/rightArmKinematics.ini --angleRepr axisAngle --robot remote_controlboard --local /BasicCartesianControl/teoSim/rightArm --remote /teoSim/rightArm
[on terminal 3] ./cartesianControlExample
\endverbatim
 *
 *
 *
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
    options.put("device","CartesianControlClient"); // Aquí es donde le indicamos el device (shared library)
    options.put("cartesianRemote","/teoSim/rightArm/CartesianControlServer"); // puerto al que nos vamos a conectar con el servidor
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

    // -- Primero, mover el brazo en posición articular:
    // -- set poss (0 0 0 90 0 0 0)

    // -- Position 1
    std::vector<double> position;
    position.push_back(0.390926); // 0.390926 -0.346663 0.166873 -0.004334 0.70944 0.704752 0.353119
    position.push_back(-0.346663);
    position.push_back(0.166873);
    position.push_back(-0.004334);
    position.push_back(0.70944);
    position.push_back(0.704752);
    position.push_back(0.353119);

    // movj -> mueve en posición
    // movl -> mueve en velocidad
    printf("Position 1: poss (0 0 0 90 0 0 0)\n");
    iCartesianControl->movj(position);

    // -- Position 2: mueve en eje X hacia al frente
    printf("Position 2: mueve en eje X hacia el frente\n");
    position [0] = 0.5;
    iCartesianControl->movj(position);

    // -- Position 3: mueve en eje Y hacia su derecha
    printf("Position 3: mueve en eje Y hacia su derecha\n");
    position [1] = -0.5;
    iCartesianControl->movj(position);

    // -- Position 4: rota en eje Y 10 grados
    printf("Position 4: rota en eje Y 10 grados\n");
    position [3] = 0.0;
    position [4] = 1.0;
    position [5] = 0.0;
    position [6] = 10.0;
    iCartesianControl->movj(position);

    // -- Position 5: rota en eje Y 30 grados
    printf("Position 5: rota en eje Y 30 grados\n");
    position [6] = 30.0;
    iCartesianControl->movj(position);


    dd.close();

    return 0;
}
