// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup examples_cpp
 *
 * \defgroup exampleYarpTinyMath exampleYarpTinyMath
 *
 * @brief This is an example of the use of YarpTinyMath.
 *
 * @section exampleYarpTinyMath_legal Legal
 *
 * Copyright: (C) 2013 Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=72">Juan G. Victores</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section exampleYarpTinyMath_build Building
\verbatim
cd examples/cpp/exampleYarpTinyMath/
mkdir build; cd build; cmake ..
make -j3
\endverbatim
 *
 * @section exampleYarpTinyMath_run Running
\verbatim
./exampleYarpTinyMath
\endverbatim
 *
 */

#include <stdio.h>

#include "YarpTinyMath.hpp"

int main(int argc, char *argv[]) {

    double x_k = 0;
    double y_k = 0;
    double z_k = 1;

    double height=0.1;
    double tilt=90;  // 90 deg from vertical = horizontal std kinect looking position.
    double pan=10;  // deg, from front
    
    printf("Lets say a 3D sensor grabbed a point at sensor coordinates %f, %f, %f.\n\n",x_k,y_k,z_k);

    yarp::sig::Matrix H_0_c = roboticslab::rotZ(pan);
    H_0_c.resize(4,4);
    H_0_c(3,3)=1;
    printf("*** H_0_c *** \n(%s)\n\n", H_0_c.toString().c_str());

    yarp::sig::Matrix H_c_k = roboticslab::rotY(tilt);
    H_c_k.resize(4,4);
    H_c_k(2,3)=height;
    H_c_k(3,3)=1;
    printf("*** H_c_k *** \n(%s)\n\n", H_c_k.toString().c_str());

    yarp::sig::Matrix H_k_P = yarp::math::eye(4,4);
    H_k_P(0,3)=x_k;
    H_k_P(1,3)=y_k;
    H_k_P(2,3)=z_k;
    printf("*** H_k_P *** \n(%s)\n\n", H_k_P.toString().c_str());

    using namespace yarp::math;
    yarp::sig::Matrix H_0_P = H_0_c * H_c_k * H_k_P;

    double x_0 = H_0_P(0,3);
    double y_0 = H_0_P(1,3);
    double z_0 = H_0_P(2,3);    

    printf("In origin coordinates this would be: %f, %f, %f.\n",x_0,y_0,z_0);

    printf("Bye!\n");

    return 0;
}

