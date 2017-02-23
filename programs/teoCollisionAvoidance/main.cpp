// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup teo_programs
 *
 * \defgroup teoCollisionAvoidance teoCollisionAvoidance
 *
 * @brief Creates an instance of teo::TeoCollisionAvoidance.
 *
 * @section teoCollisionAvoidance_legal Legal
 *
 * Copyright: 2013 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona_publ.php?id_pers=72">Juan G. Victores</a>
 *
 * Contrib: Paul Fitzpatrick (YARP sample code, email responses).
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * @section teoCollisionAvoidance_install Installation
 *
 * The module is compiled when ENABLE_teoCollisionAvoidance is activated (default: OFF). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 *
 * @section teoCollisionAvoidance_running Running (assuming correct installation)
 *
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * And then launch the actual module:
\verbatim
[on terminal 2] teoCollisionAvoidance
\endverbatim
 *
 * You should get a window similar to the one depicted on Figure 1.

\image html teoCollisionAvoidance.png
<center>Fig. 1 - An instance of the \ref teoCollisionAvoidance module.</center>

 * @section teoCollisionAvoidance_interfacing Interfacing with the teoCollisionAvoidance module
 *
\verbatim
[on terminal 3] yarp rpc /teoSim/leftArm/rpc:i
[on terminal 3] yarp rpc /teoSim/rightArm/rpc:i
\endverbatim
 *
 * Note 1: Change '/teoSim' to '/teo' for the real robot!
 *
 * @section teoCollisionAvoidance_modify Modify
 *
 * This file can be edited at 
 * programs/teoCollisionAvoidance/main.cpp
 *
 */

#include "ColorDebug.hpp"
#include "TeoCollisionAvoidance.hpp"


//YARP_DECLARE_PLUGINS(TeoYarp)

int main(int argc, char *argv[]) {

    //YARP_REGISTER_PLUGINS(TeoYarp);

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("teoCollisionAvoidance");
    rf.setDefaultConfigFile("teoCollisionAvoidance.ini");
    rf.configure(argc,argv);

    teo::TeoCollisionAvoidance mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    CD_INFO("Run \"teoCollisionAvoidance --help\" for options.\n");
    CD_INFO("Checking for yarp network...\n");
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return -1;
    } else {
        CD_SUCCESS("Yarp network found.\n");
    }

    return mod.runModule(rf);
}

