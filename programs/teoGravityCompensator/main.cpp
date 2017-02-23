// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup teo_programs
 *
 * \defgroup teoGravityCompensator teoGravityCompensator [DEPRECIATED]
 *
 * @brief [DEPRECIATED] Use \ref BasicCartesianControl instead. Creates an instance of teo::TeoGravityCompensator.
 *
 * <b> Legal </b>
 *
 * Copyright: 2014 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=72">Juan G. Victores</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see license/LGPL.TXT
 *
 * <b>Installation</b>
 *
 * The module is compiled when ENABLE_teoGravityCompensator is activated (default: OFF). For further
 * installation steps refer to <a class="el" href="pages.html">your own system installation guidelines</a>.
 *
 * <b>Running</b> (assuming correct installation)
 *
 * First we must run a YARP name server if it is not running in our current namespace:
\verbatim
[on terminal 1] yarp server
\endverbatim
 * And then launch the actual module:
\verbatim
[on terminal 2] bin/teoGravityCompensator
\endverbatim
 *
 * <b>Modify</b>
 *
 * This file can be edited at 
 * programs/teoGravityCompensator/main.cpp
 *
 */

#include "ColorDebug.hpp"
#include "TeoGravityCompensator.hpp"



//YARP_DECLARE_PLUGINS(TeoYarp)  //-- Provides "KdlSolver".

int main(int argc, char *argv[]) {

    //YARP_REGISTER_PLUGINS(TeoYarp);  //-- Provides "KdlSolver".

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("teoGravityCompensator");
    rf.setDefaultConfigFile("teoGravityCompensator.ini");
    rf.configure(argc,argv);

    teo::TeoGravityCompensator mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    CD_INFO("Run \"teoGravityCompensator --help\" for options.\n");
    CD_INFO("teoGravityCompensator checking for yarp network...\n");
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        CD_ERROR("Found no yarp network (try running \"yarpserver &\"), bye!\n");
        return -1;
    }
    CD_SUCCESS("Found Yarp network.\n");

    return mod.runModule(rf);
}

