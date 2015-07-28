// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianRateThread.hpp"

/************************************************************************/
bool teo::CartesianRateThread::threadInit() {

    iEncoders->getAxes( &numMotors );
    CD_INFO("numMotors: %d.\n",numMotors);

    solver->getNumLinks( &solverNumLinks );
    CD_INFO("solverNumLinks: %d.\n",solverNumLinks);

    if( numMotors < solverNumLinks ) {
        CD_ERROR("numMotors < solverNumLinks !!! (must be >=)\n");
        return false;
    }

    qReal.resize( numMotors );
    qDotCmd.resize( numMotors );
    xDesired.resize( 3 + 3 );  // x y z + r p y
    xDotDesired.resize( 3 + 3 );  // x y z + r p y

    iVelocityControl->setVelocityMode();

    lineCount = 1;
    return true;
}

/************************************************************************/
void teo::CartesianRateThread::run() {

    std::string line;
    if (! getline( ifs, line) )
    {
        this->stop();
        CD_INFO("[L:%d] file ended\n", lineCount );
        return;
    }

    CD_DEBUG("[L:%d] %s\n", lineCount,line.c_str() );
    lineCount++;
    yarp::os::Bottle lineBottle(line);  //-- yes, using a bottle to parse a string

    for(int i=0; i<xDesired.size(); i++)
        xDesired[i] = lineBottle.get( i ).asDouble();

    for(int i=0; i<xDotDesired.size(); i++)
        xDotDesired[i] = lineBottle.get( i+xDesired.size() ).asDouble();

    iEncoders->getEncoders( qReal.data() );

    CD_DEBUG("<-- ");
    for(int i=0; i<qReal.size(); i++)
        CD_DEBUG_NO_HEADER("%f ",qReal[i]);
    CD_DEBUG_NO_HEADER("[deg]\n");

    solver->fwdKin(qReal, xReal,oReal);

    //xError = xDesired - xReal;
    //xDotCmd = xDotDesired + GAIN * xError * (cmcMs/1000.0);  // GAIN=0 => xCmd = xDotDesired
    //solver->diffInvKin(qReal,xDotCmd, qDotCmd);
    solver->diffInvKin(qReal,xDotDesired, qDotCmd); // hack GAIN = 0

    CD_INFO("--> ");
    for(int i=0; i<qDotCmd.size(); i++) {
        CD_INFO_NO_HEADER("%f ",qDotCmd[i]);
    }
    CD_INFO_NO_HEADER("[deg/s]\n");

    iVelocityControl->velocityMove( qDotCmd.data() );

}

/************************************************************************/
