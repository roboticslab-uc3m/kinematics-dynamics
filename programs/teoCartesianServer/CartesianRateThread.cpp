// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianRateThread.hpp"

namespace teo
{

/************************************************************************/

CartesianRateThread::CartesianRateThread() : RateThread(DEFAULT_MS)
{
    currentState = STOPPED;
}

/************************************************************************/

bool CartesianRateThread::threadInit()
{
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
    xDotDesired.resize( 6 );
    xDotCmd.resize( 6 );

    return true;
}

/************************************************************************/

bool CartesianRateThread::load(const std::string& fileName)
{
    std::string fullFileName = rf->findFileByName( fileName );
    this->ifs.open( fullFileName.c_str() );
    if( ! this->ifs.is_open() )
    {
        CD_ERROR("Could not open read file: %s.\n",fullFileName.c_str());
        return false;
    }
    CD_SUCCESS("Opened file: %s.\n",fileName.c_str());

    lineCount = 1;
    currentState = PTMODE;

    iVelocityControl->setVelocityMode();

    return true;
}

/************************************************************************/

void CartesianRateThread::run()
{
    if (currentState == PTMODE)
    {
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

        //-- xDesired from file
        for(int i=0; i<xDesired.size(); i++)
            xDesired[i] = lineBottle.get( i ).asDouble();

        //-- xDotDesired from file
        for(int i=0; i<xDotDesired.size(); i++)
            xDotDesired[i] = lineBottle.get( i+xDesired.size() ).asDouble();

        //-- qReal from encoders
        iEncoders->getEncoders( qReal.data() );

        //-- xError = xDesired - xReal, where xReal is fwdKin of qReal
        solver->fwdKinError(xDesired,qReal, xError);

        //-- control law in cartesian space
        for(int i=0; i<xDotCmd.size(); i++)
            xDotCmd[i] = xDotDesired[i] + DEFAULT_GAIN * xError[i];

        //-- final command to joint space
        solver->diffInvKin(qReal,xDotCmd, qDotCmd);

        CD_INFO("--> ");
        for(int i=0; i<qDotCmd.size(); i++) {
            CD_INFO_NO_HEADER("%f ",qDotCmd[i]);
        }
        CD_INFO_NO_HEADER("[deg/s]\n");

        iVelocityControl->velocityMove( qDotCmd.data() );
    } //end{if(currentState == PTMODE)}
    /*else if (currentState == MOVL)
    {

    }*/
}

/************************************************************************/

bool CartesianRateThread::stat(std::vector<double>& stat)
{
    //-- qReal from encoders
    iEncoders->getEncoders( qReal.data() );

    solver->fwdKin(qReal, stat);
    return true;
}

/************************************************************************/

}  // namespace teo
