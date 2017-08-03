// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool roboticslab::CartesianControlClient::stat(int &state, std::vector<double> &x)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_STAT);

    rpcClient.write(cmd,response);

    state = response.get(0).asVocab();
    x.resize(response.size()-1);
    for(size_t i=0; i<response.size()-1; i++)
        x[i] = response.get(i+1).asDouble();
    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::inv(const std::vector<double> &xd, std::vector<double> &q)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_INV);
    for(size_t i=0; i<xd.size(); i++)
        cmd.addDouble(xd[i]);

    rpcClient.write(cmd,response);

    if( response.get(0).isVocab() )
    {
        if( response.get(0).asVocab() == VOCAB_FAILED )
        {
            return false;
        }
    }

    for(size_t i=0; i<response.size(); i++)
        q.push_back(response.get(i).asDouble());

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::movj(const std::vector<double> &xd)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_MOVJ);
    for(size_t i=0; i<xd.size(); i++)
        cmd.addDouble(xd[i]);

    rpcClient.write(cmd,response);

    if( response.get(0).isVocab() )
    {
        if( response.get(0).asVocab() == VOCAB_FAILED )
        {
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::relj(const std::vector<double> &xd)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_RELJ);
    for(size_t i=0; i<xd.size(); i++)
        cmd.addDouble(xd[i]);

    rpcClient.write(cmd,response);

    if( response.get(0).isVocab() )
    {
        if( response.get(0).asVocab() == VOCAB_FAILED )
        {
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::movl(const std::vector<double> &xd)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_MOVL);
    for(size_t i=0; i<xd.size(); i++)
        cmd.addDouble(xd[i]);

    rpcClient.write(cmd,response);

    if( response.get(0).isVocab() )
    {
        if( response.get(0).asVocab() == VOCAB_FAILED )
        {
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::movv(const std::vector<double> &xdotd)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_MOVV);
    for(size_t i=0; i<xdotd.size(); i++)
        cmd.addDouble(xdotd[i]);

    rpcClient.write(cmd,response);

    if( response.get(0).isVocab() )
    {
        if( response.get(0).asVocab() == VOCAB_FAILED )
        {
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::gcmp()
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_GCMP);

    rpcClient.write(cmd,response);

    if( response.get(0).asVocab() == VOCAB_FAILED )
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::forc(const std::vector<double> &td)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_FORC);
    for(size_t i=0; i<td.size(); i++)
        cmd.addDouble(td[i]);

    rpcClient.write(cmd,response);

    if( response.get(0).isVocab() )
    {
        if( response.get(0).asVocab() == VOCAB_FAILED )
        {
            return false;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::stopControl()
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_STOP);

    rpcClient.write(cmd,response);

    if( response.get(0).asVocab() == VOCAB_FAILED )
    {
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::fwd(const std::vector<double> &rot)
{
    yarp::os::Bottle& cmd = commandBuffer.get();

    cmd.clear();
    cmd.addVocab(VOCAB_CC_FWD);
    for(size_t i=0; i<rot.size(); i++)
        cmd.addDouble(rot[i]);

    commandBuffer.write(true);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::bkwd(const std::vector<double> &rot)
{
    yarp::os::Bottle& cmd = commandBuffer.get();

    cmd.clear();
    cmd.addVocab(VOCAB_CC_BKWD);
    for(size_t i=0; i<rot.size(); i++)
        cmd.addDouble(rot[i]);

    commandBuffer.write(true);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::rot(const std::vector<double> &rot)
{
    yarp::os::Bottle& cmd = commandBuffer.get();

    cmd.clear();
    cmd.addVocab(VOCAB_CC_ROT);
    for(size_t i=0; i<rot.size(); i++)
        cmd.addDouble(rot[i]);

    commandBuffer.write(true);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::vmos(const std::vector<double> &xdot)
{
    yarp::os::Bottle& cmd = commandBuffer.get();

    cmd.clear();
    cmd.addVocab(VOCAB_CC_VMOS);
    for(size_t i=0; i<xdot.size(); i++)
        cmd.addDouble(xdot[i]);

    commandBuffer.write(true);

    return true;
}

// -----------------------------------------------------------------------------

bool roboticslab::CartesianControlClient::pose(const std::vector<double> &x)
{
    yarp::os::Bottle& cmd = commandBuffer.get();

    cmd.clear();
    cmd.addVocab(VOCAB_CC_POSE);
    for(size_t i=0; i<x.size(); i++)
        cmd.addDouble(x[i]);

    commandBuffer.write(true);

    return true;
}

// -----------------------------------------------------------------------------
