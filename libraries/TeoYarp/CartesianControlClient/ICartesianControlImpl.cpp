// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlClient.hpp"

// ------------------- ICartesianControl Related ------------------------------------

bool teo::CartesianControlClient::stat(int &state, std::vector<double> &x)
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_STAT);

    rpcClient.write(cmd,response);

    state = response.get(0).asVocab();
    for(size_t i=1; i<response.size(); i++)
        x.push_back(response.get(i).asDouble());
    return true;
}

// -----------------------------------------------------------------------------

bool teo::CartesianControlClient::inv(const std::vector<double> &xd, std::vector<double> &q)
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

bool teo::CartesianControlClient::movj(const std::vector<double> &xd)
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

bool teo::CartesianControlClient::movl(const std::vector<double> &xd)
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

bool teo::CartesianControlClient::stop()
{
    yarp::os::Bottle cmd, response;

    cmd.addVocab(VOCAB_CC_STOP);

    rpcClient.write(cmd,response);

    return true;
}

// -----------------------------------------------------------------------------
