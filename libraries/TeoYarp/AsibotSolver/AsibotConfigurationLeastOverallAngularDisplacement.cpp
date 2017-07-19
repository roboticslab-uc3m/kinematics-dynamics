// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotConfiguration.hpp"

using namespace roboticslab;

AsibotConfigurationLeastOverallAngularDisplacement::AsibotConfigurationLeastOverallAngularDisplacement(
        const std::vector<double> & qMin, const std::vector<double> & qMax)
    : _qMax(qMax),_qMin(qMin),
      _q1(0.0), _q1f(0.0), _q1r(0.0),
      _q2(0.0), _q2uf(0.0), _q2df(0.0), _q2ur(0.0), _q2dr(0.0),
      _q3(0.0), _q3uf(0.0), _q3df(0.0), _q3ur(0.0), _q3dr(0.0),
      _q4(0), _q4f(0.0), _q4r(0.0),
      _q5(0), _q5f(0.0), _q5r(0.0)
{}

bool AsibotConfigurationLeastOverallAngularDisplacement::configure(double q1, double q2, double q3u, double q3d, double q4, double q5)
{
    _q1f = q1;
    _q1r = -q1;

    _q2uf = _q2dr = q2;
    _q2df = _q2ur = -q2;

    _q3uf = _q3dr = q3u;
    _q3df = _q3ur = q3d;

    _q4f = q4;
    _q4r = -q4;

    _q5f = q5;
    _q5r = -q5;

    return true;
}

bool AsibotConfigurationLeastOverallAngularDisplacement::findOptimalConfiguration()
{
    return true;
}

void AsibotConfigurationLeastOverallAngularDisplacement::retrieveAngles(std::vector<double> & q)
{
    q.resize(5);

    q[0] = _q1;
    q[1] = _q2;
    q[2] = _q3;
    q[3] = _q4;
    q[4] = _q5;
}
