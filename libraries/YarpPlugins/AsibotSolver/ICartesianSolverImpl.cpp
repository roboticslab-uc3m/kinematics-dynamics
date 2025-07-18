// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "AsibotSolver.hpp"

#include <cmath>
#include <memory>

#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include "KinematicRepresentation.hpp"
#include "LogComponent.hpp"

using namespace roboticslab;
using namespace roboticslab::KinRepresentation;

// -----------------------------------------------------------------------------

namespace
{
    yarp::sig::Matrix vectorToMatrix(const std::vector<double> &v, bool fillTransl)
    {
        yarp::sig::Vector axisAngleScaled(3);

        for (int i = 0; i < 3; i++)
        {
            axisAngleScaled[i] = v[i + 3];
        }

        double rotAngle = yarp::math::norm(axisAngleScaled);

        yarp::sig::Vector axisAngle = axisAngleScaled;

        if (rotAngle > 1e-9)
        {
            axisAngle /= rotAngle;
        }
        else
        {
            axisAngle[0] = axisAngle[1] = axisAngle[2] = 0.0;
        }

        axisAngle.push_back(rotAngle);

        yarp::sig::Matrix H = yarp::math::axis2dcm(axisAngle);

        if (fillTransl)
        {
            for (int i = 0; i < 3; i++)
            {
                H(i, 3) = v[i];
            }
        }

        return H;
    }

    void matrixToVector(const yarp::sig::Matrix &H, std::vector<double> &v, bool fillTransl)
    {
        yarp::sig::Vector axisAngle = yarp::math::dcm2axis(H);
        yarp::sig::Vector axisAngleScaled = axisAngle.subVector(0, 2) * axisAngle[3];

        v.resize(6);

        for (int i = 0; i < 3; i++)
        {
            v[i + 3] = axisAngleScaled[i];
        }

        if (fillTransl)
        {
            for (int i = 0; i < 3; i++)
            {
                v[i] = H(i, 3);
            }
        }
    }

    void computeBaseFrameDiffInvKin(double m_A1, double m_A2, double m_A3, const std::vector<double> & q, yarp::sig::Matrix & Ja)
    {
        double s1 = std::sin(q[0]);
        double c1 = std::cos(q[0]);
        double s2 = std::sin(q[1]);
        double c2 = std::cos(q[1]);

        double s23 = std::sin(q[1] + q[2]);
        double c23 = std::cos(q[1] + q[2]);

        double s234 = std::sin(q[1] + q[2] + q[3]);
        double c234 = std::cos(q[1] + q[2] + q[3]);

        Ja(0, 0) = -s1 * (m_A3 * s234 + m_A2 * s23 + m_A1 * s2);
        Ja(0, 1) =  c1 * (m_A3 * c234 + m_A2 * c23 + m_A1 * c2);
        Ja(0, 2) =  c1 * (m_A3 * c234 + m_A2 * c23);
        Ja(0, 3) =  c1 * m_A3 * c234;
        Ja(0, 4) =  0;

        Ja(1, 0) = c1 * (m_A3 * s234 + m_A2 * s23 + m_A1 * s2);
        Ja(1, 1) = s1 * (m_A3 * c234 + m_A2 * c23 + m_A1 * c2);
        Ja(1, 2) = s1 * (m_A3 * c234 + m_A2 * c23);
        Ja(1, 3) = s1 * m_A3 * c234;
        Ja(1, 4) = 0;

        Ja(2, 0) = 0;
        Ja(2, 1) = -m_A3 * s234 - m_A2 * s23 - m_A1 * s2;
        Ja(2, 2) = -m_A3 * s234 - m_A2 * s23;
        Ja(2, 3) = -m_A3 * s234;
        Ja(2, 4) = 0;

        Ja(3, 0) = 0;
        Ja(3, 1) = -s1;
        Ja(3, 2) = -s1;
        Ja(3, 3) = -s1;
        Ja(3, 4) = c1 * s234;

        Ja(4, 0) = 0;
        Ja(4, 1) = c1;
        Ja(4, 2) = c1;
        Ja(4, 3) = c1;
        Ja(4, 4) = s1 * s234;

        Ja(5, 0) = 1;
        Ja(5, 1) = 0;
        Ja(5, 2) = 0;
        Ja(5, 3) = 0;
        Ja(5, 4) = c234;
    }

    void computeTcpFrameDiffInvKin(double m_A1, double m_A2, double m_A3, const std::vector<double> & q, yarp::sig::Matrix & Ja)
    {
        double s2 = std::sin(q[1]);
        double c2 = std::cos(q[1]);
        double s4 = std::sin(q[3]);
        double c4 = std::cos(q[3]);
        double s5 = std::sin(q[4]);
        double c5 = std::cos(q[4]);

        double s23 = std::sin(q[1] + q[2]);
        double c23 = std::cos(q[1] + q[2]);

        double s34 = std::sin(q[2] + q[3]);
        double c34 = std::cos(q[2] + q[3]);

        double s234 = std::sin(q[1] + q[2] + q[3]);
        double c234 = std::cos(q[1] + q[2] + q[3]);

        Ja(0, 0) = s5 * (m_A3 * s234 + m_A2 * s23 + m_A1 * s2);
        Ja(0, 1) = c5 * (m_A3 + m_A2 * c4 + m_A1 * c34);
        Ja(0, 2) = c5 * (m_A3 + m_A2 * c4);
        Ja(0, 3) = c5 * m_A3;
        Ja(0, 4) = 0;

        Ja(1, 0) = c5 * (m_A3 * s234 + m_A2 * s23 + m_A1 * s2);
        Ja(1, 1) = -s5 * (m_A3 + m_A2 * c4 + m_A1 * c34);
        Ja(1, 2) = -s5 * (m_A3 + m_A2 * c4);
        Ja(1, 3) = -s5 * m_A3;
        Ja(1, 4) = 0;

        Ja(2, 0) = 0;
        Ja(2, 1) = m_A2 * s4 + m_A1 * s34;
        Ja(2, 2) = m_A2 * s4;
        Ja(2, 3) = 0;
        Ja(2, 4) = 0;

        Ja(3, 0) = -s234 * c5;
        Ja(3, 1) = s5;
        Ja(3, 2) = s5;
        Ja(3, 3) = s5;
        Ja(3, 4) = 0;

        Ja(4, 0) = s234 * s5;
        Ja(4, 1) = c5;
        Ja(4, 2) = c5;
        Ja(4, 3) = c5;
        Ja(4, 4) = 0;

        Ja(5, 0) = c234;
        Ja(5, 1) = 0;
        Ja(5, 2) = 0;
        Ja(5, 3) = 0;
        Ja(5, 4) = 1;
    }
}

// -----------------------------------------------------------------------------

int AsibotSolver::getNumJoints()
{
    return 5;
}

// -----------------------------------------------------------------------------

int AsibotSolver::getNumTcps()
{
    return 1;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::appendLink(const std::vector<double> &x)
{
    yarp::sig::Matrix newFrame = vectorToMatrix(x, true);

    AsibotTcpFrame tcpFrameStruct = getTcpFrame();
    tcpFrameStruct.hasFrame = true;
    tcpFrameStruct.frameTcp *= newFrame;
    setTcpFrame(tcpFrameStruct);

    return true;
}

// --------------------------------------------------------------------------

bool AsibotSolver::restoreOriginalChain()
{
    AsibotTcpFrame tcpFrameStruct = getTcpFrame();
    tcpFrameStruct.hasFrame = false;
    tcpFrameStruct.frameTcp = yarp::math::eye(4);
    setTcpFrame(tcpFrameStruct);
    return true;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::changeOrigin(const std::vector<double> &x_old_obj, const std::vector<double> &x_new_old, std::vector<double> &x_new_obj)
{
    yarp::sig::Matrix H_old_obj = vectorToMatrix(x_old_obj, true);
    yarp::sig::Matrix H_new_old = vectorToMatrix(x_new_old, true);
    yarp::sig::Matrix H_new_obj = H_new_old * H_old_obj;

    matrixToVector(H_new_obj, x_new_obj, true);

    return true;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x)
{
    std::vector<double> qInRad(q);

    for (auto & val : qInRad)
    {
        val = degToRad(val);
    }

    double s1 = std::sin(qInRad[0]);
    double c1 = std::cos(qInRad[0]);
    double s2 = std::sin(qInRad[1]);
    double c2 = std::cos(qInRad[1]);

    double s23 = std::sin(qInRad[1] + qInRad[2]);
    double c23 = std::cos(qInRad[1] + qInRad[2]);

    double s234 = std::sin(qInRad[1] + qInRad[2] + qInRad[3]);
    double c234 = std::cos(qInRad[1] + qInRad[2] + qInRad[3]);

    // P = prime
    double prP = m_A1 * s2 + m_A2 * s23 + m_A3 * s234;
    double phP = m_A1 * c2 + m_A2 * c23 + m_A3 * c234;
    double oyP = q[1] + q[2] + q[3];  // [deg]

    x.resize(5);  // translation[3] + rotation[2]

    x[0] = prP * c1;
    x[1] = prP * s1;
    x[2] = phP + m_A0;  // pz = pzP
    x[3] = oyP;  // = pitchP
    x[4] = q[4];  // = ozPP

    encodePose(x, x, coordinate_system::CARTESIAN, orientation_system::EULER_YZ, angular_units::DEGREES);

    const AsibotTcpFrame & tcpFrameStruct = getTcpFrame();

    if (tcpFrameStruct.hasFrame)
    {
        yarp::sig::Matrix H_base_tcp = vectorToMatrix(x, true) * tcpFrameStruct.frameTcp;
        matrixToVector(H_base_tcp, x, true);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::poseDiff(const std::vector<double> &xLhs, const std::vector<double> &xRhs, std::vector<double> &xOut)
{
    xOut.resize(6);

    xOut[0] = xLhs[0] - xRhs[0];
    xOut[1] = xLhs[1] - xRhs[1];
    xOut[2] = xLhs[2] - xRhs[2];

    yarp::sig::Matrix rotLhs = vectorToMatrix(xLhs, true).submatrix(0, 2, 0, 2);
    yarp::sig::Matrix rotRhs = vectorToMatrix(xRhs, true).submatrix(0, 2, 0, 2);

    yarp::sig::Matrix rotRhsToLhs = rotRhs.transposed() * rotLhs;

    yarp::sig::Vector axisAngle = yarp::math::dcm2axis(rotRhsToLhs);
    yarp::sig::Vector axis = axisAngle.subVector(0, 2) * axisAngle[3];
    yarp::sig::Vector rotd = rotRhs * axis;

    xOut[3] = rotd[0];
    xOut[4] = rotd[1];
    xOut[5] = rotd[2];

    return true;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::invKin(const std::vector<double> &xd, const std::vector<double> &qGuess, std::vector<double> &q, reference_frame frame)
{
    std::vector<double> xd_base_obj;

    if (frame == TCP_FRAME)
    {
        std::vector<double> x_base_tcp;
        fwdKin(qGuess, x_base_tcp);
        changeOrigin(xd, x_base_tcp, xd_base_obj);
    }
    else if (frame == BASE_FRAME)
    {
        xd_base_obj = xd;
    }
    else
    {
        yCError(ASIBOT) << "Unsupported reference frame";
        return false;
    }

    const AsibotTcpFrame & tcpFrameStruct = getTcpFrame();

    if (tcpFrameStruct.hasFrame)
    {
        yarp::sig::Matrix H_0_N = vectorToMatrix(xd_base_obj, true) * yarp::math::luinv(tcpFrameStruct.frameTcp);
        matrixToVector(H_0_N, xd_base_obj, true);
    }

    std::vector<double> xd_eYZ;

    if (!decodePose(xd_base_obj, xd_eYZ, coordinate_system::CARTESIAN, orientation_system::EULER_YZ))
    {
        yCError(ASIBOT) << "Unable to convert to eulerYZ angle representation";
        return false;
    }

    double ozdRad = std::atan2(xd_eYZ[1], xd_eYZ[0]);

    double prPd = std::sqrt(xd_eYZ[0] * xd_eYZ[0] + xd_eYZ[1] * xd_eYZ[1]);
    double phPd = xd_eYZ[2] - m_A0;
    double oyPdRad = xd_eYZ[3];

    if (std::sqrt(prPd * prPd + phPd * phPd) > m_A1 + m_A2 + m_A3)
    {
        yCError(ASIBOT) << "Target out of reach";
        return false;
    }

    double prWd = prPd - m_A3 * std::sin(oyPdRad);
    double phWd = phPd - m_A3 * std::cos(oyPdRad);

    double len_2 = phWd * phWd + prWd * prWd;

    if (std::sqrt(len_2) > m_A1 + m_A2)
    {
        yCError(ASIBOT) << "Target out of reach";
        return false;
    }

    double ct2 = (len_2 - m_A1 * m_A1 - m_A2 * m_A2) / (2 * m_A1 * m_A2);
    double st2 = std::sqrt(1 - ct2 * ct2);  // forces elbow-up in ASIBOT
    //double st2 = -std::sqrt(1 - ct2 * ct2);  // forces elbow-down in ASIBOT

    double t2Rad = std::atan2(st2, ct2);
    //double t2Rad = std::atan2(-st2, ct2);  // or just '-std::atan(st2, ct2)' (elbow-down)

    // 'u': elbow-up; 'd': elbow-down
    double st1u = ((m_A1 + m_A2 * ct2) * prWd - m_A2 * st2 * phWd) / len_2;
    double ct1u = ((m_A1 + m_A2 * ct2) * phWd + m_A2 * st2 * prWd) / len_2;
    // double ct1u = (phWd + m_A2 * st1 * st2) / (m_A1 + m_A2 * ct2);  // alternative method for same result

    double st1d = ((m_A1 + m_A2 * ct2) * prWd + m_A2 * st2 * phWd) / len_2;
    double ct1d = ((m_A1 + m_A2 * ct2) * phWd - m_A2 * st2 * prWd) / len_2;

    double t1uRad = std::atan2(st1u, ct1u);
    double t1dRad = std::atan2(st1d, ct1d);

    double t3uRad = oyPdRad - t1uRad - t2Rad;
    double t3dRad = oyPdRad - t1dRad + t2Rad;

    std::unique_ptr<AsibotConfiguration> conf(getJointConfiguration());

    if (!conf->configure(radToDeg(ozdRad),
            radToDeg(t1uRad),
            radToDeg(t1dRad),
            radToDeg(t2Rad),
            radToDeg(t3uRad),
            radToDeg(t3dRad),
            radToDeg(xd_eYZ[4])))
    {
        yCError(ASIBOT) << "Unable to find a valid configuration within joint limits";
        return false;
    }

    if (!conf->findOptimalConfiguration(qGuess))
    {
        yCError(ASIBOT) << "findOptimalConfiguration() failed";
        return false;
    }

    conf->retrieveAngles(q);

    return true;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::diffInvKin(const std::vector<double> &q, const std::vector<double> &xdot, std::vector<double> &qdot, reference_frame frame)
{
    std::vector<double> qInRad(q);

    for (auto & val : qInRad)
    {
        val = degToRad(val);
    }

    yarp::sig::Matrix Ja(6, 5);

    if (frame == BASE_FRAME)
    {
        computeBaseFrameDiffInvKin(m_A1, m_A2, m_A3, qInRad, Ja);
    }
    else if (frame == TCP_FRAME)
    {
        computeTcpFrameDiffInvKin(m_A1, m_A2, m_A3, qInRad, Ja);
    }
    else
    {
        yCWarning(ASIBOT) << "Unsupported frame";
        return false;
    }

    const AsibotTcpFrame & tcpFrameStruct = getTcpFrame();

    if (tcpFrameStruct.hasFrame)
    {
        std::vector<double> x;
        fwdKin(q, x);

        yarp::sig::Matrix R_0_N = vectorToMatrix(x, true).submatrix(0, 2, 0, 2);
        yarp::sig::Vector transl = tcpFrameStruct.frameTcp.subcol(0, 3, 3);
        yarp::sig::Matrix skewSM = yarp::math::crossProductMatrix(transl);
        yarp::sig::Matrix similTransform = (-1) * R_0_N * skewSM * R_0_N.transposed();

        yarp::sig::Matrix S = yarp::math::eye(6);
        S.setSubmatrix(similTransform, 0, 3);

        Ja = S * Ja;
    }

    yarp::sig::Matrix Ja_inv = yarp::math::pinv(Ja, 1e-2);

    yarp::sig::Vector xdotv(6);

    for (unsigned int i = 0; i < xdot.size(); i++)
    {
        xdotv[i] = xdot[i];
    }

    yarp::sig::Vector qdotv = Ja_inv * xdotv;

    qdot.resize(5);

    for (unsigned int i = 0; i < qdot.size(); i++)
    {
        qdot[i] = radToDeg(qdotv[i]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::invDyn(const std::vector<double> &q, std::vector<double> &t)
{
    yCWarning(ASIBOT) << "invDyn() not implemented";
    return false;
}

// -----------------------------------------------------------------------------

bool AsibotSolver::invDyn(const std::vector<double> &q, const std::vector<double> &qdot, const std::vector<double> &qdotdot,
                          const std::vector<double> &ftip, std::vector<double> &t, reference_frame frame)
{
    yCWarning(ASIBOT) << "invDyn() not implemented";
    return false;
}

// -----------------------------------------------------------------------------
