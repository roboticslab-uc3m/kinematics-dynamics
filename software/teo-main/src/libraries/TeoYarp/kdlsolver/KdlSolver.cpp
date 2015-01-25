// -----------------------------------------------------------------------------

#include "KdlSolver.hpp"

// -----------------------------------------------------------------------------

bool teo::KdlSolver::fwdKin(const yarp::sig::Vector &inUnits, yarp::sig::Vector &x, yarp::sig::Vector &o) {

    JntArray inRad = JntArray(numLinks);
    Frame fOutCart;
    for (int motor=0; motor<numLinks; motor++) {
        inRad(motor)=toRad(inUnits[motor]);
    }
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(theChain);
    fksolver.JntToCart(inRad,fOutCart);

    x.clear();
    x.push_back(fOutCart.p.data[0]);  // pushed on as [0]
    x.push_back(fOutCart.p.data[1]);  // pushed on as [1]
    x.push_back(fOutCart.p.data[2]);  // pushed on as [2]

    if (angleRepr == "axisAngle") {
        o.resize(4);
        KDL::Vector rotVector = fOutCart.M.GetRot();
        o[3] = fOutCart.M.GetRotAngle(rotVector);  // Normalizes as colateral effect
        o[0] = rotVector[0];
        o[1] = rotVector[1];
        o[2] = rotVector[2];
        CD_INFO("KDL computed current cart: %f %f %f | %f %f %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1],o[2],o[3]);
    } else if (angleRepr == "eulerYZ") {  // ASIBOT
        double alfa, beta, gamma;
        fOutCart.M.GetEulerZYZ(alfa, beta, gamma);
        o.clear();
        o.push_back(toDeg(beta));  // pushed on as [0]
        o.push_back(toDeg(gamma));  // pushed on as [1]
        CD_INFO("KDL computed current cart: %f %f %f | %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1]);
    } else if (angleRepr == "eulerZYZ") {
        double alfa, beta, gamma;
        fOutCart.M.GetEulerZYZ(alfa, beta, gamma);
        o.clear();
        o.push_back(toDeg(alfa));  // pushed on as [0]
        o.push_back(toDeg(beta));  // pushed on as [1]
        o.push_back(toDeg(gamma));  // pushed on as [2]
        CD_INFO("KDL computed current cart: %f %f %f | %f %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1],o[2]);
    } else if (angleRepr == "RPY") {
        double alfa, beta, gamma;
        fOutCart.M.GetRPY(alfa, beta, gamma);
        o.clear();
        o.push_back(toDeg(alfa));  // pushed on as [0]
        o.push_back(toDeg(beta));  // pushed on as [1]
        o.push_back(toDeg(gamma));  // pushed on as [2]
        CD_INFO("KDL computed current cart: %f %f %f | %f %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1],o[2]);
    } else {
        CD_INFO("KDL computed current cart: %f %f %f\n",fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2]);
    }

    return true;
}

// -----------------------------------------------------------------------------
