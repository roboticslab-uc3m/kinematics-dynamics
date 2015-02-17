// -----------------------------------------------------------------------------

#include "KdlSolver.hpp"

// -----------------------------------------------------------------------------

bool teo::KdlSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x, std::vector<double> &o) {

    JntArray inRad = JntArray(numLinks);
    Frame fOutCart;
    for (int motor=0; motor<numLinks; motor++) {
        inRad(motor)=toRad(q[motor]);
    }
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(theChain);
    fksolver.JntToCart(inRad,fOutCart);

    x.resize(3);
    x[0] = fOutCart.p.data[0];
    x[1] = fOutCart.p.data[1];
    x[2] = fOutCart.p.data[2];

    if (angleRepr == "axisAngle") {
        KDL::Vector rotVector = fOutCart.M.GetRot();
        o.resize(4);
        o[3] = fOutCart.M.GetRotAngle(rotVector);  // Normalizes as colateral effect
        o[0] = rotVector[0];
        o[1] = rotVector[1];
        o[2] = rotVector[2];
        CD_INFO("KDL computed cart: %f %f %f | %f %f %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1],o[2],o[3]);
    } else if (angleRepr == "eulerYZ") {  // ASIBOT
        double alfa, beta, gamma;
        fOutCart.M.GetEulerZYZ(alfa, beta, gamma);
        o.clear();
        o.push_back(toDeg(beta));  // pushed on as [0]
        o.push_back(toDeg(gamma));  // pushed on as [1]
        CD_INFO("KDL computed cart: %f %f %f | %f %f.\n",
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
        CD_INFO("KDL computed cart: %f %f %f | %f %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1],o[2]);
    } else {
        //-- No known angle repr.
        CD_INFO("KDL computed cart: %f %f %f\n",fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2]);
    }

    return true;
}

// -----------------------------------------------------------------------------
