// -----------------------------------------------------------------------------

#include "KdlSolver.hpp"

// -----------------------------------------------------------------------------

bool teo::KdlSolver::fwdKin(const std::vector<double> &q, std::vector<double> &x, std::vector<double> &o) {

    JntArray qInRad = JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qInRad(motor)=toRad(q[motor]);

    //-- Main fwdKin (pos) solver lines
    Frame fOutCart;
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    fksolver.JntToCart(qInRad,fOutCart);

    x.resize(3);
    x[0] = fOutCart.p.data[0];
    x[1] = fOutCart.p.data[1];
    x[2] = fOutCart.p.data[2];

    if (angleRepr == "axisAngle")
    {
        KDL::Vector rotVector = fOutCart.M.GetRot();
        o.resize(4);
        o[3] = fOutCart.M.GetRotAngle(rotVector);  // Normalizes as colateral effect
        o[0] = rotVector[0];
        o[1] = rotVector[1];
        o[2] = rotVector[2];
        CD_INFO("KDL computed cart: %f %f %f | %f %f %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1],o[2],o[3]);
    }
    else if (angleRepr == "eulerYZ") //-- like ASIBOT
    {
        double alfa, beta, gamma;
        fOutCart.M.GetEulerZYZ(alfa, beta, gamma);
        o.resize(2);
        o[0] = toDeg(beta);
        o[1] = toDeg(gamma);
        CD_INFO("KDL computed cart: %f %f %f | %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1]);
    }
    else if (angleRepr == "eulerZYZ")
    {
        double alfa, beta, gamma;
        fOutCart.M.GetEulerZYZ(alfa, beta, gamma);
        o.resize(3);
        o[0] = toDeg(alfa);
        o[1] = toDeg(beta);
        o[2] = toDeg(gamma);
        CD_INFO("KDL computed current cart: %f %f %f | %f %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1],o[2]);
    }
    else if (angleRepr == "RPY")
    {
        double alfa, beta, gamma;
        fOutCart.M.GetRPY(alfa, beta, gamma);
        o.resize(3);
        o[0] = toDeg(alfa);
        o[1] = toDeg(beta);
        o[2] = toDeg(gamma);
        CD_INFO("KDL computed cart: %f %f %f | %f %f %f.\n",
            fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2],o[0],o[1],o[2]);
    }
    else  //-- No known angle repr.
    {
        CD_INFO("KDL computed cart: %f %f %f\n",fOutCart.p.data[0],fOutCart.p.data[1],fOutCart.p.data[2]);
    }

    return true;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::invKin(const std::vector<double> &xd, const std::vector<double> &od, const std::vector<double> &qGuess, std::vector<double> &q) {

    Frame frameXd;
    frameXd.p.data[0]=xd[0];
    frameXd.p.data[1]=xd[1];
    frameXd.p.data[2]=xd[2];

    if (angleRepr == "axisAngle") {
        frameXd.M = Rotation::Rot(Vector(od[0],od[1],od[2]),od[3]);
    }
    else if (angleRepr == "eulerYZ")  //-- like ASIBOT
    {
        frameXd.M = Rotation::EulerZYZ(::atan2(xd[1],xd[0]),toRad(od[0]), toRad(od[1]));
    }
    else if (angleRepr == "eulerZYZ")
    {
        frameXd.M = Rotation::EulerZYZ(toRad(od[0]), toRad(od[1]), toRad(od[2]));
    }
    else if (angleRepr == "RPY")
    {
        frameXd.M = Rotation::RPY(toRad(od[0]), toRad(od[1]), toRad(od[2]));
    }
    else  //-- No known angle repr.
    {
        CD_WARNING("Not compatible angleRepr: %s\n",angleRepr.c_str());
    }

    Eigen::Matrix<double,6,1> L;
    L(0)=1;L(1)=1;L(2)=1;
    L(3)=0;L(4)=0;L(5)=0;

    JntArray qGuessInRad = JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qGuessInRad(motor)=toRad(qGuess[motor]);

    //-- Main invKin (pos) solver lines
    ChainIkSolverPos_LMA iksolver_pos(chain,L);
    JntArray kdlq = JntArray(numLinks);
    int ret = iksolver_pos.CartToJnt(qGuessInRad,frameXd,kdlq);

    q.resize(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        q[motor]=toDeg(kdlq(motor));

    if (ret == 0) return true;

    if (ret == -1)
    {
        CD_WARNING("The gradient of E towards the joints is to small.\n");
    }
    else if (ret == -2)
    {
        CD_WARNING("The joint position increments are to small.\n");
    }
    else if (ret == -3)
    {
        CD_WARNING("The number of iterations is exceeded.\n");
    }
    return false;
}

// -----------------------------------------------------------------------------

bool teo::KdlSolver::invDyn(const std::vector<double> &q,const std::vector<double> &qdot,const std::vector<double> &qdotdot, const std::vector< std::vector<double> > &fexts, std::vector<double> &t) {

    JntArray qInRad = JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qInRad(motor)=toRad(q[motor]);

    JntArray qdotInRad = JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qdotInRad(motor)=toRad(qdot[motor]);

    JntArray qdotdotInRad = JntArray(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        qdotdotInRad(motor)=toRad(qdotdot[motor]);

    Wrenches wrenches;
    for (int i=0; i<numLinks; i++)
    {
        Wrench wrench( Vector(fexts[i][0],fexts[i][1],fexts[i][2]), Vector(fexts[i][3],fexts[i][4],fexts[i][5]) );
        wrenches.push_back(wrench);
    }

    CD_DEBUG("mass: %f\n",chain.getSegment(0).getInertia().getMass());
    CD_DEBUG("cogX: %f\n",chain.getSegment(0).getInertia().getCOG().data[0]);
    CD_DEBUG("cogX: %f\n",chain.getSegment(0).getInertia().getCOG().data[1]);
    CD_DEBUG("cogX: %f\n",chain.getSegment(0).getInertia().getCOG().data[2]);
    CD_DEBUG("inertiaXX: %f\n",chain.getSegment(0).getInertia().getRotationalInertia().data[0]);
    CD_DEBUG("inertiaYY: %f\n",chain.getSegment(0).getInertia().getRotationalInertia().data[4]);
    CD_DEBUG("inertiaZZ: %f\n",chain.getSegment(0).getInertia().getRotationalInertia().data[8]);

    JntArray kdlt = JntArray(numLinks);

    //-- Main invDyn solver lines
    ChainIdSolver_RNE idsolver(chain,Vector(0.0,0.0,-9.81));
    qInRad(0) = 0.0;
    qdotInRad(0) = 0.0;
    qdotdotInRad(0) = 0.0000000000000001;
    int ret = idsolver.CartToJnt(qInRad,qdotInRad,qdotdotInRad,wrenches,kdlt);

//    qInRad(0) = 0.1;
//    ChainDynParam chainDynParam(chain,Vector(0.0,0.0,-9.81));
//    int ret = chainDynParam.JntToGravity(qInRad,kdlt);
    CD_DEBUG("kdlt0: %f\n",kdlt(0));

    t.resize(numLinks);
    for (int motor=0; motor<numLinks; motor++)
        t[motor]=kdlt(motor);

    if (ret == 0) return true;

    CD_WARNING("Something went wrong.\n");
    return false;
}

// -----------------------------------------------------------------------------
