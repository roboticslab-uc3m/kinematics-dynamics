// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJ_GEN_HPP__
#define __TRAJ_GEN_HPP__

#include <ColorDebug.h>

/**
 * @ingroup kinematics-dynamics-libraries
 *
 * @defgroup TrajGen
 *
 * @brief The TrajGen library is a collection of classes for trajectory generation.
 *
 * The TrajGen library is a collection of classes for trajectory generation. For now the functions'
 * implementations are so short that they have been programmed inline, so for now the library is actually
 * header-based (no actual real static/dynamic library to be linked).
 */

/**
 * @ingroup TrajGen
 *
 * @brief A base class for 1 degree-of-freedom trajectories.
 */
class Traj
{
public:
    virtual ~Traj() {}
    virtual bool configure(const double xi, const double xf, const double _T) = 0;
    virtual bool configure(const double xi, const double xf, const double xdoti, const double xdotf, const double _T) = 0;
    virtual double get(const double ti) const = 0;
    virtual double getdot(const double ti) const = 0;
    virtual double getdotdot(const double ti) const = 0;
    virtual bool maxVelBelow(const double thresVel) const = 0;
    virtual bool maxAccBelow(const double thresAcc) const = 0;
    virtual double getT() const = 0;
    virtual void dump(double samples) const = 0;
};

/**
 * @ingroup TrajGen
 *
 * @brief Generates a 1DOF order-one trajectory.
 */
class OrderOneTraj : public Traj
{
public:
    OrderOneTraj() : T(0.0), m(0.0), b(0.0) {}

    bool configure(const double xi, const double xf, const double _T)
    {
        T = _T;
        b = xi;
        m = (xf - xi) / T;
        return true;
    }

    bool configure(const double xi, const double xf, const double xdoti, const double xdotf, const double _T)
    {
        return false;
    }

    double get(const double ti) const
    {
        return b + (m * ti);
    }

    double getdot(const double ti) const
    {
        return m;
    }

    double getdotdot(const double ti) const
    {
        return 0;
    }

    bool maxVelBelow(const double thresVel) const
    {
        return m < thresVel;
    }

    bool maxAccBelow(const double thresAcc) const
    {
        return 0 < thresAcc;
    }

    double getT() const
    {
        return T;
    }

    void dump(double samples) const
    {
        for (double i = 0; i < T; i += (T / samples))
        {
            CD_INFO("%05.2f %+02.6f %+02.6f %+02.6f\n", i, get(i), getdot(i), getdotdot(i));
        }
    }

private:
    double T, m, b;
};

/**
 * @ingroup TrajGen
 *
 * @brief Generates a 1DOF order-three trajectory.
 */
class OrderThreeTraj : public Traj
{
public:
    OrderThreeTraj() : a3(0.0), a2(0.0), a1(0.0), a0(0.0), T(0.0) {}

    /**
     * Configure the trajectory. Forces null initial and final velocities.
     */
    bool configure(const double xi, const double xf, const double _T)
    {
        T = _T;
        a0 = xi;
        a1 = 0;
        a3 = 2 * (xi - xf) / (T * T * T);
        a2 = (xf - xi) / (T * T) - a3 * T;
        return true;
    }

    /**
     * Configure the trajectory setting an initial and final velocity too (warning: possible overshoot).
     */
    bool configure(const double xi, const double xf, const double xdoti, const double xdotf, const double _T)
    {
        T = _T;
        a0 = xi;
        a1 = xdoti;
        a3 = 2.0 * (xi - xf) / (T * T * T) + (xdotf + xdoti) / (T * T);
        a2 = (xf - xi - xdoti * T) / (T * T) - a3 * T;
        return true;
    }

    /**
     * @return the value of the function at instant ti.
     */
    double get(const double ti) const
    {
        if (ti > T)
        {
            return a3 * T * T * T + a2 * T * T + a1 * T + a0;  // Security hack
        }
        return a3 * ti * ti * ti + a2 * ti * ti + a1 * ti + a0;
    }

    /**
     * @return the value of the first derivative of the function at instant ti.
     */
    double getdot(const double ti) const
    {
        if (ti > T)
        {
            return 3 * a3 * T * T + 2 * a2 * T + a1;  // Security hack
        }
        return 3 * a3 * ti * ti + 2 * a2 * ti + a1;
    }

    /**
     * @return the value of the second derivative of the function at instant ti.
     */
    double getdotdot(const double ti) const
    {
        if (ti > T)
        {
            return 6 * a3 * T + 2 * a2;  // Security hack
        }
        return 6 * a3 * ti + 2 * a2;
    }

    /**
     * Check if the maximum of the first derivative is below a certain threshold.
     */
    bool maxVelBelow(const double thresVel) const
    {
        return getdot(T / 2) < thresVel;
    }

    /**
     * Check if the maximum of the second derivative is below a certain threshold.
     */
    bool maxAccBelow(const double thresAcc) const
    {
        return getdotdot(T / 2) < thresAcc;
    }

    /**
     * @return duration assigned to the trajectory instance.
     */
    double getT() const
    {
        return T;
    }

    /**
     * Printf of ti, f(ti), fdot(ti), fdotdot(ti) for whole duration interval.
     * @param samples number of lines to print.
     */
    void dump(double samples) const
    {
        for (double i = 0; i < T; i += (T / samples))
        {
            CD_INFO("%05.2f %+02.6f %+02.6f %+02.6f\n", i, get(i), getdot(i), getdotdot(i));
        }
    }

private:
    double a3, a2, a1, a0, T;
};

#endif  // __TRAJ_GEN_HPP__

