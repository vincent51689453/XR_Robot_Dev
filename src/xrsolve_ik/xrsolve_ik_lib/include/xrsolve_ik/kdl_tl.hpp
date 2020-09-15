#ifndef KDLCHAINIKSOLVERPOS_TL_HPP
#define KDLCHAINIKSOLVERPOS_TL_HPP

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

namespace XRSOLVE_IK
{
class XRSOLVE_IK;
}

namespace KDL
{

enum BasicJointType { RotJoint, TransJoint, Continuous };

class ChainIkSolverPos_TL
{
  friend class XRSOLVE_IK::XRSOLVE_IK;

public:
  ChainIkSolverPos_TL(const Chain& chain, const JntArray& q_min, const JntArray& q_max, double maxtime = 0.005, double eps = 1e-3, bool random_restart = false, bool try_jl_wrap = false);

  ~ChainIkSolverPos_TL();

  int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const KDL::Twist bounds = KDL::Twist::Zero());

  inline void setMaxtime(double t)
  {
    maxtime = t;
  }

private:
  const Chain chain;
  JntArray q_min;
  JntArray q_max;

  KDL::Twist bounds;

  KDL::ChainIkSolverVel_pinv vik_solver;
  KDL::ChainFkSolverPos_recursive fksolver;
  JntArray delta_q;
  double maxtime;

  double eps;

  bool rr;
  bool wrap;

  std::vector<KDL::BasicJointType> types;

  inline void abort()
  {
    aborted = true;
  }

  inline void reset()
  {
    aborted = false;
  }

  bool aborted;

  Frame f;
  Twist delta_twist;

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }


};


IMETHOD Twist diffRelative(const Frame & F_a_b1, const Frame & F_a_b2, double dt = 1)
{
  return Twist(F_a_b1.M.Inverse() * diff(F_a_b1.p, F_a_b2.p, dt),
               F_a_b1.M.Inverse() * diff(F_a_b1.M, F_a_b2.M, dt));
}

}

#endif
