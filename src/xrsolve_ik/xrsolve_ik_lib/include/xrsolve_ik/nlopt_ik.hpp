#ifndef NLOPT_IK_HPP
#define NLOPT_IK_HPP

#include <xrsolve_ik/kdl_tl.hpp>
#include <nlopt.hpp>


namespace NLOPT_IK
{

enum OptType { Joint, DualQuat, SumSq, L2 };


class NLOPT_IK
{
  friend class XRSOLVE_IK::XRSOLVE_IK;
public:
  NLOPT_IK(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, double maxtime = 0.005, double eps = 1e-3, OptType type = SumSq);

  ~NLOPT_IK() {};
  int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const KDL::Twist bounds = KDL::Twist::Zero(), const KDL::JntArray& q_desired = KDL::JntArray());

  double minJoints(const std::vector<double>& x, std::vector<double>& grad);
  //  void cartFourPointError(const std::vector<double>& x, double error[]);
  void cartSumSquaredError(const std::vector<double>& x, double error[]);
  void cartDQError(const std::vector<double>& x, double error[]);
  void cartL2NormError(const std::vector<double>& x, double error[]);

  inline void setMaxtime(double t)
  {
    maxtime = t;
  }

private:

  inline void abort()
  {
    aborted = true;
  }

  inline void reset()
  {
    aborted = false;
  }


  std::vector<double> lb;
  std::vector<double> ub;

  const KDL::Chain chain;
  std::vector<double> des;


  KDL::ChainFkSolverPos_recursive fksolver;

  double maxtime;
  double eps;
  int iter_counter;
  OptType TYPE;

  KDL::Frame targetPose;
  KDL::Frame z_up ;
  KDL::Frame x_out;
  KDL::Frame y_out;
  KDL::Frame z_target;
  KDL::Frame x_target;
  KDL::Frame y_target;

  std::vector<KDL::BasicJointType> types;

  nlopt::opt opt;

  KDL::Frame currentPose;

  std::vector<double> best_x;
  int progress;
  bool aborted;

  KDL::Twist bounds;

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }


};

}

#endif
