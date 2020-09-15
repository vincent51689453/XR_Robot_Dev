#ifndef XRSOLVE_IK_HPP
#define XRSOLVE_IK_HPP

#include <xrsolve_ik/nlopt_ik.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <thread>
#include <mutex>
#include <memory>
#include <boost/date_time.hpp>

namespace XRSOLVE_IK
{

enum SolveType { Speed, Distance, Manip1, Manip2 };

class XRSOLVE_IK
{
public:
  XRSOLVE_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime = 0.005, double _eps = 1e-5, SolveType _type = Speed);

  XRSOLVE_IK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param = "/robot_description", double _maxtime = 0.005, double _eps = 1e-5, SolveType _type = Speed);

  ~XRSOLVE_IK();

  bool getKDLChain(KDL::Chain& chain_)
  {
    chain_ = chain;
    return initialized;
  }

  bool getKDLLimits(KDL::JntArray& lb_, KDL::JntArray& ub_)
  {
    lb_ = lb;
    ub_ = ub;
    return initialized;
  }

  // Requires a previous call to CartToJnt()
  bool getSolutions(std::vector<KDL::JntArray>& solutions_)
  {
    solutions_ = solutions;
    return initialized && !solutions.empty();
  }

  bool getSolutions(std::vector<KDL::JntArray>& solutions_, std::vector<std::pair<double, uint> >& errors_)
  {
    errors_ = errors;
    return getSolutions(solutions);
  }

  bool setKDLLimits(KDL::JntArray& lb_, KDL::JntArray& ub_)
  {
    lb = lb_;
    ub = ub_;
    nl_solver.reset(new NLOPT_IK::NLOPT_IK(chain, lb, ub, maxtime, eps, NLOPT_IK::SumSq));
    iksolver.reset(new KDL::ChainIkSolverPos_TL(chain, lb, ub, maxtime, eps, true, true));
    return true;
  }

  static double JointErr(const KDL::JntArray& arr1, const KDL::JntArray& arr2)
  {
    double err = 0;
    for (uint i = 0; i < arr1.data.size(); i++)
    {
      err += pow(arr1(i) - arr2(i), 2);
    }

    return err;
  }

  int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds = KDL::Twist::Zero());

  inline void SetSolveType(SolveType _type)
  {
    solvetype = _type;
  }

private:
  bool initialized;
  KDL::Chain chain;
  KDL::JntArray lb, ub;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver;
  double eps;
  double maxtime;
  SolveType solvetype;

  std::unique_ptr<NLOPT_IK::NLOPT_IK> nl_solver;
  std::unique_ptr<KDL::ChainIkSolverPos_TL> iksolver;

  boost::posix_time::ptime start_time;

  template<typename T1, typename T2>
  bool runSolver(T1& solver, T2& other_solver,
                 const KDL::JntArray &q_init,
                 const KDL::Frame &p_in);

  bool runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in);
  bool runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in);

  void normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution);
  void normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution);

  std::vector<KDL::BasicJointType> types;

  std::mutex mtx_;
  std::vector<KDL::JntArray> solutions;
  std::vector<std::pair<double, uint> >  errors;

  std::thread task1, task2;
  KDL::Twist bounds;

  bool unique_solution(const KDL::JntArray& sol);

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }


  double manipPenalty(const KDL::JntArray&);
  double ManipValue1(const KDL::JntArray&);
  double ManipValue2(const KDL::JntArray&);

  inline bool myEqual(const KDL::JntArray& a, const KDL::JntArray& b)
  {
    return (a.data - b.data).isZero(1e-4);
  }

  void initialize();

};

inline bool XRSOLVE_IK::runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in)
{
  return runSolver(*iksolver.get(), *nl_solver.get(), q_init, p_in);
}

inline bool XRSOLVE_IK::runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in)
{
  return runSolver(*nl_solver.get(), *iksolver.get(), q_init, p_in);
}

}

#endif
