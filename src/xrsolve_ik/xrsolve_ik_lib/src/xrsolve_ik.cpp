#include <xrsolve_ik/xrsolve_ik.hpp>
#include <boost/date_time.hpp>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <limits>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace XRSOLVE_IK
{

XRSOLVE_IK::XRSOLVE_IK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param, double _maxtime, double _eps, SolveType _type) :
  initialized(false),
  eps(_eps),
  maxtime(_maxtime),
  solvetype(_type)
{

  ros::NodeHandle node_handle("~");

  urdf::Model robot_model;
  std::string xml_string;

  std::string urdf_xml, full_urdf_xml;
  node_handle.param("urdf_xml", urdf_xml, URDF_param);
  node_handle.searchParam(urdf_xml, full_urdf_xml);

  ROS_DEBUG_NAMED("xrsolve_ik", "Reading xml file from parameter server");
  if (!node_handle.getParam(full_urdf_xml, xml_string))
  {
    ROS_FATAL_NAMED("xrsolve_ik", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return;
  }

  node_handle.param(full_urdf_xml, xml_string, std::string());
  robot_model.initString(xml_string);

  ROS_DEBUG_STREAM_NAMED("xrsolve_ik", "Reading joints and links from URDF");

  KDL::Tree tree;

  if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
    ROS_FATAL("Failed to exxrsolvet kdl tree from xml robot description");

  if (!tree.getChain(base_link, tip_link, chain))
    ROS_FATAL("Couldn't find chain %s to %s", base_link.c_str(), tip_link.c_str());

  std::vector<KDL::Segment> chain_segs = chain.segments;

  urdf::JointConstSharedPtr joint;

  std::vector<double> l_bounds, u_bounds;

  lb.resize(chain.getNrOfJoints());
  ub.resize(chain.getNrOfJoints());

  uint joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {
    joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      joint_num++;
      float lower, upper;
      int hasLimits;
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        if (joint->safety)
        {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        }
        else
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      }
      else
      {
        hasLimits = 0;
      }
      if (hasLimits)
      {
        lb(joint_num - 1) = lower;
        ub(joint_num - 1) = upper;
      }
      else
      {
        lb(joint_num - 1) = std::numeric_limits<float>::lowest();
        ub(joint_num - 1) = std::numeric_limits<float>::max();
      }
      ROS_DEBUG_STREAM_NAMED("xrsolve_ik", "IK Using joint " << joint->name << " " << lb(joint_num - 1) << " " << ub(joint_num - 1));
    }
  }

  initialize();
}


XRSOLVE_IK::XRSOLVE_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime, double _eps, SolveType _type):
  initialized(false),
  chain(_chain),
  lb(_q_min),
  ub(_q_max),
  eps(_eps),
  maxtime(_maxtime),
  solvetype(_type)
{
  initialize();
}

void XRSOLVE_IK::initialize()
{

  assert(chain.getNrOfJoints() == lb.data.size());
  assert(chain.getNrOfJoints() == ub.data.size());

  jacsolver.reset(new KDL::ChainJntToJacSolver(chain));
  nl_solver.reset(new NLOPT_IK::NLOPT_IK(chain, lb, ub, maxtime, eps, NLOPT_IK::SumSq));
  iksolver.reset(new KDL::ChainIkSolverPos_TL(chain, lb, ub, maxtime, eps, true, true));

  for (uint i = 0; i < chain.segments.size(); i++)
  {
    std::string type = chain.segments[i].getJoint().getTypeName();
    if (type.find("Rot") != std::string::npos)
    {
      if (ub(types.size()) >= std::numeric_limits<float>::max() &&
          lb(types.size()) <= std::numeric_limits<float>::lowest())
        types.push_back(KDL::BasicJointType::Continuous);
      else
        types.push_back(KDL::BasicJointType::RotJoint);
    }
    else if (type.find("Trans") != std::string::npos)
      types.push_back(KDL::BasicJointType::TransJoint);
  }

  assert(types.size() == lb.data.size());

  initialized = true;
}

bool XRSOLVE_IK::unique_solution(const KDL::JntArray& sol)
{

  for (uint i = 0; i < solutions.size(); i++)
    if (myEqual(sol, solutions[i]))
      return false;
  return true;

}

inline void normalizeAngle(double& val, const double& min, const double& max)
{
  if (val > max)
  {
    //求实际角度偏移
    double diffangle = fmod(val - max, 2 * M_PI);
    // 把它加到上界，然后返回一个完整的旋转
    val = max + diffangle - 2 * M_PI;
  }

  if (val < min)
  {
    //求实际角度偏移
    double diffangle = fmod(min - val, 2 * M_PI);
    //把它加到上界，然后返回一个完整的旋转
    val = min - diffangle + 2 * M_PI;
  }
}

inline void normalizeAngle(double& val, const double& target)
{
  double new_target = target + M_PI;
  if (val > new_target)
  {

    double diffangle = fmod(val - new_target, 2 * M_PI);

    val = new_target + diffangle - 2 * M_PI;
  }

  new_target = target - M_PI;
  if (val < new_target)
  {

    double diffangle = fmod(new_target - val, 2 * M_PI);
    val = new_target - diffangle + 2 * M_PI;
  }
}


template<typename T1, typename T2>
bool XRSOLVE_IK::runSolver(T1& solver, T2& other_solver,
                        const KDL::JntArray &q_init,
                        const KDL::Frame &p_in)
{
  KDL::JntArray q_out;

  double fulltime = maxtime;
  KDL::JntArray seed = q_init;

  boost::posix_time::time_duration timediff;
  double time_left;

  while (true)
  {
    timediff = boost::posix_time::microsec_clock::local_time() - start_time;
    time_left = fulltime - timediff.total_nanoseconds() / 1000000000.0;

    if (time_left <= 0)
      break;

    solver.setMaxtime(time_left);

    int RC = solver.CartToJnt(seed, p_in, q_out, bounds);
    if (RC >= 0)
    {
      switch (solvetype)
      {
      case Manip1:
      case Manip2:
        normalize_limits(q_init, q_out);
        break;
      default:
        normalize_seed(q_init, q_out);
        break;
      }
      mtx_.lock();
      if (unique_solution(q_out))
      {
        solutions.push_back(q_out);
        uint curr_size = solutions.size();
        errors.resize(curr_size);
        mtx_.unlock();
        double err, penalty;
        switch (solvetype)
        {
        case Manip1:
          penalty = manipPenalty(q_out);
          err = penalty * XRSOLVE_IK::ManipValue1(q_out);
          break;
        case Manip2:
          penalty = manipPenalty(q_out);
          err = penalty * XRSOLVE_IK::ManipValue2(q_out);
          break;
        default:
          err = XRSOLVE_IK::JointErr(q_init, q_out);
          break;
        }
        mtx_.lock();
        errors[curr_size - 1] = std::make_pair(err, curr_size - 1);
      }
      mtx_.unlock();
    }

    if (!solutions.empty() && solvetype == Speed)
      break;

    for (unsigned int j = 0; j < seed.data.size(); j++)
      if (types[j] == KDL::BasicJointType::Continuous)
        seed(j) = fRand(q_init(j) - 2 * M_PI, q_init(j) + 2 * M_PI);
      else
        seed(j) = fRand(lb(j), ub(j));
  }
  other_solver.abort();

  solver.setMaxtime(fulltime);

  return true;
}


void XRSOLVE_IK::normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution)
{


  bool improved = false;

  for (uint i = 0; i < lb.data.size(); i++)
  {

    if (types[i] == KDL::BasicJointType::TransJoint)
      continue;

    double target = seed(i);
    double val = solution(i);

    normalizeAngle(val, target);

    if (types[i] == KDL::BasicJointType::Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, lb(i), ub(i));

    solution(i) = val;
  }
}

void XRSOLVE_IK::normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution)
{

  bool improved = false;

  for (uint i = 0; i < lb.data.size(); i++)
  {

    if (types[i] == KDL::BasicJointType::TransJoint)
      continue;

    double target = seed(i);

    if (types[i] == KDL::BasicJointType::RotJoint && types[i] != KDL::BasicJointType::Continuous)
      target = (ub(i) + lb(i)) / 2.0;

    double val = solution(i);

    normalizeAngle(val, target);

    if (types[i] == KDL::BasicJointType::Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, lb(i), ub(i));

    solution(i) = val;
  }

}


double XRSOLVE_IK::manipPenalty(const KDL::JntArray& arr)
{
  double penalty = 1.0;
  for (uint i = 0; i < arr.data.size(); i++)
  {
    if (types[i] == KDL::BasicJointType::Continuous)
      continue;
    double range = ub(i) - lb(i);
    penalty *= ((arr(i) - lb(i)) * (ub(i) - arr(i)) / (range * range));
  }
  return std::max(0.0, 1.0 - exp(-1 * penalty));
}


double XRSOLVE_IK::ManipValue1(const KDL::JntArray& arr)
{
  KDL::Jacobian jac(arr.data.size());

  jacsolver->JntToJac(arr, jac);

  Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
  Eigen::MatrixXd singular_values = svdsolver.singularValues();

  double error = 1.0;
  for (unsigned int i = 0; i < singular_values.rows(); ++i)
    error *= singular_values(i, 0);
  return error;
}

double XRSOLVE_IK::ManipValue2(const KDL::JntArray& arr)
{
  KDL::Jacobian jac(arr.data.size());

  jacsolver->JntToJac(arr, jac);

  Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
  Eigen::MatrixXd singular_values = svdsolver.singularValues();

  return singular_values.minCoeff() / singular_values.maxCoeff();
}


int XRSOLVE_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& _bounds)
{

  if (!initialized)
  {
    ROS_ERROR("XRSOLVE-IK was not properly initialized with a valid chain or limits.  IK cannot proceed");
    return -1;
  }


  start_time = boost::posix_time::microsec_clock::local_time();

  nl_solver->reset();
  iksolver->reset();

  solutions.clear();
  errors.clear();

  bounds = _bounds;

  task1 = std::thread(&XRSOLVE_IK::runKDL, this, q_init, p_in);
  task2 = std::thread(&XRSOLVE_IK::runNLOPT, this, q_init, p_in);

  task1.join();
  task2.join();

  if (solutions.empty())
  {
    q_out = q_init;
    return -3;
  }

  switch (solvetype)
  {
  case Manip1:
  case Manip2:
    std::sort(errors.rbegin(), errors.rend());
    break;
  default:
    std::sort(errors.begin(), errors.end());
    break;
  }

  q_out = solutions[errors[0].second];

  return solutions.size();
}


XRSOLVE_IK::~XRSOLVE_IK()
{
  if (task1.joinable())
    task1.join();
  if (task2.joinable())
    task2.join();
}
}
