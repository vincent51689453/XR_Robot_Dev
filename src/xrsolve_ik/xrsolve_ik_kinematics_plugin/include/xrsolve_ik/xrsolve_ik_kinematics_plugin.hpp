#ifndef XRSOLVE_IK_KINEMATICS_PLUGIN_
#define XRSOLVE_IK_KINEMATICS_PLUGIN_

#include <moveit/kinematics_base/kinematics_base.h>
#include <kdl/chain.hpp>

namespace xrsolve_ik_kinematics_plugin
{

class XRSOLVE_IKKinematicsPlugin : public kinematics::KinematicsBase
{
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  uint num_joints_;
  bool active_; // 内部变量，指示求解器是否已配置并准备就绪

  KDL::Chain chain;
  bool position_ik_;

  KDL::JntArray joint_min, joint_max;

  std::string solve_type;

public:
  const std::vector<std::string>& getJointNames() const
  {
    return joint_names_;
  }
  const std::vector<std::string>& getLinkNames() const
  {
    return link_names_;
  }


  XRSOLVE_IKKinematicsPlugin(): active_(false), position_ik_(false) {}

  ~XRSOLVE_IKKinematicsPlugin()
  {
  }


  // 返回在联合限制内的第一个IK解决方案，它由get_ik()服务调用
  bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                     const std::vector<double> &ik_seed_state,
                     std::vector<double> &solution,
                     moveit_msgs::MoveItErrorCodes &error_code,
                     const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;


  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        const std::vector<double> &consistency_limits,
                        std::vector<double> &solution,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;


  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;


  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        const std::vector<double> &consistency_limits,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const std::vector<double> &consistency_limits,
                        const kinematics::KinematicsQueryOptions &options) const;



  bool getPositionFK(const std::vector<std::string> &link_names,
                     const std::vector<double> &joint_angles,
                     std::vector<geometry_msgs::Pose> &poses) const;


  bool initialize(const std::string &robot_description,
                  const std::string& group_name,
                  const std::string& base_name,
                  const std::string& tip_name,
                  double search_discretization);

private:

  int getKDLSegmentIndex(const std::string &name) const;

}; // end class
}

#endif
