#ifndef LWR_CONTROLLERS_ONE_TASK_INVERSE_KINEMATICS_H
#define LWR_CONTROLLERS_ONE_TASK_INVERSE_KINEMATICS_H

// Controller base
#include "kinematic_chain_controller_base.h"

// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

// Boost
#include <boost/scoped_ptr.hpp>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface
// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_interface.h> // contains definition of KUKAJointInterface

// Ros messages generated
#include <kuka_lwr_controllers/PoseRPY.h>

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

// realtime tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <geometry_msgs/Pose.h>
#include <kdl_conversions/kdl_msg.h>


namespace kuka_lwr_controllers
{
	class OneTaskInverseKinematics: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface>
	{
		public:
			OneTaskInverseKinematics();
			~OneTaskInverseKinematics();

			bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n);
			void starting(const ros::Time& time);
			void update(const ros::Time& time, const ros::Duration& period);
			void command(const kuka_lwr_controllers::PoseRPY::ConstPtr &msg);

		private:
		
		    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Pose> > realtime_x_pub_; // real time publisher to publish current cartesian position and orientation
		    
			ros::Subscriber sub_command_;
			ros::Subscriber sub_damping_, sub_stiffness_;

			KDL::Frame x_;		//current pose
			KDL::Frame x_des_;	//desired pose

			KDL::Twist x_err_;

			KDL::JntArray q_cmd_; // computed set points

			KDL::Jacobian J_;	//Jacobian

			Eigen::MatrixXd J_pinv_;
			Eigen::Matrix<double,3,3> skew_;

			struct quaternion_
			{
				KDL::Vector v;
				double a;
			} quat_curr_, quat_des_;

			KDL::Vector v_temp_;
	
			int cmd_flag_;
	
			boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
			boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
			
			void setDamping(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe setDamping topic
			void setStiffness(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe setStiffness topic
			
			KDL::JntArray damping_, stiffness_;
			
			std::string robot_namespace_;


	};
}

#endif
