/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#ifndef LWR_CONTROLLERS_JOINT_POSITION_CONTROLLER_H
#define LWR_CONTROLLERS_JOINT_POSITION_CONTROLLER_H

// Controller base
#include "kinematic_chain_controller_base.h"

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include <string>

#include <kdl/jntarrayacc.hpp>

#include <robot_motion_generation/CDDynamics.h>

#include <boost/scoped_ptr.hpp>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>


// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include <kdl_conversions/kdl_msg.h>

// Service GetJointVelocity
#include <kuka_lwr_controllers/GetJointVelocity.h>



namespace kuka_lwr_controllers
{
	class JointPositionController: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
		public:
			JointPositionController();
			~JointPositionController();

			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			
		private:
			void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe command topic
			void setMaxVelocityCB(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe setMaxVelocity topic
			ros::Subscriber sub_command_, sub_max_velovity_;
			ros::ServiceServer srv_get_velocity_;
			std::string robot_namespace_;
			int cmd_flag_;  // flag set only to 1 when the controller receive a message to the command topic
			
			realtime_tools::RealtimeBuffer<KDL::JntArray> q_target_buffer_;
			KDL::JntArray *q_target_;
			KDL::JntArray q_target_cmd_;
			KDL::JntArray q_cmd_; // q desired setting by command topic
			
			boost::scoped_ptr<motion::CDDynamics>   joint_cddynamics;
			
			
			std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Pose> > realtime_x_pub_;
			std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_state_pub_;
			
			// KDL solvers
			boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
			
			KDL::Frame x_;		//current pose
			
			motion::Vector velLimits_;
			
			bool getCurrentJointVelocity(kuka_lwr_controllers::GetJointVelocity::Request& req, kuka_lwr_controllers::GetJointVelocity::Response& resp);
    };
}


#endif

