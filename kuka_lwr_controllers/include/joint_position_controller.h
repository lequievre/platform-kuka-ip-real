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

#include <string>

#include <kdl/jntarrayacc.hpp>

#include <robot_motion_generation/CDDynamics.h>

#include <boost/scoped_ptr.hpp>

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
			ros::Subscriber sub_command_;
			std::string robot_namespace_;
			int cmd_flag_;  // flag set only to 1 when the controller receive a message to the command topic
			
			KDL::JntArray    q_target_;
			KDL::JntArray q_cmd_; // q desired setting by command topic
			
			boost::scoped_ptr<motion::CDDynamics>   joint_cddynamics;
    };
}


#endif

