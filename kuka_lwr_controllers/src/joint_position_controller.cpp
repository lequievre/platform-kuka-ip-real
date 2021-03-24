/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include <joint_position_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>


namespace kuka_lwr_controllers 
{
    JointPositionController::JointPositionController() {}
    JointPositionController::~JointPositionController() 
    {
    }
    
    bool JointPositionController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
        robot_namespace_ = n.getNamespace();
			
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("GroupCommandControllerFRI: Couldn't initilize GroupCommandController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        
        sub_command_ = nh_.subscribe("command", 1, &JointPositionController::commandCB, this);
        
        cmd_flag_ = 0;  // set this flag to 0 to not run the update method
        
        q_target_.resize(joint_handles_.size());
        q_cmd_.resize(joint_handles_.size());

       
       joint_cddynamics.reset(new motion::CDDynamics(joint_handles_.size(),1e-6,1));
       
       motion::Vector velLimits(joint_handles_.size());
      
       for (std::size_t i = 0; i < joint_handles_.size(); i++){
           velLimits(i)  = 0.25; // x ms^-1
       }
       
        
      joint_cddynamics->SetVelocityLimits(velLimits);
        
        
        return true;
    }
    
    void JointPositionController::starting(const ros::Time& time)
    {
    
        // get joint positions
		// KDL::JntArrayAcc -> joint_msr_states_
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition(); // get current position mes
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity(); // get current velocity mes
    		q_cmd_.data(i) = joint_msr_states_.q(i);
    		q_target_.data(i) = joint_msr_states_.q(i);
    		joint_handles_[i].setCommand(q_cmd_.data(i));
    	}
    	
        cmd_flag_ = 0;  // set this flag to 0 to not run the update method
    }
    
    void JointPositionController::stopping(const ros::Time& time)
    {
        cmd_flag_ = 0;  // set this flag to 0 to not run the update method
    }
    
    void JointPositionController::update(const ros::Time& time, const ros::Duration& period)
    {
    
        for(size_t i=0; i<joint_handles_.size(); i++) 
		{
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();  // get current position
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity(); // get current velocity
		}
    
        if (cmd_flag_) {
            joint_cddynamics->SetState(joint_msr_states_.q.data);
            cmd_flag_ = 0;
        }

        joint_cddynamics->SetDt(period.toSec());
        joint_cddynamics->SetTarget(q_target_.data);
        joint_cddynamics->Update();
        joint_cddynamics->GetState(q_cmd_.data);
        
        
        for (int i = 0; i < joint_handles_.size(); i++)
					joint_handles_[i].setCommand(q_cmd_.data(i));
					
    }
    
    void JointPositionController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		 

		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("GroupCommandControllerFRI: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			cmd_flag_ = 0;
			return; 
		}
		
		for (std::size_t j = 0; j < joint_handles_.size(); ++j){
            q_target_(j)     = msg->data[j];
        }

		cmd_flag_ = 1; // set this flag to 1 to run the update method
    }

}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::JointPositionController, controller_interface::ControllerBase)


