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
        sub_max_velovity_ = nh_.subscribe("setMaxVelocity", 1, &JointPositionController::setMaxVelocityCB, this);
        srv_get_velocity_ = n.advertiseService("get_joint_velocity", &JointPositionController::getCurrentJointVelocity, this);
        
        v_max_vel_.resize(joint_handles_.size());
        
        for (size_t i=0; i<joint_handles_.size(); i++) {
            v_max_vel_[i] = 0.25;
         }
        
        
        realtime_x_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Pose>(n, "current_x", 4));
        
        realtime_state_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(n, "current_state", 4));
        
        // get joints and allocate message
        for (size_t i=0; i<joint_handles_.size(); i++){
            realtime_state_pub_->msg_.name.push_back(joint_handles_[i].getName());
            realtime_state_pub_->msg_.position.push_back(0.0);
            realtime_state_pub_->msg_.velocity.push_back(0.0);
            realtime_state_pub_->msg_.effort.push_back(0.0);
        }
        
        // kinematic solvers setup
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        
        cmd_flag_ = 0;  // set this flag to 0 to not run the update method
        
        q_target_cmd_.resize(joint_handles_.size());
        q_cmd_.resize(joint_handles_.size());

       
       joint_cddynamics.reset(new motion::CDDynamics(joint_handles_.size(),1e-6,1));
       
       motion::Vector velLimits(joint_handles_.size());
      
       for (std::size_t i = 0; i < joint_handles_.size(); i++){
           velLimits(i)  = v_max_vel_[i]; // x ms^-1
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
    		q_target_cmd_.data(i) = joint_msr_states_.q(i);
    		joint_handles_[i].setCommand(q_cmd_.data(i));
    	}
    	
    	
    	// Using realtime buffer (writing Non RT)
		q_target_buffer_.writeFromNonRT(q_target_cmd_);
		
		// Computing current cartesian position by using the forward kinematic solver
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
		
		
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
			realtime_state_pub_->msg_.position[i] = joint_msr_states_.q(i);
			realtime_state_pub_->msg_.velocity[i] = joint_msr_states_.qdot(i);
			realtime_state_pub_->msg_.effort[i] = 0.0;
		}
    
        if (cmd_flag_) {
            joint_cddynamics->SetState(joint_msr_states_.q.data);
            cmd_flag_ = 0;
        }
        
        // Computing current cartesian position by using the forward kinematic solver
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
        
        // Read realtime buffer without to be blocked.
		q_target_ = q_target_buffer_.readFromRT();

        //joint_cddynamics->SetDt(period.toSec());
        joint_cddynamics->SetDt(0.002);
        joint_cddynamics->SetTarget(q_target_->data);
        joint_cddynamics->Update();
        joint_cddynamics->GetState(q_cmd_.data);
        
        
        for (int i = 0; i < joint_handles_.size(); i++)
					joint_handles_[i].setCommand(q_cmd_.data(i));
			
	    if (realtime_x_pub_->trylock()) {
			tf::poseKDLToMsg(x_,realtime_x_pub_->msg_);
			 realtime_x_pub_->unlockAndPublish();
		}
				
	    if (realtime_state_pub_->trylock()) {
	         realtime_state_pub_->msg_.header.stamp = time;
			 realtime_state_pub_->unlockAndPublish();
		}
				
    }
    
    void JointPositionController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		 

		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("GroupCommandControllerFRI: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			cmd_flag_ = 0;
			return; 
		}
		
		
		for (std::size_t j = 0; j < joint_handles_.size(); ++j) {
		
		    if ( (msg->data[j] >= joint_limits_.min(j)) && (msg->data[j] <= joint_limits_.max(j)) ) {
                  q_target_cmd_(j)     = msg->data[j];
             }      
            else {
                    ROS_ERROR("Try to set joint index by %zu with %f rad is over limit min=%f, max=%f.", j, msg->data[j], joint_limits_.min(j), joint_limits_.max(j));
                    return;
            }
        }
        
        // Using realtime buffer (writing Non RT)
		q_target_buffer_.writeFromNonRT(q_target_cmd_);

		cmd_flag_ = 1; // set this flag to 1 to run the update method
    }
    
    
    
    void JointPositionController::setMaxVelocityCB(const std_msgs::Float64MultiArrayConstPtr& msg)
	{
		
		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("JointPositionController: Dimension (of robot " << robot_namespace_.c_str() << ") of set max velocity command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		for (size_t i=0; i<joint_handles_.size(); ++i)
		{
			v_max_vel_[i] = (double)msg->data[i];
			ROS_INFO("JointPositionController::setMaxVelocityCB Joint[%zu] = %f rad , %f ° !",i, msg->data[i], v_max_vel_[i]);
		}
		
	}
	
    
    bool JointPositionController::getCurrentJointVelocity(kuka_lwr_controllers::GetJointVelocity::Request& req, kuka_lwr_controllers::GetJointVelocity::Response& resp)
	{
		
		resp.arrayVelocities.data.resize(joint_handles_.size());
		
		for (size_t i=0; i<joint_handles_.size(); ++i)
		{
			resp.arrayVelocities.data[i] = v_max_vel_[i];
		}
		
		return true;
	}
	

}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::JointPositionController, controller_interface::ControllerBase)


