#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
//#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>
#include<chrono>
#include <bits/stdc++.h>
#include <mutex>
#include <condition_variable>

#include "robot_motion/srv/map_path.hpp"
#include "robot_motion/action/base_control.hpp"
#include "robot_motion/action/arm_control.hpp"
#include "ros2_grasping/action/attacher.hpp"

#include"Manipulability.cpp"
#include"GlobalPlannerOld.cpp"
//#include"AStar.cpp"

class pickPlace : public rclcpp::Node
{
  public:
    using ArmControl =  robot_motion::action::ArmControl; 
    using BaseControl =  robot_motion::action::BaseControl;
    using Attacher = ros2_grasping::action::Attacher; 

    
    pickPlace()
    : Node("pick_place")
    {
      rclcpp::QoS qos_profile(10);
      qos_profile.transient_local();
      callBackGroup1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      callBackGroup2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      
      mapSubscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos_profile, std::bind(&pickPlace::dyn_manip, this, std::placeholders::_1));
      
      baseClient = rclcpp_action::create_client<BaseControl>(this, "/move_base", callBackGroup2);
      armClient = rclcpp_action::create_client<ArmControl>(this, "/move_arm", callBackGroup1);
      attachClient = rclcpp_action::create_client<Attacher>(this, "/Attacher", callBackGroup1);
      
      rclcpp::on_shutdown(std::bind(&pickPlace::onShutdown, this));
    }
  private:
    void dyn_manip(const nav_msgs::msg::OccupancyGrid mapMsg);

    
    std_msgs::msg::Float64MultiArray generatePathMsg(std::vector<std::vector<double>> globalPath)
    {
      auto pathMsg = std_msgs::msg::Float64MultiArray();
      pathMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      pathMsg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      pathMsg.layout.dim[0].size = globalPath.size();
      pathMsg.layout.dim[1].size = globalPath[0].size();
      pathMsg.layout.dim[0].stride = globalPath.size() * globalPath[0].size();
      pathMsg.layout.dim[1].stride = globalPath[0].size();
      pathMsg.layout.data_offset = 0;
      std::vector<double> tempVec(globalPath.size()*globalPath[0].size(), 0);
      for (int i=0; i<int(globalPath.size()); i++)
      {
        for (int j=0; j<int(globalPath[0].size()); j++)
        { 
          tempVec[i*globalPath[0].size() + j] = globalPath[i][j];
        }
      }
      pathMsg.data = tempVec;

      return pathMsg;
    }
    
    void generateBaseMsg(BaseControl::Goal& baseMsg, const std_msgs::msg::Float64MultiArray& pathMsg, const std::vector<double>& graspAreaEndPose, bool inGraspArea = false){
      std::vector<std::vector<double>> null = {{0}, {0,0}};    
      //auto baseMsg = BaseControl::Goal();
      baseMsg.path = pathMsg;
      baseMsg.in_workspace = inGraspArea;
      if(inGraspArea){
        baseMsg.workspace_end_pose = graspAreaEndPose;
        baseMsg.workspace_speed = 0.04;
      }
      else{
        baseMsg.workspace_speed = null[0][0];
        baseMsg.workspace_end_pose = null[1];
      }
      baseMsg.goal_threshold = 0.1;
      

    }
    

    void armResultCallback(const rclcpp_action::ClientGoalHandle<ArmControl>::WrappedResult & result)  
    {
      std::cout<<"Calling armResultCallBack\n";
      switch (result.code) 
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          std::cout << "Arm moved to desired position" << std::endl;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          //std::cout << "Stopping arm hold " << std::endl;
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
        
      }
      auto armMsg = ArmControl::Goal();
      armMsg.time = armMoveTime;
      armMsg.goal = result.result->last_position;
      armMsg.hold = true;
      auto armSendGoalOpt = rclcpp_action::Client<ArmControl>::SendGoalOptions();
      armSendGoalOpt.result_callback = std::bind(&pickPlace::armResultCallback, this, std::placeholders::_1);
      armClient->async_send_goal(armMsg, armSendGoalOpt);
      std::cout << "Holding arm to current position" << std::endl;
      baseMove = true;
    }

    void baseFeedbackCallback(rclcpp_action::ClientGoalHandle<BaseControl>::SharedPtr, const std::shared_ptr<const BaseControl::Feedback> feedback)
    { 
      currBasePos.clear();
      currBasePos = feedback->curr_position;
      if(std::sqrt(std::pow(currBasePos[0] - armMoveLoc[0], 2) + std::pow(currBasePos[1] - armMoveLoc[1], 2)) <= 0.05)
      {
        armMove = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(17));
      }
    }
  
    void baseResultCallback(const rclcpp_action::ClientGoalHandle<BaseControl>::WrappedResult & result)  
    {
      std::cout<<"Inside base result callback\n";
      switch (result.code) 
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          std::cout << "Base reached successfully to destination. Setting goal reached to true." << std::endl;
          goalReached = true;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
    }
  
    void onShutdown() 
    {
      RCLCPP_INFO(get_logger(), "Ctrl+C pressed. Shutting down the node");
      rclcpp::shutdown();
    }
    
    bool readyToLift = false;
    bool baseMove = false;
    bool armMove = false;
    bool goalReached = false;
    std::vector<double> currBasePos;
    std::vector<double> armMoveLoc;

    std::vector<std::vector<double>> startToGoal;
    std::vector<std::vector<double>> startToPick;
    std::vector<std::vector<double>> graspAreaStartToEnd;
    std::vector<std::vector<double>> pickToGoal;

    //Arm related variables
    //std::vector<double> targetPose {-1.6, -1.31, 0.68, 1, 0, 0, 0};
    std::vector<double> targetPose {4.61, 2.71, 0.68, 1, 0, 0, 0};
    double armMoveTime = 10.0;

    //Callback groups for paraller execution    
    rclcpp::CallbackGroup::SharedPtr callBackGroup1;
    rclcpp::CallbackGroup::SharedPtr callBackGroup2;
    
    //Ros variables
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription_;   
    rclcpp_action::Client<BaseControl>::SharedPtr baseClient;
    rclcpp_action::Client<ArmControl>::SharedPtr armClient;
    rclcpp_action::Client<Attacher>::SharedPtr attachClient;
    
    
    IRB_FK_IK irb;
};