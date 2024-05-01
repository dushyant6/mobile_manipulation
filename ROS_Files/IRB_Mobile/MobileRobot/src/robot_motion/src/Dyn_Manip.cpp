#include"Dyn_Manip.h"

void pickPlace::dyn_manip(const nav_msgs::msg::OccupancyGrid mapMsg)
{    
    //define baseControl action goaloptions
    auto baseSendGoalOpt = rclcpp_action::Client<BaseControl>::SendGoalOptions();
    baseSendGoalOpt.result_callback = std::bind(&pickPlace::baseResultCallback, this, std::placeholders::_1);
    baseSendGoalOpt.feedback_callback = std::bind(&pickPlace::baseFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

    //define armcontrol action goalopotions    
    auto armSendGoalOpt = rclcpp_action::Client<ArmControl>::SendGoalOptions();
    armSendGoalOpt.result_callback = std::bind(&pickPlace::armResultCallback, this, std::placeholders::_1);

    //Find grasp pose and base pose while grasping
    irb.getBasePose(targetPose);
    std::vector<double> graspAreaStart = {irb.pickBasePose[0] - 0.4, irb.baseY, irb.pickBasePose[2]};
    double graspAreaEndX = graspAreaStart[0] + 2.0;
    std::vector<double> graspAreaEnd = {graspAreaEndX, graspAreaStart[1], 0.0};

    armMoveLoc = {graspAreaStart[0]+0.5, graspAreaStart[1], graspAreaStart[2]};
    std::cout << " Base location at which arm starts moving: " << armMoveLoc[0] << " " << armMoveLoc[1] << std::endl; 

    std::vector<double> start = {1.0, 2.0, 0.0};
    std::vector<double> goal = {7.0, 1.5, 0.0};    
    
    //Find global paths
    globalPlanner gPlanner;
    //globPlanner gPlanner;

    std::cout<<"Running AStar\n";
    
    startToPick = gPlanner.findGlobalPath(mapMsg, start, graspAreaStart);
    graspAreaStartToEnd = gPlanner.findGlobalPath(mapMsg, graspAreaStart, graspAreaEnd);
    pickToGoal = gPlanner.findGlobalPath(mapMsg, graspAreaEnd, goal);
    
    auto pathMsg = generatePathMsg(startToPick);
    auto pathMsg2 = generatePathMsg(graspAreaStartToEnd);
    auto pathMsg3 = generatePathMsg(pickToGoal);
    ///*
    for(auto pt: startToPick){
        for(auto cord: pt){
            std::cout<<cord<<", ";
        }
        std::cout<<"\n";
    }
    std::cout<<"------\n";
    for(auto pt: graspAreaStartToEnd){
        for(auto cord: pt){
            std::cout<<cord<<", ";
        }
        std::cout<<"\n";
    }
    std::cout<<"------\n";
    for(auto pt: pickToGoal){
        for(auto cord: pt){
            std::cout<<cord<<", ";
        }
        std::cout<<"\n";
    }
    //*/

    auto startTime = std::chrono::steady_clock::now();
    // Run Pure Pursuit Controller to go to pick pose
    std::vector<std::vector<double>> null = {{0}, {0,0}};    
    auto baseMsg = BaseControl::Goal();
    generateBaseMsg(baseMsg, pathMsg, null[1], false);

    baseClient->async_send_goal(baseMsg, baseSendGoalOpt); 
    
    while(!goalReached){}      

    goalReached = false;

    auto baseMsg2 = BaseControl::Goal();
    std::vector<double> graspEnd = {graspAreaStart[0] + 0.5, graspAreaEnd[1]};
    generateBaseMsg(baseMsg2, pathMsg2, graspEnd, true);
    baseClient->async_send_goal(baseMsg2, baseSendGoalOpt); 
    std::cout << "Going to inside grasp area " << std::endl;   

    baseMove = false;
    auto armMsg = ArmControl::Goal();
    armMsg.time = 8.0;
    armMsg.goal = irb.pickArmPose;
    std::cout<<"Manip arm goal == "<<irb.pickArmPose[0]<<", "<<irb.pickArmPose[1]<<", "<<irb.pickArmPose[2]<<", "<<irb.pickArmPose[3]<<", "<<irb.pickArmPose[4]<<", "<<irb.pickArmPose[5]<<", "<<"\n";
    std::cout<<irb.pickArmPose[1]*180/M_PI<<", "<<irb.pickArmPose[4]*180/M_PI<<"\n";
    armMsg.hold = false;
    armClient->async_send_goal(armMsg, armSendGoalOpt);

    while(!baseMove){}
    std::cout << "Attaching the object to gripper" << std::endl;       
    auto attachMsg = Attacher::Goal();
    attachMsg.object = "box";
    attachMsg.endeffector = "EE_egp64";
    auto attachSendGoalOpt = rclcpp_action::Client<Attacher>::SendGoalOptions();
    attachClient->async_send_goal(attachMsg, attachSendGoalOpt); 

    //wait till end of grasp area
    while(!goalReached){}
    goalReached = false;    
   

auto end = std::chrono::steady_clock::now();

// Calculate the duration
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - startTime);

// Output the duration in milliseconds
std::cout << "Elapsed time: " << duration.count() << " milliseconds" << std::endl;

}  