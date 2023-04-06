#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.005f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    // bool moved = 0;
    // double dx_temp = odometry.x - previousPose_.x;
    // double dy_temp = odometry.y - previousPose_.y;
    // double dtheta_temp = odometry.theta - previousPose_.theta;
    // double ds_temp = sqrt(dx_temp*dx_temp + dy_temp*dy_temp);
    // double alpha_temp = atan2(dy_temp,dx_temp) - previousPose_.theta;

    // if (fabs(dtheta_temp) >= min_theta_ && ds_temp >= min_dist_)
    //     moved = 1;
    //     dx_ = dx_temp;
    //     dy_ = dy_temp;
    //     dtheta_ = dtheta_temp;
    //     alpha_ = alpha_temp;
    //     ds_ = ds_temp;
    // return moved;
    if(!initialized_){
        previousPose_ = odometry;
        initialized_ = true;
    }
    float deltaX = odometry.x - previousPose_.x;
    float deltaY = odometry.y - previousPose_.y;
    float deltaTheta = odometry.theta - previousPose_.theta;
    trans_ = sqrt(deltaX*deltaX + deltaY*deltaY);
    rot1_ = angle_diff(atan2(deltaY,deltaX),previousPose_.theta);
    float direction;
    if(abs(rot1_) > M_PI/2){
        rot1_ = angle_diff(M_PI,rot1_);
        direction = -1;
    }
    rot2_ = angle_diff(deltaTheta, rot1_);
    moved_ = (deltaX != 0)||(deltaY !=0)||(deltaTheta != 0);
    if(moved_){
        rot1Std_ = sqrt(k1_ * abs(rot1_));
        transStd_ = sqrt(k2_* abs(trans_));
        rot2Std_ = sqrt(k1_ * abs(rot2_));
    }
    previousPose_ = odometry;
    utime_ = odometry.utime;
    return moved_;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    float sampleRot1 = std::normal_distribution<>(rot1_,rot1Std_)(numberGenerator_);
    float sampleTrans = std::normal_distribution<>(trans_,transStd_)(numberGenerator_);
    float sampleRot2 = std::normal_distribution<>(rot2_,rot2Std_)(numberGenerator_);

    newSample.pose.x += sampleTrans*cos(sample.pose.theta + sampleRot1);
    newSample.pose.y += sampleTrans*sin(sample.pose.theta + sampleRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    
    
    
    
    
    
    
    
    

        // std::normal_distribution<double>epsilon1(0,k1_*fabs(alpha_));
    // std::normal_distribution<double>epsilon2(0,k2_*fabs(ds_));
    // std::normal_distribution<double>epsilon3(0,k1_*fabs(dtheta_ - alpha_));

    // newSample.pose.x = (ds_ + epsilon2)*cos(previousPose_.theta + alpha_ + epsilon1);
    // newSample.pose.y = (ds_ + epsilon2)*sin(previousPose_.theta + alpha_ + epsilon1);
    // newSample.pose.theta = dtheta_ + epsilon1 + epsilon3;


    return newSample;
}
