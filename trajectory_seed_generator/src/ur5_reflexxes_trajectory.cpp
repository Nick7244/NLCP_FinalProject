#include <trajectory_seed_generator/ur5_reflexxes_trajectory.hpp>

#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <ur5_box_msgs/ur5_trajectory.h>

#include <vector>

ur5_reflexxes_trajectory::ur5_reflexxes_trajectory( ros::NodeHandle& nh, float controllerPeriod )
    : nh(nh),
    box_pos_sub(nh)
{
    // Get the robot description from the parameter server
    std::string robot_description_string;
    nh.param("/robot_description", robot_description_string, std::string());

    // Obtain the FK solver
    if ( kdl_parser::treeFromString(robot_description_string, tree) )
    {
        if ( tree.getChain("base_link", "tool0", chain) )
        {
            fk_pos = new KDL::ChainFkSolverPos_recursive(chain);
            ik_vel = new KDL::ChainIkSolverVel_pinv(chain);
            ik_pos = new KDL::ChainIkSolverPos_NR(chain, *fk_pos, *ik_vel);
        }

        else 
        {
            ROS_ERROR("Cannot get a chain");
        }
    }

    else
    {
        ROS_ERROR("Cannot get a tree");
    }

    // Create the joint state subscriber
    sub_js = nh.subscribe("/joint_states", 10, &ur5_reflexxes_trajectory::jointStateCallback, this);

    // Initialize the internal joint state variable
    joint_state.resize(6);
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = 0.0;
    }

    // Create the trajectory publisher
    pub_traj = nh.advertise<ur5_box_msgs::ur5_trajectory>("/ur5_seed_trajectory",10);

    // Initialize trajectory generation objects for 3-dimensional cartesian trajectory
    rml = new ReflexxesAPI(6, controllerPeriod);
    ip = new RMLPositionInputParameters(6);
    op = new RMLPositionOutputParameters(6);

    // Initialize trajectory generation parameters
    for ( int i = 0; i < 6; i++ )
    {
        ip->CurrentPositionVector->VecData[i] = 0.0;
        ip->CurrentVelocityVector->VecData[i] = 0.0;
        ip->CurrentAccelerationVector->VecData[i] = 0.0;

        ip->MaxVelocityVector->VecData[i] = 0.5;
        ip->MaxAccelerationVector->VecData[i] = 0.75;
        ip->MaxJerkVector->VecData[i] = 1.0;

        ip->SelectionVector->VecData[i] = true;

        ip->TargetPositionVector->VecData[i] = 0.0;
        ip->TargetVelocityVector->VecData[i] = 0.0;
    }
}


void ur5_reflexxes_trajectory::computeTrajectory()
{
    // Declare trajectory variables
    std_msgs::Float64MultiArray q1_traj;
    std_msgs::Float64MultiArray q2_traj;
    std_msgs::Float64MultiArray q3_traj;
    std_msgs::Float64MultiArray q4_traj;
    std_msgs::Float64MultiArray q5_traj;
    std_msgs::Float64MultiArray q6_traj;

    std_msgs::Float64MultiArray q1_dot_traj;
    std_msgs::Float64MultiArray q2_dot_traj;
    std_msgs::Float64MultiArray q3_dot_traj;
    std_msgs::Float64MultiArray q4_dot_traj;
    std_msgs::Float64MultiArray q5_dot_traj;
    std_msgs::Float64MultiArray q6_dot_traj;

    int result = 0;

    // Compute trajectory
    while ( result != ReflexxesAPI::RML_FINAL_STATE_REACHED )
    {
        q1_traj.data.push_back(ip->CurrentPositionVector->VecData[0]);
        q2_traj.data.push_back(ip->CurrentPositionVector->VecData[1]);
        q3_traj.data.push_back(ip->CurrentPositionVector->VecData[2]);
        q4_traj.data.push_back(ip->CurrentPositionVector->VecData[3]);
        q5_traj.data.push_back(ip->CurrentPositionVector->VecData[4]);
        q6_traj.data.push_back(ip->CurrentPositionVector->VecData[5]);

        q1_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[0]);
        q2_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[1]);
        q3_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[2]);
        q4_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[3]);
        q5_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[4]);
        q6_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[5]);

        // Compute an iteration of the trajectory
        result = rml->RMLPosition(*ip, op, flags);

        // Setup for next iteration
        *ip->CurrentPositionVector = *op->NewPositionVector;
        *ip->CurrentVelocityVector = *op->NewVelocityVector;
        *ip->CurrentAccelerationVector = *op->NewAccelerationVector;
    }

    q1_traj.data.push_back(ip->CurrentPositionVector->VecData[0]);
    q2_traj.data.push_back(ip->CurrentPositionVector->VecData[1]);
    q3_traj.data.push_back(ip->CurrentPositionVector->VecData[2]);
    q4_traj.data.push_back(ip->CurrentPositionVector->VecData[3]);
    q5_traj.data.push_back(ip->CurrentPositionVector->VecData[4]);
    q6_traj.data.push_back(ip->CurrentPositionVector->VecData[5]);

    q1_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[0]);
    q2_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[1]);
    q3_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[2]);
    q4_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[3]);
    q5_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[4]);
    q6_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[5]);

    // Create trajectory message
    ur5_box_msgs::ur5_trajectory trajectoryMsg;
    trajectoryMsg.q1_trajectory = q1_traj;
    trajectoryMsg.q2_trajectory = q2_traj;
    trajectoryMsg.q3_trajectory = q3_traj;
    trajectoryMsg.q4_trajectory = q4_traj;
    trajectoryMsg.q5_trajectory = q5_traj;
    trajectoryMsg.q6_trajectory = q6_traj;

    trajectoryMsg.q1_dot_trajectory = q1_traj;
    trajectoryMsg.q2_dot_trajectory = q2_traj;
    trajectoryMsg.q3_dot_trajectory = q3_traj;
    trajectoryMsg.q4_dot_trajectory = q4_traj;
    trajectoryMsg.q5_dot_trajectory = q5_traj;
    trajectoryMsg.q6_dot_trajectory = q6_traj;

    ROS_INFO("Trajectory successfully generated, publishing to topic...");

    // Publish trajectory
    pub_traj.publish(trajectoryMsg);
}


void ur5_reflexxes_trajectory::generateSeedTrajectory()
{
    // Obtain current end-effector position from the FK
    KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;
    fk_pos->JntToCart(q_cur, p_cur, -1);





    // Obtain box position
    std::vector<float> box_pos = box_pos_sub.getBoxPos();
    
    // Compute  position of being underneath the box
    KDL::Frame p_desired = p_cur;
    KDL::Vector desired_pos(box_pos[0], box_pos[1], box_pos[2]);
    p_desired.p = desired_pos;
    //p_desired.M = robotOrientation;

    // Compute inverse kinematics to box x-y position
    KDL::JntArray q_desired(6);
    ik_pos->CartToJnt(q_cur, p_desired, q_desired);

    // Initialize trajectory with current end-effector position
    ip->CurrentPositionVector->VecData[0] = joint_state.data[0];
    ip->CurrentPositionVector->VecData[1] = joint_state.data[1];
    ip->CurrentPositionVector->VecData[2] = joint_state.data[2];
    ip->CurrentPositionVector->VecData[3] = joint_state.data[3];
    ip->CurrentPositionVector->VecData[4] = joint_state.data[4];
    ip->CurrentPositionVector->VecData[5] = joint_state.data[5];

    // Set desired point of trajectory to the box position
    ip->TargetPositionVector->VecData[0] = q_desired.data[0];
    ip->TargetPositionVector->VecData[1] = q_desired.data[1];
    ip->TargetPositionVector->VecData[2] = q_desired.data[2];
    ip->TargetPositionVector->VecData[3] = q_desired.data[3];
    ip->TargetPositionVector->VecData[4] = q_desired.data[4];
    ip->TargetPositionVector->VecData[5] = q_desired.data[5];

    for ( int i = 0; i < 6; i++ )
    {
        ip->TargetVelocityVector->VecData[i] = 0.0;
    }

    computeTrajectory();
}


void ur5_reflexxes_trajectory::TPoseTrajectory()
{
    // Initialize trajectory with current end-effector position
    ip->CurrentPositionVector->VecData[0] = joint_state.data[0];
    ip->CurrentPositionVector->VecData[1] = joint_state.data[1];
    ip->CurrentPositionVector->VecData[2] = joint_state.data[2];
    ip->CurrentPositionVector->VecData[3] = joint_state.data[3];
    ip->CurrentPositionVector->VecData[4] = joint_state.data[4];
    ip->CurrentPositionVector->VecData[5] = joint_state.data[5];

    // Set desired point of trajectory to T-Pose
    /*t_pose_coords.data[0] = 0.96;
    t_pose_coords.data[1] = -1.04591;
    t_pose_coords.data[2] = 1.11256 ;
    t_pose_coords.data[3] = -0.0667037;
    t_pose_coords.data[4] = 1.5708;
    t_pose_coords.data[5] = -0.0506;*/

    ip->TargetPositionVector->VecData[0] = 0.0;
    ip->TargetPositionVector->VecData[1] = -1.04591;
    ip->TargetPositionVector->VecData[2] = 1.11256;
    ip->TargetPositionVector->VecData[3] = -0.0667037;
    ip->TargetPositionVector->VecData[4] = 1.5708;
    ip->TargetPositionVector->VecData[5] = 0.0;

    for ( int i = 0; i < 6; i++ )
    {
        ip->TargetVelocityVector->VecData[i] = 0.0;
    }

    computeTrajectory();
}


void ur5_reflexxes_trajectory::jointStateCallback(const sensor_msgs::JointState& js )
{
    // Copy over the joint state values from the UR5
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = js.position[i];
    }

    // Switch the first and third elements to make ordering consistent with FK chain
    float future0 = joint_state.data[2];
    float future2 = joint_state.data[0];
    joint_state.data[0] = future0;
    joint_state.data[2] = future2;
}