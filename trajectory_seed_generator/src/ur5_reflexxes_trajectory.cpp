#include <trajectory_seed_generator/ur5_reflexxes_trajectory.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <ur5_box_msgs/ur5_trajectory.h>

#include <vector.h>

ur5_reflexxes_trajectory::ur5_reflexxes_trajectory( ros::NodeHandle& nh )
    : nh(nh),
    box_pos_sub(nh),
    result(0)
{
    // Get the robot description from the parameter server
    std::string robot_description_string;
    nh.param("/robot_description", robot_description_string, std::string());

    // Obtain the FK solver
    if ( kdl_parser::treeFromString(robot_description_string, tree) )
    {
        if ( tree.getChain("base_link", "cup_link", chain) )
        {
            fk_pos = new KDL::ChainFkSolverPos_recursive(chain);
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
    sub_js = nh.subscribe("/joint_states", 10, &ur5_kendama_controller::jointStateCallback, this);

    // Initialize the internal joint state variable
    joint_state.resize(6);
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = 0.0;
    }

    // Create the trajectory publisher
    pub_traj = nh.advertise<ur5_box_msgs::ur5_trajectory>("ur5_trajectory",10);

    // TODO:
    // - fill in the correct period to ReflexxesAPI based on the higher-level task context object,
    //   maybe give input to constructor

    // Initialize trajectory generation objects for 3-dimensional cartesian trajectory
    rml = new ReflexxesAPI(3, 100);//getPeriod()); // gets period from task context object
    ip = new RMLPositionInputParameters(3);
    op = new RMLPositionOutputParameters(3);

    // Initialize trajectory generation parameters
    for ( int i = 0; i < 3; i++ )
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


void ur5_reflexxes_trajectory::generateTrajectory()
{
    // Obtain current end-effector position from the FK
    KDL::JntArray q_cur = joint_state;
    KDL::Frame p_cur;
    fk_pos->JntToCart(q_cur, p_cur, -1);

    // Initialize trajectory with current end-effector position
    ip->CurrentPositionVector->VecData[0] = p_cur.p.x();
    ip->CurrentPositionVector->VecData[1] = p_cur.p.y();
    ip->CurrentPositionVector->VecData[2] = p_cur.p.z();

    // Obtain box position
    std::vector<float> box_pos = box_pos_sub.getBoxPos();


    // Set desired point of trajectory to the box position
    ip->TargetPositionVector->VecData[0] = box_pos[0];
    ip->TargetPositionVector->VecData[1] = box_pos[1];
    ip->TargetPositionVector->VecData[2] = box_pos[2];

    for ( int i = 0; i < 3; i++ )
    {
        ip->TargetVelocityVector->VecData[i] = 0.0;
    }

    // Compute trajectory 
    std_msgs::Float64MultiArray x_traj;
    std_msgs::Float64MultiArray x_dot_traj;
    std_msgs::Float64MultiArray y_traj;
    std_msgs::Float64MultiArray y_dot_traj;
    std_msgs::Float64MultiArray z_traj;
    std_msgs::Float64MultiArray z_dot_traj;
    
    while ( result != ReflexxesAPI::RML_FINAL_STATE_REACHED )
    {
        // Compute an iteration of the trajectory
        result = rml->RMLPosition(*ip, op, flags);

        x_traj.data.push_back(ip->CurrentPositionVector->VecData[0]);
        x_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[0]);
        y_traj.data.push_back(ip->CurrentPositionVector->VecData[1]);
        y_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[1]);
        z_traj.data.push_back(ip->CurrentPositionVector->VecData[2]);
        z_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[2]);

        // Setup for next iteration
        *ip->CurrentPositionVector = *op->NewPositionVector;
        *ip->CurrentVelocityVector = *op->NewVelocityVector;
        *ip->CurrentAccelerationVector = *op->NewAccelerationVector;
    }

    x_traj.data.push_back(ip->CurrentPositionVector->VecData[0]);
    x_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[0]);
    y_traj.data.push_back(ip->CurrentPositionVector->VecData[1]);
    y_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[1]);
    z_traj.data.push_back(ip->CurrentPositionVector->VecData[2]);
    z_dot_traj.data.push_back(ip->CurrentVelocityVector->VecData[2]);

    // Create trajectory message
    ur5_box_msgs::ur5_trajectory trajectoryMsg;
    trajectoryMsg.x_trajectory = x_traj;
    trajectoryMsg.x_trajectory = x_dot_traj;
    trajectoryMsg.x_trajectory = y_traj;
    trajectoryMsg.x_trajectory = y_dot_traj;
    trajectoryMsg.x_trajectory = z_traj;
    trajectoryMsg.x_trajectory = z_dot_traj;

    // Publish trajectory
    pub_traj.publish(trajectoryMsg);
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