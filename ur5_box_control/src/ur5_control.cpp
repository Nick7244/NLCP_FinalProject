#include <ur5_box_control/ur5_control.hpp>

#include <std_msgs/Float64.h>

#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

#include <string>

#include <rtt/Component.hpp> // This should come last in the include list

ur5_control::ur5_control( const std::string& name ) 
    : TaskContext(name),
    port_cmd_jnt_pos("Commanded joint position"),
    trajectoryIteration(0),
    trajectoryLength(0),
    trajectoryReceived(false),
    executeTrajectoryCmd(false)
{
    // Create the trajectory subscriber
    sub_traj = nh.subscribe("/ur5_seed_trajectory", 10, &ur5_control::trajectoryCallback, this);

    addPort("CmdJntPos", port_cmd_jnt_pos);
    addOperation("TPose", &ur5_control::TPose, this, RTT::OwnThread); 
    addOperation("GenerateTrajectory", &ur5_control::generateTrajectory, this, RTT::OwnThread); 
    addOperation("ExecuteTrajectory", &ur5_control::executeTrajectory, this, RTT::OwnThread);
}


bool ur5_control::configureHook() 
{
     std::cout << "ur5_control::configureHook" << std::endl;

    // Create trajectory seed generator object
    trajectorySeedGenerator =  new ur5_reflexxes_trajectory(nh, getPeriod());
}


bool ur5_control::startHook() 
{
    std::cout << "ur5_control::startHook" << std::endl;
}


void ur5_control::updateHook() 
{
    // If trajectory has been received
    if ( executeTrajectoryCmd )
    {
        // While we have not yet finished the trajectory
        if ( trajectoryIteration < trajectoryLength )
        {
            // get the joint values and write them to the port
            std_msgs::Float64MultiArray js;
            js.data.resize(6);

            js.data[0] = jointTrajectory.q1_trajectory.data[trajectoryIteration];
            js.data[1] = jointTrajectory.q2_trajectory.data[trajectoryIteration];
            js.data[2] = jointTrajectory.q3_trajectory.data[trajectoryIteration];
            js.data[3] = jointTrajectory.q4_trajectory.data[trajectoryIteration];
            js.data[4] = jointTrajectory.q5_trajectory.data[trajectoryIteration];
            js.data[5] = jointTrajectory.q6_trajectory.data[trajectoryIteration];
            
            port_cmd_jnt_pos.write(js);
            trajectoryIteration++;
        }

        // Else, trajectory is finished, reset parameters
        else
        {
            ROS_INFO("Trajectory completed!");

            trajectoryReceived = false;
            executeTrajectoryCmd = false;
            trajectoryIteration = 0;
        }
    }    
}


void ur5_control::stopHook() 
{
    std::cout << "ur5_control::stopHoop" << std::endl;
}


void ur5_control::cleanupHook() 
{
    std::cout << "ur5_control::cleanupHook" << std::endl;
}

void ur5_control::TPose()
{ 
    ROS_INFO("Generating T-Pose trajectory...");
    trajectorySeedGenerator->TPoseTrajectory();
}


void ur5_control::generateTrajectory()
{
    ROS_INFO("Generating seed trajectory...");
    trajectorySeedGenerator->generateSeedTrajectory();
}


void ur5_control::executeTrajectory()
{
    if ( trajectoryReceived )
    {
        ROS_INFO("Executing trajectory...");
        executeTrajectoryCmd = true;
    }

    else
    {
        ROS_WARN("No trajectory has been received yet. Please generate a trajectory first, and wait for it to be received before attempting to execute.");
    }
}


void ur5_control::trajectoryCallback( const ur5_box_msgs::ur5_trajectory& trajectory )
{
    ROS_INFO("Trajectory received!");

    trajectoryReceived = true;
    trajectoryLength = trajectory.q1_trajectory.data.size();
    
    jointTrajectory = trajectory;
}

ORO_CREATE_COMPONENT(ur5_control) // register the RTT component
