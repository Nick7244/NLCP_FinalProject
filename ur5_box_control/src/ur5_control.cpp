#include <ur5_box_control/ur5_control.hpp>

#include <std_msgs/Float64.h>

#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames.hpp>

#include <string>

#include <iostream>
#include <fstream>
#include <ctime>

#include <rtt/Component.hpp> // This should come last in the include list

ur5_control::ur5_control( const std::string& name ) 
    : TaskContext(name),
    port_cmd_jnt_pos("Commanded joint position"),
    port_cmd_jnt_torque("Commanded joint torques"),
    trajectoryIteration(0),
    trajectoryLength(0),
    trajectoryReceived(false),
    executeTrajectoryCmd(false),
    pushTraj(false)
{
    // Create the trajectory subscriber
    sub_traj = nh.subscribe("/ur5_seed_trajectory", 10, &ur5_control::trajectoryCallback, this);

    // Create the joint state subscriber
    sub_js = nh.subscribe("/joint_states", 10, &ur5_control::jointStateCallback, this);

    // Create the joint torques subscriber
    sub_torques = nh.subscribe("/control_policy_torques", 10, &ur5_control::cmdTorqueCallback, this);

    // Initialize the internal joint state variable
    joint_state.resize(6);
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = 0.0;
    }

    // Initialize joint torques
    cmd_Torques.data.resize(6);
    
    for ( int i = 0; i < 6; i++ )
    {
        cmd_Torques.data[i] = 0.0;
    }

    addPort("CmdJntPos", port_cmd_jnt_pos);
    addPort("CmdJntTorque", port_cmd_jnt_torque);
    addOperation("TPose", &ur5_control::TPose, this, RTT::OwnThread); 
    addOperation("GenerateTrajectory", &ur5_control::generateTrajectory, this, RTT::OwnThread); 
    addOperation("ExecuteTrajectory", &ur5_control::executeTrajectory, this, RTT::OwnThread);

    // Get the robot description from the parameter server
    std::string robot_description_string;
    nh.param("/robot_description", robot_description_string, std::string());

    // Obtain the FK solver
    if ( kdl_parser::treeFromString(robot_description_string, tree) )
    {
        if ( tree.getChain("base_link", "tool0", chain) )
        {
            fk_pos = new KDL::ChainFkSolverPos_recursive(chain);
            fk_vel = new KDL::ChainFkSolverVel_recursive(chain);
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
    bool effort_control;
    nh.param("/effort_control", effort_control, bool());

    if ( effort_control )
    {
        port_cmd_jnt_torque.write(cmd_Torques);
    }

    else
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

                if ( pushTraj && trajectoryIteration < 40 )
                {
                    float x[47] = {0.6867, 0.6859, 0.6831, 0.6775, 0.6684,
                        0.6561, 0.6419, 0.6273, 0.6133, 0.6000, 0.5867, 0.5726,
                        0.5579, 0.5434, 0.5289, 0.5144, 0.5001, 0.4863, 0.4727,
                        0.4591, 0.4452, 0.4310, 0.4180, 0.4075, 0.4000, 0.3960,
                        0.3975, 0.4034, 0.4101, 0.4157, 0.4207, 0.4271, 0.4350,
                        0.4414, 0.4448, 0.4457, 0.4445, 0.4416, 0.4375, 0.4326,
                        0.4273, 0.4221, 0.4171, 0.4120, 0.4069, 0.4022, 0.4000};

                    float y[47] = {0.1091, 0.1141, 0.1264, 0.1411, 0.1558,
                        0.1705, 0.1853, 0.2000, 0.2147, 0.2294, 0.2441, 0.2588,
                        0.2735, 0.2882, 0.3029, 0.3176, 0.3323, 0.3470, 0.3618,
                        0.3765, 0.3912, 0.4059, 0.4206, 0.4353, 0.4500, 0.4645,
                        0.4788, 0.4932, 0.5075, 0.5217, 0.5362, 0.5506, 0.5650,
                        0.5795, 0.5941, 0.6088, 0.6235, 0.6382, 0.6528, 0.6673,
                        0.6817, 0.6958, 0.7102, 0.7246, 0.7373, 0.7465, 0.7500};

                    float z[47] = {0.3362, 0.3348, 0.3287, 0.3181, 0.3075,
                        0.2984, 0.2902, 0.2824, 0.2744, 0.2654, 0.2552, 0.2437,
                        0.2313, 0.2189, 0.2070, 0.1965, 0.1878, 0.1810, 0.1756,
                        0.1711, 0.1668, 0.1622, 0.1562, 0.1503, 0.1500, 0.1572,
                        0.1675, 0.1761, 0.1827, 0.1907, 0.2008, 0.2102, 0.2167,
                        0.2200, 0.2205, 0.2187, 0.2149, 0.2097, 0.2033, 0.1962,
                        0.1891, 0.1830, 0.1780, 0.1724, 0.1639, 0.1544, 0.1500};

                    KDL::Vector p(x[trajectoryIteration], y[trajectoryIteration], z[trajectoryIteration]);

                    KDL::Frame frame;
                    frame.p = p;
                    frame.M = M;

                    KDL::JntArray q_cur = joint_state;
                    KDL::JntArray q_desired(6);
                    ik_pos->CartToJnt(q_cur, frame, q_desired);

                    js.data[0] = q_desired.data[0];
                    js.data[1] = q_desired.data[1];
                    js.data[2] = q_desired.data[2];
                    js.data[3] = q_desired.data[3];
                    js.data[4] = q_desired.data[4];
                    js.data[5] = q_desired.data[5];
                }

                else
                {
                    js.data[0] = jointTrajectory.q1_trajectory.data[trajectoryIteration];
                    js.data[1] = jointTrajectory.q2_trajectory.data[trajectoryIteration];
                    js.data[2] = jointTrajectory.q3_trajectory.data[trajectoryIteration];
                    js.data[3] = jointTrajectory.q4_trajectory.data[trajectoryIteration];
                    js.data[4] = jointTrajectory.q5_trajectory.data[trajectoryIteration];
                    js.data[5] = jointTrajectory.q6_trajectory.data[trajectoryIteration];
                }
                
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

    pushTraj = true;
}


void ur5_control::executeTrajectory()
{
    if ( trajectoryReceived )
    {
        ROS_INFO("Executing trajectory...");
        executeTrajectoryCmd = true;

        generateMatlab();

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

    trajectoryLength = 47;
    
    jointTrajectory = trajectory;
}


void ur5_control::jointStateCallback(const sensor_msgs::JointState& js )
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


void ur5_control::cmdTorqueCallback( const sensor_msgs::JointState& torques )
{
    for ( int i = 0; i < 6; i++ )
    {
        cmd_Torques.data[i] = torques.effort[i];
    }

}


void ur5_control::generateMatlab()
{
    ROS_INFO("Generating matlab script...");

    std::vector<float> x_trajectory;
    std::vector<float> x_dot_trajectory;

    std::vector<float> y_trajectory;
    std::vector<float> y_dot_trajectory;

    std::vector<float> z_trajectory;
    std::vector<float> z_dot_trajectory;

    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size(); i ++ )
    {
        // Obtain current end-effector position from the FK
        KDL::JntArray q_cur(6);
        q_cur.data[0] = jointTrajectory.q1_trajectory.data[i];
        q_cur.data[1] = jointTrajectory.q2_trajectory.data[i];
        q_cur.data[2] = jointTrajectory.q3_trajectory.data[i];
        q_cur.data[3] = jointTrajectory.q4_trajectory.data[i];
        q_cur.data[4] = jointTrajectory.q5_trajectory.data[i];
        q_cur.data[5] = jointTrajectory.q6_trajectory.data[i];

        KDL::Frame frame_cur;
        fk_pos->JntToCart(q_cur, frame_cur, -1);

        if ( i == 1 )
        {
            this->M = frame_cur.M;
        }


        // Obtain current end-effector position from the FK
        KDL::JntArray q_dot_cur(6);
        q_dot_cur.data[0] = jointTrajectory.q1_dot_trajectory.data[i];
        q_dot_cur.data[1] = jointTrajectory.q2_dot_trajectory.data[i];
        q_dot_cur.data[2] = jointTrajectory.q3_dot_trajectory.data[i];
        q_dot_cur.data[3] = jointTrajectory.q4_dot_trajectory.data[i];
        q_dot_cur.data[4] = jointTrajectory.q5_dot_trajectory.data[i];
        q_dot_cur.data[5] = jointTrajectory.q6_dot_trajectory.data[i];

        KDL::JntArrayVel q_array_vel(6);
        q_array_vel.q = q_cur;
        q_array_vel.qdot = q_dot_cur;

        KDL::FrameVel frame_vel;
        fk_vel->JntToCart(q_array_vel, frame_vel, -1);

        x_dot_trajectory.push_back(frame_vel.p.v.x());
        y_dot_trajectory.push_back(frame_vel.p.v.y());
        z_dot_trajectory.push_back(frame_vel.p.v.x());

        x_trajectory.push_back(frame_cur.p.x());
        y_trajectory.push_back(frame_cur.p.y());
        z_trajectory.push_back(frame_cur.p.z());

    }

    // Create Matlab script to plot the solution
    ofstream file;
    string filename = "ur5_trajectory.m";
    file.open(filename.c_str());
    file << "% Results file from " __FILE__ << std::endl;
    file << "% Generated " __DATE__ " at " __TIME__ << std::endl;
    file << std::endl;
    
    // Save results to file
    file << "t = linspace(0," << jointTrajectory.q1_trajectory.data.size()*0.01 << "," << 
        jointTrajectory.q1_trajectory.data.size()    + ")" << std::endl;

    file << "q1_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q1_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q1_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q2_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q2_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q2_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q3_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q3_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q3_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q4_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q4_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q4_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q5_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q5_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q5_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q6_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q6_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q6_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;





    
    file << "q1_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q1_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q1_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q2_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q2_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q2_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q3_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q3_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q3_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q4_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q4_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q4_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q5_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q5_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q5_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q6_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q6_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q6_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;






    file << "q1_dot_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q1_dot_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q1_dot_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q2_dot_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q2_dot_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q2_dot_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q3_dot_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q3_dot_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q3_dot_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q4_dot_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q4_dot_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q4_dot_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q5_dot_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q5_dot_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q5_dot_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;
    file << "q6_dot_dot_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << jointTrajectory.q6_dot_dot_trajectory.data[i] << ",";
    }
    file << jointTrajectory.q6_dot_dot_trajectory.data[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;

    
    
    
    
    
    
    file << "x_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << x_trajectory[i] << ",";
    }
    file << x_trajectory[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;


    file << "x_d_dot = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << x_dot_trajectory[i] << ",";
    }
    file << x_dot_trajectory[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;


    file << "y_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << y_trajectory[i] << ",";
    }
    file << y_trajectory[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;


    file << "y_d_dot = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << y_dot_trajectory[i] << ",";
    }
    file << y_dot_trajectory[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;


    file << "z_d = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << z_trajectory[i] << ",";
    }
    file << z_trajectory[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;


    file << "z_d_dot = [";
    for ( int i = 0; i < jointTrajectory.q1_trajectory.data.size()-1; i++ )
    {
        file << z_dot_trajectory[i] << ",";
    }
    
    file << z_dot_trajectory[jointTrajectory.q1_trajectory.data.size()-1];
    file << "];" << std::endl;


    file << "figure;" << std::endl;
    file << "hold on;" << std::endl;

    file << "subplot(3, 1, 1)" << std::endl;
    file << "plot(t,q1_desired);" << std::endl;
    file << "xlabel('Time [s]');" << std::endl;
    file << "ylabel ('Joint position [rad]')" << std::endl;
    file << "title('Desired q1 trajectory')" << std::endl;

    file << "subplot(3, 1, 2)" << std::endl;
    file << "plot(t,q1_dot_desired);" << std::endl;
    file << "xlabel('Time [s]');" << std::endl;
    file << "ylabel ('Joint speed [rad/s]')" << std::endl;
    file << "title('Desired q1_dot trajectory')" << std::endl;

    file << "subplot(3, 1, 3)" << std::endl;
    file << "plot(t,q1_dot_dot_desired);" << std::endl;
    file << "xlabel('Time [s]');" << std::endl;
    file << "ylabel ('Joint accel [rad/s^2]')" << std::endl;
    file << "title('Desired q1_dot_dot trajectory')" << std::endl;


    //file << "legend('speed','pos','speed limit','throttle','Location','northwest');" << std::endl;

    file.close();
}

ORO_CREATE_COMPONENT(ur5_control) // register the RTT component
