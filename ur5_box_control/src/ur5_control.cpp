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

    // Initialize the internal joint state variable
    joint_state.resize(6);
    for (int i = 0; i < 6; i++)
    {
        joint_state(i) = 0.0;
    }

    addPort("CmdJntPos", port_cmd_jnt_pos);
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
    // If trajectory has been received
    if ( executeTrajectoryCmd )
    {
        // While we have not yet finished the trajectory
        if ( trajectoryIteration < trajectoryLength )
        {
            // get the joint values and write them to the port
            std_msgs::Float64MultiArray js;
            js.data.resize(6);

            if ( 0 ) //pushTraj && trajectoryIteration < 40 )
            {
                float x[47] = {0.6867, 0.6860, 0.6837, 0.6786, 0.6702, 0.6585,
                            0.6445, 0.6298, 0.6153, 0.6012, 0.5872, 0.5730, 0.5589,
                            0.5452, 0.5313, 0.5170, 0.5025, 0.4881, 0.4739, 0.4597,
                            0.4453, 0.4308, 0.4176, 0.4073, 0.4000, 0.3962, 0.3978,
                            0.4038, 0.4104, 0.4160, 0.4209, 0.4273, 0.4351, 0.4415,
                            0.4449, 0.4457, 0.4445, 0.4416, 0.4375, 0.4326, 0.4273,
                            0.4221, 0.4171, 0.4120, 0.4069, 0.4022, 0.4000};

                float y[47] = {0.1091, 0.1141, 0.1264, 0.1411, 0.1558, 0.1705,
                            0.1852, 0.2000, 0.2147, 0.2294, 0.2441, 0.2588, 0.2735,
                            0.2882, 0.3029, 0.3176, 0.3323, 0.3470, 0.3617, 0.3765,
                            0.3912, 0.4059, 0.4206, 0.4353, 0.4500, 0.4646, 0.4791,
                            0.4937, 0.5082, 0.5225, 0.5367, 0.5509, 0.5652, 0.5796,
                            0.5942, 0.6089, 0.6236, 0.6382, 0.6525, 0.6665, 0.6806,
                            0.6948, 0.7094, 0.7238, 0.7368, 0.7464, 0.7500};

                float z[47] = {0.3362, 0.3348, 0.3287, 0.3182, 0.3075, 0.2985, 
                            0.2903, 0.2826, 0.2745, 0.2656, 0.2553, 0.2438, 0.2314, 
                            0.2190, 0.2071, 0.1966, 0.1879, 0.1811, 0.1757, 0.1712,
                            0.1670, 0.1623, 0.1563, 0.1504, 0.1500, 0.1571, 0.1674,
                            0.1759, 0.1824, 0.1903, 0.2003, 0.2097, 0.2163, 0.2195,
                            0.2200, 0.2182, 0.2144, 0.2092, 0.2028, 0.1958, 0.1888,
                            0.1827, 0.1778, 0.1723, 0.1639, 0.1544, 0.1500};


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
            
            
            if ( abs(js.data[1]) < 3)
            {
                port_cmd_jnt_pos.write(js);
            }

            else
            {
                std::cout << "BAD ITERATION: " << trajectoryIteration << std::endl;
            }
            
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
