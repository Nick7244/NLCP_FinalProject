#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ur5_box_msgs/ur5_trajectory.h>

#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>

#include <trajectory_seed_generator/ur5_reflexxes_trajectory.hpp>


// This is the component that actuall connects to the hardware
class ur5_control : public RTT::TaskContext {

    private :

        ros::NodeHandle nh;
        ros::Subscriber sub_traj;
        ros::Subscriber sub_js;
        ros::Subscriber sub_torques;

        RTT::OutputPort<std_msgs::Float64MultiArray> port_cmd_jnt_pos;

        RTT::OutputPort<std_msgs::Float64MultiArray> port_cmd_jnt_torque;

        int trajectoryIteration;
        int trajectoryLength;
        bool trajectoryReceived;
        bool executeTrajectoryCmd;

        bool pushTraj;

        KDL::Tree tree;
        KDL::Chain chain;
        KDL::ChainFkSolverPos* fk_pos;
        KDL::ChainFkSolverVel* fk_vel;
        KDL::ChainIkSolverPos* ik_pos;
        KDL::ChainIkSolverVel* ik_vel;

        KDL::Rotation M;

        ur5_box_msgs::ur5_trajectory jointTrajectory;
        
        KDL::JntArray joint_state;

        std_msgs::Float64MultiArray cmd_Torques;

        ur5_reflexxes_trajectory* trajectorySeedGenerator;

        void generateMatlab();


    public : 

        ur5_control( const std::string& name );
        ~ur5_control() {}

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();

        void TPose();
        void generateTrajectory();
        void executeTrajectory();

        void trajectoryCallback( const ur5_box_msgs::ur5_trajectory& trajectory );
        void jointStateCallback(const sensor_msgs::JointState& js );
        void cmdTorqueCallback( const sensor_msgs::JointState& torques );

};