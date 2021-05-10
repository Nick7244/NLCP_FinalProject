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

        RTT::OutputPort<std_msgs::Float64MultiArray> port_cmd_jnt_pos;

        int trajectoryIteration;
        int trajectoryLength;
        bool trajectoryReceived;
        bool executeTrajectoryCmd;

        ur5_box_msgs::ur5_trajectory jointTrajectory;
        
        KDL::JntArray joint_state;

        ur5_reflexxes_trajectory* trajectorySeedGenerator;


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

};