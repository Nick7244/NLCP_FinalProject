#include <box_position_subscriber/box_position_subscriber.hpp> 
#include <ros/ros.h>

int main( int argc, char** argv )
{
    // Initialize ROS node and create node handle
    ros::init(argc, argv, "boxPosSubNode");
    ros::NodeHandle nh_sub;

    // Pass node handle to subscriber class to create subscriber
    box_pos_subscriber sub(nh_sub);

    ros::spinOnce();

    // ROS command to pause the program, preventing it from exiting,
	// allowing the subscriber to do its job
    while ( ros::ok() )
    {
        std::vector<float> box_pos = sub.getBoxPos();
        std::cout << "Box Position: x = " << box_pos.at(0) << ", y = " << 
            box_pos.at(1) << ", z = " << box_pos.at(2) << std::endl;

        ros::spinOnce();
    }
    

    return 0;
}