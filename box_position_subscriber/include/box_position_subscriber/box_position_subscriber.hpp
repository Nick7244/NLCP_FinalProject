#include <ros/ros.h>
#include <ur5_box_msgs/box_position.h>
#include <vector>

// Class definition for subscriber class
class box_pos_subscriber {

    private:

        ros::NodeHandle nh;
        ros::Subscriber sub;
        std::vector<float> box_pos;

    public: 

        box_pos_subscriber( ros::NodeHandle& nh );
        ~box_pos_subscriber();

        void callback( const ur5_box_msgs::box_position& msg );
        std::vector<float> getBoxPos();
};