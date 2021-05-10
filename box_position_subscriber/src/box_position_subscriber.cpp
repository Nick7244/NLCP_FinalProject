#include <box_position_subscriber/box_position_subscriber.hpp> 

box_pos_subscriber::box_pos_subscriber( ros::NodeHandle& nh )
    : nh(nh)
{
    // node_handle.subscribe(topic_name, queue_size, function_pointer, obj) returns a subscriber 
    // that is subscribed to <topic_name>, and calls <function_pointer> on the object <obj> whenever 
    // a message has arrived
    sub = nh.subscribe("/box_position", 10, &box_pos_subscriber::callback, this);

    for( int i = 0; i < 3; i++ )
    {
        box_pos.push_back(0.0);
    }
}

box_pos_subscriber::~box_pos_subscriber() {}

// Function that automatically gets called whenever new data comes in
void box_pos_subscriber::callback( const ur5_box_msgs::box_position& msg )
{
    // Update internal state for box position
    box_pos[0] = msg.x.data;
    box_pos[1] = msg.y.data;
    box_pos[2] = msg.z.data;
}

std::vector<float> box_pos_subscriber::getBoxPos()
{
    return box_pos;
}