#include <gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo {

    class BoxPlugin : public ModelPlugin {

        public:
            BoxPlugin();
            ~BoxPlugin();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void UpdateChild();

        private:
            physics::WorldPtr world_ptr_;
            GazeboRosPtr gazebo_ros_;
            event::ConnectionPtr update_connetion_;
            std::string box_name;
            ros::Publisher pub;
            
  };

}