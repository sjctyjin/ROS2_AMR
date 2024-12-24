#include "huanyu_joy/huanyu_joy.h"



huanyu_joy::huanyu_joy()
    : Node("huanyu_joy_node")
{
    joystick_fd = -1;
    leftVertical    = rightVertical  = 0;
    rightHorizontal = leftHorizontal = 0;
    publicTopic = true;

    this->declare_parameter<std::string>("joystick_device", "/dev/input/js0");
    this->get_parameter("joystick_device", this->joystick_device);
    this->declare_parameter<float>("maxLinear_x", 0.4);
    this->get_parameter("maxLinear_x", this->maxLinear_x);
    this->declare_parameter<float>("maxLinear_y", 0.4);
    this->get_parameter("maxLinear_y", this->maxLinear_y);
    this->declare_parameter<float>("maxAngular_z", 1.5);
    this->get_parameter("maxAngular_z", this->maxAngular_z);

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1000);


    joystick_fd = open(joystick_device.c_str(), O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
    if (joystick_fd < 0)
    {
        RCLCPP_INFO(this->get_logger(), "Open joystick device success!");
    }
}

 huanyu_joy::~huanyu_joy()
{
    close(joystick_fd);
}


void huanyu_joy::publish_joystick_event()
{
    unsigned int JS_EVENT_AXIS_COUNT = 0;
    rclcpp::Node::SharedPtr node_(this); // 创建基类指针，指向子类对象this   
    rclcpp::WallRate rosSleep(20);
    while(rclcpp::ok())
    {
        int bytes = read(joystick_fd, &jse, sizeof(jse));
        if(bytes != sizeof(jse) && bytes != -1)
        {
            RCLCPP_INFO(this->get_logger(), "Read joystick file sizeof error!");
        }
        jse.type &= ~JS_EVENT_INIT;
        if (jse.type == JS_EVENT_AXIS)
        {
			(JS_EVENT_AXIS_COUNT<=10)?(memset(&jse, 0, sizeof(jse)),JS_EVENT_AXIS_COUNT++):(JS_EVENT_AXIS_COUNT = 255);
            switch (jse.number)
            {
                case 0: this->leftHorizontal  = jse.value; break;
                case 1: this->leftVertical    = jse.value; break;
                case 2: this->rightHorizontal = jse.value; break;
                case 3: this->rightVertical   = jse.value; break;

                default: break;
            }
        }
        if (jse.type == JS_EVENT_BUTTON && jse.value == JS_EVENT_BUTTON_DOWN)
        {
            switch(jse.number)
            {
                case 6:maxLinear_x += 0.1;
                    break; 
                case 8:(maxLinear_x > 0.1)?(maxLinear_x -= 0.1):(maxLinear_x = maxLinear_x);
                    break;

                case 1:maxLinear_y += 0.1;
                    break;
                case 3:(maxLinear_y > 0.1)?(maxLinear_y -= 0.1):(maxLinear_y = maxLinear_y);
                    break;

                case 7:maxAngular_z += 0.1;
                    break;
                case 9:(maxAngular_z > 0.1)?(maxAngular_z -= 0.1):(maxAngular_z = maxAngular_z);
                    break;

                case 10: publicTopic = !publicTopic;
                        if(publicTopic)
                            {RCLCPP_INFO(this->get_logger(), "Open publish cmd_vel!");}
                        else
                            {RCLCPP_INFO(this->get_logger(), "Close publish cmd_vel!");}
                    break;
                default: break;   
            }
            if(jse.number != 10)
            {
                RCLCPP_INFO(this->get_logger(),"maxX, maxY, maxZ = [%f %f %f]",maxLinear_x, maxLinear_y, maxAngular_z);
            }
        }
        memset(&jse, 0, sizeof(jse));
        if (publicTopic)
        {
            vel_msg_.linear.y  = maxLinear_y /32767 * leftHorizontal  * -1;
            vel_msg_.linear.x  = maxLinear_x /32767 * leftVertical    * -1;
            vel_msg_.angular.z = maxAngular_z/32767 * rightHorizontal * -1;
            
            cmd_vel_pub->publish(vel_msg_);
        }


        rosSleep.sleep();
        rclcpp::spin_some(node_);
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  
    huanyu_joy  huanyu_joystick;
    RCLCPP_INFO(huanyu_joystick.get_logger(),"[ZHOUXUEWEI] joy controller node start! ");

    huanyu_joystick.publish_joystick_event();
    rclcpp::shutdown();
    return 0;
}
