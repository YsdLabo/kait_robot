#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class DS4WiredTwistPublisher
{
private:
    ros::NodeHandle nh;
    ros::Publisher  cmd_pub;
    ros::Subscriber joy_sub;
    ros::Timer timer;
    sensor_msgs::Joy last_joy;

public:
    DS4WiredTwistPublisher()
    {
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        joy_sub = nh.subscribe("joy", 10, &DS4WiredTwistPublisher::joyCallback, this);
        timer = nh.createTimer(ros::Duration(0.1), &DS4WiredTwistPublisher::timerCallback, this);
    }
    
    void joyCallback(const sensor_msgs::Joy& joy_msg)
    {
        last_joy = joy_msg;
    }
    
    void timerCallback(const ros::TimerEvent& e)
    {
        geometry_msgs::Twist cmd_vel;
        
        if(last_joy.axes.size() == 8) {
            cmd_vel.linear.x = last_joy.axes[1];
            cmd_vel.linear.y = last_joy.axes[0];
            cmd_vel.angular.z = 0.25 * (last_joy.axes[5] - last_joy.axes[2]);
        }
        
        cmd_pub.publish(cmd_vel);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ds4_wired_twist_publisher");
    DS4WiredTwistPublisher  twist_publishre;
    ros::spin();
    return 0;
}
