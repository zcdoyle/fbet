#include <ros/ros.h>
#include <fbet/FBETServer.h>

using namespace fbet_server;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fbet_server");

    FBETServer fbet;
    
    ros::spin();
    return 0;
}