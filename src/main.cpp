#include "point_labeling/Process.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_labeling");
    ros::NodeHandle nh;
    Process proc;

    proc.subscribe();
    ros::spin();

    return 0;
}