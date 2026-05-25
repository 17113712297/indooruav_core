#include <ros/ros.h>

#include "indooruav_core/event_response.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "indooruav_core");

    EventResponder event_responder;

    ros::spin();

    return 0;
}
