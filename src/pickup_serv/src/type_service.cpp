#include <ros/ros.h>
#include "pcc_srvs/RobotType.h"

typedef pcc_srvs::RobotType robottype;
static int id=0;

bool increment(robottype::Request& req,robottype::Response& res){
    ++id;
    res.id = id;
    return true;
}

bool robot_size(robottype::Request& req,robottype::Response& res){
    res.id = id;
    return true;
}

int main (int argc,char** argv){
    ros::init(argc,argv,"robot_id");
	ros::NodeHandle n;
	ros::ServiceServer service1 = n.advertiseService("getID",increment);
    ros::ServiceServer service2 = n.advertiseService("num_of_robots",robot_size);
	ros::spin();

}


