#include <ros/ros.h>
#include "ccny_srvs/RobotType.h"

typedef ccny_srvs::RobotType robottype;
static int id=0;
bool increment(robottype::Request& req,robottype::Response& res){

id++;
res.id=id;
return true;

}

int main (int argc,char** argv){
        ros::init(argc,argv,"ID");
	ros::NodeHandle n;
	ros::ServiceServer service1 = n.advertiseService("getID",increment);
	ros::spin();

}


