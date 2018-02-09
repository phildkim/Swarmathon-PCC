#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <queue>
#include "ccny_srvs/GetPickup.h"
#include "ccny_srvs/SetPickup.h"

std::queue<geometry_msgs::Pose2D> pickuplist;

typedef ccny_srvs::SetPickup setpickup;
typedef ccny_srvs::GetPickup getpickup;
bool setpoints(geometry_msgs::Pose2D msg){

    pickuplist.push(msg);
    return true;

}
bool setcords(setpickup::Request& req, setpickup::Response& res){

res.success = setpoints(req.point);
return true;

}
bool getcords(getpickup::Request& req,getpickup::Response& res){

if(req.pickup && !pickuplist.empty()){
res.point=pickuplist.front();
pickuplist.pop();
res.empty=false;
}else{
    res.empty= true;

}

return true;

}



int main(int argc, char **argv){
        ros::init(argc, argv ,"pickup_server");
	ros::NodeHandle n;
	
        ros::ServiceServer service1 =n.advertiseService("pickupsetter",setcords);
        ros::ServiceServer service2 =n.advertiseService("pickupgetter",getcords);

	ros::spin();



}
