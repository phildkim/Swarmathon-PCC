#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <queue>
#include "ccny_srvs/GetPickup.h"
#include "ccny_srvs/SetPickup.h"
#include <math.h>
std::list<geometry_msgs::Pose2D> pickuplist;

typedef ccny_srvs::SetPickup setpickup;
typedef ccny_srvs::GetPickup getpickup;

geometry_msgs::Pose2D comparison_point;
struct Comparator {
    bool operator ()(const geometry_msgs::Pose2D a, const geometry_msgs::Pose2D b){
    double dist_a=hypot(a.x-comparison_point.x,a.y-comparison_point.y);
    double dist_b=hypot(b.x-comparison_point.x,b.y-comparison_point.y);
    return (dist_a>dist_b)?false:true;
    }
};

bool setpoints(geometry_msgs::Pose2D msg){

    pickuplist.push_back(msg);
    return true;

}

bool setCords(setpickup::Request& req, setpickup::Response& res){

res.success = setpoints(req.point);
return true;

}
bool getCords(getpickup::Request& req,getpickup::Response& res){
comparison_point=req.point;

if(req.pickup && !pickuplist.empty()){
    pickuplist.sort(Comparator());
    res.point=pickuplist.front();
    pickuplist.pop_front();
    res.empty=false;
}else{
    res.empty= true;

}

return true;

}
bool getSize(getpickup::Request& req,getpickup::Response& res){
    res.size=pickuplist.size();
    return true;
}



int main(int argc, char **argv){
        ros::init(argc, argv ,"pickup_server");
	ros::NodeHandle n;
	
        ros::ServiceServer service1 =n.advertiseService("pickup_setter",setCords);
        ros::ServiceServer service2 =n.advertiseService("pickup_getter",getCords);
        ros::ServiceServer service3 =n.advertiseService("list_size",getSize);

	ros::spin();



}
