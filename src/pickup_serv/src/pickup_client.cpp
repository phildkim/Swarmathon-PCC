#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "ccny_srvs/GetPickup.h"
#include "ccny_srvs/SetPickup.h"


int main(int argc , char ** argv)
{
ros::init(argc,argv,"client_pickup");
ros::NodeHandle	n;
ros::ServiceClient client = n.serviceClient<ccny_srvs::SetPickup>("pickupsetter");
ros::ServiceClient client2 = n.serviceClient<ccny_srvs::GetPickup>("pickupgetter");

ccny_srvs::SetPickup ms;
geometry_msgs::Pose2D po; 
po.x=atoll(argv[1]);
po.y=atoll(argv[2]);
po.theta = 0;

ms.request.point = po;

ccny_srvs::GetPickup ms2;
ms2.request.pickup = true;

if(client.call(ms)){

ROS_INFO("Sum: %s", ms.response.success?"true":"false");

}
if(client2.call(ms2)){

    ROS_INFO("Sum: %d %d",(int)ms2.response.point.x,(int)ms2.response.point.y);

}


return 0;

}
