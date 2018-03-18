#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "ccny_srvs/GetPickup.h"
#include "ccny_srvs/SetPickup.h"
#include "ccny_srvs/RobotType.h"


int main(int argc , char ** argv)
{
ros::init(argc,argv,"client_pickup");
ros::NodeHandle	n;
ros::ServiceClient client = n.serviceClient<ccny_srvs::SetPickup>("pickupsetter");
ros::ServiceClient client2 = n.serviceClient<ccny_srvs::GetPickup>("pickupgetter");
ros::ServiceClient client3 = n.serviceClient<ccny_srvs::RobotType>("getID");
ccny_srvs::SetPickup ms;
geometry_msgs::Pose2D po; 
geometry_msgs::Pose2D po1;

po1.x=atoll(argv[1]);
po1.y=atoll(argv[2]);
po1.theta = 0;
po.x=2;
po.y=2;
ms.request.point=po;
client.call(ms);
po.x=3;
po.y=4;
ms.request.point=po;
client.call(ms);
po.x=-1;
po.y=-2;
ms.request.point=po;
client.call(ms);




ccny_srvs::GetPickup ms2;



ms2.request.pickup = true;
po.x=2;
po.y=2;
ms2.request.point=po1;
ccny_srvs::RobotType m3;

//if(client.call(ms)){

//ROS_INFO("Sum: %s", ms.response.success?"true":"false");

//}
if(client2.call(ms2)){

    ROS_INFO("Sum: %d %d",(int)ms2.response.point.x,(int)ms2.response.point.y);

}
if(client3.call(m3)){

    ROS_INFO("ID: %d",m3.response.id);

}


return 0;

}
