#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "ccny_srvs/GetPoint.h"
#include "ccny_srvs/SetPoint.h"


int main(int argc, char** argv){
    ros::init(argc,argv,"grid_client");
    ros::NodeHandle n;
    ros::ServiceClient cl1 = n.serviceClient<ccny_srvs::SetPoint>("set_sample");
    ros::ServiceClient cl2 = n.serviceClient<ccny_srvs::GetPoint>("get_point");
    ccny_srvs::SetPoint ms;
    ms.request.point.x=-11;
    ms.request.point.y=-11;
    cl1.call(ms);
    ms.request.point.x=-10.5;
    ms.request.point.y=-10.5;
    cl1.call(ms);
    ms.request.point.x=-9;
    ms.request.point.y=-9;
    cl1.call(ms);
    ms.request.point.x=1;
    ms.request.point.y=1;
    cl1.call(ms);
    ms.request.point.x=2;
    ms.request.point.y=2;
    cl1.call(ms);

    ccny_srvs::GetPoint ms2;
    cl2.call(ms2);

    ROS_INFO("%d",(int)ms2.response.success);


    return 0;
}
