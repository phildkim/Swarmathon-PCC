#include <vector>
#include <ros/ros.h>
#include "ccny_srvs/SetPoint.h"
#include "ccny_srvs/GetPoint.h"
#include <iostream>
#include <math.h>
#include "geometry_msgs/Pose2D.h"
#include <queue>

struct Cell {
bool sample;
};

struct comparator{
    bool operator() (geometry_msgs::Pose2D a,geometry_msgs::Pose2D b ){
        return (a.theta<b.theta);
    }
};

typedef ccny_srvs::SetPoint SetPoint;
typedef ccny_srvs::GetPoint GetPoint;
typedef std::priority_queue<geometry_msgs::Pose2D,std::vector<geometry_msgs::Pose2D>, comparator> queue_t;

std::vector<Cell> rows;
std::vector< std::vector<Cell> > grid;
std::size_t size = 222;
std::size_t offset = 110;
queue_t high_priority_points;

bool sample_setter(SetPoint::Request& req,SetPoint::Response& res){
    if(hypot(req.point.x,req.point.y)<1.9){
        return true;
    }

    int x = (req.point.x*10)+offset;
    int y = (req.point.y*10)+offset;

    grid.at(x).at(y).sample=true;

    return true;


}
bool unset_sample(SetPoint::Request& req,SetPoint::Response& res){
    int x = (req.point.x*10)+offset;
    int y = (req.point.y*10)+offset;

    grid.at(x).at(y).sample=false;

}
geometry_msgs::Pose2D perform_stats(int x_low,int x_high,int y_low, int y_high){
    int x_bar=0;
    int y_bar=0;
    int count=0;


    for(int m=x_low;m<=x_high;m++){
        for(int n=y_low;n<=y_high;n++){
            if(grid.at(m).at(n).sample){
                x_bar+=m;
                y_bar+=n;
                count++;
            }
        }
    }




    geometry_msgs::Pose2D point;
    if(count==0){
    point.x=0;
    point.y=0;
    point.theta=0;
    }else{
    point.x=(int)x_bar/count;
    point.y=(int)y_bar/count;
    point.theta=count;
    }

    return point;


}
bool get_point(GetPoint::Request& req,GetPoint::Response& res){
    high_priority_points = queue_t();


    for(int i=0; i<=size-12;i+=10){
        for(int j=0;j<=size-12;j+=10){
            high_priority_points.push(perform_stats(i,i+10,j,j+10));
        }
    }


    res.point=high_priority_points.top();
    if(res.point.theta==0){
        res.success=false;
        return true;
    }
    res.point.theta=0;
    res.point.x=(float)(res.point.x-(float)offset)/10.0;
    res.point.y=(float)(res.point.y-(float)offset)/10.0;
    res.success=true;

    return true;
}


int main(int argc, char **argv) {
    rows = std::vector<Cell>(size,Cell());
    rows.resize(size);
    grid = std::vector<std::vector<Cell> >(size,rows);
    grid.resize(size);


    ros::init(argc,argv,"Resource_Grid");
    ros::NodeHandle n;

    ros::ServiceServer set_sample = n.advertiseService("set_sample",sample_setter);
    ros::ServiceServer get_sample = n.advertiseService("get_point",get_point);
    ros::ServiceServer unset_samples = n.advertiseService("unset_sample",unset_sample);
    ros::spin();
    return 0;
}

