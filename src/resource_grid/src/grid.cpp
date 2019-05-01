#include <vector>
#include <ros/ros.h>
#include "pcc_srvs/SetPoint.h"
#include "pcc_srvs/GetPoint.h"
#include <iostream>
#include <math.h>
#include "geometry_msgs/Pose2D.h"
#include <queue>


struct Cell {
bool sample;
};

// comparator used intside the priority queue.
struct comparator{
    bool operator() (geometry_msgs::Pose2D a,geometry_msgs::Pose2D b ){
        return (a.theta<b.theta);
    }
};

//custom Types
typedef pcc_srvs::SetPoint SetPoint;
typedef pcc_srvs::GetPoint GetPoint;
typedef std::priority_queue<geometry_msgs::Pose2D,std::vector<geometry_msgs::Pose2D>, comparator> queue_t;


// Grid Attributes
std::vector<Cell> rows;
std::vector< std::vector<Cell> > grid;
std::size_t size = 222;
std::size_t offset = 110;
queue_t high_priority_points;

//sets the location of a sample in the grid.
bool sample_setter(SetPoint::Request& req,SetPoint::Response& res){
    if(hypot(req.point.x,req.point.y)<1.9){
        return true;
    }

    int x = (req.point.x*10)+offset;
    int y = (req.point.y*10)+offset;

    grid.at(x).at(y).sample=true;

    return true;


}
//unset the location of a sample inside the grid.
bool unset_sample(SetPoint::Request& req,SetPoint::Response& res){
    int x = (req.point.x*10)+offset;
    int y = (req.point.y*10)+offset;

    grid.at(x).at(y).sample=false;

}

//Statistic functinon

/*Looks into the portion of the grid given by the range in the parameters.
 * tries to find the average of the location where the most samples are
 * Sends that average location as a return value.
 */
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

//Retrieves best possible location in the grid for resource collection.
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



//INITIAL CODE
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

