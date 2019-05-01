#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "Result.h"
#include "pcc_srvs/GetPickup.h"
#include "pcc_srvs/SetPickup.h"
#include "pcc_srvs/GetStatus.h"
#include "pcc_srvs/SetStatus.h"
#include "pcc_srvs/GetStatistic.h"
#include "pcc_srvs/SetStatistic.h"
#include "pcc_srvs/GetMVPoint.h"
#include "pcc_srvs/RobotType.h"
#include <ros/ros.h>
typedef pcc_srvs::GetStatus GStatus;
typedef pcc_srvs::SetStatus SStatus;
typedef pcc_srvs::SetPickup SPickup;
typedef pcc_srvs::GetPickup GPickup;
typedef pcc_srvs::GetStatistic GStatistic;
typedef pcc_srvs::SetStatistic SStatistic;
typedef pcc_srvs::GetMVPoint GMVP;
typedef pcc_srvs::RobotType GRobnum;

/*
 * This class is meant to serve as a template for all Controllers,
 * including new Controllers defined by each team.
 *
 * It also exists so that Controllers have a generic interface to
 * use for processing interrupts in LogicController.
 */
class Controller {

public:
  Controller() {}
  ~Controller() {}
  //Resets internal state to defaults
  virtual void Reset() = 0;
  //Determines what action should be taken based on current
  //internal state and data
  virtual Result DoWork() = 0;
  //Returns whether or not an interrupt must be thrown
  virtual bool ShouldInterrupt() = 0;
  //Returns whether or not a controller should be polled for a Result
  virtual bool HasWork() = 0;
  //to do check this enum type
  virtual string toString() = 0;

protected:
  //Looks at external data and determines if an interrupt must be thrown
  //or if the controller should be polled
  virtual void ProcessData() = 0;

  enum change{
         OBSTACLE=1,
         ROVER=1<<1,
         ROUTE=1<<2,
         SAMPLE=1<<3,
         ANY=1|1<<1|1<<2
     };

    // Service List
    // All Services that are to be used by Controllers must go here.
    ros::ServiceClient set_sample;
    ros::ServiceClient set_robot;
    ros::ServiceClient set_obstacle;
    ros::ServiceClient set_route;
    ros::ServiceClient set_searched;
    ros::ServiceClient is_robot;
    ros::ServiceClient is_obstacle;
    ros::ServiceClient is_any;
    ros::ServiceClient is_route;
    ros::ServiceClient has_sample;
    ros::ServiceClient has_searched;
    ros::ServiceClient get_pickup;
    ros::ServiceClient set_pickup;
    ros::ServiceClient get_pickuplist_size;
    ros::ServiceClient get_traversal_cost;
    ros::ServiceClient get_desirability_index;
    ros::ServiceClient update_traversal_cost;
    ros::ServiceClient update_desirability_index;
    ros::ServiceClient has_updated;
    ros::ServiceClient has_new_occupancy_data;
    ros::ServiceClient get_num_samples;
    ros::ServiceClient flag_new_occupancy_data;
    ros::ServiceClient update_num_samples;
    ros::ServiceClient get_MVP;
    ros::ServiceClient get_rob_num;
    //Reusable Messages
    SStatistic statisticSet;
    GStatistic statisticGet;
    SStatus statusSet;
    GStatus statusGet;
    SPickup pickupSet;
    GPickup pickupGet;
    GMVP MVPget;
    GRobnum robnumGet;

    //Initialize all services for a specific Controller.
    //Has issues initializing with the initialization of
    //the controller class since ros::init was being
    //called after and not before
    void initialize_services(){
        ros::NodeHandle nm;
        //Stats Setters
        update_traversal_cost = nm.serviceClient<SStatistic>("update_traversal_cost");
        update_desirability_index = nm.serviceClient<SStatistic>("update_desirability_index");

        //Stats Getter
        get_traversal_cost = nm.serviceClient<GStatistic>("get_traversal_cost");
        get_desirability_index = nm.serviceClient<GStatistic>("get_desirability_index");

        // Status Setters
        set_obstacle = nm.serviceClient<SStatus>("set_occupied_obstacle");
        set_robot = nm.serviceClient<SStatus>("set_occupied_rover");
        set_route = nm.serviceClient<SStatus>("set_occupied_planned_route");
        set_sample = nm.serviceClient <SStatus>("mark_samples");
        set_searched = nm.serviceClient<SStatus>("mark_searched");
        flag_new_occupancy_data = nm.serviceClient<SStatus>("flag_new_occupancy_data");
        update_num_samples = nm.serviceClient<SStatus>("update_num_of_samples");

        // Status Getters
        is_any = nm.serviceClient<GStatus>("is_occupied_any");
        is_obstacle = nm.serviceClient<GStatus>("is_occupied_obstacle");
        is_robot = nm.serviceClient<GStatus>("is_occupied_rover");
        is_route = nm.serviceClient<GStatus>("is_occupied_planned_route");
        has_sample = nm.serviceClient<GStatus>("has_samples");
        has_searched = nm.serviceClient<GStatus>("has_been_searched");
        has_updated = nm.serviceClient<GStatus>("has_recently_updated");
        has_new_occupancy_data = nm.serviceClient<GStatus>("has_new_occupancy_data");
        get_num_samples = nm.serviceClient<GStatus>("get_num_of_samples");

        //Pickup List
        get_pickup = nm.serviceClient<GPickup>("pickup_getter");
        set_pickup = nm.serviceClient<SPickup>("pickup_setter");
        get_pickuplist_size = nm.serviceClient<GPickup>("list_size");

        //MVP
        get_MVP = nm.serviceClient<GMVP>("most_valuable_point");

        //number of robots
        get_rob_num = nm.serviceClient<GRobnum>("num_of_robots");
    }


    //Reset all messages to default values.
    void reset_msgs(){
        // statistics
        statisticSet.request.x = 0;
        statisticSet.request.y = 0;
        statisticSet.request.data = 0;
        statisticGet.request.x = 0;
        statisticGet.request.y = 0;
        // status
        statusSet.request.x = 0;
        statusSet.request.y = 0;
        statusSet.request.data = change::ROVER;
        statusGet.request.x = 0;
        statusGet.request.y = 0;
        // pickup
        pickupGet.request.pickup = false;
        pickupGet.request.point.x = 0;
        pickupGet.request.point.y = 0;
        pickupSet.request.point.x = 0;
        pickupSet.request.point.y = 0;
    }
};

#endif // CONTROLLER_H
