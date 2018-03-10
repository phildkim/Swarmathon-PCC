#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Result.h"
#include <ros/ros.h>
#include "ccny_srvs/GetPickup.h"
#include "ccny_srvs/GetStatus.h"
#include "ccny_srvs/SetStatus.h"

typedef ccny_srvs::GetStatus Gstatus;
typedef ccny_srvs::SetStatus Sstatus;
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
  virtual string toString()=0;
protected:

  //Looks at external data and determines if an interrupt must be thrown
  //or if the controller should be polled
  virtual void ProcessData() = 0;

  //Node Handle
  ros::NodeHandle nm;
  // Service List
  ros::ServiceClient set_sample = nm.serviceClient <Sstatus>("mark_samples");
  ros::ServiceClient set_robot= nm.serviceClient<Sstatus>("set_occupied_rover");
  ros::ServiceClient set_object=nm.serviceClient<Sstatus>("set_occupied_object");
  ros::ServiceClient set_route=nm.serviceClient<Sstatus>("set_occupied_planned_route");
  ros::ServiceClient is_robot=nm.serviceClient<Gstatus>("is_occupied_rover");
  ros::ServiceClient is_obstacle=nm.serviceClient<Gstatus>("is_occupied_obstacle");
  ros::ServiceClient is_any=nm.serviceClient<Gstatus>("is_occupied_any");
  ros::ServiceClient is_route=nm.serviceClient<Gstatus>("is_occupied_planned_route");
  ros::ServiceClient has_sample=nm.serviceClient<Gstatus>("has_samples");
  ros::ServiceClient has_searched=nm.serviceClient<Gstatus>("has_been_searched");
  ros::ServiceClient pickup_req=nm.serviceClient<ccny_srvs::GetPickup>("pickupgetter");

};

#endif // CONTROLLER_H
