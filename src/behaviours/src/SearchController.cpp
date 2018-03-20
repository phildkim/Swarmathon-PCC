#include "SearchController.h"
#include <algorithm>
#include <math.h>
#include <angles/angles.h>



SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;

  square_angles = { M_PI/4, 3 * M_PI/4, 5 * M_PI/4, 7 * M_PI/4 };
}

void SearchController::Reset() {
  result.reset = false;
}

Result SearchController::PickupWork(){
    ROS_WARN("PICKING UP");
    if (!result.wpts.waypoints.empty()) {

      if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.3) {
        attemptCount = 0;
      }

    }

    if (attemptCount > 0 && attemptCount < 5) {
      attemptCount++;
      if (succesfullPickup) {
        succesfullPickup = false;
        attemptCount = 1;
      }
      return result;
    }
    else if (attemptCount >= 5 || attemptCount == 0)
    {
      get_pickuplist_size.call(pickupGet);
      if(pickupGet.response.size==0){
          SearchWork();
          result.change_type=true;
          return result;
      }
      attemptCount = 1;


      result.type = waypoint;
      Point  searchLocation;
      float radius;
      float angle;
      float curr_angle;

    if (first_waypoint)
    {
      first_waypoint = false;

      pickupGet.request.pickup = true;
      pickupGet.request.point.x= currentLocation.x;
      pickupGet.request.point.y=currentLocation.y;

      get_pickup.call(pickupGet);
      if(!pickupGet.response.empty){
          searchLocation.x=pickupGet.response.point.x;
          searchLocation.y=pickupGet.response.point.y;

          statusSet.request.x= pickupGet.response.point.x;
          statusSet.request.y= pickupGet.response.point.x;
          statusSet.request.data=0;
          set_sample.call(statusSet);

      }else{
          if(currentLocation.x!=0){
          curr_angle = atan(currentLocation.y/currentLocation.x);
          if(currentLocation.x<0&&currentLocation.y<0){
              curr_angle+=M_PI;
          }else if(currentLocation.x<0&&currentLocation.y>0){
              curr_angle+=M_PI;
          }else if(currentLocation.x>0&&currentLocation.y<0){
              curr_angle+=2*M_PI;
          }
          }
          radius = +rng->uniformReal(1,2.6);
          angle = (curr_angle+rng->uniformReal(-M_PI/3,M_PI/3));
       searchLocation.x = radius*cos(angle);
       searchLocation.y = radius*sin(angle);



    }
    }else{
        pickupGet.request.pickup = true;
        get_pickup.call(pickupGet);
        if(!pickupGet.response.empty){
            searchLocation.x=pickupGet.response.point.x;
            searchLocation.y=pickupGet.response.point.y;

            statusSet.request.x= pickupGet.response.point.x;
            statusSet.request.y= pickupGet.response.point.x;
            statusSet.request.data=0;
            set_sample.call(statusSet);
        }else{
            if(currentLocation.x!=0){
            curr_angle = atan(currentLocation.y/currentLocation.x);
            if(currentLocation.x<0&&currentLocation.y<0){
                curr_angle+=M_PI;
            }else if(currentLocation.x<0&&currentLocation.y>0){
                curr_angle+=M_PI;
            }else if(currentLocation.x>0&&currentLocation.y<0){
                curr_angle+=2*M_PI;
            }
            }
            radius = +rng->uniformReal(1,2.6);
            angle = (curr_angle+rng->uniformReal(-M_PI/3,M_PI/3));
         searchLocation.x = radius*cos(angle);
         searchLocation.y = radius*sin(angle);

    }
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    reset_msgs();
    return result;
}

}
Result SearchController::SearchWork(){
    ROS_WARN("Searching");
    if (!result.wpts.waypoints.empty()) {

        if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.3) {
            attemptCount = 0;
        }

    }

    if (attemptCount > 0 && attemptCount < 5) {
      attemptCount++;
      if (succesfullPickup) {
        succesfullPickup = false;
        attemptCount = 1;
      }
      return result;
    }
    else if (attemptCount >= 5 || attemptCount == 0)
    {
      get_pickuplist_size.call(pickupGet);
      if(pickupGet.response.size>6){
          PickupWork();
          result.change_type=true;
          return result;
      }
      attemptCount = 1;
      if(initial){
          Point initial_point;
          initial_point.x=currentLocation.x*2.5;
          initial_point.y=currentLocation.y*2.5;
          result.type = waypoint;
          result.wpts.waypoints.clear();
          result.wpts.waypoints.insert(result.wpts.waypoints.begin(),initial_point);
          initial=false;
          return result;
      }

      result.type = waypoint;
      Point  searchLocation;
      float radius;
      float angle;
      float curr_angle;
    if (first_waypoint)
    {
      first_waypoint = false;
      if(currentLocation.x!=0){
      curr_angle = atan(currentLocation.y/currentLocation.x);
      if(currentLocation.x<0&&currentLocation.y<0){
          curr_angle+=M_PI;
      }else if(currentLocation.x<0&&currentLocation.y>0){
          curr_angle+=M_PI;
      }else if(currentLocation.x>0&&currentLocation.y<0){
          curr_angle+=2*M_PI;
      }
      }

      radius = +rng->uniformReal(3.2,5);
      angle = (curr_angle+rng->uniformReal(-M_PI/3,M_PI/3));
      searchLocation.x = radius*cos(angle);
      searchLocation.y = radius*sin(angle);
    }else{
        if(currentLocation.x!=0){
        curr_angle = atan(currentLocation.y/currentLocation.x);
        if(currentLocation.x<0&&currentLocation.y<0){
            curr_angle+=M_PI;
        }else if(currentLocation.x<0&&currentLocation.y>0){
            curr_angle+=M_PI;
        }else if(currentLocation.x>0&&currentLocation.y<0){
            curr_angle+=2*M_PI;
        }
        }
        radius = +rng->uniformReal(3.2,5);
        angle = (curr_angle+rng->uniformReal(-M_PI/3,M_PI/3));
        searchLocation.x = radius*cos(angle);
        searchLocation.y = radius*sin(angle);
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    return result;
    }

}

Result SearchController::DoWork() {
return (this->*SearchController::Work)();
}

void SearchController::SetCenterLocation(Point centerLocation) {

  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;

  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }

}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}

void SearchController::setType(){
    if(Work==&SearchController::PickupWork){
        Work=&SearchController::SearchWork;
    }else{
        Work=&SearchController::PickupWork;
    }
}
