#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER
#include <array>
#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include <functional>
/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {
public:
  SearchController();
  void Reset() override;
  // performs search pattern
  Result DoWork() override;
  Result (SearchController::* Work)()= &SearchController::SearchWork;
  Result PickupWork();
  Result SearchWork();
  bool ShouldInterrupt() override;
  bool HasWork() override;
  string toString(){return "SearchController";}
  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();
  void setRecruitmentLocation(Point p);
  void setType ();
  void init_services(){
      initialize_services();
  }
protected:
  void ProcessData();
private:
  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int square_index;
  std::array<float, 4> square_angles;
  std::array<Point, 4> square_waypoints;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;
  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;
  bool initial = true;
  float semi= 6.7;
  float mega = 10.5;
  float map_size=semi;
};

#endif /* SEARCH_CONTROLLER */
