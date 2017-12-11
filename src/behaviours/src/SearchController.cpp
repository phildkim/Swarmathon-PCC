#include <algorithm>
#include "SearchController.h"
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

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {

  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
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
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

    //select new position 50 cm from current location
    if (first_waypoint)
    {
      first_waypoint = false;
      square_index = rng->uniformInteger(0, square_angles.size() - 1);
      std::transform(square_angles.begin(), square_angles.end(), square_waypoints.begin(),
            [](float angle) -> Point {
                  Point newPoint;
                  newPoint.theta = angle;
                  newPoint.x = sqrt(12.5) * cos(angle);
                  newPoint.y = sqrt(12.5) * sin(angle);
                  return newPoint;
            });
      
      searchLocation = square_waypoints.at(square_index);
      
      /*
      std::for_each(square_waypoints.begin(), square_waypoints.end(), [](Point p) {
            p.theta += M_PI/4;
      });
      */
    }
    else
    {
      //select new heading from Gaussian distribution around current heading
      square_index = (square_index + 1) % square_waypoints.size();
      searchLocation = square_waypoints.at(square_index);
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
  }

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


