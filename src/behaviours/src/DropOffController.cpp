#include "DropOffController.h"
#include <std_msgs/String.h>
#include <ros/console.h>


DropOffController::DropOffController() {

  reachedCollectionPoint = false;
  result.type = behavior;
  result.b = wait;
  result.wristAngle = 0.7;
  result.reset = false;
  interrupt = false;
  circularCenterSearching = false;
  spinner = 0;
  centerApproach = false;
  seenEnoughCenterTags = false;
  prevCount = 0;

  countLeft = 0;
  countRight = 0;
  returnpoint= false;

  isPrecisionDriving = false;
  startWaypoint = false;
  timerTimeElapsed = -1;

}

DropOffController::~DropOffController() {

}

Result DropOffController::DoWork(){
return (this->*DropOffController::Work)();
}
Result DropOffController::SearchWork(){

    if(!initial_test && !dropset){
        if(hypot(currentLocation.x,currentLocation.y)<3.1){
        PickupWork();
        return result;
        }else{
          initial_test=true;
        }
    }
    int count = countLeft + countRight;

    if(timerTimeElapsed > -1) {

      long int elapsed = current_time - returnTimer;
      timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
    }else{
        dropofftimer=current_time/1e3;
     }
     if(((current_time/1e3)-dropofftimer)>20 && !reachedCollectionPoint){
         float current_distance =hypot(currentLocation.x,currentLocation.y);
         dropOffLocation.x=current_distance*cos(currentLocation.theta+0.3)/2;
         dropOffLocation.y=current_distance*sin(currentLocation.theta+0.3)/2;
         dropofftimer=current_time/1e3;
     }

    //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
    //to resart our search.
    if(reachedCollectionPoint)
    {
      cout << "2" << endl;
      if (timerTimeElapsed >= 3)
      {
        //ROS_WARN("%s","Timer is greater than 5 seconds");
        if (finalInterrupt)
        {
          result.type = behavior;
          result.b = nextProcess;
          result.reset = true;
          dropset=true;
          pickupSet.request.point.x = dropOffLocation.x;
          pickupSet.request.point.y = dropOffLocation.y;
          set_pickup.call(pickupSet);
          reset_msgs();
          return result;
        }
        else
        {
          finalInterrupt = true;
          seenEnoughCenterTags = true;
          first_center=false;
          result.type=behavior;
          result.b=wait;
          cout << "1" << endl;
        }
      }else if(timerTimeElapsed>=1.5){
          isPrecisionDriving = true;
          result.type = precisionDriving;
          result.b=wait;
          result.fingerAngle = M_PI_2; //open fingers
          result.wristAngle = 0; //raise wrist
          result.wpts.waypoints.clear();
          result.pd.cmdVel = 0.0;
          result.pd.cmdAngularError = -2.0;
      }
      else
      {
        //ROS_WARN("%s","we have reached our drop off but time is greater than 0.1 but less than 5");
        isPrecisionDriving = true;
        result.type = precisionDriving;
        result.b=wait;
        result.fingerAngle = M_PI_2; //open fingers
        result.wristAngle = 0; //raise wrist
        result.wpts.waypoints.clear();
        result.pd.cmdVel = -1.0;
        result.pd.cmdAngularError = 0.0;
      }

      return result;
    }

    double distanceToLocation = hypot(this->dropOffLocation.x - this->currentLocation.x, this->dropOffLocation.y - this->currentLocation.y);
    if(distanceToLocation> 0.2&&!dropset){
        //ROS_WARN("%s","Distance is greater than 0.2");

        result.type = waypoint;
        result.wpts.waypoints.clear();
        result.wpts.waypoints.push_back(this->dropOffLocation);
        startWaypoint = false;
        isPrecisionDriving = false;
        timerTimeElapsed = 0;

        return result;

      }else if(distanceToLocation <0.2){
        //ROS_ERROR("%s","Robot is close enough");
        result.type=behavior;
        result.b=nextProcess;
        result.reset= false;
        reachedCollectionPoint = true;
        returnTimer = current_time;
        return result;

    }else{
        result.type=behavior;
        result.b=prevProcess;
        dropset=false;

    }
    return result;

}
Result DropOffController::PickupWork(){

    cout << "8" << endl;
    //ROS_WARN("%s","HELLOWORLD  ");
    int count = countLeft + countRight;

    if(timerTimeElapsed > -1) {
      //ROS_WARN("%s","timeTimeElapsed is greater than negative one");
      long int elapsed = current_time - returnTimer;
      timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
    }

    //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
    //to resart our search.
    if(reachedCollectionPoint)
    {
      cout << "2" << endl;
      if (timerTimeElapsed >= 3)
      {
          //ROS_WARN("%s","We have reached collection point and also timer is greater than 5");
        if (finalInterrupt)
        {
          //ROS_WARN("%s","This is the final interrupt");
          result.type = behavior;
          result.b = nextProcess;
          dropset=true;
          result.reset = true;
          return result;
        }
        else
        {
          //ROS_WARN("%s","FInal Interupt is true ");
          finalInterrupt = true;
          seenEnoughCenterTags = true;
          first_center=false;
          result.type=behavior;
          result.b=wait;
          cout << "1" << endl;
        }
      }else if(timerTimeElapsed>=1.5){
          isPrecisionDriving = true;
          result.type = precisionDriving;

          result.fingerAngle = M_PI_2; //open fingers
          result.wristAngle = 0; //raise wrist

          result.pd.cmdVel = 0;
          result.pd.cmdAngularError = -3.0;
      }
      else if (timerTimeElapsed >= 0.1)
      {
        //ROS_WARN("%s","TImer is greater than 0.1 and not greater than 5");
        isPrecisionDriving = true;
        result.type = precisionDriving;

        result.fingerAngle = M_PI_2; //open fingers
        result.wristAngle = 0; //raise wrist

        result.pd.cmdVel = -1;
        result.pd.cmdAngularError = 0.0;
      }

      return result;
    }
    if(!returnpoint){
    double angle= atan(currentLocation.x/currentLocation.y);
    dropOffLocation.x=0;
    dropOffLocation.y=0;
    dropOffLocation.theta=angle;
    returnpoint=true;
    }

    double distanceToCenter = hypot(0 - this->currentLocation.x, 0 - this->currentLocation.y)-0.02;
    dropOffLocation.x= currentLocation.x*0.75;
    dropOffLocation.y=currentLocation.y*0.75;
    double angletocenter =  atan(currentLocation.x/distanceToCenter);

    //double distanceToLocation = hypot(this->currentLocation.x/2,this->currentLocation.y/2);// This must be moved to a controlled area where specific functions are set for different robot types.
    //check to see if we are driving to the center location or if we need to drive in a circle and look.
    if (distanceToCenter > 0.65) {
      //ROS_WARN("%s","DIstance is greater than collection point visual distance");
      result.type = waypoint;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.push_back(dropOffLocation);
      startWaypoint = false;
      isPrecisionDriving = false;

      timerTimeElapsed = 0;

      return result;

    }else if(distanceToCenter <0.65){
        result.type=behavior;
        result.b=nextProcess;
        result.reset= false;
        reachedCollectionPoint = true;
        returnTimer = current_time;
        return result;
    }else{
    result.type=behavior;
    result.b=prevProcess;
    dropset=false;
    }
    return result;
}

void DropOffController::Reset() {
  result.type = behavior;
  result.b = wait;
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError = 0;
  result.fingerAngle = -1;
  result.wristAngle = 0.7;
  result.reset = false;
  result.wpts.waypoints.clear();
  spinner = 0;
  spinSizeIncrease = 0;
  prevCount = 0;
  timerTimeElapsed = -1;

  countLeft = 0;
  countRight = 0;


  //reset flags
  reachedCollectionPoint = false;
  seenEnoughCenterTags = false;
  circularCenterSearching = false;
  isPrecisionDriving = false;
  finalInterrupt = false;
  precisionInterrupt = false;
  targetHeld = false;
  startWaypoint = false;
  first_center = true;
  returnpoint = false;
  initial_test=false;
  cout << "6" << endl;

}

void DropOffController::SetTargetData(vector<Tag> tags) {
  countRight = 0;
  countLeft = 0;

  if(targetHeld) {
    // if a target is detected and we are looking for center tags
    if (tags.size() > 0 && !reachedCollectionPoint) {

      // this loop is to get the number of center tags
      for (int i = 0; i < tags.size(); i++) {
        if (tags[i].getID() == 256) {


          // checks if tag is on the right or left side of the image
          if (tags[i].getPositionX() + cameraOffsetCorrection > 0) {
            countRight++;

          } else {
            countLeft++;
          }
        }
      }
    }
  }

}

void DropOffController::ProcessData() {
  if((countLeft + countRight) > 0) {
    isPrecisionDriving = true;
  }else{
    startWaypoint = true;
  }
}

bool DropOffController::ShouldInterrupt() {
  ProcessData();
  if (startWaypoint && !interrupt) {
    interrupt = true;
    precisionInterrupt = false;
    return true;
  }
  else if (isPrecisionDriving && !precisionInterrupt) {
    precisionInterrupt = true;
    return true;
  }
  if (finalInterrupt) {
    return true;
  }
}

bool DropOffController::HasWork() {

  if(timerTimeElapsed > -1) {
    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  if (circularCenterSearching && timerTimeElapsed < 2 && !isPrecisionDriving) {
    return false;
  }

  return ((startWaypoint || isPrecisionDriving));
}

bool DropOffController::IsChangingMode() {
  return isPrecisionDriving;
}

void DropOffController::SetCenterLocation(Point center) {
  centerLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
  currentLocation = current;
}
void DropOffController::SetDropoffLocation(Point dropoff){
    dropOffLocation.x = dropoff.x/2;
    dropOffLocation.y = dropoff.y/2;
}

void DropOffController::SetTargetPickedUp() {
  targetHeld = true;
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
  targetHeld = targetHeld || blockBlock;
}

void DropOffController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}
void DropOffController::changeType(){
    if(Work==&DropOffController::PickupWork){
        Work=&DropOffController::SearchWork;
    }else{
        Work=&DropOffController::PickupWork;
    }
}
string DropOffController::getdata(){
    string msg="";
    msg+="\ntargetHeld: ";
    targetHeld?msg+="true":msg+="false";
    msg+="\nreachedCollectionPoint: ";
    reachedCollectionPoint?msg+="true":msg+="false";
    msg+="\ncircularsearch: ";
    circularCenterSearching?msg+="true":msg+="false";
    msg+="\ncenterapproach: ";
    centerApproach?msg+="true":msg+="false";
    msg+="\nseenenough: ";
    seenEnoughCenterTags?msg+="true":msg+="false";
    msg+="\nisprecisiondriving: ";
    isPrecisionDriving?msg+="true":msg+="false";
    msg+="\nstartwaypoint: ";
    startWaypoint?msg+="true":msg+="false";
    msg+="\ninterrupt: ";
    interrupt?msg+="true":msg+="false";
    msg+="\nprecision interrupt: ";
    precisionInterrupt?msg+="true":msg+="false";
    msg+="\nfinalinterrupt: ";
    finalInterrupt?msg+="true":msg+="false";
    msg+="\nfirst center: ";
    first_center?msg+="true":msg+="false";
    msg+="\n";

    return msg;

}
