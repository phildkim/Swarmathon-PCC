#include "DropOffController.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include "ccny_srvs/SetPickup.h"

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
return Work(*this);
}
Result DropOffController::SearchWork(){
    ros::NodeHandle gm;
    ros::ServiceClient putdown = gm.serviceClient<ccny_srvs::SetPickup>("pickupsetter");
    int count = countLeft + countRight;

    if(timerTimeElapsed > -1) {

      long int elapsed = current_time - returnTimer;
      timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
    }

    //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
    //to resart our search.
    if(reachedCollectionPoint)
    {
      cout << "2" << endl;
      if (timerTimeElapsed >= 5)
      {
        //ROS_WARN("%s","Timer is greater than 5 seconds");
        if (finalInterrupt)
        {
          result.type = behavior;
          result.b = nextProcess;
          result.reset = true;
          dropset=true;
          ccny_srvs::SetPickup msg;
          msg.request.point.x = dropOffLocation.x;
          msg.request.point.y = dropOffLocation.y;
          //ROS_WARN(" drop x:%f y:%f",dropOffLocation.x,dropOffLocation.y);
          putdown.call(msg);
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
      }else if(timerTimeElapsed>=2){
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
        result.pd.cmdVel = -1.5;
        result.pd.cmdAngularError = 0.0;
      }

      return result;
    }

    double distanceToLocation = hypot(this->dropOffLocation.x - this->currentLocation.x, this->dropOffLocation.y - this->currentLocation.y);
    if(distanceToLocation> 0.2&&!dropset){
        //ROS_WARN("%s","Distance is greater than 0.2");
        if(hypot(currentLocation.x,currentLocation.y)<3.1){
        dropOffLocation.x=currentLocation.x*0.75;
        dropOffLocation.y=currentLocation.y*0.75;
    }
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
      if (timerTimeElapsed >= 5)
      {
          //ROS_WARN("%s","We have reached collection point and also timer is greater than 5");
        if (finalInterrupt)
        {
          //ROS_WARN("%s","This is the final interrupt");
          result.type = behavior;
          result.b = nextProcess;
          //dropset=true;
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
      }
      else if (timerTimeElapsed >= 0.1)
      {
        //ROS_WARN("%s","TImer is greater than 0.1 and not greater than 5");
        isPrecisionDriving = true;
        result.type = precisionDriving;

        result.fingerAngle = M_PI_2; //open fingers
        result.wristAngle = 0; //raise wrist

        result.pd.cmdVel = -0.3;
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
    ROS_WARN("X: %f Y:%f  angle: %f angle2center: %f",dropOffLocation.x,dropOffLocation.y,distanceToCenter,angletocenter);
    //double distanceToLocation = hypot(this->currentLocation.x/2,this->currentLocation.y/2);// This must be moved to a controlled area where specific functions are set for different robot types.
    //check to see if we are driving to the center location or if we need to drive in a circle and look.
    if (distanceToCenter > 0.62) {
      //ROS_WARN("%s","DIstance is greater than collection point visual distance");
      result.type = waypoint;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.push_back(dropOffLocation);
      startWaypoint = false;
      isPrecisionDriving = false;

      timerTimeElapsed = 0;

      return result;

    }else if(distanceToCenter <0.62){
        result.type=behavior;
        result.b=nextProcess;
        result.reset= false;
        reachedCollectionPoint = true;
        returnTimer = current_time;
        return result;
    }
    return result;
    /*else if (timerTimeElapsed >= 2)//spin search for center
    {
      //ROS_WARN("%s","We are spin searching");
      Point nextSpinPoint;

      //sets a goal that is 60cm from the centerLocation and spinner
      //radians counterclockwise from being purly along the x-axis.
      nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
      nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
      nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);

      result.type = waypoint;
      result.wpts.waypoints.clear();
      result.wpts.waypoints.push_back(nextSpinPoint);

      spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
      if (spinner > 2*M_PI) {
        spinner -= 2*M_PI;
      }
      spinSizeIncrease += spinSizeIncrement/8;
      circularCenterSearching = true;
      //safety flag to prevent us trying to drive back to the
      //center since we have a block with us and the above point is
      //greater than collectionPointVisualDistance from the center.

      returnTimer = current_time;
      timerTimeElapsed = 0;

    }

    bool left = (countLeft > 0);
    bool right = (countRight > 0);
    bool centerSeen = (right || left);

    //reset lastCenterTagThresholdTime timout timer to current time
    if ((!centerApproach && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {
      //ROS_WARN("%s","We are resetting LastcenterTagThreshold");
      lastCenterTagThresholdTime = current_time;

    }

    if (count > 0 || seenEnoughCenterTags || prevCount > 0) //if we have a target and the center is located drive towards it.
    {
      //ROS_WARN("%s","we have a target and the center is located drive towards it");
      cout << "9" << endl;
      centerSeen = true;

      if (first_center && isPrecisionDriving)
      {
        first_center = false;
        result.type = behavior;
        result.reset = false;
        result.b = nextProcess;
        return result;
      }
      isPrecisionDriving = true;

      if (seenEnoughCenterTags) //if we have seen enough tags
      {
        if ((countLeft-5) > countRight) //and there are too many on the left
        {
          right = false; //then we say none on the right to cause us to turn right
        }
        else if ((countRight-5) > countLeft)
        {
          left = false; //or left in this case
        }
      }

      float turnDirection = 1;
      //reverse tag rejection when we have seen enough tags that we are on a
      //trajectory in to the square we dont want to follow an edge.
      if (seenEnoughCenterTags) turnDirection = -3;

      result.type = precisionDriving;

      //otherwise turn till tags on both sides of image then drive straight
      if (left && right) {
        result.pd.cmdVel = searchVelocity;
        result.pd.cmdAngularError = 0.0;
      }
      else if (right) {
        result.pd.cmdVel = -0.1 * turnDirection;
        result.pd.cmdAngularError = -centeringTurnRate*turnDirection;
      }
      else if (left){
        result.pd.cmdVel = -0.1 * turnDirection;
        result.pd.cmdAngularError = centeringTurnRate*turnDirection;
      }
      else
      {
        result.pd.cmdVel = searchVelocity;
        result.pd.cmdAngularError = 0.0;
      }

      //must see greater than this many tags before assuming we are driving into the center and not along an edge.
      if (countLeft>5 && countRight>5)
      {
        seenEnoughCenterTags = true; //we have driven far enough forward to be in and aligned with the circle.
        lastCenterTagThresholdTime = current_time;
      }
      if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
      {
        lastCenterTagThresholdTime = current_time;
      }
      //time since we dropped below countGuard tags
      long int elapsed = current_time - lastCenterTagThresholdTime;
      float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds

      //we have driven far enough forward to have passed over the circle.
      if (count < 1 && seenEnoughCenterTags && timeSinceSeeingEnoughCenterTags > dropDelay) {
        centerSeen = false;
      }
      centerApproach = true;
      prevCount = count;
      count = 0;
      countLeft = 0;
      countRight = 0;
    }

    //was on approach to center and did not seenEnoughCenterTags
    //for lostCenterCutoff seconds so reset.
    else if (centerApproach) {
      //ROS_WARN("%s","center approach");
      long int elapsed = current_time - lastCenterTagThresholdTime;
      float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds
      if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff)
      {
        cout << "4" << endl;
        //go back to drive to center base location instead of drop off attempt
        reachedCollectionPoint = false;
        seenEnoughCenterTags = false;
        centerApproach = false;

        result.type = waypoint;
        result.wpts.waypoints.push_back(this->centerLocation);
        if (isPrecisionDriving) {
          result.type = behavior;
          result.b = prevProcess;
          result.reset = false;
        }
        isPrecisionDriving = false;
        interrupt = false;
        precisionInterrupt = false;
      }
      else
      {
        result.pd.cmdVel = searchVelocity;
        result.pd.cmdAngularError = 0.0;
      }

      return result;

    }
    if (!centerSeen && seenEnoughCenterTags)
    {
        //ROS_WARN("%s","We have reached our collection point ");
      reachedCollectionPoint = true;
      centerApproach = false;
      returnTimer = current_time;
    }

    return result;
*/
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
    Work=&DropOffController::SearchWork;
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
