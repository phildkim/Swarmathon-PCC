#include<string>
#include <bits/stdc++.h>
CreateLog::CreateLog(std_msgs::String * msg, Result * res){
    this->res = res;
    this->msg = msg;
    getLog();

}

void CreateLog::getLog(){
    msg->data+="type: ";
    switch(res->type){

       case behavior: msg->data += "behavior";break;
       case waypoint: msg->data += "waypoint";break;
       case precisionDriving: msg->data+= "precision";break;
       default:break;


    }
    msg->data+= ", b: ";
    switch(res->b){
    case wait: msg->data+="wait";break;
    case prevProcess: msg->data+="prev";break;
    case noChange: msg->data+= "nochange";break;
    case nextProcess: msg->data+="next";break;
    default:break;
    }
    msg->data+=", \n waypoints: ";
    if(res->wpts.waypoints.empty()){
        msg->data+="empty \n ";
    }else{
        for(Point i: res->wpts.waypoints){
            msg->data += std::to_string(i.x) +" "+std::to_string(i.y)+" "+std::to_string(i.theta)+" \n";
        }
    }
    msg->data+="Precision: \n";
    msg->data+="Angular:"+std::to_string(res->pd.cmdAngular)+" AngularError:"+
            std::to_string(res->pd.cmdAngularError)+" Velocity:"+
            std::to_string(res->pd.cmdVel)+" left:"+
            std::to_string(res->pd.left)+" Right:"+
            std::to_string(res->pd.right)+" SetPointVel:"+
            std::to_string(res->pd.setPointVel)+" SetPointYaw:"+
            std::to_string(res->pd.setPointYaw)+" \n";
    msg->data+="FingerAngle:"+std::to_string(res->fingerAngle)+"\n";
    msg->data+="WristAngle:"+std::to_string(res->wristAngle)+"\n";
    msg->data+="PID Mode: ";
    switch(res->PIDMode){
    case FAST_PID: msg->data+="FAST_PID";break;
    case SLOW_PID: msg->data+="SLOW_PID";break;
    case CONST_PID: msg->data+="CONST_PID";break;
    }
    std::string data(msg->data);
    char data_array(data.length());
//    strcpy(data_array,data.c_str());
//    ROS_WARN("%s",msg->data);

}
