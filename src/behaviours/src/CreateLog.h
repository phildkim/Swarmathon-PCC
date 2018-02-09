#ifndef LOG_CLASS
#define LOG_CLASS

#include <std_msgs/String.h>
#include "ros/console.h"

#ifndef CONTROLLER_H
#define RESULTS_
#include "Result.h"
#endif //RESULTS_
#include "Point.h"

class CreateLog{

public:
        CreateLog(std_msgs::String *,Result*);
        void getLog();

        Result * res;
        std_msgs::String * msg;


};
#include "CreateLog.cpp"
#endif //LOG_CLASS
