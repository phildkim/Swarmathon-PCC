#include "GridService.h"

GridService::GridService() : data(0)
{ }

GridService::~GridService() { }

bool GridService::getStatus(ccny_srvs::GetStatus::Request& req, ccny_srvs::GetStatus::Response& res, uint8_t stride, uint8_t offset, uint8_t mask) {
        res.data = (this->data & mask) >> offset;
	ROS_INFO("Status Register: [%u]", this->data);
        res.success = true;
        return true;
}

bool GridService::setStatus(ccny_srvs::SetStatus::Request& req, ccny_srvs::SetStatus::Response& res, uint8_t stride, uint8_t offset, uint8_t mask) {
        if(req.data != (this->data & mask) >> offset)
                this->data ^= (1 << offset);
	
        res.success = true;
        return true;
}

ros::ServiceServer GridService::registerStatusGetter(const std::string &name, uint8_t stride, uint8_t offset, uint8_t mask) {
        return node.advertiseService<ccny_srvs::GetStatus::Request, ccny_srvs::GetStatus::Response>(name, boost::bind(&GridService::getStatus, this, _1, _2, stride, offset, mask));
}

ros::ServiceServer GridService::registerStatusSetter(const std::string &name, uint8_t stride, uint8_t offset, uint8_t mask) {
        return node.advertiseService<ccny_srvs::SetStatus::Request, ccny_srvs::SetStatus::Response>(name, boost::bind(&GridService::setStatus, this, _1, _2, stride, offset, mask));
}

