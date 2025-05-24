#pragma once
#include "control.h"

class VoyagerControl : public Control
{
public:
    VoyagerControl(double range, double maxv, double maxw)
        : min_range(range), max_vel(maxv), max_omega(maxw) {}
    
    void setLaserData(const std::vector<float>& data) override;
    
    void setRobotPose(double, double, double) override {}
    
    void getControl(double& v, double& w) override;
    
    std::string getName() override { return "Voyager"; }

private:
    double min_range;
    double max_vel;
    double max_omega;
    bool obstacle = false;
};