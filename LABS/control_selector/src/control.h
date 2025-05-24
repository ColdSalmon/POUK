#ifndef CONTROL_H
#define CONTROL_H

#include <vector>
#include <string>

class Control
{
public:
    virtual void setLaserData(const std::vector<float>& data) = 0;
    virtual void setRobotPose(double x, double y, double theta) = 0;
    virtual void getControl(double& v, double& w) = 0;
    virtual std::string getName() = 0;
    virtual ~Control() {}
};

#endif // CONTROL_H