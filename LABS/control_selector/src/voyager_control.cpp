#include "voyager_control.h"

 void VoyagerControl::setLaserData(const std::vector<float>& data){
	obstacle = false;
	for(auto d : data) if(d < min_range) { obstacle = true; break; }
}

void VoyagerControl::getControl(double& v, double& w){
	v = obstacle ? 0 : max_vel;
	w = obstacle ? max_omega : 0;
}