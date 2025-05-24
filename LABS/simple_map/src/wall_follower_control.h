#ifndef WALL_FOLLOWER_CONTROL_H
#define WALL_FOLLOWER_CONTROL_H

#include "control.h"
#include <vector>
#include <algorithm>
#include <string>

class WallFollowerControl : public Control
{
    public:
        void obstaclePush(bool isRight);
        bool wasObstacleNearby(bool isRight);
        WallFollowerControl(double target_dist, double max_speed, double p_coeff, double front_threshold)
            : target_distance(target_dist),
            max_vel(max_speed),
            kP(p_coeff),
            front_safe_dist(front_threshold) {}

        void setLaserData(const std::vector<float>& data) override;
        void setRobotPose(double x, double y, double theta) override {}
        void getControl(double& v, double& w) override;
        std::string getName() override { return "WallFollower"; }

    private:
        float right_min_dist = 0.0;
        float left_min_dist = 0.0;
        float front_min_dist = 0.0;
        bool followRightWall = true;
        bool pastRightObstacle[5] = { false };
        bool pastLeftObstacle[5] = { false };
        // Параметры управления
        double target_distance;  // Желаемое расстояние до стены (аналог nomRange)
        double max_vel;          // Максимальная линейная скорость
        double kP;               // Коэффициент пропорционального регулятора
        double front_safe_dist;  // Безопасное расстояние впереди (аналог kMinRange)

        // Состояние
        bool obstacle = false;
        float obstacleRange = 0.0f;
        bool pastObstacle[5] = { false, false, false, false, false };





};

#endif // WALL_FOLLOWER_CONTROL_H