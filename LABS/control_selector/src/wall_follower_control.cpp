#include "wall_follower_control.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"

// Константы для настройки поведения
constexpr double SAFE_DISTANCE_BUFFER = 0.1;  // Буфер безопасности
constexpr double ANGULAR_COEFFICIENT = 1.5;   // Коэффициент угловой скорости
constexpr double WALL_LOST_THRESHOLD = 1.5;   // Порог для определения потери стены
constexpr double WALL_SEARCH_SPEED = 0.4;     // Скорость при поиске стены

void WallFollowerControl::obstaclePush(bool isRight)
{
    auto& pastObs = isRight ? pastRightObstacle : pastLeftObstacle;
    for (int i = 4; i > 0; i--) {
        pastObs[i] = pastObs[i - 1];
    }
    pastObs[0] = (isRight ? right_min_dist : left_min_dist) <= target_distance;
}

bool WallFollowerControl::wasObstacleNearby(bool isRight)
{
    const auto& pastObs = isRight ? pastRightObstacle : pastLeftObstacle;
    for (int i = 0; i < 5; i++) {
        if (pastObs[i]) return true;
    }
    return false;
}

void WallFollowerControl::setLaserData(const std::vector<float>& data)
{
    if (data.empty()) return;

    // Определяем индексы лучей для разных направлений
    size_t front_idx = data.size() / 2;        // Центральный луч (перед)
    size_t right_start = data.size() / 4;      // Начало правой стороны
    size_t right_end = data.size() * 3 / 8;    // Конец правой стороны
    size_t left_start = data.size() * 5 / 8;   // Начало левой стороны
    size_t left_end = data.size() * 3 / 4;     // Конец левой стороны

    // Находим минимальные расстояния
    right_min_dist = *std::min_element(data.begin() + right_start, data.begin() + right_end);
    left_min_dist = *std::min_element(data.begin() + left_start, data.begin() + left_end);
    front_min_dist = data[front_idx];

    // Обновляем историю препятствий для обеих сторон
    obstaclePush(true);  // Правая сторона
    obstaclePush(false); // Левая сторона

    // Определяем состояние препятствий
    obstacle = (front_min_dist < front_safe_dist) ||
        (right_min_dist < target_distance) ||
        (left_min_dist < target_distance);
}

void WallFollowerControl::getControl(double& v, double& w)
{
    // Экстренное торможение при близком препятствии спереди
    if (front_min_dist < front_safe_dist - SAFE_DISTANCE_BUFFER) {
        v = 0;
        w = followRightWall ? 0.5 : -0.5; // Поворот от стены
        return;
    }

    // Определяем текущую сторону и расстояние
    double current_dist = followRightWall ? right_min_dist : left_min_dist;
    bool wall_detected = (current_dist < target_distance + SAFE_DISTANCE_BUFFER);
    bool wall_lost = (current_dist > target_distance * WALL_LOST_THRESHOLD);

    // Основные режимы работы
    if (wall_detected) {
        // Режим следования вдоль стены
        double error = current_dist - target_distance;
        v = max_vel * 0.7;
        w = followRightWall ? (-error * ANGULAR_COEFFICIENT) : (error * ANGULAR_COEFFICIENT);
    }
    else if (wall_lost) {
        // Режим возврата к потерянной стене
        v = WALL_SEARCH_SPEED;
        w = followRightWall ? -0.5 : 0.5; // Активный поворот к стене
    }
    else {
        // Промежуточный режим - плавный поиск стены
        v = WALL_SEARCH_SPEED;
        w = followRightWall ? -0.2 : 0.2; // Плавный поворот к стене
    }


}