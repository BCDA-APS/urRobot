#pragma once
#include <array>
#include <cstddef>
#include <vector>

constexpr size_t SERVO_JOINT_COUNT = 6;
constexpr size_t SERVO_POINT_WIDTH = 19;

struct TrajectoryPoint {
    double time;
    // std::array<double, SERVO_JOINT_COUNT> position;
    // std::array<double, SERVO_JOINT_COUNT> velocity;
    // std::array<double, SERVO_JOINT_COUNT> acceleration;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
};

struct TrajectorySegment {
    double start_time;
    double end_time;
    std::array<std::array<double, 6>, SERVO_JOINT_COUNT> coefficients;
};

struct Trajectory {
    std::vector<TrajectoryPoint> points;
    std::vector<TrajectorySegment> segments;
};

Trajectory parse_sparse_trajectory(const std::vector<double>& values, size_t count, double max_velocity,
                                       double max_acceleration);

// std::array<double, SERVO_JOINT_COUNT> sample_servo_trajectory();
