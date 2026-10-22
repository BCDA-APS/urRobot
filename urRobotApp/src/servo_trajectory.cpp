#include <stdexcept>
#include <iostream>
#include "servo_trajectory.hpp"

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T>>& input) {
    std::vector<T> result;

    std::size_t total_size = 0;
    for (const auto& inner : input) {
        total_size += inner.size();
    }

    result.reserve(total_size);

    for (const auto& inner : input) {
        result.insert(result.end(), inner.begin(), inner.end());
    }

    return result;
}

template <typename T>
std::vector<std::vector<T>> unflatten(const std::vector<T>& input, size_t inner_size) {

    if (inner_size == 0 || (input.size() % inner_size) != 0) {
        printf("input.size() / inner_size = %ld\n", input.size() / inner_size);
        throw std::invalid_argument("Flat vector cannot be reshaped to requested size");
    }

    std::vector<std::vector<T>> result;

    for (std::size_t i = 0; i < input.size(); i += inner_size) {
        result.emplace_back(input.begin() + i, input.begin() + i + inner_size);
    }

    return result;
}

// TODO:
// - Validate length 19 flat values
// - Require at least 2 points
// - Reject non-finite values
// - Validate timestamps
// - Precompute all quintic coefficients
// - Throw std::invalid_argument on invalid inputs
Trajectory parse_sparse_trajectory(const std::vector<double>& values, size_t count, double max_velocity,
                                   double max_acceleration) {

    if (values.size() < SERVO_POINT_WIDTH * 2) {
        throw std::invalid_argument("Sparse flat trajectory is invalid length");
    }

    auto trajectory_points = unflatten(values, SERVO_POINT_WIDTH);
    Trajectory traj;
    traj.points.reserve(trajectory_points.size());

    for (const auto& row : trajectory_points) {
        traj.points.push_back(TrajectoryPoint{
            row[0],
            std::vector<double>(row.begin() + 1, row.begin() + 7),
            std::vector<double>(row.begin() + 7, row.begin() + 13),
            std::vector<double>(row.begin() + 13, row.begin() + 19),
        });
    }

    traj.segments.reserve(traj.points.size() - 1);
    for (size_t point_index = 0; point_index + 1 < traj.points.size(); ++point_index) {
        const auto& start = traj.points[point_index];
        const auto& end = traj.points[point_index + 1];
        const double duration = end.time - start.time;

        TrajectorySegment segment{};
        segment.start_time = start.time;
        segment.end_time = end.time;

        for (size_t joint = 0; joint < SERVO_JOINT_COUNT; ++joint) {
            auto& coefficients = segment.coefficients[joint];
            coefficients[0] = start.position[joint];
            coefficients[1] = duration * start.velocity[joint];
            coefficients[2] = 0.5 * duration * duration * start.acceleration[joint];

            const double position_error =
                end.position[joint] - (coefficients[0] + coefficients[1] + coefficients[2]);
            const double velocity_error =
                duration * end.velocity[joint] - (coefficients[1] + 2.0 * coefficients[2]);
            const double acceleration_error =
                duration * duration * end.acceleration[joint] - 2.0 * coefficients[2];

            coefficients[3] =
                10.0 * position_error - 4.0 * velocity_error + 0.5 * acceleration_error;
            coefficients[4] = -15.0 * position_error + 7.0 * velocity_error - acceleration_error;
            coefficients[5] = 6.0 * position_error - 3.0 * velocity_error + 0.5 * acceleration_error;
        }

        traj.segments.push_back(segment);
    }

    return traj;
}

// TODO:
// - Select appropriate segment
// - Clamp time to the trajectory range
// - Evaluate all six joints with Horner's method
// - Return the exact first or final position at the boundaries
// std::array<double, SERVO_JOINT_COUNT> sample_servo_trajectory() {
// }
