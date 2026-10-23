#include <epicsUnitTest.h>
#include <testMain.h>

#include "trajectory.hpp"

#include <iostream>

template <typename Container>
void print_vector(const Container& vec) {
    std::cout << "[";
    for (size_t i = 0; i < vec.size(); i++) {
        if (i == vec.size()-1) {
            std::cout << vec[i] << "]\n";
        } else {
            std::cout << vec[i] << " ";
        }
    }
}


MAIN(testCollisions) {
    testPlan(0);

    std::vector<double> sparse_traj = {
        0.0000,-1.5700,-1.5700,-1.4000,-1.5700,1.0100,-0.0200,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
        1.0100,-1.2500,-1.2500,-1.6500,-1.3000,1.2000,0.1500,0.3564,0.0594,-0.2228,0.2574,-0.0545,0.1832,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
        2.0200,-0.8500,-1.4500,-1.8500,-1.0500,0.9000,0.3500,0.3465,0.0495,0.0495,-0.0248,-0.0495,-0.0248,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
        3.0300,-0.5500,-1.1500,-1.5500,-1.3500,1.1000,0.1000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000
    };

    auto traj = build_trajectory(sparse_traj);
    for (const auto& p : traj.points) {
        printf("Time = %lf\n", p.time);
        print_vector(p.position);
        print_vector(p.velocity);
        print_vector(p.acceleration);
        printf("\n");
    }

    // printf("Segment count = %zu\n", traj.segments.size());
    // for (size_t segment_index = 0; segment_index < traj.segments.size(); ++segment_index) {
        // const auto& segment = traj.segments[segment_index];
        // const double duration = segment.end_time - segment.start_time;
//
        // printf("\nSegment %zu: [%lf, %lf]\n", segment_index, segment.start_time, segment.end_time);
        // for (size_t joint = 0; joint < NUM_JOINTS; ++joint) {
            // const auto& c = segment.coefficients[joint];
            // const double start_position = c[0];
            // const double end_position = c[0] + c[1] + c[2] + c[3] + c[4] + c[5];
            // const double start_velocity = c[1] / duration;
            // const double end_velocity =
                // (c[1] + 2.0 * c[2] + 3.0 * c[3] + 4.0 * c[4] + 5.0 * c[5]) / duration;
            // const double start_acceleration = 2.0 * c[2] / (duration * duration);
            // const double end_acceleration =
                // (2.0 * c[2] + 6.0 * c[3] + 12.0 * c[4] + 20.0 * c[5]) /
                // (duration * duration);
//
            // printf("Joint %zu: q=(%.6f, %.6f) v=(%.6f, %.6f) a=(%.6f, %.6f)\n", joint,
                   // start_position, end_position, start_velocity, end_velocity, start_acceleration,
                   // end_acceleration);
        // }
    // }

    testOk1(1==1);

    {
        std::vector<int> vec = {1, 2, 3, 4, 5, 6};
        auto unflattened_vec = unflatten(vec, 3);
        testOk1(unflattened_vec.size() == 2);
        testOk1(unflattened_vec[0].size() == 3);
    }

    {
        std::vector<std::vector<int>> vec = {
            {1,2,3},
            {4,5,6}
        };
        std::vector<int> unflattened_vec = flatten(vec);
        testOk1(unflattened_vec.size() == 6);
        testOk1(unflattened_vec.front() == 1);
        testOk1(unflattened_vec.back() == 6);
    }

    return testDone();
}
