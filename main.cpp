#include <vector>
#include <iostream>
#include "motion_profile_sigmoid/motion_profile_sigmoid.h"

// parameters: [distance_total, velocity_max, acceleration_max, jerk]
SigmoidMotionProfile motion_profile = SigmoidMotionProfile(100, 17, 5, 2);

int main() {
    float time_full = motion_profile.get_time_end();
    printf("Total Motion Time: %f\n\n", time_full);
    for (float progress_percentage = 0.0f; progress_percentage <= 1; progress_percentage += 0.01f) {
        float progress_value = motion_profile.get_velocity(time_full * progress_percentage);
        printf("Value: %07.3f |   ", progress_value);
        for (int spacing = 1; spacing < std::abs(progress_value / (17 / 90.0f)); spacing++) printf("X");
        printf("\n");
    }
    return 0;
}