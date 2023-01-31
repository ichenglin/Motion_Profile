#include <vector>
#include <iostream>
#include "motion_profile_sigmoid/motion_profile_sigmoid.h"

// parameters: [distance_total, velocity_max, acceleration_max, jerk]
SigmoidMotionProfile motion_profile = SigmoidMotionProfile(100, 13, 5, 2.26);

int main() {
    float time_full = motion_profile.get_time_end();
    printf("Total Motion Time: %f\n\n", time_full);
    for (float progress_percentage = 0.0f; progress_percentage <= 1; progress_percentage += 0.03f) {
        float progress_value = motion_profile.get_distance(time_full * progress_percentage);
        printf("Phase: %d Time: %08.3f Value: %08.3f |   O", motion_profile.get_phase(time_full * progress_percentage), time_full * progress_percentage, progress_value);
        for (int spacing = 1; spacing < std::abs(progress_value / (500 / 90.0f)); spacing++) printf("X");
        printf("\n");
    }
    return 0;
}