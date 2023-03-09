#include <vector>
#include <iostream>
#include "motion_profile_sigmoid/motion_profile_sigmoid.h"

// parameters: [distance_total, velocity_max, acceleration_max, jerk]
SigmoidMotionProfile motion_profile = SigmoidMotionProfile(300, 50, 5, 2);

int main() {
    float time_full = motion_profile.get_time_end();
    printf("Total Motion Time: %f\n\n", time_full);
    float value_last = 0.0f;
    for (float progress_percentage = 0.0f; progress_percentage <= 1; progress_percentage += 0.02f) {
        /*float progress_value = motion_profile.get_distance(time_full * progress_percentage);
        float progress_time_traced = motion_profile.get_time(progress_value);
        printf("Phase: %d Time: %08.3f Traced: %08.3f Value: %08.3f (+ %08.3f) |   O", motion_profile.get_phase(time_full * progress_percentage), time_full * progress_percentage, progress_time_traced, progress_value, progress_value - value_last);
        for (int spacing = 1; spacing < std::abs(progress_value / (300 / 40.0f)); spacing++) printf("X");
        printf("\n");
        value_last = progress_value;*/
        float progress_distance = 300 * progress_percentage;
        float progress_velocity = motion_profile.get_distance_velocity(progress_distance);
        printf("Distance: %08.3f |   O", progress_distance);
        for (int spacing = 1; spacing < std::abs(progress_velocity / (50 / 40.0f)); spacing++) printf("X");
        printf("\n");
    }
    printf("Final Distance: %f\n", motion_profile.get_time_distance(time_full));
    return 0;
}