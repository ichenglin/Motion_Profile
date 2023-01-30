#include <cmath>
#include "motion_profile_trapezoidal.h"

/**
 * Construct a new Motion Profile object
 * 
 * @param distance The total distance of the path
 * @param velocity_max The maximum velocity during the motion
 * @param acceleration The acceleration of the motion
 */
TrapezoidalMotionProfile::TrapezoidalMotionProfile(float distance, float velocity_max, float acceleration) {
    // constants
    this->motion_distance      = distance;
    this->motion_acceleration  = acceleration;
    // calculates the reachable maximum velocity
    float velocity_max_actual  = std::fmin(std::sqrt(acceleration * distance), velocity_max);
    this->motion_velocity_max  = velocity_max_actual;
    // calculates the time of motion
    float speeding_time        = velocity_max_actual / acceleration; // for either accelerate/decelerate
    float speeding_distance    = velocity_max_actual * speeding_time;
    float sliding_distance     = distance - speeding_distance;
    float sliding_time         = sliding_distance / velocity_max_actual;
    this->motion_time_speeding = speeding_time;
    this->motion_time_sliding  = sliding_time;
    this->motion_time_full     = 2 * speeding_time + sliding_time;
}

/**
 * Calculates the instantaneous distance at time
 * 
 * @param time The time since the start of the motion
 * @return instantaneous distance at time
 */
float TrapezoidalMotionProfile::get_distance(float time) {
    float distance_net = 0.0f;
    // accelerate
    float accelerate_time = std::fmin(time, this->motion_time_speeding);
    distance_net += 0.5f * this->motion_acceleration * std::pow(accelerate_time, 2);
    // slide
    float slide_time = std::fmin(time - this->motion_time_speeding, this->motion_time_sliding);
    if (slide_time > 0) distance_net += this->motion_velocity_max * slide_time;
    // decelerate
    float decelerate_time = time - this->motion_time_speeding - this->motion_time_sliding;
    if (decelerate_time > 0) distance_net += this->motion_velocity_max * decelerate_time - 0.5f * this->motion_acceleration * std::pow(decelerate_time, 2);
    return distance_net;
}

/**
 * Calculates the instantaneous velocity at time
 * 
 * @param time The time since the start of the motion
 * @return Instantaneous velocity
 */
float TrapezoidalMotionProfile::get_velocity(float time) {
    // accelerate
    if (time < this->motion_time_speeding) return this->motion_acceleration * time;
    // slide
    if (time < this->motion_time_speeding + this->motion_time_sliding) return this->motion_velocity_max;
    // decelerate
    return this->motion_velocity_max - this->motion_acceleration * (time - (this->motion_time_speeding + this->motion_time_sliding));
}

/**
 * Calculates the total time of the motion
 * 
 * @return Total time of the motion
 */
float TrapezoidalMotionProfile::get_time() {
    return this->motion_time_full;
}