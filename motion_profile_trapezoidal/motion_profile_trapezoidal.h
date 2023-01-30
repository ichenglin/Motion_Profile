#pragma once

class TrapezoidalMotionProfile {

private:
    float motion_distance;
    float motion_velocity_max;
    float motion_acceleration;
    float motion_time_full;
    float motion_time_sliding;
    float motion_time_speeding;

public:
    TrapezoidalMotionProfile(float distance, float velocity_max, float acceleration);
    float get_distance(float time);
    float get_velocity(float time);
    float get_time();

};