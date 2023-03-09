#pragma once
#include <map>

class SigmoidMotionProfile {
public:
	enum class SigmoidParameter {
		DISTANCE,
		VELOCITY,
		ACCELERATION,
		JERK,
		TIME
	};

	enum class SigmoidPhase {
		ACCELERATE_BEGIN,
		ACCELERATE_RETAIN,
		ACCELERATE_END,
		DRIFT,
		DECELERATE_BEGIN,
		DECELERATE_RETAIN,
		DECELERATE_END,
	};

	struct SigmoidPhaseAnchors {
		float time_phase_begin;       // time the phase begins
		float time_phase_section;     // time the phase last
		float time_phase_end;         // time the phase ends
		float distance_phase_begin;   // distance the phase begins
		float distance_phase_section; // distance the phase last
		float distance_phase_end;     // distance the phase ends
	};

	struct SigmoidCubicConstants {
		float cubic_degree_third;
		float cubic_degree_second;
		float cubic_degree_first;
		float cubic_degree_zero;
	};

	SigmoidMotionProfile                          (float distance_total, float velocity_max, float acceleration_max, float jerk);
	float               get_distance_velocity     (float progress_distance);
	float               get_distance_acceleration (float progress_distance);
	float               get_distance_jerk         (float progress_distance);
	float               get_distance_time         (float progress_distance);
	float               get_time_distance         (float progress_time);
	float               get_time_velocity         (float progress_time);
	float               get_time_acceleration     (float progress_time);
	float               get_time_jerk             (float progress_time);
	float               get_time_end              ();
	SigmoidPhase        get_phase                 (float progress_time);
	SigmoidPhaseAnchors get_anchors               (SigmoidPhase anchor_phase);
private:
	float distance_total;
	float velocity_max;
	float velocity_max_actual;
	float acceleration_max;
	float acceleration_max_actual;
	float jerk;
	std::map<SigmoidPhase, SigmoidPhaseAnchors> phase_anchors;

	float sigmoid_value(SigmoidParameter sigmoid_parameter, float progress_time);
};