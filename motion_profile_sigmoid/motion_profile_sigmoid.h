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
		float time_phase_begin;   // time the phase begins
		float time_phase_section; // time the phase last
		float time_phase_end;     // time the phase ends
	};

	SigmoidMotionProfile(float distance_total, float velocity_max, float acceleration_max, float jerk);
	float get_distance(float progress_time);
	float get_velocity(float progress_time);
	float get_acceleration(float progress_time);
	float get_jerk(float progress_time);
	float get_time_end();
	SigmoidPhase get_phase(float progress_time);
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