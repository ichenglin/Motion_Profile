#include <map>
#include <cmath>
#include "motion_profile_sigmoid.h"

std::map<SigmoidMotionProfile::SigmoidPhase, SigmoidMotionProfile::SigmoidPhaseAnchors> sigmoid_phase_anchors(float distance_total, float velocity_max, float acceleration_max, float jerk);

SigmoidMotionProfile::SigmoidMotionProfile(float distance_total, float velocity_max, float acceleration_max, float jerk) {
	// initialize parameters
	this->distance_total   = distance_total;
	this->velocity_max     = velocity_max;
	this->acceleration_max = acceleration_max;
	this->jerk             = jerk;
	// calculate phase time
	this->phase_anchors           = sigmoid_phase_anchors(distance_total, velocity_max, acceleration_max, jerk);
	this->acceleration_max_actual = this->jerk * this->phase_anchors.at(SigmoidPhase::ACCELERATE_BEGIN).time_phase_end;
	this->velocity_max_actual     = this->sigmoid_value(SigmoidParameter::VELOCITY, this->phase_anchors.at(SigmoidPhase::ACCELERATE_END).time_phase_end);
}

float SigmoidMotionProfile::get_distance(float progress_time) {
	return this->sigmoid_value(SigmoidParameter::DISTANCE, progress_time);
}

float SigmoidMotionProfile::get_velocity(float progress_time) {
	return this->sigmoid_value(SigmoidParameter::VELOCITY, progress_time);
}

float SigmoidMotionProfile::get_acceleration(float progress_time) {
	return this->sigmoid_value(SigmoidParameter::ACCELERATION, progress_time);
}

float SigmoidMotionProfile::get_jerk(float progress_time) {
	return this->sigmoid_value(SigmoidParameter::JERK, progress_time);
}

SigmoidMotionProfile::SigmoidPhase SigmoidMotionProfile::get_phase(float progress_time) {
	for (int phase_index = 6; phase_index >= 0; phase_index--) {
		if (this->phase_anchors.at((SigmoidPhase)phase_index).time_phase_begin > progress_time) continue;
		return (SigmoidPhase)phase_index;
	}
}

float SigmoidMotionProfile::get_time_end() {
	return this->phase_anchors.at(SigmoidPhase::DECELERATE_END).time_phase_end;
}

float SigmoidMotionProfile::sigmoid_value(SigmoidParameter sigmoid_parameter, float progress_time) {
	SigmoidPhase progress_phase;
	float        time_progress_section;
	// find the corresponding phase by progress time
	for (int phase_index = 6; phase_index >= 0; phase_index--) {
		if (this->phase_anchors.at((SigmoidPhase) phase_index).time_phase_begin > progress_time) continue;
		progress_phase        = (SigmoidPhase) phase_index;
		time_progress_section = progress_time - this->phase_anchors.at((SigmoidPhase) phase_index).time_phase_begin;
		break;
	}
	// return phase values
	float value_phase_accumulate    = 0.0f;
	bool  parameter_type_accumulate = sigmoid_parameter == SigmoidParameter::VELOCITY || sigmoid_parameter == SigmoidParameter::DISTANCE;
	for (int phase_index = (!parameter_type_accumulate ? (int) progress_phase : 0); phase_index <= (int) progress_phase; phase_index++) {
		// phase progress time
		SigmoidPhaseAnchors phase_object = this->phase_anchors.at((SigmoidPhase) phase_index);
		float time_phase_section         = (phase_index != (int)progress_phase ? phase_object.time_phase_section : time_progress_section);
		// phase equations
		switch ((SigmoidPhase) phase_index) {
			case SigmoidPhase::ACCELERATE_BEGIN:
				if      (sigmoid_parameter == SigmoidParameter::JERK)         return                    this->jerk;
				else if (sigmoid_parameter == SigmoidParameter::ACCELERATION) return                    this->jerk * time_phase_section;
				else if (sigmoid_parameter == SigmoidParameter::VELOCITY)     value_phase_accumulate += (1 / 2.0f) * this->jerk * std::pow(time_phase_section, 2);
				else if (sigmoid_parameter == SigmoidParameter::DISTANCE)     value_phase_accumulate += (1 / 6.0f) * this->jerk * std::pow(time_phase_section, 3);
				break;
			case SigmoidPhase::ACCELERATE_RETAIN:
				if      (sigmoid_parameter == SigmoidParameter::JERK)         return                    0.0f;
				else if (sigmoid_parameter == SigmoidParameter::ACCELERATION) return                    this->acceleration_max_actual;
				else if (sigmoid_parameter == SigmoidParameter::VELOCITY)     value_phase_accumulate += this->acceleration_max_actual * time_phase_section;
				else if (sigmoid_parameter == SigmoidParameter::DISTANCE)     value_phase_accumulate += (1 / 2.0f) * this->acceleration_max_actual * std::pow(time_phase_section, 2);
				break;
			case SigmoidPhase::ACCELERATE_END:
				if      (sigmoid_parameter == SigmoidParameter::JERK)         return                    (-1) * this->jerk;
				else if (sigmoid_parameter == SigmoidParameter::ACCELERATION) return                    this->acceleration_max_actual - this->jerk * time_phase_section;
				else if (sigmoid_parameter == SigmoidParameter::VELOCITY)     value_phase_accumulate += this->acceleration_max_actual * time_phase_section - (1 / 2.0f) * this->jerk * std::pow(time_phase_section, 2);
				else if (sigmoid_parameter == SigmoidParameter::DISTANCE)     value_phase_accumulate += (-1 / 6.0f) * this->jerk * std::pow(time_phase_section, 3) + (1 / 2.0f) * this->acceleration_max_actual * std::pow(time_phase_section, 2);
				break;
			case SigmoidPhase::DRIFT:
				if      (sigmoid_parameter == SigmoidParameter::JERK)         return                    0.0f;
				else if (sigmoid_parameter == SigmoidParameter::ACCELERATION) return                    0.0f;
				else if (sigmoid_parameter == SigmoidParameter::VELOCITY)     value_phase_accumulate += 0.0f;
				else if (sigmoid_parameter == SigmoidParameter::DISTANCE)     value_phase_accumulate += 0.0f;
				break;
			case SigmoidPhase::DECELERATE_BEGIN:
				if      (sigmoid_parameter == SigmoidParameter::JERK)         return                    (-1) * this->jerk;
				else if (sigmoid_parameter == SigmoidParameter::ACCELERATION) return                    (-1) * this->jerk * time_phase_section;
				else if (sigmoid_parameter == SigmoidParameter::VELOCITY)     value_phase_accumulate -= (1 / 2.0f) * this->jerk * std::pow(time_phase_section, 2);
				else if (sigmoid_parameter == SigmoidParameter::DISTANCE)     value_phase_accumulate -= (1 / 6.0f) * this->jerk * std::pow(time_phase_section, 3);
				break;
			case SigmoidPhase::DECELERATE_RETAIN:
				if      (sigmoid_parameter == SigmoidParameter::JERK)         return                    0.0f;
				else if (sigmoid_parameter == SigmoidParameter::ACCELERATION) return                    (-1) * this->acceleration_max_actual;
				else if (sigmoid_parameter == SigmoidParameter::VELOCITY)     value_phase_accumulate -= this->acceleration_max_actual * time_phase_section;
				else if (sigmoid_parameter == SigmoidParameter::DISTANCE)     value_phase_accumulate -= (1 / 2.0f) * this->acceleration_max_actual * std::pow(time_phase_section, 2);
				break;
			case SigmoidPhase::DECELERATE_END:
				if      (sigmoid_parameter == SigmoidParameter::JERK)         return                    this->jerk;
				else if (sigmoid_parameter == SigmoidParameter::ACCELERATION) return                    (-1) * this->acceleration_max_actual + this->jerk * time_phase_section;
				else if (sigmoid_parameter == SigmoidParameter::VELOCITY)     value_phase_accumulate -= this->acceleration_max_actual * time_phase_section - (1 / 2.0f) * this->jerk * std::pow(time_phase_section, 2);
				else if (sigmoid_parameter == SigmoidParameter::DISTANCE)     value_phase_accumulate -= (-1 / 6.0f) * this->jerk * std::pow(time_phase_section, 3) + (1 / 2.0f) * this->acceleration_max_actual * std::pow(time_phase_section, 2);
				break;
		};
		if (sigmoid_parameter == SigmoidParameter::DISTANCE) {
			value_phase_accumulate += this->sigmoid_value(SigmoidParameter::VELOCITY, phase_object.time_phase_begin) * time_phase_section;
		}
	}
	return value_phase_accumulate;
}

std::map<SigmoidMotionProfile::SigmoidPhase, SigmoidMotionProfile::SigmoidPhaseAnchors> sigmoid_phase_anchors(float distance_total, float velocity_max, float acceleration_max, float jerk) {
	struct TimeAnchors {
		float time_accelerate;
		float time_retain;
		float time_drift;
	} time_accelerate_anchors;
	// calculate maximum time in acceleration and velocity limits
	float time_accelerate_max = acceleration_max / jerk;
	float time_velocity_max   = velocity_max / acceleration_max;
	time_accelerate_anchors   = {std::min(time_accelerate_max, time_velocity_max), 0.0f, 0.0f};
	// calculate maximum retain time in velocity limit
	float velocity_accelerate_full      = jerk * std::pow(time_accelerate_anchors.time_accelerate, 2);
	float velocity_retain               = velocity_max - velocity_accelerate_full;
	float time_retain_max               = velocity_retain / (jerk * time_accelerate_anchors.time_accelerate);
	time_accelerate_anchors.time_retain = time_retain_max;
	// calculate best shape for exact distance
	float velocity_phase[3];
	velocity_phase[0] = (1 / 2.0f) * jerk * std::pow(time_accelerate_anchors.time_accelerate, 2);
	velocity_phase[1] = (jerk * time_accelerate_anchors.time_accelerate) * time_accelerate_anchors.time_retain;
	velocity_phase[2] = (jerk * time_accelerate_anchors.time_accelerate) * time_accelerate_anchors.time_accelerate - (1 / 2.0f) * jerk * std::pow(time_accelerate_anchors.time_accelerate, 2);
	float distance_phase[3];
	distance_phase[0] = (1 / 6.0f) * jerk * std::pow(time_accelerate_anchors.time_accelerate, 3);
	distance_phase[1] = (1 / 2.0f) * (jerk * time_accelerate_anchors.time_accelerate) * std::pow(time_accelerate_anchors.time_retain, 2) + velocity_phase[0] * time_accelerate_anchors.time_retain;
	distance_phase[2] = (-1 / 6.0f) * jerk * std::pow(time_accelerate_anchors.time_accelerate, 3) + (1 / 2.0f) * (jerk * time_accelerate_anchors.time_accelerate) * std::pow(time_accelerate_anchors.time_accelerate, 2) + (velocity_phase[0] + velocity_phase[1]) * time_accelerate_anchors.time_accelerate;
	if (distance_total / 2 > distance_phase[0] + distance_phase[1] + distance_phase[2]) {
		// have more than enough distance, yay! (calculate new drift)
		float distance_accelerate          = distance_phase[0] + distance_phase[1] + distance_phase[2];
		float velocity_accelerate          = velocity_phase[0] + velocity_phase[1] + velocity_phase[2];
		float distance_drift               = distance_total - (2 * distance_accelerate);
		float time_drift                   = distance_drift / velocity_accelerate;
		time_accelerate_anchors.time_drift = time_drift;
	} else if (velocity_phase[1] <= 0) {
		// don't have enough for full accelerate (calculate new accelerate max)
		float time_accelerate   = std::pow((distance_total / (2 * jerk)), 1 / 3.0f);
		time_accelerate_anchors = {time_accelerate, 0.0f, 0.0f};
	} else {
		// have enough for full accelerate, but not retain
		float equation_a                    = (1 / 2.0f) * jerk * time_accelerate_anchors.time_accelerate;
		float equation_b                    = (3 / 2.0f) * jerk * std::pow(time_accelerate_anchors.time_accelerate, 2);
		float equation_c                    = jerk * std::pow(time_accelerate_anchors.time_accelerate, 3);
		float time_retain                   = ((-1) * equation_b + std::sqrt(std::pow(equation_b, 2) - (4 * equation_a * (equation_c - (distance_total / 2))))) / (2 * equation_a);
		time_accelerate_anchors.time_retain = time_retain;
	}
	// restructure result
	float phases_time_full[7] = {
		time_accelerate_anchors.time_accelerate, time_accelerate_anchors.time_retain, time_accelerate_anchors.time_accelerate,
		time_accelerate_anchors.time_drift,
		time_accelerate_anchors.time_accelerate, time_accelerate_anchors.time_retain, time_accelerate_anchors.time_accelerate
	};
	std::map<SigmoidMotionProfile::SigmoidPhase, SigmoidMotionProfile::SigmoidPhaseAnchors> phase_anchors_new;
	float distance_phase_accumulate = 0.0f;
	for (int phase_index = 0; phase_index < 7; phase_index++) {
		phase_anchors_new.insert(std::pair<SigmoidMotionProfile::SigmoidPhase, SigmoidMotionProfile::SigmoidPhaseAnchors>((SigmoidMotionProfile::SigmoidPhase)phase_index, {
			distance_phase_accumulate,                                 // float time_phase_begin;   (time the phase begins)
			phases_time_full[phase_index],                             // float time_phase_section; (time the phase last)
			distance_phase_accumulate + phases_time_full[phase_index], // float time_phase_end;     (time the phase ends)
			}));
		distance_phase_accumulate += phases_time_full[phase_index];
	}
	return phase_anchors_new;
}