#include <map>
#include <array>
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
	this->velocity_max_actual = this->sigmoid_value(SigmoidParameter::VELOCITY, this->phase_anchors.at(SigmoidPhase::ACCELERATE_END).time_phase_end);
	// debug
	for (int i = 0; i < 7; i++) {
		printf("[Phase #%d] Time: %f End: %f Distance: %f Velocity: %f\n", i, this->phase_anchors.at((SigmoidPhase) i).time_phase_section, this->phase_anchors.at((SigmoidPhase) i).time_phase_end, this->get_distance(this->phase_anchors.at((SigmoidPhase)i).time_phase_end) - this->get_distance(this->phase_anchors.at((SigmoidPhase)i).time_phase_begin), this->get_velocity(this->phase_anchors.at((SigmoidPhase)i).time_phase_end));
	}
	printf("\nVelocity Max: %f\nAcceleration Max: %f\n", velocity_max_actual, acceleration_max_actual);
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
	float value_phase_accumulate = 0.0f;
	bool  parameter_type_accumulate = sigmoid_parameter == SigmoidParameter::VELOCITY || sigmoid_parameter == SigmoidParameter::DISTANCE;
	for (int phase_index = (!parameter_type_accumulate ? (int) progress_phase : 0); phase_index <= (int) progress_phase; phase_index++) {
		// phase progress time
		SigmoidPhaseAnchors phase_object = this->phase_anchors.at((SigmoidPhase) phase_index);
		float time_phase_section = (phase_index != (int)progress_phase ? phase_object.time_phase_section : time_progress_section);
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
	float time_velocity_max = velocity_max / acceleration_max;
	time_accelerate_anchors   = {std::min(time_accelerate_max, time_velocity_max), 0.0f, 0.0f};
	// calculate maximum retain time in velocity limit
	float velocity_accelerate_full = jerk * std::pow(time_accelerate_anchors.time_accelerate, 2);
	float velocity_retain          = velocity_max - velocity_accelerate_full;
	float time_retain_max          = velocity_retain / (jerk * time_accelerate_anchors.time_accelerate);
	time_accelerate_anchors.time_retain = time_retain_max;
	// idk
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
		float distance_accelerate = distance_phase[0] + distance_phase[1] + distance_phase[2];
		float velocity_accelerate = velocity_phase[0] + velocity_phase[1] + velocity_phase[2];
		float distance_drift = distance_total - (2 * distance_accelerate);
		float time_drift = distance_drift / velocity_accelerate;
		time_accelerate_anchors.time_drift = time_drift;
		printf("Situation #1\n");
	} else if (distance_total / 2 > distance_phase[0] + distance_phase[2]) {
		// have enough for full accelerate, but not retain (calculate new retain)
		float distance_retain = (distance_total / 2) - distance_phase[0] - distance_phase[2];
		float time_retain     = ((-1) * velocity_phase[0] + std::sqrt(std::pow(velocity_phase[0], 2) + 2 * acceleration_max * distance_retain)) / acceleration_max;
		time_accelerate_anchors.time_retain = time_retain;
		printf("Situation #2\n");
	} else {
		// don't have enough for full accelerate (calculate new accelerate max)
		float time_accelerate = std::pow((distance_total / (2 * jerk)), 1 / 3.0f);
		time_accelerate_anchors = {time_accelerate, 0.0f, 0.0f};
		printf("Situation #3\n");
	}
	// conclude
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















	
	
	
	
	
	// OLD
	// OLD
	// OLD
	
	/*float phases_time_accelerate[3];
	// the velocity if accelerate to full triangle
	float velocity_accelerate_max = std::pow(acceleration_max, 2) / jerk;
	// check maximum velocity reached
	if (velocity_max >= velocity_accelerate_max) {
		// with retain
		float time_accelerate_max    = acceleration_max / jerk;
		float time_accelerate_retain = (velocity_max - time_accelerate_max) / acceleration_max;
		float phases_time_accelerate_new[3]     = {time_accelerate_max, time_accelerate_retain, time_accelerate_max};
		std::copy(std::begin(phases_time_accelerate_new), std::end(phases_time_accelerate_new), std::begin(phases_time_accelerate));
	} else {
		// without retain
		float time_accelerate_full = std::sqrt(velocity_max * jerk) / jerk;
		float phases_time_accelerate_new[3]   = {time_accelerate_full, 0.0f, time_accelerate_full};
		std::copy(std::begin(phases_time_accelerate_new), std::end(phases_time_accelerate_new), std::begin(phases_time_accelerate));
	}
	float acceleration_max_actual = jerk * phases_time_accelerate[0];
	printf("A MAX: %f\n", acceleration_max_actual);
	for (int i = 0; i < 3; i++) printf("T #%d: %f\n", i, phases_time_accelerate[i]);
	// the absolute velocity of each phases
	float phases_velocity_accelerate[3]; // (1 / 2.0f) * this->jerk * std::pow(time_phase_section, 2); this->acceleration_max_actual * time_phase_section;
	phases_velocity_accelerate[0] = 0                             + (1 / 2.0f) * jerk       * std::pow(phases_time_accelerate[0], 2);
	phases_velocity_accelerate[1] = phases_velocity_accelerate[0] + acceleration_max_actual * phases_time_accelerate[1];
	phases_velocity_accelerate[2] = phases_velocity_accelerate[1] - (1 / 2.0f) * jerk       * std::pow(phases_time_accelerate[2], 2) + acceleration_max_actual * phases_time_accelerate[2];
	for (int i = 0; i < 3; i++) printf("V #%d: %f\n", i, phases_velocity_accelerate[i]);
	// the offset distance of each phases
	float phases_distane_accelerate[3];
	phases_distane_accelerate[0] = (1 / 6.0f) * jerk                    * std::pow(phases_time_accelerate[0], 3);
	phases_distane_accelerate[1] = (1 / 2.0f) * acceleration_max_actual * std::pow(phases_time_accelerate[1], 2)                                                                                 + phases_velocity_accelerate[0] * phases_time_accelerate[1];
	phases_distane_accelerate[2] = (-1 / 6.0f) * jerk                   * std::pow(phases_time_accelerate[2], 3) + (1 / 2.0f) * acceleration_max_actual * std::pow(phases_time_accelerate[2], 2) + phases_velocity_accelerate[1] * phases_time_accelerate[2];
	for (int i = 0; i < 3; i++) printf("D #%d: %f\n", i, phases_distane_accelerate[i]);
	float phases_time_full[7];
	// check maximum distance reached
	if (distance_total >= 2 * (phases_distane_accelerate[0] + phases_distane_accelerate[1] + phases_distane_accelerate[2])) {
		// we have enough distance for drift, yay!
		float distance_drift = distance_total - (2 * (phases_distane_accelerate[0] + phases_distane_accelerate[1] + phases_distane_accelerate[2]));
		float time_drift = distance_drift / phases_velocity_accelerate[2];
		float phases_time_full_new[7] = {
			phases_time_accelerate[0], phases_time_accelerate[1], phases_time_accelerate[2], // accelerate
			time_drift,                                                                      // drift
			phases_time_accelerate[2], phases_time_accelerate[1], phases_time_accelerate[0]  // decelerate
		};
		std::copy(std::begin(phases_time_full_new), std::end(phases_time_full_new), std::begin(phases_time_full));*/
	/* } else if (distance_total >= 2 * (phases_distane_accelerate[0] + phases_distane_accelerate[2])) {
		printf("Situation #2\n");
		// not enough distance for drift, but enough for acceleration retain, yay!
		float distance_retain = (distance_total - (2 * (phases_distane_accelerate[0] + phases_distane_accelerate[2]))) / 2;
		printf("Distance Retain: %f (%f %f)\n", distance_retain, phases_distane_accelerate[0], phases_distane_accelerate[2]);
		float time_retain = ((-1) * phases_velocity_accelerate[0] + std::sqrt(std::pow(phases_velocity_accelerate[0], 2) + 2 * acceleration_max_actual * distance_retain)) / acceleration_max_actual;
		float phases_time_full_new[7] = {
			phases_time_accelerate[0], time_retain, phases_time_accelerate[2], // accelerate
			0.0f,                                                              // drift
			phases_time_accelerate[2], time_retain, phases_time_accelerate[0]  // decelerate
		};
		std::copy(std::begin(phases_time_full_new), std::end(phases_time_full_new), std::begin(phases_time_full));*/
		/*} else {
		printf("Situation #3\n");
		// 
		float phases_time_full_new[7] = {
			0.0f, 0.0f, 0.0f, // accelerate
			0.0f,             // drift
			0.0f, 0.0f, 0.0f  // decelerate
		};
		std::copy(std::begin(phases_time_full_new), std::end(phases_time_full_new), std::begin(phases_time_full));
	}
	// cache the time for later user
	std::map<SigmoidMotionProfile::SigmoidPhase, SigmoidMotionProfile::SigmoidPhaseAnchors> phase_anchors_new;
	float distance_phase_accumulate = 0.0f;
	for (int phase_index = 0; phase_index < 7; phase_index++) {
		phase_anchors_new.insert(std::pair<SigmoidMotionProfile::SigmoidPhase, SigmoidMotionProfile::SigmoidPhaseAnchors>((SigmoidMotionProfile::SigmoidPhase) phase_index, {
			distance_phase_accumulate,                                 // float time_phase_begin;   (time the phase begins)
			phases_time_full[phase_index],                             // float time_phase_section; (time the phase last)
			distance_phase_accumulate + phases_time_full[phase_index], // float time_phase_end;     (time the phase ends)
		}));
		distance_phase_accumulate += phases_time_full[phase_index];
	}
	return phase_anchors_new;*/
}