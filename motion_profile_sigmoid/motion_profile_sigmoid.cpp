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
}

float SigmoidMotionProfile::get_velocity(float progress_time) {
	return this->sigmoid_value(SigmoidParameter::VELOCITY, progress_time);
}

float SigmoidMotionProfile::get_acceleration(float progress_time) {
	return this->sigmoid_value(SigmoidParameter::ACCELERATION, progress_time);
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
	switch (sigmoid_parameter) {
		case SigmoidParameter::VELOCITY: {
			float velocity_phase_accumulate = 0.0f;
			for (int phase_index = 0; phase_index <= (int) progress_phase; phase_index++) {
				float time_phase_section = (phase_index != (int) progress_phase ? this->phase_anchors.at((SigmoidPhase)phase_index).time_phase_section : time_progress_section);
				if      (phase_index == (int)SigmoidPhase::ACCELERATE_BEGIN)   velocity_phase_accumulate += (1 / 2.0f) * this->jerk * std::pow(time_phase_section, 2);
				else if (phase_index == (int) SigmoidPhase::ACCELERATE_RETAIN) velocity_phase_accumulate += this->acceleration_max_actual * time_phase_section;
				else if (phase_index == (int) SigmoidPhase::ACCELERATE_END)    velocity_phase_accumulate += this->acceleration_max_actual * time_phase_section - (1 / 2.0f) * this->jerk * std::pow(time_phase_section, 2);
				else if (phase_index == (int) SigmoidPhase::DRIFT)             velocity_phase_accumulate += 0.0f;
				else if (phase_index == (int) SigmoidPhase::DECELERATE_BEGIN)  velocity_phase_accumulate -= (1 / 2.0f) * this->jerk * std::pow(time_phase_section, 2);
				else if (phase_index == (int) SigmoidPhase::DECELERATE_RETAIN) velocity_phase_accumulate -= this->acceleration_max_actual * time_phase_section;
				else if (phase_index == (int) SigmoidPhase::DECELERATE_END)    velocity_phase_accumulate -= this->acceleration_max_actual * time_phase_section - (1 / 2.0f) * this->jerk * std::pow(time_phase_section, 2);
			}
			return velocity_phase_accumulate;
			break;
		};
		case SigmoidParameter::ACCELERATION:
			if (progress_phase == SigmoidPhase::ACCELERATE_BEGIN)  return this->jerk * time_progress_section;
			if (progress_phase == SigmoidPhase::ACCELERATE_RETAIN) return this->acceleration_max_actual;
			if (progress_phase == SigmoidPhase::ACCELERATE_END)    return this->acceleration_max_actual - this->jerk * time_progress_section;
			if (progress_phase == SigmoidPhase::DRIFT)             return 0.0f;
			if (progress_phase == SigmoidPhase::DECELERATE_BEGIN)  return (-1) * this->jerk * time_progress_section;
			if (progress_phase == SigmoidPhase::DECELERATE_RETAIN) return (-1) * this->acceleration_max_actual;
			if (progress_phase == SigmoidPhase::DECELERATE_END)    return (-1) * (this->acceleration_max_actual - this->jerk * time_progress_section);
			break;
	}
}

std::map<SigmoidMotionProfile::SigmoidPhase, SigmoidMotionProfile::SigmoidPhaseAnchors> sigmoid_phase_anchors(float distance_total, float velocity_max, float acceleration_max, float jerk) {
	float phases_time_accelerate[3];
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
	// first integration (velocity) of each phases' acceleration
	float phases_velocity_accelerate[3] = {
		(1 / 2.0f) * jerk             * std::pow(phases_time_accelerate[0], 2),
		acceleration_max_actual       * phases_time_accelerate[1],
		(1 / 2.0f) * jerk             * std::pow(phases_time_accelerate[2], 2),
	};
	// second integration (distance) of each phases' acceleration
	float phases_distane_accelerate[3] = {
		(1 / 6.0f) * jerk                    * std::pow(phases_time_accelerate[0], 3),
		(1 / 2.0f) * acceleration_max_actual * std::pow(phases_time_accelerate[1], 2),
		(1 / 6.0f) * jerk                    * std::pow(phases_time_accelerate[2], 3)
	};
	float distance_accelerate = phases_distane_accelerate[0] + phases_distane_accelerate[1] + phases_distane_accelerate[2];
	float phases_time_full[7];
	// check maximum distance reached
	if (distance_total >= 2 * distance_accelerate) {
		// we have enough distance, yay!
		float distance_retain = distance_total - (2 * distance_accelerate);
		float time_retain = ((-1 * phases_velocity_accelerate[0]) + std::sqrt(std::pow(phases_velocity_accelerate[0], 2) + 2 * acceleration_max_actual * distance_retain)) / acceleration_max_actual;
		float phases_time_full_new[7] = {
			phases_time_accelerate[0], phases_time_accelerate[1], phases_time_accelerate[2], // accelerate
			time_retain,                                                                     // retain
			phases_time_accelerate[2], phases_time_accelerate[1], phases_time_accelerate[0]  // decelerate
		};
		std::copy(std::begin(phases_time_full_new), std::end(phases_time_full_new), std::begin(phases_time_full));
	} else {
		// fuck off, lazy to make this now
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
	return phase_anchors_new;
}