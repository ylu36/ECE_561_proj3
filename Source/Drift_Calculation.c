//#include <MKL25Z4.H>

#include <math.h>
#include <arm_math.h>
#include "Drift_Calculation.h"
#include "trig_approx.h"

#define RAD 0.017453292
#define RAD_INV 57.2957795
//#define RAD_INV_Q 
void Compute_Current(float speed_water, float angle_heading, float speed_ground, float angle_track, 
	float * speed_current, float * angle_current){
	float angle_drift_rad = 0.0;
	float local_speed_current, local_angle_current;
	float temp, rad_inv_q, q_180;
	float oneenighty = 180;
	float rad_inv = 57.2957795;
	/*
	q31_t q_speed_water, q_angle_heading, q_speed_ground, q_angle_track;
	arm_float_to_q31(&speed_water, &q_speed_water, 1);
	arm_float_to_q31(&angle_heading, &q_angle_heading, 1);
	arm_float_to_q31(&speed_ground, &q_speed_ground, 1);	
	arm_float_to_q31(&angle_track, &q_angle_track, 1);	
	arm_float_to_q31(&rad_inv, &rad_inv_q, 1);
	arm_float_to_q31(&oneenighty, &q_180, 1);
	
  arm_sub_q31(&q_angle_track, &q_angle_heading, &temp);		*/
	angle_drift_rad = (angle_track - angle_heading)*RAD;
	if (fabs(speed_ground) < MIN_SPEED_TOLERANCE) { 		// Check for special cases - no relative motion
		// not moving relative to ground, so current direction is opposite of heading through water
		local_angle_current = angle_heading-180;
		local_speed_current = speed_water;
	}	else if (fabs(speed_water) < MIN_SPEED_TOLERANCE) { 
		// not moving relative to water, so current direction is track over ground
		local_angle_current = angle_heading;
		local_speed_current = speed_ground;
	}	else if (fabs(angle_drift_rad) < MIN_ANGLE_TOLERANCE) { 	// Check for special cases - no angular difference
		// track and heading are same: case IV
		local_angle_current = angle_track;
		local_speed_current = speed_ground - speed_water;
	} else if (fabs(angle_drift_rad - 180) < MIN_ANGLE_TOLERANCE) { 
		// track and heading are opposite: case III
		local_angle_current = angle_heading-180;
		local_speed_current = speed_ground + speed_water;
	} else {
		local_speed_current = sqrt(speed_ground*speed_ground + speed_water*speed_water - 2*speed_water*speed_ground*cos_32(angle_drift_rad));
		temp = sin_32(angle_drift_rad)*speed_ground/local_speed_current;
		if (temp > 1)
			local_angle_current = 90 + angle_heading;
		else if (temp < -1)
			local_angle_current =  angle_heading - 90;
		else 
			local_angle_current = (180-asin(temp)*RAD_INV) + angle_heading;
	}	
	if (local_angle_current < 0)
		local_angle_current += 360;
	if (local_angle_current >= 360)
		local_angle_current -= 360;

	*speed_current = local_speed_current;
	*angle_current = local_angle_current;
}
	
