/*
 * stepper.h
 *
 *  Created on: 10 mar 2021
 *      Author: piotr
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#define STEPPER_MOTOR_TASK_FREQUENCY	100
#define STEPPER_MOTOR_TASK_PERIOD		(1000/STEPPER_MOTOR_TASK_FREQUENCY)

#define STEPPER_MOTOR_MAX_FREQ_HZ		(MICRO_STEP * 1000)
#define STEPPER_MOTOR_MIN_FREQ_HZ		1

#define STEPPER_MOTOR_MAX_SPEED_USTEP_PER_SEC		STEPPER_MOTOR_MAX_FREQ_HZ
#define STEPPER_MOTOR_MIN_SPEED_USTEP_PER_SEC		(STEPPER_MOTOR_MAX_SPEED_USTEP_PER_SEC / 100)

#define STEP_PER_REVOLUTION			200
#define MICRO_STEP					32

#define CONSTANT_SPEED				0
#define LINEAR_SPEED				1

typedef uint32_t profile_mode;

typedef enum
{
	idle = 0,
	angle = 1,
	continous = 2,
	stop = 3
}stepper_mode;

typedef enum
{
	CCW = 0,
	CW = 1
}direction;

struct htim_s
{
	TIM_HandleTypeDef *htim;
	uint32_t channel;
};

struct speed_profile_s
{
	profile_mode mode;
	uint32_t target_speed;
	uint32_t accel;
	uint32_t decel;
};

struct ramp_s
{
	uint32_t usteps_to_full_speed;
	uint32_t usteps_to_stop;
};

struct stepper_s
{
	struct htim_s timer;

	struct speed_profile_s profile;
	struct ramp_s ramp;

	stepper_mode mode;
	volatile uint32_t ustep_counter;
	uint32_t usteps_to_count;
	uint32_t actual_speed;

	uint32_t target_speed_achieved;
};

void stepper_init(struct stepper_s *_stepper, TIM_HandleTypeDef *_htim, uint32_t _channel);
void stepper_set_ramp_profile(struct stepper_s *_stepper, profile_mode _mode, uint32_t _target_speed, uint32_t _accel, uint32_t _deaccel);
void stepper_set_continous(struct stepper_s *_stepper, direction _dir);
void stepper_set_angle(struct stepper_s *_stepper, direction _dir, uint32_t _angle);
void stepper_imediate_stop(struct stepper_s *_stepper);
void stepper_stop_with_profile(struct stepper_s *_stepper);
void stepper_set_direction(struct stepper_s *_stepper, direction _dir);
void stepper_set_speed(struct stepper_s *_stepper, uint32_t _speed);

void stepper_ramp_init(struct stepper_s *_stepper);
void stepper_ramp_init_linear_mode(struct stepper_s *_stepper);
void stepper_ramp_process(struct stepper_s *_stepper);
void stepper_ramp_process_linear_mode(struct stepper_s *_stepper);

#endif /* INC_STEPPER_H_ */
