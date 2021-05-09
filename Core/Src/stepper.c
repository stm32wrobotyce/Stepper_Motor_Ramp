/*
 * stepper.c
 *
 *  Created on: 10 mar 2021
 *      Author: piotr
 */
#include <math.h>

#include "main.h"

void stepper_init(struct stepper_s *_stepper, TIM_HandleTypeDef *_htim, uint32_t _channel)
{
	_stepper->timer.htim = _htim;
	_stepper->timer.channel = _channel;

	_stepper->target_speed_achieved = 0;
}

void stepper_set_ramp_profile(struct stepper_s *_stepper, profile_mode _mode, uint32_t _target_speed, uint32_t _accel, uint32_t _decel)
{
	_stepper->profile.mode = _mode;
	_stepper->profile.target_speed = _target_speed * MICRO_STEP;
	_stepper->profile.accel = _accel * MICRO_STEP;
	_stepper->profile.decel = _decel * MICRO_STEP;
}

void stepper_set_continous(struct stepper_s *_stepper, direction _dir)
{
	_stepper->mode = continous;

	stepper_set_direction(_stepper, _dir);
	stepper_ramp_init(_stepper);

	HAL_TIM_PWM_Start_IT(_stepper->timer.htim, _stepper->timer.channel);
}

void stepper_set_angle(struct stepper_s *_stepper, direction _dir, uint32_t _angle)
{
	_stepper->mode = angle;

	_stepper->ustep_counter = 0;
	_stepper->usteps_to_count = _angle * (STEP_PER_REVOLUTION * MICRO_STEP) / 360;

	if(0 == _stepper->usteps_to_count)
	{
		stepper_imediate_stop(_stepper);
		return;
	}

	stepper_set_direction(_stepper, _dir);
	stepper_ramp_init(_stepper);

	HAL_TIM_PWM_Start_IT(_stepper->timer.htim, _stepper->timer.channel);
}

void stepper_imediate_stop(struct stepper_s *_stepper)
{
	_stepper->mode = idle;

	__HAL_TIM_SET_COMPARE(_stepper->timer.htim, _stepper->timer.channel, 0);
	HAL_TIM_PWM_Stop(_stepper->timer.htim, _stepper->timer.channel);
}

void stepper_stop_with_profile(struct stepper_s *_stepper)
{
	_stepper->mode = stop;
	_stepper->ustep_counter = 0;
}

void stepper_set_direction(struct stepper_s *_stepper, direction _dir)
{
	if(_dir == CCW)
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, CCW);
	else if(_dir == CW)
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, CW);
}

void stepper_set_speed(struct stepper_s *_stepper, uint32_t _speed)
{
	uint32_t counter, freq;

	_stepper->actual_speed = _speed;

	if(_speed > STEPPER_MOTOR_MAX_SPEED_USTEP_PER_SEC)
	{
		_speed = STEPPER_MOTOR_MAX_SPEED_USTEP_PER_SEC;
	}
	else if(_speed == 0)
	{
		stepper_imediate_stop(_stepper);
		return;
	}

	freq = (_speed * (STEPPER_MOTOR_MAX_FREQ_HZ - STEPPER_MOTOR_MIN_FREQ_HZ))/STEPPER_MOTOR_MAX_SPEED_USTEP_PER_SEC;
	counter = HAL_RCC_GetPCLK1Freq() / (_stepper->timer.htim->Init.Prescaler * freq);

	__HAL_TIM_SET_COUNTER(_stepper->timer.htim, 0);
	__HAL_TIM_SET_AUTORELOAD(_stepper->timer.htim, counter - 1);
	__HAL_TIM_SET_COMPARE(_stepper->timer.htim, _stepper->timer.channel, (counter/2) - 1);
}

void stepper_ramp_init(struct stepper_s *_stepper)
{
	switch (_stepper->profile.mode)
	{
		case CONSTANT_SPEED:
		{
			stepper_set_speed(_stepper, _stepper->profile.target_speed);
			break;
		}
		case LINEAR_SPEED:
		{
			stepper_ramp_init_linear_mode(_stepper);
			break;
		}
		default:
			break;
	}
}

void stepper_ramp_init_linear_mode(struct stepper_s *_stepper)
{
	stepper_set_speed(_stepper, _stepper->profile.accel/STEPPER_MOTOR_TASK_FREQUENCY);

	if(continous == _stepper->mode)
	{
		_stepper->ramp.usteps_to_full_speed = (_stepper->profile.target_speed * _stepper->profile.target_speed) / (2 *_stepper->profile.accel);
		_stepper->ramp.usteps_to_stop = (_stepper->profile.target_speed * _stepper->profile.target_speed) / (2 *_stepper->profile.decel);
	}
	else if(angle == _stepper->mode)
	{
		_stepper->ramp.usteps_to_full_speed = (_stepper->profile.target_speed * _stepper->profile.target_speed) / (2 *_stepper->profile.accel);
		_stepper->ramp.usteps_to_stop = (_stepper->profile.target_speed * _stepper->profile.target_speed) / (2 *_stepper->profile.decel);

		if(_stepper->usteps_to_count < (_stepper->ramp.usteps_to_full_speed + _stepper->ramp.usteps_to_stop))
		{
			_stepper->ramp.usteps_to_full_speed = (_stepper->usteps_to_count * _stepper->profile.decel) / (_stepper->profile.accel + _stepper->profile.decel);
			_stepper->ramp.usteps_to_stop = _stepper->usteps_to_count - _stepper->ramp.usteps_to_full_speed;
		}
	}
}

void stepper_ramp_process(struct stepper_s *_stepper)
{
	static uint32_t time_ms = 0;

	if(0 == time_ms)
		time_ms = HAL_GetTick();

	if((HAL_GetTick() - time_ms) < STEPPER_MOTOR_TASK_PERIOD)
		return;

	time_ms = HAL_GetTick();

	if(idle == _stepper->mode)
		return;

	switch (_stepper->profile.mode)
	{
		case CONSTANT_SPEED:
		{
			break;
		}
		case LINEAR_SPEED:
		{
			stepper_ramp_process_linear_mode(_stepper);
			break;
		}
		default:
			break;

	}
}

void stepper_ramp_process_linear_mode(struct stepper_s *_stepper)
{
	uint32_t speed = 0;

	if(continous == _stepper->mode)
	{
		if(_stepper->ustep_counter < _stepper->ramp.usteps_to_full_speed && _stepper->target_speed_achieved != 1)
		{
			speed = _stepper->actual_speed + (_stepper->profile.accel / STEPPER_MOTOR_TASK_FREQUENCY);
		}
		else
		{
			_stepper->target_speed_achieved = 1;
			speed = _stepper->profile.target_speed;
		}

		stepper_set_speed(_stepper, speed);
	}
	else if(angle == _stepper->mode)
	{
		if(_stepper->ustep_counter < _stepper->ramp.usteps_to_full_speed)
		{
			speed = _stepper->actual_speed + (_stepper->profile.accel / STEPPER_MOTOR_TASK_FREQUENCY);
		}
		else if((_stepper->usteps_to_count - _stepper->ustep_counter) < _stepper->ramp.usteps_to_stop)
		{
			speed = _stepper->actual_speed - (_stepper->profile.decel / STEPPER_MOTOR_TASK_FREQUENCY);
			if(speed < STEPPER_MOTOR_MIN_SPEED_USTEP_PER_SEC)
			{
				speed = STEPPER_MOTOR_MIN_SPEED_USTEP_PER_SEC;
			}
		}
		else
		{
			speed = _stepper->profile.target_speed;
		}

		stepper_set_speed(_stepper, speed);
	}
	else if(stop == _stepper->mode)
	{
		if(_stepper->ustep_counter < _stepper->ramp.usteps_to_stop)
		{
			speed = _stepper->actual_speed - (_stepper->profile.decel / STEPPER_MOTOR_TASK_FREQUENCY);

			if(speed < STEPPER_MOTOR_MIN_SPEED_USTEP_PER_SEC)
			{
				speed = 0;
			}

			stepper_set_speed(_stepper, speed);
		}
		else
		{
			stepper_imediate_stop(_stepper);
		}
	}
}
