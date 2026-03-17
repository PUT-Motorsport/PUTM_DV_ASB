/*
 * timer.cpp
 *
 *  Created on: 6 kwi 2022
 *      Author: Piotr Lesicki
 */

#include "timer.hpp"

Timer::Timer()
{
	this->start_time = HAL_GetTick();
}

Timer::Timer(uint32_t timeout) : timeout(timeout)
{
	this->start_time = HAL_GetTick();
}

void Timer::setTimeOut(uint32_t timeout)
{
	this->timeout = timeout;
}

void Timer::restart()
{
	this->start_time = HAL_GetTick();
}

bool Timer::checkIfTimedOutThenReset()
{
	if(this->timeout == 0) return false;

	if(this->getPassedTime() >= this->timeout)
	{
		this->restart();
		return true;
	}

	return false;
}

uint32_t Timer::getPassedTime()
{
	return HAL_GetTick() - this->start_time;
}

uint32_t Timer::getPassedTimeThenReset()
{
	volatile uint32_t passed_time = HAL_GetTick() - this->start_time;
	this->restart();
	return passed_time;
}
