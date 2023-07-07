/*
 * timer.hpp
 *
 *  Created on: 6 kwi 2022
 *      Author: Piotr Lesicki
 */

#ifndef INC_TIMER_HPP_
#define INC_TIMER_HPP_

#include "main.h"

class Timer
{
	public:
		Timer();
		Timer(uint32_t);

		void restart();
		void setTimeOut(uint32_t);

		bool checkIfTimedOutThenReset();

		uint32_t getPassedTime();
		uint32_t getPassedTimeThenReset();

	private:
		volatile uint32_t start_time;
		/*
		 * if timeout == 0 timer always will return false (timer deactivated)
		 */
		volatile uint32_t timeout = 0;
};

#endif /* INC_TIMER_HPP_ */
