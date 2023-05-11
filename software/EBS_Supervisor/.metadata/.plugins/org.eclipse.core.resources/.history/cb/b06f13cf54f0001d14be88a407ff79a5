/*
 * assi.cpp
 *
 *  Created on: 27 wrz 2022
 *      Author: pile
 */

#include <gpioHandler.hpp>

void GpioOutHandler::add(GpioOutElement* gpio, uint32_t on_durration_ms, uint32_t off_durration_ms, HandlerToken* token)
{
	elements.emplace_back(gpio, on_durration_ms, off_durration_ms, token);
	elements.back().timer.restart();
}

GpioOutHandler::HandlerStruct::HandlerStruct(GpioOutElement* gpio, uint32_t on_durration, uint32_t off_durration, HandlerToken* token) : gpio(gpio), on_durration(on_durration), off_durration(off_durration), token(token) { }

void GpioOutHandler::handle()
{
	for (auto& [gpio, timer, on_durration, off_durration, current_state, token] : elements)
	{
		if (token->enabled())
		{
			switch(current_state)
			{
				case HandlerStruct::CurrentState::Off:
					if(timer.getPassedTime() >= off_durration)
					{
						gpio->activate();
						current_state = HandlerStruct::CurrentState::On;
						timer.restart();
					}
					break;
				case HandlerStruct::CurrentState::On:
					if(timer.getPassedTime() >= on_durration)
					{
						gpio->deactivate();
						current_state = HandlerStruct::CurrentState::Off;
						timer.restart();
					}
					break;
			}
		}
	}
}
