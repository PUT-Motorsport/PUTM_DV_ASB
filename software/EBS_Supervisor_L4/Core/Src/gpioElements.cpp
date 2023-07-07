/*
 * gpio elements.cpp
 *
 *  Created on: 6 maj 2022
 *      Author: Piotr Lesicki
 */

#include <gpioElements.hpp>

GpioElement::GpioElement(const GPIO_TypeDef * const port, const uint32_t pin, const bool is_inverted) :
						 port(port), pin(pin), is_inverted(is_inverted) { }

GpioOutElement::GpioOutElement(const GPIO_TypeDef *port, const uint32_t pin) :
							   GpioElement(port, pin, false) { }

GpioOutElement::GpioOutElement(const GPIO_TypeDef *port, const uint32_t pin, const bool is_inverted) :
							   GpioElement(port, pin, is_inverted) { }

void GpioOutElement::activate()
{
	HAL_GPIO_WritePin((GPIO_TypeDef*)(this->port), this->pin, (this->is_inverted ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

void GpioOutElement::deactivate()
{
	HAL_GPIO_WritePin((GPIO_TypeDef*)(this->port), this->pin, (this->is_inverted ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void GpioOutElement::toggle()
{
	HAL_GPIO_TogglePin((GPIO_TypeDef*)(this->port), this->pin);
}

GpioInElement::GpioInElement(const GPIO_TypeDef * const port, const uint32_t pin) :
							 GpioElement(port, pin, false) { }

GpioInElement::GpioInElement(const GPIO_TypeDef * const port, const uint32_t pin, const bool is_inverted) :
							 GpioElement(port, pin, is_inverted) { }

bool GpioInElement::isActive()
{
	/*
	 * 			truth table
	 *
	 *		port | is_inv | out
	 * 	   ======|========|======
	 *		  0  |    0   |  0
	 *		  1  |    0   |  1
	 *		  0  |    1   |  1
	 *		  1  |    1   |  0
	 *
	 *	  basically a XOR operation
	 *
	 */
	return ((HAL_GPIO_ReadPin((GPIO_TypeDef*)(this->port), this->pin) == GPIO_PIN_SET) != this->is_inverted);
}




