#include <stdint.h>

#include "gpio.hpp"
#include "bsp.hpp"

void GPIOPins::initialise()
{
	for (uint32_t i = 0; i < (uint32_t)BSP::GPIO::GPIO_TOTAL_NUMBER; i++)
	{
		nrf_gpio_cfg(BSP::GPIO_Inits[i].pin_number,
					 BSP::GPIO_Inits[i].dir, BSP::GPIO_Inits[i].input, BSP::GPIO_Inits[i].pull,
				     BSP::GPIO_Inits[i].drive, BSP::GPIO_Inits[i].sense);
	}
}

void GPIOPins::set(uint32_t pin)
{
	nrf_gpio_pin_set(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::clear(uint32_t pin)
{
	nrf_gpio_pin_clear(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::toggle(uint32_t pin)
{
	nrf_gpio_pin_toggle(BSP::GPIO_Inits[pin].pin_number);
}

uint32_t GPIOPins::value(uint32_t pin)
{
	return nrf_gpio_pin_read(BSP::GPIO_Inits[pin].pin_number);
}

void GPIOPins::enable(uint32_t pin)
{
	nrf_gpio_cfg(BSP::GPIO_Inits[pin].pin_number,
				 BSP::GPIO_Inits[pin].dir, BSP::GPIO_Inits[pin].input, BSP::GPIO_Inits[pin].pull,
			     BSP::GPIO_Inits[pin].drive, BSP::GPIO_Inits[pin].sense);
}

void GPIOPins::disable(uint32_t pin)
{
	nrf_gpio_cfg_default(BSP::GPIO_Inits[pin].pin_number);
}
