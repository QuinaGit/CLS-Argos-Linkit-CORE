#include <stdint.h>

#include "nrf_battery_mon.hpp"
#include "nrfx_saadc.h"
#include "bsp.hpp"
#include "error.hpp"
#include "debug.hpp"

// ADC constants
#define ADC_MAX_VALUE (16384)      // 2^14
#define ADC_REFERENCE (0.6f)       // 0.6v internal reference

// LUT steps from 4.2V down to 3.2V in 0.1V steps
#define BATT_LUT_ENTRIES 11
#define BATT_LUT_MIN_V   3200
#define BATT_LUT_MAX_V   4200

static const constexpr uint8_t battery_voltage_lut[][BATT_LUT_ENTRIES] = {
	{ 100, 91, 79, 62, 42, 12, 2, 0, 0, 0, 0 },
	{ 100, 93, 84, 75, 64, 52, 22, 9, 0, 0, 0 },
	{ 100, 94, 83, 59, 50, 33, 15, 6, 0, 0, 0 }
};

static void nrfx_saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
	(void)p_event;
}

NrfBatteryMonitor::NrfBatteryMonitor(uint8_t adc_channel, BatteryChemistry chem)
{
	// One-time initialise the driver (assumes we are the only instance)
    nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
    nrfx_saadc_calibrate_offset();

    DEBUG_TRACE("Enter ADC calibration...");
    // Wait for calibrate offset done event
    while (nrfx_saadc_is_busy()) // Wait for calibration to complete
    {}
    DEBUG_TRACE("ADC calibration complete..."); // NOTE: Calibration is retained until power reset. Init/uninit does not clear it

	nrfx_saadc_uninit();

    m_adc_channel = adc_channel;
    m_is_init = false;
    m_chem = chem;
}

float NrfBatteryMonitor::sample_adc()
{
    nrf_saadc_value_t raw = 0;

	// We need to init and uninit the SAADC peripheral here to reduce our sleep current
	
	nrfx_saadc_init(&BSP::ADC_Inits.config, nrfx_saadc_event_handler);
	nrfx_saadc_channel_init(m_adc_channel, &BSP::ADC_Inits.channel_config[m_adc_channel]);

    nrfx_saadc_sample_convert(m_adc_channel, &raw);
	
	nrfx_saadc_uninit();

    return ((float) raw) / ((ADC_GAIN / ADC_REFERENCE) * ADC_MAX_VALUE) * 1000.0f;
}

uint8_t NrfBatteryMonitor::get_level()
{
	uint16_t mV = get_voltage();
	// Convert to value between 0..100 based on LUT conversion
	int lut_index = (BATT_LUT_ENTRIES - 1) - ((mV / 100) - (BATT_LUT_MIN_V / 100));
	DEBUG_TRACE("NrfBatteryMonitor::get_level: mV = %u lut_index=%d", mV, lut_index);

	if (lut_index <= 0) {
		return battery_voltage_lut[(unsigned)m_chem][0];
	} else if (lut_index > (BATT_LUT_ENTRIES - 1)) {
		return battery_voltage_lut[(unsigned)m_chem][BATT_LUT_ENTRIES - 1];
	} else {
		// Linear extrapolation
		uint8_t upper = battery_voltage_lut[(unsigned)m_chem][lut_index-1];
		uint8_t lower = battery_voltage_lut[(unsigned)m_chem][lut_index];
		uint16_t upper_mV = BATT_LUT_MAX_V - ((lut_index-1)*100);
		float t = (upper_mV - mV) / 100.0f;
		float result = (float)upper + (t * ((float)lower - (float)upper));
		DEBUG_TRACE("NrfBatteryMonitor::get_level: upper_mV=%u upper=%u lower=%u t=%f r=%f", upper_mV, upper, lower, (double)t, (double)result);
		return (uint8_t)result;
	}
}

uint16_t NrfBatteryMonitor::get_voltage()
{
#ifdef BATTERY_NOT_FITTED
	return BATT_LUT_MAX_V;
#else
	float adc = sample_adc();
	return (uint16_t)(adc * RP506_ADC_GAIN);
#endif
}
