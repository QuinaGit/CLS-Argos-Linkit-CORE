#pragma once

#include <functional>
#include <cstdint>
#include "sensor.hpp"
#include "nrf_irq.hpp"
#include "timer.hpp"
#include "messages.hpp"
#include "bma400.h"

extern "C" {
#include "bma400_defs.h"
}

extern Timer *system_timer;

/*! Read write length varies based on user requirement */
#define BMA400_READ_WRITE_LENGTH  		UINT8_C(64)
#define BMA400_PREDICT_READ_INTERVAL_MS	UINT8_C(50)
#define BMA400_AXL_BUF_MAX 				UINT16_C(64)

class BMA400LL
{
public:
	unsigned int m_bus;
	unsigned char m_addr;
	BMA400LL(unsigned int bus, unsigned char addr, int wakeup_pin);
	~BMA400LL();
	double read_temperature();
	void read_xyz(double& x, double& y, double& z);
	void dump_x_axis(uint16_t *buffer);
	void predict_movement(uint8_t& last_predict);
	void set_wakeup_threshold(double threshold);
	void set_wakeup_duration(double duration);
	void enable_wakeup(std::function<void()> func);
	void disable_wakeup();
	bool check_and_clear_wakeup();

	void read_64_x_samples(unsigned int interval_ms, uint16_t *buffer) {
		InterruptLock lock;
		system_timer->cancel_schedule(m_timer_task);
		m_read_interval = interval_ms;
		m_is_reading = true;
		m_read_count = 0;
		p_buffer = buffer;
		read_x_timer();
	};


private:
	NrfIRQ m_irq;
	uint8_t m_unique_id;
	uint8_t m_accel_sleep_mode;
	struct bma400_dev m_bma400_dev;
	double m_wakeup_threshold;
	double m_wakeup_slope;
	double m_wakeup_duration;
	bool m_irq_pending;
	struct bma400_sensor_conf m_bma400_sensor_conf;
	struct bma400_device_conf m_bma400_device_conf;
    struct bma400_int_enable m_bma400_int_en; // interrupt to be declared

	void init();
	static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
	static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
	static void delay_us(uint32_t period, void *intf_ptr);
	double convert_g_force(unsigned int g_scale, int16_t axis_value);
    void bma400_check_rslt(const char api_name[], int8_t rslt);

	//prediction sampling at constant interval
	union __attribute__((packed)) {
        uint8_t buffer[6];
        struct {
        	int16_t x;
        	int16_t y;
        	int16_t z;
        };
    } bma400_data;
	bool m_is_reading;
	uint16_t m_read_count;
	unsigned int m_read_interval;// we want it 20Hz -> 50ms
	uint16_t *p_buffer;
	Timer::TimerHandle m_timer_task;//

	void read_x_timer() {
		InterruptLock lock; //todo: should this take priority?
		if (!m_is_reading)
			return;

		// get and save data
		bma400_get_regs(BMA400_REG_ACCEL_DATA, (uint8_t *)(p_buffer+m_read_count), 2, &m_bma400_dev);  //todo: check
		// printf should not be in an interruptLock !! //replace with mem-log or buffer
        // printf("xyz=%ul,%ul,%ul", bma400_data.x, bma400_data.y, bma400_data.z); //todo
		printf("p_buffer=%p, p_buffer[m_read_count]=%ul",p_buffer,p_buffer[m_read_count]);

		// buffer[m_read_count] = bma400_data.x; //check m_read_count to be safe

		m_read_count++;
		if (m_read_count < BMA400_AXL_BUF_MAX) {
			//reschedule
			m_timer_task = system_timer->add_schedule([this]() {
				if (m_is_reading)
					read_x_timer();
			}, system_timer->get_counter() + m_read_interval);
		}
	};
};


class BMA400 : public Sensor {
public:
	BMA400();
	void calibration_write(const double, const unsigned int) override;
	double read(unsigned int offset) override;
	void install_event_handler(unsigned int, std::function<void()>) override;
	void remove_event_handler(unsigned int) override;

private:
	BMA400LL m_bma400;
	double m_last_x;
	double m_last_y;
	double m_last_z;
	uint8_t m_last_predict;
	uint16_t m_axis_buffer[BMA400_AXL_BUF_MAX];
};
