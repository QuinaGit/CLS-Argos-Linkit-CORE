#pragma once

#include "logger.hpp"
#include "messages.hpp"
#include "sensor_service.hpp"
#include "timeutils.hpp"

// switch between logging accelerometer -- also do it in bma400.h
#define AXL_LOG_TYPE 		2 	//0:single-default, 1:predict, 2:stream-dump
#define AXL_DATADUMP_SIZE 	128

struct __attribute__((packed)) AXLLogEntry {
	union {
		struct {
			LogHeader header;
			union {
				struct {
					double x;
					double y;
					double z;
					double temperature;
					bool   wakeup_triggered;
					uint8_t movement_predict;
				};
				uint8_t data[MAX_LOG_PAYLOAD];
			};
		};
		uint16_t data_dump[AXL_DATADUMP_SIZE]; // For headerless data dump
	};
};

class AXLLogFormatter : public LogFormatter {
public:
	const std::string header() override {
		return "log_datetime,x,y,z,wakeup_triggered,temperature,movement_predict\r\n";
	}
	// I need to use this function called by 
	const std::string log_entry(const LogEntry& e) override {
		char entry[512];
		const AXLLogEntry *log = (const AXLLogEntry *)&e;

#if AXL_LOG_TYPE == 2	// Data Dump
		std::memcpy(entry, log->data_dump, AXL_DATADUMP_SIZE * sizeof(uint16_t));

		return std::string(entry, AXL_DATADUMP_SIZE * sizeof(uint16_t));
#else //Single measurement or prediction
		char d1[128];
		std::time_t t;
		std::tm *tm;

		t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours, log->header.minutes, log->header.seconds);
		tm = std::gmtime(&t);
		std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		// Convert to CSV
		snprintf(entry, sizeof(entry), "%s,%f,%f,%f,%u,%f,%u\r\n",
				d1,
				log->x, log->y, log->z, log->wakeup_triggered, log->temperature,
				log->movement_predict);
		
		return std::string(entry);
#endif
	}
};

enum AXLSensorPort : unsigned int {
	TEMPERATURE,
	X,
	Y,
	Z,
	WAKEUP_TRIGGERED,
	MOVEMENT_PREDICT
};

enum AXLCalibration : unsigned int {
	WAKEUP_THRESH,
	WAKEUP_DURATION
};

enum AXLEvent : unsigned int {
	WAKEUP
};

class AXLSensorService : public SensorService {
public:
	AXLSensorService(Sensor& sensor, Logger *logger = nullptr) : SensorService(sensor, ServiceIdentifier::AXL_SENSOR, "AXL", logger) {}

private:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
	void read_and_populate_log_entry(LogEntry *e) override {
		AXLLogEntry *log = (AXLLogEntry *)e;
#if AXL_LOG_TYPE == 2
		m_sensor.dump_read((void*)log->data_dump); //!todo: this hack might be the temporary solution

		DEBUG_TRACE("AXLSensorService::read_and_populate_log_entry ... log->data_dump=%x",log->data_dump);
		uint16_t i;
		for (i = 0; i < AXL_DATADUMP_SIZE; i++)
		{
			printf("%d:%x\r\n",i,log->data_dump[i]);
		}
#else
  #if AXL_LOG_TYPE == 1
		log->movement_predict = m_sensor.read(AXLSensorPort::MOVEMENT_PREDICT);
  #endif 
		log->x = m_sensor.read(AXLSensorPort::X);
		log->y = m_sensor.read(AXLSensorPort::Y);
		log->z = m_sensor.read(AXLSensorPort::Z);
		log->wakeup_triggered = m_sensor.read(AXLSensorPort::WAKEUP_TRIGGERED);
		log->temperature = m_sensor.read((unsigned int)AXLSensorPort::TEMPERATURE);
		service_set_log_header_time(log->header, service_current_time());
#endif
	}
#pragma GCC diagnostic pop

	void service_init() override {
		// Setup 0.1G threshold and 1 sample duration
		double g_thresh = service_read_param<double>(ParamID::AXL_SENSOR_WAKEUP_THRESH);
		unsigned int duration = service_read_param<unsigned int>(ParamID::AXL_SENSOR_WAKEUP_SAMPLES);
		if (g_thresh && service_is_enabled()) {
			m_sensor.calibration_write(g_thresh, AXLCalibration::WAKEUP_THRESH);
			m_sensor.calibration_write(duration, AXLCalibration::WAKEUP_DURATION);
			m_sensor.install_event_handler(AXLEvent::WAKEUP, [this]() {
				DEBUG_TRACE("AXLSensorService::event");
				LogEntry e;
				ServiceEventData data = true;
				read_and_populate_log_entry(&e);
				service_complete(&data, &e, false);
			});
		}
	};
	void service_term() override {
		m_sensor.remove_event_handler(AXLEvent::WAKEUP);
	};
	bool service_is_enabled() override {
		bool enable = service_read_param<bool>(ParamID::AXL_SENSOR_ENABLE);
		return enable;
	}
	unsigned int service_next_schedule_in_ms() override {
		unsigned int schedule =
				1000 * service_read_param<unsigned int>(ParamID::AXL_SENSOR_PERIODIC);
		return schedule == 0 ? Service::SCHEDULE_DISABLED : schedule;
	}
	void service_initiate() override {
		ServiceEventData data = false;
		LogEntry e;
		read_and_populate_log_entry(&e);
		service_complete(&data, &e);
	}
	bool service_cancel() override { return false; }
	unsigned int service_next_timeout() override { return 0; }
	bool service_is_usable_underwater() override { return true; }
};
