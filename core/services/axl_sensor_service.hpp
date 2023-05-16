#pragma once

#include "logger.hpp"
#include "messages.hpp"
#include "sensor_service.hpp"
#include "timeutils.hpp"

struct __attribute__((packed)) AXLLogEntry {
	union { //todo: think about this possibility
		struct {
			LogHeader header;
			union {
				struct {
					double x;
					double y;
					double z;
					double temperature;
					bool   wakeup_triggered;
					uint8_t movement_predict; // todo: maybe use up the whole AXLLogEntry data buffer to log readings (maybe only one axis)
				};
				uint8_t data[MAX_LOG_PAYLOAD]; // log payload is basically 128-9 = 119 bytes
			};
		};
		char data_dump[128*2]; // For header-less data dump
	};
};

class AXLLogFormatter : public LogFormatter {
public:
	const std::string header() override {
		return "log_datetime,x,y,z,wakeup_triggered,temperature,movement_predict\r\n";
	}
	// I need to use this function called by 
	const std::string log_entry(const LogEntry& e) override {
		char entry[512];//, d1[128];
		const AXLLogEntry *log = (const AXLLogEntry *)&e;
		// std::time_t t;
		// std::tm *tm;

		// t = convert_epochtime(log->header.year, log->header.month, log->header.day, log->header.hours, log->header.minutes, log->header.seconds);
		// tm = std::gmtime(&t);
		// std::strftime(d1, sizeof(d1), "%d/%m/%Y %H:%M:%S", tm);

		// // Convert to CSV
		// snprintf(entry, sizeof(entry), "%s,%f,%f,%f,%u,%f,%u\r\n",
		// 		d1,
		// 		log->x, log->y, log->z, log->wakeup_triggered, log->temperature,
		// 		log->movement_predict);
		
		snprintf(entry, sizeof(entry), "%s\r\n",log->data_dump);
		return std::string(entry);
	}
};

enum AXLSensorPort : unsigned int {
	TEMPERATURE,
	X,
	Y,
	Z,
	WAKEUP_TRIGGERED,
	MOVEMENT_PREDICT,
	DUMP_X
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

	// void read_axl_and_populate_log_entry(LogEntry *e) {
	// 	AXLLogEntry *log = (AXLLogEntry *)e;
	// 	log->data_dump = m_sensor.read(AXLSensorPort::DUMP_X);
	// }


	void read_and_populate_log_entry(LogEntry *e) override {
		AXLLogEntry *log = (AXLLogEntry *)e;
		// log->x = m_sensor.read(AXLSensorPort::X);
		// log->y = m_sensor.read(AXLSensorPort::Y);
		// log->z = m_sensor.read(AXLSensorPort::Z);
		// log->wakeup_triggered = m_sensor.read(AXLSensorPort::WAKEUP_TRIGGERED);
		// log->temperature = m_sensor.read((unsigned int)AXLSensorPort::TEMPERATURE);
		// log->movement_predict = m_sensor.read(AXLSensorPort::MOVEMENT_PREDICT);
		// service_set_log_header_time(log->header, service_current_time());
		char test_data[] = "This is a string of 120 bytes long to see if it can be stored in the memory for retrieval of the accelerometer data...";
		printf("%s\r\n",test_data);
		// log->data_dump = "This is a string of 120 bytes long to see if it can be stored in the memory for retrieval of the accelerometer data...\r\n";
		snprintf(log->data_dump, sizeof(log->data_dump), "%s\r\n",test_data);
		printf("%s\r\n",log->data_dump);
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
