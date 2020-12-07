#ifndef __LOGGER_HPP_
#define __LOGGER_HPP_

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include "messages.hpp"


enum LogLevel {
	LOG_LEVEL_OFF,
	LOG_LEVEL_ERROR,
	LOG_LEVEL_WARN,
	LOG_LEVEL_INFO,
	LOG_LEVEL_DEBUG
};


class Logger {

private:
	int m_log_level = LOG_LEVEL_DEBUG;

public:
	virtual ~Logger() {}

	void set_log_level(int level) {
		m_log_level = level;
	}
	void warn(const char *msg, ...) {
		if (m_log_level >= LOG_LEVEL_WARN) {
			LogEntry buffer;
			va_list args;
			va_start(args, msg);
			vsnprintf(reinterpret_cast<char*>(buffer.data), sizeof(buffer.data), msg, args);
			va_end(args);
			buffer.header.log_type = LOG_WARN;
			// TODO: set date time in header
			write(&buffer);
		}
	}
	void error(const char *msg, ...) {
		if (m_log_level >= LOG_LEVEL_ERROR) {
			LogEntry buffer;
			va_list args;
			va_start(args, msg);
			vsnprintf(reinterpret_cast<char*>(buffer.data), sizeof(buffer.data), msg, args);
			va_end(args);
			buffer.header.log_type = LOG_ERROR;
			// TODO: set date time in header
			write(&buffer);
		}
	}
	void info(const char *msg, ...) {
		if (m_log_level >= LOG_LEVEL_INFO) {
			LogEntry buffer;
			va_list args;
			va_start(args, msg);
			vsnprintf(reinterpret_cast<char*>(buffer.data), sizeof(buffer.data), msg, args);
			va_end(args);
			buffer.header.log_type = LOG_INFO;
			// TODO: set date time in header
			write(&buffer);
		}
	}
	void trace(const char *msg, ...) {
		if (m_log_level >= LOG_LEVEL_INFO) {
			LogEntry buffer;
			va_list args;
			va_start(args, msg);
			vsnprintf(reinterpret_cast<char*>(buffer.data), sizeof(buffer.data), msg, args);
			va_end(args);
			buffer.header.log_type = LOG_TRACE;
			// TODO: set date time in header
			write(&buffer);
		}
	}
	virtual void create() = 0;
	virtual void write(void *) = 0;
	virtual void read(void *, int index = 0) = 0;
	virtual unsigned int num_entries() = 0;
};

#endif // __LOGGER_HPP_
