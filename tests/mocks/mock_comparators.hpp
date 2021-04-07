#pragma once

#include "CppUTestExt/MockSupportPlugin.h"
#include <functional>
#include "gps_scheduler.hpp"

class MockGPSNavSettingsComparator : public MockNamedValueComparator
{
public:
    virtual bool isEqual(const void* object1, const void* object2)
	{
    	return ((GPSNavSettings *)object1)->fix_mode == ((GPSNavSettings *)object2)->fix_mode &&
    			((GPSNavSettings *)object1)->dyn_model == ((GPSNavSettings *)object2)->dyn_model;
	}

    virtual SimpleString valueToString(const void* object)
	{
    	(void)object;
		// The valueToString is called when an error message is printed and it needs to print the actual and expected values
		// It is unclear how this should be implemented
    	return "Unknown";
	}
};

// Compares two "const std::function<void()>" for equality
class MockStdFunctionVoidComparator : public MockNamedValueComparator
{
public:
    virtual bool isEqual(const void* object1, const void* object2)
	{
		typedef void(functionType)();

		auto object1_func_ptr = reinterpret_cast< const std::function<void()>* >(object1);
		auto object2_func_ptr = reinterpret_cast< const std::function<void()>* >(object2);

		// Check function target is of the same type
		if (object1_func_ptr->target_type() != object2_func_ptr->target_type())
			return false;
		
		auto object1_func_target = object1_func_ptr->target<functionType*>();
		auto object2_func_target = object1_func_ptr->target<functionType*>();

		if (object1_func_target != object2_func_target)
			return false;

		return true;
	}

    virtual SimpleString valueToString(const void* object)
	{
		(void) object;
		// The valueToString is called when an error message is printed and it needs to print the actual and expected values
		// It is unclear how this should be implemented
		return "Unknown";
	}
};

// Compares two "const std::function<void(ServiceEvent)>" for equality
class MockStdFunctionServiceEventComparator : public MockNamedValueComparator
{
public:
    virtual bool isEqual(const void* object1, const void* object2)
	{
		typedef void(functionType)();

		auto object1_func_ptr = reinterpret_cast< const std::function<void(ServiceEvent)>* >(object1);
		auto object2_func_ptr = reinterpret_cast< const std::function<void(ServiceEvent)>* >(object2);

		// Check function target is of the same type
		if (object1_func_ptr->target_type() != object2_func_ptr->target_type())
			return false;

		auto object1_func_target = object1_func_ptr->target<functionType*>();
		auto object2_func_target = object1_func_ptr->target<functionType*>();

		if (object1_func_target != object2_func_target)
			return false;

		return true;
	}

    virtual SimpleString valueToString(const void* object)
	{
		(void) object;
		// The valueToString is called when an error message is printed and it needs to print the actual and expected values
		// It is unclear how this should be implemented
		return "Unknown";
	}
};