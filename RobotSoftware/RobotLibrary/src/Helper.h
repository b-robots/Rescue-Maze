/*
In this file are all helper functions and helper types (ReturnState, fastSin, fastCos...)
*/

#pragma once

#include <stdint.h>

enum class ReturnState : uint8_t
{
	fatalError,
	error,
	aborted,
	ok
};