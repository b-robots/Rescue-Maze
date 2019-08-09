/*
Int hsi file are all helper functions and helper types (ReturnState, fastSin, fastCos...)
*/

#pragma once

enum struct ReturnState : uint8_t
{
	fatalError,
	error,
	aborted,
	ok
};