/*
In this file is the Array-based queue
*/

#pragma once

#include "Helper.h"

#include <cstdint>

namespace JAFTD
{
	namespace internal
	{
		// Template class for an Array-based queue
		template <typename T, uint8_t maxSize>
		class StaticQueue
		{
		private:
			T arr[maxSize];		// Array to store queue elements
			int8_t front;  		// Front points to front element in the queue
			int8_t rear;   		// Rear points to last element in the queue
			uint8_t count;  	// Current size of the queue

		public:
			// Constructor
			StaticQueue();

			// Remove front element from the queue
			ReturnCode dequeue(T* element);

			// Add an item to the queue
			ReturnCode enqueue(T item);

			// Check if the queue is empty or not
			bool isEmpty();

			// Check if the queue is full or not
			bool isFull();
		};
	}
}