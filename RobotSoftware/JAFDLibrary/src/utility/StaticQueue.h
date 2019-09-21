/*
In this file is the Array-based queue
*/

#pragma once

#include "../implementation/ReturnCode_public.h"
#include <stdint.h>

namespace JAFD
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
		StaticQueue()
		{
			front = 0;
			rear = -1;
		}

		// Remove front element from the queue
		ReturnCode dequeue(T* element)
		{
			if (count == 0)
			{
				return ReturnCode::error;
			}

			*element = arr[front];

			front = (front + 1) % maxSize;
			count--;

			return ReturnCode::ok;
		}

		// Add an item to the queue
		ReturnCode enqueue(T item)
		{
			if (count == maxSize)
			{
				return ReturnCode::error;
			}

			rear = (rear + 1) % maxSize;
			arr[rear] = item;
			count++;

			return ReturnCode::ok;
		}

		// Return the size
		uint8_t size()
		{
			return count;
		}

		// Check if queue is full
		bool isFull()
		{
			return (count == maxSize);
		}

		// Check if queue is empty
		bool isEmpty()
		{
			return (count == 0);
		}
	};
}