/*
In this file is the Array-based queue
*/

#include "StaticQueue.h"

namespace JAFTD
{
	// Constructor
	template<typename T, uint8_t maxSize>
	StaticQueue<T, maxSize>::StaticQueue()
	{
		front = 0;
		rear = -1;
	}

	// Remove front element from the queue
	template<typename T, uint8_t maxSize>
	ReturnCode StaticQueue<T, maxSize>::dequeue(T* element)
	{
		if (isEmpty())
		{
			return ReturnCode::error;
		}

		*element = arr[front];

		front = (front + 1) % maxSize;
		count--;

		return ReturnCode::ok;
	}

	// Add an item to the queue
	template<typename T, uint8_t maxSize>
	ReturnCode StaticQueue<T, maxSize>::enqueue(T item)
	{
		if (isFull())
		{
			return ReturnCode::error;
		}

		rear = (rear + 1) % maxSize;
		arr[rear] = item;
		count++;

		return ReturnCode::ok;
	}

	// Check if the queue is empty or not
	template<typename T, uint8_t maxSize>
	bool StaticQueue<T, maxSize>::isEmpty()
	{
		return (count == 0);
	}

	// Check if the queue is full or not
	template<typename T, uint8_t maxSize>
	bool StaticQueue<T, maxSize>::isFull()
	{
		return (count == maxSize);
	}
}