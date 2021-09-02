/*
 * queue.cpp
 *
 * Created: 28.05.2021 15:23:33
 *  Author: User
 */ 

#include "queue.h"

void initQueue(Queue* queue, int32_t _queueArray[], uint8_t capacity, int32_t _sortedArray[])
{
	queue->capacity = capacity;
	queue->size = 0;
	queue->front = 0;
	queue->rear = capacity - 1;
	queue->queueArray = &_queueArray[0];
	queue->sortedArray = &_sortedArray[0];
}

uint8_t queueIsFull(Queue* queuePtr)
{
	return (queuePtr->size == queuePtr->capacity);	
}

uint8_t queueIsEmpty(Queue* queuePtr)
{
	return (queuePtr->size == 0);
}

// returns queue usage in percent.
uint8_t queueUsed(Queue* queuePtr)
{
	return (((float )queuePtr->size / queuePtr->capacity) * 100);
}

int32_t dequeue(Queue* queuePtr)
{
	// check on µC not necessary
	/*if(queueIsEmpty(queuePtr))
	{
		return;
	}*/
	int32_t item = queuePtr->queueArray[queuePtr->front];
	queuePtr->front = (queuePtr->front + 1) % queuePtr->capacity;
	queuePtr->size = queuePtr->size - 1;
	
	return item;	
}

void enqeue(int32_t newItem, Queue *queuePtr)
{
	if(queueIsFull(queuePtr))
	{
		//deleteElement(queuePtr->sortedArray, queuePtr->capacity, dequeue(queuePtr));
		dequeue(queuePtr);
	}
	queuePtr->rear = (queuePtr->rear + 1) % queuePtr->capacity;
	queuePtr->queueArray[queuePtr->rear] = newItem;
	//insertSorted(queuePtr->sortedArray, queuePtr->size, newItem, queuePtr->capacity);
	queuePtr->size = queuePtr->size + 1;
}

int32_t getMedian(Queue* queuePtr)
{
	uint8_t median = queuePtr->size / 2;
	int32_t sum = 0;
	
	//return queuePtr->sortedArray[median];
	
	
	for(int i = median - 2; i <= median + 2; i++)
	{
		sum += queuePtr->sortedArray[i];
	}
	
	return (uint16_t) (sum / 5);
}

int32_t getAverage(Queue* queuePtr)
{
	int32_t sum = 0;
	
	//return queuePtr->sortedArray[median];
	
	
	for(int i = 0; i < queuePtr->size; i++)
	{
		sum += queuePtr->queueArray[i];
	}
	
	return (uint16_t) (sum / queuePtr->size);
}

int32_t getNthElement(Queue* queuePtr, uint8_t n)
{
	if (queuePtr->size > n)
	{
		return queuePtr->queueArray[(queuePtr->rear + queuePtr->capacity - n)%queuePtr->capacity];
	}
	else return 0;
}
