/*
 * queue.h
 *
 * Created: 19.11.2020 18:22:42
 *  Author: Andy
 */ 


#ifndef QUEUE_H_
#define QUEUE_H_

#include <stddef.h>
#include "helperFunctions.h"


typedef struct
{
	uint8_t capacity;
	uint8_t size;
	uint8_t front;
	uint8_t rear;
	int32_t* queueArray;
	int32_t* sortedArray;
}Queue;


void initQueue(Queue* queue, int32_t _queueArray[], uint8_t capacity, int32_t _sortedArray[] = NULL);
uint8_t queueIsFull(Queue* queuePtr);
uint8_t queueIsEmpty(Queue* queuePtr);
uint8_t queueUsed(Queue* queuePtr);
int32_t dequeue(Queue* queuePtr);
void enqeue(int32_t newItem, Queue *queuePtr);
int32_t getMedian(Queue* queuePtr);
int32_t getAverage(Queue* queuePtr);
int32_t getNthElement(Queue* queuePtr, uint8_t n);



#endif /* QUEUE_H_ */