/*
 * helperFunctions.cpp
 *
 * Created: 02.12.2020 15:08:11
 *  Author: Andy
 */ 

#include "helperFunctions.h"
#include <math.h>

/* Function to sort an array using insertion sort*/
void insertionSort(int32_t arr[], uint8_t n) 
{ 
    int32_t i, key, j; 
    for (i = 1; i < n; i++) { 
        key = arr[i]; 
        j = i - 1; 
  
        /* Move elements of arr[0..i-1], that are 
          greater than key, to one position ahead 
          of their current position */
        while (j >= 0 && arr[j] > key) { 
            arr[j + 1] = arr[j]; 
            j = j - 1; 
        } 
        arr[j + 1] = key; 
    } 
} 

uint8_t binarySearch(int32_t arr[], uint8_t low, uint8_t high, int32_t key)
{
	if (high < low)
	return -1;
	uint8_t mid = (low + high) / 2;
	if (key == arr[mid])
	return mid;
	if (key > arr[mid])
	return binarySearch(arr, (mid + 1), high, key);
	return binarySearch(arr, low, (mid - 1), key);
}

uint8_t deleteElement(int32_t arr[], uint8_t n, int32_t key)
{
	// Find position of element to be deleted
	uint8_t pos = binarySearch(arr, 0, n - 1, key);
	
	/*if (pos == -1) {
		printf("Element not found");
		return n;
	}*/
	
	// Deleting element
	uint8_t i;
	for (i = pos; i < n - 1; i++)
	arr[i] = arr[i + 1];
	
	return n - 1;
}

uint8_t insertSorted(int32_t arr[], int8_t n, int32_t key, uint8_t capacity)
{
	// Cannot insert more elements if n is already
	// more than or equal to capcity
	/*if (n >= capacity)
	return n;*/
	
	int8_t i;
	for (i = n - 1; (i >= 0 && arr[i] > key); i--)
	arr[i + 1] = arr[i];
	
	arr[i + 1] = key;
	
	return (n + 1);
}

float map(float value, float r1Start, float r1End, float r2Start, float r2End)
{
	return (int16_t) ((value - r1Start) * (r2End - r2Start) / (r1End - r1Start) + r2Start);
}

int16_t convert(float F)
{
	float f_mapped = map(F, -835, 835, -21, 21);
	f_mapped = (asin(sin(0.175)+(f_mapped/39))-0.175)*39;
	return map(f_mapped, -21, 21, -835, 835);
}

uint8_t readInport(volatile uint8_t* PINx, uint8_t PINxN)
{
	if((*PINx) & (1<<PINxN))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
