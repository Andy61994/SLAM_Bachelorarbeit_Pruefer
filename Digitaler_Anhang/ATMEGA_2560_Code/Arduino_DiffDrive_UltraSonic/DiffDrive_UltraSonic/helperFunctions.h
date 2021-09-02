/*
 * insertionSort.h
 *
 * Created: 24.11.2020 17:47:13
 *  Author: Andy
 */ 


#ifndef HELPERFUNCTIONS_H_
#define HELPERFUNCTIONS_H_

#include <avr/io.h>

void insertionSort(int32_t arr[], uint8_t n);
uint8_t binarySearch(int32_t arr[], uint8_t low, uint8_t high, int32_t key); 
uint8_t deleteElement(int32_t arr[], uint8_t n, int32_t key);
uint8_t insertSorted(int32_t arr[], int8_t n, int32_t key, uint8_t capacity);
int16_t convert(float F);
float map(float value, float r1Start, float r1End, float r2Start, float r2End);
uint8_t readInport(volatile uint8_t* PINx, uint8_t PINxN);

#endif /* HELPERFUNCTIONS_H_*/