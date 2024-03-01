//
// Created by adepe on 01-03-24.
//
#ifndef HEADERS
#include "headers.h"
#define HEADERS
#endif
volatile float* positionReceived= (float*) malloc(sizeof(float)*3);
volatile pthread_mutex_t lockPosition;