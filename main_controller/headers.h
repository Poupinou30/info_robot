#include <stdio.h>
#include <pigpio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <pthread.h>
#define _Pi 3.1415927
#ifndef MAIN_CONTROLLER_HEADERS_H
#define MAIN_CONTROLLER_HEADERS_H

#endif //MAIN_CONTROLLER_HEADERS_H
#define VERBOSE 1
#define timeDelay 100 //ms


extern volatile float* positionReceived;
extern volatile pthread_mutex_t lockPosition;

void createArray(int16_t num1, int16_t num2, uint8_t* output);
double degToRad(double deg);
int initializeSPI(int channel);
void test_pattern(int spi_handle);
void close_spi(int spi_handle);
void SPI_send(uint8_t send_data[4],int spi_handle, uint8_t *received_data);
void print_data(uint8_t *received_data);
void convertsVelocity(double v_x, double v_y, double omega, double* output_speed);
void myController(double* speedTab, double currentX, double currentY, double currentTheta,double* absoluteSpeedTab, double* targetPosition);
double randomDouble(double min, double max);
void processInstruction(float v_x, float v_y, float omega, double* speedTab, int spi_handle_rear,int spi_handle_front,uint8_t *dataFront, uint8_t *dataRear);
void* executeProgram(void* arg);

void extractBytes(uint16_t nombre, uint8_t *octet_haut, uint8_t *octet_bas) ;
void tunePID(int spi_handle_front,int spi_handle_rear, uint16_t Kp_m, int8_t Kp_e,uint16_t Ki_m, int8_t Ki_e);
void receptionPipe(void* pipefdvoid, void* buffervoid);
