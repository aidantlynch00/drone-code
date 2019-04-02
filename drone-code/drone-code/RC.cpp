#include "RC.h"
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string>
#include <cmath>
#include <cstring>
#include <string>
#include "GPIO.h"

#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  4		//RUDDER
#define RC_CH2_INPUT  17	//AILERON
#define RC_CH3_INPUT  27	//ELEVATIONATOR
#define RC_CH4_INPUT  22	//THROTHRLE

using namespace std;

uint32_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint32_t rc_shared[RC_NUM_CHANNELS];

void calc_input(int channel, int pin)
{
    if (digitalRead(pin) == HIGH) 
    {
        rc_start[channel] = micros();
    } 
    else 
    {
        uint32_t rc_compare = (uint32_t)(micros() - rc_start[channel]);
        rc_shared[channel] = rc_compare;
    }
}

void calc_ch1()
{
    calc_input(RC_CH1, RC_CH1_INPUT);
}

void calc_ch2()
{
    calc_input(RC_CH2, RC_CH2_INPUT);
}

void calc_ch3()
{
    calc_input(RC_CH3, RC_CH3_INPUT);
}

void calc_ch4()
{
    calc_input(RC_CH4, RC_CH4_INPUT);
}

RC::RC()
{
    setup();
}

void RC::setup() {
  //Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  wiringPiISR(RC_CH1_INPUT, INT_EDGE_BOTH, &calc_ch1);       //Creates Interrupt
  wiringPiISR(RC_CH2_INPUT, INT_EDGE_BOTH, &calc_ch2);
  wiringPiISR(RC_CH3_INPUT, INT_EDGE_BOTH, &calc_ch3);
  wiringPiISR(RC_CH4_INPUT, INT_EDGE_BOTH, &calc_ch4);
}

void RC::read() {
	/*
	for (int channel = RC_CH1; channel < RC_CH4 + 1; channel++)
	{
		rc_values[channel] = rc_shared[channel];
	}*/
	
	memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
}

uint32_t* RC::getValues() {
	read();
	return rc_values;
}


