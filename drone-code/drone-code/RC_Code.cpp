#include "RC_Code.h"
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string>
#include <cmath>
#include <cstring>
#include <string>
#include "GPIO.h"

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  7
#define RC_CH2_INPUT  17
#define RC_CH3_INPUT  18
#define RC_CH4_INPUT  19

using namespace std;

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

RC_Code::RC_Code(int pin)
{
	GPIO* pinGPIO = new GPIO(pin, INPUT, PUD_UP);
	this->pin = pin;
    setup();
}
//FIX
void RC_Code::setup(void) {
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

void RC_Code::calc_ch1(void)
{
    calc_input(RC_CH1, RC_CH1_INPUT);
}

void RC_Code::calc_ch2(void)
{
	calc_input(RC_CH2, RC_CH2_INPUT);
}

void RC_Code::calc_ch3(void)
{
	calc_input(RC_CH3, RC_CH3_INPUT);
}

void RC_Code::calc_ch4(void)
{
	calc_input(RC_CH4, RC_CH4_INPUT);
}

void RC_Code::calc_input(int channel, int pin)
{
    if (digitalRead(input_pin) == HIGH) 
    {
        rc_start[channel] = micros();
    } 
    else 
    {
        uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
        rc_shared[channel] = rc_compare;
    }
}

void RC_Code::rc_read_values() {
	//noInterrupts();
	memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
	//interrupts();
}

void RC_Code::read()
{
	rc_read_values();

	cout << "CH1: " cout << (rc_values[RC_CH1]) << cout << "\t" << endl;
}

RC_Code::~RC_Code()
{
}
