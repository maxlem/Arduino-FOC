#pragma once

#include "Arduino.h"

void _configureSAMD51VoltageSense(int pinA, int pinB, int pinC);
void _configureSAMD51VMSense(int pin);
void _serviceSAMD51Sensing();
float _readSAMD51VoltageSense(const int pin);
float _readSAMD51VMSense(const int pin);
uint64_t _getSAMD51SensingTimestamp(const int pin);
