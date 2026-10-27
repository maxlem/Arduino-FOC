#pragma once

#include "Arduino.h"

void _configureSAMD51VoltageSense(int pinA, int pinB, int pinC);
void _serviceSAMD51Sensing();
float _readSAMD51VoltageSense(const int pin);
