#include "main.h"
#include "BLDCMotor.h"
#define POWER_VOLTAGE         12.0f
#define POLE_PAIRS            9
#define Clamp(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

float velocityOpenloop(float target_velocity);