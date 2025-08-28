#pragma once
#include <Arduino.h>
#include "Config/Param_Config.h"
#include "Config/Types.h"
#include "IoHwAb_HandMotion.h"

void DataHandler_Service_Init(bool forwardAxisIsX = true);

void DataHandler_Service_CalibrateNeutral(uint16_t samples = 400, uint32_t us_dt = 2000);

bool DataHandler_Service_Update(float dt, ControlIn *u_out);
