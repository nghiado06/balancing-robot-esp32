#ifndef IOHWAB_LEDMATRIX_H
#define IOHWAB_LEDMATRIX_H

#include <Arduino.h>
#include "LedControl.h"
#include "Pin_Config.h"

void IoHwAb_LedMatrix_Init();

void IoHwAb_LedMatrix_Display(const uint8_t bmp[8]);

#endif // IOHWAB_LEDMATRIX_H