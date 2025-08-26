#include "IoHwAb_LedMatrix.h"

LedControl lc(DIN_PIN, SCLK_PIN, CS_PIN, 1);

void IoHwAb_LedMatrix_Init()
{
    lc.shutdown(0, false); // bật hiển thị
    lc.setIntensity(0, 1); // 0..15
    lc.clearDisplay(0);
}

void IoHwAb_LedMatrix_Display(const uint8_t bmp[8])
{
    lc.clearDisplay(0);

    for (uint8_t row = 0; row < 8; row++)
    {
        lc.setRow(0, row, bmp[row]);
    }
}