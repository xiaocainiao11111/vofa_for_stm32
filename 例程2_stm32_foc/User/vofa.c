#include "vofa.h"

// 按printf格式写，最后必须加\r\n
void Vofa_FireWater(const char *format, ...)
{
    uint8_t txBuffer[100];
    uint32_t n;
    va_list args;
    va_start(args, format);
    n = vsnprintf((char *)txBuffer, 100, format, args);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)txBuffer, n);
    va_end(args);
}

// 输入数组地址和个数
void Vofa_JustFloat(float *_data, uint8_t _num)
{
    uint8_t tempData[100];
    uint8_t temp_end[4] = {0, 0, 0x80, 0x7F};
    float temp_copy[_num];

    memcpy(&temp_copy, _data, sizeof(float) * _num);
    memcpy(tempData, (uint8_t *)&temp_copy, sizeof(temp_copy));
    memcpy(&tempData[_num * 4], &temp_end[0], 4);
    HAL_UART_Transmit_DMA(&huart1, tempData, (_num + 1) * 4);
}

/*...........示例..............
    float f1=0.5,f2=114.514;
    Vofa_FireWater("%f,%f\r\n", f1, f2);

    float f3[3]={88.77,0.66,55.44};
    Vofa_JustFloat(f3, 3);
*/
