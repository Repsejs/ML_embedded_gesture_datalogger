#pragma once

#include "gd32vf103.h"
#define FILTERSIZE 100

typedef struct {
    /***************************** MPU6500 vector *****************************/
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t temp;
    int16_t gyroX; 
    int16_t gyroY; 
    int16_t gyroZ;

    /******************************* Extra data *******************************/
    uint16_t milli;
    uint16_t seconds;
    uint16_t minutes;
    int16_t emg;

    /***************************** Debugging data *****************************/


}dma_vector_t;

dma_vector_t dataBuffer;

int timer_config(uint32_t sample_rate);
void adc_config(void);
void dma_config(void);
//void set_dma_address(dma_vector_t* pBuffer);
void config_clic_irqs (void);
void DMA0_Channel0_IRQHandler(void);
//void init_dma_i2c(dma_vector_t* pBuff);