#include "gd32vf103.h"
#include "systick.h"
#include "dma.h"

#define FILTERSIZE 100
#define I2C0_DATA_ADDRESS 0x40005410

int timer_config(uint32_t sample_rate){
	// Max 2000hz 108
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    /* deinit a timer */
    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 5399; //  108 000 000
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = sample_rate - 1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* CH0 configuration in PWM mode1 */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 1);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

	timer_auto_reload_shadow_enable(TIMER1);
}

void dma_config(void){
    /* DMA_channel configuration */
    /* initialize DMA mode */
    dma_parameter_struct dma_init_struct;
    dma_deinit(DMA0, DMA_CH6);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)&dataBuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = 14;  //fourteen memory adresses for all datapoints 0x3B-0x48
    dma_init_struct.periph_addr = (uint32_t)I2C0_DATA_ADDRESS; //DMA0 adress
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH6, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH6);

    dma_interrupt_enable(DMA0, DMA_CH6, DMA_FLAG_FTF);
    /* enable DMA channel5 */
    dma_channel_enable(DMA0, DMA_CH6);
}

void config_clic_irqs (void){

    eclic_global_interrupt_enable();
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    eclic_irq_enable(DMA0_Channel6_IRQn, 1, 1);

}

//uint8_t* pDataBuffer;

void DMA0_Channel6_IRQHandler(void){
    i2c_stop_on_bus(I2C0);

	if(dma_interrupt_flag_get(DMA0, DMA_CH6, DMA_INT_FLAG_FTF)){ 
		dma_interrupt_flag_clear(DMA0, DMA_CH6, DMA_INT_FLAG_G);  
    }

}
