#include "gd32vf103.h"
#include "usb_serial_if.h"
#include "usb_delay.h"
#include <stdlib.h>
#include <string.h>
#include "systick.h"
#include "gd32v_tf_card_if.h"
#include "ff.h"
#include "gd32v_mpu6500_if.h"
#include "dma.h"
#include "mpu6500_registers.h"
#include "mpu6500_driver.h"
#include "queue.h"
#include "tf_card.h"

#define BUFFER_SIZE 512
#define DESIRED_FREQUENCY_HZ 50

/* FILENAME should be at most 8 chars long and the extension should be at most 3 */

volatile int16_t last_Sample_buffer[7] = {0};

void ledInit (void);
void led_on_B1();
void led_off_B1();
void led_on_B0();
void led_off_B0();
void timer_interrupt_config();
void init_mpu(void);
int millis(void);
void append_newLine(char *string, int length);
int append_int_to_string(char *string, int integer);
int int_to_string(char *string, int integer);
void formatTime(char *output, int minutes, int seconds, int milliseconds);
         
uint8_t* pDatabuffer = (uint8_t*)&dataBuffer;
uint32_t time_elapsed = 0;

int main(void){
	uint32_t count = 0;
    char usb_data_buffer[512] = {'\0'};
    char usb_read_buf[80] = {'\0'};

    configure_usb_serial();
    usb_delay_1ms(1);

    while (!usb_serial_available()) {
        usb_delay_1ms(100);
    }
	printf("ready\r\n");

    init_q();
    ledInit();
    usb_delay_1ms(1);
    /* Handle for the mounted filesystem */
    FATFS fs;

    /* FatFs return code */
    volatile FRESULT fr;

    /* File handle */
    FIL fil;

    /* Used for bytes written */
    UINT bw = 0;
    
    /* Sets a valid date for when writing to file */
    set_fattime(1980, 1, 1, 0, 0, 0); // 1980 Jan 1st, 00:00:00
    delay_1ms(100);

    /* This function "mounts" the SD-card which makes the filesystem available */
    fr = f_mount(&fs, "test", 1); // Mount storage device
    f_sync(&fil);

    /* This function opens a file. In this case, we are creating a file which we want to write to */
    
    delay_1ms(400);
	printf("mpu\r\n");
    init_mpu();
	printf("mpu2\r\n");

    rcu_periph_clock_enable(RCU_DMA0);

    delay_1ms(20);

    char write_to_sd[2560] = {'\0'};
    char buf[BUFFER_SIZE] = {'\0'};
    
    
    int blink = 0;
    int onOff = 0;
	
	fr = f_mount(&fs, "test", 1); // Mount storage device
	f_sync(&fil);

	fr = f_open(&fil, "TData.CSV", FA_WRITE | FA_CREATE_ALWAYS);
	clear_queues();
	/* Write the header to the file */
	char header[] = "X-acc;Y-acc;Z-acc\n";
	//clear queue
	enqueue_string(header);
	
	//START IMU DMA
	//i2c_dma_master_read_register(I2C0, MPU6500_ADDR, MPU6500_ACCEL_XOUT_H);
	//ENABLE interrupts for data collection
	timer_interrupt_config();
	config_clic_irqs();
	eclic_global_interrupt_enable();  
	printf("running\r\n");
    uint64_t start_time = millis();
    while(time_elapsed < 10000){
		blink++;
		if (blink == 1000000){
			blink = 0;
			if(onOff){
				led_on_B0();
				onOff = 0;
			}else{
				led_off_B0();
				onOff = 1;
			}
		}
		
		if(queue_str_len() > 5){ //Behövs högre hastighet så kan vi öka denna 5:an
			memset(write_to_sd, '\0', sizeof(write_to_sd));
			//printf("x=%d, y=%d, z=%d\r\n", last_Sample_buffer[0],last_Sample_buffer[1],last_Sample_buffer[2]);
			fflush(0);
			if (dequeue_string(write_to_sd, 5)) { //Behövs högre hastighet så kan vi öka denna 5:an
				f_write(&fil, write_to_sd, strlen(write_to_sd), &bw);
				f_sync(&fil);
			}
		}

	}
    uint64_t end_time = millis();
	printf("breaking\r\n");
    printf("finished in %d (s)\r\n", ((end_time - start_time)/1000.0));
	timer_interrupt_disable(TIMER1, TIMER_INT_CH0);
	write_to_sd[0] = '\0';
	while(empty_string_queue(write_to_sd, 5)){
		f_write(&fil, write_to_sd, strlen(write_to_sd), &bw); // testa byt strlen till buf_size för bättre prestanda
		f_sync(&fil);
		write_to_sd[0] = '\0';
	}    

	f_close(&fil);
	printf("done\r\n");
	fflush(0);
}


void TIMER1_IRQHandler(void){
    char buf[64] = {'\0'};
    int buf_size = 0;   
	time_elapsed++;
    dma_vector_t vector_write;
	mpu_vector_t vec;
    /* Check which channel triggered the interrupt */
    if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_CH0)){
        
		mpu6500_getAccel(&vec);
		last_Sample_buffer[0] = (int16_t)vec.x;
		last_Sample_buffer[1] = (int16_t)vec.y;
		last_Sample_buffer[2] = (int16_t)vec.z;
		append_int_to_string(buf, last_Sample_buffer[0]);
        append_int_to_string(buf, last_Sample_buffer[1]);
        append_int_to_string(buf, last_Sample_buffer[2]);

        append_newLine(buf, strlen(buf)); //Testa byt strlen till buf_size för bättre prestanda
        enqueue_string(buf);

        timer_interrupt_flag_clear(TIMER1, TIMER_INT_CH0);
    }
}

void ledInit (void){
    rcu_periph_clock_enable(RCU_GPIOB);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);
}

void led_on_B1(){
    gpio_bit_write(GPIOB, GPIO_PIN_1, 1);
}
void led_off_B1(){
    gpio_bit_write(GPIOB, GPIO_PIN_1, 0);
}

void led_on_B0(){
    gpio_bit_write(GPIOB, GPIO_PIN_0, 1);
}
void led_off_B0(){
    gpio_bit_write(GPIOB, GPIO_PIN_0, 0);
}

void timer_interrupt_config(){
    /* This configuration has a lot in common with the PWM configuration since both relies on a timer. 
       this time though instead of generating a puls each timer period we get an interrupt. This example
       generates an interrupt each millisecond to update a counter and when an even count of 1000 (1s)
       the interrupt will toggle an LED. */

    /* Configuration structs */ 
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    /* Enable the TIMER1 interrupt request */
    eclic_irq_enable(TIMER1_IRQn,1,0);
    
    /* Enable the peripheral clock. */
    rcu_periph_clock_enable(RCU_TIMER1);

    /* Reset the timer */
    timer_deinit(TIMER1);

    /* initialize timer configuration struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    /* Frequency = core clock / ((1+prescaler)*period) = 108MHz / (2*54000) = 1KHz */
    
    uint32_t period = SystemCoreClock / (DESIRED_FREQUENCY_HZ * 2);
    timer_initpara.prescaler         = 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);

    /* Set the channel configuration */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocinitpara);
    
    /* CH0 configuration in OC timing mode */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, SystemCoreClock/4000);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_TIMING);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* Enable interrupt on the channel */
    timer_interrupt_enable(TIMER1, TIMER_INT_CH0);
    /* Make sure interrupt flag is clear */
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_CH0);

    /* Start the timer */
    timer_enable(TIMER1);
}

int int_to_string(char *string, int integer){
    int i = 0;
    int divider = 1000;
    if(integer < 0){
        string[i++] = '-';
        integer = -integer;
    }else if(integer == 0){
        string[i++] = '0';
        string[i++] = ';';
        string[i] = '\0';
        return i;
    }
    int og_num = integer;
    while (integer >= divider * 10) {
        divider *= 10; 
    }

    while(divider){
        char digit = (integer/divider)+ '0';
        if(digit != '0' || og_num > integer){
            string[i++] = digit; 
        }
        
        integer %= divider;
        divider /= 10;
    }
    string[i++] = ';';
    string[i] = '\0';
    return i;
}

int append_int_to_string(char *string, int integer){
    char temp[20] = {'\0'};
    int_to_string(temp, integer);
    int len = fuseStrings(string, temp);
    return len;
}

void append_newLine(char *string, int length) {
    string[length-1] = '\n';
    string[length] = '\0'; // Add a null terminator at the new end
}

int millis(void) {
    uint64_t mtime = get_timer_value();
    return ((mtime * 4000.0) / SystemCoreClock); // Adjusted for seconds and milliseconds
}

void init_mpu(void){
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7); //led?

    mpu6500_install(I2C0);

    delay_1ms(100);   
}