/**
	- Find and edit the logic errors in the code below, report the errors and their solutions.
	- Make sure the whole program runs with as little latency as possible.
	- Make it more readable modular.
	- Enter a description line in the fields you deem necessary.
**/

#include <stdint.h>

#define BUTTON_COUNT 10

static void delay_ms(uint32_t time);
uint8_t button_read(uint8_t button_no);
void button_init(void);
void led_init(void);
void led_toggle(uint8_t led_no);
uint8_t button_read_time(uint32_t time);
uint8_t send_ack(void);


static int32_t counter = 0;

void main(void)
{
	
	button_init();
	led_init();
	
	uint8_t button_no = 0;
	
	while(1)
	{
		if(button_read_time(button_no, 1000) == 1)
		{	
			button_no  = (button_no < BUTTON_COUNT) ? button_no + 1 : 0;

			led_toggle(button_no);
		}
		
		
		
		delay(100);
		send_ack();
	}
}


void ISR(void) //Internal interrupt Period 1ms
{
	counter++;
}

static void delay_ms(uint32_t time)
{
	uint32_t tickstart = counter;
	
	while((counter - tickstart) <= time)
	{
	}
}

uint8_t button_read(uint8_t button_no)
{
}

void button_init(void)
{
}

void led_init(void)
{
}

void led_toggle(uint8_t led_no);
{
}

uint8_t button_read_time(uint8_t button_no, uint32_t time)
{
	uint32_t tickstart = counter;
	while(button_read(button_no) == 1)
	{
		if((counter - tickstart) >= time)
		{
			return 1;
		}
	}
	
	return 0;
}


uint8_t send_ack(void)
{	
}