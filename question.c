/**
	- Find out and edit the logic errors in the code below, report the errors and their solutions.
	- Make sure the whole program runs with as little latency as possible.
	- Design your code modularly.
	- Enter a comment line in the fields you deem necessary.
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
			// butondan elin çekilmesi beklenir
			while (button_read_time(button_no, 1000) != 0) {}

			// butona basıldığında okunan buton indexi bir artar ve arttırılmış indexteki led de yanar 
			// index buton sayısına gelince tekrar 0 olarak başa sarar
			button_no  = (button_no < BUTTON_COUNT) ? button_no + 1 : 0;

			led_toggle(button_no);
		}
		
	}
}


void ISR(void) //Internal interrupt Period 1ms
{
	counter++;

	if (counter % 100  == 0)
	{
		send_ack();
	}
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

	
	while ((counter - tickstart) >= time)
	{
		if (button_read(button_no) == 1)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	
	// timeout
	return 99;
}

uint8_t send_ack(void)
{	
}
