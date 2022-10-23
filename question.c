/**
	- Find out and edit the logic errors in the code below, report the errors and their solutions.
	- Make sure the whole program runs with as little latency as possible.
	- Design your code modularly.
	- Enter a comment line in the fields you deem necessary.
**/

#include <stdint.h>

#define BUTTON_COUNT 10	// Number of buttons and LEDs
#define ACK_INT_MS 100	// Acknowledge interval in ms

static void delay_ms(uint32_t time);
uint8_t button_read(uint8_t button_no);
void button_init(void);
void led_init(void);
void led_toggle(uint8_t led_no);
uint8_t button_read_time(uint8_t button no,uint32_t time);
uint8_t send_ack(void);

// Used to count time since start
static uint32_t counter = 0;

// Keeps track of last acknowledge time
uint32_t last_ack_tick = 0;

void main(void)
{
	// Set up inputs
	button_init();
	// Set up outputs
	led_init();
	
	// For keeping button being checked
	uint8_t button_no = 0;
	
	while(1)
	{
		// Check if button is pressed for 1 second
		if(button_read_time(button_no, 1000) == 1)
		{	
			// Toggle LED corresponding to button
			led_toggle(button_no);
			
			// update button_no for next button
			button_no  = (button_no < BUTTON_COUNT) ? button_no + 1 : 0;
		}
		
		// Check if you need to acknowledge
		if(counter - last_ack_tick >= ACK_INT_MS) {
			send_ack();
			last_ack_tick = counter;
		}
	}
}


void ISR(void) //Internal interrupt Period 1ms
{
	counter++;
}

/** Waits for given miliseconds
  * @param time miliseconds to wait
*/
static void delay_ms(uint32_t time)
{
	uint32_t tickstart = counter;
	
	while((counter - tickstart) <= time)
	{
	}
}

/** Reads button state 
  * @param led_no LED to toggle
  * @return button state
*/
uint8_t button_read(uint8_t button_no)
{
}

/** Setup buttons
*/
void button_init(void)
{
}

/** Setup LEDs
*/
void led_init(void)
{
}

/** Toggles LED
  * @param led_no LED to toggle
*/
void led_toggle(uint8_t led_no);
{
}

/** Checks if button is pressed longer than given time
  *	@param button_no button to be checked
  * @param time desired time to be pressed
  * @return 1 pressed as long as desired othewise 0
*/
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

/** Sends acknowledment
*/
uint8_t send_ack(void)
{	
}