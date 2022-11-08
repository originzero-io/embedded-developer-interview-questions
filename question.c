//
//  question.c
//  OriginZeroInterviewCase
//
//  Created by Ali Berkay BERBER on 7.11.2022.
//

#include <stdio.h>

#define buttonCount 10 // Maximum button and led number
#define buttonArkThershold 10 // 10ms thershold

static void delayMs(uint32_t time);
uint8_t buttonRead(uint8_t buttonNo) ;
void buttonInit(void);
void ledInit(void);
void ledToggle(uint8_t ledNumber);
uint8_t buttonReadTime(uint8_t buttonNumber, uint32_t time);  // Must be prototyped like this. "uint8_t button_read_time(uint32_t time)" is false
uint8_t sendAck(void);

// For created to count elapsed time
static int32_t counter = 0;


int main(void)  // must be int instead of void "void main(void)" is false
{
    // Initial settings of the inputs are made
    buttonInit();
    
    // Initial settings of the outputs are made
    ledInit();
    
    // To be able to control which button it is
    uint8_t buttonNo = 0;
    
    while(1)
    {
        // Is the correct button pressed for 1 second?
        if(buttonReadTime(buttonNo, 1000) == 1)
        {
            // toggle the correct led
            ledToggle(buttonNo);
            
            // switch to next led
            buttonNo  = (buttonNo < buttonCount) ? buttonNo + 1 : 0; // If we increase it by one at first, the led with 1 more than the number of buttons will light up.

            
        }
        
        
        // 100 millisecond delay
        delayMs(100);  // Func name is false. "delay(100)" is false;
        
        // send acknowledge about buton state
        sendAck();
    }
}

/*
 * @brief Internal interrupt runs periodically
 * @param[in] NONE
 * @return NONE
 * @details This function is Internal interrupt Period 1ms
 */
void ISR(void) //
{
    counter++;
}

/*
 * @brief One millisecond delay
 * @param[in] time Desired delay time in milliseconds
 * @return NONE
 * @details This function is used for one millisecond delay
 */
static void delayMs(uint32_t time)
{
    uint32_t tickStart = counter;
    
    while((counter - tickStart) <= time);
}

/*
 * @brief Reads state of specified button
 * @param[in] buttonNo Desired led number
 * @return Button state
 * @details Reads the given button number. If it is pressed, it returns 1, otherwise it returns 0
 */
uint8_t buttonRead(uint8_t buttonNo)  // func have to return value and read button
{
    if(buttonNo == 0) return 1;
    else return 0;
}

/*
 * @brief Button initialize
 * @param[in] NONE
 * @return NONE
 * @details Button initial settings. Buttons to be read are introduced
 */
void buttonInit(void)
{
    
}

/*
 * @brief Led initialize
 * @param[in] NONE
 * @return NONE
 * @details Led initial settings. Led to be read are introduced
 */
void ledInit(void)
{
    
}

/*
 * @brief Toogle led
 * @param[in] ledNo
 * @return NONE
 * @details Toggle the desired led
 */
void ledToggle(uint8_t ledNumber) //  void led_toggle(uint8_t led_no);
{
    
}

/*
 * @brief Detects the button hold time
 * @param[in] buttonNumber Desired
 * @param[in] time Desired threshold value
 * @return 1 or 0 Depends on how long it is pressed
 * @details If the button is pressed for the desired threshold time, it return 1, if it is not pressed, it return 0.
 */
uint8_t buttonReadTime(uint8_t buttonNumber, uint32_t time)
{
    uint32_t tickStart = counter;
    while(buttonRead(buttonNumber) == 1)
    {
        if((counter - tickStart) >= time)
        {
            return 1;
        }
    }
    
    return 0;
}

/*
 * @brief Send acknowledge
 * @param[in] NONE
 * @return 1 or 0 depend on accuracy
 * @details Confirms the button status
 */
uint8_t sendAck(void)
{
    volatile uint8_t buttonNo = '\0';
    uint32_t tickStart = counter;
    while(buttonRead(buttonNo) == 0)
    {
        if((counter - tickStart) >= buttonArkThershold)
        {
            return 1;
        }
    }
    
    return 0;
}


