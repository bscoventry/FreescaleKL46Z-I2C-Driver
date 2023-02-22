/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "MKL46Z4.h"
#include "slcd.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

// -----------------------------------------------------------------
// Place your macro here - Start
#define PORT_PCR_PS(x)                  (((uint32_t)(((uint32_t)(x))<<PORT_PCR_PS_SHIFT))&PORT_PCR_PS_MASK)
#define PORT_PCR_PE(x)                  (((uint32_t)(((uint32_t)(x))<<PORT_PCR_PE_SHIFT))&PORT_PCR_PE_MASK)

#define PIN(x)							(1<<x)

#define I2C_SCL_PIN						(24)
#define I2C_SDA_PIN						(25)

#define GREEN_LED						(5)
#define RED_LED							(29)
#define SW1								(3)
#define SW2								(12)

#define GREEN_LED_ON()			PTD->PCOR |= PIN(GREEN_LED);
#define GREEN_LED_OFF()			PTD->PSOR |= PIN(GREEN_LED);
#define GREEN_LED_TOGGLE()		PTD->PTOR |= PIN(GREEN_LED);
#define RED_LED_ON()			PTE->PCOR |= PIN(RED_LED);
#define RED_LED_OFF()			PTE->PSOR |= PIN(RED_LED);
#define RED_LED_TOGGLE()		PTE->PTOR |= PIN(RED_LED);
// Place your macro here - End
// -----------------------------------------------------------------

typedef enum {
	STOP,
	MAG_ACQ,
	MAG_CAL,
	ACC_CAL,
	RUN
} enumDigitalCompassOperationState;

enumDigitalCompassOperationState enumDigitalCompassState = STOP;

bool bSW1Pressed = false;
bool bSW3Pressed = false;

bool bIsTimerExpired         = false;

unsigned char    ucSecond = 0;
unsigned char    ucHundredsMilliSecond = 0;
unsigned char    ucMinute = 0;
unsigned short   usTimeElapsed = 0;

unsigned char    ucaryLCDMsg[5] = "";

// --------------------------------------------------------------------
// Place your global variable(s) here - Start
//Globals for calibration
bool firstRead = true;
int16_t maxX = 0;
int16_t minX = 0;
int16_t maxY = 0;
int16_t minY = 0;
int16_t maxZ = 0;
int16_t minZ = 0;

int16_t maxXAccel = 0;
int16_t minXAccel = 0;
int16_t maxYAccel = 0;
int16_t minYAccel = 0;
int16_t maxZAccel = 0;
int16_t minZAccel = 0;

//Global for state machine
bool newStateChange = true;
uint16_t heading = 0;
// Place your global variable(s) here - End
// --------------------------------------------------------------------

void LED_Init(void)
{
// --------------------------------------------------------------------
// Place your LED related code and register settings here - Start

	//Turn on the clock to Port D, and Port E
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	//Set the LED pins to be GPIO pins
	PORTD->PCR[GREEN_LED] = PORT_PCR_MUX(1);
	PORTE->PCR[RED_LED] = PORT_PCR_MUX(1);

	//Set the initial state of the LEDs to be low
	PTD->PCOR |= PIN(GREEN_LED);
	PTE->PCOR |= PIN(RED_LED);

	//Set the LEDs to be outputs
	PTD->PDDR |= PIN(GREEN_LED);
	PTE->PDDR |= PIN(RED_LED);

	GREEN_LED_OFF();
	RED_LED_OFF();

// Place your LED related code and register settings here - End
// --------------------------------------------------------------------
}

void delay (unsigned int uiDelayCycles){
	for (int i = 0; i < uiDelayCycles; ++i);
}

void SWITCH_Init(void)
{
// --------------------------------------------------------------------
// Place your switch related code and register settings here - Start

	//Turn on the clock to Port C
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

	//Enable a pull-up resistor on the switch inputs and designate them as GPIOs
	PORTC->PCR[SW1] = PORT_PCR_MUX(1) | PORT_PCR_PS(1) | PORT_PCR_PE(1);
	PORTC->PCR[SW2] = PORT_PCR_MUX(1) | PORT_PCR_PS(1) | PORT_PCR_PE(1);

	//Set the switch pin direction to input
	PTC->PDDR &= ~PIN(SW1);
	PTC->PDDR &= ~PIN(SW2);

	//Need to setup the switch interrupt to trigger on the falling edge!
	PORTC->PCR[SW1]  |= PORT_PCR_IRQC(0xA);
	PORTC->PCR[SW2]  |= PORT_PCR_IRQC(0xA);

// Place your switch related code and register settings here - End
// --------------------------------------------------------------------
}

void TIMER_Init(void)
{
// --------------------------------------------------------------------
// Place your timer related code and register settings here - Start
	//Select the OSCERCLK clock for the TMPSRC
	SIM->SOPT2 |= 2<<SIM_SOPT2_TPMSRC_SHIFT;
	//Select CLK_IN0 for the external clock
	SIM->SOPT4 |= SIM_SOPT4_TPM0CLKSEL_MASK;
	//Turn on the clock to TMP0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	//Enable counting during debug mode
	TPM0->CONF |= TPM_CONF_DBGMODE(3);
	//Set the TPM0 prescaler to 128
	TPM0->SC |= TPM_SC_PS(7);
	//Set the MOD to be 6,250 in order to increment once every 100 ms.
	TPM0->MOD = 6250;
	//Enable the overflow interrupt
	TPM0->SC |= (1<<TPM_SC_TOIE_SHIFT);
	//Enable the timer.
	TPM0->SC |=	(0x1<<3);

// Place your timer related code and register settings here - End
// --------------------------------------------------------------------
}

void TPM0_IRQHandler(void)
{
// --------------------------------------------------------------------
// Place your timer ISR related code and register settings here - Start
	if(TPM0->SC & TPM_SC_TOF_MASK)
	{
		//Update the bIsTimerExpired bool
		bIsTimerExpired = true;
		//Clear the interrupt
		TPM0->SC |= TPM_SC_TOF_MASK;
	}

// Place your timer ISR related code and register settings here - End
// --------------------------------------------------------------------
}

void PORTC_PORTD_IRQHandler(void)
{
// --------------------------------------------------------------------
// Place your port ISR related code and register settings here - Start

	if (PORTC->ISFR & PIN(SW2))
	{
		bSW3Pressed = true;
		PORTC->PCR[SW2]  |= PORT_PCR_ISF_MASK;
	}

	if (PORTC->ISFR & PIN(SW1))
	{
		bSW1Pressed = true;
		PORTC->PCR[SW1]  |= PORT_PCR_ISF_MASK;
	}

// Place your port ISR related code and register settings here - End
// --------------------------------------------------------------------
}

void I2C_Init()
{
	//Enable clock gate to port
	SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;

	//May need to enable the pins...
	PORTE->PCR[I2C_SCL_PIN] = PORT_PCR_MUX(5);// | PORT_PCR_PS(1) | PORT_PCR_PE(1);
	PORTE->PCR[I2C_SDA_PIN] = PORT_PCR_MUX(5);// | PORT_PCR_PS(1) | PORT_PCR_PE(1);

	//set up frequency divider register. This sets hold times as:
	//SDA = 3.5us
	//SCL Start = 3.00us
	//SCL Stop = 5.5us
	I2C0->F |= (0x2<<6);
	//set up config register to enable IICEN, and IICIE
	I2C0->C1 |= (0x1<<7) | (0x1<<6);
}

uint8_t ResetI2CBus()
{
	//Map clock pin as GPIO with enabled pull ups
	PORTE->PCR[I2C_SCL_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PS(1) | PORT_PCR_PE(1);
	PORTE->PCR[I2C_SDA_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PS(1) | PORT_PCR_PE(1);

	//Set I2C pins as inputs
	PTE->PDDR &= ~PIN(I2C_SCL_PIN);
	PTE->PDDR &= ~PIN(I2C_SDA_PIN);

	if ((PTE->PDIR & PIN(I2C_SDA_PIN)) == 0)
	{
		//The SDA line is stuck low, release it!
		//First, set the SCL to be an output
		PTE->PDDR |= PIN(I2C_SCL_PIN);

		//Now, generate clocks until SDA goes high
		while ((PTE->PDIR & PIN(I2C_SDA_PIN)) == 0)
		{
			//Drive the clock high
			PTE->PCOR |= PIN(I2C_SCL_PIN);
			delay(50);
			//Drive the clock low
			PTE->PSOR |= PIN(I2C_SCL_PIN);
			delay(50);
			//Drive the clock high
			PTE->PCOR |= PIN(I2C_SCL_PIN);
			delay(50);
		}

		//Set the clock back to an input
		PTE->PDDR &= ~PIN(I2C_SCL_PIN);
		//clear the pin settings for I2C0
		PORTE->PCR[I2C_SCL_PIN] = 0x0;
		PORTE->PCR[I2C_SDA_PIN] = 0x0;
	}
	else
	{
		//clear the pin settings for I2C0
		PORTE->PCR[I2C_SCL_PIN] = 0x0;
		PORTE->PCR[I2C_SDA_PIN] = 0x0;
		//The SDA line is NOT being driven low, so the I2C bus is free.
		return 0;
	}
}

uint8_t I2C_SingleByteRead(uint8_t deviceAddress, uint8_t registerAddress)
{
	//Setup configure register to TX
	I2C0->C1 |= (0x1<<4);
	//Send the Start Byte
	I2C0->C1 |= (0x1<<5);
	//Send Address Byte with Write. Address = 0x0E. R/W bit = 0x0
	I2C0->D = ((deviceAddress<<1) | 0x0);

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}

	//Check for ack
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Send Register address data
	I2C0->D = registerAddress;

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}
	//Check for ACK
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Generate SR
	I2C0->C1 |= (0x1<<2);
	//Send Device address again with Read.
	I2C0->D = ((deviceAddress<<1) | 0x1);

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}

	//Check for ack
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Change to read mode by setting TX=0
	I2C0->C1 &= ~(0x1<<4);
	//Generate NACK at the end of this read
	I2C0->C1 |= 0x1<<3;
	//Read Data
	uint8_t data = I2C0->D;
	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}

	//Generate stop bit
	I2C0->C1 &= ~(0x1<<5);
	//Delay to make sure that the stop bit stays low long enough
    delay(50);

	//Read data register
	data = I2C0->D;
	return data;
}

uint8_t I2C_MultipleByteRead(uint8_t deviceAddress, uint8_t *data, uint8_t registerAddress, uint8_t numberOfBytes)
{
	//Check if numberOfBytes = 1

	//direct copy of I2C_SingleByteRead START

	//Setup configure register to TX
	I2C0->C1 |= (0x1<<4);
	//Send the Start Byte
	I2C0->C1 |= (0x1<<5);
	//Send AddressI2C_MultipleByteRead Byte with Write. Address = 0x0E. R/W bit = 0x0
	I2C0->D = ((deviceAddress<<1) | 0x0);

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}

	//Check for ack
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Send Register address data
	I2C0->D = registerAddress;

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}
	//Check for ACK
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Generate SR
	I2C0->C1 |= (0x1<<2);
	//Send Device address again with Read.
	I2C0->D = ((deviceAddress<<1) | 0x1);

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}

	//Check for ack
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//direct copy of I2C_SingleByteRead END
	//start read loop
	//Change to read mode by setting TX=0
	I2C0->C1 &= ~(0x1<<4);
	//Set ACK (should check to make sure that numberOfBytes is not 1. If so, skip to end...OR check this at first and impliment singleByteRead
	I2C0->C1 &= ~(0x1<<3);
	//Dummy read
	uint8_t tmpData = I2C0->D;
	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}

	//Read all but the last byte
	uint8_t i = 0;
	for (i = 0; i < (numberOfBytes-1); i += 1)
	{
		tmpData = I2C0->D;
		*data = tmpData;
		//Wait for the TCF bit to go low
		while ((I2C0->S & (0x1<<7)) != 0x0){
		}
		//Wait for the transfer to complete
		while ((I2C0->S & (0x1<<7)) == 0x0){
		}
		data += 1;
	}

	//Generate NACK at the end of this read for the last byte
	I2C0->C1 |= 0x1<<3;
	//Read Data
	*data = I2C0->D;
	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}

	//Generate stop bit
	I2C0->C1 &= ~(0x1<<5);
	//Delay to make sure that the stop bit stays low long enough
    delay(50);

	return numberOfBytes;
}

uint8_t I2C_SingleByteWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t dataByte)
{
	//Setup configure register to TX
	I2C0->C1 |= (0x1<<4);
	//Send the Start Byte
	I2C0->C1 |= (0x1<<5);
	//Send Address Byte with Write. Address = 0x0E. R/W bit = 0x0
	I2C0->D = ((deviceAddress<<1) | 0x0);

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}

	//Check for ack
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Send Register address data
	I2C0->D = registerAddress;

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}
	//Check for ACK
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Send Data
	I2C0->D = dataByte;

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}
	//Check for ACK
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Generate stop bit
	I2C0->C1 &= ~(0x1<<5);
	//Delay to make sure that the stop bit stays low long enough
    delay(50);

    return dataByte;
}

uint8_t I2C_MultipleByteWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t *dataBytes, uint8_t numberOfBytes)
{
	//Setup configure register to TX
	I2C0->C1 |= (0x1<<4);
	//Send the Start Byte
	I2C0->C1 |= (0x1<<5);
	//Send Address Byte with Write. Address = 0x0E. R/W bit = 0x0
	I2C0->D = ((deviceAddress<<1) | 0x0);

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}

	//Check for ack
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Send Register address data
	I2C0->D = registerAddress;

	//Wait for the TCF bit to go low
	while ((I2C0->S & (0x1<<7)) != 0x0){
	}
	//Wait for the transfer to complete
	while ((I2C0->S & (0x1<<7)) == 0x0){
	}
	//Check for ACK
	if ((I2C0->S & 0x1) == 0x1)
	{
		//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
		I2C0->C1 &= ~(0x1<<5);
		return 0x0;
	}

	//Send Data

	for (uint8_t i = 0; i < numberOfBytes; ++i)
	{
		I2C0->D = *dataBytes;

		//Wait for the TCF bit to go low
		while ((I2C0->S & (0x1<<7)) != 0x0){
		}
		//Wait for the transfer to complete
		while ((I2C0->S & (0x1<<7)) == 0x0){
		}
		//Check for ACK
		if ((I2C0->S & 0x1) == 0x1)
		{
			//we didn't get an ack! stop transmission by generating stop signal (write MST=0)
			I2C0->C1 &= ~(0x1<<5);
			return 0x0;
		}
		dataBytes += 1;
	}

	//Generate stop bit
	I2C0->C1 &= ~(0x1<<5);
	//Delay to make sure that the stop bit stays low long enough
    delay(50);

    return numberOfBytes;
}

bool Magnometer_Init()
{
	//Look for the correct device ID
	uint8_t devID = I2C_SingleByteRead(0x0E, 0x07);

	if (devID != 0xc4)
	{
		return false;
	}

	//Enable automatic magnetic sensor resets (CTRL_REG2 = 0x80)
	I2C_SingleByteWrite(0x0E, 0x11, 0x80);

	//Put MAG3110 in active mode 80 Hz ODR with OSR = 1 (CTRL_REG1 = 0x01)
	I2C_SingleByteWrite(0x0E, 0x10,0x01);
	return true;
}

bool Accelerometer_Init()
{
	//Look for the correct device ID
	uint8_t devID = I2C_SingleByteRead(0x1D,0x0D);
	if (devID != 0x1A)
	{
		return false;
	}

	//Use 2g mode
	I2C_SingleByteWrite(0x1D,0x0E,0x00);
	//Set to high-resolution mode (‘SMODS’ field in ’CTRL_REG2’ register)
	I2C_SingleByteWrite(0x1D,0x2B,0x10);
	//Put the accelerometer in Active Mode and use 14-bit full resolution mode with low noise mode activated
	I2C_SingleByteWrite(0x1D,0x2A,0x05);
	return true;
}

void main(void)
{
    int16_t currentX = 0;
    int16_t currentY = 0;
    int16_t currentZ = 0;
    uint8_t data_l = 0;
    uint8_t data_h = 0;
    uint8_t dataWord[2];

    int16_t xHardIronOffset = 0;
	int16_t yHardIronOffset = 0;
	int16_t zHardIronOffset = 0;

	int16_t xAccelOffset = 0;
	int16_t yAccelOffset = 0;
	int16_t zAccelOffset = 0;

	int16_t currentXAccel = 0;
	int16_t currentYAccel = 0;
	int16_t currentZAccel = 0;

	uint8_t accelVals[6];

	int16_t xAccelVals[4] = {0,0,0,0};
	int16_t yAccelVals[4] = {0,0,0,0};
	int16_t zAccelVals[4] = {0,0,0,0};
	uint8_t index;

    char str[4];

    while(1){

      // State transition upon a switch-press
      // Check if SW3 is pressed
        if(bSW3Pressed == true){
          // Clear the flag
            bSW3Pressed = false;
            if(enumDigitalCompassState == STOP){
                enumDigitalCompassState = MAG_ACQ;
                newStateChange = true;
            }else if(enumDigitalCompassState == MAG_ACQ){
                enumDigitalCompassState = MAG_CAL;
                newStateChange = true;
            }else if(enumDigitalCompassState == MAG_CAL){
                enumDigitalCompassState = ACC_CAL;
                newStateChange = true;
            }else if(enumDigitalCompassState == ACC_CAL){
				enumDigitalCompassState = RUN;
				newStateChange = true;
            }else if(enumDigitalCompassState == RUN){
                 //Nothing to be done
            }
        // Check if SW1 is pressed
        }else if(bSW1Pressed == true){
        	// Clear the flag
			bSW1Pressed = false;
			if(enumDigitalCompassState == STOP){
				//Do nothing
			}else if(enumDigitalCompassState == MAG_ACQ){
				//Do nothing
			}else if(enumDigitalCompassState == MAG_CAL){
				//Do nothing
			}else if(enumDigitalCompassState == ACC_CAL){
				//Do nothing
			}else if(enumDigitalCompassState == RUN){
				enumDigitalCompassState = STOP;
				newStateChange = true;
			}
        }
        // Carry out the given tasks defined in the current state
        if(enumDigitalCompassState == STOP){
        	if (newStateChange)
        	{
        		//clear flag
        		newStateChange = false;

        		//Global disable IRQ
        	    __disable_irq();
                //Initialize switches
        	    SWITCH_Init();
            	//Initialize LEDs
        	    LED_Init();
            	//Initialize Timer (IRQ Enabled, IRQ every 100 ms)

        	    TIMER_Init();
        	    // Enable individual interrupt
        	    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
        	    NVIC_EnableIRQ(TPM0_IRQn);
        	    // Enable global interrupt
        	    __enable_irq();

        	    //Initialize the LCD
        	    SLCD_Init();
            	//Sensors not yet initialized
            	//Display "STOP" on the LCD
                SLCD_WriteMsg((unsigned char *)"STOP");
        	}

        }else if(enumDigitalCompassState == MAG_ACQ){
        	if (newStateChange)
        	{
        		//clear flag
        		newStateChange = false;

            	//Initialize the Magnometer
        	    ResetI2CBus();
        	    I2C_Init();
        		Magnometer_Init();
        		//Set flag to ignore old min and max values
        		firstRead = true;
        		//Clear out saved offsets
        		uint8_t clearOffsets[6] = {0,0,0,0,0,0};
				I2C_MultipleByteWrite(0x0E, 0x09,clearOffsets, 6);
            	//Display "MACQ" on the LCD
            	SLCD_WriteMsg((unsigned char *)"MACQ");
        	}

        	data_l = I2C_SingleByteRead(0x0E, 0x00);
			if ((data_l & (0x1<<3)) != 0x0)
			{
				//Read X,Y,Z data
				I2C_MultipleByteRead(0x0E, dataWord,0x01,2);
				currentX =  dataWord[0];
				currentX = currentX<<8;
				currentX |= dataWord[1];
				I2C_MultipleByteRead(0x0E, dataWord,0x03,2);
				currentY =  dataWord[0];
				currentY = currentY<<8;
				currentY |= dataWord[1];
				I2C_MultipleByteRead(0x0E, dataWord,0x05,2);
				currentZ =  dataWord[0];
				currentZ = currentZ<<8;
				currentZ |= dataWord[1];

				//Update min values
				if ((currentX < minX) || firstRead)
				{
					minX = currentX;
				}
				if ((currentY < minY) || firstRead)
				{
					minY = currentY;
				}
				if ((currentZ < minZ) || firstRead)
				{
					minZ = currentZ;
				}

				//update max values
				if ((currentX > maxX) || firstRead)
				{
					maxX = currentX;
				}
				if ((currentY > maxY) || firstRead)
				{
					maxY = currentY;
				}
				if ((currentZ > maxZ) || firstRead)
				{
					maxZ = currentZ;
				}
				firstRead = false;
			}

			//consider shoving this delay into the I2C access functions
			delay(1000);

        }else if(enumDigitalCompassState == MAG_CAL){
        	if (newStateChange)
        	{
        		//clear flag
        		newStateChange = false;

            	//Calibrate the magnometer for hard-iron effects
        		//Don't divide by 2 because we have to shift to the left anyway to put it in the registers
        		int16_t xOffset = (maxX+minX);
				int16_t yOffset = (maxY+minY);
				int16_t zOffset = (maxZ+minZ);

				uint8_t allOffsets[6];
				allOffsets[0] = ((xOffset>>8)&0xFF);
				allOffsets[1] = ((xOffset)&0xFF);
				allOffsets[2] = ((yOffset>>8)&0xFF);
				allOffsets[3] = ((yOffset)&0xFF);
				allOffsets[4] = ((zOffset>>8)&0xFF);
				allOffsets[5] = ((zOffset)&0xFF);

				//I2C_MultipleByteWrite(0x0E, 0x09,allOffsets, 6);

				xHardIronOffset = xOffset>>1;
				yHardIronOffset = yOffset>>1;
				zHardIronOffset = zOffset>>1;

            	//Display "MCAL" on the LCD
            	SLCD_WriteMsg((unsigned char *)"MCAL");
        	}

        }else if(enumDigitalCompassState == ACC_CAL){
			if (newStateChange)
			{
				//clear flag
				newStateChange = false;
				bool init = Accelerometer_Init();
				firstRead = true;
				//Display "MCAL" on the LCD
				SLCD_WriteMsg((unsigned char *)"ACAL");

				xAccelVals[0] = 0;
				xAccelVals[1] = 0;
				xAccelVals[2] = 0;
				xAccelVals[3] = 0;
				yAccelVals[0] = 0;
				yAccelVals[1] = 0;
				yAccelVals[2] = 0;
				yAccelVals[3] = 0;
				zAccelVals[0] = 0;
				zAccelVals[1] = 0;
				zAccelVals[2] = 0;
				zAccelVals[3] = 0;
				index = 0;
			}

			data_l = I2C_SingleByteRead(0x1D, 0x00);
			if ((data_l & (0x1<<3)) != 0x0)
			{
				I2C_MultipleByteRead(0x1D, accelVals,0x01,6);
				currentX = accelVals[0];
				currentX = currentX<<8;
				currentX |= accelVals[1];
				currentY = accelVals[2];
				currentY = currentY<<8;
				currentY |= accelVals[3];
				currentZ = accelVals[4];
				currentZ = currentZ<<8;
				currentZ |= accelVals[5];

				//Save new value
				xAccelVals[index] = currentX;
				yAccelVals[index] = currentY;
				zAccelVals[index] = currentZ;

				//increment index
				index += 1;
				if (index >= 4)
				{
					index = 0;
				}

				//Calculate average
				xAccelOffset = (xAccelVals[0]+xAccelVals[1]+xAccelVals[2]+xAccelVals[3])>>2;
				yAccelOffset = (yAccelVals[0]+yAccelVals[1]+yAccelVals[2]+yAccelVals[3])>>2;
				zAccelOffset = (zAccelVals[0]+zAccelVals[1]+zAccelVals[2]+zAccelVals[3])>>2;
			}

			delay(1000);

        }else if(enumDigitalCompassState == RUN){
        	if (newStateChange)
        	{
        		newStateChange = false;
        	}
            // Check if timer is expired
            if(bIsTimerExpired == true){
                // Clear the flag
                bIsTimerExpired = false;
                //Get updated XYZ values from the magnometer
                uint8_t directions[6];
                I2C_MultipleByteRead(0x0E, directions,0x01,6);
                currentX = directions[0];
                currentX = currentX<<8;
                currentX |= directions[1];
                currentY = directions[2];
                currentY = currentY<<8;
                currentY |= directions[3];
                currentZ = directions[4];
                currentZ = currentZ<<8;
                currentZ |= directions[5];

                //Subtract out hard-iron offset
                currentX -= xHardIronOffset;
                currentY -= yHardIronOffset;

				float angle = atan2(currentY,currentX)*180/3.14159;
				if (angle < 0)
				{
					angle += 360;
				}

				if (((angle >= 345) && (angle <= 365)) || ((angle >= 0) && (angle <= 15)))
				{
					GREEN_LED_ON();
				}
				else
				{
					GREEN_LED_OFF();
				}

				I2C_MultipleByteRead(0x1D, accelVals,0x01,6);
				currentXAccel = accelVals[0];
				currentXAccel = currentXAccel<<8;
				currentXAccel |= accelVals[1];
				currentYAccel = accelVals[2];
				currentYAccel = currentYAccel<<8;
				currentYAccel |= accelVals[3];
				currentZAccel = accelVals[4];
				currentZAccel = currentZAccel<<8;
				currentZAccel |= accelVals[5];

				currentXAccel -= xAccelOffset;
				currentYAccel -= yAccelOffset;
				currentZAccel -= zAccelOffset;
				currentZAccel += 4096;

				//If we deviate outside of 0.9*4096 or 1.1*4096
				if ((currentZAccel < 3686) || (currentZAccel > 4506))
				{
					RED_LED_ON();
					SLCD_WriteMsg((unsigned char *)"ERR");
				}
				else
				{
					RED_LED_OFF();
	                //update screen
					int heading_int = (int)angle;
	                //Display heading

	                sprintf(str, "%d", heading_int);
	                SLCD_WriteMsg(str);
				}
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
