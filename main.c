#include "MKL46Z4.h"
#include <stdbool.h>
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "pin_mux.h"
#include "board.h"
#include "lcd.h"
#include <math.h>

#define MAG3110_I2C_ADDR 0x0E
#define PORTC_D_IRQ_NBR (IRQn_Type) 31 // Define the interrupt number for PORTC and PORTD

void initLed(); // Function declaration for initializing LEDs
void initSW(); // Function declaration for initializing switches
// Function declaration for the interrupt handler
void PORTC_PORTD_IRQHandler(void);
int32_t volatile systemState = 0; // Variable to hold the system state
int32_t system_stop = 0; // Define constant for system stop state
int32_t system_running = 1; // Define constant for system running state

void initLed(){
    // Initialize RED LED on PTE29
    SIM->SCGC5 |= (1<<13); // Enable clock to PORTE
    PORTE->PCR[29] = (1<<8); // Set PTE29 as GPIO
    PTE->PDDR |= (1<<29); // Set PTE29 as output
    
    // Initialize GREEN LED on PTD5
    SIM->SCGC5 |= (1<<12); // Enable clock to PORTD
    PORTD->PCR[5] = (1<<8); // Set PTD5 as GPIO
    PTD->PDDR |= (1<<5); // Set PTD5 as output
}

void initSW(){
    SIM->SCGC5 |= (1<<11); // Enable clock to PORTC

    // Initialize SW1 on PTC3
    PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // Set PTC3 as GPIO with pull-up resistor
    PTC->PDDR &= ~(uint32_t)(1u<<3); // Set PTC3 as input
    PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); // Enable interrupt on falling edge for PTC3
    
    // Initialize SW2 on PTC12
    PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // Set PTC12 as GPIO with pull-up resistor
    PTC->PDDR &= ~(uint32_t)(1u<<12); // Set PTC12 as input
    PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); // Enable interrupt on falling edge for PTC12
    
    NVIC_ClearPendingIRQ(PORTC_D_IRQ_NBR); // Clear any pending interrupts
    NVIC_EnableIRQ(PORTC_D_IRQ_NBR); // Enable interrupts for PORTC and PORTD
}

void PORTC_PORTD_IRQHandler(){
    uint32_t i = 0;
	for(i=0; i<1000; i++);
	if((PTC->PDIR & (1<<3)) == 0){
		if(systemState == system_running){
			systemState = system_stop;
			PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
		}
		else{
			systemState = system_running;
			PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
		}
	}
		
	if ((PTC->PDIR & (1<<12)) == 0) { 
        NVIC_SystemReset();
        PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
    }
}

uint32_t volatile msTicks = 0; // Variable to count milliseconds

void init_Systick_interrupt(){
    SysTick->LOAD = SystemCoreClock/1000; // Configure the SysTick to generate interrupt every 1 ms
    // Select Core Clock, Enable SysTick, Enable Interrupt
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    
}

void SysTick_Handler (void) { // SysTick interrupt handler
    msTicks++; // Increment counter
}

void Delay (uint32_t TICK) {
    while (msTicks < TICK); // Wait until the desired delay is reached
    msTicks = 0; // Reset counter
}

void NVIC_SetPriorityLevels(void) {
    NVIC_SetPriority(SysTick_IRQn, 1); // Set priority for SysTick interrupt
    NVIC_SetPriority(PORTC_PORTD_IRQn, 2); // Set priority for GPIO interrupts
}

void init_i2c(void);
void send_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t value);
void read_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
uint16_t DataProcess (int X,int Y);
volatile int16_t x_max = 0, x_min = 0, y_max = 0,y_min = 0,x_offset = 0,y_offset = 0,x,y,z;
int main(void)
{
	int first = 0;
	initLed(); // Initialize LEDs
  initSW(); // Initialize switches
  init_Systick_interrupt(); // Initialize SysTick interrupt

	init_i2c();
	send_i2c(MAG3110_I2C_ADDR, 0x10, 0x01);
	send_i2c(MAG3110_I2C_ADDR, 0x11, 0x80);

	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD->PCR[1] = PORT_PCR_MUX(1);
	PTD->PDDR &= ~(1 << 1);
	
	LCD_Init();

	int32_t state;
	
	uint8_t rxBuff[6];
	while (1)
	{ 
		
    state = systemState;
		if(state == system_running){
		PTE->PSOR |= (1<<29);
		PTD->PTOR |= (1<<5);
		Delay(500);
			
		
		if((PTD->PDIR & (1<<1)) == 0) continue;
		
		read_i2c(MAG3110_I2C_ADDR, 1, rxBuff, 6);
		
		x = ((int16_t)(rxBuff[0] * 256 + rxBuff[1]));
		y = ((int16_t)(rxBuff[2] * 256 + rxBuff[3]));
		z = ((int16_t)(rxBuff[4] * 256 + rxBuff[5]));
		
		if (!first)
	{
	x_max = x;
	x_min = x;
	y_max = y;
	y_min = y;
	first = 1;
	}
	if (x > x_max)
	{
	x_max =  x;
	}
	else if (x < x_min)
	{
	x_min =  x;
	}
	if (y > y_max)
	{
	y_max =  y;
	}
	else if (y < y_min)
	{
	y_min =  y;
	}
	x_offset = (x_max + x_min) / 2;
	y_offset = (y_max + y_min) / 2;
	LCD_DisplayDecimal(DataProcess(x_offset,y_offset));
		//PRINTF("status_reg = 0x%x , x = %5d , y = %5d , z = %5d \r\n", PTD->PDIR & (1<<1), x, y, z);
		}
		if(state == system_stop){
		LCD_StopDisplay();
		PTD->PSOR |= (1<<5);
		PTE->PTOR |= (1<<29);
		Delay(1000);
		}
	}
}

void init_i2c()
{
	i2c_master_config_t masterConfig;
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_I2C_ConfigurePins();

	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000U;
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(I2C0_CLK_SRC));
}

void send_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
	i2c_master_transfer_t masterXfer;

	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = (uint32_t)reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &value;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferBlocking(I2C0, &masterXfer);
}

void read_i2c(uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
	i2c_master_transfer_t masterXfer;

	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = (uint32_t)reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = rxBuff;
	masterXfer.dataSize = rxSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	I2C_MasterTransferBlocking(I2C0, &masterXfer);
}
uint16_t DataProcess (int X,int Y)
{


int result;

X -= x_offset;
Y -= y_offset;
if (X == 0)
{
if (Y>0)
{
result = 90;
}
else
{
result = 270;
}
}
else if (Y == 0)
{
if (X>0)
{
result = 0;
}
else
{
result = 180;
}
}
else if ((X > 0) && (Y > 0))
{
result = (atan ( ( (float)Y) / ( (float) X ) ) ) * 180 / 3.14;
}
else if ((X < 0) && (Y > 0))
{
X = -X;
result = 180 - (atan ( ( (float)Y) / ( (float)X ) ) ) * 180 / 3.14;
}
else if ((X < 0) && (Y < 0))
{
X = -X;
Y = -Y;
result = (atan ( ( (float)Y) / ( (float) X ) ) ) * 180 / 3.14 + 180;
}
else if ((X > 0) && (Y < 0))
{
Y = -Y;
result = 360 - (atan ( ( (float)Y) / ( (float)X ) ) ) * 180 / 3.14;
}

return 	 result;
}
