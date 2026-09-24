/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/



#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up




// Measurement Status (PN1): FlashLED1(1);
// UART Tx (PN0): 					 FlashLED2(1);
// Button Status (PF4):			 FlashLED3(1);




//*********************************************************************************************************
//*********************************************************************************************************
//***********					GLOBAL Functions				*************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
uint32_t delay = 2;	
int spin = 0;


//*********************************************************************************************************
//*********************************************************************************************************
//***********					INIT Functions				***************************************************************
//*********************************************************************************************************
//*********************************************************************************************************


void I2C_Init(void){ //I2C 
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

  GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
  GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

  GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
	GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
		
		
		
		
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
  I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
	I2C0_MTPR_R = 0b00000000000001010000000000001010; // Bits 31:16 are 0, Bits 15:8 are filter to account for glitch (00000101) and bits 7:0 are for speed
	//MTPR = 22000000/(2*10*100000) - 1 = 
	// 100000 is for 100kbps since I2C data transfers are 100k bps in standard mode (what we're using)
		
//I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}












//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

// Stepper Motor Initialization
void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
		
	GPIO_PORTL_DIR_R |= 0x0F;        								// configure Port H pins (PH0-PH3) as output
  GPIO_PORTL_AFSEL_R &= ~0x0F;     								// disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTL_DEN_R |= 0x0F;        								// enable digital I/O on Port H pins (PH0-PH3)
  GPIO_PORTL_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port H	pins (PH0-PH3)
		
	return;
}



//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}




// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}



void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize

  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 amd PJ0 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1 and PJ0
	
	GPIO_PORTJ_PCTL_R &= ~0x000000FF;	 								//? Configure PJ0 and PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											//??Disable analog functionality on PJ1 and PJ0
	GPIO_PORTJ_PUR_R |= 0x03;													//	Enable weak pull up resistor on PJ1 and PJ0 (so no floating signal when button not pressed)
}


void PortJ_Interrupt_Init(void){

		GPIO_PORTJ_IS_R = 0x00;     						// (Step 1) PJ1 is Edge-sensitive:     On falling edge - 0 indicates level detected
		GPIO_PORTJ_IBE_R = 0x00;    						//     			PJ1 is not triggered by both edges:  so clear it
		GPIO_PORTJ_IEV_R = 0x00;    						//     			PJ1 is falling edge event:   so set 0
		GPIO_PORTJ_ICR_R = 0x03;      					// 					Clear interrupt flag by setting proper bit in ICR register (for PJ0 and PJ1)
		GPIO_PORTJ_IM_R = 0x03;      					// 					Arm interrupt on PJ1 and PJ0 by setting proper bit in IM register
    
	
		// IQR# from startup file under "device" 
		// ENn# = IQR# / 32 = 51/32 = 1 ----> EN1
		// Bit# = IRQ# % 32 = 51%32 = 19 ----> Bit 19
		NVIC_EN1_R = 0x00080000;            					// (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1):   Setting Bit 19 to EN1 
	
		// PRin# = IRQ#/4 = 51/4 = PRI12    Use 4 since PRIn contains priority number for 4 interrupts (check slides)
		// Starting Bit # = 5 + 8*(51 % 4) = Bit 29 (So working with IntD) again check slides (some of reserved and rest are for different ints)
		// Set bits 31:29 to 101 (since priority number is 5)
		NVIC_PRI12_R = 0xA000000; 									// (Step 4) Set interrupt priority to 5

		EnableInt();																	// (Step 3) Enable Global Interrupt. lets go!
		
																			// Check NVIC Registers in microcontroller doc
}


void GPIOJ_IRQHandler(void)
{
	if(GPIO_PORTJ_MIS_R == 0x01) //Spins motor
	{
		spin = 1 - spin;
		FlashLED3(1); // BUTTON PRESS STATUS
		
		GPIO_PORTJ_ICR_R = 0x01;
	}
}
	





//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

void spinCheck()
{
	for(int i=0; i<16; i++) //11.25 degrees, (360/11.25 = 32 steps for inner rotor --> 2048/32 = 64 steps --> 64/4 = 16) - 64:1 outer to inner rotor ratio
	{
		GPIO_PORTL_DATA_R = 0b00000011;
		SysTick_Wait10ms(delay);											
		GPIO_PORTL_DATA_R = 0b00000110;													
		SysTick_Wait10ms(delay);
		GPIO_PORTL_DATA_R = 0b00001100;												
		SysTick_Wait10ms(delay);
		GPIO_PORTL_DATA_R = 0b00001001;													
		SysTick_Wait10ms(delay);
	}
}

void busCheck() // Infinite loop which toggles PL0 every 10ms (the systickwait10ms is actually 1ms since 1000ms per 1s so 22000000/1000 = 22000 (1ms)
{
	while(1)
	{
	GPIO_PORTL_DATA_R ^= 0b00000001;
	SysTick_Wait10ms(10);
	}
}

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus; 
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	
	// Uses bidirectional wires for data transfer (through serial data line (SDA) and serial clock line (SCL)), synchronous interface, uses leader, follower relationship 
	I2C_Init(); // Pull up resistors enabled in the I2C initialization through open draining, similar process to pull up resistors
	
	// An interface which transfers data serially in asynchronous mode
	// Enabled by start bit sent by asynchronous data communication
	UART_Init();
	PortL_Init();
	PortJ_Init();						// Initialize the onboard push button on PJ1 and PJ0
	PortJ_Interrupt_Init();	// Initalize and configure the Interrupt on Port J
	
	//busCheck();
	
	
  while(1) 
	{
		if (spin == 1)
		{
			while(UART_InChar() != 's' && spin == 1); // Doesn't run until MATLAB is ready to run, and the spin variable is 1 (in case user clicks PJ0 again before s is sent from MATLAB
			
			FlashLED2(1); // UART TX STATUS - Beginning of transmission block (handshake between MATLAB and UART successful

			// Wait for device booted
			while(sensorState==0){
				status = VL53L1X_BootState(dev, &sensorState); // Keeps checking sensor state until device booted
				SysTick_Wait10ms(10);
			}
			
			status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
			
			// Initialize the sensor with the default setting 
			status = VL53L1X_SensorInit(dev);


			status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

			while(spin == 1) // Runs until PJ0 pressed again
			{
				// Get the Distance Measures 32 times
				for(int j = 0; j < 32; j++) { // 360/11.25 = 32 steps
					
					// 5 wait until the ToF sensor's data is ready
					while (dataReady == 0){
						status = VL53L1X_CheckForDataReady(dev, &dataReady);
								VL53L1_WaitMs(dev, 5);
					}
					dataReady = 0;
					
					//7 read the data values from ToF sensor
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus); // range status must be 0 for accurate readings, if not 0, discard those readings
					status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
					status = VL53L1X_GetAmbientRate(dev, &AmbientRate); // How bright the light shining on it is
					
					//status = VL53L1X_GetSignalRate(dev, &SignalRate);  // The returned signals received in SPADs (desired signal)
					//status = VL53L1X_GetSpadNb(dev, &SpadNum); // number of detectors
					
					FlashLED1(1); // MEASUREMENT STATUS LED - Distance has been measured

					status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
					
					
					// IF TOP NOT TRANSMISSION BLOCK, THEN HERE IS TRANSMISSION BLOCK
					// FlashLED2(1);
					
					
					// print the resulted readings to UART
					sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, j, AmbientRate, SpadNum); // Where j is the step that motor currently on
					UART_printf(printf_buffer);
					SysTick_Wait10ms(50);
					
					spinCheck();
					SysTick_Wait10ms(5); // Delay to allow for motor to settle (could be the cause of motor randomly breaking down)
				}
				
				// SPIN BACK IN OPPO DIRECTION FOR 360 DEGREES EACH SCAN
				for(int i = 0; i < 512; i++)
				{
					GPIO_PORTL_DATA_R = 0b00001001;
					SysTick_Wait10ms(delay);											
					GPIO_PORTL_DATA_R = 0b00001100;													
					SysTick_Wait10ms(delay);
					GPIO_PORTL_DATA_R = 0b00000110;												
					SysTick_Wait10ms(delay);
					GPIO_PORTL_DATA_R = 0b00000011;													
					SysTick_Wait10ms(delay);
				}
				
				SysTick_Wait10ms(300); // Allows for sync up between UART and PC
			}
			
			VL53L1X_StopRanging(dev);
			
		}
		
		else
		{
			GPIO_PORTL_DATA_R = 0b00000000; // Motor not moving
			WaitForInt(); // Waiting for another interrupt
		}
		
	}

}




