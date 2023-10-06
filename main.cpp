/**
 * \file SignalEditorAVR128DB28.cpp
 * \brief Generates and edits signals from up to 16 channels (After 8 requires 2 ADC Multiplexers) ADC, a pot for signal smoothing, and a pot used for timing \n
 * For in-depth details, checkout the schematic for this. \n
 *
 *
 * Uses a 24MHz external crystal, 8 pots connected to a 3 line select ADC multiplexer (MC14051BCP), a TIP31C NPN, a 358 external Op Amp, and other components.
 *
 *
 * \author Tim Robbins
 */

//Headers, Macros, and Definitions-----------------------------------------------

///If we have a second adc mux connected
#define ADC_MUX_2_INCLUDED			0

///The cpu speed
#define F_CPU						24000000UL

#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

///The max value for the ADC register
#define ADC_RESOLUTION				4095

///The DAC out port
#define DAC_OUT_PORT				VPORTD_OUT

///The ADC direction register
#define ADC_DIR_REG					VPORTD_DIR

///The DAC out pin
#define DAC_OUT_PIN					6 //Pin 12

///The port for the adc mux
#define ADC_MUX_OUT					VPORTC_OUT

///The port for the second adc mux
#define ADC_MUX_2_OUT				VPORTA_OUT

///Pins to control the external MUX
#define ADC_MUX_PIN_MASK			((1 << 0) | (1 << 1) | (1 << 2))

///Control pins for adc mux 2
#define ADC_MUX_2_PIN_MASK			((1 << 2) | (1 << 3) | (1 << 4))

///The start bit for the adc mux pin mask
#define ADC_MUX_LSB					0

///The start bit for the second adc mux pin mask
#define ADC_MUX_2_LSB				2

///The ADC Input pins
#define ADC_PIN_1					1
#define ADC_PIN_2					2
#define ADC_PIN_3					3
#define ADC_PIN_4					4
#define ADC_PIN_5					5
#define ADC_PIN_6					7 //<- Being used by DAC out, adc input pin 6 will be adc 7

///The pin mask for the ADC pins
#define ADC_PIN_MASK				((1 << ADC_PIN_1) | (1 << ADC_PIN_2) | (1 << ADC_PIN_3) | (1 << ADC_PIN_4) | (1 << ADC_PIN_5) | (1 << ADC_PIN_6))

///The adc pin connected to the adc mux
#define ADC_MUX_ADC					ADC_PIN_1

///Th adc connected to the second adc mux
#define ADC_MUX_2_ADC				ADC_PIN_2

///The adc pin to read the value for the step delay value
#define ADC_STEP_READ				ADC_PIN_3

///The adc connected to timing
#define ADC_TIMING_READ				ADC_PIN_4

///The cutout value for the dac to step, above this it will just move to the next value
#define DAC_STEP_HIGH_CUTOUT		750

///The cutout value for the dac to step, below this it will just move to the next value
#define DAC_STEP_LOW_CUTOUT			0

///The maximum step value
#define MAX_STEP_VALUE				0x3FF

///The cutout value for the DAC to register as LOW instead of its actual value
#define DAC_LOW_CUTOUT				10

///The cutout value for the DAC to register as HIGH instead of its actual value
#define DAC_HIGH_CUTOUT				1024

///Configures the mirror slope status pin as an input pull-up
#define SETUP_MIRROR_SLOPE_PIN()	VPORTA_OUT |= (1 < 7); VPORTA_DIR &= ~(1 << 7); PORTA_PIN7CTRL |= (1 << 3)

///Reads the status of the mirror slope status. Returns true is grounded, false if not
#define READ_MIRROR_SLOPE()			( ((VPORTA_IN >> 7) && 0x01 ) ? false : true)


//-------------------------------------------------------------------------------


//Data types---------------------------------------------------------------------


//-------------------------------------------------------------------------------


//Variables----------------------------------------------------------------------

///The phases for each step for the DAC out signal
#if defined(ADC_MUX_2_INCLUDED) && ADC_MUX_2_INCLUDED > 0
static uint16_t dacPhases[16] =
#else
static uint16_t dacPhases[8] =
#endif
{
	0
};

///The delay for each dac step interrupt
static uint16_t dacStepTime = 24000;

///The amount for each DAC step
static uint16_t dacStepSize = 1;

///The current dac value
static uint16_t currentDacValue = 0;

///The current DAC phase
static uint8_t currentDacPhase = 0;

///The current prescaler value
static uint8_t currentPrescaler = 0;

///If we should count back down when reaching the max phase or if we should cycle back to 0
static bool mirrorSlope = false;

///The amount of phases in the dac phases
static const uint8_t dacPhaseCount = sizeof(dacPhases);


//-------------------------------------------------------------------------------


//Functions----------------------------------------------------------------------
void SysInit();
void DacInit();
void AdcInit(uint8_t adcPinMask);
void TimerInit();
void NextPhase(uint8_t countsPerPhase, bool reverseCounts);
void WriteDac();
uint16_t SampleAdc(uint8_t adcChannel, uint8_t sampleCount);
void ReadMux();



//-------------------------------------------------------------------------------


//Interrupts---------------------------------------------------------------------

/**
* \brief Timer counter overflow interrupt
*
*/
ISR(TCA0_OVF_vect)
{
	//Stop our timer
	TCA0_SINGLE_CTRLA &= ~(1 << TCA_SINGLE_ENABLE_bp);
	
	//Clear the timer overflow flag
	TCA0_SINGLE_INTFLAGS |= (1 << TCA_SINGLE_OVF_bp);
	
	//Write our dac value
	WriteDac();
	
	//Set our timer period
	TCA0_SINGLE_PER = 0xFFFF - dacStepTime;
	
	//Set our timers prescaler
	TCA0_SINGLE_CTRLA = (currentPrescaler << TCA_SINGLE_CLKSEL_0_bp);
	
	//Restart our timer
	TCA0_SINGLE_CTRLA |= (1 << TCA_SINGLE_ENABLE_bp);
	
}

//-------------------------------------------------------------------------------


/**
* \brief Entry point to program
*
*/
int main(void)
{
	//Initialize the controller
	SysInit();
	
	while (1)
	{
		//Read our mux values
		ReadMux();
		
		//Get our step time
		dacStepTime = (uint16_t)(((float)SampleAdc(ADC_TIMING_READ,12) / (float)ADC_RESOLUTION) * (float)(0xEDFE)) + 1200;
		
		//Get our step size
		dacStepSize = (uint16_t)(((float)SampleAdc(ADC_STEP_READ,12) / (float)ADC_RESOLUTION) * (float)(MAX_STEP_VALUE));
		
		//Get out mirror slope state
		mirrorSlope = READ_MIRROR_SLOPE();
		
		//Short delay for processing
		while(0);
		
	}
}



/**
* \brief System Initialization
*
*/
void SysInit()
{
	//Initialize external crystal
	CLKCTRL_XOSCHFCTRLA |= (CLKCTRL_FRQRANGE_24M_gc);
	CLKCTRL_XOSCHFCTRLA |= CLKCTRL_ENABLE_bm;
	
	//Select external crystal as the main clock
	CLKCTRL_MCLKCTRLA |= CLKCTRL_CLKSEL_EXTCLK_gc;
	
	//Make sure clock set to appropriate speed
	CLKCTRL_OSCHFCTRLA |= CLKCTRL_FRQSEL_24M_gc;
	
	//Setup default pin settings
	VPORTA_OUT = 0x00;
	VPORTC_OUT = 0x00;
	VPORTD_OUT = 0x00;
	VPORTF_OUT = 0x00;
	
	VPORTA_DIR = 0xff;
	VPORTC_DIR = 0xff;
	VPORTD_DIR = 0xff;
	VPORTF_DIR = 0xff;
	
	
	//Initialize our DAC
	DacInit();
	
	//Initialize our ADC
	AdcInit(ADC_PIN_MASK);
	
	//Setup timer
	TimerInit();
	
	//Setup our pin for mirror slop detection
	SETUP_MIRROR_SLOPE_PIN();
	
}



/**
* \brief Initializes our DAC
*
*/
void DacInit()
{
	//Select the VREF for the DAC
	VREF_DAC0REF = (1 << VREF_ALWAYSON_bp | 2);
	
	//Enable our DAC output
	DAC0_CTRLA |= (1 << DAC_OUTEN_bp);
	
	//Make sure to clear our DAC data on startup
	DAC0.DATA = 0;
	
	//Enable our DAC
	DAC0_CTRLA |= (1 << DAC_ENABLE_bp);
	
	
}



/**
* \brief Initializes our ADC
*
*/
void AdcInit(uint8_t adcPinMask)
{
	//Configure the VREF for the ADC
	VREF_ADC0REF |= (1 << VREF_ALWAYSON_bp | 2);
	
	//Select pins to be enabled as ADC input
	ADC_DIR_REG &= ~(adcPinMask);
	
	//Disable digital input
	
	//Make sure results are right justified
	ADC0_CTRLA &= ~(1 << ADC_LEFTADJ_bp);
	
	//Make sure we're in free running mode
	ADC0_CTRLA |= (1 << ADC_FREERUN_bp);
	
	//Enable ADC
	ADC0_CTRLA |= (1 << ADC_ENABLE_bp);
	
}



/**
* \brief initializes our timers
*
*/
void TimerInit()
{
	
	//Set our timer period
	TCA0_SINGLE_PER = 0xffff - dacStepSize;
	
	//Set our timers prescaler
	TCA0_SINGLE_CTRLA = (currentPrescaler << TCA_SINGLE_CLKSEL_0_bp);
	
	//Enable timer 0 overflow interrupt
	TCA0_SINGLE_INTCTRL |= (1 << 0);
	
	//Start our timer
	TCA0_SINGLE_CTRLA |= (1 << TCA_SINGLE_ENABLE_bp);
	
	//Make sure global interrupts are on
	sei();
}



/**
* \brief Moves towards the next phase
*
*/
void NextPhase(uint8_t countsPerPhase, bool reverseCounts)
{
	//Variables
	static bool countingBack; //If we're counting backwards or not
	static uint8_t countsPerStep; //The counts per step
	
	//If we can go to the next phase...
	if(++countsPerStep > countsPerPhase)
	{
		//Reset the counts
		countsPerStep = 0;
		
		//If we're reversing the counts...
		if(reverseCounts)
		{
			//If we're counting down...
			if(countingBack)
			{
				//If the next phase is out of range...
				if(currentDacPhase <= 0)
				{
					//Set the current phase
					currentDacPhase = 1;
					
					//Toggle the state
					countingBack = false;
				}
				//else...
				else
				{
					//Decrement
					currentDacPhase--;
				}
			}
			else
			{
				//If the next phase is out of range...
				if(currentDacPhase >= dacPhaseCount - 1)
				{
					//Set the current phase
					currentDacPhase = dacPhaseCount - 2;
					
					//Toggle the state
					countingBack = true;
				}
				//else...
				else
				{
					//Increment
					currentDacPhase++;
				}
			}
			
		}
		else
		{
			//If the next phase is out of range...
			if(++currentDacPhase >= dacPhaseCount)
			{
				//Set the current phase
				currentDacPhase = 0;
			}
		}
	}
}



/**
* \brief Handles writing to the DAC
*
*/
void WriteDac()
{
	//Variables
	static bool countingDown; //If we're currently in the counting down phase
	
	//Set our DAC data
	DAC0_DATA = (currentDacValue << 6);
	
	//If we are counting down...
	if(countingDown)
	{
		//If we can count down even more...
		if(currentDacValue > (dacPhases[currentDacPhase]) && currentDacValue >= dacStepSize && dacStepSize < DAC_STEP_HIGH_CUTOUT && dacStepSize > DAC_STEP_LOW_CUTOUT)
		{
			//Count down
			currentDacValue -= dacStepSize;
		}
		else
		{
			//Set the current DAC value
			currentDacValue = dacPhases[currentDacPhase];
			
			//Move to the next phase
			NextPhase(0,mirrorSlope);
			
			//Set the current state
			countingDown = (currentDacValue > dacPhases[currentDacPhase]) ? true : false;
		}
	}
	//else...
	else
	{
		//If we can count up more...
		if(currentDacValue < (dacPhases[currentDacPhase]) && currentDacValue < (1023 - dacStepSize) && dacStepSize < DAC_STEP_HIGH_CUTOUT && dacStepSize > DAC_STEP_LOW_CUTOUT )
		{
			currentDacValue += dacStepSize;
		}
		else
		{
			//Set the current DAC value
			currentDacValue = dacPhases[currentDacPhase];
			
			//Move to the next phase
			NextPhase(0,mirrorSlope);
			
			//Set the current state
			countingDown = (currentDacValue > dacPhases[currentDacPhase]) ? true : false;
		}
	}
}



/**
* \brief Samples ADC for sampleCount Counts on channel adcChannel, returning the average
*
*/
uint16_t SampleAdc(uint8_t adcChannel, uint8_t sampleCount)
{
	//Variables
	uint16_t adcResult = 0; //Return value from the ADC
	
	//If the sample count is greater than 0, avoid divide by 0 errors,...
	if(sampleCount > 0)
	{
		//Create a variable for the loop
		uint8_t i = 0;

		//Select the passed channel
		ADC0_MUXPOS |= adcChannel;

		//While the index is less than the passed sample count...
		while(i < sampleCount)
		{
			i++;

			//Start our conversion
			ADC0_COMMAND |= (1 << ADC_STCONV_bp);
			
			//Wait until the conversion is finished
			while( !((ADC0_INTFLAGS >> ADC_RESRDY_bp) & 0x01));
			
			//Add our result
			adcResult += ADC0_RES;
		}
		
		//Divide our result by the passed sample count
		adcResult /= sampleCount;

		//Make sure the clear the selected channel on the way out
		ADC0_MUXPOS &= ~adcChannel;

	}
	
	//Return our result
	return adcResult;
}



/**
* \brief Reads the external ADC multiplexer
*
*/
void ReadMux()
{
	//Variables
	volatile uint16_t currentAdcReading = 0; //The current reading from the ADC. We need this so the interrupt does not set to the wrong value and also allow us to remove some troublesome noise
	
	//Loop through each of the ADC mux channels and...
	for(uint8_t i = 0; i < dacPhaseCount; i++)
	{
		
		#if defined(ADC_MUX_2_INCLUDED) && ADC_MUX_2_INCLUDED > 0
		
			if(i >= 8)
			{
				//Clear the mux pins
				ADC_MUX_2_OUT &= ~(ADC_MUX_2_PIN_MASK);
				
				//Select the current adc mux channel
				ADC_MUX_2_OUT |= (i << ADC_MUX_2_LSB);
				
				//Read the ADC after converting it into the correct range
				currentAdcReading = (uint16_t)(((float)SampleAdc(ADC_MUX_2_ADC, 12) / (float)ADC_RESOLUTION) * 1023.0f);
				
				//If our DAC phase is too small...
				if(currentAdcReading < DAC_LOW_CUTOUT)
				{
					//Set out DAC phase to 0
					dacPhases[i] = 0;
				}
				//else, if our dac phase is too big...
				else if(currentAdcReading > DAC_HIGH_CUTOUT)
				{
					//Set our dac phase to its max
					dacPhases[i] = 0x3FF;
				}
				//else...
				else
				{
					//Set our dac value
					dacPhases[i] = currentAdcReading;
				}
				
				//Clear the mux pins
				ADC_MUX_2_OUT &= ~(ADC_MUX_2_PIN_MASK);
				
			}
			else
			{
				//Clear the mux pins
				ADC_MUX_OUT &= ~(ADC_MUX_PIN_MASK);
				
				//Select the current adc mux channel
				ADC_MUX_OUT |= (i << ADC_MUX_LSB);
				
				//Read the ADC into the saved DAC register after converting it into the correct range
				currentAdcReading = (uint16_t)(((float)SampleAdc(ADC_MUX_ADC, 12) / (float)ADC_RESOLUTION) * 1023.0f);
				
				//If our DAC phase is too small...
				if(currentAdcReading < DAC_LOW_CUTOUT)
				{
					//Set out DAC phase to 0
					dacPhases[i] = 0;
				}
				//else, if our dac phase is too big...
				else if(currentAdcReading > DAC_HIGH_CUTOUT)
				{
					//Set our dac phase to its max
					dacPhases[i] = 0x3FF;
				}
				//else...
				else
				{
					//Set our dac value
					dacPhases[i] = currentAdcReading;
				}
				
				//Clear the mux pins
				ADC_MUX_OUT &= ~(ADC_MUX_PIN_MASK);
			}
		
		#else
		
			//Clear the mux pins
			ADC_MUX_OUT &= ~(ADC_MUX_PIN_MASK);
				
			//Select the current adc mux channel
			ADC_MUX_OUT |= (i << ADC_MUX_LSB);
			
			//Read the ADC into the saved DAC register after converting it into the correct range
			currentAdcReading = (uint16_t)(((float)SampleAdc(ADC_MUX_ADC, 12) / (float)ADC_RESOLUTION) * 1023.0f);
			
			//If our DAC phase is too small...
			if(currentAdcReading < DAC_LOW_CUTOUT)
			{
				//Set out DAC phase to 0
				dacPhases[i] = 0;
			}
			//else, if our dac phase is too big...
			else if(currentAdcReading > DAC_HIGH_CUTOUT)
			{
				//Set our dac phase to its max
				dacPhases[i] = 0x3FF;
			}
			//else...
			else
			{
				//Set our dac value
				dacPhases[i] = currentAdcReading;
			}
			
			//Clear the mux pins
			ADC_MUX_OUT &= ~(ADC_MUX_PIN_MASK);
			
		#endif
	}

}