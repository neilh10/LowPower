/* Low Power test for SAMD51
https://github.com/LowPowerLab/LowPower/blob/master/Examples/standbyExternalInterruptSAMD51/standbyExternalInterruptSAMD51.ino

First Pass Testing, Ardafruit M4 Express
On a USB Current Meter, it reads 23mA. 
When it sleeps still has others running
 for complete POWER_OFF ~ less than 0.1mA, measurement error at limit of method
 for cpu SLEEP_STANDBY then probably less than 0.6mA measurement at limit of method
 for SLEEP_STANDBY 120MHZ ~ 26MA-->8mA
 for SLEEP_STANDBY  60MHZ ~ 13mA-->5.2mA
                    48Mhz   11mA-->4.4MA 

 for SLEEP_STANDBY  10..24Mhz ~ 1.6mA but not console

See start up interrupts enabled 
packages\framework-arduino-samd-adafruit\cores\arduino\
Reset.cpp
cortex_handlers.c reset-->SystemInit()-->main()
startup.cpp:SystemInit()
   48MHZ USB Clock
  100MHz for peripherals
   12MHz for DAC
  Debug Trace Unit to use counter
main.cpp-->main() init() initVariant() USBDevice.attach() setup() {loop(),yield()}
wiring.c:init()
SysTick - 1MS
*/

#include "LowPower.h"

// The defintion of this build
extern const String build_ref = "a\\" __FILE__ " " __DATE__ " " __TIME__ " ";

// External interrupt on pin 0 (use pin 0 to 24, except pin 4 on Arduino Zero)
const int pin = 0;
#define COUNT_DOWN 10
int count = COUNT_DOWN;
#define SerialUSB Serial

 void print_mclk(){
  SerialUSB.print("MCLK ");
  SerialUSB.print(MCLK->APBAMASK.reg,HEX);
  SerialUSB.print(" ");
  SerialUSB.print(MCLK->APBBMASK.reg,HEX);
  SerialUSB.print(" ");
  SerialUSB.print(MCLK->APBCMASK.reg,HEX);
  SerialUSB.print(" ");
  SerialUSB.println(MCLK->APBDMASK.reg,HEX);
 }
void setup()
{
	// Wait for serial USB port to open
  SerialUSB.begin(115200);
  delay(100);
  count = 1000;
	while(!SerialUSB) {
    delay(10);
    if (count-- < 0) break;
    };
  delay(100);
  SerialUSB.println(F("\n\n---Boot Sw Build: "));
  SerialUSB.println(build_ref);
	SerialUSB.print("  ***** ATSAMD51 Low Power ");
  SerialUSB.print(F_CPU);
  SerialUSB.print("MHz *****");
	
	// ***** IMPORTANT *****
	// Delay is required to allow the USB interface to be active during
	// sketch upload process
	SerialUSB.println("Entering standby mode in:");
	for (count=COUNT_DOWN; count > 0; count--)
	{
	  SerialUSB.print(count);	
	  SerialUSB.println(" s");
	  delay(1000);
  }
  // *********************
    
  // External interrupt on pin (example: press of an active low button)
  // A pullup resistor is used to hold the signal high when no button press
  //attachInterrupt(pin, blink, LOW);

  print_mclk();

  uint32_t nvicPriority= NVIC_GetPriorityGrouping();
  SerialUSB.print("NVIC ");
  SerialUSB.println(nvicPriority);
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;

  SerialUSB.print("Check actIRQ:");
  int intlp;
  for (intlp=0; intlp< PERIPH_COUNT_IRQn; intlp++)
  {
    if (NVIC_GetEnableIRQ((IRQn_Type)intlp)) {
        SerialUSB.print(" ");
        SerialUSB.print(intlp);
    }
  }
  SerialUSB.print(" TotChecked=");
  SerialUSB.println(intlp);
}


//PM_IRQn Power Manager can't be disabled 
void lowpower_disable_ints(void) {
  SysTick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);

  // wiring.c turn off un-needed peripherals
  // need int CLKS, CLK_APBAMASK_RTC 
  MCLK->APBAMASK.reg &= ~(MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1);

  //Need 
  MCLK->APBBMASK.reg &= ~(MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TC2);

                    // 0x2000 bit appearrs to be always on;
  MCLK->APBCMASK.reg &=  ~(MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3 | MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5 );

  MCLK->APBDMASK.reg &= ~(MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
		  | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7);


}

void lowpower_enable_ints(void) {
  SysTick_Config( SystemCoreClock / 1000 );
}  


void loop() 
{
  int irq_mapping = 
	SerialUSB.println("Entering standby mode.");
	SerialUSB.println("Apply low signal to wake the processor.");
	SerialUSB.println("Zzzz...");

  // wiring.c turn off un-needed peripherals
  // need int CLKS, CLK_APBAMASK_RTC 
  MCLK->APBAMASK.reg &= ~(MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1);

  //Need 
  MCLK->APBBMASK.reg &= ~(MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TC2);

  MCLK->APBCMASK.reg &= ~(MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3 | MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5);

  MCLK->APBDMASK.reg &= ~(MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
		  | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7);  

  print_mclk();

  delay(100); //Let print finish
	// Detach USB interface
	USBDevice.detach();

  lowpower_disable_ints();
  /* Enter SAMD51 sleep mode  see manual PM_
  SAMD51 0->7 0=light, 7=all domains off,  
  mA / SLEEPMODE/Current
  23.0 RUN 
  (mA ?? - not been able to measure as other interrupts kick in)
  ??  0 IDLE0 CPU clock is OFF 
  ??  1 IDLE1 AHB clock is OFF
  ??  2 IDLE2 APB clock are OFF
  ??  4 STANDBY All Clocks are OFF, Ultra Low Power

   (mA below are low and may need more accurate measurements)
   0.6 5 HIBERNATE Backup domain is ON and some PDRAMs. Internal Reset
   0.6 6 BACKUP Only Backup domain is powered ON. Internal Reset
   0.1 7 OFF All power domains are powered OFF. External RESET 
  */
  LowPower.idle(SLEEP_STANDBY ); 
  lowpower_enable_ints();
  //Get source of interrupt
  // Attach USB interface
  USBDevice.attach();
  // Wait for serial USB port to open
  while(!SerialUSB);
  // Serial USB is blazing fast, you might miss the messages
  delay(1000);
  SerialUSB.println("Awake!");
  SerialUSB.println("Send any character to enter standby mode again");
  // Wait for user response
  while(!SerialUSB.available());
  while(SerialUSB.available() > 0)
  {
		SerialUSB.read();
	}
}

void blink(void)
{

}
