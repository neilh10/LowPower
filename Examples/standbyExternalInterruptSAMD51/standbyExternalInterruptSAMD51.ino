/* Low Power test for SAMD51
 wip - work in progress
 
Based on Examples in https://github.com/LowPowerLab/LowPower

First Pass Testing, Ardafruit M4 Express
On a USB Current Meter, it reads 23mA. 
When it sleeps still has others running
 for complete POWER_OFF ~ less than 0.1mA, measurement error at limit of method
 for cpu SLEEP_STANDBY then probably less than 0.6mA measurement at limit of method
 for SLEEP_STANDBY 120MHZ ~ 26MA-->8mA
 for SLEEP_STANDBY  60MHZ ~ 13mA-->5.2mA
                    48Mhz   11mA-->4.4MA  with USB
                    48Mhz    3.3-->1.3mA  Disable USB 1.3mA
                    12Mhz    1.6mA ~ but freezes somewhere

 for SLEEP_STANDBY  10..24Mhz ~ 1.6mA but not console

Flow of control on start up,
interrupts enabled 
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

From Variant.h 
 * PB16 SAMD51 SERCOM5/PAD[1] Rx - FTDI Pin4 Orange 
 * PB16 SAMD51 SERCOM5/PAD[0] Tx - FTDI Pin5 Yellow 
 * SERCOM5 to Serial1
*/
#include "Arduino.h"
#include "LowPower.h"
#include "RTC_SAMD51.h"

// The defintion of this build
extern const String build_ref = "a\\" __FILE__ " " __DATE__ " " __TIME__ " ";

// External interrupt on pin 0 (use pin 0 to 24, except pin 4 on Arduino Zero)
const int pin = 0;
#define COUNT_DOWN 5
int count = COUNT_DOWN;

//230126 With TTY not sleeping 
#define RUN_WITH_TTY 1
#if F_CPU >= 48000000L
#if !defined( RUN_WITH_TTY )
#if defined USBCON
//#define RUN_WITH_USB 1
#endif // USBCON
#endif // RUN_WITH_TTY 
#endif 


#if defined RUN_WITH_TTY
#define SerialTty Serial1
//#undef RUN_WITH_USB
#elif defined  RUN_WITH_USB
#define SerialTty Serial
#else
#define SerialTty Serial
#endif

RTC_SAMD51 rtcIntPhy;

 void flash_redLed(int count,int space_ms) {
  for (int lpcnt=count;  lpcnt>0; lpcnt--) {
    digitalWrite(LED_BUILTIN, HIGH); // Show we're awake again
    delay(space_ms);
    digitalWrite(LED_BUILTIN, LOW);
    delay(space_ms);
  }
 }

 void print_mclk(){
  #if 1//defined RUN_WITH_USB
  SerialTty.print("MCLK ");
  SerialTty.print(MCLK->APBAMASK.reg,HEX);
  SerialTty.print(" ");
  SerialTty.print(MCLK->APBBMASK.reg,HEX);
  SerialTty.print(" ");
  SerialTty.print(MCLK->APBCMASK.reg,HEX);
  SerialTty.print(" ");
  SerialTty.println(MCLK->APBDMASK.reg,HEX);
  #endif // RUN_WITH_USB
 }
    #define RTC_ALM_ID 0
 void print_rtc_time_field(uint32_t time_value) {
    //SerialTtyprint("Time ");
    // Doesn't like this RtcMode2Alarm rtc_mode2alm = RTC->MODE2.Mode2Alarm[RTC_ALM_ID];
    SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.YEAR );
    SerialTty.print("/");
    SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.MONTH );
    SerialTty.print("/");
    SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.DAY );
    SerialTty.print(" ");
    SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.HOUR );
    SerialTty.print(":");
    SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.MINUTE );
    SerialTty.print(":");
    SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.SECOND );        

 }

bool alarmUpdate_sema=false;
void alarmMatch(uint32_t flag)
{
    alarmUpdate_sema= true;
}
void updateAlarm(int update_secs) {
    DateTime now = rtcIntPhy.now();
    DateTime alarm = DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second() + update_secs);

    rtcIntPhy.setAlarm(RTC_ALM_ID,alarm);
    rtcIntPhy.attachInterrupt(alarmMatch);   
    rtcIntPhy.enableAlarm(RTC_ALM_ID, rtcIntPhy.MATCH_SS); // match Every Second
}

#define LED_RED 13
#define LED1 LED_RED
void setup()
{

  pinMode(LED1 , OUTPUT);
  digitalWrite(LED1 , HIGH);
  count = 50;

	// Wait for serial (possibly USB) port to open
  #define SERIAL_TTY_BAUD 115200
  SerialTty.begin(SERIAL_TTY_BAUD);
  #if defined RUN_WITH_USB
  #error
  delay(100);
	while(!SerialTty) {
    delay(100);
    if (count-- < 0) break;
    digitalWrite(LED1 , count&0x01);
    };
  delay(100);
  #else 
  #warning No USB
  USBDevice.detach();
  Serial.end();
  #endif 
  SerialTty.println(F("\n\n---Boot Sw Build: "));
  SerialTty.println(build_ref);
	SerialTty.print("  ***** ATSAMD51 Low Power ");
  SerialTty.print(F_CPU);
  SerialTty.print("MHz *****");

  //Start the RTC & set an alarm to go off once a minute
  if (!rtcIntPhy.begin()) {
      SerialTty.println("Couldn't find RTC -halting");
      while (1) delay(10); // stop operating
  }  
  #define ALARM_UPDATE_SEC 0
  updateAlarm(ALARM_UPDATE_SEC);



	// ***** IMPORTANT *****
	// Delay is required to allow the USB interface to be active during
	// sketch upload process
  #if 1//defined RUN_WITH_USB
	SerialTty.println("\n\rEntering standby mode in:");
  #endif
	for (count=COUNT_DOWN; count > 0; count--)
	{
    #if 1//defined RUN_WITH_USB
	  SerialTty.print(count);	
	  SerialTty.println(" s");
    #endif
    digitalWrite(LED1,count&0x01);
	  delay(1000);
  }
  // *********************
    
  // External interrupt on pin (example: press of an active low button)
  // A pullup resistor is used to hold the signal high when no button press
  //attachInterrupt(pin, blink, LOW);

  print_mclk();
	delay(100);
  #if 1//defined RUN_WITH_USB


  uint32_t nvicPriority= NVIC_GetPriorityGrouping();
  SerialTty.print("NVIC ");
  SerialTty.println(nvicPriority);
  #endif // RUN_WITH_USB
	delay(100);
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;

#if 1//defined RUN_WITH_USB

  SerialTty.print("Alm ");
  //SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.reg,HEX);
  print_rtc_time_field(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.reg);
  SerialTty.print(" Match ");
  //SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.reg,HEX);
  SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].MASK.bit.SEL ,HEX);

  SerialTty.print(" Ctl ");
  SerialTty.print(RTC->MODE2.CTRLA.reg ,HEX);
  SerialTty.println();
  SerialTty.print("Check actIRQ:");
  int intlp;
  for (intlp=0; intlp< PERIPH_COUNT_IRQn; intlp++)
  {
    if (NVIC_GetEnableIRQ((IRQn_Type)intlp)) {
        SerialTty.print(" ");
        SerialTty.print(intlp);
    }
  }
  SerialTty.print(" TotChecked=");
  SerialTty.println(intlp);
#endif //
	delay(100);
  digitalWrite(LED1 , LOW);
  count =0;
} //setup()


//PM_IRQn Power Manager can't be disabled 
// https://www.avrfreaks.net/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
void lowpower_disable_ints(void) {
  SerialTty.flush();
  SerialTty.end();

  SysTick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);

  // wiring.c turn off un-needed peripherals - dpesn't seem to make any difference to mA
  // need int CLKS, CLK_APBAMASK_RTC 
  /*MCLK->APBAMASK.reg &= ~(MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1);

  //Need 
  MCLK->APBBMASK.reg &= ~(MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TC2);

                    // 0x2000 bit appearrs to be always on;
  MCLK->APBCMASK.reg &=  ~(MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3 | MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5 );

  //Express M4 Serial - SERCOM5
  //MCLK->APBDMASK.reg &= ~(MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4                           | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
	//	  | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7);
  MCLK->APBDMASK.reg &= ~(MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
		  | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7);/**/


 
}

void lowpower_enable_ints(void) {
  SysTick_Config( SystemCoreClock / 1000 );
  #if defined RUN_WITH_TTY 
  //MCLK->APBDMASK.reg |=  MCLK_APBDMASK_SERCOM5;
  SerialTty.begin(SERIAL_TTY_BAUD);
  #endif 

}  


void loop() 
{
  //int irq_mapping ;
  #if 1//defined RUN_WITH_USB
	SerialTty.println("Entering standby mode.");
	SerialTty.println("RTC enabled  to wake the processor.");
	SerialTty.println("Zzzz...");
  #endif 
	delay(10);
  // wiring.c turn off un-needed peripherals
  // need int CLKS, CLK_APBAMASK_RTC 
  /*MCLK->APBAMASK.reg &= ~(MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1);

  //Need 
  MCLK->APBBMASK.reg &= ~(MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TC2);

  MCLK->APBCMASK.reg &= ~(MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3 | MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5);

  MCLK->APBDMASK.reg &= ~(MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
		  | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7);  */

  print_mclk();
  delay(100); //Let print finish

  #if defined RUN_WITH_USB
	// Detach USB interface
	USBDevice.detach();
#endif
	//delay(10);
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
#if defined RUN_WITH_USB
  // Attach USB interface
  USBDevice.attach();
  // Wait for serial USB port to open
  while(!SerialTty);
#endif 
  // Serial USB is blazing fast, you might miss the messages

  count++;

  #if 1//defined RUN_WITH_USB
  delay(200);
  SerialTty.print(count);
  SerialTty.print(":Awake ");
  SerialTty.println(alarmUpdate_sema);
  alarmUpdate_sema=false;
  //SerialTty.println(alarmUpdate_sema);
  #endif // RUN_WITH_USB

  flash_redLed(5,500);
  digitalWrite(LED1,count&0x01);

  /*SerialTty.println("Send any character to enter standby mode again");
  // Wait for user response
  while(!SerialTty.available());
  while(SerialTty.available() > 0)
  {
		SerialTty.read();
	}*/
}

void blink(void)
{

}
