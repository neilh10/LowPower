/* Low Power RTC characterizationtest for SAMD51
 wip - work in progress
 
Based on Examples in https://github.com/LowPowerLab/LowPower

For upload, double press the reset line to put into boot.

user interface is 
a) Blue LED - flashes at various stages of 
b) print via UART_TXRX or USB

This program create options for measuring current via an external method
eg USB Current meter with no battery
more accurately a current device on battery

Current is affected by hardware enalbed and speed
board_build.f_cpu 
serial - either USB or  UART TXRX or none

Ardafruit M4 Express
PRINT_WITH_TXRX requires FTDI (or similar) connected - 
    3 wires afM4:FTDI ==> GND:GND TX:RX RX:TX


When it sleeps still has others running
 for complete POWER_OFF ~ less than 0.1mA, measurement error at limit of method
 for cpu SLEEP_STANDBY then probably less than 0.6mA measurement at limit of method
 for SLEEP_STANDBY 120MHZ ~ 26MA-->8.0mA with USB
 for SLEEP_STANDBY  60MHZ ~ 13mA-->5.2mA with USB
                    48Mhz   11mA-->4.4MA with USB, sleeps, wakes
                    48Mhz    3.3-->1.3mA  with Uart TxRx  
                    12Mhz    1.6mA ~ but freezes somewhere, probably USBCON init


WIO Terminal
USB +5V always has weak Green LED on, current unknown likely to be ~200uA
XOU/XIN 32?Hz & 32Mhz (Xtal=306010026) 
Startup with LCD and RTL820 off, no microSD in slot
                        Wake-->Sleep
for SLEEP_STANDBY 48MHz 18mA -->11mA with USB
for SLEEP_STANDBY 48MHz 11mA --> 5.5mA SERIAL1/FTDI

Circuit ANalysis - USB_5V ~ MP2161GJ buck VCC_3V3  IQ=17uA very good
    LED1 GREEN Pwr - always On - naughty 


Arduino BSP Flow of control on start up,
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

From \.platformio\packages\framework-arduino-samd-adafruit\variants\feather_m4\Variant.h\cpp
 * PB16 SAMD51 SERCOM5/PAD[1] Rx WioPin10 -WwWhite- FTDI Pin4 Orange 
 * PB16 SAMD51 SERCOM5/PAD[0] Tx WioPin8  -WwRed  - FTDI Pin5 Yellow
 * GND WioPin6                     - WireWrapGreen- FTDI Pin1 Black 
 * SERCOM5 to Serial1


From \.platformio\packages\framework-arduino-samd-adafruit\variants\feather_m4\Variant.h\cpp 
#define PIN_SERIAL1_RX (41ul)
#define PIN_SERIAL1_TX (40ul)
 * PB27 SAMD51 SERCOM2/PAD[1] Rx - FTDI Pin4 Orange 
 * PB26 SAMD51 SERCOM2/PAD[0] Tx - FTDI Pin5 Yellow 
 * SERCOM2(alt SERCOM4) to Serial1
*/
#include "Arduino.h"
#include "LowPower.h"
#include "RTC_SAMD51.h"
#if defined WIO_TERMINAL
//#include <rpcWiFi.h>
//#include <SPI.h>
//#include "TFT_eSPI.h"
#endif //WIO_TERMINAL

// ** CHARACTERIZATION ** MODIFY FOR TESTING ***

//UART can be <none>, PRINT_DETAILS 0
// Serial1 - PRINT_WITH_TXRX
// default USB - no PRINT_WITH_TXRX

// PRINT_DETAILS 0 - not print including setup information ** not going to sleep
// PRINT_DETAILS 1 - state information
// PRINT_DETAILS 2 - internal information
#define PRINT_DETAILS 2
#define PRINT_WITH_TXRX 1
#define SERIAL_TTY_BAUD 115200
// Note : This module can be configured to be for AF4 or WioTerminal 
// There are two types of USB Drivers, and for TinyUSB two variants
// framework-arduino-samd-adafruit\cores\arduino\mmain.cpp 
// framework-arduino-samd-seeed\cores\arduino\main.cpp 
// These are USE_TINYUSB and USBCON
//  USE_TINYUSB is initialized slightly differently between the two
// Adafruit Express M4 uses USBCON --? alternate TinyUSB_Device_Init(0)
// Seeed Wio Terminal use TINYUSB Adafruit_TinyUSB_Core_init();tinyusb_task();
//
//

/// **** DON'T CHANGE DEFINES BELOW   *****

// The defintion of this build
extern const String build_ref = "a\\" __FILE__ " " __DATE__ " " __TIME__ " ";

//User signaling options - see code
// LED always enabled
#define LED_RED 13
#define LED1 LED_RED

#if defined WIO_TERMINAL
#define LCD_BACKLIGHT (72Ul) // Control Pin of LCD
#endif //WIO_TERMINAL
// CPU clock can be varied 120MH-->48Mhz
// platformio  none default - 120MHz
// board_build.f_cpu= 48000000L for 48MHz
#if F_CPU >= 48000000L
#if defined USBCON & !defined(PRINT_WITH_TXRX)
#define RUN_WITH_USB 1
#endif // USBCON
#endif //F_CPU

#if defined PRINT_WITH_TXRX
#define SerialTty Serial1
//#undef RUN_WITH_USB
#elif defined RUN_WITH_USB
#define SerialTty Serial
#else
#define SerialTty Serial
#endif // PRINT_WITH_TXRX

// Internal Physical RTC and definitions
RTC_SAMD51 rtcIntPhy;
#define RTC_ALM_ID 0
#define ALARM_UPDATE_SEC 0
bool alarmUpdate_sema = false;

#define COUNT_DOWN 5
int count;

// Forward references
void lowpower_disable_ints(void);
void lowpower_enable_ints(void);
void flash_redLed(int count, int space_ms);
void print_mclk(void);
void print_rtc_time_field(uint32_t time_value);
void alarmMatch(uint32_t flag);
void updateAlarm(int update_secs);

void setup()
{
  /*32.5.2 On Reset, all PORT lines are configured as inputs with input buffers, 
  On BACKUP sleep mode, 
  even if the PORT configuration registers and input synchronizers will lose their contents 
  (these will not be restored when PORT is powered up again), 
  the latches in the pads will keep their current configuration, 
  such as the output value and pull settings
  The PORT peripheral will continue operating in any Sleep mode where its source clock is running.
  */
#if defined WIO_TERMINAL
#warning compiling for WIO_TERMINAL
  // in ordfder of variant.h 
  // LED
  // TX/RX - can be switched with ROLE
  // Digital and Analog Arduino pins
  // RPI BCM Connector - no action
  // FPC Connector - no action 
  // RPI Analog Iverlay - no action
  // USB - PIN_USB_HOST_ENABLE (for power)
  //Button_1 _2 _3 - cct pullup 4.7K  no action
  //SWITCH_X _Z _Y  _B _U pulled up 100K - no action

  // IRQ0  PC20 From RTL8720D - 
  pinMode(IRQ0,  INPUT_PULLUP); //?
  //Buzzer_CTR - Output control, Q4 driver, pulled down
  //pinMode(BUZZER_CTR,  INPUT_PULLDOWN);

  //MIC_INPUT - no action

  //GCLK - debug leave
  //Serial1 sercom2
  //Serial2 sercon1
  // I2C WIRE sercom3 - pulled up ext
  // I2C WIRE1 sercom4 -pulled up?
  //    GYROSCOPE - no external pull U5 LIS3DHTR PIN_WRE1_SCL _SDA 
  //  
  //pinMode(GYROSCOPE_INT1,  INPUT_PULLUP); // Push-Pull - no action 

  // micro SD socket 
  //SDCARD_SPI/SPI2  _SCK_PIN _SS_PIN _MOSI  _MISO _DET
  //  SDCARD_DET_PIN is pulled high with 100K
  pinMode(SDCARD_SS_PIN,  INPUT_PULLUP);
  pinMode(SDCARD_DET_PIN,  INPUT);

  //LCD
  //SPI LCD_SCK _CS  _MOSI_ MISOC  
  //    LCD_D/C 
  //    LCD_RESET  - Pulled hihg 4.7K
  //    LCD_BACKLIGHT=LOW  off

  pinMode(LCD_SS_PIN,   INPUT_PULLUP);
  pinMode(LCD_SCK_PIN,  INPUT_PULLDOWN);
  pinMode(LCD_MISO_PIN, INPUT_PULLDOWN);
  pinMode(LCD_MOSI_PIN, INPUT_PULLDOWN);
  pinMode(LCD_RESET,    INPUT);
  //Something causes to go from 6.0 to 6.5mA
  /*pinMode(LCD_DC,       INPUT_PULLDOWN); //??
  pinMode(LCD_XL,       INPUT_PULLDOWN);  
  pinMode(LCD_YU,       INPUT_PULLDOWN);  
  pinMode(LCD_XR,       INPUT_PULLDOWN);  
  pinMode(LCD_YD,       INPUT_PULLDOWN); /* */

  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, LOW);

  //Turn off WiFi RTL8720D_CHIP_PU = LOW
  pinMode(RTL8720D_CHIP_PU, OUTPUT);
  
  digitalWrite(RTL8720D_CHIP_PU, LOW);
  // For power off, should other pins be low
  // RTL8720D_TXD _RXD
  // RTL8720D_SPI  _MISO_PIN _MOSI_PIN _SCK_PIN _SS_PIN  
  //RTL8720D_GPIO0    //low

  //QSPI  W25Q32JVZPIM
  // PIN_QSPI_CS _SCK _IO0  _IO1 _IO2 _IO3
  // PIN_QSPI_CS  should be high, 
  pinMode(PIN_QSPI_CS,  INPUT_PULLUP);
  // others are high impedance, so could pulled weakly low to hold them
  pinMode(PIN_QSPI_SCK, INPUT_PULLDOWN);
  pinMode(PIN_QSPI_IO0, INPUT_PULLDOWN);
  pinMode(PIN_QSPI_IO1, INPUT_PULLDOWN);
  pinMode(PIN_QSPI_IO2, INPUT_PULLDOWN);
  pinMode(PIN_QSPI_IO3, INPUT_PULLDOWN);

  //I2S sent to RPI - not used
  //Light sensor - input normally pulled low through 10K
  //ir sensors - output pulled low 

 //U11 ATECC608/DNP  I2C0_SCL _SDA 
 
//Power SW for RPI IO - outputs pulled low.
//OUTPUT_CTR_5V 
//OUTPUT_CTR_3V3

#endif //WIO_TERMINAL
  //Simple user interface
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

#if PRINT_DETAILS > 0
  // Wait for serial (possibly USB) port to open
  SerialTty.begin(SERIAL_TTY_BAUD);
#if defined RUN_WITH_USB
  delay(100);
  count = 500;
  while (!SerialTty)
  {
    delay(100);
    if (count-- < 0)
      break;
    digitalWrite(LED1, count & 0x01);
  };
  delay(100);
#else
//Ensure USB is removed
#warning USB Disabled
  bool statusUsb=false;
  statusUsb = USBDevice.detach();
  Serial.end();
  statusUsb &= USBDevice.end();

#endif //RUN_WITH_USB

  SerialTty.println(F("\n\n---Boot Sw Build: "));
  SerialTty.println(build_ref);
  SerialTty.print("  ***** Low Power RTC SAMD51 ");
  SerialTty.print(F_CPU);
  SerialTty.print("MHz ***** ");
#if defined RUN_WITH_USB
  SerialTty.print("USB");
#else
  SerialTty.print("UART UsbDis=");
    SerialTty.print(statusUsb);
#endif // RUN_WITH_USB
#endif // PRINT_DETAILS

  if (!rtcIntPhy.begin())
  {
#if PRINT_DETAILS > 0
    SerialTty.println("Couldn't find RTC -halting");
#endif // PRINT_DETAILS
    while (1)
    {
      flash_redLed(10, 100);
      delay(2000);
    }
  }

  updateAlarm(ALARM_UPDATE_SEC);

#if PRINT_DETAILS > 0
  SerialTty.println("\n\rSet Alarm, starting in:");
#endif // PRINT_DETAILS
  for (count = COUNT_DOWN; count > 0; count--)
  {
#if PRINT_DETAILS > 0
    SerialTty.print(count);
    SerialTty.println(" s");
#endif // PRINT_DETAILS
    digitalWrite(LED1, count & 0x01);
    delay(1000);
  }
  // *********************

  // External interrupt on pin (example: press of an active low button)
  // A pullup resistor is used to hold the signal high when no button press
  //attachInterrupt(pin, blink, LOW);

  print_mclk();
  delay(100);

#if PRINT_DETAILS > 1
  uint32_t nvicPriority = NVIC_GetPriorityGrouping();
  SerialTty.print("NVIC ");
  SerialTty.println(nvicPriority);

  delay(100);
#endif //  PRINT_DETAILS > 1

#if PRINT_DETAILS > 1

  SerialTty.print("Alm ");
  //SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.reg,HEX);
  print_rtc_time_field(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.reg);
  SerialTty.print(" Match ");
  //SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.reg,HEX);
  SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].MASK.bit.SEL, HEX);

  SerialTty.print(" Ctl ");
  SerialTty.print(RTC->MODE2.CTRLA.reg, HEX);
  SerialTty.println();
  SerialTty.print("Check actIRQ:");
  int intlp;
  for (intlp = 0; intlp < PERIPH_COUNT_IRQn; intlp++)
  {
    if (NVIC_GetEnableIRQ((IRQn_Type)intlp))
    {
      SerialTty.print(" ");
      SerialTty.print(intlp);
    }
  }
  SerialTty.print(" TotChecked=");
  SerialTty.println(intlp);
#endif //PRINT_DETAILS

  delay(100);
  digitalWrite(LED1, LOW);
  count = 0;

  
 //Data Watchpoint and Trace Unit - 
 // Seperate core arm_cortexm4_processor_trm_100166_0001_00_en Technical Ref Manual.pdf
 // Turn off free running Cycle Count Register in DWT_CTRL
 // May be a problem for some debug
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  
} //setup()

void loop()
{
//int irq_mapping ;
#if PRINT_DETAILS > 0
  SerialTty.println("Entering sleep standby mode.");
  SerialTty.println("RTC enabled  to wake the processor.");
  SerialTty.println("Zzzz...");
  delay(10);
#endif //PRINT_DETAILS

  print_mclk();

#if defined RUN_WITH_USB & (PRINT_DETAILS > 0)
  // Detach USB interface
  USBDevice.detach();
#endif //RUN_WITH_USB

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
  LowPower.idle(SLEEP_STANDBY);
  lowpower_enable_ints();
  //Get source of interrupt
#if defined RUN_WITH_USB & (PRINT_DETAILS > 0)
  // Attach USB interface
  USBDevice.attach();
  // Wait for serial USB port to open
  while (!SerialTty)
    ;
#endif
  // Serial USB is blazing fast, you might miss the messages

  count++;
#if PRINT_DETAILS > 0
  delay(200);
  SerialTty.print(count);
  SerialTty.print(":Awake ");
  SerialTty.println(alarmUpdate_sema);
  alarmUpdate_sema = false;
#endif                  // PRINT_DETAILS
  delay(2000);          //Allow 2secs measurement time
  flash_redLed(5, 500); //5secs medium flashing
  digitalWrite(LED1, count & 0x01);

} // loop()

//PM_IRQn Power Manager can't be disabled
// https://www.avrfreaks.net/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
void lowpower_disable_ints(void)
{
#if PRINT_DETAILS > 0
  SerialTty.flush();
  SerialTty.end();
#endif // PRINT_DETAILS

  SysTick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);

#if 0
  // wiring.c turn OFF un-needed peripherals - and then need to turn them ON
  // - dpesn't seem to make measureable difference to mA
  // need int CLKS, CLK_APBAMASK_RTC 
  MCLK->APBAMASK.reg &= ~(MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1);

  //Need 
  MCLK->APBBMASK.reg &= ~(MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TC2);

                    // 0x2000 bit appearrs to be always on;
  MCLK->APBCMASK.reg &=  ~(MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3 | MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5 );

  //Express M4 Serial - SERCOM5
  //MCLK->APBDMASK.reg &= ~(MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4                           | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
	//	  | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7);
  MCLK->APBDMASK.reg &= ~(MCLK_APBDMASK_DAC | MCLK_APBDMASK_SERCOM4 | MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_ADC0 | MCLK_APBDMASK_ADC1 | MCLK_APBDMASK_TCC4
		  | MCLK_APBDMASK_TC6 | MCLK_APBDMASK_TC7 | MCLK_APBDMASK_SERCOM6 | MCLK_APBDMASK_SERCOM7);/**/
#endif

} // lowpower_disable_ints

void lowpower_enable_ints(void)
{
  SysTick_Config(SystemCoreClock / 1000);
#if PRINT_DETAILS > 0 //defined PRINT_WITH_TXRX
  //MCLK->APBDMASK.reg |=  MCLK_APBDMASK_SERCOM5;
  SerialTty.begin(SERIAL_TTY_BAUD);
#endif
} // lowpower_enable_ints

void flash_redLed(int count, int space_ms)
{
  for (int lpcnt = count; lpcnt > 0; lpcnt--)
  {
    digitalWrite(LED_BUILTIN, HIGH); // Show we're awake again
    delay(space_ms);
    digitalWrite(LED_BUILTIN, LOW);
    delay(space_ms);
  }
} // flash_redLed

// This is called at interrupt -
// Wakes up processor and then resumes normal processing
void alarmMatch(uint32_t flag)
{
  alarmUpdate_sema = true;
} // alarmMatch

void updateAlarm(int update_secs)
{
  DateTime now = rtcIntPhy.now();
  DateTime alarm = DateTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second() + update_secs);

  rtcIntPhy.setAlarm(RTC_ALM_ID, alarm);
  rtcIntPhy.attachInterrupt(alarmMatch);
  rtcIntPhy.enableAlarm(RTC_ALM_ID, rtcIntPhy.MATCH_SS); // match Every Second
} // updateAlarm

void print_rtc_time_field(uint32_t time_value)
{
#if PRINT_DETAILS > 0
  //SerialTtyprint("Time ");
  // Doesn't like this RtcMode2Alarm rtc_mode2alm = RTC->MODE2.Mode2Alarm[RTC_ALM_ID];
  SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.YEAR);
  SerialTty.print("/");
  SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.MONTH);
  SerialTty.print("/");
  SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.DAY);
  SerialTty.print(" ");
  SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.HOUR);
  SerialTty.print(":");
  SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.MINUTE);
  SerialTty.print(":");
  SerialTty.print(RTC->MODE2.Mode2Alarm[RTC_ALM_ID].ALARM.bit.SECOND);
#endif // PRINT_DETAILS

} //  print_rtc_time_field(

void print_mclk(void)
{
#if PRINT_DETAILS > 1
  SerialTty.print("MCLK ");
  SerialTty.print(MCLK->APBAMASK.reg, HEX);
  SerialTty.print(" ");
  SerialTty.print(MCLK->APBBMASK.reg, HEX);
  SerialTty.print(" ");
  SerialTty.print(MCLK->APBCMASK.reg, HEX);
  SerialTty.print(" ");
  SerialTty.println(MCLK->APBDMASK.reg, HEX);
  delay(100); //Let print finish
#endif        // PRINT_DETAILS
} // print_mclk
