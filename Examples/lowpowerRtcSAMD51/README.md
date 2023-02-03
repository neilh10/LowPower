This puts a SAMD51 to sleep, and the wakes it with the internal RTC every minute.   
Ihas been coded for two boards with SAMD51   
- Adafruit Express M4   
- Seeed Wio Terminal   
It use platformio that puts its configuration in platformio.ini   
The clock speed can be configured from 120Mhz to 48Hz.  
The serial can use the buildin USB, or external serial1 Tx Rx on each board    

# Low Power Serial1 to Ftdi cable 

To reduce power to the minimium in SLEEP, the USB is switched off and output is via the Serial Tx Rx pins.    
An FTDI cable (partnumber TTL-232R-3V3 ) - USB to 6pin 3.3V can be used for connecting on the serial.   


# Adafruit Express M4 Serial1 Tx Rx  to FTDI wiring 
See mapping of FTDI cable connector to Express M4 labeled pins.    
I use wire-wrap wire to make the links.

Ftdi   
6pin     /FeatherM4   
Black  1 – GND Feather   
Orange 4 - Rxd Feather   
Yellow 5  -Txd Feather   

<picture>
 <img alt="board support hole dimensions" src="diagrams/afm4_wiring graphic.jpg">
</picture>   

# Seeed Wio Terminal Serial1 Tx Rx  to FTDI wiring 
 See mapping of FTDI Cable connector to Wio Terminal pins 6 8 10.    
I used a right angle 6pin 0.1" to connect to the Wio T 40pin connector.
I use wire-wrap wire to make the links.    

FTDI    /Wio Terminal    
6pin    /  40pin    
Black  1 – 6/GND WioT    
Orange 4 – 10/Rxd WioT    
Yellow 5 -  8/Txd WioT   

<picture>
 <img alt="board support hole dimensions" src="diagrams/wioterm_wiring graphic.jpg">
</picture>

