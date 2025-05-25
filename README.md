# inFactory-weather-sensor-decoder
/*  This is an ESP32 program to decode and display temperature and humidity readings from an inFactory 
 *   weather sensor.  The data is transmitted on the 433 MHz band.  The protocol is nicely described
 *   in https://github.com/merbanan/rtl_433/blob/master/src/devices/infactory.c
 *   
 *   I wanted to use one of these sensors to record the temperature and humidity at a remote location.  
 *   At first I modified the ISR (Interrupt Service Routine) I use for an Acurite weather sensor for the
 *   inFactory sensor which worked well.  Then on a whim, I asked ChatGPT to write the ISR.  The ChatGPT ISR
 *   was smaller and simpler than my hand coded version, and after correcting the ChatGPT code for a minor error,
 *   it works as well as my version, so I am making it available to anyone that wants to try it.
 *   
 *   I use a CC1101 receiver module to receive the signal but any decent 433Mhz Superheterodyne receiver will
 *   also work.  One module I tried needed a pullup resistor to work, so I used 
 *   pinMode(RADIO_RCV_PIN, INPUT_PULLUP);
 *   
 *   The decoder code is mostly from the RTL_433 library with extra error detecting added.  The inFactory module
 *   transmits about once every 68 seconds so once I receive a good message I wait 67 seconds before re-enabling
 *   interrupts to look for the next message.  This eliminates having to detect other nearby devices and frees up
 *   some processor time.  I discovered during my testing that relying on the 4 bit CRC for error checking was
 *   not reliable but if the message has the correct ID (ID changes when batteries replaced), CRC, and Channel 
 *   number, you can be sure you have good data.  You can also compare the temperature and humidity to the 
 *   previous reading and if they are identical, you should have a valid reading.  I get reliable data even 
 *   through two brick walls.
 *   
 *   The inFactory sensors are available on ebay, et al. for around $12.00
 *   
 *   Sample output:
18:02:56.035 -> Start
18:03:19.786 -> Setting ID
18:03:19.786 -> ID = 54, CRC = 2, Tx = 0, Bat = 0, Ch = 1, CRC OK = 1, H = 31%, T = 24.4°C
18:03:19.786 -> 
18:04:28.465 -> ID = 54, CRC = 8, Tx = 0, Bat = 0, Ch = 1, CRC OK = 1, H = 31%, T = 24.6°C
18:04:28.465 -> 68, seconds since previous message received.
18:04:28.465 -> 68.00, average time between messages.
18:04:28.465 -> 38.51, good messages per hour.
18:04:28.465 -> 
18:05:37.471 -> ID = 54, CRC = 8, Tx = 0, Bat = 0, Ch = 1, CRC OK = 1, H = 31%, T = 24.6°C
18:05:37.471 -> 68, seconds since previous message received.
18:05:37.471 -> 68.00, average time between messages.
18:05:37.471 -> 44.32, good messages per hour.

 *    The output gives you an idea of the quality of reception.  The ideal average time between messages
 *    should be around 68 seconds, and 53 messages per hour.
 *
 *     These modules contain crystal oscilators which are not very accurate and I wanted to have the
 *     CC1101 receive frequency set as close as possible to the inFactory transmit frequency so I could use 
 *     a narrow receiver bandwidth to improve reception.  
 *     In order to set the CC1101 receive frequency to the inFactory module transmit frequency, I used
 *     SDR# to isolate the inFactory transmit frequency and then set the CC1101 to transmit, and adjusted
 *     the CC1101 frequency until it closely matched the inFactory transmit frequency. I'm assuming the
 *     CC1101 transmits and receives on the same frequency.  If you change modules, you may need to repeat 
 *     this process for optimum reception.
 *   
 *   
 *   Varialble name convention used in this program wherever possible:
     global variables first letter capitalized e.g. GlobalVariable (Pascal case),
     local variables first letter lower case e.g. localVariable (lower camel case),
     constants all capitals e.g. CONSTANT_NUMBER (screaming snake case.)
 * */
