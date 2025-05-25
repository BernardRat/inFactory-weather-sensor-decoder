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


#include <CC1101_ESP_Arduino.h>

//***************************************************************************************************************
//CC1101 ESP32 PINS, etc.
//***************************************************************************************************************//

const int SPI_SCK =           18;       
const int SPI_MISO =          19;      
const int SPI_MOSI =          23;      
const int SPI_CS =            17;      
const int RADIO_RCV_PIN =     13;       //select any pin GD2
const int RADIO_XMT_PIN =     25;       //select any pin GD0
int       CC1101Power = 1;              // default lowest xmit power

//Bandwidth options
//  RX_BW_812_KHZ = 0,
//  RX_BW_650_KHZ = 1,
//  RX_BW_541_KHZ = 2,
//  RX_BW_464_KHZ = 3,
//  RX_BW_406_KHZ = 4,
//  RX_BW_325_KHZ = 5,
//  RX_BW_270_KHZ = 6,
//  RX_BW_232_KHZ = 7,
//  RX_BW_203_KHZ = 8,
//  RX_BW_162_KHZ = 9,
//  RX_BW_135_KHZ = 10,
//  RX_BW_116_KHZ = 11,
//  RX_BW_102_KHZ = 12,
//  RX_BW_81_KHZ = 13,
//  RX_BW_68_KHZ = 14,
//  RX_BW_58_KHZ = 15

// XMT power options
//  TX_DEFAULT_DBM = 0,
//  TX_MINUS_30_DBM = 1,
//  TX_MINUS_20_DBM = 2,
//  TX_MINUS_15_DBM = 3,
//  TX_MINUS_10_DBM = 4,
//  TX_0_DBM = 5,
//  TX_PLUS_5_DBM = 6,
//  TX_PLUS_7_DBM = 7,
//  TX_PLUS_10_DBM = 8


CC1101 cc1101(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS, RADIO_XMT_PIN, RADIO_RCV_PIN);



//***************************************************************************************************************
//***************************************************************************************************************
static  int   rcvErrors, packetsReceived, ValuesChanged;
//***************************************************************************************************************

//***************************************************************************************************************
// Protocol timing
//***************************************************************************************************************

#define MAX_BITS        40  // max framesize


volatile bool           MsgReceived = false;
uint8_t                 buf[5];  // Store exactly 40 bits
volatile uint8_t        bitIndex = 0;
volatile unsigned long  lastTime;




//***************************************************************************************************************
void IRAM_ATTR Radio_ISR() {
  if(MsgReceived)return;
  unsigned long currentTime = micros();
  unsigned long gap = currentTime - lastTime;
  lastTime = currentTime;

  if (gap >= 5500) {
    // End of transmission
    if (bitIndex == MAX_BITS) {
      MsgReceived = true;
      bitIndex = 0;
    } else {
      // Incomplete message — reset
      bitIndex = 0;
      //for (int i = 0; i < 5; i++) buf[i] = 0;
    }
    return;
  }

  if (bitIndex < MAX_BITS) {
    uint8_t byteIndex = bitIndex / 8;
    uint8_t bitPos = 7 - (bitIndex % 8);  // MSB first

    if (gap > 3000) {           // 3000
      buf[byteIndex] |= (1 << bitPos);  // Set bit to 1
    } else {
      buf[byteIndex] &= ~(1 << bitPos); // Explicitly clear bit (optional)
    }

    bitIndex++;
  }
}

//**********************************************************************************
void setup() {


 
  Serial.begin(115200);
  delay(1000);
  Serial.println("Start");
 
  cc1101.init();
  cc1101.setTXPwr((TX_DBM)CC1101Power);
  cc1101.setMHZ(433.966);                    // Use SDR# to determine XMT freq.
  cc1101.setRxBW(RX_BW_135_KHZ);             // RX_BW_812_KHZ
  cc1101.setDataRate(2500);
  cc1101.setModulation(ASK_OOK);
  cc1101.setRx();
}
//***************************************************************************************************************/
//***************************************************************************************************************/
#define CHAN            1     // Can be 1,2 or,3 based on switch position

float                   PrevT;
int                     PrevH;
char                    Text[256];
byte                    ID;
bool                    intEnabled;
unsigned long           LastMsgRcvTime; // last time good msg recvd
unsigned long           NextTime;       // next time to enable interrupt & look for next msg
unsigned long           MsgSum;         // sum of elapsed time between received messages
unsigned long           TotalGoodMsgsRcvd;
//**********************************************************************************
void loop() {
  if(millis()>NextTime && !intEnabled){
    intEnabled = true;
    pinMode(RADIO_RCV_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RADIO_RCV_PIN), Radio_ISR, RISING);
  } 

  if (MsgReceived){
    detachInterrupt(digitalPinToInterrupt(RADIO_RCV_PIN));
    intEnabled = false;
    
    byte rcvCRC = buf[1]>>4;
    byte chan = buf[4]&0x7;
      
    int h = (buf[3] & 0x0F) * 10 + (buf[4] >> 4);
    int t1 = buf[2];
    t1 = t1 << 4;
    int t2 = (buf[3] >> 4) & 0x0F;
    int t3 = t1 + t2;
    float t = (t3-900.0)/10.0;
    t = 5.0/9.0*(t-32.0);

    bool CRC_OK = infactory_crc_check(buf);
    
    sprintf(Text, "ID = %d, CRC = %d, Tx = %d, Bat = %d, Ch = %d, CRC OK = %d, H = %d%%, T = %03.1f°C\n",
    buf[0], rcvCRC, (buf[1]>>3)&0x1, (buf[1]>>2)&0x1, chan, CRC_OK, h, t);

    if(CRC_OK && chan==CHAN && t==PrevT && h==PrevH && ID==0){
      LastMsgRcvTime = millis();
      NextTime = millis()+67000;
      ID=buf[0];
      Serial.println("Setting ID");
      Serial.println(Text);
    }
    else
      if((CRC_OK && chan==CHAN) && ((t==PrevT && h==PrevH) || ID==buf[0])){
        NextTime = millis()+67000;
        Serial.print(Text);
        unsigned long time = (millis()-LastMsgRcvTime)/1000;
        Serial.print(time);
        Serial.println(", seconds since previous message received.");
        LastMsgRcvTime = millis();
        TotalGoodMsgsRcvd++;
        MsgSum+=time;
        Serial.print((float)MsgSum/(float)TotalGoodMsgsRcvd);
        Serial.println(", average time between messages.");
        Serial.print((float)TotalGoodMsgsRcvd/(float)(millis()/3600000.0));
        Serial.println(", good messages per hour.\n");
      }
    else
      if (CRC_OK && chan==CHAN){
        PrevT = t;
        PrevH = h;
       }
    MsgReceived = false;
  }
}
//***************************************************************************************************************/
// Function to calculate CRC-4
uint8_t crc4(uint8_t const message[], unsigned nBytes, uint8_t polynomial, uint8_t init)
{
    unsigned remainder = init << 4; // LSBs are unused
    unsigned poly = polynomial << 4;
    unsigned bit;

    while (nBytes--) {
        remainder ^= *message++;
        for (bit = 0; bit < 8; bit++) {
            if (remainder & 0x80) {
                remainder = (remainder << 1) ^ poly;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder >> 4 & 0x0f; // discard the LSBs
}

//***************************************************************************************************************/

static int infactory_crc_check(uint8_t *b)
{
    uint8_t msg_crc, crc, msg[5];
    memcpy(msg, b, 5);
    msg_crc = msg[1] >> 4;
    // for CRC computation, channel bits are at the CRC position(!)
    msg[1] = (msg[1] & 0x0F) | (msg[4] & 0x0F) << 4;
    // crc4() only works with full bytes
    crc = crc4(msg, 4, 0x13, 0); // Koopmann 0x9, CCITT-4; FP-4; ITU-T G.704
    crc ^= msg[4] >> 4; // last nibble is only XORed
    return (crc == msg_crc);
}
