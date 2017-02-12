#include <SI4703.h>     // http://www.mathertel.de/Arduino/RadioLibrary.aspx 
#include <RDSParser.h>  // https://github.com/mathertel/Radio
#include "LowPower.h"
#include <EEPROM.h>
#include <U8g2lib.h>    // https://github.com/olikraus/u8g2/wiki/u8g2reference#reference


//************************* EEPROM internal usage ****************************
#define EEPROM_FreqH   0       
#define EEPROM_FreqL   1       
#define EEPROM_VOL     2       

//******************************* Radio **************************************
#define resetRadio      2      // SI4703 reset pin    
#define FMIN 8500              // FMIN for display only
#define FMAX 11000             // FMAX for display only

//******************************* OLED ***************************************
#define oledVcc         3      // OLED VCC (for power down operation)


//****************************** 5 way ALPS switch ***************************
//          PIN_VOL_UP  1------------o   o------6  PIN_PUSH     
//     
//    PIN_CHANNEL_DOWN  2--------o   o   o------5  PIN_CHANNEL_UP        
//                                    \    
//        PIN_VOL_DOWN  3------------o  --------4  Common  

#define PIN_CHANNEL_DOWN     7      
#define PIN_VOL_DOWN         8      
#define PIN_VOL_UP           9      
#define PIN_CHANNEL_UP      10      
#define PIN_PUSH            11      


//********************************** Battery ********************************* 
#define lowVoltageWarning  3080    // Warn low battery volts.
#define lowVoltsCutoff     2900    // Kill power to display and sleep ATmega328.

//*********************************** icons ********************************** 
// BATTERY LEVEL 
#define BAT_X_POS  90
#define BAT_Y_POS   2 
#define BAT_HEIGHT  6
#define BAT_LENGTH  8
#define BAT_LOW     3000
#define BAT_FULL    3400
#define LOW_BAT     "LOW BAT"

// VOLUME LEVEL 
#define VOL_X_POS  105
#define VOL_Y_POS   0
#define VOL_WIDTH   3
#define VOL_HEIGHT  8

// RADIO LEVEL
#define RADIO_LEVEL_X_POS 110
#define RADIO_LEVEL_Y_POS 8

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define FREQSTRSIZE 12




//************************** OLED ************************************
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);


char tmp[FREQSTRSIZE];
char RDSName[FREQSTRSIZE];
RADIO_FREQ freq;
int volume = 5;
long volts;
boolean lowVolts = false;
unsigned long timerLowVoltage;
boolean oledIsOn;
RADIO_INFO ri;
RDSParser rds;
SI4703 radio;


//************************************ SETUP ******************************
void setup() {
  Serial.begin(9600);
  pinMode(PIN_VOL_DOWN, INPUT_PULLUP);
  pinMode(PIN_VOL_UP, INPUT_PULLUP);
  pinMode(PIN_CHANNEL_UP, INPUT_PULLUP);
  pinMode(PIN_CHANNEL_DOWN, INPUT_PULLUP);  
  pinMode(PIN_PUSH, INPUT_PULLUP);
  pinMode(oledVcc, OUTPUT);

  oledIsOn = false;
  startOled();
  Serial.println(F("Adafruit Radio - Si4703 "));
  
  radio.init();
//  radio.debugEnable();    //debug infos to the Serial port
  EEPROMRead();
  sprintf(RDSName, "%d.%d MHz", (unsigned int)freq/100, (unsigned int)(freq%100/10));

  radio.setBandFrequency(RADIO_BAND_FM,freq);
  radio.setMono(false);
  radio.setMute(false);
  radio.setVolume(volume);
  // setup the information chain for RDS data.
  radio.attachReceiveRDS(RDS_process);
  rds.attachServicenNameCallback(DisplayRDSName);
  
  delay(500);
  u8g2.begin();
}




//******************************* MAIN LOOP ***************************
void loop() {
   radio.checkRDS();
   u8g2.firstPage();
   volts = readVcc();
   if (volts >= lowVoltsCutoff) { 
     timerLowVoltage = millis();                             // Battery volts ok so keep resetting cutoff timer.
   }
    
   if (millis() > (timerLowVoltage + 5000)) {                // more than 5 seconds with low Voltage ?
      digitalWrite(oledVcc, LOW);                            // Kill power to display
      
      //Wire.beginTransmission(SI4703Address);                 // Put Si4703 tuner into power down mode.
      Wire.write(0x00);
      Wire.write(0x41);
      Wire.endTransmission(true);
      delay(500);
      LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);    // Put ATmega328 to sleep.
   }
   
   if ((!lowVolts) && (volts < lowVoltageWarning)) {
     lowVolts = true;  
   }   
   
   


  if (digitalRead(PIN_PUSH) == LOW) {
      EEPROMSave();
  }

  if (digitalRead(PIN_VOL_UP) == LOW) {
    if (volume < 15) volume++; 
    radio.setVolume(volume);
  }
  
  if (digitalRead(PIN_VOL_DOWN) == LOW) {
     if (volume > 0) volume--;
     radio.setVolume(volume);
  }  
  
  if (digitalRead(PIN_CHANNEL_UP) == LOW) {
     startOled();
     seekAuto(radio.getFrequencyStep());
  }
  
  if (digitalRead(PIN_CHANNEL_DOWN) == LOW) {
     startOled();
     seekAuto(-radio.getFrequencyStep());
  }
  updateDisplay();
}



void RDS_process(uint16_t block1, uint16_t block2, uint16_t block3, uint16_t block4) {
//  g_block1 = block1;
  rds.processData(block1, block2, block3, block4);
}


void DisplayRDSName(char *name)
{
  bool found = false;

  for (uint8_t n = 0; n < 8; n++)
    if (name[n] != ' ') found = true;

  if (found) {
    for (uint8_t n = 0; n < 8; n++) RDSName[n]  = name[n];
    RDSName[8]  = 0;
  }
} 




void seekAuto (int step)
{
    while (1) {
      freq = radio.getFrequency() + step;
      if (freq >radio.getMaxFrequency())  freq = radio.getMinFrequency();    
      if (freq <radio.getMinFrequency())  freq = radio.getMaxFrequency();
      radio.clearRDS();
      radio.setFrequency(freq);
      radio.formatFrequency(RDSName, FREQSTRSIZE);
      updateDisplay();
      delay(50);
      radio.getRadioInfo(&ri);      
      if (ri.rssi>10  || digitalRead(PIN_VOL_UP)==LOW   || digitalRead(PIN_VOL_DOWN) == LOW || digitalRead(PIN_CHANNEL_UP)==LOW || digitalRead(PIN_CHANNEL_DOWN)==LOW) {
        break;
      } 
    } 
}


void EEPROMSave() {
  EEPROM.write(EEPROM_FreqL, (unsigned char)(freq & 0xFF));
  EEPROM.write(EEPROM_FreqH, (unsigned char)((freq>>8) & 0xFF));
  EEPROM.write(EEPROM_VOL, volume);
}

void EEPROMRead() {
  freq =  EEPROM.read(EEPROM_FreqL) + (EEPROM.read(EEPROM_FreqH) << 8) ;
  volume =  EEPROM.read(EEPROM_VOL);
  if (volume > 15) volume=0; 

}

void stopOled() {
  digitalWrite(oledVcc, LOW);
  oledIsOn = false;
}  


void startOled() {
  if (oledIsOn == false) { 
    digitalWrite(oledVcc, HIGH);
    delay(1000);
    u8g2.begin();
  }  
}  



void updateDisplay() {
  radio.getRadioInfo(&ri);

  u8g2.firstPage();
  u8g2.setFont(u8g2_font_5x7_tr);
  do {  
      radio.formatFrequency(tmp, FREQSTRSIZE);
      u8g2.drawStr(0,8, tmp);

      // BATTERY
      u8g2.drawBox(BAT_X_POS+BAT_LENGTH, BAT_Y_POS+2, 2, 2);
      u8g2.drawFrame(BAT_X_POS , BAT_Y_POS, BAT_LENGTH, BAT_HEIGHT);
      if (volts>BAT_LOW) {
        u8g2.drawBox(BAT_X_POS , BAT_Y_POS, ((BAT_LENGTH-2)*(volts-BAT_LOW))/(BAT_FULL-BAT_LOW), BAT_HEIGHT);
      }
  
      // VOL
      u8g2.drawFrame(VOL_X_POS, VOL_Y_POS, VOL_WIDTH, VOL_HEIGHT);  
      u8g2.drawBox(VOL_X_POS, VOL_Y_POS+VOL_HEIGHT-(volume/2), VOL_WIDTH, volume/2);

      // RADIO
      for (unsigned char i=0; i<(ri.rssi/4); i++) {
        u8g2.drawVLine(RADIO_LEVEL_X_POS+(2*i), RADIO_LEVEL_Y_POS-i, i);
      }
    
      u8g2.setFont(u8g2_font_fub14_tf);
      if (lowVolts) {
        u8g2.drawStr((SCREEN_WIDTH-u8g2.getStrWidth(LOW_BAT))/2,36, LOW_BAT);
      } else {
        u8g2.drawStr((SCREEN_WIDTH-u8g2.getStrWidth(RDSName))/2,36, RDSName);
      }
      
      u8g2.setFont(u8g2_font_u8glib_4_tf);
      u8g2.drawLine(0, SCREEN_HEIGHT-1, SCREEN_WIDTH-1, SCREEN_HEIGHT-1 );
      for (RADIO_FREQ f=FMIN; f<=FMAX; f+=100 ) {
        unsigned int x = ((f-FMIN)*12)/((FMAX-FMIN)/10);
        if (f%500==0) {
          u8g2.drawVLine(x+4, SCREEN_HEIGHT-8, 8);
          sprintf(tmp, "%d", (unsigned int)f/100);
          u8g2.drawStr(x,SCREEN_HEIGHT-12, tmp);
        } else {
          u8g2.drawVLine(x+4, SCREEN_HEIGHT-4, 4);
        }
      }
      unsigned int x = 4+((freq-FMIN)*12)/((FMAX-FMIN)/10);        
      u8g2.drawTriangle(x-3, 54, x, 60, x+3, 54);
      
    } while( u8g2.nextPage() );
}





// Get Battery Voltage 

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1113000L / result; // Back-calculate AVcc in mV
  return result;
}
