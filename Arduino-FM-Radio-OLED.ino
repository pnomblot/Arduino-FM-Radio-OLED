#include <Wire.h>
#include <SI4703.h>     // http://www.mathertel.de/Arduino/RadioLibrary.aspx  https://github.com/mathertel/Radio
#include <RDSParser.h>
#include "LowPower.h"
#include <EEPROM.h>
#include <U8g2lib.h>


#define EEPROM_FreqH    0      // EEPROM locations
#define EEPROM_FreqL    1      // EEPROM locations

// Pin connection
#define resetRadio     2
#define oledVcc        3
#define channelDown    7
#define volDown        8
#define volUp          9
#define channelUp      10
#define push           11

//********************* Battery ************************************** 
#define lowVoltageWarning  180    // Warn low battery volts.
#define lowVoltsCutoff     100    // Kill power to display and sleep ATmega328.


//************************** OLED ************************************
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);


//************************** RADIO **********************************

//uint16_t g_block1;
char tmp[12];
char RDSName[12];
RADIO_FREQ freq;
int volume = 5;
float volts;
boolean lowVolts = false;
unsigned long timerLowVoltage;
boolean oledIsOn;
RADIO_INFO ri;
RDSParser rds;
SI4703 radio;


void setup() {
  Serial.begin(9600);
  pinMode(volDown, INPUT_PULLUP);
  pinMode(volUp, INPUT_PULLUP);
  pinMode(channelUp, INPUT_PULLUP);
  pinMode(channelDown, INPUT_PULLUP);  
  pinMode(push, INPUT_PULLUP);
  pinMode(oledVcc, OUTPUT);

  oledIsOn = false;
  startOled();
  Serial.println(F("Adafruit Radio - Si4703 "));
  
  radio.init();
//  radio.debugEnable();    //debug infos to the Serial port
  EEPROMReadFreq();
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
     updateDisplay();
   }   
   
   


  if (digitalRead(push) == LOW) {
      EEPROMSaveFreq();
  }

  if (digitalRead(volUp) == LOW) {
    if (volume < 15) volume++; 
    radio.setVolume(volume);
  }
  
  if (digitalRead(volDown) == LOW) {
     if (volume > 0) volume--;
     radio.setVolume(volume);
  }  
  
  if (digitalRead(channelUp) == LOW) {
     startOled();
     seekAuto(radio.getFrequencyStep());
  }
  
  if (digitalRead(channelDown) == LOW) {
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
      sprintf(RDSName, "%d.%d MHz", (unsigned int)freq/100, (unsigned int)(freq%100/10));

      radio.setFrequency(freq);
      updateDisplay();
      delay(100);
      radio.getRadioInfo(&ri);      
      if (ri.rssi>8  || digitalRead(volUp)==LOW   || digitalRead(volDown) == LOW || digitalRead(channelUp)==LOW || digitalRead(channelDown)==LOW) {
        break;
      } else {
        freq += radio.getFrequencyStep();
      } 
    } 

  EEPROMSaveFreq();
}


void EEPROMSaveFreq() {
  EEPROM.write(EEPROM_FreqL, (unsigned char)(freq & 0xFF));
  EEPROM.write(EEPROM_FreqH, (unsigned char)((freq>>8) & 0xFF));
}

void EEPROMReadFreq() {
  freq =  EEPROM.read(EEPROM_FreqL) + (EEPROM.read(EEPROM_FreqH) << 8) ;
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
    if (!lowVolts) {
      sprintf(tmp, "%d.%d MHz", (unsigned int)freq/100, (unsigned int)(freq%100/10));
      u8g2.drawStr(0,8, tmp);
    }
    sprintf(tmp, "%d.%02d", (int)volts/1000, (int)(volts/10)%100);
    //u8g2.drawStr(100,8, tmp);

    for (unsigned char i=0; i<(ri.rssi/4); i++) {
      u8g2.drawVLine(98+(2*i), 8-i, i);
    }
    
    u8g2.drawFrame(80, 0, 4, 8);  
    u8g2.drawBox(80, 8-(volume/2), 4, volume/2);
 
    u8g2.setFont(u8g2_font_fub14_tf);
    if (lowVolts) {
      u8g2.drawStr(0,40, "LOW BAT");
    } else {
      u8g2.drawStr(0,40, RDSName);
    }
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
