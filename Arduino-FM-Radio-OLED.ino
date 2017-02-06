#include <Wire.h>
#include <SI4703.h>     // http://www.mathertel.de/Arduino/RadioLibrary.aspx
#include <RDSParser.h>
#include "LowPower.h"
#include <EEPROM.h>
#include <U8g2lib.h>


#define EEPROM_Channel_SAVED   0           // EEPROM locations

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
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);  
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);


//************************** RADIO **********************************
SI4703 radio;
RADIO_INFO ri;

/// get a RDS parser
RDSParser rds;
uint16_t g_block1;
char RDSName[18];


int volume = 5;
int channel;

float volts;
boolean lowVolts = false;
unsigned long timerLowVoltage;

unsigned long timerDisplayUpdate;       // Display update timer
#define DisplayUpdateDelay 300000  // Display update set time (5 minutes).

boolean oledIsOn;


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
  
  radio.init();
//  radio.debugEnable();    //debug infos to the Serial port
  radio.setBandFrequency(RADIO_BAND_FM, 8930);
  radio.setMono(false);
  radio.setMute(false);
  radio.setVolume(volume);

  // setup the information chain for RDS data.
  radio.attachReceiveRDS(RDS_process);
  rds.attachServicenNameCallback(DisplayRDSName);
    
  delay(500);
  Serial.println(F("Adafruit Radio - Si4703 "));
  u8g2.begin();
}





void loop() {
   radio.checkRDS();
   u8g2.firstPage();
   timerDisplayUpdate = millis();
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
   

   if (millis() > (timerDisplayUpdate + DisplayUpdateDelay) ) {
     updateDisplay();
     timerDisplayUpdate = millis();
   }
   
  if (digitalRead(push) == LOW) {
     radio.seekUp(true);        
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
     Serial.println(F("UP"));
     seekAuto(radio.getFrequencyStep());
  }
  
  if (digitalRead(channelDown) == LOW) {
     startOled();
     Serial.println(F("DOWN"));
     seekAuto(-radio.getFrequencyStep());
  }
  updateDisplay();
}



void RDS_process(uint16_t block1, uint16_t block2, uint16_t block3, uint16_t block4) {
  g_block1 = block1;
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
      RADIO_FREQ f = radio.getFrequency() + step;
      if (f >radio.getMaxFrequency())  f = radio.getMinFrequency();    
      if (f <radio.getMinFrequency())  f = radio.getMaxFrequency();    
      radio.setFrequency(f);
      sprintf(RDSName, "%d.%02d MHz", (unsigned int)f/100, (unsigned int)f%100);
      updateDisplay();
      delay(100);
      radio.getRadioInfo(&ri);      
      if (ri.rssi>10  || digitalRead(volUp)==LOW   || digitalRead(volDown) == LOW || digitalRead(channelUp)==LOW || digitalRead(channelDown)==LOW) {
        break;
      } else {
        f += radio.getFrequencyStep();
      } 
    } 
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
    char buf[9];

    int level = ri.rssi;
    u8g2.drawBox(0,  62, level*3, 2);
    u8g2.drawBox(120,  64-(volume*3), 2, 64);
    
    if (!lowVolts) {
      sprintf(buf, "%d.%02d MHz", (int)radio.getFrequency() / 100, (int)(radio.getFrequency())%100);
      u8g2.drawStr(10,8, buf);
    }
    sprintf(buf, "%d.%02d", (int)volts/1000, (int)(volts/10)%100);
    u8g2.drawStr(80,8, buf);

        
    u8g2.setFont(u8g2_font_ncenB14_tr);
    if (lowVolts) {
      u8g2.drawStr(0,35, "LOW BAT");
    } else {
      u8g2.drawStr(0, 35, RDSName);
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
