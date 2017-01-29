#include <SPI.h>
#include <Si4703_Breakout.h> // https://github.com/Chnalex/Si4703_FM_Tuner_Evaluation_Board/tree/master/Libraries/Si4703_Breakout
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "LowPower.h"
#include <EEPROM.h>

#define SI4703Address 0x10
#define oledAddress 0x3c
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
#define lowVoltageWarning  1180    // Warn low battery volts.
#define lowVoltsCutoff     1100    // Kill power to display and sleep ATmega328.


//************************** OLED ************************************
#if ( SSD1306_LCDHEIGHT != 64 )
#error( "Height incorrect, please fix Adafruit_SSD1306.h!" );
#endif

#define OLED_RESET 8
Adafruit_SSD1306 oled(OLED_RESET);


//************************** RADIO **********************************
Si4703_Breakout radio(resetRadio, A4, A5,0);

int volume = 5;
int channel;

float volts;
boolean lowVolts = false;
unsigned long timerLowVoltage;


#define MAXFREQ 5 
unsigned int presetFreq[MAXFREQ] = {
  1044, 
  929, 
  910, 
  1040, 
  1060, 
 };
  
char* presetNames[MAXFREQ] = {
  "F-INFO",
  "F-MUS",
  "RIRE",
  "RTL",
  "RTL 2",
  };
unsigned int currentFreq;
char currentName[20];
char rdsBuffer[20];


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

  radio.powerOn();
  radio.setVolume(volume);
  
  channel = constrain(EEPROM.read(EEPROM_Channel_SAVED), 0, MAXFREQ-1);
  volts = readVcc();
  updateDisplay();
  
  timerDisplayUpdate = millis();
  delay(500);
  Serial.println("Adafruit Radio - Si4703 ");
}

void loop() {
   volts = readVcc();
   if (volts >= lowVoltsCutoff) { 
     timerLowVoltage = millis();                             // Battery volts ok so keep resetting cutoff timer.
   }
    
   if (millis() > (timerLowVoltage + 5000)) {                // more than 5 seconds with low Voltage ?
      digitalWrite(oledVcc, LOW);                            // Kill power to display
      
      Wire.beginTransmission(SI4703Address);                 // Put Si4703 tuner into power down mode.
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
   

  // Update display and battery voltage reading every timerUpdateTime (5 mins).
  // Don't update too often because it causes a slight click on the radio.
   if (millis() > (timerDisplayUpdate + DisplayUpdateDelay) ) {
     updateDisplay();
     timerDisplayUpdate = millis();
   }
   
  if (digitalRead(push) == LOW) {
     currentFreq = radio.seekUp();        
     radio.readPS(currentName, 2000);

 /*   if (oledIsOn == true) {
      stopOled();
    } else  {
      startOled();
    }
    */
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
     changeChannel(+1);
  }
  
  if (digitalRead(channelDown) == LOW) {
     startOled();
     changeChannel(-1);
  }

  updateDisplay();
  delay(200);
}





void changeChannel(int d) {
     channel+=d;
     if (channel < 0) channel = MAXFREQ-1;
     if (channel >= MAXFREQ) channel = 0;

     currentFreq = presetFreq[channel];
     radio.setChannel(currentFreq);
     if (EEPROM.read(EEPROM_Channel_SAVED) != channel) {
       EEPROM.write(EEPROM_Channel_SAVED, channel);
     }
     strcpy(currentName, presetNames[channel]);
}



void stopOled() {
  digitalWrite(oledVcc, LOW);
  oledIsOn = false;
}  


void startOled() {
    if (oledIsOn == false) { 
      digitalWrite(oledVcc, HIGH);
      delay(500);
      oledIsOn = true;
      oled.begin(SSD1306_SWITCHCAPVCC, oledAddress);
  }  
}  

    
void updateDisplay() {
    oled.clearDisplay();   
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(10,8);
    if (!lowVolts) {
      oled.print((float)currentFreq / 10, 1); 
      oled.print(" FM");
      Serial.println((float)currentFreq / 10);
    }
    oled.setCursor(80, 8);
    oled.print(volts/1000);
    oled.print("v"); 

    oled.setCursor(80, 50);
    int level = radio.getRSSI();
    oled.fillRect(0,  62, level*3, 2, WHITE);
    oled.fillRect(120,  64-(volume*3), 2, 64, WHITE);
    
    
    if (lowVolts) {
      oled.dim(true);
      oled.setTextSize(2);
      oled.setCursor(35,28);
      oled.print("LOW BATT");
    } else {
      oled.setTextSize(2);
      oled.setCursor(5,28);
      oled.print(currentName);  
    }
    oled.display();   
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
