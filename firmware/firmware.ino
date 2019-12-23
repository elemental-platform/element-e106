// Element E106 v1.0.6 firmware

// Developed by AKstudios
// Updated: 12/22/2019

#include <RFM69.h>  //  https://github.com/LowPowerLab/RFM69
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h> 
#include <Adafruit_SHT31.h> //https://github.com/adafruit/Adafruit_SHT31
#include <Adafruit_TSL2591.h> // https://github.com/adafruit/Adafruit_TSL2591_Library
//#include <Adafruit_ADS1015.h> // https://github.com/adafruit/Adafruit_ADS1X15
#include <avr/sleep.h>
#include <avr/wdt.h>

// define node parameters
#define NODEID              41  // supports 10bit addresses (up to 1023 node IDs)
#define NETWORKID           40
#define ROOM_GATEWAYID      40
#define GATEWAYID           1
#define GATEWAY_NETWORKID   1
#define FREQUENCY           RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define ENCRYPTKEY          "Tt-Mh=SQ#dn#JY3_" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW          //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define LED                 9 // led pin
#define RED                 5
#define GREEN               6
#define BLUE                7
#define POWER               4

// define objects
RFM69 radio;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
//Adafruit_ADS1115 ads;

// define other global variables
int wake_interval = 0;
float batt;
//float adc16;

char dataPacket[150];

ISR(WDT_vect)  // Interrupt Service Routine for WatchDog Timer
{
  wdt_disable();  // disable watchdog
}

void setup()
{
  Serial.begin(115200);
  
  pinMode(POWER, OUTPUT);
  pinMode(LED, OUTPUT);  // pin 9 controls LED
  
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);

  //fadeRGBLED(GREEN);
  fadeLED(LED);
}


void sleep()
{
  Serial.flush(); // empty the send buffer, before continue with; going to sleep
  radio.sleep();
  digitalWrite(POWER, LOW);
  delay(1);

  cli();          // stop interrupts
  MCUSR = 0;
  WDTCSR  = (1<<WDCE | 1<<WDE);     // watchdog change enable
  WDTCSR  = 1<<WDIE | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0); // set  prescaler to 8 second
  sei();  // enable global interrupts
 
  byte _ADCSRA = ADCSRA;  // save ADC state
  ADCSRA &= ~(1 << ADEN);

  asm("wdr");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();       

  sleep_enable();  
  sleep_bod_disable();
  sei();       
  sleep_cpu();   
    
  sleep_disable();   
  sei();  

  ADCSRA = _ADCSRA; // restore ADC state (enable ADC)
  //delay(1);
}


void loop() 
{
  sleep();
  
  if(wake_interval == 4)    // if enough time has passed (5 intervals = ~54 seconds), take measurements and transmit
  {
    readSensors();
   
    // send datapacket
    radio.sendWithRetry(ROOM_GATEWAYID, dataPacket, strlen(dataPacket));  // send data
    sleep();   // sleep 8 seconds before sending data to main gateway
    radio.setNetwork(GATEWAY_NETWORKID);
    radio.sendWithRetry(GATEWAYID, dataPacket, strlen(dataPacket));
    radio.setNetwork(NETWORKID);
    
    memset(dataPacket, 0, sizeof dataPacket);   // clear array
    blinkLED(LED);
    //blinkRGBLED(GREEN);
    //checkBattery();

    wake_interval = 0;    // reset wake interval to 0
  }
  else
    wake_interval++;    // increment no. of times node wakes up
}


void readSensors()
{
  digitalWrite(POWER, HIGH);
  delay(1);

  // Light Intensity - TSL2591
  float lux, infrared;
  tsl.begin();
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  sensors_event_t event;
  tsl.getEvent(&event);
  if ((event.light == 0) | (event.light > 4294966000.0) | (event.light <-4294966000.0))
  {
    lux = 0;  // invalid value; replace with 'NAN' if needed
  }
  else
  {
    lux = event.light;
  }
  //delay(1);
  
  // T/RH - SHT31
  sht31.begin(0x44);
  float temp = sht31.readTemperature();
  float rh = sht31.readHumidity();

  // external temp reading
  float adc = averageADC(A0);
  float R = resistance(adc, 10000); // Replace 10,000 ohm with the actual resistance of the resistor measured using a multimeter (e.g. 9880 ohm)
  float air_temp = steinhart_2(R);  // get temperature from thermistor using the custom Steinhart-hart equation
  if (air_temp < -200 || air_temp > 1000) // thermistor not connected or incorrect value
    air_temp = 0;

//  // ADS1115 16-bit ADC -- external air temperature
//  ads.begin();
//  ads.setGain(GAIN_ONE);
//  delay(1);
//  adc16 = samples(1);   // get avg ADC value from channel 1 
//  float R = resistance(adc16, 10000); // Replace 10,000 ohm with the actual resistance of the resistor measured using a multimeter (e.g. 9880 ohm)
//  float air_temp = steinhart_2(R);  // get temperature from thermistor using the custom Steinhart-hart equation by US sensors

  // read battery level
  float avg=0.0;
  for(int i=0; i<5; i++)
  {
    avg = avg + analogRead(A7);
  }
  float adc_a7 = avg / 5.0;
  batt = (adc_a7/1023) * 2 * 3.3;

  
  // define character arrays for all variables
  //char _i[3];
  char _t[7];
  char _h[7];
  char _l[7];
  char _a[7];
  char _b[5];

  // convert all flaoting point and integer variables into character arrays
  //dtostrf(NODEID, 1, 0, _i);
  dtostrf(temp, 3, 2, _t);  // this function converts float into char array. 3 is minimum width, 2 is decimal precision
  dtostrf(rh, 3, 2, _h);
  dtostrf(lux, 1, 0, _l);
  dtostrf(air_temp, 3, 2, _a);
  dtostrf(batt, 4, 2, _b);
  
  dataPacket[0] = 0;  // first value of dataPacket should be a 0
  
  // create datapacket by combining all character arrays into a large character array
  //strcat(dataPacket, "i:");
  //strcat(dataPacket, _i);
  strcat(dataPacket, "t:");
  strcat(dataPacket, _t);
  strcat(dataPacket, ",h:");
  strcat(dataPacket, _h);
  strcat(dataPacket, ",l:");
  strcat(dataPacket, _l);
  strcat(dataPacket, ",a:");
  strcat(dataPacket, _a);
  strcat(dataPacket, ",b:");
  strcat(dataPacket, _b);
  delay(5);
  
}

//// Perform multiple iterations to get higher accuracy ADC values (reduce noise) ******************************************
//float samples(int pin)
//{
//  float n=5.0;  // number of iterations to perform
//  float sum=0.0;  //store sum as a 32-bit number
//  for(int i=0;i<n;i++)
//  {
//    //float value = analogRead(pin);
//    float value = ads.readADC_SingleEnded(pin);
//    sum = sum + value;
//    delay(1); // makes readings slower - probably don't need this delay, but ¯\_(ツ)_/¯
//  }
//  float average = sum/n;   //store average as a 32-bit number with decimal accuracy
//  return average;
//}


// Averaging ADC values to counter noise in readings  *********************************************
float averageADC(int pin)
{
  float sum=0.0;
  for(int i=0;i<5;i++)
  {
     sum = sum + analogRead(pin);
  }
  float average = sum/5.0;
  return average;
}

// Get resistance ****************************************************************
float resistance(float adc, int true_R)
{
  float R = true_R/(1023.0/adc-1.0);   // convert 10-bit reading into resistance
  //float ADCvalue = adc*(8.192/3.3);  // Vcc = 8.192 on GAIN_ONE setting, Arduino Vcc = 3.3V in this case
  //R = 10000/(65535/ADCvalue-1);  // 65535 refers to 16-bit number
  return R;
}


// Get temperature from Steinhart equation (10K Precision Epoxy Thermistor - 3950 NTC) *****************
float steinhart_2(float R)
{
  float steinhart;
  steinhart = R / 10000;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= 3950.0;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25.0 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C

  return steinhart;
}


void checkBattery()
{
  if(batt < 3.65)
    fadeLED(RED);
}


//// Fade LED *****************************************
void fadeLED(int pin)
{
  int brightness = 0;
  int fadeAmount = 5;
  for(int i=0; i<510; i=i+5)  // 255 is max analog value, 255 * 2 = 510
  {
    analogWrite(pin, brightness);  // pin 9 is LED
  
    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;  // increment brightness level by 5 each time (0 is lowest, 255 is highest)
  
    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255)
    {
      fadeAmount = -fadeAmount;
    }
    // wait for 20-30 milliseconds to see the dimming effect
    delay(10);
  }
  digitalWrite(pin, LOW); // switch LED off at the end of fade
}

////// blink LED *****************************************
void blinkLED(int pin)
{
  digitalWrite(pin, HIGH);
  delay(5);
  digitalWrite(pin, LOW);
}

// Fade RGB LED (common anode) *****************************************
void fadeRGBLED(int pin)
{
  int brightness = 255;
  int fadeAmount = 5;
  for(int i=0; i<510; i=i+5)  // 255 is max analog value, 255 * 2 = 510
  {
    analogWrite(pin, brightness);
  
    // change the brightness for next time through the loop:
    brightness = brightness - fadeAmount;  // increment brightness level by 5 each time (0 is lowest, 255 is highest)
  
    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= 0 || brightness >= 255)
    {
      fadeAmount = -fadeAmount;
    }
    // wait for 20-30 milliseconds to see the dimming effect
    delay(10);
  }
  digitalWrite(pin, HIGH); // switch LED off at the end of fade
}
// blink RGB LED (common anode) *****************************************
void blinkRGBLED(int pin)
{
  digitalWrite(pin, LOW);
  delay(5);
  digitalWrite(pin, HIGH);
}

// bruh
