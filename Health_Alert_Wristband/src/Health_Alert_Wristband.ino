/*
 * Project: Capstone JAS Part
 * Description: HR/O2 with Alert Button
 * Author: Janel Sanchez
 * Date: 07-Dec-2020
 */

// SYSTEM_MODE(SEMI_AUTOMATIC);

#include <Adafruit_MQTT.h>

#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "Adafruit_MQTT/Adafruit_MQTT.h" 

#include "credentials.h"

#include <Wire.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "max30102.h"
#include "MAX30105.h"                                     // MAX3010x library

#include <Adafruit_GFX.h>                                 // OLED Library
#include <Adafruit_SSD1306.h>                             // OLED Library

TCPClient TheClient; 

Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

Adafruit_MQTT_Publish heartRate = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/HR");
Adafruit_MQTT_Publish oxygen = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/O2");
Adafruit_MQTT_Publish button = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/alertButton");


#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);


// Interrupt pin
const byte oxiInt = D8;                                   // pin connected to MAX30102 INT
MAX30105 particleSensor;

uint32_t aun_ir_buffer[BUFFER_SIZE];                      //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  
uint32_t hr;                                              //red LED sensor data
float old_n_spo2, spo2;                                   // Previous SPO2 value
uint8_t uch_dummy,k;
bool isWristPlaced = false;

unsigned long lastDisplayTime;
unsigned long lastPublishTime;
unsigned long last;

const int buttonPin = D2;

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);              // Start the OLED display
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.setTextColor(WHITE);
  display.println("Place on wrist \nand wait...");
  display.display();

  pinMode (buttonPin, INPUT_PULLDOWN);                    //pin D2 connects to the button
  pinMode(oxiInt, INPUT);                                 //pin D10 connects to the interrupt output pin of the MAX30102
  Wire.begin();

  particleSensor.begin(Wire, I2C_SPEED_FAST);             // Use default I2C port, 400kHz speed
  particleSensor.setup();  
  
  Serial.begin(9600);
  maxim_max30102_reset();                                 //resets the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  maxim_max30102_init();                                  //initialize the MAX30102
  old_n_spo2=0.0;

  sync_my_time();
}

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() {
  MQTT_ping();
  processHRandSPO2();
  displayPrint(hr,spo2);
  MQTT_connect();
  publish(hr,spo2);
  pushAlertButton();
}

void processHRandSPO2(){
  long irValue = particleSensor.getIR();                  // Reading the IR value it will permit us to know if there's a finger on the sensor or not
  Serial.println(irValue);
  if (irValue > 50000){
    if(isWristPlaced == false) {
      isWristPlaced = true;
    }
    float n_spo2,ratio,correl;                            //SPO2 value
    int8_t ch_spo2_valid;                                 //indicator to show if the SPO2 calculation is valid
    int32_t n_heart_rate;                                 //heart rate value
    int8_t  ch_hr_valid;                                  //indicator to show if the heart rate calculation is valid
    int32_t i;
       
    //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
    //read BUFFER_SIZE samples, and determine the signal range
    for(i=0;i<BUFFER_SIZE;i++)
    {
      while(digitalRead(oxiInt)==1){                      //wait until the interrupt pin asserts
         yield();
      }
      
      //IMPORTANT: 
      //IR and LED are swapped here for MH-ET MAX30102. Check your vendor for MAX30102
      //and use this or the commented-out function call.
      maxim_max30102_read_fifo((aun_ir_buffer+i), (aun_red_buffer+i));
      // maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
    }
  
    //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
    rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 

    hr = n_heart_rate;
    spo2 = n_spo2;
  }
  else {
    isWristPlaced = false;
    Serial.println("No wrist detected");
  }
}


void sync_my_time () {
  Time.zone(-7);
  Particle.syncTime();
  waitUntil(Particle.syncTimeDone);
}

void displayPrint (int32_t dhr, float dspo2) {
  String DateTime, DateOnly, TimeOnly, YearOnly;
  char currentDate[11], currentTime[9], currentYear[5];

  DateTime = Time.timeStr();

  DateOnly = DateTime.substring(0,10);
  DateOnly.toCharArray(currentDate,11);

  YearOnly = DateTime.substring(20,24);
  YearOnly.toCharArray(currentYear,5);

  TimeOnly = DateTime.substring(11,19);
  TimeOnly.toCharArray(currentTime,9);

  if (isWristPlaced == true) {
    if ((millis()-lastDisplayTime)>10000) {
      display.clearDisplay();
      display.display();

      display.setTextSize(1);
      display.setCursor(0,0);
      display.setTextColor(BLACK, WHITE);
      Serial.printf("Display Date: %s %s \n", currentDate,currentYear);
      display.printf("Date: %s %s", currentDate,currentYear);
      Serial.printf("Display Time: %s \n", currentTime);
      display.printf("Time: %s\n", currentTime);

      if (dhr>1 && dspo2>1) {
        display.setTextColor(WHITE);
        Serial.printf("Display Heart Rate: %i \n", dhr);
        display.println();
        display.printf("Heart Rate: %i \n", dhr);

        display.setTextColor(WHITE);
        Serial.printf("Display Oxygen: %0.1f \n", dspo2);
        display.printf("Oxygen: %0.1f \n", dspo2);
      }
      else {
        display.setTextColor(WHITE);
        display.println();
        display.println("Measuring Vitals...");    
      }
    
      display.display();
      lastDisplayTime = millis();
    }
  }
}

void publish (int32_t phr, float pspo2) {
  if (phr>1 && pspo2>1) {
    if ((millis()-lastPublishTime)>10000) {
      if(mqtt.Update()) {
        heartRate.publish(phr);
        Serial.printf("Publishing HR: %i \n", phr);
        oxygen.publish(pspo2);
        Serial.printf("Publishing O2: %0.1f \n", pspo2);
      }
      lastPublishTime = millis();
    }
  }
} 

void pushAlertButton () {
  bool buttonState;
  static bool lastButton;

  buttonState = digitalRead(buttonPin);
  if(buttonState != lastButton) {
    if(buttonState == HIGH) {
      if(mqtt.Update()) {
        button.publish(buttonState);
        Serial.printf("Alert Button Pressed \n"); 
      } 
    }
  lastButton = buttonState;
  }
}

void MQTT_connect() {
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

void MQTT_ping() {
  if ((millis()-last)>120000) {
    Serial.printf("Pinging MQTT \n");
    if(! mqtt.ping()) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }
}