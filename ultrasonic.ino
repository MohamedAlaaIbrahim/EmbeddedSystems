#include <dht.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include "RTClib.h"
#include <LiquidCrystal.h>
#include "SoftwareSerial.h"
#include <DS3231.h>

DS3231  rtc(SDA, SCL);

SoftwareSerial mySerial(6, 7);

#define DHT11_PIN 4

#define trigBackPin 41
#define echoBackPin 43
#define trigFrontPin 24
#define echoFrontPin 26

#define buzzer 8

#define joystickSwitch 3
#define joystickYaxis A0
#define joystickXaxis A1

#define GearSwitch 31
#define GearYaxis A3
#define GearXaxis A2

#define INCLUDE_vTaskDelayUntil    1

#define enA 11
#define enB 12
#define in1 28
#define in2 30
#define in3 32
#define in4 34
#define enA1 10
#define enB1 9

#define cs 53

#define led 2
#define ldr A4

#define rs 39
#define en 40
#define d4 42
#define d5 44
#define d6 46
#define d7 48

#define Start_Byte 0x7E
#define Version_Byte 0xFF
#define Command_Length 0x06
#define End_Byte 0xEF
#define Acknowledge 0x00
#define ACTIVATED LOW

#define buttonNext 33
#define buttonPause 35
#define buttonPrevious 37

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
dht DHT; 

long durationBack;
int distanceBack;
long durationFront;
int distanceFront;
bool daylight = false;
bool forward = true;
int pwmOutputA;
int pwmOutputB;
char gear = 'P';
int isPlaying = 0;

byte customCharD[] = {
  B00100,
  B10101,
  B01110,
  B11111,
  B01110,
  B10101,
  B00100,
  B00000
};

byte customCharN[] = {
  B00000,
  B00100,
  B01110,
  B11111,
  B01110,
  B00100,
  B00000,
  B00000
};

SemaphoreHandle_t isPlayingS;
SemaphoreHandle_t motor;
//SemaphoreHandle_t isPlayingC;

void Brakes( void *pvParameters); 
void JoyStick(void *pvParameters);
void LCD(void *pvParameters);
void Gear(void *pvParameters);
void Light(void *pvParameters);
void MusicPlay(void *pvParameters);
void MusicSkip(void *pvParameters);

void pauseS()
{
  execute_CMD(0x0E,0,0);
  delay(500);
}

void playS()
{
  execute_CMD(0x0D,0,1); 
  delay(500);
}

void playNext()
{
  execute_CMD(0x01,0,1);
  delay(500);
}

void playPrevious()
{
  execute_CMD(0x02,0,1);
  delay(500);
}

void setVolume(int volume)
{
  execute_CMD(0x06, 0, volume); // Set the volume (0x00~0x30)
  delay(2000);
}

void execute_CMD(byte CMD, byte Par1, byte Par2)
// Excecute the command and parameters
{
    // Calculate the checksum (2 bytes)
    word checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);
    // Build the command line
    byte Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge,
    Par1, Par2, highByte(checksum), lowByte(checksum), End_Byte};
    //Send the command line to the module
    for (byte k=0; k<10; k++){
      mySerial.write( Command_line[k]);
  }
}

void playFirst()
{
  execute_CMD(0x3F, 0, 0);
  delay(500);
  setVolume(20);
  delay(500);
  execute_CMD(0x11,0,1); 
  delay(500);
}

void setup() {
  // put your setup code here, to run once:
  
  pinMode(trigBackPin, OUTPUT);
  pinMode(echoBackPin,INPUT);
  pinMode(trigFrontPin, OUTPUT);
  pinMode(echoFrontPin,INPUT);
  
  pinMode(buzzer, OUTPUT);
  
  pinMode(joystickSwitch, INPUT);
  pinMode(joystickYaxis, INPUT);
  pinMode(joystickXaxis, INPUT);
  digitalWrite(joystickSwitch, HIGH);

  pinMode(GearSwitch, INPUT);
  pinMode(GearYaxis, INPUT);
  pinMode(GearXaxis, INPUT);
  digitalWrite(GearSwitch, HIGH);
  
  pinMode(enA, OUTPUT);
  pinMode(enA1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enB1, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  pinMode(53, OUTPUT);

  pinMode(buttonPause, INPUT);
  digitalWrite(buttonPause,HIGH);
  pinMode(buttonNext, INPUT);
  digitalWrite(buttonNext,HIGH);
  pinMode(buttonPrevious, INPUT);
  digitalWrite(buttonPrevious,HIGH);

  pinMode(led, OUTPUT);
  pinMode(ldr, INPUT);
  motor = xSemaphoreCreateBinary();
  isPlayingS = xSemaphoreCreateBinary();
  xSemaphoreGive(motor);
  
  Serial.begin(9600);

  rtc.begin();
  
  mySerial.begin (9600);
  delay(1000);
  playFirst();
  xSemaphoreGive(isPlayingS);
  
  Serial.println("LCD INTIAL");
  
  xTaskCreate(Brakes, "Brakes", 128, NULL, 4, NULL);
  xTaskCreate(JoyStick, "JoyStick", 128, NULL, 3, NULL);
  xTaskCreate(LCD, "LCD", 128, NULL, 3, NULL);
  xTaskCreate(Gear, "Gear", 128, NULL, 3, NULL);
  xTaskCreate(Light, "Light", 128, NULL, 2, NULL);
  xTaskCreate(MusicPlay, "MusicPlay", 128, NULL, 2, NULL);
  xTaskCreate(MusicSkip, "MusicSkip", 128, NULL, 2, NULL);
}

void Brakes(void *pvParameters){
  TickType_t xLastWakeTime ;
  const TickType_t xDelay200ms = pdMS_TO_TICKS ( 200 );
  const TickType_t xDelay150ms = pdMS_TO_TICKS ( 150 );
  const TickType_t xDelay50ms = pdMS_TO_TICKS ( 100 );
  const TickType_t xDelay10ms = pdMS_TO_TICKS ( 10 );
  const TickType_t xDelay2ms = pdMS_TO_TICKS ( 2 );
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    if(forward){
      digitalWrite(trigFrontPin, LOW);
      vTaskDelayUntil(&xLastWakeTime, xDelay150ms);
      digitalWrite(trigFrontPin, HIGH);
      vTaskDelayUntil(&xLastWakeTime, xDelay10ms);
      digitalWrite(trigFrontPin, LOW);
      durationFront = pulseIn(echoFrontPin, HIGH);
      distanceFront = durationFront * 0.034/2;
      
      digitalWrite(trigBackPin, LOW);
      vTaskDelayUntil(&xLastWakeTime, xDelay150ms);
      digitalWrite(trigBackPin, HIGH);
      vTaskDelayUntil(&xLastWakeTime, xDelay10ms);
      digitalWrite(trigBackPin, LOW);
      durationBack = pulseIn(echoBackPin, HIGH);
      distanceBack = durationBack * 0.034/2;
      
      if(distanceFront <= 20 || distanceBack <= 20){
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        digitalWrite(buzzer,HIGH);
        vTaskDelayUntil(&xLastWakeTime, xDelay200ms);
        digitalWrite(buzzer, LOW);
        Serial.println("BUZZER LOW");
        vTaskDelayUntil(&xLastWakeTime, xDelay200ms);
    }
    else{
      xSemaphoreGive(motor);
      Serial.println("Semaphore given");
   }
  }
  else{
//      digitalWrite(trigBackPin,LOW);
//      vTaskDelayUntil(&xLastWakeTime, xDelay2ms);
//      digitalWrite(trigBackPin,HIGH);
//      vTaskDelayUntil(&xLastWakeTime, xDelay10ms);
//      digitalWrite(trigBackPin,LOW);
//      durationBack = pulseIn(echoBackPin, HIGH);
//      distanceBack = durationBack * 0.034/2;
//      if(distanceBack <= 20){
//        digitalWrite(in1, LOW);
//        digitalWrite(in2, LOW);
//        digitalWrite(in3, LOW);
//        digitalWrite(in4, LOW);
//      digitalWrite(buzzer,HIGH);
//      vTaskDelayUntil(&xLastWakeTime, xDelay50ms);
//      digitalWrite(buzzer, LOW);
//      vTaskDelayUntil(&xLastWakeTime, xDelay50ms);
//      }
//      else{
//      xSemaphoreGive(backwards);
//        Serial.println("Semaphore backwards given");
//    }
    }
  }
}

void JoyStick(void *pvParameters){
  TickType_t xLastWakeTime ;
  const TickType_t xDelayms = pdMS_TO_TICKS ( 100 );
  xLastWakeTime = xTaskGetTickCount();
  while(true){
  xSemaphoreTake(motor,portMAX_DELAY);
  int yAxis = analogRead(joystickYaxis);
  int xAxis = analogRead(joystickXaxis);
  //yAxis = analogRead(joystickYaxis);
  if(yAxis< 470){
    pwmOutputA = map(yAxis, 470, 0, 0 , 255);
    pwmOutputB = map(yAxis, 470, 0, 0 , 255); 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    //forward = false;
  }
  else if(yAxis> 550){
    pwmOutputA = map(yAxis, 550, 1023, 0 , 255);
    pwmOutputB = map(yAxis, 550, 1023, 0 , 255); 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    forward = true;
  }
  else{
    pwmOutputA = 0;
    pwmOutputB = 0; 
  }
  if(xAxis< 470){
    int xMapped = map(xAxis, 470, 0, 0, 255);
    pwmOutputA = pwmOutputA - xMapped;
    pwmOutputB = pwmOutputB + xMapped;
    if(pwmOutputA < 0){
      pwmOutputA = 0;
    }
    if(pwmOutputB > 255){
      pwmOutputB = 255;
    }
  }
  else if(xAxis> 550){
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    pwmOutputA = pwmOutputA + xMapped;
    pwmOutputB = pwmOutputB - xMapped;
    if(pwmOutputB < 0){
      pwmOutputB = 0;
    }
    if(pwmOutputA > 255){
      pwmOutputA = 255;
    }
  }
   analogWrite(enA,pwmOutputA);
   analogWrite(enB,pwmOutputB);
   analogWrite(enA1,pwmOutputA);
   analogWrite(enB1,pwmOutputB);
   vTaskDelayUntil(&xLastWakeTime, xDelayms);
  }
}

void Gear(void *pvParameters){
  TickType_t xLastWakeTime ;
  const TickType_t xDelayms = pdMS_TO_TICKS ( 100 );
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    int yAxis = analogRead(GearYaxis);
    int xAxis = analogRead(GearXaxis);
    if(yAxis > 900){
      gear = 'D';
    }
    else{
      if(xAxis < 100){
        gear = 'N';
      }
      else{
        if(xAxis > 900){
          gear = 'P'; 
        }
        else{
          if(yAxis < 100){
            gear = 'R';
          }
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xDelayms);
  }
}

void LCD(void *pvParameters){
  TickType_t xLastWakeTime ;
  const TickType_t xDelayms = pdMS_TO_TICKS ( 1000 );
  xLastWakeTime = xTaskGetTickCount();
  int chk = DHT.read11(DHT11_PIN);
  lcd.begin(16, 2);
  lcd.createChar(0, customCharN);
  lcd.createChar(1, customCharD);
  while(true){
    lcd.clear();
    lcd.print("Temp:");
    lcd.print(DHT.temperature);
    lcd.print(" ");
    lcd.print(gear);
    lcd.print(" ");
    if(daylight){
      lcd.write(byte(0));
    }
    else{
      lcd.write(byte(1));
    }
    lcd.setCursor(0, 1);
    lcd.print(rtc.getDateStr());
    lcd.print(' ');
    lcd.print(rtc.getTimeStr());
    Serial.println(DHT.temperature);
    vTaskDelayUntil(&xLastWakeTime, xDelayms);
  }
}

void Light(void *pvParameters){
  TickType_t xLastWakeTime ;
  const TickType_t xDelayms = pdMS_TO_TICKS ( 5000 );
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    int ldrReading = analogRead(ldr);
    if(ldrReading <= 300){
      digitalWrite(led, HIGH);
      daylight = true;
    }
    else{
      digitalWrite(led, LOW);
      daylight = false;
    }
    vTaskDelayUntil(&xLastWakeTime, xDelayms);
  }
}

void MusicPlay(void *pvParameters){
  TickType_t xLastWakeTime ;
  const TickType_t xDelayms = pdMS_TO_TICKS ( 500 );
  xLastWakeTime = xTaskGetTickCount();
  Serial.println("MUSIC");
  while(true){
    if (digitalRead(buttonPause) == ACTIVATED){
      if(isPlaying%2 == 0){
        xSemaphoreTake(isPlayingS,portMAX_DELAY);
        pauseS();
      }else{
        playS();
        xSemaphoreGive(isPlayingS);
      }
      isPlaying++;
      Serial.print("isPlaying counter  ");
      Serial.println(isPlaying);
    }
  }
}

void MusicSkip(void *pvParameters){
  TickType_t xLastWakeTime ;
  const TickType_t xDelayms = pdMS_TO_TICKS ( 500 );
  const TickType_t xDelay30ms = pdMS_TO_TICKS ( 30 );
  xLastWakeTime = xTaskGetTickCount();
  while(true){
    if (digitalRead(buttonPrevious) == ACTIVATED){
      if( xSemaphoreTake(isPlayingS,xDelay30ms) == pdTRUE ){
        xSemaphoreTake(isPlayingS,xDelay30ms);
        playPrevious();
        xSemaphoreGive(isPlayingS);
      }
    }
    
   if (digitalRead(buttonNext) == ACTIVATED){
      if( xSemaphoreTake(isPlayingS,xDelay30ms) == pdTRUE ){
          playNext(); 
          xSemaphoreGive(isPlayingS);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xDelayms);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
