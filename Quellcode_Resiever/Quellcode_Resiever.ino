#include <esp_now.h>
#include <WiFi.h>
#include "LedRing.h"
LedRing speedRing(12);

uint8_t receiverMacAddress[] = {0xF8, 0xB3, 0xB7, 0x47, 0xF8, 0x7C};


//Right motor
int enableRightMotor=22; 
int rightMotorPin1=16;
int rightMotorPin2=17;
//Left motor
int enableLeftMotor=23;
int leftMotorPin1=18;
int leftMotorPin2=19;

#define MAX_MOTOR_SPEED 230

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;

#define SIGNAL_TIMEOUT 1000  // This is signal timeout in milli seconds. We will reset the data if no signal
unsigned long lastRecvTime = 0;

struct PacketData
{
  byte xAxisValue;
  byte yAxisValue;
  byte switchPressed;
};
PacketData receiverData;

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);
void simpleMovements();
void throttleAndSteeringMovements();

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  String inputData ;
  inputData = inputData + "values " + receiverData.xAxisValue + "  " + receiverData.yAxisValue + "  " + receiverData.switchPressed;
  Serial.println(inputData);
  
  throttleAndSteeringMovements();
  
  lastRecvTime = millis();   
}

void throttleAndSteeringMovements()
{
  int throttle = map( receiverData.yAxisValue, 254, 0, -255, 255);
  int steering = map( receiverData.xAxisValue, 0, 254, -100, 100);  
  int motorDirection = 1;
  
  if (throttle < 0)       //Move car backward
  {
    motorDirection = -1;    
  }

  int rightMotorSpeed, leftMotorSpeed;
  rightMotorSpeed =  abs(throttle) - steering;
  leftMotorSpeed =  abs(throttle) + steering;
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

  rotateMotor(rightMotorSpeed * motorDirection, leftMotorSpeed * motorDirection);

  int avgSpeed = (abs(leftMotorSpeed) + abs(rightMotorSpeed)) / 2;

  speedRing.setSpeed(avgSpeed);
  speedRing.update();
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }
  
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);   
  
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  } 

  ledcWrite(enableRightMotor, abs(rightMotorSpeed));
  ledcWrite(enableLeftMotor, abs(leftMotorSpeed));    
}

void setUpPinModes()
{
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  //Set up PWM for motor speed  
  ledcAttach(enableRightMotor, PWMFreq, PWMResolution);  
  ledcAttach(enableLeftMotor, PWMFreq, PWMResolution);
  
  rotateMotor(0, 0);
}

void setup() 
{
  speedRing.begin();
  setUpPinModes();
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  //peerInfo.ifidx=WIFI_IF_AP;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  else
  {
    Serial.println("Succes: Added peer");
  } 
}


 
void loop() {

  }
  


