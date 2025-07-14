#include <esp_now.h>
#include <WiFi.h>
#include "LedRing.h"
#include "DistanceSensor.h"
#include "AudioPlayer.h"

LedRing speedRing(12);

// Distance sensor pins
#define TRIG_PIN 33
#define ECHO_PIN 14
#define DF_RX 26   // ESP32 receives here → DFPlayer TX
#define DF_TX 25   // ESP32 sends here → DFPlayer RX
#define MAX_DIST 400

DistanceSensor distanceSensor(TRIG_PIN, ECHO_PIN, MAX_DIST);
AudioPlayer audioPlayer(DF_RX, DF_TX); // Single audio player for all sounds

uint8_t receiverMacAddress[] = {0xF8, 0xB3, 0xB7, 0x47, 0xF8, 0x7C};


// NOTE:
// Upload this code to the receiver ESP32 board (The car)!
//

//Right motor
int enableRightMotor=22; 
int rightMotorPin1=16;
int rightMotorPin2=17;
//Left motor
int enableLeftMotor=23;
int leftMotorPin1=18;
int leftMotorPin2=19;

#define MAX_MOTOR_SPEED 230

// Smoothing and sensitivity constants
#define SMOOTHING_FACTOR 0.3  // Lower = smoother, higher = more responsive
#define STEERING_SENSITIVITY 0.6  // Reduce steering sensitivity (0.5 = half sensitivity)
#define MIN_THROTTLE_THRESHOLD 15  // Minimum throttle to start moving
#define MIN_STEERING_THRESHOLD 10  // Minimum steering input to register

// Smoothing variables
float smoothedThrottle = 0;
float smoothedSteering = 0;
int lastRightMotorSpeed = 0;
int lastLeftMotorSpeed = 0;

// Drifting detection variables
#define DRIFT_SPEED_THRESHOLD 50    // Minimum speed to trigger drift sound
#define DRIFT_STEERING_THRESHOLD 10  // Minimum steering angle to trigger drift sound
#define DRIFT_COOLDOWN 2000         // Cooldown between drift sounds (ms)
unsigned long lastDriftTime = 0;
bool isDrifting = false;

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
  //Serial.println(inputData);
  
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

  // Exponential smoothing for throttle and steering
  smoothedThrottle = (SMOOTHING_FACTOR * throttle) + ((1 - SMOOTHING_FACTOR) * smoothedThrottle);
  smoothedSteering = (SMOOTHING_FACTOR * steering) + ((1 - SMOOTHING_FACTOR) * smoothedSteering);

  // Apply steering sensitivity reduction
  smoothedSteering *= STEERING_SENSITIVITY;

  // Deadband to prevent small fluctuations from causing movement
  if (abs(smoothedThrottle) < MIN_THROTTLE_THRESHOLD) {
    smoothedThrottle = 0;
  }
  if (abs(smoothedSteering) < MIN_STEERING_THRESHOLD) {
    smoothedSteering = 0;
  }

  int rightMotorSpeed, leftMotorSpeed;
  rightMotorSpeed =  abs(smoothedThrottle) - smoothedSteering;
  leftMotorSpeed =  abs(smoothedThrottle) + smoothedSteering;
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);

  rotateMotor(rightMotorSpeed * motorDirection, leftMotorSpeed * motorDirection);

  int avgSpeed = (abs(leftMotorSpeed) + abs(rightMotorSpeed)) / 2;

  // Check for drifting conditions
  unsigned long currentTime = millis();
  bool shouldDrift = (avgSpeed > DRIFT_SPEED_THRESHOLD) &&
                     (abs(smoothedSteering) > DRIFT_STEERING_THRESHOLD) &&
                     (currentTime - lastDriftTime > DRIFT_COOLDOWN);

  if (shouldDrift && !isDrifting) {
    Serial.println("Drift detected! Playing drifting sound...");
    audioPlayer.playDriftingSound();
    lastDriftTime = currentTime;
    isDrifting = true;
  } else if (!shouldDrift && isDrifting) {
    isDrifting = false;
  }

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

  // Initialize audio player
  Serial.println("Initializing audio player...");
  if (audioPlayer.begin()) {
    Serial.println("Playing startup sound...");
    audioPlayer.playStartingSound();
  }

  // Initialize distance sensor
  if (!distanceSensor.begin()) {
    Serial.println("Error initializing distance sensor");
    // Continue anyway - don't block the main functionality
  }

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
  // Update distance sensor for proximity detection
  distanceSensor.update();

  // Handle proximity alerts with audio
  static bool wasProximityAlert = false;
  bool isProximityAlert = distanceSensor.isProximityAlert();

  if (isProximityAlert && !wasProximityAlert) {
    // Start playing siren sound when proximity alert begins
    Serial.println("Reciever should play siren sound");
    audioPlayer.playSirenSound();
  } 

  wasProximityAlert = isProximityAlert;

  //Check Signal lost.
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT )
  {
    rotateMotor(0, 0);
  }
}
