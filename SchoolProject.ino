#include <AccelStepper.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "DHT.h"
#include "arduino_secrets.h"

#define dhtHeadPin A1 // what pin our dht head sensor is conneted to
#define dhtFootPin 0  // what pin our 2nd dht foot sensor is connected to
#define dhtType DHT22    // DHT 22  (AM2302) what model we are using
#define MotorInterfaceType 8 // halfstep mode

DHT dhtHead(dhtHeadPin, dhtType); // dht sensor that are placed in head height
DHT dhtFeet(dhtFootPin, dhtType); // dht sensor that are placed in foot height

float headTemp = 0;
float headHum = 0;
float feetTemp = 0;

char ssid[] = SECRET_SSID; // wifi ssid
char pass[] = SECRET_PASS; // wifi password

int status = WL_IDLE_STATUS; // wifi status
char server[] = "192.168.1.71"; // web api ip-address
int serverPort = 5001; // api port
String queryString = "";
String apiPath = "/api/PostData/PostRoomData";

WiFiClient client; // wifi client for http request

//pins
const char soundSensorPin = A3; // input pin for soundSensor
const char motionSensorPin = A5; // input pin for motionSensor
const char lightSensorPin = A6; // input pin for lightSensor


//lights
const char postRequestLedPin = 1; // output pin for postRequestLed
const char soundSensorLedPin = 2; // output pin for whenever the soundSensor detects a sound
const char lightIndicatorLedPin = A4; // output pin to turn on when the motionSensor detects motion


//status
int motionSensorValue = 0;
int ledState = LOW;
bool sensor = true;
int lightSensorValue = 0;
bool curtainStatus = false;
String curtainValue = "";


// stepper motor
const int stepsPerRevolution = 2048; // amount of steps to go

const char stepperMotorIN1 = 4;
const char stepperMotorIN2 = A2;
const char stepperMotorIN3 = 3;
const char stepperMotorIN4 = 5;

AccelStepper stepperMotor(MotorInterfaceType, stepperMotorIN1,stepperMotorIN3 ,stepperMotorIN2 , stepperMotorIN4); // new instance of AccelStepper

//listener

void setup() {

  pinMode(motionSensorPin, INPUT); // input pin for motionSensor
  pinMode(soundSensorPin, INPUT); // input pin for soundSensor
  pinMode(lightSensorPin, OUTPUT); // output to show if the light is on or off
  pinMode(soundSensorLedPin, OUTPUT); // output to show if the soundSensor has detected sound
  pinMode(postRequestLedPin, OUTPUT); // output to show whenever we are sending a postRequest to the api
  Serial.begin(9600); // start serial with 9600 baud rate
  ConnectToWifi(); // run method to connect to wifi
  dhtHead.begin(); // start the dht head sensor
  dhtFeet.begin(); // start the dht foot sensor
  stepperMotor.setMaxSpeed(1000); // set max speed of stepper motor to be 1000 steps per minut
  stepperMotor.setAcceleration(500); // set the acceleration to 500steps
}

void loop() {
  
  if (client.connect(server, serverPort)) { // connect to the api with wificlient
    String roomName = ROOM_NAME;
    String soundStatus = GetSoundSensorValue();
    String curtainStatus = CurtainHandler();
    String lightStatus = GetMotionSensorStaus();
    HumAndTempReader();
    PostRoomDataToApi(roomName, headTemp, headHum, feetTemp, soundStatus, curtainStatus, lightStatus);
    delay(5000);
    BlinkPostLed();
  }
}

bool LightSensorIndicator(){ // get the lightSensor status
  lightSensorValue = analogRead (lightSensorPin); // read the lightSensorPin
  if( lightSensorValue <= 1000){ 
    return false;
  }
  else{
    return true;
  }
  delay(100);
}

String CurtainHandler(){

  bool lightSensorStatus = LightSensorIndicator();

  if (lightSensorStatus){ // if the lightsensor detects light

      RunStepperMotor(1); // roll down curtains
      curtainValue = "down";
  }
  else if (!lightSensorStatus){ // if the lightsensor dosn't detect light
    RunStepperMotor(0); // roll up curtains
      curtainValue ="up";
  }

  return curtainValue;
  
}

void RunStepperMotor(int rotation){

  if(rotation == 1 && curtainStatus == false){ // if the rotatin is set to roll down curtain and the curtains is rolled up
  stepperMotor.moveTo(8192); // move the stepper motor 8192 steps clockwise
  stepperMotor.runToPosition();
  curtainStatus = true; // set the curtainStatus to rolled down
  delay(200);
 
  }
  else if (rotation == 0 && curtainStatus == true){ // if the rotatin is set to roll up curtain and the curtains is rolled down

    stepperMotor.moveTo(0); // rotate 8192 steps counter clockwise
    // Run to position with set speed and acceleration:
    stepperMotor.runToPosition();
    curtainStatus = false; // set curtain status to rolled up
    delay(200);

  }
}

String GetSoundSensorValue(){

  int soundSensorValue = digitalRead(soundSensorPin); //get the soundSensorValue
  if (soundSensorValue == 1){ // if there is sound detected
    digitalWrite(soundSensorLedPin, HIGH); // set the led to on
    return "on";
  }
  else {
      digitalWrite(soundSensorLedPin, LOW); // set led to off
      return "off";
  }
}
 
void BlinkPostLed(){ // method to blink the 
  if (ledState == LOW){
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(postRequestLedPin,ledState);
    
    delay(100);
    if (ledState == HIGH) {
     digitalWrite(postRequestLedPin,LOW);
    }
    delay(200);
}

void HumAndTempReader(){

  headTemp = dhtHead.readTemperature();
  headHum = dhtHead.readHumidity();
  feetTemp = dhtFeet.readTemperature();
}

 String GetMotionSensorStaus(){ // check if the motionSensor detects any motion
  motionSensorValue = digitalRead(motionSensorPin);
  if(motionSensorValue == HIGH){
      digitalWrite(lightIndicatorLedPin,HIGH); // turn on the light
      return "on";
  }
  else{
      digitalWrite(lightIndicatorLedPin, LOW); // turn off the light
      return "off";         
  }
  delay(10);
}

void printWifiStatus() { // print the wifi connection information
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void ConnectToWifi(){ // connect to wifi
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to WiFi");
  printWifiStatus();
}

void PostRoomDataToApi(String roomName, float tempHead, float humHead, float tempFeet, String soundStatus, String curtainStatus, String lightStatus){

  queryString = String("?roomName=") + roomName + String("&tempHead=")+ String(tempHead) + String("&humHead=") + String(humHead);
  queryString = queryString + String("&tempFeet=") + String(tempFeet) + String("&soundStatus=") + soundStatus + String("&curtainStatus=") + curtainStatus + String("&lightStatus=") + lightStatus; 
  
  client.println("GET "+ apiPath + queryString + " HTTP/1.1");
  client.println("Host: "+String(server)+":" + String(serverPort));
  client.println("Connection: close");
  client.println();
}
