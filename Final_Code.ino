// --- declaring libraries ---
#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>
#include <NewPing.h>

///IMU variables ---
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t ax, ay, az, gx, gy, gz;
unsigned long timer = 0;
unsigned long prevTime = 0;
unsigned long interval = 100; // Interval set to 100 ms
float yaw = 0;
float gz_bias = 0;
float vx = 0, vy = 0, vz = 0;

//servo settings ---
Servo myservo;
const int Servopin = 9; //PB1
int front_angle = 70; //initial position of the fan
int servoAngle = front_angle;
const int turn_left =0 ;
const int turn_right =140;

// Fan settings ---
const int fan1Pin = 5; // PD5 trust
const int fan2Pin = 6; // PD6 lift
int fanSpeedtrust = 255; // (0-255)
int fanSpeedlift = 255; //(0-255)

// Ultrasonic sensor setting ---
  //(Sensor 1)-
const int trigPin = 11;    //PB3
const int echoPin = 2;     //PD2

  //(Sensor 2)-
const int trigPin2 = 13; // PB5
const int echoPin2 = 3;  // PD3

//creating the Newping object to get accurate measuremnts
const int MAX_DISTANCE =200;
NewPing sonar1(trigPin, echoPin, MAX_DISTANCE);
NewPing sonar2(trigPin2, echoPin2, MAX_DISTANCE);


// Angular Control test setting ---
const int slow_down_distance = 70; //from max fan to lower fan
const int Stop_distance =25;
const float yawOffset = 180.0; // 90 degrees clockwise rotation
float speed=0;
bool turnLeft=false;
bool turnRight=false;
unsigned long tempo;
unsigned long tempo2;
unsigned long tempo3=0;
bool bug=true;
bool secondturnL =false;
bool secondturnR =false;



void setup() {

  Wire.begin();
  Wire.setWireTimeout(3000); // Set I2C timeout to 3ms
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up MPU-6050
  Wire.endTransmission(true);

  Serial.begin(9600);

  myservo.attach(Servopin); // Attach servo to proper pin
  gz_bias = calculate_gz_bias(); // Calculate error for the Z-axis gyroscope

  // Set up the fan control pins as outputs ---
  pinMode(fan1Pin, OUTPUT);
  pinMode(fan2Pin, OUTPUT);

  // Set initial fan speeds ---
  analogWrite(fan1Pin, fanSpeedtrust);
  analogWrite(fan2Pin, fanSpeedlift);


  // --- not of any use since we are using Newping library ---
  
  // // Set up ultrasonic sensor pins
  // pinMode(trigPin, OUTPUT);
  // pinMode(echoPin, INPUT);

  // // Set up ultrasonic sensor 2 pins
  // pinMode(trigPin2, OUTPUT);
  // pinMode(echoPin2, INPUT);
  
  // --- variable to control to control navigation timing ---
  tempo=millis();
  tempo2=millis();

}
float ax_prev = 0, ay_prev = 0, az_prev = 0;

void loop() {

  bug=true;//debuging tool 

  // Find yaw ---
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);  // Starting with register 0x43 (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);  // Request 6 registers

  // Check if I2C timeout occurred ---
  
  if (Wire.getWireTimeoutFlag()) {
    Wire.end();
    delay(10);
    Wire.begin();
    return; // Skip this iteration of the loop
  }


  gx = Wire.read() << 8 | Wire.read();   // GYRO_XOUT
  gy = Wire.read() << 8 | Wire.read();   // GYRO_YOUT
  gz = Wire.read() << 8 | Wire.read();   // GYRO_ZOUT
  float gz_dps = (gz / 131.0) - gz_bias;
  float dt = (millis() - timer) / 1000.0;
  yaw += gz_dps * dt;
  timer = millis();

  // Get distance from the 2 ultrasonic sensors ---
  int distance = getDistance(sonar1);
  int distance2 = getDistance(sonar2);

  if (distance >= 0 && distance2 >= 0 && millis()%10==0 ) {

//     // Print result for yaw, distance, and velocity
//    Serial.print("fan 1 : ");
//    Serial.print(fanSpeedtrust);
//    Serial.print("| fan 2 : ");
//    Serial.print(fanSpeedlift);
//    Serial.print("| Yaw: ");
//    Serial.print(yaw);
//    Serial.print("| Distance: ");
//    Serial.print(distance);
//    Serial.print("| Distance 2 : ");
//    Serial.println(distance2);


    if(abs(yaw)<20 && distance < 45 && distance != 0){ // distance < 42

      if(distance2<50 && distance2!=0 ){
        //right
        Serial.println("| >> |");

        if(turnRight==false && tempo+3000<millis()){
          yaw-=yawOffset;
          tempo=millis();
        
        }

        turnRight=true;

          // --- Speed control to navigate turns ---
          
//        if(tempo+1000<millis() && turnRight==true && distance<10 && distance !=0)  {
//
//            fanSpeedlift = 255; 
//            fanSpeedtrust = 255;
//          
//        }else{
//            fanSpeedlift = 220; 
//            fanSpeedtrust = 220;
//          }             

        //need it too know when to turn again ---

        bug=false;

        servoAngle = front_angle + yaw;
      }

      if( (distance2>50 || distance2==0) ){
        //Left

        if(turnLeft==false && tempo+3000<millis()){//if yaw is smaller than 5 degree
          yaw+=yawOffset;
          tempo=millis();
       
        }



        turnLeft=true;
        
         // --- Speed control to navigate turns ---

//        if(tempo+1000<millis() && turnLeft==true && distance<10 && distance !=0 )  {
//          
//            fanSpeedlift = 255; 
//            fanSpeedtrust = 255;
//           
//        }else{
//            fanSpeedlift = 220; 
//            fanSpeedtrust = 220;
//          }  

        bug=false;

       servoAngle = front_angle + yaw;
      }

    }
     
    if((abs(yaw)<15 && tempo+3000>millis()) || abs(yaw)<15 || (abs(yaw)<75 && (turnLeft==true || turnRight==true))  ){
      //go straight
      Serial.println("| ^ |");

      turnLeft=false;
      turnRight=false;
      bug=false;

      servoAngle = front_angle + yaw;
      fanSpeedlift = 255; 
      fanSpeedtrust = 255;
      
    }

      // --- control for case scenario where the hovercraft get stuck in an corner ---

//     if ((abs(yaw)>30 && abs(yaw)<50 && distance<10 && distance2<10 && distance!=0 && distance2!=0)){
//      
//      fanSpeedlift = 0; 
//      fanSpeedtrust = 0; 
//      tempo3=millis();
//
//    }
    

    // --- reset the direction to being straight ---
    if(turnLeft==false && turnRight==false){
      servoAngle = front_angle + yaw;      
    }


    // --- set the angle to the next navigartion point ---
    servoAngle = constrain(servoAngle, 0, 150);
    myservo.write(servoAngle);

    // --- setup fans speed ---
    if(tempo3 == 0 || tempo3+2000 < millis() || tempo+3000<millis()){

    fanSpeedlift=255 ; 
    fanSpeedtrust=255; 
      
    }

    analogWrite(fan1Pin, fanSpeedtrust);
    analogWrite(fan2Pin, fanSpeedlift);
  }


}

// ------ function to fix the IMU drifting ------

float calculate_gz_bias() {
  int samples = 2000; // Increase the number of samples to improve accuracy
  float sum = 0;

  // take a sample of the drifting for 2 seconds and set it up as the IMU bias 
  
  for (int i = 0; i < 10; i++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x47);  // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 2, true);
    gz = Wire.read() << 8 | Wire.read();   // GYRO_ZOUT
    delay(10);
  }

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x47);  // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 2, true);
    gz = Wire.read() << 8 | Wire.read();   // GYRO_ZOUT

    sum += gz;
    delay(2); 
  }

  return sum / samples / 131.0;
}

// ------ Funtion to measure the distance of the Utrasonic ------
unsigned int getDistance(NewPing& sonar) {
  unsigned int distance = sonar.ping_cm(); // Get the distance in centimeters
  return distance;
}
