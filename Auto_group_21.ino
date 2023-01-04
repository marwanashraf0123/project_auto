//include required libraries
#include<PID_v2.h>
#include<Servo.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

//define our variables
//our servo will begin from 90 degrees 

// imu related 

#define enA 11
#define in1 4
#define in2 5
 
 
double Setpoint ; // will be 0 value of desired position of imu
double Input ; // imu readings
double Output ; //servo angle
int ServoOutput, val1;
float mpu_z;

//PID parameters 
double Kp = 10 , Ki = 0.1  , Kd = 0.01;

// create PID instance
PID myPID(&Input , &Output , &Setpoint , Kp ,Ki , Kd , DIRECT);

//Intialize servo object 
Servo s1 ;

//intialize imu
MPU6050 mpu6050(Wire);

//prototype of function 
int imu_reading();





void setup() 
{

  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  //servo related
  
  s1.attach(3);
  s1.write(100);
  delay(1000);

  //PID related
  
  //define the setpoint 
  Setpoint = 0 ;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //set the output limits (-60,60)
  myPID.SetOutputLimits(35,155);
  //adjust PID values 
  myPID.SetTunings(Kp,Ki,Kd); 

  //imu related 
  
  Wire.begin();
  mpu6050.begin();
  //calibration 
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  //get the angle around z
  mpu_z = mpu6050.getAngleZ();

   digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  

}

void loop() 
{

  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  analogWrite(enA,120) ;
  
 // calculating angle around z
 imu_reading();

//read the value of imu and maping it to servo angles
Input = val1 ;

//PID calculation 
myPID.Compute();

//write the output as calculated by pid function
s1.write(ceil(Output));



//send data for observation 
Serial.print(Input);
Serial.print("  ");
Serial.print(Output);
Serial.print("  ");
Serial.print(Setpoint);
Serial.print("  ");
Serial.println(val1);
//delay(1000);

}

//imu reading function 
int imu_reading()
{
   mpu6050.update();

 int val = -(mpu_z-mpu6050.getAngleZ())-3; 

 if (val > 0)
 {
  val=val+6;
  
  }

 val1= val;

 return val1;
  
  
  }
