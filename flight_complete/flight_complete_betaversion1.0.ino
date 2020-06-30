/* Polat Space Systems

    22.06.20

*/

//Libraries


#include <Servo.h>
#include<Wire.h>
#include <MPU6050_tockn.h>
#include <AFMotor.h>
#include <I2Cdev.h> 



MPU6050 mpu6050(Wire);



const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyroY,GyroZ;
double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, previouslog, OutX, OutY, OutZ, OreX, OreY, OreZ;
double PreviousGyroX, PreviousGyroY, PreviousGyroZ, IntGyroX, IntGyroY, IntGyroZ, DifferenceGyroX, DifferenceGyroY, DifferenceGyroZ, matrix1, matrix2, matrix3;
double matrix4, matrix5, matrix6, matrix7, matrix8, matrix9, Ax, Ay;

//Upright Angle of the Flight Computer
int desired_angleX = 0;


//Offsets for tuning 
int servoX_offset = 100;


//Position of servos through the startup function
int servoXstart = 80;

//The amount the servo moves by in the startup function
int servo_start_offset = 20;

//Ratio between servo gear and tvc mount
float servo_gear_ratio = 5.8;

double OrientationX = 0;
double OrientationY = 0;
double OrientationZ = 1;
double accAngleX;
double accAngleY;
double yaw;
double GyroX;
double gyroAngleX;
double gyroAngleY;
double pitch;



Servo servoX;

AF_DCMotor motor1(1); // create motor #1
AF_DCMotor motor2(2); // create motor #2
AF_DCMotor motor3(3); // create motor #1
AF_DCMotor motor4(4); // create motor #2

int buzzer = 3;

double dt, currentTime, previousTime;



//"P" Constants-Orantısal Değişken
float pidX_p = 0;
float pidY_p = 0;

//"I" Constants-Integral Değişken
float pidY_i = 0;
float pidX_i = 0;

//"D" Constants-Türevsel Değişken
float pidX_d = 0;
float pidY_d = 0;


int pos;

//PID Gains
double kp = 0.2;
double ki = 0.0;
double kd = 0.05;

int state = 0;




void setup(){
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  gyrocalibrate();
  
  servoX.attach(10);

  motor1.setSpeed(255);     
  motor2.setSpeed(255);     
  motor3.setSpeed(255);    
  motor4.setSpeed(255);    
  
  
 
  pinMode(buzzer, OUTPUT);

  
 
}
void loop() {
  mpu6050.update();
  //Defining Time Variables-Zaman değişkeni tanımlanması
  previousTime = currentTime;        
  currentTime = millis();            
  dt = (currentTime - previousTime) / 1000; 
  launchdetect();
  rotationmatrices();
  delay(4100);
  parachute();

}
void parachute (){
  
    motor1.run(FORWARD);      
    motor2.run(BACKWARD);
    delay(3000);
   

    motor1.run(RELEASE);
    motor2.run(RELEASE);
}
void gyrocalibrate () {
  mpu6050.calcGyroOffsets(true); 
}


void rotationmatrices () {
  mpu6050.update();
  PreviousGyroX = IntGyroX;
  PreviousGyroY = IntGyroY;
  PreviousGyroZ = IntGyroZ;

  IntGyroX = mpu6050.getGyroAngleX() * (PI / 180);
  IntGyroY = mpu6050.getGyroAngleY() * (PI / 180);
  IntGyroZ = mpu6050.getGyroAngleZ() * (PI / 180);

  DifferenceGyroX = (IntGyroX - PreviousGyroX);
  DifferenceGyroY = (IntGyroY - PreviousGyroY);
  DifferenceGyroZ = (IntGyroZ - PreviousGyroZ);

  OreX = OrientationX;
  OreY = OrientationY;
  OreZ = OrientationZ;

 //X Matrices
  matrix1 = (cos(DifferenceGyroZ) * cos(DifferenceGyroY));
  matrix2 = (((sin(DifferenceGyroZ) * -1) * cos(DifferenceGyroX) + (cos(DifferenceGyroZ)) * sin(DifferenceGyroY) * sin(DifferenceGyroX)));
  matrix3 = ((sin(DifferenceGyroZ) * sin(DifferenceGyroX) + (cos(DifferenceGyroZ)) * sin(DifferenceGyroY) * cos(DifferenceGyroX)));
 
 //Y Matrices
  matrix4 = sin(DifferenceGyroZ) * cos(DifferenceGyroY);
  matrix5 = ((cos(DifferenceGyroZ) * cos(DifferenceGyroX) + (sin(DifferenceGyroZ)) * sin(DifferenceGyroY) * sin(DifferenceGyroX)));
  matrix6 = (((cos(DifferenceGyroZ) * -1) * sin(DifferenceGyroX) + (sin(DifferenceGyroZ)) * sin(DifferenceGyroY) * cos(DifferenceGyroX)));
 
 //Z Matrices
  matrix7 = (sin(DifferenceGyroY)) * -1;
  matrix8 = cos(DifferenceGyroY) * sin(DifferenceGyroX);
  matrix9 = cos(DifferenceGyroY) * cos(DifferenceGyroX);

 
 OrientationX = ((OreX * matrix1)) + ((OreY * matrix2)) + ((OreZ * matrix3));
 OrientationY = ((OreX * matrix4)) + ((OreY * matrix5)) + ((OreZ * matrix6));
 OrientationZ = ((OreX * matrix7)) + ((OreY * matrix8)) + ((OreZ * matrix9));

 
Serial.println(Ax);


 
OutX = OrientationX * 60;


Ax = asin(OrientationX) * (-180 / PI);



pidcompute();

  
}


void servowrite() {

 servoX.write(pwmX);


}
void pidcompute () {

previous_errorX = errorX;


errorX = Ax - desired_angleX;


//Defining "P" 
pidX_p = kp*errorX;


//Defining "D"
pidX_d = kd*((errorX - previous_errorX)/dt);

//Defining "I"
pidX_i = ki * (pidX_i + errorX * dt);


PIDX = pidX_p + pidX_i + pidX_d;



pwmX = ((PIDX * servo_gear_ratio) + servoX_offset);

servowrite();


}

void launchdetect () {
  if (mpu6050.getAccZ() > 1) {
  state = 1;
 }
  if (state == 1) {
  
  
 }
}

void abortstart () {
 
  if (OrientationX > 40 || OrientationY > 40) {
      digitalWrite(buzzer, HIGH);
  
}
    else if (OrientationX < 40 || OrientationY < 40) {
    
}

}
