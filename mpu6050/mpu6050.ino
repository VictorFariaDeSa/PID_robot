// biblioteca para comunicação I2C
#include "Wire.h"       


//Endereço I2C do sensor, obtido atraves do dataSheet
const int MPU = 0x68;

//Criação das variaveis que receberão os valores medidos:
double accX, accY, accZ, gyrX, gyrY, gyrZ, temp, ang;
double dt, last_time;
const double kp = 0.8, ki = 0.1, kd = 0.001;
const double setpoint = 0;
double pid_output;
double previous = 0, integral = 0;

int pwm_pin=3,a1_pin=4,a2_pin=5;


double calculateDegree(double x, double z){
  return atan(x/z)*180/3.14;
}

double readAngle(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();

  temp = Wire.read() << 8 | Wire.read();

  gyrX = Wire.read() << 8 | Wire.read();
  gyrY = Wire.read() << 8 | Wire.read();
  gyrZ = Wire.read() << 8 | Wire.read();

  return calculateDegree(accX,accY);
}

void mpu6050_configuration(){
    // Inicialização do MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Configuração do giroscópio para fundo de escala desejado

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00011000);
  Wire.endTransmission();

  // Configuração do acelerômetro para fundo de escala desejado

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

double PID(double error){
  double proportional = error;
  double derivative = (error - previous)/dt;
  integral += error*dt;
  previous = error;
  double output = (kp*proportional) + (ki*integral) + (kd*derivative);
  return output;
}

void setup(){
  Serial.begin(9600);
  mpu6050_configuration();
  pinMode(pwm_pin,OUTPUT);
  pinMode(a1_pin,OUTPUT);
  pinMode(a2_pin,OUTPUT);
  digitalWrite(a1_pin,LOW);
  digitalWrite(a2_pin,HIGH);
  delay(100);
}

void loop(){
  double now = millis();
  dt = (now-last_time)/1000;
  last_time = now;
  ang = readAngle();

  double error = setpoint-ang;
  pid_output = PID(error);
  
  analogWrite(pwm_pin,100);

  Serial.println(pid_output);
  delay(300);
}


