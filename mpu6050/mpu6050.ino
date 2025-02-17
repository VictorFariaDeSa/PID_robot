// biblioteca para comunicação I2C
#include "Wire.h"       


//Endereço I2C do sensor, obtido atraves do dataSheet
const int MPU = 0x68;

//Criação das variaveis que receberão os valores medidos:
double accX, accY, accZ, gyrX, gyrY, gyrZ, temp, ang;
double dt, last_time;
const double kp = 5, ki = 0.001, kd = 0.1;
const double setpoint = 90;
double pid_output;
double previous = 0, integral = 0;

int pwm_pin_a=5, pwm_pin_b = 10, a1_pin=6, a2_pin=7,b1_pin = 9, b2_pin = 8;

float accel_factor, gyr_factor;

float kalman_gain = 0.75;

double kalman_1d(float kalman_gain, float prev_angle, float gyr, float accel_angle, float dt){

  double theta = kalman_gain*(prev_angle+gyr*dt) + (1-kalman_gain)*accel_angle;

  return theta;
  }



double calculateDegree(double x,double y){
  return atan2(y,x)*180/3.14 + 180;
}


void adjust_rotation_direction(double output) {
  bool forward = output > 0;

  digitalWrite(a1_pin, !forward);
  digitalWrite(a2_pin, forward);
  digitalWrite(b1_pin, forward);
  digitalWrite(b2_pin, !forward);
}


void apply_PID(double output){
  double calibration_tax = 60;
  int pwm_value = constrain(abs(output)+calibration_tax, 0, 255);
  analogWrite(pwm_pin_a,pwm_value);
  analogWrite(pwm_pin_b,pwm_value);
}

double readSensor(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();

  accX = accX/accel_factor;
  accY = accY/accel_factor;
  accZ = accZ/accel_factor;

  temp = Wire.read() << 8 | Wire.read();

  gyrX = Wire.read() << 8 | Wire.read();
  gyrY = Wire.read() << 8 | Wire.read();
  gyrZ = Wire.read() << 8 | Wire.read();

  gyrX = gyrX/gyr_factor;
  gyrY = gyrY/gyr_factor;
  gyrZ = gyrZ/gyr_factor;
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
  //Wire.write(0x00000000) +/- 250°/s
  //Wire.write(0x00001000) +/- 500°/s
  //Wire.write(0x00010000) +/- 1000°/s
  //Wire.write(0x00011000) +/- 2000°/s

  //factor = 131 --> 250°/s
  //factor = 65.6 --> 500°/s
  //factor = 32.8 --> 1000°/s
  //factor = 16.4 --> 2000°/s

  gyr_factor = 131;
  Wire.write(0x00000000);
  Wire.endTransmission();

  // Configuração do acelerômetro para fundo de escala desejado

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  //Wire.write(0b00000000) +/- 2g
  //Wire.write(0b00001000) +/- 4g
  //Wire.write(0b00010000) +/- 8g
  //Wire.write(0b00011000) +/- 16g

  //factor = 16384 --> 2g
  //factor = 8192 --> 4g
  //factor = 4096 --> 8g
  //factor = 2048 --> 16g

  accel_factor = 16384;
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
  pinMode(pwm_pin_a,OUTPUT);
  pinMode(a1_pin,OUTPUT);
  pinMode(a2_pin,OUTPUT);
  pinMode(pwm_pin_b,OUTPUT);
  pinMode(b1_pin,OUTPUT);
  pinMode(b2_pin,OUTPUT);
  last_time = micros();
  // TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  readSensor();
  ang = calculateDegree(accX,accY);
  delay(100);
  adjust_rotation_direction(1);
}

void loop(){
  unsigned long now = micros();
  dt = (now - last_time) / 1000000.0;
  last_time = now;
  readSensor();
  double acc_angle = calculateDegree(accX,accY);
  ang = kalman_1d(kalman_gain, ang, gyrZ, acc_angle, dt);
  double error = setpoint-ang;
  pid_output = PID(error);
  adjust_rotation_direction(pid_output);
  apply_PID(pid_output);
  Serial.print(ang);
  Serial.print(" ");  // Usa espaço para separar os valores
  Serial.println(acc_angle);
}




