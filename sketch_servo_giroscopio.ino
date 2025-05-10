#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Servo.h>

// Definición de pines
const int SERVO1_PIN = 9;  

// Objetos
MPU6050 mpu;
Servo servo1;

// Variables de control
float targetAngle = 40;    // Ángulo objetivo
float currentAngle = 0;   // Ángulo actual medido
float prevError = 0;
float integral = 0; 

// Parámetros PID
const float Kp = 2.0;    // Ganancia proporcional
const float Ki = 0.05;   // Ganancia integral
const float Kd = 1.0;    // Ganancia derivativa

// Variables giroscopio
float accel_angle_x, accel_angle_y;
float gyro_angle_x = 0;
unsigned long lastTime = 0;
float dt;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Inicializar MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Error en MPU6050");
    while(1);
  }
  
  // Configurar servo
  servo1.attach(SERVO1_PIN, 1000, 2000); // Ajusta los valores mínimo/máximo si es necesario
  
  // Calibración inicial
  calculateInitialAngles();
  currentAngle = accel_angle_x;
  lastTime = millis();
  
  Serial.println("Sistema listo");
}

void calculateInitialAngles() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  
  accel_angle_x = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
}

void loop() {
  // Calcular tiempo transcurrido
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // en segundos
  lastTime = currentTime;
  
  // Leer sensores
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Calcular ángulo del acelerómetro
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  
  accel_angle_x = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
  
  // Calcular ángulo del giroscopio (integración)
  float gyro_rate_x = gx / 131.0; // °/s
  gyro_angle_x += gyro_rate_x * dt;
  
  // Filtro complementario
  float alpha = 0.96;
  currentAngle = alpha * (currentAngle + gyro_rate_x * dt) + (1.0 - alpha) * accel_angle_x;
  
  // Control PID
  float error = targetAngle - currentAngle;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  prevError = error;
  
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  // Mapear salida PID a valores de servo (0-180°)
  int servoAngle = constrain(map(output, -90, 90, 0, 180), 0, 180);
  servo1.write(servoAngle);
  
  // Monitorización (opcional)
  Serial.print("Target: "); Serial.print(targetAngle);
  Serial.print("° Current: "); Serial.print(currentAngle);
  Serial.print("° Output: "); Serial.println(servoAngle);
  
  delay(20); // Pequeña pausa para estabilidad
}