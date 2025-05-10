#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu;

// Variables para calibración
float accel_angle_x, accel_angle_y;
float gyro_angle_x, gyro_angle_y;
float angle_x, angle_y;
unsigned long last_time;
float dt;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
    
    if (mpu.testConnection()) {
        Serial.println("MPU6050 conectado correctamente");
    } else {
        Serial.println("Error al conectar con MPU6050");
    }
    
    // Calcular ángulos iniciales del acelerómetro
    calculateInitialAngles();
    angle_x = accel_angle_x;
    angle_y = accel_angle_y;
    last_time = millis();
}

void calculateInitialAngles() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Convertir valores raw a g (9.8 m/s²)
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;
    
    // Calcular ángulos iniciales (en radianes)
    accel_angle_x = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g));
    accel_angle_y = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g));
    
    // Convertir a grados
    accel_angle_x = accel_angle_x * 180.0 / PI;
    accel_angle_y = accel_angle_y * 180.0 / PI;
}

void loop() {
    // Calcular tiempo transcurrido
    unsigned long current_time = millis();
    dt = (current_time - last_time) / 1000.0; // en segundos
    last_time = current_time;
    
    // Leer valores del sensor
    int16_t ax, ay, az; // Acelerómetro
    int16_t gx, gy, gz; // Giroscopio
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // 1. Calcular ángulos del acelerómetro
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;
    
    accel_angle_x = atan2(ay_g, sqrt(ax_g * ax_g + az_g * az_g)) * 180.0 / PI;
    accel_angle_y = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180.0 / PI;
    
    // 2. Calcular ángulos del giroscopio (integración)
    float gyro_rate_x = gx / 131.0; // °/s
    float gyro_rate_y = gy / 131.0;
    
    gyro_angle_x += gyro_rate_x * dt;
    gyro_angle_y += gyro_rate_y * dt;
    
    // 3. Fusión complementaria (95% giroscopio + 5% acelerómetro)
    float alpha = 0.95;
    angle_x = alpha * (angle_x + gyro_rate_x * dt) + (1.0 - alpha) * accel_angle_x;
    angle_y = alpha * (angle_y + gyro_rate_y * dt) + (1.0 - alpha) * accel_angle_y;
    
    // Mostrar resultados
    Serial.print("Roll (X): "); Serial.print(angle_x);
    Serial.print("°\tPitch (Y): "); Serial.print(angle_y);
    Serial.println("°");
    
    delay(200); // Pequeño retardo para estabilidad
}