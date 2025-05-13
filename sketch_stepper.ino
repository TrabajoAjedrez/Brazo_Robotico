#include <Wire.h>
#include <AS5600.h>
#include "PIDController.hpp"

#define stepPin 3
#define dirPin 4

AS5600 encoder;

enum Mode {IDLE, FORWARD, RETURNING};
Mode mode = IDLE;

double initialAngle = 0.0;
double currentAngle = 0.0;
double previousAngle = 0.0;
double finalAngle = 0.0;
int vueltas = 0;
const double GEAR_RATIO = 5.0;

PID::PIDParameters<double> pidParams(2.5,0.4,0.04); //mejor hasta el momento: 2.5,0.4,0.04
PID::PIDController<double> pid(pidParams);

const int stepIntervalMicros = 50;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  Serial.begin(9600);
  Wire.begin();
  encoder.begin();

  if (!encoder.isConnected()) {
    Serial.println("¡Error! No se detecta el AS5600.");
    while (1);
  }

  previousAngle = encoder.readAngle() * 360.0 / 4096.0;
  pid.SetOutputLimits(-30, 30); 
  pid.TurnOn();

  Serial.println("Listo. Comandos: GO, STOP, BACK, RESET");
}

void loop() {
  // Leer comando Serial
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "GO" && mode == IDLE) {
      initialAngle = getRealAngle();
      mode = FORWARD;
      Serial.println("Girando hacia adelante...");
    } 
    else if (command == "STOP" && mode == FORWARD) {
      finalAngle = getRealAngle();
      mode = IDLE;
      Serial.print("Motor detenido. Ángulo final: ");
      Serial.println(finalAngle);
    } 
    else if (command == "BACK" && mode == IDLE) {
      mode = RETURNING;
      Serial.print("Regresando a ");
      Serial.print(initialAngle);
      Serial.println(" grados...");
    } 
    else if (command == "STOP" && mode == RETURNING) {
      finalAngle = getRealAngle();
      mode = IDLE;
      Serial.print("Motor detenido. Ángulo final: ");
      Serial.println(finalAngle);
    } 
    else if (command == "RESET") {
      mode = IDLE;
      Serial.println("Sistema reiniciado.");
    }

    while (Serial.available()) Serial.read(); // Limpiar buffer
  }

  // Leer ángulo actual
  currentAngle = getRealAngle();

  if (mode == FORWARD) {
    digitalWrite(dirPin, HIGH);  // sentido horario
    doStep();
  } 
 else if (mode == RETURNING) {
  pid.Setpoint = initialAngle;
  pid.Input = currentAngle;
  pid.Update();

  double error = pid.Setpoint - pid.Input;
  double output = pid.Output;

  Serial.print("Ángulo actual: ");
  Serial.print(currentAngle);
  Serial.print(" | Objetivo: ");
  Serial.print(initialAngle);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | Salida PID: ");
  Serial.println(output);

  if (abs(error) < 0.5) {
    mode = IDLE;
    Serial.println("¡Regresó al punto inicial!");
  } else {
    // Dirección correcta basada en el signo del error
    if (output > 0) {
      digitalWrite(dirPin, HIGH);  // sentido horario
    } else {
      digitalWrite(dirPin, LOW);   // sentido antihorario
    }

    // Pasos proporcionales a la magnitud del error
  int steps = abs((int)output); 
    if (steps == 0 && abs(output) > 0.5) {
      steps = 1;
    }

    for (int i = 0; i < steps; i++) {
      doStep();
    }
  }
}

  delay(20); // Tasa de muestreo
}

// Hace un paso
void doStep() {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepIntervalMicros);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepIntervalMicros);
}

// Calcula el ángulo absoluto considerando vueltas y engranaje
double getRealAngle() {
  double rawAngle = encoder.readAngle() * 360.0 / 4096.0;
  double delta = rawAngle - previousAngle;
  if (delta > 180.0) vueltas--;
  else if (delta < -180.0) vueltas++;
  previousAngle = rawAngle;
  double absoluteAngle = vueltas * 360.0 + rawAngle;
  return absoluteAngle / GEAR_RATIO;
}