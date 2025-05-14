#include <Servo.h>
#include <math.h>
//codigo que pasa recibe un punto 2D y dice a los servos como colocarse por cinematica incersa
// Longitudes de los eslabones (en mm)
const float l1 = 169.46;   // distancia del centro de la base al primer eslabon
const float l2 = 140.0;    // primer brazo
const float l3 = 168.816;  // segundo brazo

// Ángulo de inclinación de la base (en grados)
const float a_deg = 72.22;

// Servos
Servo servo1;  // q1 (hombro)
Servo servo2;  // q2 (codo)

void setup() {
  Serial.begin(9600);
  servo1.attach(9);  // pin del servo 1
  servo2.attach(10); // pin del servo 2

  // Ejemplo: mover a un punto x, z
  float x = 200;
  float z = 150;

  moverA(x, z);
}

void loop() {
  // Puedes poner aquí comandos para mover el brazo en tiempo real
}

// --- Función para mover al punto deseado ---
void moverA(float x, float z) {
  // Convertir ángulo de base a radianes
  float a_rad = radians(a_deg);

  // Coordenadas relativas quitando la base inclinada
  float x_ = x - l1 * cos(a_rad); //seno y coseno funcionan en radianes
  float z_ = z - l1 * sin(a_rad);

  // Distancia al objetivo al cuadrado
  float r2 = x_ * x_ + z_ * z_;

  // Calcular cos(q2)
  float cos_q2 = (r2 - (l2 * l2) - (l3 * l3)) / (2 * l2 * l3);

  // Verificar si es alcanzable
  if (abs(cos_q2) > 1) {
    Serial.println("Punto fuera de alcance.");
    return;
  }
  // Calcular q2 (negativo porque el codo gira hacia atrás)
  float q2_rad = acos(cos_q2);

  // Calcular q1
  float phi = atan2(z_, x_); //arctan(z_/x_)
  float k1 = l2 + l3 * cos(q2_rad);
  float k2 = l3 * sin(q2_rad);
  float q1_rad = phi + atan2(k2, k1);

  // Convertir a grados
  float q1_deg = degrees(q1_rad);
  float q2_deg = degrees(q2_rad);

  // Mostrar ángulos
  Serial.print("q1 = "); Serial.print(q1_deg); Serial.print("°, ");
  Serial.print("q2 = "); Serial.print(q2_deg); Serial.println("°");

  // Esto hay que configurarlo segun funcionen los servos, cesitamos saber donde está el 0 de los servos
  //El angulo es 0 cuando el eslabon esta alienado con el anterior (siendo el primero los 72º con el suelo) hay que ajustar
  int servo1_angle = constrain(q1_deg+45, -45, 135);//esto devuelve el valor si esta entre 0 y 180
  int servo2_angle = constrain(q2_deg , 0, 180); 

  // Enviar a servos
  servo1.write(servo1_angle);
  servo2.write(servo2_angle);
}
