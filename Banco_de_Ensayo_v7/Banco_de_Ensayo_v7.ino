#include "NBHX711.h"
#include "LiquidCrystal_I2C.h"
#include "Servo.h"

// Información
int led1 = 30;
int led2 = 34;

// Controles
int pot_motor = A7;
int pot_paso = A8;

// Salidas
Servo motor;
Servo paso;

// Entradas
int encoder = 0; // interrupt 0, pin 2
int contador = 0;
unsigned long timer_segundo = 0;
NBHX711 celda1(A0, A1, 20);
int non1 = 0;
int off1 = 0;
int off2 = 0;
NBHX711 celda2(A3, A4, 20);
int rpm_sensor = 7;

void setup() {
    Serial.begin(9600);
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);

    motor.attach(9);
    motor.write(50);

    paso.attach(5);
    paso.write(84);

    pinMode(rpm_sensor, INPUT);
    attachInterrupt(encoder, []() -> void {
        contador++;
        }, FALLING);

    celda1.begin();
    celda2.begin();

    // calibracion
    while (non1 < 200) {
        non1 = non1 + 1;
        celda1.update();
        off1 = -celda1.getValue(10), 0;
        celda2.update();
        off2 = celda2.getValue(10), 0;
    }
}

int lectura_rpm_sensor = 0;
int lectura_encoder = 0;
int lectura_empuje = 0;
int lectura_torque = 0;
void loop() {
    // Servos
    motor.write(map(analogRead(pot_motor), 0, 1023, 87, 169));
    paso.write(map(analogRead(pot_paso), 0, 1023, 62, 116));

    {
        int rpm1, rpm2, rpm3, rpm4, rpm5, rpm6, rpm7;
        int rpm = pulseIn(rpm_sensor, HIGH);
        int n = 1;
        if (n == 1) {
            rpm1 = pow(24604025 / rpm, 0.8071) - 203;
        } else if (n == 2) {
            rpm2 = pow(24604025 / rpm, 0.8071) - 203;
        } else if (n == 3) {
            rpm3 = pow(24604025 / rpm, 0.8071) - 203;
        } else if (n == 4) {
            rpm4 = pow(24604025 / rpm, 0.8071) - 203;
        } else if (n == 5) {
            rpm5 = pow(24604025 / rpm, 0.8071) - 203;
        } else if (n == 6) {
            rpm6 = pow(24604025 / rpm, 0.8071) - 203;
        } else if (n == 7) {
            rpm7 = pow(24604025 / rpm, 0.8071) - 203;
            n = 0;
        }
        lectura_rpm_sensor = round((rpm1 + rpm2 + rpm3 + rpm4 + rpm5 + rpm6 + rpm7) / 7 / 60);
    }
    if (millis() > timer_segundo + 1000) {
        lectura_encoder = contador * 60 / 20;
        timer_segundo = millis();
    }
    celda1.update();
    lectura_empuje = (-celda1.getValue(10) - off1) / 1242.4; // gramos fuerza
    celda2.update();
    lectura_torque = ((celda2.getValue(10) - off2) / 384.7) * 16.7 / 1000; // kg mm

    // print
    Serial.print("\n LECTURAS ");
    Serial.print(millis());
    Serial.print("\nrpm malo: ");
    Serial.print(lectura_rpm_sensor);
    Serial.print("\nrpm weno: ");
    Serial.print(lectura_encoder);
    Serial.print("\n gramos fuerza: ");
    Serial.print(lectura_empuje);
    Serial.print("\n kg milimetro: ");
    Serial.print(lectura_torque);
    Serial.print("\n CONTROLES ");
    Serial.print("\n potencia: ");
    Serial.print(motor.read());
    Serial.print("\n potencia: ");
    Serial.print(motor.read());
    Serial.print("\n paso: ");
    Serial.print(paso.read());
}