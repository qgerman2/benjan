/************************************************************************
 Bloque escritura/Notas para operador:
 a) SIEMPRE revisar la calibracion de la helice-servo despues de cambiar o reinstalar el sistema helice paso variable
    a.- apretar la helice con el conector cuando helice y servo esten paralelos (y perpendicular al eje)
    b.- calibrar en modo manual los nuevos valores, solo se debe imprimir "Serial.print(ang_servo)"
      dentro de la linea de impresion (lineas de codigo importantes: 112, 287, 413)
 b) Usar los separadores para apilar la informacion y facilitar la revision del codigo:
      - acercar el mouse a la linea 1 de codigo y notaras las ">", presionalas para minimizar la informacion
 c) Indice del codigo:
    1.- Datos iniciales: librerias, constantes, deficiones, etc
    2.- VOID SETUP() : inicio del programa + calibracion inicial
    3.- VOID LOOP() : el ciclo para la toma de datos y operacion manual
    4.- VOID CAMBIO() : codigo para iniciar el cambio de modo
    5.- VOIDs PID() : PID para empuje y PID para RPS, cada uno por separado
    6.- // Codigo tunel de viento - Comentado //: solo esta como extra para referencia, no tiene setup ni constantes definidas
      Si se quiere usar, mejor revisar el codigo original, requiere copiar las constantes y añadir un pinmode en setup
 d) En caso de requerir añadir mucho codigo, lo mejor sera crear un nuevo "void XXXX()", asi se puede trabajar
    independiente del resto y solo debe ser llamado dentro de LOOP()


 Datos extras:
 2lb = 907.2 g, masa calibracion del LTA
 Cables de la HX711: verde amarillo rojo para celda EMPUJE; blanco, gris y negros para celda TORQUE
 orden pines placa HX711 : rojo E+, negro E-, Verde A+, Blanco A-

 VERSION 7.0 = CONTROL EMPUJE-PASO (ON)   |    CONTROL RPS-POTENCIA (ON)
 **************************************************************************/







 //======================================================================//
 //                        BANCO DE ENSAYOS V7.0                         //
 //======================================================================//

#include "NBHX711.h"
#include "LiquidCrystal_I2C.h"
#include "Servo.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);
Servo paso_v;
Servo motor;
NBHX711 celda1(A0, A1, 20);
NBHX711 celda2(A3, A4, 20);

// Hardware
int const pot_motor = A7;       // Potenciometro
int const pot_pasov = A8;       // Potenciometro
int const lector = 7;           // RPS
int const led1 = 30;            // LED Rojo
int const led2 = 34;            // LED Azul

// Recoleccion de data
int pot_m_val, pot_pv_val, ang_servo, angulo, potencia;
float torque, empuje, offset1, offset2, off1, off2;

// PID1 : Empuje - Paso
double y1_ref, y1, error11, error12, error13, U10, U11, kp1 = 1.9, ki1 = 1.53, kd1 = 0;

// PID2 : RPS - Potencia
double y2_ref, y2, error21, error22, error23, U20, U21, kp2 = 1.2, ki2 = 1.5, kd2 = 0;

// Promedio RPM
int n = 0;
float rpm, rpm1, rpm2, rpm3, rpm4, rpm5, rpm6, rpm7, rpm8, rpm9, rpm10;
// Promedio EMP y TRQ
int m = 0, l = 0;
float emp1, emp2, emp3, emp4, trq1, trq2, trq3, trq4;

// tiempo
float tiempo, prev_tiempo;
float ts = 0.1, t_ref;
int mins, segs;

// print de datos finales y calibrados    
float data_rps, data_empuje, data_torque, data_angulo;

int modo = 1;                    //Al tomar valor 1, se activa el modo manual el 100% de las veces
int non, non1, non2;             //variable vacia para almacenar valores temporales o cambiantes sin relevancia para los calculos

void setup() {

    Serial.begin(9600);                     //inicio de la comunicacion serial (PC)
    Serial.println("Inicia en modo manual {1} de forma forzada, asi se evitan problemas.");
    Serial.println("El cambio de modo o una interrupcion del PID se puede dar con presinar tecla,");
    Serial.println("pero para MANTENER el modo PID/automatico solo se lograra presionando el nro '2'.");

    pinMode(led1, OUTPUT);                  //inicio de LEDs
    pinMode(led2, OUTPUT);
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);

    lcd.init();                          //inicio de pantalla LCD (i2c)
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Iniciando.");
    delay(2000);

    motor.attach(9);                   //inicio de la ESC del motor
    motor.write(50);

    paso_v.attach(5);                  //inicio del Servo de la helice
    paso_v.write(70);
    delay(500);
    paso_v.write(84);                    //angulo pala 0° = 84° servo

    pinMode(lector, INPUT);             //iniciando el lector de RPM

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Iniciando..");
    delay(2000);

    celda1.begin();                     //inciando las celdas de carga
    celda2.begin();
    while (non1 < 200) {                 // calibracion inicial, se utiliza el ultimo valor medido como referencia, promediar podria llevar a errores
        non1 = non1 + 1;

        celda1.update();
        off1 = -celda1.getValue(10), 0;
        celda2.update();
        off2 = celda2.getValue(10), 0;
        Serial.print("offset: ");
        Serial.print(non1);
        Serial.print("|");
        Serial.print(off1 / 1242.4);
        Serial.print("|");
        Serial.println(off2 / 384.7 * 16.7 / 1000);
    }
    Serial.print(off1 / 1242.4);                    // offset en gramos, empuje
    Serial.print("|");
    Serial.println(off2 / 384.7 * 16.7 / 1000);         // offset en kg-mm, torque
    Serial.println("---------------");
    delay(500);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Iniciando...");
    delay(1000);                                  // todos los delay estan para dar tiempo al operador a reaccionar al inicio del programa
}

void loop() {
    /*              Recopilacion de datos sin procesar                  */
      // Tiempo
    tiempo = millis();
    // Revoluciones
    rpm = pulseIn(lector, HIGH);
    // Empuje
    celda1.update();
    empuje = -celda1.getValue(10), 0;
    // Torque
    celda2.update();
    torque = celda2.getValue(10), 0;

    /*    Reduciendo ruido de las RPS, prom 7 valores     */
    n = n + 1;
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

    /*                 Calibracion de datos                */
    data_rps = round((rpm1 + rpm2 + rpm3 + rpm4 + rpm5 + rpm6 + rpm7) / 7 / 60);            //unidad: RPS
    data_empuje = (empuje - off1) / 1242.4;                                     //unidad: gf    (gramos-fuerza)
    data_torque = ((torque - off2) / 384.7) * 16.7 / 1000;                          //unidad: kg-mm
    ang_servo = paso_v.read();
    data_angulo = round(1.1 * ang_servo - 92);                                  //unidad: angulo helice (beta)

    y1 = data_empuje;
    y2 = data_rps;
    /*              Impresion de datos (via serial 9600)          */

    Serial.print(tiempo);          // milisegundos
    Serial.print("|");
    Serial.print(data_rps);        // RPS medidas
    Serial.print("|");
    Serial.print(data_empuje);     // Empuje medido
    Serial.print("|");
    Serial.print(data_angulo);     // angulo entregado manualmente
    Serial.print("|");
    Serial.print(potencia);        // potencia entregada manualmente

    Serial.print("  |  ");

    Serial.print(y1_ref);
    Serial.print("|");
    Serial.print(y2_ref);
    Serial.print("|");
    Serial.print(y1);
    Serial.print("|");
    Serial.print(y2);
    Serial.print("|");
    Serial.print(error11);
    Serial.print("|");
    Serial.print(error21);
    Serial.print("|");
    Serial.print(U10);
    Serial.print("|");
    Serial.print(U20);
    Serial.println();

    /******************                          Imprime en pantalla Serial Arduino o RealTerm (9600)                                 *********************** /

    tiempo (milis) | RPS | empuje (gf) | angulo servo |  potencia  |  Empuje obj | RPS obj  | Empuje | RPS | error Emp | error RPS | PID_ang/emp_0 | PID_ESC/RPS_0

    *******                  datos modo manual                     |                                         datos modo PID                            *******/

    /*              Impresion en la pantalla LCD           */
    if ((tiempo - prev_tiempo) > 200) {             // intervalo para evitar un mal refresco en la pantalla (se puede cambiar a gusto)
        prev_tiempo = tiempo;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("RPS:");              //linea 1: imprime RPS|RPS_referencia
        lcd.print(data_rps, 0);
        lcd.print("|");
        lcd.print(y2_ref, 0);

        lcd.setCursor(14, 0);            //linea 1: timer
        lcd.print("mm:sg");
        segs = tiempo / 1000 - 60 * non2;
        if (segs == 60) {
            non2++;
            mins++;
        }
        lcd.print(mins, 0);
        lcd.print(":");
        lcd.print(segs, 0);

        lcd.setCursor(0, 1);            //linea 2: imprime Empuje|Emp_referencia
        lcd.print("EM:");
        lcd.print(data_empuje, 1);
        lcd.print("|");
        lcd.print(y1_ref, 1);
        lcd.print(" [gf]");

        lcd.setCursor(0, 2);            //linea 3: imprime torque
        lcd.print("TQ:");
        lcd.print(data_torque, 2);
        lcd.print(" [gf-m]");

        lcd.setCursor(0, 3);            //linea 4: imprime angulo servo
        lcd.print("Ang:");
        lcd.print(data_angulo, 0);
    }



    //  Condicion para cambiar de modo, estado de pseudo-espera, con introducir un caracter se interrumpe el LOOP
    if (Serial.available() == 0) {


        // Modo {1}. Modo manual
        if (modo != 2) {
            digitalWrite(led1, HIGH);                       //LED roja ON
            digitalWrite(led2, LOW);

            // Control manual Paso
            pot_pv_val = analogRead(pot_pasov);
            angulo = map(pot_pv_val, 0, 1023, 62, 116);     //Calibracion de potenciometro: 62 y 116, excederse podria romper la helice
            paso_v.write(angulo);

            // Control manual motor
            pot_m_val = analogRead(pot_motor);
            potencia = map(pot_m_val, 0, 1023, 87, 169);    //Calibracion de potenciometro: 88 y 169
            motor.write(potencia);

        }
        // Modo {2}. Modo Automatico PID
        else {
            digitalWrite(led1, LOW);
            digitalWrite(led2, HIGH);                       //LED azul ON

            PID_emp();                                     // se procesa cada 80ms ~ 100ms, lo que corresponde al ciclo de LOOP()...
            PID_rps();                                     // ... por eso SIEMPRE se procesa y calculan errores, pero no siempre se escribe, vease void PID()
        }
    }



    // Interrupcion del programa, tanto para manual como Auto-PID
    else {
        non = Serial.parseInt();                  //vacio, evita errores de info
        delay(700);
        U10 = paso_v.read();
        U20 = motor.read();
        delay(70);

        digitalWrite(led1, HIGH);
        digitalWrite(led2, HIGH);
        paso_v.write(U10);
        motor.write(U20);                         //se mantiene el motor en la ultima potencia a un paso estable

        cambio();                                 //LLAMANDO EL CAMBIO DE MODO

    }
}








void cambio() {
    if (non != 2) {        // COMANDOS PARA OPERADOR - seleccion de modo (no tocar)
        Serial.println();
        Serial.println("Interrupcion del programa...");
        delay(1000);
        Serial.println("Seleccione un modo de operacion");
        Serial.println("Modo Manual {1}");
        Serial.println("Modo Automatico {2}");
        delay(1000);
        Serial.println("-------------------------");
        Serial.print("Modo: ");
        while (Serial.available() == 0) {
            //bloquea hasta introducir datos
        }
        modo = Serial.parseInt();
        Serial.print(modo);
        delay(200);
        Serial.println();
        Serial.println("Activando el modo seleccionado...");
        delay(200);
    }

    if (modo == 2) {
        // COMANDOS PARA OPERADOR - valores referencia del PID (no tocar)
        Serial.println("Entrando al modo automatico...");
        delay(500);
        Serial.println("Se debe ingresar los parametros para trabajar");
        Serial.println("% Consta del 'Empuje' y 'RPS' de operacion");
        Serial.println("% para evitar problemas, evite seleccionar fuera del rango");
        Serial.println("% Empuje    :   500 gf  -   20 gf");
        Serial.println("% Rev x Seg :   75 RPS  -   20 RPS");
        delay(500);
        Serial.println("Seleccione los valores...");

        Serial.print("Empuje: ");           //intruduce el empuje
        while (Serial.available() == 0) {
            //bloqueo
        }
        y1_ref = Serial.parseFloat();
        Serial.println(y1_ref);
        delay(200);

        Serial.print("RPS: ");            // introduce las RPS
        while (Serial.available() == 0) {
            //bloqueo
        }
        y2_ref = Serial.parseFloat();
        Serial.println(y2_ref);
        Serial.println("-------------------------");
        delay(100);
        Serial.println("Iniciando el control PID...");
        // Condiciones iniciales para comenzar a calcular, previos al cambio de modo "efectivo"
        error11 = (y1_ref - y1) / 25;
        error12 = (y1_ref - y1) / 25;
        error13 = (y1_ref - y1) / 25;

        error21 = (y2_ref - y2) / 1.2;
        error22 = (y2_ref - y2) / 1.2;
        error23 = (y2_ref - y2) / 1.2;

        U11 = U10;
        U21 = U20;
    }
}








void PID_emp() {                     // pid para calcular el Ut_servo segun el empuje yt_emp
    error11 = (y1_ref - y1) / 25;         // factor divisor (25) por efectos del no-escalon unitario. Puede cambiarse para un control mas suave

    if (y1_ref - y1 > 10 || y1_ref - y1 < -10) {                      //error de +-14gf
        U10 = U11 + (kp1 + ki1 * ts / 2 + kd1 / ts) * error11 + (-kp1 + ki1 * ts / 2 - 2 * kd1 / ts) * error12 + (kd1 / ts) * error13;     // PID-Z (pag 48 o 57 dela memoria)
        error13 = error12;                    // cambio de error
        error12 = error11;
        U11 = U10;                         // U(t) anterior
        if (U10 > 116) {                      // saturacion de salida
            U10 = 116;
        } else if (U10 < 62) {
            U10 = 62;
        }
        paso_v.write(U10);              //REVISAR LOS ANGULOS CADA VEZ QUE SE CAMBIA LA RELACION SERVO-HELICE
    }
}

void PID_rps() {                    // pid para calcular el Ut_motor segun las RPS yt_rps
    error21 = (y2_ref - y2) / 1.2;       //factor divisor (1.2) por efectos del no-escalon unitario. Puede cambiarse para un control mas suave

    if (y2_ref - y2 > 5 || y2_ref - y2 < -5) {                      //error de +-5rps
        U20 = U21 + (kp2 + ki2 * ts / 2 + kd2 / ts) * error21 + (-kp2 + ki2 * ts / 2 - 2 * kd2 / ts) * error22 + (kd2 / ts) * error23;
        error23 = error22;
        error22 = error21;
        U21 = U20;
        if (U20 > 150) {
            U20 = 150;
        } else if (U20 < 87) {
            U20 = 87;
        }
        motor.write(U20);                  //Valores saturados segun la ESC, solo se cambian si la ESC es cambiada
    }
}


/***************************************************************
  Serial.println("Ingrese un valor de offset: ");           // Presenta el mensaje inicial 1

  while(Serial.available()==0)                              // Esto permite que no se imprima nada mientras no se ingrese nada.
  {}

  Offset= Serial.parseFloat();                               // Se guarda el valor inicial por el transductor para una velocidad de flujo de aire igual a cero

  do {                                                      //
    while(Serial.available()==0)                              // Esto permite que no se imprima nada mientras no se ingrese nada
    {}

    Serial.println("Ingrese un valor de velocidad en m/s [Rango recomendado de 5-18.7 m/s] [Velocidad máx aprox. 18.7 m/s]: ");    // Presenta el mensaje inicial 1
    Vin= Serial.parseFloat();                                  // Se guarda la velocidad de viento ingresada
    if (Serial.available() > 0) {                             // Si se ingresa la velocidad el programa comienza a correr
      do {
        key = Serial.read();                                 // Lee si se presiona la tecla "+".
        DD_entrada= (analogRead(A0));                        // Voltaje del transductor transformado a datos digitales (DD) de 0 a 1023
        Voltaje= DD_entrada/102.3;                           // Voltaje real medido por el transductor
        Diferencia= 0.14-Offset;                              // Diferencia entre el offset de la calibración (0.140 [V]) y Offset obtenido con el programa "velocidad cero"
        Voltaje_real= Voltaje+Diferencia;                         // Voltaje real ajustado a la curva de calibración
        P_dinamica= (0.327*(Voltaje_real)-0.043)*100;            // Presión dinámica calculada a partir de la medición del transductor [Pa]
        Vmed= sqrt(2*(abs(P_dinamica))/densidad_aire);            // Velocidad medida calculada a partir de la medición del transductor [m/s]
        error= (Vin-Vmed);                                        // Diferencia entre la velocidad deseada (ingresada) y la velocidad medida
        if (error>0.4 || error<-0.4)

         Margen de error impuesto para evitar una
         variación continua de datos digtiales
         al variador de frecuencia (si no se considera
         esto, se puede generar un desperfecto a largo plazo en el variador)
        {                                                        // Si el error se mantiene fuera de este rango el control PI actua, sino el control se desactiva hasta que se vuelva a salir del rango
          s_error= s_error+error;                                 // Error acumulado
          DD_salida= Kp*error+Ki*s_error;                         // Dato digital calculado a partir de la estructura PI
          if (DD_salida<255 && DD_salida>0){
            // Si DD_salida esta entre 0 y 255 arduino
            debe otorgar el valor calculado (Es importante mantenerlo en este rango si es
            mayor a 255, por ejemplo, 256 Ardunio tomará este valor como un dato
            digital igual a 0 y si fuese 256 lo toma como dato 1 dando la vuelta completa al
            rango//

            analogWrite(ventilador,DD_salida);                      // Esto permite que el pin digital de salida otorge un valor de voltaje asociado al dato digital calculado
          }
          if (DD_salida>= 255){                                      // Si DD_salida es mayor o igual a 255 se obliga a DD_salida a tener un valor de 255 fijo para que el dutty cycle no de la vuelta completa a los datos
           DD_salida= 255;
           s_error= s_error-error;                                  // Se resta el error acumulado para que no siga aumentando
           analogWrite(ventilador,255);                            // Esto permite que el pin digital de salida otorge un valor de voltaje asociado al dato digital calculado
          }
          else if (DD_salida<=0){                                  //Si DD_salida es menor o igual a 0 se obliga a DD_salida a tener un valor de 0 fijo para que el dutty cycle no de la vuelta completa a los datos
           s_error= s_error-error;                                  //Se resta el error acumulado para que no siga aumentando
           DD_salida= 0;
           analogWrite(ventilador,0);                              // Esto permite que el pin digital de salida otorge un valor de voltaje asociado al dato digital calculado
          }
        }
      }
    }
  }
****************************************************************/
