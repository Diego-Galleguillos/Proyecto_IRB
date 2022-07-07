#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;

// Definición de PINs
#define encoder0PinA  19
#define encoder0PinB  18
#define encoder1PinA  20
#define encoder1PinB  21

char msgEnd = '\n';
bool newMsg = false;
String msg;

int potencia_1, potencia_2 = 0;

// Variables Tiempo
unsigned long time_ant = 0;
const int Period = 10000;   // 10 ms = 100Hz
float escalon = 0;
const float dt = Period *0.000001f;
float referencial, motorout0, motorout1    = 0.0;
float ek, ek1, ek2;
float ck , ck1;
float kp = 2;
float kd = 0;
float ki = 0.0000005;
float k0, k1, k2;
float ts = 1;

float ek_2, ek1_2, ek2_2;
float ck_2 , ck1_2;
float kp_2 = 2;
float kd_2 = 0;
float ki_2 = 0.0000005;
float k0_2, k1_2, k2_2;
float ts_2 = 1;

#define PinA 21
#define PinB 20
#define PinC 19
#define PinD 18

volatile double pos1 = 0.0;
volatile double pos2 = 0.0;

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
long newposition0;
long oldposition0 = 0;
long newposition1;
long oldposition1 = 0;
unsigned long newtime;

float vel0;
float vel1;

float ek_3, ek1_3, ek2_3;
float ck_3 , ck1_3;
float kp_3 = 0.3;
float kd_3 = 0;
float ki_3 = 0;
float k0_3, k1_3, k2_3;
float ts_3 = 1;
float e_total_3;


void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void EncoderA() {
  if (digitalRead(PinA) != digitalRead(PinB)) { //
    pos1 = pos1 + 1;
  }
  else { //reversa
    pos1 = pos1 - 1;
  }
}
void EncoderB() {
  if (digitalRead(PinA) == digitalRead(PinB)) { //
    pos1 = pos1 + 1;
  }
  else { //Reversa
    pos1 = pos1 - 1;
  }
}
void EncoderC() {
  if (digitalRead(PinC) != digitalRead(PinD)) { //
    pos2 = pos2 + 1;
  }
  else { //reversa
    pos2 = pos2 - 1;
  }
}
void EncoderD() {
  if (digitalRead(PinC) == digitalRead(PinD)) { //
    pos2 = pos2 + 1;
  }
  else { //Reversa
    pos2 = pos2 - 1;
  }
}
void setup() {
  Serial.begin(9600);
  Serial3.begin(38400);


  pinMode(PinA, INPUT);
  pinMode(PinB, INPUT);
  pinMode(PinC, INPUT);
  pinMode(PinD, INPUT);

  attachInterrupt(digitalPinToInterrupt(PinA), EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinB), EncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinC), EncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinD), EncoderD, CHANGE);

}
int separador;
int dist;
float angulo;
int largo;
float multiplicador = 1;

void loop() {

  msg = readBuff();
  separador = msg.indexOf(",");
  largo = msg.length();
  dist = msg.substring(0,separador).toInt();

  if (dist> 400 ){
    msg = 400;
    }
  if (dist < -400 ){
    msg = -400;
    }
  if (newMsg == true){
    Serial.println(msg);
    angulo = msg.substring(separador + 1, largo).toInt();
     multiplicador = angulo/90;
    if (dist > 50 or abs(angulo) < 120){
    potencia_1 = (1 + multiplicador)* dist;
    potencia_2 = (1 - multiplicador)* dist;
    }
    else {
      potencia_1 = abs(angulo)*0.2;
      potencia_2 = -abs(angulo)*0.2;
      }//
    newMsg = false;
  }


  
  newtime = micros();

  ts = (newtime - time_ant);
  // PID
  // Controlador 1
  k0 = kp*(1 + ts*ki +kd/ts);
  k1 = -kp*(1 + 2*kd/ts);
  k2 = kp*kd/ts;
  ck = ck1 + k0*ek +k1*ek1;
  
  newtime = micros();

  ts = (newtime - time_ant);
  // Controlador 2
  k0_2 = kp_2*(1 + ts*ki_2 +kd_2/ts);
  k1_2 = -kp_2*(1 + 2*kd_2/ts);
  k2_2 = kp_2*kd_2/ts;
  ck_2 = ck1_2 + k0_2*ek_2 +k1_2*ek1_2;
 
  // Controlador 3
  k0_3 = kp_3*(1 + ts*ki_3 +kd_3/ts);
  k1_3 = -kp_3*(1 + 2*kd_3/ts);
  k2_3 = kp_3*kd_3/ts;
  ck_3 = ck1_3 + k0_3*ek_3 +k1_3*ek1_3;



  //-----------------------------------
  // Calculando Velocidad del motor
  float rpm = 31250;
  vel0 = -(float)(pos1 - oldposition0) * rpm / (newtime - time_ant); //RPM
  vel1 = -(float)(pos2 - oldposition1) * rpm / (newtime - time_ant); //RPM
  oldposition0 = pos1;
  oldposition1 = pos2;
  time_ant = newtime;
  Serial.print("vel_R:  ");
  Serial.print(vel1);
  
  Serial.print("   vel_L:  ");
  Serial.print(vel0);
  Serial.print("   angulo:  ");
  Serial.println(angulo);
//  Serial3.print("   Pos1:  ");
//  Serial3.print(pos1);
//  Serial3.print("   Pos2:  ");
//  Serial3.println(pos2);
  //Serial.println("     error: ");
  //Serial.println(ek);
  Serial3.print("ck_3  ");
  Serial3.print(ck_3);
  Serial3.print("    potencia_1  ");
  Serial3.print(potencia_1);
  Serial3.print("   potencia_2  ");
  Serial3.println(potencia_2);
  
  
    md.setM1Speed(ck);
   md.setM2Speed(-ck_2);

  
  ck1 = ck;
  ek1 = ek;
  ek = potencia_1 - vel1;

  ck1_2 = ck_2;
  ek1_2 = ek_2;
  ek_2 = potencia_2 - vel0;

  ck1_3 = ck_3;
  ek1_3 = ek_3;
  ek_3 =  - angulo;
  time_ant = newtime;
}


String readBuff() {
  String buffArray;
  //int i = 0;

  while (Serial3.available() > 0) { //Entro a este while mientras exista algo en el puerto serial
    char buff = Serial3.read(); //Leo el byte entrante
    if (buff == msgEnd) {
      newMsg = true;
      break; //Si el byte entrante coincide con mi delimitador, me salgo del while
    } else {
      buffArray += buff; //Si no, agrego el byte a mi string para construir el mensaje
      //i += 1;
        //evita imprimir de mas
      
    }
    delay(10);
  }
  return buffArray;  //Retorno el mensaje
}
