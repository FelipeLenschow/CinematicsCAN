#include "STM32_CAN.h"
STM32_CAN Can(CAN1, ALT); 
HardwareSerial SerialX(PA10, PA9);


CAN_message_t M1_msg;
CAN_message_t M2_msg;
CAN_message_t M3_msg;

float L1 = 91.76;
float L2 = 160;
float L3 = 158.5;


float Angle[3];

void SendData() {
  Angle[0] = Angle[0] * 15;
  Angle[1] = Angle[1] * 15;
  Angle[2] = Angle[2] * 15 * 27 / 38;


  memcpy(M1_msg.buf, &Angle[0], 4);
  memcpy(M2_msg.buf, &Angle[1], 4);
  memcpy(M3_msg.buf, &Angle[2], 4);

  Can.write(M1_msg);
  //delay(1);
  Can.write(M2_msg);
  //delay(1);
  Can.write(M3_msg);
  //delay(1);
}

void CalcTheta(float x, float y, float z) {
  Angle[0] = -atan2(-y, x) - atan2(sqrt(x * x + y * y - L1 * L1), -L1);
  float D = (x * x + y * y - L1 * L1 + z * z - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  Angle[2] = atan2(sqrt(1 - D * D), D);
  Angle[1] = atan2(z, sqrt(x * x + y * y - L1 * L1)) - atan2(L3 * sin(Angle[2]), L2 + L3 * cos(Angle[2]));

  Angle[0] = (3.1415926 / 2 + Angle[0]);  // * 180 / 3.1415926;
  Angle[1] = Angle[1];                    // * 180 / 3.1415926;
  Angle[2] = (Angle[2] - 3.1415926);      // * 180 / 3.1415926;
}

void setup() {
  M1_msg.id = (0x11);
  M1_msg.flags.extended = 0;
  M1_msg.len = 4;
  M2_msg.id = (0x12);
  M2_msg.flags.extended = 0;
  M2_msg.len = 4;
  M3_msg.id = (0x13);
  M3_msg.flags.extended = 0;
  M3_msg.len = 4;

  SerialX.begin(115200);
  pinMode(PC12, OUTPUT);
  Can.begin();
  Can.setBaudRate(1000000);  //1MBPS

  pinMode(PC12, OUTPUT);
  delay(2500);
}

float z_ = 0;
void loop() {

  /*
  //Posição de z pelo serial
  if (SerialX.available() > 0) {
    float z_ = SerialX.parseFloat();
    if (z_ != 0) {
      SerialX.println("\n Recebido: " + String(z_));
      CalcTheta(15, L1, z_);
      SerialX.println(String(z_) + " " + String(Angle[0]) + " " + String(Angle[1]) + " " + String(Angle[2]));
    }
  }
  SendData();
*/

  /*
  // Levantar apenas uma vez
  if (z_++ < L2 + L3 - 1)
    CalcTheta(15, L1, z_);
  else
    while (1)
      ;
  SendData();
  SerialX.println(String(z_) + " " + String(Angle[0]) + " " + String(Angle[1]) + " " + String(Angle[2]));
  //delay(30);
  digitalWrite(PC12, !digitalRead(PC12));
  //*/

  // Agachamento
  z_ = 110*cos(millis()/1000 * 2*3.1415926/10) + 200;
  CalcTheta(15, L1, z_);
  SendData();
  SerialX.println(String(z_) + " " + String(Angle[0]) + " " + String(Angle[1]) + " " + String(Angle[2]));
  delay(30);
  digitalWrite(PC12, !digitalRead(PC12));
}
