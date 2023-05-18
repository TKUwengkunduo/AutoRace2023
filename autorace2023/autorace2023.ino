#include <DynamixelWorkbench.h>
#include <Servo.h>

//#include <random.h>
Servo myservo; 

#define DXL_BUS_SERIAL3 "3"            //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#define BAUDRATE  1000000
#define R_Motor_ID    1
#define L_Motor_ID    2
#define KI             0
#define SERIAL Serial2


DynamixelWorkbench R_Motor;
DynamixelWorkbench L_Motor;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SERIAL.begin(9600);
  Serial.setTimeout(100);
  SERIAL.setTimeout(100);

  R_Motor.begin(DXL_BUS_SERIAL3, BAUDRATE);
  L_Motor.begin(DXL_BUS_SERIAL3, BAUDRATE);

  R_Motor.ping(R_Motor_ID);
  L_Motor.ping(L_Motor_ID);

  R_Motor.wheelMode(R_Motor_ID);
  L_Motor.wheelMode(L_Motor_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  String L_spd,R_spd;
  if (Serial.available()){
    L_spd = Serial.readStringUntil('L');
    R_spd = Serial.readStringUntil('R');
    Serial.print("L_Speed = ");
    Serial.println(L_spd);
    Serial.print("R_Speed = ");
    Serial.println(R_spd);
    int R_speed = R_spd.toInt();
    int L_speed = L_spd.toInt();
    R_Motor.goalSpeed(R_Motor_ID, -R_speed);
    L_Motor.goalSpeed(L_Motor_ID, -L_speed);
  }
}
