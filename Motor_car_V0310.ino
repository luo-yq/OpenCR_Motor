#include "Motor_v1.h"

MotorDriver motor;
HardwareTimer Timer(TIMER_CH1);

void controlMotor(void);
void MotorInit(void);
void EncoderLeftA(void);
void EncoderLeftB(void);
void EncoderRightA(void);
void EncoderRightB(void);
void serial_scan();
void serial_return();

boolean var = false;
String inputString = "";
String NumData = "";
int num = 0;
int inchar = 0;


unsigned long tTime[6] = {0};
unsigned long serial_time[2] = {0};
String list_string = "" ;
float motor_line = 6.457273; //cm
double control_l_motor = 0;
double control_r_motor = 0;
int serial_read_ok = 0;

void setup() {
  // put your setup code here, to run once:
  //Serial1.begin(38400);

  MotorInit();
  motor.motor_init();
  
  Serial1.begin(38400);
}

void loop() 
{
  if(millis() - tTime[0] >= 500)  // motor scanning cycle (ms)
  {
    tTime[0] = millis();
  }

  serial_scan();
  serial_return();

}

void ControlMoror(void)
{
  motor.Pid_Control((int)control_l_motor, (int)control_r_motor, &serial_read_ok);
  //motor.Pid_Control(-1000, -1000, &serial_read_ok);
  /*
  if(abs(motor.wheel_motor_l.count - control_l_motor) > 50)
  {
    Serial1.print(motor.wheel_motor_l.count);Serial1.print("   ");
  Serial1.print(motor.wheel_motor_r.count);Serial1.println("   ");
  }*/
}

void MotorInit(void)
{
  pinMode(motor.wheel_motor_l.encoder_a, INPUT_PULLUP);
  pinMode(motor.wheel_motor_l.encoder_b, INPUT_PULLUP);
  pinMode(motor.wheel_motor_r.encoder_a, INPUT_PULLUP);
  pinMode(motor.wheel_motor_r.encoder_b, INPUT_PULLUP);

  pinMode(motor.motor_Spid_l.motorA,OUTPUT);
  pinMode(motor.motor_Spid_l.motorB,OUTPUT);
  pinMode(motor.motor_Spid_r.motorA,OUTPUT);
  pinMode(motor.motor_Spid_r.motorB,OUTPUT);

  attachInterrupt(0, EncoderLeftA, CHANGE);
  attachInterrupt(2, EncoderLeftB, CHANGE);
  attachInterrupt(4, EncoderRightA, CHANGE);
  attachInterrupt(3, EncoderRightB, CHANGE);

  Timer.stop();
  Timer.setPeriod(motor_time_int);// in microseconds 
  Timer.attachInterrupt(ControlMoror);
  Timer.start();
}

void EncoderLeftA(void)
{
  motor.Count_and_Direction_A(&motor.wheel_motor_l);
}

void EncoderLeftB(void)
{
  motor.Count_and_Direction_B(&motor.wheel_motor_l);
}

void EncoderRightA(void)
{
  motor.Count_and_Direction_A(&motor.wheel_motor_r);
}

void EncoderRightB(void)
{
  motor.Count_and_Direction_B(&motor.wheel_motor_r);
}

void serial_scan()
{
  while(Serial1.available() > 0)
  {
    inchar = Serial1.read();
    inputString += (char)inchar;
    
    if(inputString.length() >= 5)
    {
      var = true;
    }
    delay(1);
  }
   
  if(var == true && inputString.length() != 5 )
  {
    //Serial.println(inputString);
    var = false;
    serial_read_ok = 0;
    inputString = "";
    Serial1.println("command error");
  }
  
  if(var == true)
  {
    NumData = "";
    list_string = inputString;
    switch(inputString.charAt(0))
    {
      case 'Q': for(int i =1; i<5; i++)
                {
                  NumData += (char)inputString.charAt(i);
                }
                num = NumData.toInt();
                control_l_motor = motor.wheel_motor_l.count - motor_line*num;
                control_r_motor = motor.wheel_motor_r.count - motor_line*num;
                //Serial.print("Q:");Serial.println(num);
        break;
        
      case 'H':for(int i =1; i<5; i++)
                {
                  NumData += (char)inputString.charAt(i);
                }
                num = NumData.toInt();
                control_l_motor = motor.wheel_motor_l.count + motor_line*num;
                control_r_motor = motor.wheel_motor_r.count + motor_line*num;
                
                //Serial.print("H:");Serial.println(num);
        break;

      default:  var = false;
                serial_read_ok = 0;
                inputString = "";
                Serial1.println("error");
        break;
    }
    if(var == true)
    {
      serial_time[1] = num*15;
      serial_time[1] = constrain(serial_time[1],2500,30000);
      serial_time[0] = millis();
      serial_read_ok = 1;
    }
    
    inputString = "";
    var = false;
  }
}

void serial_return()
{
  static int var = 0;
  if(serial_read_ok == 1)
  {
    var =1;
    if(millis() - serial_time[0] > 300)
    {
      if(serial_read_ok == 0)
      {
        Serial1.print(list_string);
        Serial1.println("1");
      }
      if(millis() - serial_time[0] > serial_time[1])
      {
        Serial1.print(list_string);
        Serial1.println("0");
        serial_read_ok = 0;
        var = 0;
        control_l_motor = motor.wheel_motor_l.count;
        control_r_motor = motor.wheel_motor_r.count;
      }
    }
  }
  else if(var == 1 && serial_read_ok == 0)
  {
    var = 0;
    Serial1.print(list_string);
    Serial1.println("1");
  }
}

