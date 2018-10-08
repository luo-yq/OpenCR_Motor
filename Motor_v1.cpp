#include "Motor_v1.h"
#include <Arduino.h>

bool MotorDriver::motor_init(void)
{
	// left motor init
	wheel_motor_l.count = 0;
	wheel_motor_l.count_sp = 0;
	wheel_motor_l.Mspeed = 0;
	wheel_motor_l.dir = 0;
	wheel_motor_l.encoder_a = encoder_pin_l_a;
	wheel_motor_l.encoder_b = encoder_pin_l_b;
	
	motor_Wpid_l.target = 0.0;
	motor_Wpid_l.feedback = 0.0;
	motor_Wpid_l.output = 0.0;
	motor_Wpid_l.e_0 = 0.0;
	motor_Wpid_l.e_1 = 0.0;
	motor_Wpid_l.e_2 = 0.0;
	motor_Wpid_l.kp = W_KP;
	motor_Wpid_l.ki = W_KI;
	motor_Wpid_l.kd = W_KD;
	motor_Wpid_l.Proportion =0.0;
	motor_Wpid_l.Integral =0.0;
	motor_Wpid_l.Differential =0.0;
  motor_Wpid_l.SetPointChang = 0;
	motor_Wpid_l.mode = LOCATION;					//pid mode
	motor_Wpid_l.motorA = motor_pin_l_a;
	motor_Wpid_l.motorB = motor_pin_l_b;
	
	motor_Spid_l.target = 0.0;
	motor_Spid_l.feedback = 0.0;
	motor_Spid_l.output = 0.0;
	motor_Spid_l.e_0 = 0.0;
	motor_Spid_l.e_1 = 0.0;
	motor_Spid_l.e_2 = 0.0;
	motor_Spid_l.kp = S_KP;
	motor_Spid_l.ki = S_KI;
	motor_Spid_l.kd = S_KD;
	motor_Spid_l.Proportion =0.0;
	motor_Spid_l.Integral =0.0;
	motor_Spid_l.Differential =0.0;
	motor_Spid_l.SetPointChang = 0;
	motor_Spid_l.mode = INCREMENT;
	motor_Spid_l.motorA = motor_pin_l_a;
	motor_Spid_l.motorB = motor_pin_l_b;
	
	//right motor int
	wheel_motor_r.count = 0;
	wheel_motor_r.count_sp = 0;
	wheel_motor_r.Mspeed = 0;
	wheel_motor_r.dir = 0;
	wheel_motor_r.encoder_a = encoder_pin_r_a;
	wheel_motor_r.encoder_b = encoder_pin_r_b;
	
	motor_Wpid_r.target = 0.0;
	motor_Wpid_r.feedback = 0.0;
	motor_Wpid_r.output = 0.0;
	motor_Wpid_r.e_0 = 0.0;
	motor_Wpid_r.e_1 = 0.0;
	motor_Wpid_r.e_2 = 0.0;
	motor_Wpid_r.kp = W_KP;
	motor_Wpid_r.ki = W_KI;
	motor_Wpid_r.kd = W_KD;
	motor_Wpid_r.Proportion =0.0;
	motor_Wpid_r.Integral =0.0;
	motor_Wpid_r.Differential =0.0;
	motor_Wpid_r.SetPointChang = 0;
	motor_Wpid_r.mode = LOCATION;
	motor_Wpid_r.motorA = motor_pin_r_a;
	motor_Wpid_r.motorB = motor_pin_r_b;
	
	motor_Spid_r.target = 0.0;
	motor_Spid_r.feedback = 0.0;
	motor_Spid_r.output = 0.0;
	motor_Spid_r.e_0 = 0.0;
	motor_Spid_r.e_1 = 0.0;
	motor_Spid_r.e_2 = 0.0;
	motor_Spid_r.kp = S_KP;
	motor_Spid_r.ki = S_KI;
	motor_Spid_r.kd = S_KD;
	motor_Spid_r.Proportion =0.0;
	motor_Spid_r.Integral =0.0;
	motor_Spid_r.Differential =0.0;
	motor_Spid_r.SetPointChang = 0;
	motor_Spid_r.mode = INCREMENT;
	motor_Spid_r.motorA = motor_pin_r_a;
	motor_Spid_r.motorB = motor_pin_r_b; 
  
	return true;
}


// feedback Mspeed counter
bool MotorDriver::Count_and_Direction_A(Wheel *omni)
{
	if(digitalRead(omni->encoder_a) == HIGH)
		{
			if(digitalRead(omni->encoder_b) == LOW)
				{
					omni->count++;
					omni->count_sp++;
					omni->dir = true;
				}
			else
				{
					omni->count--;
					omni->count_sp--;
					omni->dir = false;
				}
		}
	else
		{
			if(digitalRead(omni->encoder_b) == HIGH)
				{
					omni->count++;
					omni->count_sp++;
					omni->dir = true;
				}
			else
				{
					omni->count--;
					omni->count_sp--;
					omni->dir = false;
				}
		}
   return true;
}

bool MotorDriver::Count_and_Direction_B(Wheel *omni)
{
  if(digitalRead(omni->encoder_b) == HIGH)
    {
      if(digitalRead(omni->encoder_a) == HIGH)
        {
          omni->count++;
          omni->count_sp++;
          omni->dir = true;
        }
      else
        {
          omni->count--;
          omni->count_sp--;
          omni->dir = false;
        }
    }
  else
    {
      if(digitalRead(omni->encoder_a) == LOW)
        {
          omni->count++;
          omni->count_sp++;
          omni->dir = true;
        }
      else
        {
          omni->count--;
          omni->count_sp--;
          omni->dir = false;
        }
    }
    return true;
}

bool MotorDriver::SPID(MOTOR_PID *pid)
{
  if(pid->SetPointChang == 1)
  {
     pid->SetPointChang = 0;
     pid->e_0 = pid->e_1 = pid->e_2 = 0;
  }
  
	pid->e_0 = pid->target - pid->feedback;			//this error
	
	if(pid->mode == LOCATION)
		{
			pid->Proportion = pid->e_0;
			pid->Integral +=  pid->e_0;
			pid->Differential = pid->e_0 - pid->e_1;
      
			pid->e_0 = pid->e_1;      //last error
			pid->Integral = constrain(pid->Integral,-Integral_limit,Integral_limit);
			
					//pwm=Kp*e(k)+Ki*e(k)+Kd[e(k)-e(k-1)]
			pid->output = pid->kp * pid->Proportion + pid->ki * pid->Integral + pid->kd * pid->Differential;
      
      //Serial.println('W');
		} 
	else if(pid->mode == INCREMENT)
		{
			pid->Proportion = pid->e_0 - pid->e_1;
			pid->Integral = pid->e_0;
			pid->Differential = (pid->e_0 - pid->e_1) - (pid->e_1 - pid->e_2);

      pid->e_2 = pid->e_1;      //last error
      pid->e_1 = pid->e_0;      //pre  error
			
					//pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[(e(k)-e(k-1))-(e(k-1)-e(k-2))]
			pid->output += pid->kp * pid->Proportion + pid->ki * pid->Integral + pid->kd * pid->Differential;
     //Serial.println('S');
		}
	return true;
}

bool MotorDriver::Pid_Control(int left_target, int right_target,int *read_data)
{ 
  static double cha_li, cha_ri;
  static long last_l, last_r;
  Speed_Interrupt();
  if(*read_data == 1)
  {
    motor_Wpid_l.target = (double)left_target;
    motor_Wpid_r.target = (double)right_target;
  }
  
  motor_Wpid_l.feedback = (double)wheel_motor_l.count;
  motor_Wpid_r.feedback = (double)wheel_motor_r.count;

  cha_li = motor_Wpid_l.target - motor_Wpid_l.feedback;
  cha_ri = motor_Wpid_r.target - motor_Wpid_r.feedback;
  
  if(abs(wheel_motor_l.count - last_l) < 150 || abs(wheel_motor_r.count - last_r) < 150)
    motor_Wpid_l.kp = motor_Wpid_r.kp = 0.05;
  else if(abs(wheel_motor_l.count - last_l) < 350 || abs(wheel_motor_r.count - last_r) < 350)
    motor_Wpid_l.kp = motor_Wpid_r.kp = 0.15;
  else if(*read_data == 1 && (abs(cha_li) < 200 && abs(cha_li) >5 ) || (abs(cha_ri) < 200 && abs(cha_ri) >5))
  {
    motor_Wpid_l.kp = motor_Wpid_r.kp = 0.12;
  }
  else 
  
    motor_Wpid_l.kp = motor_Wpid_r.kp = 0.27;
    
  SPID(&motor_Wpid_l);
  SPID(&motor_Wpid_r);
  
  motor_Spid_l.target = constrain(motor_Wpid_l.output,-motor_Mspeed,motor_Mspeed);
  motor_Spid_r.target = constrain(motor_Wpid_r.output,-motor_Mspeed,motor_Mspeed);

  //The motor speed is synchronized.

  if(*read_data == 1 && (abs(cha_li) > 100 || abs(cha_li) > 100))
  {
    if(motor_Spid_l.target > motor_Spid_r.target)
    {
      motor_Spid_l.target = motor_Spid_r.target;
    }
    else if(motor_Spid_l.target < motor_Spid_r.target)
    {
      motor_Spid_r.target = motor_Spid_l.target;
    }
  }
    
  motor_Spid_l.feedback = wheel_motor_l.Mspeed;
  motor_Spid_r.feedback = wheel_motor_r.Mspeed;
  SPID(&motor_Spid_l);
  SPID(&motor_Spid_r);
  
  motor_Spid_l.output = constrain(motor_Spid_l.output,-motor_max_pwm,motor_max_pwm);
  motor_Spid_r.output = constrain(motor_Spid_r.output,-motor_max_pwm,motor_max_pwm);

  if(motor_Wpid_l.target == wheel_motor_l.count)
    motor_Spid_l.output = 0.0;
  if(motor_Wpid_r.target == wheel_motor_r.count)
    motor_Spid_r.output = 0.0;
  if(motor_Spid_l.output == 0.0 || motor_Spid_r.output == 0.0)
    {
      *read_data = 0;
      last_l =  wheel_motor_l.count;
      last_r =  wheel_motor_r.count;
    }

  Motor_Star(&motor_Spid_l);
  Motor_Star(&motor_Spid_r);
  return true;
}

bool MotorDriver::Speed_Interrupt(void)
{
	wheel_motor_l.Mspeed = wheel_motor_l.count_sp;
	wheel_motor_l.count_sp = 0;
	wheel_motor_r.Mspeed = wheel_motor_r.count_sp;
	wheel_motor_r.count_sp = 0;
	
	return true;
}

bool MotorDriver::Motor_Star(MOTOR_PID *star)
{
	if(abs(star->output) > motor_max_pwm)
		{
		  return false;
		}
 if(star->output > 1.0)
    {
      analogWrite(star->motorA, (uint8_t)abs(star->output));
      analogWrite(star->motorB, LOW);
    }
  else if(star->output < 1.0)
    {
      analogWrite(star->motorB, (uint8_t)abs(star->output));
		
			analogWrite(star->motorA, LOW);
		}
	else
		{
			analogWrite(star->motorA, LOW);
			analogWrite(star->motorB, LOW);
		}

	return true;
}



