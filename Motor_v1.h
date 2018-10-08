#ifndef Motor_v1_h
#define Motor_v1_h

#define motor_pin_l_a					6
#define motor_pin_l_b					9
#define	motor_pin_r_a					10
#define	motor_pin_r_b					11

#define	encoder_pin_l_a				2
#define encoder_pin_l_b				4
#define encoder_pin_r_a				8
#define encoder_pin_r_b 			7

#define W_KP									0.27//0.35  0.27
#define W_KI									0.0//0.0
#define W_KD									0.0//0.0

#define S_KP									20.0//25.0//20.0
#define S_KI									6.8//5.8//5.0
#define S_KD									0.02//0.6//0.0

#define motor_max_pwm 				210
#define motor_Mspeed				  20
#define Integral_limit				200

#define motor_time_int        10000  //us

typedef struct
{
  long count;
  int count_sp;
  int Mspeed;
  bool dir;
  int encoder_a;
  int encoder_b;

}Wheel;

typedef struct
{
	double target;
	double feedback;
  double output;
	
	double e_0;			//this error
	double e_1;			//last error
	double e_2;			//pre  error
	
	double kp;
  double ki;
  double kd;
  
  double Proportion;
	double Integral;
	double Differential;
  
  char SetPointChang;
  int mode;					// incremen or location
  int motorA;
  int motorB;

}MOTOR_PID;
    
class MotorDriver
{
	public:
		
	// PID mode
	#define	INCREMENT		0	//default 
	#define LOCATION		1
	#define	ENCOUDER_A	0
	#define ENCOUDER_B	1
	
	
		Wheel wheel_motor_l;
    Wheel wheel_motor_r;
    
    MOTOR_PID motor_Wpid_l;
    MOTOR_PID motor_Spid_l;
    MOTOR_PID motor_Wpid_r;
    MOTOR_PID motor_Spid_r;
    
    bool motor_init(void);		// motor parameters init
    bool Count_and_Direction_A(Wheel *omni);		// feedback Mspeed counter
    bool Count_and_Direction_B(Wheel *omni);
 		bool Pid_Control(int left_target, int right_target, int *read_data);
 		bool Speed_Interrupt(void);
    
	private:
		
		bool SPID(MOTOR_PID *pid);
    bool Motor_Star(MOTOR_PID *star);
		
};
#endif
