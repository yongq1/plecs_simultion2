#include <float.h>
#include <stdio.h>

#define Vreal Input(0)   //modulation index
#define Vref Input(1)   //modulation index
#define isense Input(2)   //modulation index
#define hall_a Input(3)   //modulation index
#define hall_b Input(4)   //modulation index
#define hall_c Input(5)   //modulation index
#define speed_sense Input(6)   //modulation index
#define speed_ref Input(7)   //modulation index
#define phase_ina_vol Input(8)   //modulation index
#define phase_inb_vol Input(9)   //modulation index
#define phase_inc_vol Input(10)   //modulation index
#define OUT1 Output(0)  //switch output
#define OUT2 Output(1)  //switch output
#define OUT3 Output(2)  //switch output
#define PWM_AH Output(3)  //switch output
#define PWM_AL Output(4)  //switch output
#define PWM_BH Output(5)  //switch output
#define PWM_BL Output(6)  //switch output
#define PWM_CH Output(7)  //switch output
#define PWM_CL Output(8)  //switch output
#define phase_a_vol Output(9)  //switch output
#define phase_b_vol Output(10)  //switch output
#define phase_c_vol Output(11)  //switch output

#define TS ParamRealData(0,0)  //sample period
#define DEADTIME ParamRealData(1,0)
#define dmax ParamRealData(2,0)
#define kv_p ParamRealData(3,0)
#define kv_i ParamRealData(4,0)
#define ki_p ParamRealData(5,0)
#define ki_i ParamRealData(6,0)
int tick=1;
int deltatick=100;
int init_count=0;


void speed_limit(float speed_original, int *ppp, float *iii );
void filter_trapezoid(float input_now, float input_old, float output_old, float *output, float Tsample, float rc);
void PWM_out(int hall_number, float current, int duty, int *pwm_ah, int *pwm_al, int *pwm_bh, int *pwm_bl, int *pwm_ch, int *pwm_cl);
int PWM_duty(float vout, float vdc, float maximum_duty);
void pwm_out_120_hall(int hall_in_a, int hall_in_b, int hall_in_c, int duty, int *pwm_ah, int *pwm_al, int *pwm_bh, int *pwm_bl, int *pwm_ch, int *pwm_cl);
float ramp_up(float ref);
void pwm_force(int state, float current, int *pwm_ah, int *pwm_al, int *pwm_bh, int *pwm_bl, int *pwm_ch, int *pwm_cl);

//double deltatick_p1=deltatick*TS*20;
/*
int deltatick_p2=deltatick*TS*10;
int deltatick_p3=deltatick*TS*5;
int deltatick_p4=deltatick*TS*2;
int deltatick_p5=deltatick*TS;
int deltatick_m1=-deltatick*TS*20;
int deltatick_m2=-deltatick*TS*10;
int deltatick_m3=-deltatick*TS*5;
int deltatick_m4=-deltatick*TS*2;
int deltatick_m5=-deltatick*TS;
*/
int hall_number_out=99;

float err_speed=0.1;
//float kv_p=0.1;
//float kv_i=0.001;
float speed_p=0;
float speed_i=0;
float speed_pi=0;
float speed_pi_lim=0;
float i_p=0;
float i_i=0;
float i_pi=0;
float i_pi_lim=0;
float kspeed_anti=0.5;
float speed_anti=0;
float ki_anti=0.9;
float i_anti=0;
float err_i=0;

float isense_lpfs=0;
float isense_lpf=0; 
float isense_ad=0;
float tau_speed=0.00005;
float tau_i=0.0001;
float speed_out=0.01;

float fc_isense = 10000;
float Out_current_trapezoid = 0;
float isense_old = 0;
float i_trapezoid_old = 0;

float Out_trapezoid_2=0;
float isense_old_2=0;
float i_trapezoid_old_2=0;

int state_hh = 1;

float fc_speed=4000;

float speed_old=0;
float speed_trapezoid_old=0;
float Out_speed_trapezoid=0;

float speed_lpfs=0;
float speed_lpf=0;
int dir =0;
int hall_number=0;
int debug =1;
float hall_gain=1400;
int hall_a_old=99;
int hall_b_old=99;
int hall_c_old=99;

int hall_a_out=0;
int hall_b_out=0;
int hall_c_out=0;

int hall_offset=1;//hall sensor offset 보정 기능 0 active, 1 deactive

float hall_offset_a=0;
float hall_offset_b=0;
float hall_offset_c=0;
float test_tick = 0.0;

int pwm_ah = 0;
int pwm_al = 0;
int pwm_bh = 0;
int pwm_bl = 0;
int pwm_ch = 0;
int pwm_cl = 0;

int counter_hall = 0;

int speed_lpf_temp=0;

int hall_temp_a=0;
int hall_temp_b=0;
int hall_temp_c=0;

int hall_number_old=0;
int hall_offset_count=0;

float out_speedref=0; //20220919 ramp_up 함수용



//speed limit and direction
void speed_limit(float speed_original, int *ppp, float *iii)
{
	
	if (speed_original>0)
	{
		*ppp = 1;
   	if(speed_original>1000)
   	{
      	//speed_out=1000;
      	*iii=1000;
	   }     
	}
	else if (speed_original<0)
	{
   	if(speed_original<-1000)
	   {
	      //speed_out=-1000;
    	   *iii=-1000;
   	}
   *ppp = -1;
	}
}

//20221019_Trapezoid filter
void filter_trapezoid(float input_now, float input_old, float output_old, float *output, float Tsample, float rc)
{
	*output = output_old + (Tsample/(Tsample + (2*rc))) * (input_now + input_old - 2*output_old);
}


//20221019 PWM output
void PWM_out(int hall_number, float current, int duty, int *pwm_ah, int *pwm_al, int *pwm_bh, int *pwm_bl, int *pwm_ch, int *pwm_cl)
{
	if (current >0)  //20221019 출력 전류 rms가 양수
	{
	   
		switch (hall_number)
		{
			case 1: //hall signal(abc) 001
				*pwm_ah = 0;
				*pwm_al = 0;				
				*pwm_bh = 0;
				*pwm_bl = duty;
				*pwm_ch = duty;
				*pwm_cl = 0;
				break;
			case 2: //hall signal(abc) 010
				*pwm_ah = 0;
				*pwm_al = duty;
				*pwm_bh = duty;
				*pwm_bl = 0;
				*pwm_ch = 0;
				*pwm_cl = 0;
				break;
			case 3: //hall signal(abc) 011
				*pwm_ah = 0;
				*pwm_al = duty;
				*pwm_bh = 0;
				*pwm_bl = 0;
				*pwm_ch = duty;
				*pwm_cl = 0;
				break;
			case 4: //hall signal(abc) 100
				*pwm_ah = duty;
				*pwm_al = 0;
				*pwm_bh = 0;
				*pwm_bl = 0;
				*pwm_ch = 0;
				*pwm_cl = duty;
				break;
			case 5: //hall signal(abc) 101
				*pwm_ah = duty;
				*pwm_al = 0;
				*pwm_bh = 0;
				*pwm_bl = duty;
				*pwm_ch = 0;
				*pwm_cl = 0;
				break;
			case 6: //hall signal(abc) 110
				*pwm_ah = 0;
				*pwm_al = 0;
				*pwm_bh = duty;
				*pwm_bl = 0;
				*pwm_ch = 0;
				*pwm_cl = duty;
				break;
		}
	}
	else   //20221019 출력 전류 rms가 음수
	{
		switch (hall_number)
		{
			case 1: //hall signal(abc) 001
				*pwm_ah = 0;
				*pwm_al = 0;
				*pwm_bh = duty;
				*pwm_bl = 0;
				*pwm_ch = 0;
				*pwm_cl = duty;
				break;
			case 2: //hall signal(abc) 010
				*pwm_ah = duty;
				*pwm_al = 0;
				*pwm_bh = 0;
				*pwm_bl = duty;
				*pwm_ch = 0;
				*pwm_cl = 0;
				break;
			case 3: //hall signal(abc) 011
				*pwm_ah = duty;
				*pwm_al = 0;
				*pwm_bh = 0;
				*pwm_bl = 0;
				*pwm_ch = 0;
				*pwm_cl = duty;
				break;
			case 4: //hall signal(abc) 100
				*pwm_ah = 0;
				*pwm_al = duty;
				*pwm_bh = 0;
				*pwm_bl = 0;
				*pwm_ch = duty;
				*pwm_cl = 0;
				break;
			case 5: //hall signal(abc) 101
				*pwm_ah = 0;
				*pwm_al = duty;
				*pwm_bh = duty;
				*pwm_bl = 0;
				*pwm_ch = 0;
				*pwm_cl = 0;
				break;
			case 6: //hall signal(abc) 110
				*pwm_ah = 0;
				*pwm_al = 0;
				*pwm_bh = 0;
				*pwm_bl = duty;
				*pwm_ch = duty;
				*pwm_cl = 0;
				break;
		}
	}
	
}
//20221019_PWM duty 계산 count 5000으로 계산
int PWM_duty(float vout, float vdc, float maximum_duty)
{
  if(vout > 0)
  {
    return (int)((vout * maximum_duty) / vdc);
  }
  else
  {
    return (int)(-(vout * maximum_duty) / vdc);
  }
}
void pwm_force(int state, float current, int *pwm_ah, int *pwm_al, int *pwm_bh, int *pwm_bl, int *pwm_ch, int *pwm_cl)
{
   int duty = 5000;
   switch (state)
   {
      case 1:
           *pwm_ah = 0 ;
           *pwm_al = 0;

	   *pwm_bh = 0;
	   *pwm_bl = duty;

	   *pwm_ch = duty;
	   *pwm_cl = 0;
            break;
      case 2:
           *pwm_ah = duty ;
           *pwm_al = 0;

	   *pwm_bh = 0;
	   *pwm_bl = duty;

	   *pwm_ch = 0;
	   *pwm_cl = 0;
            break;
      case 3:
           *pwm_ah = duty ;
           *pwm_al = 0;

	   *pwm_bh = 0;
	   *pwm_bl = 0;

	   *pwm_ch = 0;
	   *pwm_cl = duty;
            break;
      case 4:
           *pwm_ah = 0;
           *pwm_al = 0;

	   *pwm_bh = duty;
	   *pwm_bl = 0;

	   *pwm_ch = 0;
	   *pwm_cl = duty;
            break;
      case 5:
           *pwm_ah = 0;
           *pwm_al = duty;

	   *pwm_bh = duty;
	   *pwm_bl = 0;

	   *pwm_ch = 0;
	   *pwm_cl = 0;
            break;
      case 6:
           *pwm_ah = 0;
           *pwm_al = duty;

	   *pwm_bh = 0;
	   *pwm_bl = 0;

	   *pwm_ch = duty;
	   *pwm_cl = 0;
            break;
   }
}

void pwm_out_120_hall(int hall_in_a, int hall_in_b, int hall_in_c, int duty, int *pwm_ah, int *pwm_al, int *pwm_bh, int *pwm_bl, int *pwm_ch, int *pwm_cl)
{
    switch (hall_in_a)
    {
        case 1:
        *pwm_ah = duty ;
        *pwm_al = 0;
	switch (hall_in_b)
	{	
		case 1: //경우의 수 없음
			*pwm_bh = duty;
			*pwm_bl = 0;
			break;
		case 0:
			*pwm_bh = 0;
			*pwm_bl = 0;
			break;
		case -1:
			*pwm_bh = 0;
			*pwm_bl = duty;
			break;
	}
	switch (hall_in_c)
	{
		case 1: //경우의 수 없음
			*pwm_ch = duty;
			*pwm_cl = 0;
			break;
		case 0:
			*pwm_ch = 0;
			*pwm_cl = 0;
			break;
		case -1:
			*pwm_ch = 0;
			*pwm_cl = duty;
			break;
	}
	break;
        
        case 0:
        *pwm_ah = 0;
        *pwm_al = 0;
	switch (hall_in_b)
	{			
		case 1:
			*pwm_bh = duty;
			*pwm_bl = 0;
			break;
		case 0: //경우의 수 없음
			*pwm_bh = 0;
			*pwm_bl = 0;
			break;
		case -1:
			*pwm_bh = 0;
			*pwm_bl = duty;
			break;
	}
	switch (hall_in_c)
	{
		case 1:
			*pwm_ch = duty;
			*pwm_cl = 0;
			break;
		case 0: //경우의 수 없음
			*pwm_ch = 0;
			*pwm_cl = 0;
			break;
		case -1:
			*pwm_ch = 0;
			*pwm_cl = duty;
			break;
	}
        break;
        
        case -1:
        *pwm_ah = 0;
        *pwm_al = duty;
	switch (hall_in_b)
	{			
		case 1:
			*pwm_bh = duty;
			*pwm_bl = 0;
			break;
		case 0:
			*pwm_bh = 0;
			*pwm_bl = 0;
			break;
		case -1: //경우의 수 없음
			*pwm_bh = 0;
			*pwm_bl = duty;
			break;
	}
	switch (hall_in_c)
	{
		case 1:
			*pwm_ch = duty;
			*pwm_cl = 0;
			break;
		case 0:
			*pwm_ch = 0;
			*pwm_cl = 0;
			break;
		case -1: //경우의 수 없음
			*pwm_ch = 0;
			*pwm_cl = duty;
			break;
	}
	break;
    }   
}
float ramp_up(float ref)
{
  static float rAccDF = 0;
  float ramp_time=0.3;  //20220919 ramp_up 함수용
  static int tick_1ms=0;       //20220919 ramp_up 함수용
  float con_1ms=0.001;   //20220919 ramp_up 함수용
  static float ramp_speed;
  rAccDF = (ref/ramp_time)*con_1ms;
  if(tick_1ms<10)
  {
     tick_1ms++;
  }
  else
  {
    if(ref>0)
    {
      if(ramp_speed+rAccDF<ref)
      {
        ramp_speed = ramp_speed+rAccDF;
      }
      else
      {
        ramp_speed = ref;
      }
    }
    else
    {
      if(ramp_speed+rAccDF>ref)
      {
        ramp_speed = ramp_speed+rAccDF;
      }
      else
      {
        ramp_speed = ref;
      }
    }   
    tick_1ms=0;
  }
  return ramp_speed;
}