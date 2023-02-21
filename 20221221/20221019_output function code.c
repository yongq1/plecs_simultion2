float duty_max=dmax/(2*TS);
int fs=1/TS;
float RC_isense=1/fc_isense;
float RC_speed=1/fc_speed;

float hall_gain=(fs*6.28)/(12*4); //pole 넘버가 4이브로 4, 속도가 rad/s 이므로 2pi 곱합


//----step.1 속도 limit

speed_out=speed_sense;
speed_limit(speed_sense, &dir, &speed_out);



//----step.2 속도 trapezoide filter 적용
//20221019_speed filter
filter_trapezoid(speed_out, speed_old, speed_trapezoid_old, &Out_speed_trapezoid, TS, RC_speed);

speed_old=speed_out; //계측값 전 값
speed_trapezoid_old=Out_speed_trapezoid; //필터 출력 전 값

//20221007_Trapezoid적용으로인한미사용
/* 
speed_lpfs=(tau_speed*speed_lpf+TS*speed_out)/(tau_speed+TS);
speed_lpf=speed_lpfs; 
*/ 
//20221007_Trapezoid적용으로인한미사용



//----step.3 전류 trapezoid filter 적용
//20221019 current filter
filter_trapezoid(isense, isense_old, i_trapezoid_old, &Out_current_trapezoid, TS, RC_isense);
isense_old = isense; //계측값 전 값
i_trapezoid_old = Out_current_trapezoid; //필터 출력 전 값


//----step.4 속도 지령 ramp 업-----
//version 0.1 20220918 speed ramp 기능 추가 
rAccDF=(speed_ref/ramp_time)*con_1ms;

if(tick_1ms<10)
{
   tick_1ms++;
}
else
{
  if(speed_ref>0)
  {
    if(out_speedref+rAccDF<speed_ref)
    {
      out_speedref=out_speedref+rAccDF;
    }
    else
    {
      out_speedref=speed_ref;
    }
  }
  else
  {
    if(out_speedref+rAccDF>speed_ref)
    {
      out_speedref=out_speedref+rAccDF;
    }
    else
    {
      out_speedref=speed_ref;
    }
  }   
  tick_1ms=0;
}



// step.5 속도 PI 제어 및 limit
err_speed=(out_speedref)-Out_speed_trapezoid;//ramp up 작성 후 지우기 20220919

speed_p=err_speed*kv_p;
speed_i+=(err_speed+speed_anti)*kv_i;
speed_pi=speed_p+speed_i;

if (speed_pi>100)
{
	speed_pi_lim=100;
}
else if (speed_pi<-100)
{
	speed_pi_lim=-100;
}
else
{
	speed_pi_lim=speed_pi;
}
speed_anti=kspeed_anti*(speed_pi_lim-speed_pi);




//step.6 전류 PI 제어 및 limit
err_i=(speed_pi_lim)-Out_current_trapezoid;
i_p=ki_p*err_i;
i_i+=ki_i*(err_i+i_anti);
i_pi=i_p+i_i;



if (i_pi>Vreal*0.95) //Vreal*0.95
{
	i_pi_lim=Vreal*0.95;
}
else if (i_pi<-Vreal*0.95)
{
	i_pi_lim=-Vreal*0.95;
}
else
{
	i_pi_lim=i_pi;
}
i_anti=ki_anti*(i_pi_lim-i_pi);

//----step.7 PWM duty 계산
if (i_pi_lim>0)
{
   OUT1 = (int)((i_pi_lim/(Vreal))*duty_max);
}
else
{
   OUT1 = (int)(-(i_pi_lim/(Vreal))*duty_max);
}
/*
//sensorless 제어용
switch((int)phase_ina_vol)
{
    case 1:
        switch ((int)phase_inb_vol)
        {
           case 1:
           if  (phase_inc_vol==1)
           {
            hall_number =99;
           }
           else
           {
            hall_number =2;
           }
           break;
           case 0:
           if  (phase_inc_vol==1)
           {
            hall_number=0;
           }
            else
           {
            hall_number=1;
           }
           break;
        }
    break;
    case 0:
        switch ((int)phase_inb_vol)
        {
           case 1:
           if  (phase_inc_vol==1)
           {
            hall_number =4;
           }
           else
           {
            hall_number =3;
           }
           break;
           case 0:
           if  (phase_inc_vol==1)
           {
            hall_number=5;
           }
            else
           {
            hall_number=50;
           }
           break;
        }
    break;
}
*/
//홀이 120도 계측인 경우
/*
switch ((int)hall_a)
{
	case 1:
		PWM_AH=OUT1;
		PWM_AL=0;
		switch ((int)hall_b)
		{	
			case 1: //경우의 수 없음
				PWM_BH=OUT1;
				PWM_BL=0;
				break;
			case 0:
				PWM_BH=0;
				PWM_BL=0;
				break;
			case -1:
				PWM_BH=0;
				PWM_BL=OUT1;
				break;
		}
		switch ((int)hall_c)
		{
			case 1: //경우의 수 없음
				PWM_CH=OUT1;
				PWM_CL=0;
				break;
			case 0:
				PWM_CH=0;
				PWM_CL=0;
				break;
			case -1:
				PWM_CH=0;
				PWM_CL=OUT1;
				break;
		}
		break;
	case 0:
		PWM_AH=0;
		PWM_AL=0;
		switch ((int)hall_b)
		{			
			case 1:
				PWM_BH=OUT1;
				PWM_BL=0;
				break;
			case 0: //경우의 수 없음
				PWM_BH=0;
				PWM_BL=0;
				break;
			case -1:
				PWM_BH=0;
				PWM_BL=OUT1;
				break;
		}
		switch ((int)hall_c)
		{
			case 1:
				PWM_CH=OUT1;
				PWM_CL=0;
				break;
			case 0: //경우의 수 없음
				PWM_CH=0;
				PWM_CL=0;
				break;
			case -1:
				PWM_CH=0;
				PWM_CL=OUT1;
				break;
		}
		break;		
	case -1:
		PWM_AH=0;
		PWM_AL=OUT1;
		switch ((int)hall_b)
		{			
			case 1:
				PWM_BH=OUT1;
				PWM_BL=0;
				break;
			case 0:
				PWM_BH=0;
				PWM_BL=0;
				break;
			case -1: //경우의 수 없음
				PWM_BH=0;
				PWM_BL=OUT1;
				break;
		}
		switch ((int)hall_c)
		{
			case 1:
				PWM_CH=OUT1;
				PWM_CL=0;
				break;
			case 0:
				PWM_CH=0;
				PWM_CL=0;
				break;
			case -1: //경우의 수 없음
				PWM_CH=0;
				PWM_CL=OUT1;
				break;
		}
		break;		
}
*/
//홀이 180도 계측인 경우


//----step.8 hall 센서 영점 보정
      speed_lpf_temp=(int)Out_speed_trapezoid;
if(Out_speed_trapezoid<0)
{
      speed_lpf_temp=-(int)Out_speed_trapezoid;
}


hall_number=(((int)hall_a+1)*2)+(((int)hall_b+1))+(((int)hall_c+1)*0.5);
if(hall_number_out==0)
{
hall_number_out=hall_number;
hall_number_old=hall_number;
}
if(hall_offset==0)  //0이면 offset 보정 기능, 1이면 off
{
   if(hall_number_old==hall_number)
   {
      hall_number_out=hall_number;
      hall_offset_a=0;
   }
  else
  {
     hall_offset_a=hall_gain/speed_lpf_temp;
     if(hall_offset_count<hall_offset_a)
     {
        hall_number_out=hall_number_old;
        hall_offset_count++;
     }
     else
     {
        hall_offset_count=0;
        hall_number_out=hall_number;
     }
  }

  hall_number_old=hall_number_out;
  if(dir==-1)
  {
    switch (hall_number_out)
    {
      case 1:
        hall_number_out=3;
        break;
      case 2:
        hall_number_out=6;
        break;
      case 3:
        hall_number_out=2;
        break;
      case 4:
        hall_number_out=5;
        break;
      case 5:
        hall_number_out=1;
        break;
      case 6:
        hall_number_out=4;
        break;
    }
  }

}
else
  {
     hall_number_out=hall_number;
  }


//----step.9 PWM 출력
PWM_out(hall_number_out, i_pi_lim, OUT1, &pwm_ah, &pwm_al, &pwm_bh, &pwm_bl, &pwm_ch, &pwm_cl);

PWM_AH = pwm_ah;
PWM_AL = pwm_al;
PWM_BH = pwm_bh;
PWM_BL = pwm_bl;
PWM_CH = pwm_ch;
PWM_CL = pwm_cl;

phase_a_vol=phase_ina_vol;
phase_b_vol=phase_inb_vol;
phase_c_vol=phase_inc_vol;
OUT2 = pwm_ah;
OUT3 = pwm_al;