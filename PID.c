作者：小树
链接：https://www.zhihu.com/question/336703216/answer/763097810
来源：知乎
著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。

struct _pid{
    float SetSpeed;            	//定义设定值
    float ActualSpeed;        	//定义实际值
    float err;                	//定义偏差值
    float err_last;            	//定义上一个偏差值
    float Kp,Ki,Kd;            	//定义比例、积分、微分系数
    float voltage;            	//定义电压值（控制执行器的变量）
    float integral;            	//定义积分值
    float umax;					//正饱和值
    float umin;					//负饱和值
}pid;

void PID_init(){
    printf("PID_init begin \n");
    pid.SetSpeed=0.0;	//设置的温度值
    pid.ActualSpeed=0.0;//实际的温度值
    pid.err=0.0;		//实际误差
    pid.err_last=0.0;	//上一次的误差
    pid.voltage=0.0;	//输出的电压
    pid.integral=0.0;	//积分值

    pid.Kp=0.3;
    pid.Ki=0.15;//注意，和上几次相比，这里加大了积分环节的值
    pid.Kd=0.3;
    pid.umax=280;		//最大值不能大于330
    pid.umin=30;
    printf("PID_init end \n");
}
float PID_realize(uint16_t speed)
{
	static int index;//用于积分分离,即通过该值来确定是否使用积分常数
	pid.SetSpeed=(float)speed;	
	pid.ActualSpeed=get_temperature();//采集出实际值（get_temperature()）
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	if(pid.ActualSpeed>pid.umax)  //若实际值大于最大值
	{
	   if(abs(pid.err)>230)      //蓝色标注为积分分离过程
		{
			index=0;
		}
	   else
	   {
			index=1;
			if(pid.err<0)
			{
				pid.integral+=pid.err;
			}
	   }
	}
	else if(pid.ActualSpeed<pid.umin)
	{
		if(abs(pid.err)>230)      //积分分离过程
		{
			index=0;
		}
		else
		{
			index=1;
			if(pid.err>0)
			{
				pid.integral+=pid.err;
			}
		}
	}
	else
	{
		if(abs(pid.err)>230)                    //积分分离过程
		{
			index=0;
		}
		else
		{
			index=1;
			pid.integral+=pid.err;
		}
	}

	pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
	if(pid.voltage<0)pid.voltage=0;
	pid.err_last=pid.err;
	pid.ActualSpeed=pid.voltage*10.0;
	printf("pid.ActualSpeed=%f \n",pid.ActualSpeed);
	return pid.ActualSpeed;
}