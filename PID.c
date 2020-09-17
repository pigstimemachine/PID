���ߣ�С��
���ӣ�https://www.zhihu.com/question/336703216/answer/763097810
��Դ��֪��
����Ȩ���������С���ҵת������ϵ���߻����Ȩ������ҵת����ע��������

struct _pid{
    float SetSpeed;            	//�����趨ֵ
    float ActualSpeed;        	//����ʵ��ֵ
    float err;                	//����ƫ��ֵ
    float err_last;            	//������һ��ƫ��ֵ
    float Kp,Ki,Kd;            	//������������֡�΢��ϵ��
    float voltage;            	//�����ѹֵ������ִ�����ı�����
    float integral;            	//�������ֵ
    float umax;					//������ֵ
    float umin;					//������ֵ
}pid;

void PID_init(){
    printf("PID_init begin \n");
    pid.SetSpeed=0.0;	//���õ��¶�ֵ
    pid.ActualSpeed=0.0;//ʵ�ʵ��¶�ֵ
    pid.err=0.0;		//ʵ�����
    pid.err_last=0.0;	//��һ�ε����
    pid.voltage=0.0;	//����ĵ�ѹ
    pid.integral=0.0;	//����ֵ

    pid.Kp=0.3;
    pid.Ki=0.15;//ע�⣬���ϼ�����ȣ�����Ӵ��˻��ֻ��ڵ�ֵ
    pid.Kd=0.3;
    pid.umax=280;		//���ֵ���ܴ���330
    pid.umin=30;
    printf("PID_init end \n");
}
float PID_realize(uint16_t speed)
{
	static int index;//���ڻ��ַ���,��ͨ����ֵ��ȷ���Ƿ�ʹ�û��ֳ���
	pid.SetSpeed=(float)speed;	
	pid.ActualSpeed=get_temperature();//�ɼ���ʵ��ֵ��get_temperature()��
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	if(pid.ActualSpeed>pid.umax)  //��ʵ��ֵ�������ֵ
	{
	   if(abs(pid.err)>230)      //��ɫ��עΪ���ַ������
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
		if(abs(pid.err)>230)      //���ַ������
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
		if(abs(pid.err)>230)                    //���ַ������
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