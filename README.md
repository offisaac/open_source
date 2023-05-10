## 基于stm32f407zgt6的工程机车

#### 总体介绍

```
华南理工大学2023校内在-BT动力队比赛车辆
电控负责人：刘鹏翔
本车使用stm32f407zgt6作为主控板，使用ps2遥控，使用步进电机，直流减速电机，数字舵机实现麦轮全向运动和矿物抓取
```

#### 电控软件

##### 本工程使用CubeMX+hal库对单片机进行底层配置，用keil5对代码进行编写和程序生成

1.对电机运动的封装

```
1.规定使定电机定向转动的引脚
例如  void cw1()
{
      __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,counter1);
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//右上轮接pe9pe11 pe9正转pe11反转
}

2.采用宏定义确定运动方向
例如
#define forward 1
#define backward 2
#define towardsleft 3
#define towardsright 4
#define cl 5
#define anticl 6

3.通过理解麦轮运动的逻辑，对定时器pwm通道开关进行封装
例如
开启：
void engine_start(void)
{
if(direction==forward)
	{
		ccw1();
		cw2();
		ccw3();
		cw4();
	}
}//向前运动时，左前右后轮反转，右前左后轮正转
关闭：
void engine_end(void)
	{
	     HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		 HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
		 HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
		 HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
		 HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
		 HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
		 HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_3);
		 HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_4);
	}
这样做极大的方便了移植，同时使主函数代码简洁，只需调用engine_start()函数即可，例如
if(XY[2]<300)//向左转
	  {
	      direction=towardsleft;
		  engine_start();
		  HAL_Delay(2);
		  engine_end();//每收到一次信号，电机运动2ms后关闭，实现了按键按下开启，松开关闭
	  }


```

2.对标准库ps2代码的改装（使用NSS软件使能）

```
void PS2_Get(void)    //接受ps2数据
{
	short i = 0;
	
	HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,GPIO_PIN_RESET);  //拉高，开始通讯
	delay_us(10);//用滴答定时器实现的微妙级延迟
	HAL_SPI_TransmitReceive(&hspi1,&cmd[0],&PS2data[0],1,0xffff); // 发送0x01，请求接受数据
	delay_us(10);
	HAL_SPI_TransmitReceive(&hspi1,&cmd[1],&PS2data[1],1,0xffff); // 发送0x42，接受0x01（PS2表示开始通信）
	delay_us(10);
	HAL_SPI_TransmitReceive(&hspi1,&cmd[2],&PS2data[2],1,0xffff); // 发送0x00，接受ID（红绿灯模式）
	delay_us(10);
	for(i = 3;i <9;i++)
	{
		HAL_SPI_TransmitReceive(&hspi1,&cmd[2],&PS2data[i],1,0xffff); // 接受数据
		delay_us(10);
	}
	HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,GPIO_PIN_SET);  //拉低，准备下次通讯
}
```

ps2的按钮使用说明

```
printf(" Lx: %d Ly: %d Rx: %d Ry: %d    ",XY[2],XY[3],XY[0],XY[1]);
for(i1 = 0;i1 < 16;i1++)
{
    if(All_But[i1]) 
    {
        switch(i1)
        {
            case(0): printf(" Left");break;
            case(1): printf(" Down");break;
            case(2): printf(" Right");break;
            case(3): printf(" Up");break;
            case(4): printf(" Start");break;
            case(5): printf(" Select");break;
            case(6): printf(" Left");break;
            case(7): printf(" Select");break;
            case(8): printf(" Square");break;
            case(9): printf(" Cross");break;
            case(10): printf(" Circle");break;
            case(11): printf(" Triangle");break;
            case(12): printf(" R1");break;
            case(13): printf(" L1");break;
            case(14): printf(" R2");break;
            case(15): printf(" L2");break;			
        }
    }
}
```

3.pid算法的运用(使用定时器中断对数据进行更改)

```
使用四个结构体变量储存四个轮子的pid实时参数
想使用积分项，则需要在更改运动状态时清零积分项，防止本次运动被前一个运动干扰
并且需要定时计算实时速度，pid代码不可能放在主函数实行
为了实现以上功能，设定按钮按下标志位以及tim定时中断
逻辑如下
```
![image](https://github.com/offisaac/open_source/blob/main.code/img/img/5ca04d5d161d0198ae3739d226a9ca4.png)

```
相关代码演示
struct PID
{
  float target_val;
	float actual_val;
	float err;
	float err_last;
	float integral_err;
	float output;
	float kp;
	float ki;
	float kd;
};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if((htim->Instance==TIM6)&&(my_flag==0))
{
		PID_param_init1();//这里只写了左前轮的初始化
}
	
if((htim->Instance==TIM6)&&(my_flag==1))//十毫秒中断一次，这里只写了左前轮的pid调用
{
      circle1=(pulse_count1)/(18.8*13);//总圈数
	  circle_cha1=(circle1-circle_last1)*50;//每秒转速/20)*1000(单位由ms到s)
	  PID_param_cha1();
 	  counter1= (int)PID_param_cal1();
	  if(counter1<20)
	  {
	  counter1=0;
	  }
	  if(counter1>71)
	  {
	  counter1=71;
	  }
	  circle_last1=circle1;
	      my_flag=0;//标志位置零
}
}
```

4.根据yPlot上位机协议编写的.h.c文件（配合蓝牙实现远程调节pid）

 1.发送函数

```
void send_wave(void)
{
  //定义通道名帧头帧尾
  uint8_t frameNameHead[] = "AABBCC";
  uint8_t frameNameEnd[] = "CCBBAA";
  //定义数据帧头帧尾
  uint8_t frameDataHead[] = "DDEEFF";
  uint8_t frameDataEnd[] = "FFEEDD";
  //定义通道名
  uint8_t channels_name[] = {“”，“”，“”，“”，“”，“”，“”};
  //设定通道名对应的数据
  channels_data[0]=0;
  channels_data[1]=0;
  channels_data[2]=0;
  channels_data[3]=0;
  channels_data[4]=0;
  channels_data[5]=0;
  channels_data[6]=0;
  //赋值数据
  HAL_UART_Transmit(&huart1,frameNameHead,sizeof(frameDataHead)-1,100); 
  HAL_UART_Transmit(&huart1,channels_name,sizeof(channels_name)-1,100); 
  HAL_UART_Transmit(&huart1,frameNameEnd,sizeof(frameNameEnd)-1,100); 
  HAL_UART_Transmit(&huart1,frameDataHead,sizeof(frameNameHead)-1,100); 
  HAL_UART_Transmit(&huart1,(uint8_t*)channels_data,sizeof(channels_data),100); 
  HAL_UART_Transmit(&huart1,frameDataEnd,sizeof(frameNameHead)-1,100); 
  
}

```

2.接收函数

```
解析函数(针对接收数字)
void use_wave(int uint)//单片机串口通信接收的是ascii码，这里对缓冲区接受的ascii码值进行处理
{
	
pid.kp=(data_rex[5]-48)*unit*100+(data_rex[6]-48)*uint*10+(data_rex[7]-48)*uint
	
pid.ki=(data_rex[9]-48)*unit*100+(data_rex[10]-48)*uint*10+(data_rex[11]-48)*uint;
	
pid.kd=(data_rex[13]-48)*unit*100+(data_rex[14]-48)*uint*10+(data_rex[15]-48)*uint;
	
Speed_set=(data_rex[17]-48)*unit*100+(data_rex[18]-48)*uint*10+(data_rex[19]-48)*uint；
}
数据缓冲区初始化
void initial_wave(void)
{
data_rex[0]=48;
data_rex[1]=48;
data_rex[2]=48;
}
```

本工程小车对步进电机和舵机的使用要求较为简单

步进电机控制连杆上下移动，舵机控制前铲转动

这里只放部分代码



```
 if(XY[0]<300)
	  {
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_8,GPIO_PIN_SET);//步进电机的方向引脚
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_RESET);
            cw9();
			cw10();//同上，对引脚做出规定，方便移植
		    HAL_Delay(2);
			engine_end();
	  }
```

