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

3.pid算法的运用(使用定时器中断对数据进行更改)

```
使用四个结构体变量储存四个轮子的pid实时参数
因为在更改小车运动状态时，积分项想使用，则需要清零
并且需要定时计算实时速度，pid代码不可能放在主函数实行
为了实现以上功能，设定按钮按下标志位以及tim定时中断
逻辑如下
```
![image](https://github.com/offisaac/open_source/blob/main.code/img/img/5ca04d5d161d0198ae3739d226a9ca4.png)
