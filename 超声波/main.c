#include<stdio.h>
#include"stm32f4xx.h"
#include"sys.h"

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

 int fputc(int ch, FILE *f);
 void _sys_exit(int return_code) ;
int fputc(int ch, FILE *f) {
	USART_SendData(USART1,ch);//设置串口一的printf重定向
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return (ITM_SendChar(ch));
}

void _sys_exit(int return_code) {
label: goto label; /* endless loop */
}
//定义GPIO结构体
static GPIO_InitTypeDef  GPIO_InitStructure;//GPIO
//static EXTI_InitTypeDef  EXTI_InitStructure;//EXIT
static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//TIM
static TIM_OCInitTypeDef  TIM_OCInitStructure;//PWM
static NVIC_InitTypeDef  NVIC_InitStructure;//NVIC
static USART_InitTypeDef  USART_InitStructure;//USART

void usart1_init(uint32_t baud){
	//使能串口硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	//使能引脚硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;//PA6号引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//高速模式100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//不使用上下拉电阻
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//连接引脚和串口
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	USART_InitStructure.USART_BaudRate=baud;//设置波特率
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//无硬件流控制
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//数据位为8
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位为1
	USART_InitStructure.USART_Parity=USART_Parity_No;//无奇偶校验
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//可以接收或者发送信息
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//收到数据且缓冲区非空触发中断
	
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//响应
	NVIC_Init(&NVIC_InitStructure);
	//使能USART
	USART_Cmd(USART1,ENABLE);
	
}

void sr_init(){
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//PA1号引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//高速模式100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//不使用上下拉电阻
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//PA2号引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;//输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//高速模式100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//不使用上下拉电阻
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	PAout(3)=0;

}

int32_t sr_distance(){
	uint32_t t=0;
			
	int32_t d=0;
		PAout(3)=1;
		delay_us(50);//由时序图至少发出超过10us的高电平
		PAout(3)=0;
		while(PAin(2)==0){
				delay_us(1);
				t++;		//等待高电平出现
			if(t>10000000){
				return -1;//超时处理
			}
		}
		t=0;
		while(PAin(2)){
			delay_us(9);//当高电平出现开始记高电平时间由原理图可知每9us=3mm
			t++;
			if(t>10000000){
				return -1;//超时处理
			}
		}
		t=t/2;
		d=3*t;
		return d;
}

void tim3_init(void)
{
	
	//使能TIM3硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//定时器初始化
	//TIM_TimeBaseStructure.TIM_ClockDivision=0;//时钟分频（F407不支持）
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//计数模式为向上计数
	TIM_TimeBaseStructure.TIM_Period=(10000/100)-1;//计数值为0~99输出频率为20Hz，0.01s
	//TIM_TimeBaseStructure.TIM_Period=(10000/20)-1;//计数值为0~499输出频率为20Hz，0.05s
	//TIM_TimeBaseStructure.TIM_Period=(10000/1)-1;//计数值为0~9999输出频率为1Hz，1s
	TIM_TimeBaseStructure.TIM_Prescaler=8400-1;//预分频值，系统会自动加一则预分频值为8400-1+1=8400，此时频率为84000000/8400=10000Hz
	//TIM_TimeBaseStructure.TIM_RepetitionCounter=
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//PE6使用的为TIM3_CHN1所以应配置通道一
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//设置为PWM1模式
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//使能（打开）输出
    TIM_OCInitStructure.TIM_Pulse =20;//比较值（应在 0~计数值 之间）
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//有效模式为高电平
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//设置为PWM1模式
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//使能（打开）输出
    TIM_OCInitStructure.TIM_Pulse =80;//比较值（应在 0~计数值 之间）
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//有效模式为高电平
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	//此时占空比为=比较值（50）/计数值+1（499+1）=50%
	TIM_Cmd(TIM3,ENABLE);


}

int main(void){

	//uint16_t d;
	int32_t L;
	NVIC_PriorityGroupConfig(2);
	//GPIO引脚初始化
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//PA6号引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出模式
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//高速模式100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//不使用上下拉电阻
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	//使能定时器3硬时钟
	//tim3_init();
	
	
	
	usart1_init(9600);
	sr_init();
	PAout(6)=1;
	PAout(7)=1;
	
	while(1){
		L=sr_distance();
		if(L > 0)
		{
			if(L>=20 && L<=4000)
			{
				printf("distance=%dmm\r\n",L);
			
			}
			
		}
		else{
			printf("error");
		}
		
		delay_ms(1000);
	}
	
}
void USART1_IRQHandler(void)
{
	uint8_t d;
	//判断标志位
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)
	{
		
		d=USART_ReceiveData(USART1);
		
		USART_SendData(USART1,d);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
		if(d == 'A')PAout(6)=0;
		if(d == 'S')PAout(6)=1;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}
void TIM5_IRQHandler(void)
{
	//判断标志位
	if(SET==TIM_GetITStatus(TIM5, TIM_IT_Update))
	{
		PAout(6)^=1;
	
	
		//清空标志位
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
	}
}

void TIM3_IRQHandler(void)
{
	//判断标志位
	if(SET==TIM_GetITStatus(TIM3, TIM_IT_Update))
	{
		PAout(7)^=1;
	
	
		//清空标志位
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}
