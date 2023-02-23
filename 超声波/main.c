#include<stdio.h>
#include"stm32f4xx.h"
#include"sys.h"

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

 int fputc(int ch, FILE *f);
 void _sys_exit(int return_code) ;
int fputc(int ch, FILE *f) {
	USART_SendData(USART1,ch);//���ô���һ��printf�ض���
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return (ITM_SendChar(ch));
}

void _sys_exit(int return_code) {
label: goto label; /* endless loop */
}
//����GPIO�ṹ��
static GPIO_InitTypeDef  GPIO_InitStructure;//GPIO
//static EXTI_InitTypeDef  EXTI_InitStructure;//EXIT
static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//TIM
static TIM_OCInitTypeDef  TIM_OCInitStructure;//PWM
static NVIC_InitTypeDef  NVIC_InitStructure;//NVIC
static USART_InitTypeDef  USART_InitStructure;//USART

void usart1_init(uint32_t baud){
	//ʹ�ܴ���Ӳ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	//ʹ������Ӳ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;//PA6������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//����ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//����ģʽ100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//��ʹ������������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//�������źʹ���
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	USART_InitStructure.USART_BaudRate=baud;//���ò�����
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//��Ӳ��������
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//����λΪ8
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλΪ1
	USART_InitStructure.USART_Parity=USART_Parity_No;//����żУ��
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;//���Խ��ջ��߷�����Ϣ
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//�յ������һ������ǿմ����ж�
	
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//��Ӧ
	NVIC_Init(&NVIC_InitStructure);
	//ʹ��USART
	USART_Cmd(USART1,ENABLE);
	
}

void sr_init(){
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//PA1������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//���ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//����ģʽ100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//��ʹ������������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//PA2������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;//���ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//����ģʽ100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//��ʹ������������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	PAout(3)=0;

}

int32_t sr_distance(){
	uint32_t t=0;
			
	int32_t d=0;
		PAout(3)=1;
		delay_us(50);//��ʱ��ͼ���ٷ�������10us�ĸߵ�ƽ
		PAout(3)=0;
		while(PAin(2)==0){
				delay_us(1);
				t++;		//�ȴ��ߵ�ƽ����
			if(t>10000000){
				return -1;//��ʱ����
			}
		}
		t=0;
		while(PAin(2)){
			delay_us(9);//���ߵ�ƽ���ֿ�ʼ�Ǹߵ�ƽʱ����ԭ��ͼ��֪ÿ9us=3mm
			t++;
			if(t>10000000){
				return -1;//��ʱ����
			}
		}
		t=t/2;
		d=3*t;
		return d;
}

void tim3_init(void)
{
	
	//ʹ��TIM3Ӳ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//��ʱ����ʼ��
	//TIM_TimeBaseStructure.TIM_ClockDivision=0;//ʱ�ӷ�Ƶ��F407��֧�֣�
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//����ģʽΪ���ϼ���
	TIM_TimeBaseStructure.TIM_Period=(10000/100)-1;//����ֵΪ0~99���Ƶ��Ϊ20Hz��0.01s
	//TIM_TimeBaseStructure.TIM_Period=(10000/20)-1;//����ֵΪ0~499���Ƶ��Ϊ20Hz��0.05s
	//TIM_TimeBaseStructure.TIM_Period=(10000/1)-1;//����ֵΪ0~9999���Ƶ��Ϊ1Hz��1s
	TIM_TimeBaseStructure.TIM_Prescaler=8400-1;//Ԥ��Ƶֵ��ϵͳ���Զ���һ��Ԥ��ƵֵΪ8400-1+1=8400����ʱƵ��Ϊ84000000/8400=10000Hz
	//TIM_TimeBaseStructure.TIM_RepetitionCounter=
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//PE6ʹ�õ�ΪTIM3_CHN1����Ӧ����ͨ��һ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//����ΪPWM1ģʽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ʹ�ܣ��򿪣����
    TIM_OCInitStructure.TIM_Pulse =20;//�Ƚ�ֵ��Ӧ�� 0~����ֵ ֮�䣩
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//��ЧģʽΪ�ߵ�ƽ
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//����ΪPWM1ģʽ
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//ʹ�ܣ��򿪣����
    TIM_OCInitStructure.TIM_Pulse =80;//�Ƚ�ֵ��Ӧ�� 0~����ֵ ֮�䣩
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//��ЧģʽΪ�ߵ�ƽ
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	//��ʱռ�ձ�Ϊ=�Ƚ�ֵ��50��/����ֵ+1��499+1��=50%
	TIM_Cmd(TIM3,ENABLE);


}

int main(void){

	//uint16_t d;
	int32_t L;
	NVIC_PriorityGroupConfig(2);
	//GPIO���ų�ʼ��
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//PA6������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//���ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//����ģʽ100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//��ʹ������������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	//GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	//ʹ�ܶ�ʱ��3Ӳʱ��
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
	//�жϱ�־λ
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
	//�жϱ�־λ
	if(SET==TIM_GetITStatus(TIM5, TIM_IT_Update))
	{
		PAout(6)^=1;
	
	
		//��ձ�־λ
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
	}
}

void TIM3_IRQHandler(void)
{
	//�жϱ�־λ
	if(SET==TIM_GetITStatus(TIM3, TIM_IT_Update))
	{
		PAout(7)^=1;
	
	
		//��ձ�־λ
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}
