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

void usart3_init(uint32_t baud){
	//ʹ�ܴ���3Ӳ��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 
	
	//ʹ�ܶ˿�B��Ӳ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//����PB10 PB11Ϊ���ù���ģʽ
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;	//��10 11������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;		//���ù���ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;	//���Ÿ��ٹ������յ�ָ������������ȱ�㣺���ĸ�
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;	//�����������������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;	//����Ҫ����������
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//��PB10��PB11���ӵ�����1Ӳ��
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);		

	//���ô���3�������ʣ�У��λ������λ��ֹͣλ
	USART_InitStructure.USART_BaudRate = baud;						//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//����λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;			//1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;				//��У��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//ȡ��������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�ܹ��շ�����
	USART_Init(USART3,&USART_InitStructure);
	
	
	//���ô���3���жϴ�����ʽ���������ֽڴ����ж�
	USART_ITConfig(USART3,USART_IT_RXNE , ENABLE);
	
	
	//ʹ�ܴ���3����	
	USART_Cmd(USART3, ENABLE);
	
	
	//����NVIC������3
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;		//����1���жϺ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;//��ռ���ȼ� 0x2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//��Ӧ���ȼ� 0x2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		//NVICʹ�ܴ���1�ж�����ͨ��
	NVIC_Init(&NVIC_InitStructure);	
	
}


void usart3_senddata(char *str){
	char *p = str;
	
	while(p && (*p!='\0'))
	{
	
		USART_SendData(USART3,*p);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);	
		p++;
		}
}

void blu_config_set(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;//PA6������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//���ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//����ģʽ100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//��ʹ������������
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	PCout(4)=1;
	delay_ms(500);
	usart3_senddata("AT\r\n");
	delay_ms(500);
	usart3_senddata("AT+NAME GEYUXUANBLE\r\n");
	delay_ms(500);
	usart3_senddata("AT+NAME\r\n");
	delay_ms(500);
	usart3_senddata("AT+RESET\r\n");
	delay_ms(500);
	PCout(4)=0;
	delay_ms(500);
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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//uint16_t d;
	//int32_t L;
	NVIC_PriorityGroupConfig(2);
	//GPIO���ų�ʼ��
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;//PA6������
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//���ģʽ
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//����ģʽ100MHZ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;//��ʹ������������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);
	//ʹ�ܶ�ʱ��3Ӳʱ��
	tim3_init();
	
	
	
	usart1_init(9600);
	usart3_init(9600);
	//sr_init();
	//USART_SendData(USART1,'K');
	blu_config_set();
	PAout(6)=1;
	PAout(7)=1;
	
	while(1){

	}
	
}

void USART3_IRQHandler(void)
{
	int16_t d;
	//�жϱ�־λ
	if(USART_GetITStatus(USART3,USART_IT_RXNE) == SET)
	{
		
		d=USART_ReceiveData(USART3);
		
		USART_SendData(USART1,d);
		if(d >=0x00&&d<=0x64)TIM_SetCompare1(TIM3,d);
		
		if(d >=0x65&&d<=0xC8)TIM_SetCompare2(TIM3,d);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
		
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
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
