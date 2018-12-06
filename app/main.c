#include "includes.h"

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define  TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE
#define  N_TASKS        9

OS_STK TaskStk[N_TASKS][TASK_STK_SIZE];
OS_EVENT* Mbox;
OS_EVENT* LightMbox;
OS_EVENT* mutex;
OS_EVENT* MsgQ;
OS_FLAG_GRP* t_grp;		//task의 실행 순서를 정하는 EventFlag
void* MsgQTbl[2];  //Message Queue Table

// 0: LED, 1: 버저, 2: 광, 3: 온도

volatile INT8U FndNum;
volatile INT8U LedOper;
volatile INT8U BuzzOper;
volatile INT8U TaskNum;
volatile INT8U on=1;
volatile INT8U IND=0;
volatile INT8U status=0;
const INT8U myMapTbl[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
const INT8U LED[4]={0,0x38,0x79,0x3F};	//LED 출력위함
const INT8U BUZZ[4]={0x7F,0x3E,0x5B,0x5B};	//BUZZ 출력위함
const INT8U DIGIT[12]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7c,0x07,0x7f,0x67,0x40,0x00};
const INT8U rabbit[25]={4,2,2,4,2,0,1,3,1,0,2,4,7,4,7,4,7,4,2,4,1,3,2,1,0};
const INT8U rabbit_dly[25]={4,2,2,2,2,4,4,2,2,2,2,4,3,1,2,2,2,2,4,4,2,2,2,2,4};
const INT8U Frequency[8]={17,43,66,77,97,114,129,137};
INT8U FNDData[4]={0x3F,0x79,0x38,0};
void FndTask(void *data);
void FndDisplayTask(void *data);
void LedTask (void *data);
void BuzzerTask(void* data);
void TemperatureTask(void *data);
void ReadTemperatureTask(void *data);
void ReadLightTask(void* data);
void LightTask(void* data);
void InitI2C();
ISR(INT4_vect);
ISR(INT5_vect);
ISR(TIMER2_OVF_vect);

struct Mail {
	INT8U data;
	INT8U sel;
};

int main (void)
{
	INT8U err;
	OSInit();
	OS_ENTER_CRITICAL();
	TCCR0 = 0x07;
	TIMSK = 0x01;
	TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024);
	DDRE=0xCF;	//SW 입력 모드
	EICRB=0x0A;
	EIMSK=0x30;
	sei();
	//interrupt설정
	OS_EXIT_CRITICAL();

	Mbox=OSMboxCreate((void*)0);
	LightMbox=OSMboxCreate((void*)0);
	mutex=OSMutexCreate(1,&err);
	t_grp=OSFlagCreate(myMapTbl[TaskNum],&err);
	MsgQ=OSQCreate(MsgQTbl,2);

	
	OSTaskCreate(TemperatureTask, (void *)0, (void *)&TaskStk[0][TASK_STK_SIZE - 1], 2);
	OSTaskCreate(ReadTemperatureTask, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 3);
	OSTaskCreate(LightTask, (void *)0, (void *)&TaskStk[2][TASK_STK_SIZE - 1], 4);
	OSTaskCreate(ReadLightTask, (void *)0, (void *)&TaskStk[3][TASK_STK_SIZE - 1], 5);
	OSTaskCreate(LedTask, (void *)0, (void *)&TaskStk[4][TASK_STK_SIZE - 1], 6);
	OSTaskCreate(BuzzerTask, (void *)0, (void *)&TaskStk[5][TASK_STK_SIZE - 1], 7);
	OSTaskCreate(FndDisplayTask, (void *)0, (void *)&TaskStk[6][TASK_STK_SIZE - 1], 8);
	OSTaskCreate(FndTask, (void *)0, (void *)&TaskStk[7][TASK_STK_SIZE - 1], 9);

	OSStart();

	return 0;
}
//온도 측정한거 받아서 FNDData 수정하는 Task 우선순위 높음
//온도 측정 Task 우선순위 낮음
//메시지 큐로 2개 데이터 (low, high)보내줌
//받는 Task가 2개 data pend해서 합침
/*
SW1: Task Switching용, 한번 누를때마다 LED->버저->광->온도 순으로 Cycle
FND에는 LED, BUZZ, 빛 정도, 온도 표시
*/
ISR(INT4_vect) {
	INT8U err,i;
	OSFlagPend(t_grp,myMapTbl[TaskNum],OS_FLAG_WAIT_SET_ALL+OS_FLAG_CONSUME,0,&err);

	OSMutexPend(mutex,0,&err);
	TaskNum=(TaskNum+1)&0x03;
	OSMutexPost(mutex);

	OSMutexPend(mutex,0,&err);
	switch(TaskNum) {
		case 0:
		for(i=0;i<4;i++) {
			FNDData[i]=LED[3-i];
		}
		break;
		case 1:
		for(i=0;i<4;i++) {
			FNDData[i]=BUZZ[3-i];
		}
		break;
	}
	OSMutexPost(mutex);
	OSFlagPost(t_grp,myMapTbl[TaskNum],OS_FLAG_SET,&err);
}

/*
SW2: operation 전환용, 한번 누를때마다 operation바뀜
LED: Left Circular Shift, Right Circular Shift, 깜빡깜빡, 왼쪽오른쪽왼쪽오른쪽,계속켜짐
Buzzer: 도레미파솔라시도~, 도시라솔파미레도~ , 음악 하나선정해서 연주
광: 생각 해봐야함...
온도: 정확도 조절?, 이것도 생각좀...., 일단 FND 온도 표시
*/
ISR(INT5_vect) {
	INT8U err;
	OSMutexPend(mutex,0,&err);
	switch(TaskNum) {
		case 0:		//LED
		LedOper=(LedOper+1)&0x03;
		break;
		case 1:		//BUZZER
		break;
	}
	OSMutexPost(mutex);
}

ISR(TIMER2_OVF_vect) {
	if(status) {
		if(on) {
			PORTB=0x00;
			on=0;
		} else {
			PORTB=0x10;
			on=1;
		}
		TCNT2=Frequency[rabbit[IND]];
	}
}

void FndTask (void *data)
{
	data = data;
	INT8U sel=0;
	struct Mail fnddata;
	while (1) {
		fnddata.data=FNDData[sel];
		fnddata.sel=sel;
		sel=(sel+1)&0x03;
		OSMboxPost(Mbox,&fnddata);
		OSTimeDlyHMSM(0,0,0,2);
	}
}

void FndDisplayTask (void *data)
{
	INT8U err;
    data = data;
    DDRC = 0xff;	
    DDRG = 0x0f;
	struct Mail* fnddata;
    while(1)  {
		fnddata=(struct Mail*)OSMboxPend(Mbox,0,&err);
		PORTG=myMapTbl[fnddata->sel];
		PORTC=fnddata->data;		
    }
}

void LedOperation(INT8U oper, INT8U* led, INT8U* ledstatus) {
	switch(oper) {
		case 0:		//left circular shift
		PORTA=(*led);
		if((*led)==0x80) {
			(*led)=0x01;
		} else {
			(*led)<<=1;
		}
		break;
		case 1:		//right circular shift
		PORTA=(*led);
		if((*led)==0x01) {
			(*led)=0x80;
		} else {
			(*led)>>=1;
		}
		break;
		case 2:		//깜빡깜빡
		if(*ledstatus) {
			PORTA=0xFF;
			*ledstatus=0;
		} else {
			PORTA=0x00;
			*ledstatus=1;
		}
		break;
		case 3:		//왼쪽오른쪽반복
		PORTA=(*led);
		if(*ledstatus) {
			if((*led)==0x80) {
				*ledstatus=0;
				(*led)>>=1;
			} else {
				(*led)<<=1;
			}
		} else {
			if((*led)==0x01) {
				*ledstatus=1;
				(*led)<<=1;
			} else {
				(*led)>>=1;
			}
		}
		break;
	}
}

void LedTask (void *data)
{
	INT8U err;
	INT8U led=0x01;	//LED 위치 -> oper0,1,3에 사용
	INT8U status=1;	//LED 상태 -> oper2,3에 사용
	DDRA=0xFF;		//LED 출력모드
	data=data;
	while(1) {
		PORTA=0x00;
		OSFlagPend(t_grp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);
		LedOperation(LedOper,&led,&status);
		OSTimeDlyHMSM(0,0,0,200);
	}
}

void BuzzerTask(void* data) {	
	INT8U err;
	INT16U dly=0;
	DDRB=0x10;
	TCCR2 = 0x03;		//timer2 32분주 -> Buzzer 위함
	TCNT2 = Frequency[IND];
	TIMSK |= 0x40;	 //Timer2 Overflow Interrupt 활성화
	data=data;
	while(1) {
		status=0;
		OSTimeDlyHMSM(0,0,0,100);
		OSFlagPend(t_grp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err);
		status=1;
		IND=(IND+1)%25;
		dly=rabbit_dly[IND];
		OSTimeDlyHMSM(0,0,0,dly*200);
	}
}

//빛양을 읽어서 메일박스로 전송하는 TASK
void ReadLightTask(void* data) {	
	INT8U err;
	data=data;
	INT8U low,high;
	INT16U value;
	ADMUX=0x00;
	ADCSRA=0x87;
	while(1) {
		OSFlagPend(t_grp,0x04,OS_FLAG_WAIT_SET_ALL,0,&err);
		ADCSRA|=0x40;
		while((ADCSRA & 0x10) != 0x10);
		low=ADCL;
		high=ADCH;
		value=(high<<8) | low;
		OSMboxPost(LightMbox,&value);
		OSTimeDlyHMSM(0,0,0,50);
	}
}

//readLightTask로부터 빛의 양을 받아서 배터리 충전하는 Task
void LightTask(void* data) {
	INT8U err,i;
	data=data;
	const INT16U LIGHT=871;
	INT16U val;
	INT16U Elec=0;
	INT16U tmp;
	while(1) {
		val=*(INT16U*)OSMboxPend(LightMbox,0,&err);
		if(val>=LIGHT) {
			if(Elec!=1000) {
				Elec++;
			}
		} else {
			if(Elec!=0) {
				Elec--;
			}
		}
		tmp=Elec/125;
		PORTA=0x00;
		for(i=0;i<tmp;i++) {
			PORTA|=myMapTbl[i];
		}
		i=0;
		tmp=Elec;
		OSMutexPend(mutex,0,&err);
		while(i<4) {			
			FNDData[i++]=DIGIT[tmp%10];
			tmp/=10;
		}
		OSMutexPost(mutex);
	}
}

void InitI2C()
{
    PORTD = 3; 						// For Pull-up override value
    SFIOR &= ~(1 << PUD); 			// PUD
    TWSR = 0; 						// TWPS0 = 0, TWPS1 = 0
    TWBR = 32;						// for 100  K Hz bus clock
	TWCR = _BV(TWEA) | _BV(TWEN);	// TWEA = Ack pulse is generated
									// TWEN = TWI 동작을 가능하게 한다
}

void TemperatureTask (void *data)
{	
	INT8U err;
	data = data;
	INT8U low,high;
	int value;
	INT8U value_int,value_deci;
	while (1)  {
		low=*(INT8U*)OSQPend(MsgQ,0,&err);
		high=*(INT8U*)OSQPend(MsgQ,0,&err);
		value=high;
		value<<=8;
		value|=low;
		OSMutexPend(mutex,0,&err);
		if((value&0x8000) != 0x8000) {
			FNDData[3]=DIGIT[11];
		} else {
			FNDData[3]=DIGIT[10];
			value=(~value)-1;
		}		
		value_int=(INT8U)((value&0x7f00)>>8);
		value_deci=(INT8U)(value&0x00ff);
		FNDData[2]=DIGIT[(value_int/10)%10];
		FNDData[1]=(DIGIT[value_int%10]|0x80);
		FNDData[0]=DIGIT[((value_deci&0x80)==0x80)*5];
		OSMutexPost(mutex);
	}
}

void ReadTemperatureTask (void *data)
{
	INT8U err;
	INT8U low,high;
	data = data;
	InitI2C();

	while (1)  {
		OSFlagPend(t_grp,0x08,OS_FLAG_WAIT_SET_ALL,0,&err);
		OS_ENTER_CRITICAL();

		TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);	// START 전송
		while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

		TWDR = 0x98 + 1; 							//TEMP_I2C_ADDR + 1 > SLA+R 준비, R=1
		TWCR = _BV(TWINT) | _BV(TWEN);				// SLA+R 전송
		while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);	// 1st DATA 준비
		while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

		//온도센서는 16bit 기준으로 값을 가져오므로
		//8비트씩 2번을 받아야 한다.
		high = TWDR;								// 1 byte DATA 수신
		TWCR = _BV(TWINT) | _BV(TWEN); 				// SLA+R 전송
		while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

		low = TWDR;								// 1 byte DATA 수신
		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);	// STOP 전송

		TIMSK = ((((int)high<<8)+low) >= 33) ? TIMSK | _BV(TOIE2): TIMSK & ~_BV(TOIE2);

		OS_EXIT_CRITICAL();
		OSQPost(MsgQ,&low);
		OSQPost(MsgQ,&high);

	// value post

		OSTimeDlyHMSM(0, 0, 0, 200);
	}
}
