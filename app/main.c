#include "includes.h"

#include <avr/io.h>
#include <util/delay.h>

#define  TASK_STK_SIZE  OS_TASK_DEF_STK_SIZE
#define  N_TASKS        8

OS_STK       TaskStk[N_TASKS][TASK_STK_SIZE];
OS_EVENT *Mbox;
OS_EVENT* mutex;
OS_EVENT* MsgQ;
OS_FLAG_GRP* t_grp;		//task의 실행 순서를 정하는 EventFlag
void* MsgQTbl[2];  //Message Queue Table

// 0: LED, 1: 버저, 2: 광, 3: 온도

volatile INT8U	FndNum;
volatile INT8U LedOper;
volatile INT8U TaskNum;
INT8U const myMapTbl[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
unsigned char FNDData[4]={0x3F,0x79,0x38,0};
void FndTask(void *data);
void FndDisplayTask(void *data);
void LedTask (void *data);
void BuzzerTask(void* data);
void TemperatureTask(void *data);
void ReadTemperatureTask(void *data);
void LightTask(void* data);
void InitI2C();
int ReadTemperature(void);
ISR(INT4_vect);
ISR(INT5_vect);

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
	TIMSK = _BV(TOIE0);
	TCNT0 = 256 - (CPU_CLOCK_HZ / OS_TICKS_PER_SEC / 1024);
	DDRE=0xCF;	//SW 입력 모드
	EICRB=0x0A;
	EIMSK=0x30;
	sei();
	//interrupt설정
	OS_EXIT_CRITICAL();

	Mbox=OSMboxCreate((void*)0);
	mutex=OSMutexCreate(1,&err);
	t_grp=OSFlagCreate(myMapTbl[TaskNum],&err);
	MsgQ=OSQCreate(MsgQTbl,2);
	OSTaskCreate(TemperatureTask, (void *)0, (void *)&TaskStk[0][TASK_STK_SIZE - 1], 2);
	OSTaskCreate(ReadTemperatureTask, (void *)0, (void *)&TaskStk[1][TASK_STK_SIZE - 1], 3);
	OSTaskCreate(LightTask, (void *)0, (void *)&TaskStk[2][TASK_STK_SIZE - 1], 4);
	OSTaskCreate(LedTask, (void *)0, (void *)&TaskStk[3][TASK_STK_SIZE - 1], 5);
	OSTaskCreate(BuzzerTask, (void *)0, (void *)&TaskStk[4][TASK_STK_SIZE - 1], 6);
	OSTaskCreate(FndDisplayTask, (void *)0, (void *)&TaskStk[5][TASK_STK_SIZE - 1], 6);
	OSTaskCreate(FndTask, (void *)0, (void *)&TaskStk[6][TASK_STK_SIZE - 1], 7);

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
	INT8U err;
	OSMutexPend(mutex,0,&err);
	OSFlagPend(t_grp,myMapTbl[TaskNum],OS_FLAG_WAIT_SET_ALL+OS_FLAG_CONSUME,0,&err);
	TaskNum=(TaskNum+1)&0x03;
	OSFlagPost(t_grp,myMapTbl[TaskNum],OS_FLAG_SET,&err);
	OSMutexPost(mutex);
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
		case 2:		//Light
		break;
		case 3:		//Temp
		break;
	}
	OSMutexPost(mutex);
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

int ReadTemperature(void)
{
	int value;

	TWCR = _BV(TWSTA) | _BV(TWINT) | _BV(TWEN);	// START 전송
	while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

	TWDR = 0x98 + 1; 							//TEMP_I2C_ADDR + 1 > SLA+R 준비, R=1
	TWCR = _BV(TWINT) | _BV(TWEN);				// SLA+R 전송
	while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);	// 1st DATA 준비
	while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

	//온도센서는 16bit 기준으로 값을 가져오므로
	//8비트씩 2번을 받아야 한다.
	value = TWDR << 8;							// 1 byte DATA 수신
	TWCR = _BV(TWINT) | _BV(TWEN); 				// SLA+R 전송
	while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

	value |= TWDR;								// 1 byte DATA 수신
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);	// STOP 전송

	value >>= 8;

	TIMSK = (value >= 33) ? TIMSK | _BV(TOIE2): TIMSK & ~_BV(TOIE2);

	return value;
}

void FndTask (void *data)
{
	INT8U err;
	data = data;
	INT8U sel=0;
	struct Mail fnddata;
	while (1) {
		fnddata.data=FNDData[sel];
		fnddata.sel=sel;
		sel=(sel+1)&0x03;
		OSMboxPost(Mbox,&fnddata);
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
			} else {
				(*led)<<=1;
			}
		} else {
			if((*led)==0x01) {
				*ledstatus=1;
			} else {
				(*led)>>=1;
			}
		}
		break;
	}
}

void LedTask (void *data)
{
	const unsigned char LED[4]={0,0x38,0x79,0x3F};	//LED 출력위함
	INT8U err;
	INT8U led=0x01;
	INT8U status=1;
	DDRA=0xFF;
	data=data;
	while(1) {
		OSFlagPend(t_grp,0x01,OS_FLAG_WAIT_SET_ALL,0,&err);
		LedOperation(LedOper,&led,&status);
		OSTimeDlyHMSM(0,0,0,200);
	}
}

void BuzzerTask(void* data) {	
	const unsigned char BUZZ[4]={0x7F,0x3E,0x5B,0x5B};	//BUZZ 출력위함
	INT8U sel=0;
	INT8U err;
	//INT8U delay=17;
	INT8U on=1;
	data=data;

	while(1) {
		OSFlagPend(t_grp,0x02,OS_FLAG_WAIT_SET_ALL,0,&err);
		if(on) {		//일단 타이머 없이 아무소리만 나게 해봄 과연....
			PORTB=0x00;
			on=0;
		} else {
			PORTB=0x10;
			on=1;
		}
		OSTimeDlyHMSM(0,0,0,1);
	}
}

void LightTask(void* data) {
	INT8U err;
	data=data;
	while(1) {
		OSFlagPend(t_grp,0x04,OS_FLAG_WAIT_SET_ALL,0,&err);

		OSTimeDlyHMSM(0,0,0,200);
	}
}

void ReadTemperatureTask (void *data)
{
	const char digit[12]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7c,0x07,0x7f,0x67,0x40,0x00};
	INT8U err;
	data = data;
	INT8U low,high;
	int value;
	INT8S value_int,value_deci;
	while (1)  {
		low=*(INT8U*)OSQPend(MsgQ,0,&err);
		high=*(INT8U*)OSQPend(MsgQ,0,&err);
		value=(high<<8+low);
		OSMutexPend(mutex,0,&err);
		if((value&0x8000) != 0x8000) {
			FNDData[3]=digit[11];
		} else {
			FNDData[3]=digit[10];
			value=(~value)-1;
		}
		value_int=((value&0x7f00)>>8);
		value_deci=(value&0x00ff);
		FNDData[2]=digit[(value_int/10)%10];
		FNDData[1]=(digit[value_int%10]|0x80);
		FNDData[0]=digit[((value_deci&0x80)==0x80)*5];
		OSMutexPost(mutex);
	}
}

void TemperatureTask (void *data)
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
		high |= TWDR;								// 1 byte DATA 수신
		TWCR = _BV(TWINT) | _BV(TWEN); 				// SLA+R 전송
		while(!(TWCR & _BV(TWINT)));				// ACK를 기다림

		low |= TWDR;								// 1 byte DATA 수신
		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);	// STOP 전송

		TIMSK = (((high<<8)+low) >= 33) ? TIMSK | _BV(TOIE2): TIMSK & ~_BV(TOIE2);

		OS_EXIT_CRITICAL();
		OSQPost(MsgQ,&low);
		OSQPost(MsgQ,&high);

	// value post

		OSTimeDlyHMSM(0, 0, 0, 100);
	}
}
